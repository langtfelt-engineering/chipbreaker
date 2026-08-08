// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Topological validation of a welded triangle mesh.
//!
//! # What is being decided
//!
//! Field building casts rays through the mesh and infers material from crossing parity. That
//! inference is only valid for a **closed, consistently oriented, outward-facing**
//! surface. This module decides whether the mesh in hand is one, and when it is
//! not, says precisely which elements are at fault.
//!
//! Nothing is repaired. A validator that quietly stitches boundary edges shut
//! produces a mesh that passes its own check and simulates a part the customer
//! did not model.
//!
//! # Exactness
//!
//! Degeneracy is decided with **exact predicates**, never a float area
//! threshold. `area < 1e-12` is not a property of the triangle, it is a property
//! of the units it happens to be expressed in — the same sliver passes in metres
//! and fails in microns. Three points are collinear iff all three of their
//! coordinate-plane projections are collinear, and
//! [`crate::predicates::orient2d`] decides each of those exactly.
//!
//! # Finding identity
//!
//! Finding IDs are derived from the finding's own content, not from a counter.
//! Two runs over the same mesh therefore produce byte-identical reports, and two
//! runs over *similar* meshes produce reports that diff cleanly — a finding that
//! did not change keeps its ID even if a dozen others appeared before it. A
//! findings report
//! will need exactly this for gouge findings; the pattern is established here.

use std::collections::{BTreeMap, BTreeSet};

use core::fmt;

use crate::golden::{CanonicalHash, Digest, Hashable};
use crate::math::Vec2;
use crate::mesh::TriMesh;
use crate::predicates::{Orientation, orient2d};

/// Report format version. Bump when the shape changes, so consumers can tell.
pub const REPORT_VERSION: u32 = 1;

/// What kind of problem a finding describes.
///
/// Ordered so that reports sort most-structural first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FindingKind {
    /// An edge with three or more incident triangles. The surface is not a
    /// manifold there and "inside" is not well defined.
    NonManifoldEdge,
    /// An edge with exactly one incident triangle: a hole in the surface.
    BoundaryEdge,
    /// Two triangles sharing an edge traverse it in the same direction, so one
    /// of them is wound backwards relative to the other.
    InconsistentOrientation,
    /// A triangle with two identical indices, or three collinear vertices.
    /// Zero area, no normal, no contribution to any ray test.
    DegenerateTriangle,
    /// Two triangles with the same three vertices in any winding.
    DuplicateTriangle,
    /// Two non-adjacent triangles that intersect.
    SelfIntersection,
    /// The surface is closed but wound inside out: the enclosed signed volume is
    /// negative.
    InvertedOrientation,
    /// A vertex referenced by no triangle. Harmless, but usually a sign the file
    /// was assembled oddly.
    UnusedVertex,
    /// A non-convex OBJ face was fan-triangulated from its first vertex, which
    /// is only correct when the face is star-shaped from that vertex.
    ///
    /// Reported rather than rejected because the fan may still be right — an
    /// L-shape is non-convex and fans correctly from every one of its vertices —
    /// but the user needs to know the assumption was made. Ear clipping is the
    /// fix if this turns out to matter on real data.
    NonConvexPolygonFan,
}

impl FindingKind {
    /// Stable machine-readable name, used in JSON and in the finding ID.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::NonManifoldEdge => "non-manifold-edge",
            Self::BoundaryEdge => "boundary-edge",
            Self::InconsistentOrientation => "inconsistent-orientation",
            Self::DegenerateTriangle => "degenerate-triangle",
            Self::DuplicateTriangle => "duplicate-triangle",
            Self::SelfIntersection => "self-intersection",
            Self::InvertedOrientation => "inverted-orientation",
            Self::UnusedVertex => "unused-vertex",
            Self::NonConvexPolygonFan => "non-convex-polygon-fan",
        }
    }

    /// True if this finding makes the mesh unusable for the parity argument.
    ///
    /// The distinction is not cosmetic. A boundary edge or a non-manifold edge
    /// breaks the closed-surface theorem outright, so ray casting cannot infer
    /// material. A degenerate triangle does not break the parity argument: it
    /// bounds no volume, so removing it changes nothing about which points are
    /// inside.
    ///
    /// **Amended.** This used to say a degenerate triangle "contributes
    /// nothing to any ray test", and that was wrong. Left in the mesh it is
    /// very much visible to a ray test: all three of its edge functions vanish
    /// for any ray coplanar with the segment it collapsed to, which is the
    /// caster's `coplanar_rejected` path — and field building treats a coplanar rejection as
    /// a hard error, because for a *real* triangle it means a hole of unknown
    /// size. One zero-area triangle in `broken-zero-area.stl` produced 102
    /// rejections across 10,404 rays, every one of them on the diagonal where
    /// the ray happens to be coplanar with that segment.
    ///
    /// So the sentence is now true only because field building acts on it:
    /// [`crate::dexel::DexelField::build`] drops exactly-degenerate triangles
    /// before casting and reports how many. Anyone casting rays at a mesh
    /// without doing that should expect the rejections.
    #[must_use]
    pub const fn is_fatal_for_raycasting(self) -> bool {
        matches!(
            self,
            Self::NonManifoldEdge | Self::BoundaryEdge | Self::InconsistentOrientation
        )
    }
}

impl fmt::Display for FindingKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// One problem found in the mesh.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    /// Content-derived identity: the first 16 hex characters of the canonical
    /// digest of `kind`, `triangles` and `vertices`.
    ///
    /// Deliberately not a counter. A counter renumbers every finding after an
    /// inserted one, so a report diff shows a hundred changes where one thing
    /// changed.
    pub id: String,
    /// What kind of problem.
    pub kind: FindingKind,
    /// Triangles involved, ascending.
    pub triangles: Vec<u32>,
    /// Vertices involved, ascending.
    pub vertices: Vec<u32>,
    /// Human-readable explanation.
    pub detail: String,
}

impl Finding {
    fn new(
        kind: FindingKind,
        mut triangles: Vec<u32>,
        mut vertices: Vec<u32>,
        detail: String,
    ) -> Self {
        triangles.sort_unstable();
        vertices.sort_unstable();
        let mut h = CanonicalHash::new();
        h.begin("Finding");
        h.str(kind.name());
        h.u64_slice(&triangles.iter().map(|v| u64::from(*v)).collect::<Vec<_>>());
        h.u64_slice(&vertices.iter().map(|v| u64::from(*v)).collect::<Vec<_>>());
        h.end();
        let id = h.finish().to_hex()[..16].to_owned();
        Self {
            id,
            kind,
            triangles,
            vertices,
            detail,
        }
    }

    /// Sort key: kind first, then the elements involved. Independent of
    /// discovery order.
    fn sort_key(&self) -> (FindingKind, Vec<u32>, Vec<u32>) {
        (self.kind, self.triangles.clone(), self.vertices.clone())
    }
}

impl Hashable for Finding {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Finding");
        h.str(&self.id);
        h.str(self.kind.name());
        h.u64_slice(
            &self
                .triangles
                .iter()
                .map(|v| u64::from(*v))
                .collect::<Vec<_>>(),
        );
        h.u64_slice(
            &self
                .vertices
                .iter()
                .map(|v| u64::from(*v))
                .collect::<Vec<_>>(),
        );
        h.end();
    }
}

/// Per-connected-component topology.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentInfo {
    /// Triangles in this component.
    pub triangles: u32,
    /// Distinct vertices referenced by those triangles.
    pub vertices: u32,
    /// Distinct undirected edges among those triangles.
    pub edges: u32,
    /// Boundary edges (exactly one incident triangle) within this component.
    pub boundary_edges: u32,
    /// `V - E + F`.
    pub euler_characteristic: i64,
    /// Genus, from `V - E + F = 2 - 2g`. `None` when the component is not closed
    /// or the characteristic is odd, in which case the formula does not apply.
    pub genus: Option<i64>,
    /// Signed volume enclosed by this component alone.
    pub signed_volume: f64,
}

impl Hashable for ComponentInfo {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("ComponentInfo");
        h.u64(u64::from(self.triangles));
        h.u64(u64::from(self.vertices));
        h.u64(u64::from(self.edges));
        h.u64(u64::from(self.boundary_edges));
        h.i64(self.euler_characteristic);
        match self.genus {
            Some(g) => {
                h.bool(true).i64(g);
            }
            None => {
                h.bool(false);
            }
        }
        h.f64(self.signed_volume);
        h.end();
    }
}

/// The full validation result.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshReport {
    /// Format version; see [`REPORT_VERSION`].
    pub version: u32,
    /// Vertex count.
    pub vertices: u32,
    /// Triangle count.
    pub triangles: u32,
    /// Distinct undirected edge count.
    pub edges: u32,
    /// Every edge has exactly two incident triangles.
    pub is_manifold: bool,
    /// No boundary edges.
    pub is_watertight: bool,
    /// Every shared edge is traversed in opposite directions by its two
    /// triangles.
    pub is_orientation_consistent: bool,
    /// Signed volume of the whole mesh, by the divergence theorem.
    pub signed_volume: f64,
    /// Total surface area.
    pub surface_area: f64,
    /// One entry per connected component, ordered by smallest triangle index.
    pub components: Vec<ComponentInfo>,
    /// Findings, sorted by kind then by the elements involved.
    pub findings: Vec<Finding>,
    /// Whether the self-intersection check was run. It is opt-in because it
    /// costs `O(n log n)` with a large constant.
    pub self_intersection_checked: bool,
}

impl MeshReport {
    /// True if the mesh is closed, manifold, consistently oriented and
    /// outward-facing — that is, safe for the parity argument.
    #[must_use]
    pub fn is_solid(&self) -> bool {
        self.is_manifold
            && self.is_watertight
            && self.is_orientation_consistent
            && self.signed_volume > 0.0
    }

    /// Findings that break ray casting outright.
    pub fn fatal_findings(&self) -> impl Iterator<Item = &Finding> {
        self.findings
            .iter()
            .filter(|f| f.kind.is_fatal_for_raycasting())
    }

    /// How many findings of a given kind.
    #[must_use]
    pub fn count_of(&self, kind: FindingKind) -> usize {
        self.findings.iter().filter(|f| f.kind == kind).count()
    }

    /// Canonical digest of the whole report.
    #[must_use]
    pub fn digest(&self) -> Digest {
        self.canonical_digest()
    }
}

impl Hashable for MeshReport {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("MeshReport");
        h.u64(u64::from(self.version));
        h.u64(u64::from(self.vertices));
        h.u64(u64::from(self.triangles));
        h.u64(u64::from(self.edges));
        h.bool(self.is_manifold);
        h.bool(self.is_watertight);
        h.bool(self.is_orientation_consistent);
        h.f64(self.signed_volume);
        h.f64(self.surface_area);
        h.add_all(self.components.iter());
        h.add_all(self.findings.iter());
        h.bool(self.self_intersection_checked);
        h.end();
    }
}

/// An undirected edge, as an ordered vertex-index pair.
type EdgeKey = (u32, u32);

#[inline]
fn edge_key(a: u32, b: u32) -> EdgeKey {
    if a <= b { (a, b) } else { (b, a) }
}

/// One triangle's use of an edge: which triangle, and whether it traverses the
/// edge in the canonical (low → high) direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct EdgeUse {
    triangle: u32,
    forward: bool,
}

/// Exact collinearity test for three points in space.
///
/// Three points are collinear iff their projections onto all three coordinate
/// planes are collinear. Each projection is decided exactly by
/// [`orient2d`], so the whole test is exact — no area threshold, and therefore
/// no dependence on what unit the model happens to be in.
///
/// Public because field building drops degenerate triangles before casting, and it must
/// use *this* test rather than a second one of its own: two definitions of
/// "degenerate" that disagree by one triangle would put the validator and the
/// field builder into an argument nobody could settle.
pub fn collinear_exact(a: crate::math::Vec3, b: crate::math::Vec3, c: crate::math::Vec3) -> bool {
    let xy = orient2d(
        Vec2::new(a.x, a.y),
        Vec2::new(b.x, b.y),
        Vec2::new(c.x, c.y),
    );
    let yz = orient2d(
        Vec2::new(a.y, a.z),
        Vec2::new(b.y, b.z),
        Vec2::new(c.y, c.z),
    );
    let zx = orient2d(
        Vec2::new(a.z, a.x),
        Vec2::new(b.z, b.x),
        Vec2::new(c.z, c.x),
    );
    xy == Orientation::Zero && yz == Orientation::Zero && zx == Orientation::Zero
}

/// Deterministic union-find over triangle indices.
///
/// Determinism comes from the caller: the union order is a fixed traversal of a
/// sorted edge map, so the resulting forest — and therefore the component
/// numbering — is a pure function of the mesh.
struct UnionFind {
    parent: Vec<u32>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n as u32).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            // Path halving; does not affect which set an element lands in.
            let grandparent = self.parent[self.parent[x as usize] as usize];
            self.parent[x as usize] = grandparent;
            x = grandparent;
        }
        x
    }

    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        let (lo, hi) = if self.rank[ra as usize] < self.rank[rb as usize] {
            (ra, rb)
        } else {
            (rb, ra)
        };
        self.parent[lo as usize] = hi;
        if self.rank[ra as usize] == self.rank[rb as usize] {
            self.rank[hi as usize] += 1;
        }
    }
}

/// Validates a mesh's topology.
///
/// The mesh should already be welded; validating a triangle soup reports every
/// edge as a boundary edge, which is true but not useful.
///
/// Self-intersection is **not** checked here — see
/// [`crate::mesh::bvh::self_intersections`], which needs a BVH and is opt-in
/// because it costs `O(n log n)` with a large constant.
#[must_use]
pub fn validate(mesh: &TriMesh) -> MeshReport {
    let mut findings: Vec<Finding> = Vec::new();

    // --- Degenerate and duplicate triangles ---------------------------------
    let mut duplicate_groups: BTreeMap<[u32; 3], Vec<u32>> = BTreeMap::new();
    for i in 0..mesh.triangle_count() {
        let t = mesh.triangles()[i as usize];
        let repeated = t[0] == t[1] || t[1] == t[2] || t[2] == t[0];
        if repeated {
            findings.push(Finding::new(
                FindingKind::DegenerateTriangle,
                vec![i],
                t.to_vec(),
                format!("triangle {i} repeats a vertex index: {t:?}"),
            ));
        } else {
            let [a, b, c] = mesh.triangle(i);
            if collinear_exact(a, b, c) {
                findings.push(Finding::new(
                    FindingKind::DegenerateTriangle,
                    vec![i],
                    t.to_vec(),
                    format!("triangle {i} has three exactly collinear vertices"),
                ));
            }
        }
        let mut sorted = t;
        sorted.sort_unstable();
        duplicate_groups.entry(sorted).or_default().push(i);
    }
    for (key, group) in &duplicate_groups {
        if group.len() > 1 {
            findings.push(Finding::new(
                FindingKind::DuplicateTriangle,
                group.clone(),
                key.to_vec(),
                format!(
                    "{} triangles share the vertex set {key:?} (in some winding)",
                    group.len()
                ),
            ));
        }
    }

    // --- Edge map -----------------------------------------------------------
    // A BTreeMap so the traversal order below — which drives union-find and
    // therefore component numbering — is a property of the mesh, not of a hash.
    let mut edges: BTreeMap<EdgeKey, Vec<EdgeUse>> = BTreeMap::new();
    for i in 0..mesh.triangle_count() {
        let t = mesh.triangles()[i as usize];
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a == b {
                // Degenerate triangles contribute no meaningful edges; they are
                // already reported above.
                continue;
            }
            edges.entry(edge_key(a, b)).or_default().push(EdgeUse {
                triangle: i,
                forward: a < b,
            });
        }
    }

    let mut is_manifold = true;
    let mut is_watertight = true;
    let mut is_orientation_consistent = true;
    let mut boundary_of: BTreeMap<u32, u32> = BTreeMap::new();

    for (&(a, b), uses) in &edges {
        match uses.len() {
            1 => {
                is_watertight = false;
                is_manifold = false;
                *boundary_of.entry(uses[0].triangle).or_insert(0) += 1;
                findings.push(Finding::new(
                    FindingKind::BoundaryEdge,
                    vec![uses[0].triangle],
                    vec![a, b],
                    format!(
                        "edge ({a}, {b}) has one incident triangle: the surface has a hole here"
                    ),
                ));
            }
            2 => {
                if uses[0].forward == uses[1].forward {
                    is_orientation_consistent = false;
                    findings.push(Finding::new(
                        FindingKind::InconsistentOrientation,
                        vec![uses[0].triangle, uses[1].triangle],
                        vec![a, b],
                        format!(
                            "triangles {} and {} both traverse edge ({a}, {b}) in the \
                             same direction, so one of them is wound backwards",
                            uses[0].triangle, uses[1].triangle
                        ),
                    ));
                }
            }
            n => {
                is_manifold = false;
                findings.push(Finding::new(
                    FindingKind::NonManifoldEdge,
                    uses.iter().map(|u| u.triangle).collect(),
                    vec![a, b],
                    format!(
                        "edge ({a}, {b}) has {n} incident triangles; the surface is not \
                         a manifold here and 'inside' is undefined"
                    ),
                ));
            }
        }
    }

    // --- Connected components ----------------------------------------------
    let triangle_count = mesh.triangle_count() as usize;
    let mut uf = UnionFind::new(triangle_count);
    for uses in edges.values() {
        for w in uses.windows(2) {
            uf.union(w[0].triangle, w[1].triangle);
        }
    }

    // Group by root, ordered by the smallest triangle index in each group so the
    // component numbering is stable and meaningful.
    let mut by_root: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for i in 0..triangle_count as u32 {
        let root = uf.find(i);
        by_root.entry(root).or_default().push(i);
    }
    let mut groups: Vec<Vec<u32>> = by_root.into_values().collect();
    groups.sort_by_key(|g| g.first().copied().unwrap_or(u32::MAX));

    let mut components = Vec::with_capacity(groups.len());
    for group in &groups {
        let mut component_vertices: BTreeSet<u32> = BTreeSet::new();
        let mut component_edges: BTreeMap<EdgeKey, u32> = BTreeMap::new();
        let mut volume = 0.0f64;
        for &i in group {
            let t = mesh.triangles()[i as usize];
            component_vertices.extend(t.iter().copied());
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if a != b {
                    *component_edges.entry(edge_key(a, b)).or_insert(0) += 1;
                }
            }
            let [p, q, r] = mesh.triangle(i);
            volume += p.dot(q.cross(r));
        }
        let boundary_edges = component_edges.values().filter(|&&n| n == 1).count() as u32;
        let v = component_vertices.len() as i64;
        let e = component_edges.len() as i64;
        let f = group.len() as i64;
        let euler = v - e + f;
        let closed = boundary_edges == 0;
        // V - E + F = 2 - 2g holds for a closed orientable surface. Reporting a
        // genus for an open or non-orientable one would be arithmetic dressed up
        // as topology.
        let genus = if closed && (2 - euler) % 2 == 0 {
            Some((2 - euler) / 2)
        } else {
            None
        };
        components.push(ComponentInfo {
            triangles: f as u32,
            vertices: v as u32,
            edges: e as u32,
            boundary_edges,
            euler_characteristic: euler,
            genus,
            signed_volume: volume / 6.0,
        });
    }

    // --- Whole-mesh orientation --------------------------------------------
    let signed_volume = mesh.signed_volume();
    if is_watertight && is_manifold && is_orientation_consistent && signed_volume < 0.0 {
        findings.push(Finding::new(
            FindingKind::InvertedOrientation,
            Vec::new(),
            Vec::new(),
            format!(
                "the surface is closed and consistently wound, but encloses a \
                 negative volume ({signed_volume}); the whole mesh is inside out"
            ),
        ));
    }

    // --- Unused vertices ----------------------------------------------------
    let mut used = vec![false; mesh.vertices().len()];
    for t in mesh.triangles() {
        for &i in t {
            used[i as usize] = true;
        }
    }
    let unused: Vec<u32> = used
        .iter()
        .enumerate()
        .filter(|(_, u)| !**u)
        .map(|(i, _)| i as u32)
        .collect();
    if !unused.is_empty() {
        findings.push(Finding::new(
            FindingKind::UnusedVertex,
            Vec::new(),
            unused.clone(),
            format!("{} vertices are referenced by no triangle", unused.len()),
        ));
    }

    // --- Load-time assumptions worth surfacing -----------------------------
    // The loader recorded that it fan-triangulated a non-convex face. That is an
    // assumption about the geometry, not a property of the topology, so it
    // cannot be rediscovered from the mesh — it has to be carried through the
    // metadata and reported here, where people actually look.
    if mesh.meta().non_convex_polygons > 0 {
        findings.push(Finding::new(
            FindingKind::NonConvexPolygonFan,
            Vec::new(),
            Vec::new(),
            format!(
                "{} non-convex face(s) were fan-triangulated from their first \
                 vertex; that is only correct if each is star-shaped from that \
                 vertex, and the resulting triangles may lie outside the face",
                mesh.meta().non_convex_polygons
            ),
        ));
    }

    findings.sort_by_key(Finding::sort_key);

    MeshReport {
        version: REPORT_VERSION,
        vertices: mesh.vertex_count(),
        triangles: mesh.triangle_count(),
        edges: edges.len() as u32,
        is_manifold,
        is_watertight,
        is_orientation_consistent,
        signed_volume,
        surface_area: mesh.surface_area(),
        components,
        findings,
        self_intersection_checked: false,
    }
}

/// Adds self-intersection findings to an existing report.
///
/// Separate from [`validate`], and opt-in behind `--check-self-intersect`,
/// because it costs `O(n log n)` with a large constant — a BVH query per
/// triangle and an exact test per candidate pair. On a million-triangle model
/// that is minutes rather than milliseconds.
///
/// Building the hierarchy here rather than taking one as a parameter keeps the
/// call site simple; a caller that already has a BVH can use
/// [`crate::mesh::intersect::self_intersections`] directly.
pub fn check_self_intersections(mesh: &TriMesh, report: &mut MeshReport) {
    let bvh = crate::mesh::bvh::Bvh::build(mesh);
    for (a, b) in crate::mesh::intersect::self_intersections(mesh, &bvh) {
        let mut vertices: Vec<u32> = mesh.triangles()[a as usize].to_vec();
        vertices.extend_from_slice(&mesh.triangles()[b as usize]);
        report.findings.push(Finding::new(
            FindingKind::SelfIntersection,
            vec![a, b],
            vertices,
            format!("triangles {a} and {b} intersect but share no vertex"),
        ));
    }
    report.self_intersection_checked = true;
    report.findings.sort_by_key(Finding::sort_key);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;
    use crate::mesh::MeshMeta;
    use crate::mesh::tests::unit_cube;

    fn mesh_of(v: Vec<Vec3>, t: Vec<[u32; 3]>) -> TriMesh {
        TriMesh::new(v, t, MeshMeta::synthetic()).expect("valid")
    }

    #[test]
    fn a_cube_is_a_solid() {
        let r = validate(&unit_cube());
        assert!(r.is_manifold);
        assert!(r.is_watertight);
        assert!(r.is_orientation_consistent);
        assert!(r.is_solid());
        assert_eq!(r.signed_volume, 1.0);
        assert_eq!(r.surface_area, 6.0);
        assert_eq!(r.edges, 18, "a triangulated cube has 18 edges");
        assert_eq!(r.components.len(), 1);
        assert_eq!(
            r.components[0].euler_characteristic, 2,
            "a sphere has chi = 2"
        );
        assert_eq!(r.components[0].genus, Some(0));
        assert!(
            r.findings.is_empty(),
            "unexpected findings: {:?}",
            r.findings
        );
    }

    #[test]
    fn euler_characteristic_is_right_for_the_cube() {
        // V - E + F = 8 - 18 + 12 = 2.
        let r = validate(&unit_cube());
        let c = &r.components[0];
        assert_eq!((c.vertices, c.edges, c.triangles), (8, 18, 12));
        assert_eq!(c.vertices as i64 - c.edges as i64 + c.triangles as i64, 2);
    }

    #[test]
    fn a_missing_face_is_reported_as_boundary_edges() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        t.remove(0); // leaves a triangular hole
        let holed = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&holed);
        assert!(!r.is_watertight);
        assert!(!r.is_manifold, "a boundary edge is also non-manifold");
        assert_eq!(r.count_of(FindingKind::BoundaryEdge), 3);
        assert!(r.fatal_findings().count() >= 3);
        assert!(!r.is_solid());
    }

    #[test]
    fn a_third_incident_triangle_is_non_manifold() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        // Re-use the edge (0, 2) with a spurious extra triangle.
        t.push([0, 2, 4]);
        let m = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&m);
        assert!(!r.is_manifold);
        assert!(r.count_of(FindingKind::NonManifoldEdge) >= 1);
        let f = r
            .findings
            .iter()
            .find(|f| f.kind == FindingKind::NonManifoldEdge)
            .expect("present");
        assert!(f.triangles.len() >= 3, "must name every incident triangle");
    }

    #[test]
    fn a_single_flipped_face_is_reported() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        t[0] = [t[0][0], t[0][2], t[0][1]];
        let m = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&m);
        assert!(!r.is_orientation_consistent);
        assert!(r.count_of(FindingKind::InconsistentOrientation) > 0);
        assert!(!r.is_solid());
    }

    #[test]
    fn a_wholly_inverted_mesh_is_consistent_but_inside_out() {
        let cube = unit_cube();
        let t: Vec<[u32; 3]> = cube
            .triangles()
            .iter()
            .map(|t| [t[0], t[2], t[1]])
            .collect();
        let m = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&m);
        // Every edge is still traversed oppositely by its two triangles, so the
        // mesh is locally consistent — it is globally inverted, which is a
        // different finding.
        assert!(r.is_orientation_consistent);
        assert!(r.is_watertight);
        assert_eq!(r.signed_volume, -1.0);
        assert_eq!(r.count_of(FindingKind::InvertedOrientation), 1);
        assert!(!r.is_solid());
    }

    #[test]
    fn degenerate_triangles_are_found_exactly_not_by_area_threshold() {
        // A repeated index.
        let m = mesh_of(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 1]]);
        assert_eq!(validate(&m).count_of(FindingKind::DegenerateTriangle), 1);

        // Exactly collinear.
        let m = mesh_of(
            vec![Vec3::ZERO, Vec3::X, Vec3::new(2.0, 0.0, 0.0)],
            vec![[0, 1, 2]],
        );
        assert_eq!(validate(&m).count_of(FindingKind::DegenerateTriangle), 1);

        // A genuine sliver: extremely thin, but NOT collinear, so not degenerate.
        // A float area threshold would wrongly flag this, and would flag it
        // differently depending on whether the model is in mm or in metres.
        let m = mesh_of(
            vec![
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.5, 1e-9, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        assert_eq!(
            validate(&m).count_of(FindingKind::DegenerateTriangle),
            0,
            "a thin triangle is not a degenerate one"
        );
    }

    #[test]
    fn collinearity_is_scale_invariant() {
        // The property an area threshold cannot have: the same triangle in
        // different units gets the same answer.
        for scale in [1e-6, 1.0, 1e6] {
            let a = Vec3::ZERO;
            let b = Vec3::new(scale, 0.0, 0.0);
            let c = Vec3::new(2.0 * scale, 0.0, 0.0);
            assert!(collinear_exact(a, b, c), "collinear at scale {scale}");
            let d = Vec3::new(scale, scale, 0.0);
            assert!(!collinear_exact(a, b, d), "not collinear at scale {scale}");
        }
    }

    #[test]
    fn duplicate_triangles_are_found_in_any_winding() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        let first = t[0];
        t.push([first[0], first[2], first[1]]); // reversed winding
        let m = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&m);
        assert_eq!(r.count_of(FindingKind::DuplicateTriangle), 1);
        let f = r
            .findings
            .iter()
            .find(|f| f.kind == FindingKind::DuplicateTriangle)
            .expect("present");
        assert_eq!(f.triangles.len(), 2);
    }

    #[test]
    fn two_disjoint_components_are_counted() {
        let cube = unit_cube();
        let mut v = cube.vertices().to_vec();
        let mut t = cube.triangles().to_vec();
        let offset = v.len() as u32;
        v.extend(cube.vertices().iter().map(|p| *p + Vec3::splat(10.0)));
        t.extend(
            cube.triangles()
                .iter()
                .map(|x| [x[0] + offset, x[1] + offset, x[2] + offset]),
        );
        let m = mesh_of(v, t);
        let r = validate(&m);
        assert_eq!(r.components.len(), 2);
        assert!(r.is_manifold && r.is_watertight);
        assert_eq!(r.signed_volume, 2.0, "two unit cubes");
        for c in &r.components {
            assert_eq!(c.genus, Some(0));
            assert_eq!(c.signed_volume, 1.0);
        }
    }

    #[test]
    fn a_nested_component_reports_its_own_positive_volume() {
        // A small cube inside a large one, both outward-oriented. Field building cares:
        // the signed volumes add rather than subtract, so a nested shell has to
        // be recognised as a separate component rather than as a cavity.
        let outer = unit_cube();
        let mut v: Vec<Vec3> = outer.vertices().iter().map(|p| *p * 10.0).collect();
        let mut t = outer.triangles().to_vec();
        let offset = v.len() as u32;
        v.extend(outer.vertices().iter().map(|p| *p * 2.0 + Vec3::splat(4.0)));
        t.extend(
            outer
                .triangles()
                .iter()
                .map(|x| [x[0] + offset, x[1] + offset, x[2] + offset]),
        );
        let m = mesh_of(v, t);
        let r = validate(&m);
        assert_eq!(r.components.len(), 2);
        assert_eq!(r.components[0].signed_volume, 1000.0);
        assert_eq!(r.components[1].signed_volume, 8.0);
        assert_eq!(r.signed_volume, 1008.0);
    }

    #[test]
    fn unused_vertices_are_reported() {
        let cube = unit_cube();
        let mut v = cube.vertices().to_vec();
        v.push(Vec3::splat(99.0));
        let m = mesh_of(v, cube.triangles().to_vec());
        let r = validate(&m);
        assert_eq!(r.count_of(FindingKind::UnusedVertex), 1);
        assert!(!r.findings[0].vertices.is_empty());
    }

    #[test]
    fn finding_ids_are_content_derived_and_stable() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        t.remove(0);
        let m = mesh_of(cube.vertices().to_vec(), t);

        let a = validate(&m);
        let b = validate(&m);
        assert_eq!(a, b, "validation must be deterministic");
        assert_eq!(a.digest(), b.digest());

        // Same defect, same ID, regardless of what else is in the report.
        //
        // The added defect must be genuinely unrelated: a degenerate triangle
        // built from *existing* vertices would add uses to existing edges and so
        // change the boundary-edge findings, which is a real change rather than
        // an identity failure. It gets its own vertices.
        let ids_a: Vec<&str> = a.findings.iter().map(|f| f.id.as_str()).collect();
        let mut v2 = m.vertices().to_vec();
        let base = v2.len() as u32;
        v2.extend_from_slice(&[Vec3::splat(50.0), Vec3::splat(51.0), Vec3::splat(52.0)]);
        let mut t2 = m.triangles().to_vec();
        t2.push([base, base + 1, base + 1]);
        let m2 = mesh_of(v2, t2);
        let c = validate(&m2);
        assert!(
            c.count_of(FindingKind::DegenerateTriangle)
                > a.count_of(FindingKind::DegenerateTriangle),
            "the unrelated defect must actually have been added"
        );
        for id in &ids_a {
            assert!(
                c.findings.iter().any(|f| f.id == *id),
                "finding {id} lost its identity when an unrelated finding appeared"
            );
        }
        // IDs are 16 hex characters.
        for f in &c.findings {
            assert_eq!(f.id.len(), 16);
            assert!(f.id.chars().all(|ch| ch.is_ascii_hexdigit()));
        }
    }

    #[test]
    fn findings_are_sorted_independently_of_discovery_order() {
        let cube = unit_cube();
        let mut t = cube.triangles().to_vec();
        t.remove(0);
        t.push([0, 1, 1]);
        let m = mesh_of(cube.vertices().to_vec(), t);
        let r = validate(&m);
        let keys: Vec<_> = r.findings.iter().map(Finding::sort_key).collect();
        let mut sorted = keys.clone();
        sorted.sort();
        assert_eq!(keys, sorted, "findings must come out sorted");
    }

    #[test]
    fn an_empty_mesh_validates_without_panicking() {
        let m = mesh_of(Vec::new(), Vec::new());
        let r = validate(&m);
        assert_eq!(r.triangles, 0);
        assert_eq!(r.edges, 0);
        assert!(r.components.is_empty());
        // Vacuously manifold and watertight; not solid, because it encloses
        // nothing.
        assert!(r.is_manifold && r.is_watertight);
        assert!(!r.is_solid());
    }

    #[test]
    fn a_triangle_soup_reports_every_edge_as_boundary() {
        // Documents why welding must happen first.
        let m = mesh_of(
            vec![
                Vec3::ZERO,
                Vec3::X,
                Vec3::Y,
                Vec3::X,
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::Y,
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        );
        let r = validate(&m);
        assert_eq!(r.count_of(FindingKind::BoundaryEdge), 6);
        assert_eq!(r.components.len(), 2, "unwelded triangles are disconnected");
    }
}
