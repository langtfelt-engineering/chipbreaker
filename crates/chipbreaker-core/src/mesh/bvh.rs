// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Bounding volume hierarchy and **leak-free** ray casting.
//!
//! This is the most important module in Unit 2, and the reason is worth stating
//! plainly before any code.
//!
//! # The parity contract
//!
//! U5 builds the dexel field by casting millions of parallel rays through a
//! closed mesh and recording where each ray is inside material. It infers that
//! from a parity argument: a ray crossing a closed surface produces an **even**
//! number of crossings, strictly alternating enter, exit, enter, exit.
//!
//! If a ray passes exactly through an edge shared by two triangles and the
//! intersection test reports *two* hits or *none* instead of one, the parity
//! flips. Every interval after that point on the ray inverts: solid becomes void
//! and void becomes solid. The symptom is a spike or a tunnel through the
//! simulated stock, appearing intermittently, on customer data, in a way that is
//! very hard to reproduce.
//!
//! Naive Möller–Trumbore does exactly this. Floating-point rounding decides the
//! edge case independently for each triangle, and the two answers need not agree.
//!
//! # How this module prevents it: Simulation of Simplicity
//!
//! Every degeneracy is removed by **symbolically perturbing the mesh vertices**
//! (Edelsbrunner–Mücke). Vertex `k`'s coordinate `l` is displaced by
//! `ε^(2^(3k+l))` for an infinitesimal `ε > 0`. Because the exponents are
//! distinct powers of two, no product of perturbations can equal another, so the
//! displacements are totally ordered and the perturbed point set is in general
//! position.
//!
//! The perturbation is never applied numerically. It only ever decides the sign
//! of a determinant that evaluated to exactly zero, by taking the sign of the
//! first non-vanishing term of the expansion.
//!
//! ## The expansion
//!
//! The edge function for the directed edge `v_i → v_j` against a ray through
//! points `O` and `P` is
//!
//! ```text
//! f = orient3d(O, P, v_i, v_j)
//! ```
//!
//! Differentiating with respect to each perturbed vertex:
//!
//! ```text
//! ∂f/∂δ_i =  (O - v_j) × (P - v_j)
//! ∂f/∂δ_j = -(O - v_i) × (P - v_i)
//! ```
//!
//! Each component of those cross products is a 2x2 determinant of coordinate
//! differences, which is precisely what [`orient2d`] evaluates — **exactly**. So
//! the whole cascade is decided by exact predicates, never by a float sign.
//!
//! When `f == 0`, the sign is taken from the first non-zero component, examined
//! in the order the perturbation hierarchy dictates: the lower-numbered vertex
//! first (its displacement is larger), and within a vertex, `x` then `y` then
//! `z`.
//!
//! ## Why this makes shared edges consistent
//!
//! Two triangles sharing an edge traverse it in opposite directions, so one
//! computes `f(v_i, v_j)` and the other `f(v_j, v_i)`. Swapping the arguments
//! negates `f`, and negates **both** partial derivatives. The examination order
//! is unchanged, so the first non-zero component found is the same one, with the
//! opposite sign.
//!
//! **The two triangles therefore always disagree, exactly.** Consistency is
//! structural — a consequence of the antisymmetry of the determinant — not
//! something asserted and hoped for. That is the whole point of choosing SoS
//! over a tie-break rule.
//!
//! ## What it buys at vertices
//!
//! A ray through a shared *vertex* is coplanar with every edge at that vertex,
//! because two lines through a common point are always coplanar. So two of the
//! three edge functions vanish for every triangle in the fan, and a naive rule
//! accepts all of them. Under SoS the vertex is displaced off the ray and the
//! perturbed ray passes through exactly one triangle of the fan — or none, if it
//! was only grazing. No fan traversal, no deduplication pass, no special case.
//!
//! # The one deviation, stated openly
//!
//! If the ray is **coplanar with the triangle's plane**, all three edge
//! functions vanish. SoS still decides hit-or-miss, but the intersection
//! *parameter* `t` is not determined by sign information: in the limit the ray
//! meets the perturbed triangle somewhere along the chord it cuts, and where
//! along that chord depends on the perturbation ratios rather than on any sign.
//!
//! Such a triangle is therefore **rejected**, and the occurrence is counted in
//! [`RayStats::coplanar_rejected`] so it is visible rather than silent. The
//! practical mitigation, which production dexel implementations use anyway, is
//! for U5 to place its ray lattice at cell centres rather than cell corners, so
//! rays do not lie in the planes of axis-aligned faces. `chipbreaker mesh parity`
//! reports the count so the choice can be checked rather than assumed.
//!
//! # Structure
//!
//! Flat node array, `u32` child indices, no pointer chasing. Built by **median
//! split on sorted centroid coordinates** along the longest axis of the node's
//! bounds, with ties broken by triangle index, so the tree is a pure function of
//! the mesh and its topology hash is identical on every target.
//!
//! Surface area heuristic is deliberately not implemented: bucketed SAH
//! accumulates floating-point costs whose summation order would then have to be
//! pinned, and median split is adequate for the coherent, near-parallel rays U5
//! actually issues. Revisit only with a benchmark that justifies it.

use core::fmt;

use crate::eps::EPS_EDGE_FN;
use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Axis, Ray, Vec2, Vec3};
use crate::mesh::TriMesh;
use crate::predicates::{ORIENT3D_COORDS, Orientation, orient2d, orient3d};

/// Triangles per leaf. Small enough that leaf scans stay cheap, large enough
/// that the tree does not degenerate into one node per triangle.
const LEAF_SIZE: usize = 4;

/// A ray-surface crossing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hit {
    /// Ray parameter: the crossing is at `ray.at(t)`.
    pub t: f64,
    /// Which triangle was crossed.
    pub triangle: u32,
    /// True if the ray is entering material here, false if leaving.
    ///
    /// Derived from the **common sign of the three exact edge functions**, not
    /// from `dot(dir, normal)`. A float dot product has a sign that is
    /// meaningless when the ray grazes the triangle, which is exactly the
    /// situation this module exists to handle. The edge-function sign is already
    /// exact and already SoS-resolved, so it costs nothing extra and it cannot
    /// be ambiguous.
    pub entering: bool,
}

impl Hashable for Hit {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Hit");
        h.f64(self.t);
        h.u64(u64::from(self.triangle));
        h.bool(self.entering);
        h.end();
    }
}

/// Counters describing how a ray query was answered.
///
/// Kept because the exact-fallback rate is a real engineering number: Unit 1
/// measured `orient3d` at roughly 17x the cost of the filtered path, so knowing
/// how often it fires sets the budget for U5 and U9.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RayStats {
    /// Ray-triangle tests performed.
    pub triangle_tests: u64,
    /// Tests where the float filter was trusted.
    pub fast_path: u64,
    /// Tests that escalated to exact predicates.
    pub exact_path: u64,
    /// Edge functions that were exactly zero and needed SoS.
    pub sos_resolutions: u64,
    /// Triangles rejected for being coplanar with the ray.
    pub coplanar_rejected: u64,
    /// BVH nodes visited.
    pub nodes_visited: u64,
}

impl RayStats {
    /// Adds another set of counters.
    pub fn merge(&mut self, other: &Self) {
        self.triangle_tests += other.triangle_tests;
        self.fast_path += other.fast_path;
        self.exact_path += other.exact_path;
        self.sos_resolutions += other.sos_resolutions;
        self.coplanar_rejected += other.coplanar_rejected;
        self.nodes_visited += other.nodes_visited;
    }

    /// Fraction of tests that took the exact path, in `[0, 1]`.
    #[must_use]
    pub fn exact_fraction(&self) -> f64 {
        if self.triangle_tests == 0 {
            0.0
        } else {
            self.exact_path as f64 / self.triangle_tests as f64
        }
    }
}

/// Why a ray query could not be answered.
#[derive(Debug, Clone, PartialEq)]
pub enum RayError {
    /// A ray coordinate lies outside the range in which `orient3d` is exact.
    OutOfRange {
        /// Which part of the ray: `"origin"` or `"direction"`.
        what: &'static str,
        /// The offending value.
        value: f64,
    },
    /// The direction was zero or non-finite, so the ray is not a line.
    DegenerateDirection {
        /// The direction as given.
        direction: Vec3,
    },
}

impl fmt::Display for RayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutOfRange { what, value } => write!(
                f,
                "ray {what} component {value:e} is outside the range \
                 [{:e}, {:e}] in which orient3d is exact",
                ORIENT3D_COORDS.min, ORIENT3D_COORDS.max
            ),
            Self::DegenerateDirection { direction } => {
                write!(f, "ray direction {direction:?} is zero or non-finite")
            }
        }
    }
}

impl core::error::Error for RayError {}

// ---------------------------------------------------------------------------
// Simulation of Simplicity
// ---------------------------------------------------------------------------

/// `orient2d` on the `(y, z)` components — the `x` component of a cross product.
#[inline]
fn orient2d_yz(a: Vec3, b: Vec3, c: Vec3) -> Orientation {
    orient2d(
        Vec2::new(a.y, a.z),
        Vec2::new(b.y, b.z),
        Vec2::new(c.y, c.z),
    )
}

/// `orient2d` on the `(z, x)` components — the `y` component of a cross product.
#[inline]
fn orient2d_zx(a: Vec3, b: Vec3, c: Vec3) -> Orientation {
    orient2d(
        Vec2::new(a.z, a.x),
        Vec2::new(b.z, b.x),
        Vec2::new(c.z, c.x),
    )
}

/// `orient2d` on the `(x, y)` components — the `z` component of a cross product.
#[inline]
fn orient2d_xy(a: Vec3, b: Vec3, c: Vec3) -> Orientation {
    orient2d(
        Vec2::new(a.x, a.y),
        Vec2::new(b.x, b.y),
        Vec2::new(c.x, c.y),
    )
}

/// The three components of `(O - v) × (P - v)`, each decided exactly.
///
/// This is `∂f/∂δ` for the vertex that does **not** appear in the cross product;
/// see the module documentation.
#[inline]
fn perturbation_coefficient(o: Vec3, p: Vec3, v: Vec3) -> [Orientation; 3] {
    [
        orient2d_yz(o, p, v),
        orient2d_zx(o, p, v),
        orient2d_xy(o, p, v),
    ]
}

/// Sign of `orient3d(O, P, v_i, v_j)` with degeneracies resolved by Simulation
/// of Simplicity. Never returns [`Orientation::Zero`].
///
/// `sos_used` is incremented when the symbolic path was taken, so the caller can
/// report how often degeneracy actually occurs.
fn sos_edge_function(
    o: Vec3,
    p: Vec3,
    vi: Vec3,
    i: u32,
    vj: Vec3,
    j: u32,
    sos_used: &mut u64,
) -> Orientation {
    let exact = orient3d(o, p, vi, vj);
    if exact != Orientation::Zero {
        return exact;
    }
    *sos_used += 1;

    // ∂f/∂δ_i =  (O - v_j) × (P - v_j)
    // ∂f/∂δ_j = -(O - v_i) × (P - v_i)
    let coeff_i = perturbation_coefficient(o, p, vj);
    let coeff_j = perturbation_coefficient(o, p, vi);

    // The lower-numbered vertex carries the larger perturbation, so its
    // coefficient is examined first; within a vertex, x before y before z.
    let (first, first_negated, second, second_negated) = if i < j {
        (coeff_i, false, coeff_j, true)
    } else {
        (coeff_j, true, coeff_i, false)
    };

    for (coeff, negated) in [(first, first_negated), (second, second_negated)] {
        for component in coeff {
            if component != Orientation::Zero {
                return if negated {
                    component.reverse()
                } else {
                    component
                };
            }
        }
    }

    // Both coefficients vanish entirely, which means the ray line passes through
    // both v_i and v_j — the ray *is* the edge line. Resolving this properly
    // needs the second-order term in δ_i × δ_j. It is a genuinely pathological
    // configuration, and what matters for correctness is that the answer stays
    // antisymmetric so the two triangles sharing the edge still disagree.
    // Vertex-index order is antisymmetric by construction.
    if i < j {
        Orientation::Positive
    } else {
        Orientation::Negative
    }
}

/// Result of testing one triangle against one ray.
enum TriangleTest {
    /// The ray crosses the triangle. `positive` is the common edge-function
    /// sign, which determines the crossing direction.
    Hit { positive: bool },
    /// The ray misses.
    Miss,
    /// The ray lies in the triangle's plane; see the module documentation.
    Coplanar,
}

/// Classifies a ray against a triangle using a float filter with an exact
/// fallback.
///
/// The float path is trusted only when every edge function is comfortably larger
/// than the accumulated rounding error of the terms that produced it — see
/// [`EPS_EDGE_FN`]. Anything closer escalates to the exact predicates, which is
/// where the whole leak-freedom argument lives.
fn classify(o: Vec3, p: Vec3, tri: [Vec3; 3], idx: [u32; 3], stats: &mut RayStats) -> TriangleTest {
    stats.triangle_tests += 1;

    // Fast path: the three edge functions in f64.
    //
    // Written as literally the same determinant `orient3d` evaluates —
    // det[O - v_j; P - v_j; v_i - v_j] — rather than the algebraically equal
    // D·((v_i - O) × (v_j - O)). Those two differ by a sign, and having the fast
    // and exact paths disagree on the convention makes `entering` depend on
    // which path happened to run. That is a bug that only shows up on grazing
    // rays, i.e. exactly where it is hardest to notice.
    // `fast_and_exact_paths_agree_in_sign` pins them together.
    let mut approx = [0.0f64; 3];
    let mut threshold = [0.0f64; 3];
    for k in 0..3 {
        let (a, b) = (k, (k + 1) % 3);
        let r1 = o - tri[b];
        let r2 = p - tri[b];
        let r3 = tri[a] - tri[b];
        approx[k] = r1.dot(r2.cross(r3));
        // The relative error of a 3x3 determinant of these rows is bounded by a
        // small multiple of eps times the product of the row magnitudes.
        threshold[k] =
            r1.abs().max_element() * r2.abs().max_element() * r3.abs().max_element() * EPS_EDGE_FN;
    }
    if (0..3).all(|k| threshold[k] > 0.0 && approx[k].abs() > threshold[k]) {
        stats.fast_path += 1;
        let positive = approx[0] > 0.0;
        return if approx.iter().all(|v| (*v > 0.0) == positive) {
            TriangleTest::Hit { positive }
        } else {
            TriangleTest::Miss
        };
    }

    // Exact path.
    stats.exact_path += 1;
    let mut sos_used = 0u64;
    let mut raw_zero = 0u32;
    let mut signs = [Orientation::Zero; 3];
    for (k, sign) in signs.iter_mut().enumerate() {
        let (a, b) = (k, (k + 1) % 3);
        if orient3d(o, p, tri[a], tri[b]) == Orientation::Zero {
            raw_zero += 1;
        }
        *sign = sos_edge_function(o, p, tri[a], idx[a], tri[b], idx[b], &mut sos_used);
    }
    stats.sos_resolutions += sos_used;

    // All three edge functions vanishing means the ray lies in the triangle's
    // plane, where `t` is not determined by sign information. See the module
    // documentation for why this is rejected rather than guessed at.
    if raw_zero == 3 {
        stats.coplanar_rejected += 1;
        return TriangleTest::Coplanar;
    }

    let positive = signs[0] == Orientation::Positive;
    if signs
        .iter()
        .all(|s| (*s == Orientation::Positive) == positive)
    {
        TriangleTest::Hit { positive }
    } else {
        TriangleTest::Miss
    }
}

/// The ray parameter at which the ray meets the triangle's plane.
///
/// Only called once [`classify`] has established that a crossing exists and that
/// the ray is not coplanar, so the denominator is non-zero.
#[inline]
fn plane_parameter(o: Vec3, d: Vec3, tri: [Vec3; 3]) -> Option<f64> {
    let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    let denom = d.dot(n);
    if denom == 0.0 || !denom.is_finite() {
        return None;
    }
    let t = (tri[0] - o).dot(n) / denom;
    if t.is_finite() { Some(t) } else { None }
}

// ---------------------------------------------------------------------------
// The hierarchy
// ---------------------------------------------------------------------------

/// One node of the flat hierarchy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BvhNode {
    /// Bounds of everything beneath this node, padded by a few ULP.
    pub bounds: Aabb3,
    /// For a leaf, the first index into the triangle order array. For an
    /// interior node, the index of the left child; the right child is `left + 1`.
    pub first_or_left: u32,
    /// Triangle count for a leaf; zero for an interior node.
    pub count: u32,
}

impl BvhNode {
    /// True if this node has no children.
    #[inline]
    #[must_use]
    pub const fn is_leaf(&self) -> bool {
        self.count > 0
    }
}

impl Hashable for BvhNode {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("BvhNode");
        self.bounds.hash_canonical(h);
        h.u64(u64::from(self.first_or_left));
        h.u64(u64::from(self.count));
        h.end();
    }
}

/// A bounding volume hierarchy over a mesh's triangles.
#[derive(Debug, Clone, PartialEq)]
pub struct Bvh {
    nodes: Vec<BvhNode>,
    /// Triangle indices, permuted so each leaf owns a contiguous run.
    order: Vec<u32>,
}

/// Widens a bound by `n` units in the last place, so that rounding in the slab
/// test can never cull a triangle the exact test would hit.
///
/// A relative epsilon would be scale-dependent and would need its own
/// justification; ULP steps are scale-free and exactly reproducible.
#[inline]
fn widen(value: f64, n: u32, up: bool) -> f64 {
    let mut v = value;
    for _ in 0..n {
        v = if up { v.next_up() } else { v.next_down() };
    }
    v
}

/// ULP of padding applied to every node bound.
///
/// Four covers the rounding of the bound itself plus a couple of operations in
/// the slab test. Being too generous costs a few extra leaf visits; being too
/// mean culls a real hit, which is a leak.
const BOUND_PAD_ULPS: u32 = 4;

fn pad(b: Aabb3) -> Aabb3 {
    if b.is_empty() {
        return b;
    }
    Aabb3::new(
        Vec3::new(
            widen(b.min.x, BOUND_PAD_ULPS, false),
            widen(b.min.y, BOUND_PAD_ULPS, false),
            widen(b.min.z, BOUND_PAD_ULPS, false),
        ),
        Vec3::new(
            widen(b.max.x, BOUND_PAD_ULPS, true),
            widen(b.max.y, BOUND_PAD_ULPS, true),
            widen(b.max.z, BOUND_PAD_ULPS, true),
        ),
    )
}

impl Bvh {
    /// Builds a hierarchy over every triangle of `mesh`.
    ///
    /// Median split on sorted centroids along the longest axis, ties broken by
    /// triangle index. Deterministic by construction: no floating-point cost
    /// accumulation, no hash iteration, no comparison whose result depends on
    /// anything but the mesh.
    #[must_use]
    pub fn build(mesh: &TriMesh) -> Self {
        let count = mesh.triangle_count() as usize;
        let mut order: Vec<u32> = (0..mesh.triangle_count()).collect();
        let centroids: Vec<Vec3> = (0..mesh.triangle_count())
            .map(|i| mesh.centroid(i))
            .collect();
        let tri_bounds: Vec<Aabb3> = (0..mesh.triangle_count())
            .map(|i| Aabb3::from_points(&mesh.triangle(i)))
            .collect();

        // A mesh with no triangles gets no nodes at all, rather than a root
        // node with `count == 0`. The leaf encoding is "count > 0", so such a
        // root would read as an interior node whose children are itself — which
        // is exactly the infinite recursion this shape exists to avoid.
        if count == 0 {
            return Self {
                nodes: Vec::new(),
                order,
            };
        }

        let mut nodes = Vec::with_capacity(count * 2);
        nodes.push(BvhNode {
            bounds: Aabb3::EMPTY,
            first_or_left: 0,
            count: 0,
        });
        build_range(&mut nodes, 0, &mut order, 0, count, &centroids, &tri_bounds);
        Self { nodes, order }
    }

    /// The flat node array. Node 0 is the root.
    #[inline]
    #[must_use]
    pub fn nodes(&self) -> &[BvhNode] {
        &self.nodes
    }

    /// The triangle index permutation leaves refer into.
    #[inline]
    #[must_use]
    pub fn order(&self) -> &[u32] {
        &self.order
    }

    /// Bounds of the whole hierarchy.
    #[must_use]
    pub fn bounds(&self) -> Aabb3 {
        self.nodes.first().map_or(Aabb3::EMPTY, |n| n.bounds)
    }

    /// Number of leaf nodes.
    #[must_use]
    pub fn leaf_count(&self) -> usize {
        self.nodes.iter().filter(|n| n.is_leaf()).count()
    }

    /// True if the hierarchy covers no triangles.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Greatest root-to-leaf depth; zero for an empty hierarchy.
    ///
    /// Recursive, which is safe here: median split halves the range at every
    /// level, so depth is `O(log n)` — at most 32 even for the four billion
    /// triangles a `u32` index could address.
    #[must_use]
    pub fn max_depth(&self) -> u32 {
        fn depth(nodes: &[BvhNode], i: u32) -> u32 {
            let n = &nodes[i as usize];
            if n.is_leaf() {
                1
            } else {
                1 + depth(nodes, n.first_or_left).max(depth(nodes, n.first_or_left + 1))
            }
        }
        if self.nodes.is_empty() {
            0
        } else {
            depth(&self.nodes, 0)
        }
    }

    /// Rejects rays the exact predicates cannot answer for.
    fn check_ray(ray: &Ray) -> Result<(), RayError> {
        if !ray.direction.is_finite() || ray.direction == Vec3::ZERO {
            return Err(RayError::DegenerateDirection {
                direction: ray.direction,
            });
        }
        for (what, v) in [("origin", ray.origin), ("direction", ray.direction)] {
            for c in v.to_array() {
                if !ORIENT3D_COORDS.contains(c) {
                    return Err(RayError::OutOfRange { what, value: c });
                }
            }
        }
        Ok(())
    }

    /// Every crossing of the **infinite line** through the ray, ascending in
    /// `t`, including crossings behind the origin at negative `t`.
    ///
    /// Line semantics rather than half-line, deliberately. U5 builds a dexel by
    /// taking the whole line through the workspace and partitioning it into
    /// inside and outside intervals; clipping at the origin would throw away the
    /// crossings that establish which side the ray starts on, and the caller
    /// would have to reconstruct them. [`Self::intersect_ray`] applies the
    /// `t >= 0` filter for callers that want a true ray.
    ///
    /// # Errors
    /// [`RayError`] if the ray is degenerate or lies outside the exact range of
    /// the predicates.
    pub fn intersect_ray_all(
        &self,
        mesh: &TriMesh,
        ray: &Ray,
    ) -> Result<(Vec<Hit>, RayStats), RayError> {
        let mut out = Vec::new();
        let stats = self.intersect_ray_all_into(mesh, ray, &mut out)?;
        Ok((out, stats))
    }

    /// [`Self::intersect_ray_all`], reusing the caller's buffer.
    ///
    /// This is the form U5 uses: millions of coherent rays, one scratch `Vec` per
    /// sweep, no allocation after the first ray.
    ///
    /// # Errors
    /// See [`Self::intersect_ray_all`].
    pub fn intersect_ray_all_into(
        &self,
        mesh: &TriMesh,
        ray: &Ray,
        out: &mut Vec<Hit>,
    ) -> Result<RayStats, RayError> {
        Self::check_ray(ray)?;
        out.clear();
        let mut stats = RayStats::default();
        if self.nodes.is_empty() || mesh.is_empty() {
            return Ok(stats);
        }

        // A second point on the ray, used by every predicate. Computed once so
        // that every triangle sees bit-identical ray geometry.
        let p = ray.origin + ray.direction;

        let inv = Vec3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );

        let mut stack: Vec<u32> = Vec::with_capacity(64);
        stack.push(0);
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index as usize];
            stats.nodes_visited += 1;
            if !slab_test(&node.bounds, ray.origin, ray.direction, inv) {
                continue;
            }
            if node.is_leaf() {
                let start = node.first_or_left as usize;
                for &tri_index in &self.order[start..start + node.count as usize] {
                    let tri = mesh.triangle(tri_index);
                    let idx = mesh.triangles()[tri_index as usize];
                    match classify(ray.origin, p, tri, idx, &mut stats) {
                        TriangleTest::Hit { positive } => {
                            if let Some(t) = plane_parameter(ray.origin, ray.direction, tri) {
                                out.push(Hit {
                                    t,
                                    triangle: tri_index,
                                    // A positive common edge-function sign means
                                    // the ray meets the triangle's front face —
                                    // equivalently `dot(dir, normal) < 0` — which
                                    // for an outward-oriented surface is the face
                                    // it enters through. Pinned by
                                    // `entering_agrees_with_the_face_normal`.
                                    entering: positive,
                                });
                            }
                        }
                        TriangleTest::Miss | TriangleTest::Coplanar => {}
                    }
                }
            } else {
                stack.push(node.first_or_left);
                stack.push(node.first_or_left + 1);
            }
        }

        // Ascending t, ties broken by triangle index so the order is total and
        // reproducible. `total_cmp` rather than `partial_cmp`: it is a total
        // order over every f64 and needs no unwrap.
        out.sort_by(|a, b| a.t.total_cmp(&b.t).then(a.triangle.cmp(&b.triangle)));
        Ok(stats)
    }

    /// The nearest crossing with `t >= 0`, if any.
    ///
    /// # Errors
    /// See [`Self::intersect_ray_all`].
    pub fn intersect_ray(&self, mesh: &TriMesh, ray: &Ray) -> Result<Option<Hit>, RayError> {
        let (hits, _) = self.intersect_ray_all(mesh, ray)?;
        Ok(hits.into_iter().find(|h| h.t >= 0.0))
    }

    /// Appends every triangle whose bounds overlap `query` to `out`, ascending.
    ///
    /// Used by the self-intersection check to find candidate pairs. The result
    /// is sorted so that the caller's pair enumeration is order-independent;
    /// without that, the findings would depend on the traversal order and the
    /// report would not be reproducible.
    pub fn query_aabb(&self, query: &Aabb3, out: &mut Vec<u32>) {
        out.clear();
        if self.nodes.is_empty() || query.is_empty() {
            return;
        }
        let mut stack = vec![0u32];
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index as usize];
            if !node.bounds.intersects(query) {
                continue;
            }
            if node.is_leaf() {
                let start = node.first_or_left as usize;
                out.extend_from_slice(&self.order[start..start + node.count as usize]);
            } else {
                stack.push(node.first_or_left);
                stack.push(node.first_or_left + 1);
            }
        }
        out.sort_unstable();
    }

    /// Canonical digest of the tree's shape and bounds.
    ///
    /// Hashed into the golden suite: the tree must be identical on Windows,
    /// Linux, macOS and WASM, because a different tree means a different
    /// traversal order and, eventually, a different answer.
    #[must_use]
    pub fn topology_digest(&self) -> crate::golden::Digest {
        self.canonical_digest()
    }
}

impl Hashable for Bvh {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Bvh");
        h.add_all(self.nodes.iter());
        h.u64_slice(&self.order.iter().map(|v| u64::from(*v)).collect::<Vec<_>>());
        h.end();
    }
}

/// Recursively builds `range` of `order` into `node_index`.
fn build_range(
    nodes: &mut Vec<BvhNode>,
    node_index: usize,
    order: &mut [u32],
    start: usize,
    end: usize,
    centroids: &[Vec3],
    tri_bounds: &[Aabb3],
) {
    let mut bounds = Aabb3::EMPTY;
    for &i in &order[start..end] {
        bounds = bounds.union(&tri_bounds[i as usize]);
    }
    let bounds = pad(bounds);

    let count = end - start;
    if count <= LEAF_SIZE {
        nodes[node_index] = BvhNode {
            bounds,
            first_or_left: start as u32,
            count: count as u32,
        };
        return;
    }

    // Split along the longest axis of the node's bounds. Aabb3::longest_axis
    // breaks ties toward the lower axis, which is what makes the choice a pure
    // function of the geometry.
    let axis = bounds.longest_axis();
    let key = |i: u32| centroids[i as usize][axis.index()];
    // `total_cmp` for a total order including NaN — impossible here, since
    // TriMesh rejects non-finite coordinates, but a sort comparator that can
    // panic has no business in the hot path. Ties fall back to the triangle
    // index so the permutation is unique.
    order[start..end].sort_by(|a, b| key(*a).total_cmp(&key(*b)).then(a.cmp(b)));

    let mid = start + count / 2;
    let left = nodes.len() as u32;
    nodes.push(BvhNode {
        bounds: Aabb3::EMPTY,
        first_or_left: 0,
        count: 0,
    });
    nodes.push(BvhNode {
        bounds: Aabb3::EMPTY,
        first_or_left: 0,
        count: 0,
    });
    nodes[node_index] = BvhNode {
        bounds,
        first_or_left: left,
        count: 0,
    };

    build_range(
        nodes,
        left as usize,
        order,
        start,
        mid,
        centroids,
        tri_bounds,
    );
    build_range(
        nodes,
        left as usize + 1,
        order,
        mid,
        end,
        centroids,
        tri_bounds,
    );
}

/// Conservative ray-box overlap.
///
/// Written to avoid `0 * inf`, which produces NaN and would make the comparison
/// silently false — culling a box the ray passes straight through. Axes with a
/// zero direction component are handled by a containment test instead.
///
/// Errs toward visiting: a false positive costs a leaf scan, a false negative is
/// a leaked ray.
fn slab_test(b: &Aabb3, origin: Vec3, dir: Vec3, inv: Vec3) -> bool {
    if b.is_empty() {
        return false;
    }
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let a = axis.index();
        let (lo, hi) = (b.min[a], b.max[a]);
        let d = dir[a];
        if d == 0.0 {
            if origin[a] < lo || origin[a] > hi {
                return false;
            }
            continue;
        }
        let mut t1 = (lo - origin[a]) * inv[a];
        let mut t2 = (hi - origin[a]) * inv[a];
        if t1 > t2 {
            core::mem::swap(&mut t1, &mut t2);
        }
        t_min = t_min.max(t1);
        t_max = t_max.min(t2);
        if t_min > t_max {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::MeshMeta;
    use crate::mesh::tests::unit_cube;

    fn ray(origin: Vec3, dir: Vec3) -> Ray {
        Ray::new(origin, dir)
    }

    #[test]
    fn a_ray_through_the_middle_of_a_cube_crosses_twice() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(0.5, 0.5, -1.0), Vec3::Z))
            .expect("valid ray");
        assert_eq!(hits.len(), 2, "in and out: {hits:?}");
        assert!((hits[0].t - 1.0).abs() < 1e-15);
        assert!((hits[1].t - 2.0).abs() < 1e-15);
    }

    #[test]
    fn cube_crossings_alternate_enter_then_exit() {
        // Pins the `entering` convention that U5 will rely on.
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(0.5, 0.5, -1.0), Vec3::Z))
            .expect("valid ray");
        assert!(hits[0].entering, "the first crossing must enter material");
        assert!(!hits[1].entering, "the second must leave it");
    }

    #[test]
    fn entering_agrees_with_the_face_normal() {
        // `entering` is derived from the exact edge-function sign rather than
        // from `dot(dir, normal)`, because a float dot product is meaningless
        // when the ray grazes a face. This ties the exact convention back to the
        // geometric one on configurations where the float answer *is* reliable,
        // so the two definitions cannot drift apart unnoticed.
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        for origin in [
            Vec3::new(0.3, 0.7, -2.0),
            Vec3::new(0.6, 0.2, -2.0),
            Vec3::new(-2.0, 0.4, 0.8),
        ] {
            let dir = (Vec3::splat(0.5) - origin).normalize().expect("non-zero");
            let (hits, _) = bvh
                .intersect_ray_all(&m, &ray(origin, dir))
                .expect("valid ray");
            assert!(!hits.is_empty());
            for h in &hits {
                let n = m.face_normal(h.triangle).expect("non-degenerate");
                let facing = dir.dot(n);
                assert!(facing.abs() > 1e-9, "not a grazing configuration");
                assert_eq!(
                    h.entering,
                    facing < 0.0,
                    "entering disagrees with the normal for {h:?}"
                );
            }
        }
    }

    #[test]
    fn nearest_hit_ignores_crossings_behind_the_origin() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        // Origin inside the cube: one crossing behind, one ahead.
        let hit = bvh
            .intersect_ray(&m, &ray(Vec3::splat(0.5), Vec3::Z))
            .expect("valid ray")
            .expect("must hit the far face");
        assert!(hit.t > 0.0);
        assert!((hit.t - 0.5).abs() < 1e-15);
    }

    #[test]
    fn a_ray_that_misses_reports_nothing() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(5.0, 5.0, -1.0), Vec3::Z))
            .expect("valid ray");
        assert!(hits.is_empty());
    }

    #[test]
    fn hits_come_back_sorted_by_t() {
        // Two nested cubes give four crossings.
        let outer = unit_cube();
        let mut v: Vec<Vec3> = outer.vertices().iter().map(|p| *p * 10.0).collect();
        let mut t = outer.triangles().to_vec();
        let off = v.len() as u32;
        v.extend(outer.vertices().iter().map(|p| *p * 2.0 + Vec3::splat(4.0)));
        t.extend(
            outer
                .triangles()
                .iter()
                .map(|x| [x[0] + off, x[1] + off, x[2] + off]),
        );
        let m = TriMesh::new(v, t, MeshMeta::synthetic()).expect("valid");
        let bvh = Bvh::build(&m);
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(5.0, 5.0, -1.0), Vec3::Z))
            .expect("valid ray");
        assert_eq!(hits.len(), 4);
        for w in hits.windows(2) {
            assert!(w[0].t <= w[1].t, "not sorted: {hits:?}");
        }
        // Nested solids do NOT alternate: the ray enters the outer shell, then
        // enters the inner one, then leaves each in turn. Alternation is a
        // property of a single closed shell, not of a scene — which is exactly
        // why U5 must track a depth counter rather than a boolean. Recorded here
        // because getting this wrong is an easy way to lose the inner solid.
        let entering: Vec<bool> = hits.iter().map(|h| h.entering).collect();
        assert_eq!(entering, vec![true, true, false, false], "{hits:?}");

        // What *is* invariant: the running depth never goes negative and returns
        // to zero, which is the real parity condition.
        let mut depth = 0i32;
        for h in &hits {
            depth += if h.entering { 1 } else { -1 };
            assert!(depth >= 0, "depth went negative at {h:?}");
        }
        assert_eq!(depth, 0, "the ray must leave everything it entered");
    }

    #[test]
    fn the_scratch_buffer_form_agrees_and_clears() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let r = ray(Vec3::new(0.5, 0.5, -1.0), Vec3::Z);
        let mut scratch = vec![Hit {
            t: 99.0,
            triangle: 999,
            entering: false,
        }];
        bvh.intersect_ray_all_into(&m, &r, &mut scratch)
            .expect("valid");
        let (expected, _) = bvh.intersect_ray_all(&m, &r).expect("valid");
        assert_eq!(scratch, expected);
        // A miss must empty the buffer rather than leave stale hits.
        bvh.intersect_ray_all_into(&m, &ray(Vec3::splat(50.0), Vec3::Z), &mut scratch)
            .expect("valid");
        assert!(scratch.is_empty());
    }

    #[test]
    fn degenerate_and_out_of_range_rays_are_rejected() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        assert!(matches!(
            bvh.intersect_ray_all(&m, &ray(Vec3::ZERO, Vec3::ZERO)),
            Err(RayError::DegenerateDirection { .. })
        ));
        assert!(matches!(
            bvh.intersect_ray_all(&m, &ray(Vec3::ZERO, Vec3::new(f64::NAN, 0.0, 1.0))),
            Err(RayError::DegenerateDirection { .. })
        ));
        let far = Vec3::new(ORIENT3D_COORDS.max * 10.0, 0.0, 0.0);
        assert!(matches!(
            bvh.intersect_ray_all(&m, &ray(far, Vec3::Z)),
            Err(RayError::OutOfRange { what: "origin", .. })
        ));
    }

    // --- Simulation of Simplicity -------------------------------------------

    #[test]
    fn sos_is_antisymmetric_which_is_what_makes_shared_edges_consistent() {
        // The central claim of the module. For every configuration, swapping the
        // edge's endpoints must flip the sign — including, and especially, when
        // the exact determinant is zero and SoS decides.
        let o = Vec3::new(0.5, 0.5, -1.0);
        let p = o + Vec3::Z;
        let cases = [
            // Generic.
            (
                Vec3::new(0.0, 0.0, 0.0),
                0u32,
                Vec3::new(1.0, 0.0, 0.0),
                1u32,
            ),
            // The ray line is coplanar with the edge: exact result is zero.
            (Vec3::new(0.5, 0.0, 0.0), 3, Vec3::new(0.5, 1.0, 0.0), 7),
            // The ray passes through one endpoint.
            (Vec3::new(0.5, 0.5, 0.0), 2, Vec3::new(1.0, 0.0, 0.0), 5),
            // ... and through the other.
            (Vec3::new(1.0, 0.0, 0.0), 9, Vec3::new(0.5, 0.5, 4.0), 4),
            // The ray IS the edge line: the pathological fallback.
            (Vec3::new(0.5, 0.5, 0.0), 6, Vec3::new(0.5, 0.5, 1.0), 8),
        ];
        for (vi, i, vj, j) in cases {
            let mut n = 0;
            let forward = sos_edge_function(o, p, vi, i, vj, j, &mut n);
            let backward = sos_edge_function(o, p, vj, j, vi, i, &mut n);
            assert_ne!(forward, Orientation::Zero, "SoS must never return zero");
            assert_eq!(
                forward,
                backward.reverse(),
                "antisymmetry failed for {vi:?}#{i} -> {vj:?}#{j}"
            );
        }
    }

    #[test]
    fn fast_and_exact_paths_agree_in_sign() {
        // The two paths must compute the *same* quantity, not merely
        // algebraically related ones. They differed by a sign once, which made
        // `entering` depend on which path ran — invisible on axis-aligned tests
        // and wrong everywhere else.
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};
        let mut rng = StdRng::seed_from_u64(0x0000_C41B_0000_0100);
        let mut checked = 0u32;
        for _ in 0..2_000 {
            let mut pt = || {
                Vec3::new(
                    rng.random_range(-2.0..2.0),
                    rng.random_range(-2.0..2.0),
                    rng.random_range(-2.0..2.0),
                )
            };
            let (o, p, vi, vj) = (pt(), pt(), pt(), pt());
            let r1 = o - vj;
            let r2 = p - vj;
            let r3 = vi - vj;
            let approx = r1.dot(r2.cross(r3));
            let exact = orient3d(o, p, vi, vj);
            // Only compare where the float value is unambiguous.
            let scale = r1.abs().max_element() * r2.abs().max_element() * r3.abs().max_element();
            if approx.abs() > scale * 1e-9 {
                let approx_positive = approx > 0.0;
                assert_eq!(
                    approx_positive,
                    exact == Orientation::Positive,
                    "fast path and orient3d disagree in sign for {o:?} {p:?} {vi:?} {vj:?}"
                );
                checked += 1;
            }
        }
        assert!(checked > 1_000, "only {checked} unambiguous cases");
    }

    #[test]
    fn sos_fires_only_when_the_exact_predicate_is_degenerate() {
        let o = Vec3::new(0.5, 0.5, -1.0);
        let p = o + Vec3::Z;
        let mut n = 0;
        // Generic edge: no SoS needed.
        sos_edge_function(o, p, Vec3::ZERO, 0, Vec3::new(1.0, 0.0, 0.0), 1, &mut n);
        assert_eq!(n, 0);
        // Coplanar with the ray: SoS decides.
        sos_edge_function(
            o,
            p,
            Vec3::new(0.5, 0.0, 0.0),
            0,
            Vec3::new(0.5, 1.0, 0.0),
            1,
            &mut n,
        );
        assert_eq!(n, 1);
    }

    #[test]
    fn a_ray_exactly_through_a_shared_edge_crosses_once_not_twice() {
        // The case the whole module exists for. The cube's face z=0 is split
        // along the diagonal from (0,0,0) to (1,1,0); a ray up that diagonal
        // passes exactly through the shared edge of two triangles.
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let (hits, stats) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(0.25, 0.25, -1.0), Vec3::Z))
            .expect("valid ray");
        assert_eq!(
            hits.len() % 2,
            0,
            "odd crossing count means a leak: {hits:?}"
        );
        assert_eq!(hits.len(), 2, "{hits:?}");
        assert!(
            stats.exact_path > 0,
            "this configuration must exercise the exact path"
        );
    }

    #[test]
    fn a_ray_exactly_through_a_vertex_crosses_evenly() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        // Straight up through the corner (0,0,0) and out through (0,0,1).
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(0.0, 0.0, -1.0), Vec3::Z))
            .expect("valid ray");
        assert_eq!(
            hits.len() % 2,
            0,
            "odd crossing count at a vertex: {hits:?}"
        );
    }

    // --- The hierarchy ------------------------------------------------------

    #[test]
    fn the_tree_covers_every_triangle_exactly_once() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let mut seen: Vec<u32> = bvh.order().to_vec();
        seen.sort_unstable();
        assert_eq!(seen, (0..m.triangle_count()).collect::<Vec<_>>());
        let leaf_total: u32 = bvh
            .nodes()
            .iter()
            .filter(|n| n.is_leaf())
            .map(|n| n.count)
            .sum();
        assert_eq!(leaf_total, m.triangle_count());
    }

    #[test]
    fn node_bounds_contain_their_children() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        for node in bvh.nodes() {
            if !node.is_leaf() {
                let l = bvh.nodes()[node.first_or_left as usize].bounds;
                let r = bvh.nodes()[node.first_or_left as usize + 1].bounds;
                assert!(node.bounds.contains_aabb(&l));
                assert!(node.bounds.contains_aabb(&r));
            }
        }
        assert!(bvh.bounds().contains_aabb(&m.bounds()));
    }

    #[test]
    fn tree_shape_is_a_pure_function_of_the_mesh() {
        let m = unit_cube();
        let a = Bvh::build(&m);
        let b = Bvh::build(&m);
        assert_eq!(a, b);
        assert_eq!(a.topology_digest(), b.topology_digest());
        // A different mesh gives a different tree.
        let mut v = m.vertices().to_vec();
        v[0] = Vec3::new(-1.0, 0.0, 0.0);
        let m2 = TriMesh::new(v, m.triangles().to_vec(), MeshMeta::synthetic()).expect("valid");
        assert_ne!(a.topology_digest(), Bvh::build(&m2).topology_digest());
    }

    #[test]
    fn an_empty_mesh_builds_a_usable_empty_tree() {
        let m = TriMesh::new(Vec::new(), Vec::new(), MeshMeta::synthetic()).expect("valid");
        let bvh = Bvh::build(&m);
        let (hits, _) = bvh
            .intersect_ray_all(&m, &ray(Vec3::ZERO, Vec3::Z))
            .expect("valid ray");
        assert!(hits.is_empty());
        assert!(bvh.is_empty());
        assert_eq!(bvh.max_depth(), 0, "no nodes means no depth");
        assert_eq!(bvh.leaf_count(), 0);
        assert_eq!(bvh.bounds(), Aabb3::EMPTY);
    }

    #[test]
    fn every_leaf_owns_at_least_one_triangle() {
        // The leaf encoding is "count > 0", so a zero-count leaf would be read
        // as an interior node pointing at itself.
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        for (i, node) in bvh.nodes().iter().enumerate() {
            if node.is_leaf() {
                assert!(node.count > 0, "node {i} is a leaf with no triangles");
            } else {
                assert!(
                    (node.first_or_left as usize) != i,
                    "node {i} is its own child"
                );
                assert!((node.first_or_left as usize + 1) < bvh.nodes().len());
            }
        }
    }

    #[test]
    fn the_slab_test_never_culls_a_box_the_ray_enters() {
        let b = Aabb3::new(Vec3::ZERO, Vec3::ONE);
        let inv = |d: Vec3| Vec3::new(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
        // Straight through.
        assert!(slab_test(
            &b,
            Vec3::new(0.5, 0.5, -1.0),
            Vec3::Z,
            inv(Vec3::Z)
        ));
        // Along a face, where a zero direction component would make 0 * inf NaN.
        assert!(slab_test(
            &b,
            Vec3::new(0.5, 0.0, -1.0),
            Vec3::Z,
            inv(Vec3::Z)
        ));
        assert!(slab_test(
            &b,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::Z,
            inv(Vec3::Z)
        ));
        // Genuine misses: the *line* passes nowhere near the box.
        assert!(!slab_test(
            &b,
            Vec3::new(2.0, 0.5, -1.0),
            Vec3::Z,
            inv(Vec3::Z)
        ));
        assert!(!slab_test(
            &b,
            Vec3::new(0.5, 2.0, -1.0),
            Vec3::Z,
            inv(Vec3::Z)
        ));
        assert!(!slab_test(&Aabb3::EMPTY, Vec3::ZERO, Vec3::Z, inv(Vec3::Z)));

        // Pointing away is NOT a miss: the traversal tests the infinite line,
        // because `intersect_ray_all` reports crossings at negative t too.
        // Clipping here would silently drop them.
        assert!(slab_test(
            &b,
            Vec3::new(0.5, 0.5, -1.0),
            -Vec3::Z,
            inv(-Vec3::Z)
        ));
    }

    #[test]
    fn bounds_are_padded_outward_only() {
        let b = Aabb3::new(Vec3::ZERO, Vec3::ONE);
        let p = pad(b);
        assert!(p.contains_aabb(&b), "padding must never shrink a bound");
        assert!(p.min.x < 0.0 && p.max.x > 1.0);
        assert_eq!(pad(Aabb3::EMPTY), Aabb3::EMPTY);
    }

    #[test]
    fn stats_count_what_they_claim_to() {
        let m = unit_cube();
        let bvh = Bvh::build(&m);
        let (_, stats) = bvh
            .intersect_ray_all(&m, &ray(Vec3::new(0.5, 0.5, -1.0), Vec3::Z))
            .expect("valid ray");
        assert!(stats.nodes_visited > 0);
        assert!(stats.triangle_tests > 0);
        assert_eq!(stats.fast_path + stats.exact_path, stats.triangle_tests);
        assert!(stats.exact_fraction() >= 0.0 && stats.exact_fraction() <= 1.0);

        let mut merged = RayStats::default();
        merged.merge(&stats);
        merged.merge(&stats);
        assert_eq!(merged.triangle_tests, stats.triangle_tests * 2);
        assert_eq!(RayStats::default().exact_fraction(), 0.0);
    }
}
