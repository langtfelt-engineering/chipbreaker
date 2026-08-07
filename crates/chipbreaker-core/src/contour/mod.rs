// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Manifold dual contouring: a tri-dexel field back to a triangle mesh.
//!
//! # The grid is the three bundles
//!
//! Dual contouring wants a cell grid with an inside/outside sign at every corner
//! and a crossing on every sign-changing edge. A tri-dexel field is already
//! exactly that, provided the grid is chosen so its **corners are ray
//! positions**: then an X-directed edge is a sub-segment of an X-bundle ray, a
//! Y edge of a Y-bundle ray, and a Z edge of a Z-bundle ray. Every one of a
//! cell's twelve edges is covered, by construction, and the three bundles are
//! not three approximations of one surface but the three edge directions of one
//! grid.
//!
//! That requires the bundles to be co-registered, which `TriDexelField` now
//! asserts. It also puts this grid half a cell from the dexel cell grid — dexel
//! cell centres are these corners — which is a relabelling and not a violation
//! of Unit 5's rule that ray origins avoid the stock's faces.
//!
//! # Each edge uses its own bundle, and nothing else
//!
//! An X edge's crossing comes from the X bundle. Never from an interpolation
//! across bundles, never from a blend. The whole architecture rests on the field
//! being *exact along the ray*, and a crossing taken from anywhere but the ray
//! it lies on would resample it away. This is also why the corner signs can
//! disagree and the crossings still be trustworthy: disagreement is a property
//! of the corner, not of the edge.
//!
//! # Signs: majority of three
//!
//! Each corner lies on one ray of each bundle, and the three were cut
//! independently, so they can disagree by `O(h)` with independent signs. The
//! rule is a majority of the three: symmetric, deterministic, no bundle
//! privileged. The disagreement rate is measured and reported, because it is a
//! direct reading of how far the three fields differ on real geometry rather
//! than an abstract error bound.
//!
//! # Why *manifold* dual contouring
//!
//! Plain DC puts one vertex in every cell. That is wrong whenever a cell's sign
//! configuration has more than one surface component — a thin wall passing
//! through, two opposite corners inside and the rest out, a near-tangential cut
//! grazing a cell. One vertex then has to serve two disconnected sheets, and the
//! result is non-manifold: an edge with four incident triangles instead of two.
//!
//! Both configurations occur in this corpus, so the fix cannot be a patch
//! applied afterwards. Instead each cell partitions its **inside** corners into
//! connected components, using the cell's own twelve edges as the adjacency, and
//! emits one vertex per component. A cell with a simple configuration has one
//! component and behaves exactly like plain DC; only the ambiguous ones split.
//!
//! An edge that generates a quad has exactly one inside endpoint — that is what
//! a sign change means — so the vertex it should use in each of the four
//! surrounding cells is unambiguous: the one for the component containing that
//! cell's copy of the inside corner. Two disconnected sheets in a cell therefore
//! draw from two different vertices and never meet.

pub mod qef;

use crate::dexel::tri::{AXES, TriDexelField};
use crate::math::{Axis, OctNormal, Vec3};
use crate::mesh::{MeshMeta, TriMesh};

use qef::Qef;

/// How far outside its cell a vertex may be placed, as a fraction of the cell.
///
/// **Not optional.** An ill-conditioned QEF puts its minimiser far outside the
/// cell it belongs to, and one such vertex produces a self-intersecting mesh
/// that still passes every manifold and watertightness check — the failure is
/// invisible to exactly the tests that are supposed to catch failures.
///
/// A little slack rather than none, because a sharp corner's true position can
/// legitimately sit slightly outside the cell that detected it, and clamping
/// hard to the cell would round off the feature the QEF just recovered.
pub const DEFAULT_CLAMP_EXPAND: f64 = 0.5;

/// Options for [`extract`].
#[derive(Debug, Clone, Copy)]
pub struct ContourOptions {
    /// Vertex clamping slack; see [`DEFAULT_CLAMP_EXPAND`].
    pub clamp_expand: f64,
    /// Use the stored normals.
    ///
    /// `false` discards them and falls back to the centroid of the crossings,
    /// which is plain surface nets: manifold, watertight, smooth, and with every
    /// sharp edge rounded off. It exists to measure what the normals bought,
    /// and to read a version 2 `.tdx` honestly.
    pub use_normals: bool,
}

impl Default for ContourOptions {
    fn default() -> Self {
        Self {
            clamp_expand: DEFAULT_CLAMP_EXPAND,
            use_normals: true,
        }
    }
}

/// What an extraction did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ContourStats {
    /// Grid corners classified.
    pub corners: u64,
    /// Corners where the three bundles did not agree.
    ///
    /// The direct measurement of how far the three fields differ. Not an error:
    /// it is the `O(h)` disagreement that has been pinned per-bundle in the
    /// corpus since Unit 6, arriving where it finally has to be resolved.
    pub corner_disagreements: u64,
    /// Corners where only one or two bundles could answer.
    ///
    /// A majority of three needs three. At the very edge of the workspace a
    /// corner can lie outside another bundle's transverse range, and there the
    /// available votes decide.
    pub corners_short_of_three_votes: u64,
    /// Edges with a sign change, and so a crossing.
    pub crossing_edges: u64,
    /// Edges carrying more than one crossing.
    ///
    /// A feature thinner than a cell. The pair bracketing the sign change is
    /// used and the rest are dropped, so this count is the honest measure of
    /// what the resolution is missing.
    pub multi_crossing_edges: u64,
    /// Edges whose sign change had no crossing on the owning ray.
    ///
    /// Should be zero. Non-zero means a corner sign and its own bundle's spans
    /// disagree, which would be an internal inconsistency rather than sampling.
    pub sign_change_without_crossing: u64,
    /// Cells that produced at least one vertex.
    pub cells_with_vertices: u64,
    /// Cells that needed more than one, which plain DC would have got wrong.
    pub cells_with_multiple_vertices: u64,
    /// Vertices whose QEF was rank 1, 2 and 3: flats, edges and corners.
    pub rank_histogram: [u64; 4],
    /// Vertices clamped back into their cell.
    pub clamped_vertices: u64,
}

/// Why an extraction could not run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ContourError {
    /// A bundle is missing. All three are needed: they are the three edge
    /// directions of the grid, and two of them cover only two thirds of a
    /// cell's edges.
    MissingBundle(&'static str),
    /// The grid is too small to contain a cell.
    TooSmall {
        /// Corner counts per axis.
        counts: [usize; 3],
    },
    /// The mesh could not be assembled.
    Mesh(String),
}

impl core::fmt::Display for ContourError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::MissingBundle(axis) => write!(
                f,
                "the {axis} bundle is missing. Dual contouring needs all three, \
                 because they are the three edge directions of one grid and \
                 without one a third of every cell's edges are uncovered. Rebuild \
                 the field with `--axes xyz`."
            ),
            Self::TooSmall { counts } => write!(
                f,
                "the grid is {counts:?} corners, which contains no complete cell"
            ),
            Self::Mesh(e) => write!(f, "the extracted mesh was rejected: {e}"),
        }
    }
}

impl std::error::Error for ContourError {}

/// A crossing on one grid edge.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    /// Position along the edge's own axis.
    at: f64,
    /// Outward normal there.
    normal: OctNormal,
}

/// The eight corners of a cell, as `(dx, dy, dz)` bit flags.
const fn corner_offset(c: usize) -> (usize, usize, usize) {
    (c & 1, (c >> 1) & 1, (c >> 2) & 1)
}

/// The twelve edges of a cell, as `(corner, corner, axis)`.
///
/// Fixed order, because everything downstream that iterates them inherits its
/// determinism from here.
const CELL_EDGES: [(usize, usize, usize); 12] = [
    (0, 1, 0),
    (2, 3, 0),
    (4, 5, 0),
    (6, 7, 0),
    (0, 2, 1),
    (1, 3, 1),
    (4, 6, 1),
    (5, 7, 1),
    (0, 4, 2),
    (1, 5, 2),
    (2, 6, 2),
    (3, 7, 2),
];

/// Per-cell vertex bookkeeping.
///
/// At most four vertices: the largest set of pairwise non-adjacent corners of a
/// cube is the four alternating ones, so no configuration can produce more
/// components than that.
#[derive(Debug, Clone, Copy)]
struct CellVertices {
    /// Which component each corner belongs to, or `u8::MAX` for an outside
    /// corner.
    component_of: [u8; 8],
    /// Mesh vertex index per component.
    vertex: [u32; 4],
}

impl Default for CellVertices {
    fn default() -> Self {
        Self {
            component_of: [u8::MAX; 8],
            vertex: [u32::MAX; 4],
        }
    }
}

/// The grid, its signs and its crossings.
struct Grid<'a> {
    field: &'a TriDexelField,
    /// Corner ordinates per world axis.
    coords: [Vec<f64>; 3],
    /// Corner counts per world axis.
    n: [usize; 3],
    /// Inside/outside per corner, indexed by [`Grid::corner_index`].
    inside: Vec<bool>,
    /// Crossings per axis-directed edge, indexed by [`Grid::edge_index`].
    crossings: [Vec<Option<Crossing>>; 3],
    spacing: f64,
}

impl<'a> Grid<'a> {
    fn corner_index(&self, i: usize, j: usize, k: usize) -> usize {
        i + self.n[0] * (j + self.n[1] * k)
    }

    /// Index of the edge from corner `(i, j, k)` along `axis`.
    fn edge_index(&self, axis: usize, i: usize, j: usize, k: usize) -> usize {
        let mut e = self.n;
        e[axis] -= 1;
        i + e[0] * (j + e[1] * k)
    }

    fn corner_position(&self, i: usize, j: usize, k: usize) -> Vec3 {
        Vec3::new(self.coords[0][i], self.coords[1][j], self.coords[2][k])
    }
}

/// Extracts a triangle mesh from a cut field.
///
/// # Errors
/// See [`ContourError`].
///
/// # Panics
/// Panics only on an internal inconsistency that the type system cannot
/// express, such as a component index exceeding four.
pub fn extract(
    field: &TriDexelField,
    options: &ContourOptions,
) -> Result<(TriMesh, ContourStats), ContourError> {
    let mut stats = ContourStats::default();

    for axis in AXES {
        if field.bundle(axis).is_none() {
            return Err(ContourError::MissingBundle(axis.as_str()));
        }
    }

    let mut coords: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for axis in AXES {
        coords[axis.index()] = field
            .corner_coordinates(axis)
            .ok_or(ContourError::MissingBundle(axis.as_str()))?;
    }
    // **A ring of virtual corners around the whole grid.**
    //
    // The ray positions span only the workspace interior -- the first is half a
    // cell in -- so the stock's own outer surface lies outside the corner grid
    // and there is no sign change to reconstruct it from. Extracting without
    // this produces the slot walls and not the block they were cut into: a
    // surface open at every boundary, which is exactly the hole this unit may
    // not ship.
    //
    // One extra corner at each end, a cell beyond, is enough. The rays already
    // start a cell before the workspace and end a cell after, so a virtual
    // corner still has a well-defined ray parameter and the boundary crossing is
    // a real span endpoint on a real ray. The other two bundles have no
    // transverse coordinate there, so such a corner gets fewer votes -- and with
    // no vote for material it classifies as outside, which is the truth.
    let spacing = field.bundle(Axis::X).map_or(1.0, |b| b.lattice().spacing());
    for c in &mut coords {
        let (first, last) = (c[0], c[c.len() - 1]);
        c.insert(0, first - spacing);
        c.push(last + spacing);
    }

    let n = [coords[0].len(), coords[1].len(), coords[2].len()];
    if n[0] < 2 || n[1] < 2 || n[2] < 2 {
        return Err(ContourError::TooSmall { counts: n });
    }
    let mut grid = Grid {
        field,
        coords,
        n,
        inside: vec![false; n[0] * n[1] * n[2]],
        crossings: [
            vec![None; (n[0] - 1) * n[1] * n[2]],
            vec![None; n[0] * (n[1] - 1) * n[2]],
            vec![None; n[0] * n[1] * (n[2] - 1)],
        ],
        spacing,
    };

    classify_corners(&mut grid, &mut stats);
    find_crossings(&mut grid, &mut stats);
    let (vertices, cells) = place_vertices(&grid, options, &mut stats);
    let triangles = build_triangles(&grid, &cells, &mut stats);

    let mesh = TriMesh::new(vertices, triangles, MeshMeta::synthetic())
        .map_err(|e| ContourError::Mesh(e.to_string()))?;
    Ok((mesh, stats))
}

/// Inside/outside at every corner, by majority of the three bundles.
fn classify_corners(grid: &mut Grid, stats: &mut ContourStats) {
    for k in 0..grid.n[2] {
        for j in 0..grid.n[1] {
            for i in 0..grid.n[0] {
                let p = grid.corner_position(i, j, k);
                let mut votes = 0u32;
                let mut yes = 0u32;
                for bundle_axis in AXES {
                    if let Some(v) = bundle_says_inside(grid.field, bundle_axis, [i, j, k], p) {
                        votes += 1;
                        if v {
                            yes += 1;
                        }
                    }
                }
                stats.corners += 1;
                if votes < 3 {
                    stats.corners_short_of_three_votes += 1;
                } else if yes != 0 && yes != 3 {
                    stats.corner_disagreements += 1;
                }
                // Majority. With three votes that is two; with fewer, a majority
                // of what is available, and a tie resolves to outside -- which
                // keeps the mesh from growing material where the evidence is
                // split.
                let inside = yes * 2 > votes;
                let index = grid.corner_index(i, j, k);
                grid.inside[index] = inside;
            }
        }
    }
}

/// Does `bundle_axis`'s ray through this corner say the corner is material?
///
/// `None` when the corner is outside that bundle's transverse range, which can
/// only happen at the very edge of the workspace.
fn bundle_says_inside(
    field: &TriDexelField,
    bundle_axis: Axis,
    corner: [usize; 3],
    p: Vec3,
) -> Option<bool> {
    let bundle = field.bundle(bundle_axis)?;
    let lattice = bundle.lattice();
    let [u, v, w] = bundle_axis.cyclic();
    // Grid indices count the virtual ring; lattice indices do not.
    let (a, b) = (corner[u].checked_sub(1)?, corner[v].checked_sub(1)?);
    let counts = lattice.counts();
    if a >= counts[0] as usize || b >= counts[1] as usize {
        return None;
    }
    let ray = lattice.index(u32::try_from(a).ok()?, u32::try_from(b).ok()?);
    // Ray parameter of this corner: the ray starts a cell behind the workspace
    // and runs along `+w` at unit speed, so the parameter is the world ordinate
    // measured from that origin. Taken from the bundle's own lattice, never
    // reconstructed, so it is the same arithmetic the spans were built with.
    let origin = lattice.origin_of(u32::try_from(a).ok()?, u32::try_from(b).ok()?);
    let t = p.to_array()[w] - origin.to_array()[w];
    Some(bundle.arena().get(ray).iter().any(|s| s.contains(t)))
}

/// The crossing on every sign-changing edge, from that edge's own bundle.
fn find_crossings(grid: &mut Grid, stats: &mut ContourStats) {
    for axis in AXES {
        let a = axis.index();
        let mut extent = grid.n;
        extent[a] -= 1;
        for k in 0..extent[2] {
            for j in 0..extent[1] {
                for i in 0..extent[0] {
                    let lo = [i, j, k];
                    let mut hi = lo;
                    hi[a] += 1;
                    let inside_lo = grid.inside[grid.corner_index(lo[0], lo[1], lo[2])];
                    let inside_hi = grid.inside[grid.corner_index(hi[0], hi[1], hi[2])];

                    // Counted on EVERY edge, not only on sign-changing ones.
                    //
                    // The first version counted only where the sign changed,
                    // which missed the case the counter exists for: a feature
                    // that fits strictly between two corners leaves both of them
                    // inside -- or both outside -- so there is no sign change at
                    // all, and the edge carries two crossings that the surface
                    // will never see. That is precisely a feature smaller than a
                    // cell, and it was being reported as zero.
                    if count_crossings_on(grid, axis, lo, i, j, k) > 1 {
                        stats.multi_crossing_edges += 1;
                    }

                    if inside_lo == inside_hi {
                        continue;
                    }
                    stats.crossing_edges += 1;

                    let t_lo = grid.coords[a][lo[a]];
                    let t_hi = grid.coords[a][hi[a]];
                    let found = crossing_on(grid, axis, lo, t_lo, t_hi, inside_lo);
                    let index = grid.edge_index(a, i, j, k);
                    match found {
                        Some(c) => grid.crossings[a][index] = Some(c),
                        None => {
                            stats.sign_change_without_crossing += 1;
                            // Fall back to the midpoint with no usable normal.
                            // Dropping the edge instead would open a hole, and a
                            // hole is the one outcome this unit may not produce.
                            grid.crossings[a][index] = Some(Crossing {
                                at: t_lo / 2.0 + t_hi / 2.0,
                                normal: OctNormal::PLACEHOLDER,
                            });
                        }
                    }
                }
            }
        }
    }
}

/// Locates the span endpoint bracketing the sign change on one edge.
fn crossing_on(
    grid: &Grid,
    axis: Axis,
    lo: [usize; 3],
    t_lo: f64,
    t_hi: f64,
    inside_lo: bool,
) -> Option<Crossing> {
    let bundle = grid.field.bundle(axis)?;
    let lattice = bundle.lattice();
    let [u, v, w] = axis.cyclic();
    // See `bundle_says_inside`: the grid carries a virtual ring the lattice does
    // not, so a boundary edge's transverse position may not exist on any ray.
    let (a, b) = (
        u32::try_from(lo[u].checked_sub(1)?).ok()?,
        u32::try_from(lo[v].checked_sub(1)?).ok()?,
    );
    let counts = lattice.counts();
    if a >= counts[0] || b >= counts[1] {
        return None;
    }
    let ray = lattice.index(a, b);
    let origin = lattice.origin_of(a, b).to_array()[w];
    // The edge in the ray's own parameter.
    let (p_lo, p_hi) = (t_lo - origin, t_hi - origin);

    // Every endpoint strictly inside the edge, in ascending order. A span
    // contributes its lower bound as a material-begins crossing and its upper
    // bound as a material-ends one.
    let mut found: Option<Crossing> = None;
    for span in bundle.arena().get(ray) {
        for (t, normal, begins) in [(span.t0, span.n0, true), (span.t1, span.n1, false)] {
            if t <= p_lo || t >= p_hi {
                continue;
            }
            // The crossing that matches the edge's own sign change: if the low
            // corner is inside, the surface must be leaving material here. Where
            // an edge carries several, this takes the first that brackets the
            // change and drops the rest -- which is the resolution limit, and is
            // counted as `multi_crossing_edges`.
            if begins == inside_lo {
                continue;
            }
            if found.is_none() {
                found = Some(Crossing {
                    at: t + origin,
                    normal,
                });
            }
        }
    }
    found
}

/// How many span endpoints lie strictly inside one edge.
///
/// Independent of the corner signs, which is the point: a feature between two
/// same-signed corners is invisible to the surface and still needs counting.
fn count_crossings_on(
    grid: &Grid,
    axis: Axis,
    lo: [usize; 3],
    i: usize,
    j: usize,
    k: usize,
) -> u32 {
    let _ = (i, j, k);
    let Some(bundle) = grid.field.bundle(axis) else {
        return 0;
    };
    let lattice = bundle.lattice();
    let [u, v, w] = axis.cyclic();
    let (Some(a), Some(b)) = (lo[u].checked_sub(1), lo[v].checked_sub(1)) else {
        return 0;
    };
    let counts = lattice.counts();
    let (Ok(a), Ok(b)) = (u32::try_from(a), u32::try_from(b)) else {
        return 0;
    };
    if a >= counts[0] || b >= counts[1] {
        return 0;
    }
    let ray = lattice.index(a, b);
    let origin = lattice.origin_of(a, b).to_array()[w];
    let ax = axis.index();
    let p_lo = grid.coords[ax][lo[ax]] - origin;
    let p_hi = grid.coords[ax][lo[ax] + 1] - origin;
    let mut seen = 0u32;
    for span in bundle.arena().get(ray) {
        for t in [span.t0, span.t1] {
            if t > p_lo && t < p_hi {
                seen += 1;
            }
        }
    }
    seen
}

/// One vertex per connected component of inside corners, per cell.
fn place_vertices(
    grid: &Grid,
    options: &ContourOptions,
    stats: &mut ContourStats,
) -> (Vec<Vec3>, Vec<CellVertices>) {
    let cells = [grid.n[0] - 1, grid.n[1] - 1, grid.n[2] - 1];
    let mut out = vec![CellVertices::default(); cells[0] * cells[1] * cells[2]];
    let mut vertices: Vec<Vec3> = Vec::new();

    // Ascending cell index, which fixes the vertex numbering and so the mesh.
    for ck in 0..cells[2] {
        for cj in 0..cells[1] {
            for ci in 0..cells[0] {
                let mut corner_inside = [false; 8];
                for (c, slot) in corner_inside.iter_mut().enumerate() {
                    let (dx, dy, dz) = corner_offset(c);
                    *slot = grid.inside[grid.corner_index(ci + dx, cj + dy, ck + dz)];
                }
                if !corner_inside.iter().any(|x| *x) {
                    continue;
                }

                // Connected components of the inside corners, via the cell's own
                // twelve edges. This is the whole of the manifold fix: a cell
                // whose inside corners fall into two groups gets two vertices,
                // and the two sheets never share one.
                let mut component_of = [u8::MAX; 8];
                let mut count = 0u8;
                for seed in 0..8 {
                    if !corner_inside[seed] || component_of[seed] != u8::MAX {
                        continue;
                    }
                    let id = count;
                    count += 1;
                    let mut stack = vec![seed];
                    component_of[seed] = id;
                    while let Some(c) = stack.pop() {
                        for (a, b, _) in CELL_EDGES {
                            let other = if a == c {
                                b
                            } else if b == c {
                                a
                            } else {
                                continue;
                            };
                            if corner_inside[other] && component_of[other] == u8::MAX {
                                component_of[other] = id;
                                stack.push(other);
                            }
                        }
                    }
                }
                assert!(count as usize <= 4, "a cube cannot have five components");

                // A QEF per component, fed by the crossings on the edges
                // incident to that component's corners. Edges are visited in
                // `CELL_EDGES` order so the accumulation order is fixed.
                let mut systems: [Qef; 4] = [Qef::new(), Qef::new(), Qef::new(), Qef::new()];
                for (a, b, axis) in CELL_EDGES {
                    if corner_inside[a] == corner_inside[b] {
                        continue;
                    }
                    let inside_corner = if corner_inside[a] { a } else { b };
                    let id = component_of[inside_corner];
                    debug_assert!(id != u8::MAX, "a sign-changing edge with no inside corner");
                    let low = a.min(b);
                    let (dx, dy, dz) = corner_offset(low);
                    let index = grid.edge_index(axis, ci + dx, cj + dy, ck + dz);
                    let Some(crossing) = grid.crossings[axis][index] else {
                        continue;
                    };
                    let (dx0, dy0, dz0) = corner_offset(low);
                    let mut p = grid
                        .corner_position(ci + dx0, cj + dy0, ck + dz0)
                        .to_array();
                    p[axis] = crossing.at;
                    let point = Vec3::from_array(p);
                    let normal = if options.use_normals {
                        crossing.normal.decode()
                    } else {
                        // Surface-nets control: no plane, only a point, so every
                        // direction is unconstrained and the solve returns the
                        // centroid.
                        Vec3::new(0.0, 0.0, 0.0)
                    };
                    systems[id as usize].add(point, normal);
                }

                let mut record = CellVertices {
                    component_of,
                    vertex: [u32::MAX; 4],
                };
                let mut emitted = 0u8;
                for id in 0..count {
                    let system = &systems[id as usize];
                    if system.count() == 0 {
                        // An inside component with no sign-changing edge is
                        // wholly interior; it has no surface and needs no
                        // vertex.
                        continue;
                    }
                    let (raw, rank) = system.solve();
                    let (v, clamped) =
                        clamp_into_cell(grid, [ci, cj, ck], raw, options.clamp_expand);
                    if clamped {
                        stats.clamped_vertices += 1;
                    }
                    stats.rank_histogram[rank.min(3) as usize] += 1;
                    let id_out = u32::try_from(vertices.len()).unwrap_or(u32::MAX);
                    vertices.push(v);
                    record.vertex[id as usize] = id_out;
                    emitted += 1;
                }
                if emitted > 0 {
                    stats.cells_with_vertices += 1;
                    if emitted > 1 {
                        stats.cells_with_multiple_vertices += 1;
                    }
                }
                let index = ci + cells[0] * (cj + cells[1] * ck);
                out[index] = record;
            }
        }
    }
    (vertices, out)
}

/// Keeps a vertex within its own cell, expanded by `slack` cells.
fn clamp_into_cell(grid: &Grid, cell: [usize; 3], v: Vec3, slack: f64) -> (Vec3, bool) {
    let pad = grid.spacing * slack;
    let mut out = v.to_array();
    let mut clamped = false;
    for a in 0..3 {
        let lo = grid.coords[a][cell[a]] - pad;
        let hi = grid.coords[a][cell[a] + 1] + pad;
        if !out[a].is_finite() {
            out[a] = grid.coords[a][cell[a]] / 2.0 + grid.coords[a][cell[a] + 1] / 2.0;
            clamped = true;
        } else if out[a] < lo {
            out[a] = lo;
            clamped = true;
        } else if out[a] > hi {
            out[a] = hi;
            clamped = true;
        }
    }
    (Vec3::from_array(out), clamped)
}

/// A quad per sign-changing interior edge, split on a fixed diagonal.
fn build_triangles(
    grid: &Grid,
    cells: &[CellVertices],
    _stats: &mut ContourStats,
) -> Vec<[u32; 3]> {
    let dims = [grid.n[0] - 1, grid.n[1] - 1, grid.n[2] - 1];
    let mut triangles: Vec<[u32; 3]> = Vec::new();

    for axis in AXES {
        let a = axis.index();
        // The two axes the four surrounding cells are offset along.
        let (p, q) = match a {
            0 => (1usize, 2usize),
            1 => (2, 0),
            _ => (0, 1),
        };
        let mut extent = grid.n;
        extent[a] -= 1;
        for k in 0..extent[2] {
            for j in 0..extent[1] {
                for i in 0..extent[0] {
                    let lo = [i, j, k];
                    let mut hi = lo;
                    hi[a] += 1;
                    let inside_lo = grid.inside[grid.corner_index(lo[0], lo[1], lo[2])];
                    let inside_hi = grid.inside[grid.corner_index(hi[0], hi[1], hi[2])];
                    if inside_lo == inside_hi {
                        continue;
                    }
                    // The four cells around this edge need the edge to be
                    // interior along both perpendicular axes.
                    if lo[p] == 0 || lo[q] == 0 || lo[p] >= dims[p] || lo[q] >= dims[q] {
                        continue;
                    }

                    // The four cells, in a consistent rotational order about the
                    // edge so the quad is not self-crossing.
                    let mut quad = [u32::MAX; 4];
                    let mut ok = true;
                    for (slot, (dp, dq)) in [(0usize, 0usize), (1, 0), (1, 1), (0, 1)]
                        .into_iter()
                        .enumerate()
                    {
                        let mut cell = lo;
                        cell[p] -= 1 - dp;
                        cell[q] -= 1 - dq;
                        // Which corner of that cell is this edge's inside end.
                        let mut corner_bits = 0usize;
                        let inside_corner = if inside_lo { lo } else { hi };
                        for axis_bit in 0..3 {
                            if inside_corner[axis_bit] - cell[axis_bit] == 1 {
                                corner_bits |= 1 << axis_bit;
                            }
                        }
                        let record = cells[cell[0] + dims[0] * (cell[1] + dims[1] * cell[2])];
                        let component = record.component_of[corner_bits];
                        if component == u8::MAX {
                            ok = false;
                            break;
                        }
                        let vertex = record.vertex[component as usize];
                        if vertex == u32::MAX {
                            ok = false;
                            break;
                        }
                        quad[slot] = vertex;
                    }
                    if !ok {
                        continue;
                    }

                    // Orientation follows the sign change: the surface normal
                    // points from material to air, so a quad whose edge runs
                    // inside-to-outside winds one way and the reverse the other.
                    let (t0, t1) = if inside_lo {
                        ([quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]])
                    } else {
                        ([quad[0], quad[2], quad[1]], [quad[0], quad[3], quad[2]])
                    };
                    // A fixed diagonal, from the cell indices alone. A geometric
                    // choice -- shorter diagonal, better aspect ratio -- would be
                    // data-dependent and so not reproducible.
                    if t0[0] != t0[1] && t0[1] != t0[2] && t0[0] != t0[2] {
                        triangles.push(t0);
                    }
                    if t1[0] != t1[1] && t1[1] != t1[2] && t1[0] != t1[2] {
                        triangles.push(t1);
                    }
                }
            }
        }
    }
    triangles
}
