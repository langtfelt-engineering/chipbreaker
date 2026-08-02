// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Vertex welding by lattice quantisation.
//!
//! STL is a triangle soup: every facet carries its own three vertices, and
//! nothing says that the corner of one triangle is the same point as the corner
//! of its neighbour. Topology — manifoldness, orientation consistency, genus,
//! and the closed-surface parity that U5 depends on — is meaningless until
//! coincident vertices have been identified. Welding is what does that.
//!
//! It is also the single most determinism-hostile operation in this unit, so the
//! method matters more than the result.
//!
//! # Why not tolerance matching
//!
//! The obvious approach — "merge `a` and `b` when `|a - b| < eps`" — is
//! **not transitive**. Take three points spaced `0.6 * eps` apart on a line:
//! `a` matches `b`, `b` matches `c`, `a` does not match `c`. Whether the result
//! is one cluster, two, or three depends entirely on the order the pairs are
//! visited in.
//!
//! That is not a subtle bias to be measured and bounded. It means the output is
//! a function of the input *ordering*, so the same part exported by two CAD
//! systems in different vertex orders welds differently, hashes differently, and
//! produces a different simulation. It is exactly the failure the determinism
//! invariant exists to prevent.
//!
//! # Lattice quantisation instead
//!
//! Snap each coordinate to a lattice of [`crate::eps::EPS_WELD`], giving an
//! integer triple. Two vertices weld **iff their integer triples are equal**.
//!
//! That relation is an equivalence relation by construction — it is equality on
//! a derived key — so it is reflexive, symmetric and transitive, and completely
//! independent of visit order. There is no pairwise comparison anywhere, and
//! therefore nothing to order.
//!
//! The welded vertex is emitted as **the lattice point itself**, not the
//! centroid of the cluster. A centroid would reintroduce order dependence
//! through floating-point accumulation: summing the same points in a different
//! sequence gives a different last bit. The lattice point is a pure function of
//! the key.
//!
//! Clusters are visited through a [`BTreeMap`](std::collections::BTreeMap) keyed
//! by the integer triple, never a hash map, so the output vertex numbering is
//! ascending in lattice order. That has a useful consequence beyond determinism:
//! the same geometry presented in two different vertex orders welds to a
//! **byte-identical** mesh.
//!
//! # The cost, stated honestly
//!
//! Lattice snapping can fail to weld two vertices that are much closer together
//! than `EPS_WELD` but happen to straddle a lattice boundary. Two points either
//! side of a cell wall, a nanometre apart, get different keys and stay separate.
//!
//! This is a real limitation and it is the price of transitivity — every
//! order-independent scheme has some such boundary. In practice it is rare,
//! because coincident vertices in a CAD export are usually *bit-identical* or
//! differ only in the last few `f32` places, and both cases land in the same cell
//! unless the shared value sits exactly on a boundary.
//!
//! The standard mitigation, if it ever bites on real data, is a second pass on a
//! half-offset lattice: a point pair can straddle a boundary on one lattice or
//! the other, but not both. **That is deliberately not implemented here** — it
//! doubles the cost and complicates the invariant for a problem we have not yet
//! observed. The failure is visible (it shows up as boundary edges in
//! [`crate::mesh::validate`]), so it will be noticed rather than silently
//! absorbed.

use std::collections::BTreeMap;

use core::fmt;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec3;
use crate::mesh::{MeshError, TriMesh};

/// The integer lattice coordinate a vertex snaps to.
type Key = [i64; 3];

/// Why welding could not proceed.
#[derive(Debug, Clone, PartialEq)]
pub enum WeldError {
    /// The lattice spacing was not a positive finite number.
    InvalidLattice {
        /// The rejected value.
        lattice: f64,
    },
    /// A coordinate divided by the lattice spacing exceeds the `i64` key space.
    ///
    /// At the default 1e-6 mm lattice this needs a coordinate beyond 9.2e12 mm,
    /// which is nine billion kilometres. Reachable only from a corrupted
    /// transform, but it must be an error rather than a silent wrap: a wrapped
    /// key would weld two unrelated vertices and quietly change the topology.
    CoordinateTooLarge {
        /// Index of the offending vertex.
        vertex: u32,
        /// Which component: 0 = x, 1 = y, 2 = z.
        axis: usize,
        /// The value.
        value: f64,
        /// The lattice spacing in force.
        lattice: f64,
    },
    /// Rebuilding the mesh from welded vertices failed.
    Rebuild(MeshError),
}

impl fmt::Display for WeldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLattice { lattice } => write!(
                f,
                "weld lattice must be a positive finite length, got {lattice}"
            ),
            Self::CoordinateTooLarge {
                vertex,
                axis,
                value,
                lattice,
            } => write!(
                f,
                "vertex {vertex} component {} is {value:e}, which does not fit \
                 the integer lattice at spacing {lattice:e}; a wrapped key would \
                 weld unrelated vertices and silently change the topology",
                ["x", "y", "z"].get(*axis).copied().unwrap_or("?")
            ),
            Self::Rebuild(e) => write!(f, "welded mesh failed validation: {e}"),
        }
    }
}

impl core::error::Error for WeldError {}

/// What welding did.
#[derive(Debug, Clone, PartialEq)]
pub struct WeldReport {
    /// Vertices before welding.
    pub vertices_before: u32,
    /// Vertices after welding.
    pub vertices_after: u32,
    /// Triangles that became degenerate *because* of welding — that is, two of
    /// their corners landed in the same lattice cell.
    ///
    /// Reported rather than removed. A triangle collapsing under welding is
    /// information: it usually means the lattice is too coarse for the model, or
    /// that the exporter emitted slivers. Deleting it silently would destroy the
    /// evidence and change the triangle numbering that every other report refers
    /// to.
    pub triangles_collapsed: u32,
    /// The lattice spacing used.
    pub lattice: f64,
}

impl Hashable for WeldReport {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("WeldReport");
        h.u64(u64::from(self.vertices_before));
        h.u64(u64::from(self.vertices_after));
        h.u64(u64::from(self.triangles_collapsed));
        h.f64(self.lattice);
        h.end();
    }
}

/// Snaps one coordinate to the lattice.
///
/// `round` is used rather than `floor` so the cell is centred on the lattice
/// point, halving the worst-case displacement. It is exactly specified by
/// IEEE-754 (round half away from zero) and therefore identical on every target.
#[inline]
fn quantise(value: f64, lattice: f64) -> Option<i64> {
    let scaled = (value / lattice).round();
    // The i64 range check has to happen in f64, before the cast: an out-of-range
    // float-to-int cast saturates in Rust rather than wrapping, which would
    // silently weld every far-away vertex to the same key.
    if scaled.abs() >= 9.0e18 || !scaled.is_finite() {
        return None;
    }
    Some(scaled as i64)
}

/// The lattice point a key denotes.
#[inline]
fn dequantise(key: Key, lattice: f64) -> Vec3 {
    Vec3::new(
        key[0] as f64 * lattice,
        key[1] as f64 * lattice,
        key[2] as f64 * lattice,
    )
}

/// Welds coincident vertices at the given lattice spacing.
///
/// Pass [`crate::eps::EPS_WELD`] unless the caller has a reason not to;
/// `--weld-tol` overrides it.
///
/// # Errors
/// [`WeldError::InvalidLattice`] for a non-positive or non-finite spacing,
/// [`WeldError::CoordinateTooLarge`] if a coordinate does not fit the integer
/// key space, and [`WeldError::Rebuild`] if the welded mesh somehow fails
/// construction validation.
pub fn weld(mesh: &TriMesh, lattice: f64) -> Result<(TriMesh, WeldReport), WeldError> {
    // Spelled out rather than `!(lattice > 0.0)`: NaN must be rejected, and a
    // negated comparison on a partially ordered type hides that intent.
    if lattice.is_nan() || lattice <= 0.0 || !lattice.is_finite() {
        return Err(WeldError::InvalidLattice { lattice });
    }

    // Pass 1: key every vertex.
    let mut keys: Vec<Key> = Vec::with_capacity(mesh.vertices().len());
    for (i, v) in mesh.vertices().iter().enumerate() {
        let mut key = [0i64; 3];
        for (axis, component) in v.to_array().into_iter().enumerate() {
            key[axis] = quantise(component, lattice).ok_or(WeldError::CoordinateTooLarge {
                vertex: i as u32,
                axis,
                value: component,
                lattice,
            })?;
        }
        keys.push(key);
    }

    // Pass 2: assign new indices in ascending lattice order. A BTreeMap, not a
    // HashMap: the iteration order becomes the output vertex numbering, so it
    // has to be a property of the geometry rather than of a hash seed.
    let mut cluster_index: BTreeMap<Key, u32> = BTreeMap::new();
    for key in &keys {
        cluster_index.entry(*key).or_insert(0);
    }
    let mut vertices = Vec::with_capacity(cluster_index.len());
    for (next, (key, slot)) in cluster_index.iter_mut().enumerate() {
        *slot = next as u32;
        vertices.push(dequantise(*key, lattice));
    }

    // Pass 3: remap.
    let remap: Vec<u32> = keys
        .iter()
        .map(|k| *cluster_index.get(k).unwrap_or(&0))
        .collect();

    let mut triangles = Vec::with_capacity(mesh.triangles().len());
    let mut triangles_collapsed = 0u32;
    for t in mesh.triangles() {
        let mapped = [
            remap[t[0] as usize],
            remap[t[1] as usize],
            remap[t[2] as usize],
        ];
        let already_degenerate = t[0] == t[1] || t[1] == t[2] || t[2] == t[0];
        let now_degenerate =
            mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0];
        if now_degenerate && !already_degenerate {
            triangles_collapsed += 1;
        }
        triangles.push(mapped);
    }

    let report = WeldReport {
        vertices_before: mesh.vertex_count(),
        vertices_after: vertices.len() as u32,
        triangles_collapsed,
        lattice,
    };

    let welded =
        TriMesh::new(vertices, triangles, mesh.meta().clone()).map_err(WeldError::Rebuild)?;
    Ok((welded, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eps::EPS_WELD;
    use crate::golden::Hashable;
    use crate::mesh::MeshMeta;

    fn mesh_of(vertices: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> TriMesh {
        TriMesh::new(vertices, triangles, MeshMeta::synthetic()).expect("valid")
    }

    /// Two triangles sharing an edge, expressed as a six-vertex soup.
    fn soup() -> TriMesh {
        mesh_of(
            vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                // Same edge again, bit-identical.
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        )
    }

    #[test]
    fn identical_vertices_weld() {
        let (welded, report) = weld(&soup(), EPS_WELD).expect("welds");
        assert_eq!(report.vertices_before, 6);
        assert_eq!(report.vertices_after, 4);
        assert_eq!(report.triangles_collapsed, 0);
        assert_eq!(welded.triangle_count(), 2);
        // The shared edge is now genuinely shared.
        let t0 = welded.triangles()[0];
        let t1 = welded.triangles()[1];
        let shared: Vec<u32> = t0.iter().filter(|v| t1.contains(v)).copied().collect();
        assert_eq!(shared.len(), 2, "the two triangles must share an edge");
    }

    #[test]
    fn vertices_within_the_lattice_cell_weld() {
        // A tenth of a cell apart: same key.
        let nudge = EPS_WELD * 0.1;
        let m = mesh_of(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(nudge, nudge, nudge)],
            vec![[0, 1, 2], [3, 1, 2]],
        );
        let (welded, report) = weld(&m, EPS_WELD).expect("welds");
        assert_eq!(report.vertices_after, 3);
        assert_eq!(welded.vertex_count(), 3);
    }

    #[test]
    fn welding_is_independent_of_input_vertex_order() {
        // The property the whole lattice approach exists to guarantee.
        let forward = soup();
        let reversed = {
            let mut v = forward.vertices().to_vec();
            v.reverse();
            let n = forward.vertex_count() - 1;
            let t: Vec<[u32; 3]> = forward
                .triangles()
                .iter()
                .map(|t| [n - t[0], n - t[1], n - t[2]])
                .collect();
            mesh_of(v, t)
        };

        let (a, _) = weld(&forward, EPS_WELD).expect("welds");
        let (b, _) = weld(&reversed, EPS_WELD).expect("welds");
        // Vertex *positions* must be identical, in identical order.
        assert_eq!(
            a.vertices(),
            b.vertices(),
            "welded vertex order must be canonical"
        );
    }

    #[test]
    fn output_vertices_are_in_ascending_lattice_order() {
        let m = mesh_of(
            vec![
                Vec3::new(5.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2]],
        );
        let (welded, _) = weld(&m, EPS_WELD).expect("welds");
        let v = welded.vertices();
        // Sorted by (x, y, z) as integer keys.
        assert_eq!(v[0], Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(v[1], Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(v[2], Vec3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn welding_is_idempotent() {
        let (once, _) = weld(&soup(), EPS_WELD).expect("welds");
        let (twice, report) = weld(&once, EPS_WELD).expect("welds");
        assert_eq!(once.canonical_digest(), twice.canonical_digest());
        assert_eq!(report.vertices_before, report.vertices_after);
    }

    #[test]
    fn the_lattice_boundary_limitation_is_real_and_characterised() {
        // Documents the accepted cost. Two points a thousandth of a cell apart,
        // straddling a cell wall, do NOT weld. This is the price of transitivity
        // and it is pinned here so a future change to the quantisation is a
        // visible behavioural change rather than a silent one.
        let boundary = EPS_WELD * 0.5;
        let below = boundary - EPS_WELD * 0.001;
        let above = boundary + EPS_WELD * 0.001;
        let m = mesh_of(
            vec![
                Vec3::new(below, 0.0, 0.0),
                Vec3::new(above, 0.0, 0.0),
                Vec3::Y,
            ],
            vec![[0, 1, 2]],
        );
        let (_, report) = weld(&m, EPS_WELD).expect("welds");
        assert_eq!(
            report.vertices_after, 3,
            "straddling the cell wall, these do not weld; see the module docs"
        );

        // Whereas the same separation not straddling a wall does weld.
        let m2 = mesh_of(
            vec![
                Vec3::new(EPS_WELD * 0.1, 0.0, 0.0),
                Vec3::new(EPS_WELD * 0.101, 0.0, 0.0),
                Vec3::Y,
            ],
            vec![[0, 1, 2]],
        );
        let (_, report2) = weld(&m2, EPS_WELD).expect("welds");
        assert_eq!(report2.vertices_after, 2);
    }

    #[test]
    fn welded_vertices_land_on_the_lattice_not_the_centroid() {
        // A centroid would depend on accumulation order; the lattice point does
        // not. Three points in one cell must yield the cell's lattice point.
        let m = mesh_of(
            vec![
                Vec3::new(EPS_WELD * 0.1, 0.0, 0.0),
                Vec3::new(EPS_WELD * 0.2, 0.0, 0.0),
                Vec3::new(EPS_WELD * 0.3, 0.0, 0.0),
                Vec3::Y,
                Vec3::Z,
            ],
            vec![[0, 3, 4], [1, 3, 4], [2, 3, 4]],
        );
        let (welded, _) = weld(&m, EPS_WELD).expect("welds");
        assert!(
            welded.vertices().contains(&Vec3::ZERO),
            "the cluster must collapse to the lattice point (0,0,0), got {:?}",
            welded.vertices()
        );
    }

    #[test]
    fn a_coarse_lattice_collapses_triangles_and_says_so() {
        // Welding can create degenerate triangles. They are reported, not
        // removed: the count is information, and deleting them would renumber
        // the triangles that every other report refers to.
        let m = mesh_of(
            vec![Vec3::ZERO, Vec3::new(0.5, 0.0, 0.0), Vec3::Y],
            vec![[0, 1, 2]],
        );
        let (welded, report) = weld(&m, 10.0).expect("welds");
        assert_eq!(report.triangles_collapsed, 1);
        assert_eq!(
            welded.triangle_count(),
            1,
            "the collapsed triangle is kept so numbering is stable"
        );
        assert_eq!(welded.face_normal(0), None);
    }

    #[test]
    fn an_already_degenerate_triangle_is_not_counted_as_collapsed() {
        let m = mesh_of(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 1]]);
        let (_, report) = weld(&m, EPS_WELD).expect("welds");
        assert_eq!(report.triangles_collapsed, 0, "it was already degenerate");
    }

    #[test]
    fn invalid_lattice_is_rejected() {
        let m = soup();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let err = weld(&m, bad).expect_err("must reject");
            assert!(matches!(err, WeldError::InvalidLattice { .. }), "{bad}");
        }
    }

    #[test]
    fn coordinates_that_do_not_fit_the_key_space_are_rejected() {
        // Well inside ORIENT3D_COORDS, so TriMesh accepts it, but far outside the
        // i64 lattice at a 1e-6 spacing.
        let m = mesh_of(
            vec![Vec3::new(1e30, 0.0, 0.0), Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        );
        let err = weld(&m, EPS_WELD).expect_err("must reject");
        match err {
            WeldError::CoordinateTooLarge { vertex, axis, .. } => {
                assert_eq!((vertex, axis), (0, 0));
            }
            other => panic!("wrong error: {other}"),
        }
        // A coarser lattice brings it back into range.
        assert!(weld(&m, 1e12).is_ok());
    }

    #[test]
    fn quantisation_handles_signed_zero_and_negatives() {
        assert_eq!(quantise(0.0, EPS_WELD), Some(0));
        assert_eq!(quantise(-0.0, EPS_WELD), Some(0));
        assert_eq!(quantise(EPS_WELD, EPS_WELD), Some(1));
        assert_eq!(quantise(-EPS_WELD, EPS_WELD), Some(-1));
        assert_eq!(quantise(f64::NAN, EPS_WELD), None);
        assert_eq!(quantise(f64::INFINITY, EPS_WELD), None);
        // -0.0 and 0.0 must land in the same cell, or a mesh straddling the
        // origin would fail to weld along the axis planes.
        assert_eq!(dequantise([0, 0, 0], EPS_WELD), Vec3::ZERO);
    }

    #[test]
    fn a_cube_soup_welds_to_eight_vertices() {
        // The realistic STL case: twelve triangles, thirty-six loose vertices.
        let cube = crate::mesh::tests::unit_cube();
        let mut soup_vertices = Vec::new();
        let mut soup_triangles = Vec::new();
        for i in 0..cube.triangle_count() {
            let [a, b, c] = cube.triangle(i);
            let base = soup_vertices.len() as u32;
            soup_vertices.extend_from_slice(&[a, b, c]);
            soup_triangles.push([base, base + 1, base + 2]);
        }
        let soup = mesh_of(soup_vertices, soup_triangles);
        assert_eq!(soup.vertex_count(), 36);

        let (welded, report) = weld(&soup, EPS_WELD).expect("welds");
        assert_eq!(report.vertices_after, 8, "a cube has eight corners");
        assert_eq!(welded.triangle_count(), 12);
        assert_eq!(report.triangles_collapsed, 0);
        // Volume survives the round trip.
        assert!((welded.signed_volume() - 1.0).abs() < 1e-12);
    }
}
