// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Indexed triangle meshes: the geometry input layer.
//!
//! # What this layer is for
//!
//! Everything downstream assumes it is given a **closed, consistently oriented,
//! finite** triangle mesh in millimetres. This module is where that assumption
//! is either established or refused. Nothing is repaired silently — a mesh that
//! cannot be trusted is reported, in detail, rather than patched into something
//! that looks fine and behaves strangely three units later.
//!
//! # Why it matters more than it looks
//!
//! A dexel field is built by casting millions of parallel rays through a
//! closed mesh and recording where each ray is inside material. That rests
//! entirely on a parity argument: a ray crossing a closed surface must produce an
//! **even** number of crossings, alternating enter/exit. If a ray passes exactly
//! through an edge shared by two triangles and the intersection test reports two
//! hits or none instead of one, the parity flips and material leaks along that
//! whole ray — a spike or a tunnel through the simulated stock, appearing
//! intermittently on customer data.
//!
//! [`bvh`] is the module that prevents that, and it is the one to read carefully.
//!
//! # Layout
//!
//! - [`units`] — the millimetre convention and the conversion boundary.
//! - [`weld`] — lattice-quantised vertex welding, the order-independent kind.
//! - [`validate`] — manifoldness, orientation, genus, degeneracy.
//! - [`bvh`] — bounding volume hierarchy and leak-free ray casting.
//! - [`io`] — STL (binary and ASCII), OBJ, 3MF.

pub mod bvh;
pub mod intersect;
pub mod io;
pub mod shapes;
pub mod units;
pub mod validate;
pub mod weld;

use core::fmt;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Vec3};
use crate::predicates::ORIENT3D_COORDS;
use units::Unit;

/// Where a mesh came from and what was done to it on the way in.
///
/// Recorded so that `mesh inspect` can answer "what units did you read this as?"
/// — the single most useful question when a part turns out to be 25.4x the size
/// the customer expected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshMeta {
    /// Format the mesh was read from, e.g. `"stl-binary"`. Empty for meshes
    /// constructed programmatically.
    pub source_format: String,
    /// Unit the source file was interpreted as.
    pub source_unit: Unit,
    /// Non-triangular faces encountered and fan-triangulated (OBJ only).
    pub polygons_triangulated: u32,
    /// Of those, how many were **not convex**, so the fan from their first
    /// vertex may have produced triangles outside the face (OBJ only).
    ///
    /// Surfaced by [`validate`] as a
    /// [`validate::FindingKind::NonConvexPolygonFan`] rather than left as a
    /// number in a report, because a count nobody reads is not a warning.
    pub non_convex_polygons: u32,
    /// Records the loader ignored: `vt`, `vn`, `usemtl`, groups, and the like.
    pub ignored_records: u32,
}

impl MeshMeta {
    /// Metadata for a mesh built in memory, already in millimetres.
    #[must_use]
    pub fn synthetic() -> Self {
        Self {
            source_format: String::new(),
            source_unit: Unit::Millimetre,
            polygons_triangulated: 0,
            non_convex_polygons: 0,
            ignored_records: 0,
        }
    }

    /// The factor applied to source coordinates to reach millimetres.
    #[must_use]
    pub fn scale_applied(&self) -> f64 {
        self.source_unit.millimetres_per()
    }
}

impl Hashable for MeshMeta {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("MeshMeta");
        h.str(&self.source_format);
        h.str(self.source_unit.name());
        h.u64(u64::from(self.polygons_triangulated));
        h.u64(u64::from(self.non_convex_polygons));
        h.u64(u64::from(self.ignored_records));
        h.end();
    }
}

/// Why a mesh could not be constructed.
///
/// Every variant names the offending element, because "invalid mesh" is not a
/// diagnosis. The caller is usually looking at a 40 MB file they did not author.
#[derive(Debug, Clone, PartialEq)]
pub enum MeshError {
    /// A coordinate was NaN or infinite.
    ///
    /// Rejected at the boundary rather than carried: `Orientation::from_determinant`
    /// panics on NaN in release builds *by design*, so a NaN that reaches the
    /// predicates aborts the process. Stopping it here turns that into a
    /// diagnosable error.
    NonFiniteCoordinate {
        /// Index of the offending vertex.
        vertex: u32,
        /// Which component: 0 = x, 1 = y, 2 = z.
        axis: usize,
        /// The value, rendered.
        value: String,
        /// Source location, if the loader knows one.
        location: Option<String>,
    },
    /// A coordinate lies outside the range in which the exact predicates are
    /// exact.
    ///
    /// [`crate::predicates::ORIENT3D_COORDS`] is what the ray caster runs, and
    /// outside its measured band the predicate over- or underflows and its sign
    /// is meaningless. A mesh like this is almost always the result of a
    /// corrupted transform upstream.
    CoordinateOutOfRange {
        /// Index of the offending vertex.
        vertex: u32,
        /// Which component: 0 = x, 1 = y, 2 = z.
        axis: usize,
        /// The value.
        value: f64,
        /// Smallest admissible non-zero magnitude.
        min: f64,
        /// Largest admissible magnitude.
        max: f64,
    },
    /// A triangle referenced a vertex that does not exist.
    IndexOutOfRange {
        /// Index of the offending triangle.
        triangle: u32,
        /// The out-of-range vertex index.
        index: u32,
        /// How many vertices the mesh has.
        vertex_count: u32,
    },
    /// More vertices or triangles than a `u32` index can address.
    TooLarge {
        /// What overflowed.
        what: &'static str,
        /// How many there were.
        count: usize,
    },
}

impl fmt::Display for MeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let axis_name = |a: usize| ["x", "y", "z"].get(a).copied().unwrap_or("?");
        match self {
            Self::NonFiniteCoordinate {
                vertex,
                axis,
                value,
                location,
            } => {
                write!(
                    f,
                    "vertex {vertex} has a non-finite {} coordinate ({value})",
                    axis_name(*axis)
                )?;
                if let Some(loc) = location {
                    write!(f, " at {loc}")?;
                }
                write!(
                    f,
                    "; a NaN or infinity here would reach the exact predicates, \
                     which reject it by panicking, so it is stopped at the boundary"
                )
            }
            Self::CoordinateOutOfRange {
                vertex,
                axis,
                value,
                min,
                max,
            } => write!(
                f,
                "vertex {vertex} has {} = {value:e}, outside the range \
                 [{min:e}, {max:e}] in which orient3d is exact; the ray caster \
                 would return a meaningless sign for it. This usually means a \
                 corrupted transform upstream rather than a real part.",
                axis_name(*axis)
            ),
            Self::IndexOutOfRange {
                triangle,
                index,
                vertex_count,
            } => write!(
                f,
                "triangle {triangle} references vertex {index}, but the mesh has \
                 only {vertex_count} vertices"
            ),
            Self::TooLarge { what, count } => write!(
                f,
                "{count} {what} exceeds the u32 index space; indices are u32 \
                 rather than usize so that a mesh hashes identically on 32-bit \
                 WASM and 64-bit native"
            ),
        }
    }
}

impl core::error::Error for MeshError {}

/// An indexed triangle mesh in millimetres.
///
/// # Why indices are `u32`
///
/// Not `usize`. This is the same class of bug that broke native/WASM parity in
/// `usize` is 64 bits natively and 32 bits on `wasm32`, so any index that
/// reaches a hash differs between targets. `u32` is the same everywhere, caps out
/// at four billion triangles — a ceiling no machining part will approach — and
/// halves the memory besides.
#[derive(Debug, Clone, PartialEq)]
pub struct TriMesh {
    vertices: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
    meta: MeshMeta,
}

impl TriMesh {
    /// Builds a mesh, validating coordinates and indices.
    ///
    /// Rejects, and does not repair:
    ///
    /// - non-finite coordinates,
    /// - coordinates outside [`ORIENT3D_COORDS`],
    /// - vertex indices past the end of the vertex list,
    /// - more elements than a `u32` can index.
    ///
    /// **Degenerate triangles are deliberately accepted.** Zero-area and
    /// repeated-index triangles are extremely common in real CAD exports, and
    /// they are a *validation finding* — something [`validate`] reports and the
    /// user decides about — not a parse error. Refusing to load a file because
    /// one triangle in eighty thousand is degenerate would make the tool useless
    /// on exactly the data it exists to inspect.
    ///
    /// # Errors
    /// Returns the first problem found, scanning vertices before triangles.
    pub fn new(
        vertices: Vec<Vec3>,
        triangles: Vec<[u32; 3]>,
        meta: MeshMeta,
    ) -> Result<Self, MeshError> {
        u32::try_from(vertices.len()).map_err(|_| MeshError::TooLarge {
            what: "vertices",
            count: vertices.len(),
        })?;
        u32::try_from(triangles.len()).map_err(|_| MeshError::TooLarge {
            what: "triangles",
            count: triangles.len(),
        })?;

        let vertex_count = vertices.len() as u32;

        for (i, v) in vertices.iter().enumerate() {
            let idx = i as u32;
            for (axis, value) in v.to_array().into_iter().enumerate() {
                if !value.is_finite() {
                    return Err(MeshError::NonFiniteCoordinate {
                        vertex: idx,
                        axis,
                        value: format!("{value}"),
                        location: None,
                    });
                }
                if !ORIENT3D_COORDS.contains(value) {
                    return Err(MeshError::CoordinateOutOfRange {
                        vertex: idx,
                        axis,
                        value,
                        min: ORIENT3D_COORDS.min,
                        max: ORIENT3D_COORDS.max,
                    });
                }
            }
        }

        for (t, tri) in triangles.iter().enumerate() {
            for &index in tri {
                if index >= vertex_count {
                    return Err(MeshError::IndexOutOfRange {
                        triangle: t as u32,
                        index,
                        vertex_count,
                    });
                }
            }
        }

        Ok(Self {
            vertices,
            triangles,
            meta,
        })
    }

    /// The vertices, in index order.
    #[inline]
    #[must_use]
    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    /// The triangles, as vertex index triples, in index order.
    #[inline]
    #[must_use]
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    /// Provenance and unit metadata.
    #[inline]
    #[must_use]
    pub fn meta(&self) -> &MeshMeta {
        &self.meta
    }

    /// Replaces the metadata, for loaders that learn the format after parsing.
    pub fn set_meta(&mut self, meta: MeshMeta) {
        self.meta = meta;
    }

    /// Number of vertices.
    #[inline]
    #[must_use]
    pub fn vertex_count(&self) -> u32 {
        self.vertices.len() as u32
    }

    /// Number of triangles.
    #[inline]
    #[must_use]
    pub fn triangle_count(&self) -> u32 {
        self.triangles.len() as u32
    }

    /// True if the mesh has no triangles.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// The three corners of triangle `i`.
    ///
    /// # Panics
    /// Panics if `i` is out of range. Indices were validated at construction, so
    /// an out-of-range triangle index is a caller bug.
    #[inline]
    #[must_use]
    pub fn triangle(&self, i: u32) -> [Vec3; 3] {
        let t = self.triangles[i as usize];
        [
            self.vertices[t[0] as usize],
            self.vertices[t[1] as usize],
            self.vertices[t[2] as usize],
        ]
    }

    /// The **computed** unit normal of triangle `i`, or `None` if the triangle is
    /// degenerate.
    ///
    /// Always computed, never read from the file. STL stores a per-facet normal
    /// and it is wrong often enough — zero, unnormalised, or pointing the wrong
    /// way — that trusting it is a liability. Recomputing costs a cross product
    /// and removes a whole class of "the file said so" bug.
    #[inline]
    #[must_use]
    pub fn face_normal(&self, i: u32) -> Option<Vec3> {
        let [a, b, c] = self.triangle(i);
        (b - a).cross(c - a).normalize()
    }

    /// Twice the area of triangle `i`, as the length of the cross product.
    #[inline]
    #[must_use]
    pub fn double_area(&self, i: u32) -> f64 {
        let [a, b, c] = self.triangle(i);
        (b - a).cross(c - a).length()
    }

    /// Axis-aligned bounds of every vertex. Empty for a mesh with no vertices.
    #[must_use]
    pub fn bounds(&self) -> Aabb3 {
        Aabb3::from_points(&self.vertices)
    }

    /// The centroid of triangle `i`, used as the BVH split key.
    #[inline]
    #[must_use]
    pub fn centroid(&self, i: u32) -> Vec3 {
        let [a, b, c] = self.triangle(i);
        // Divide each term rather than summing then dividing: the sum of three
        // large coordinates can overflow where the mean cannot.
        a / 3.0 + b / 3.0 + c / 3.0
    }

    /// Signed volume enclosed by the surface, by the divergence theorem.
    ///
    /// Summed in **ascending triangle index order**, which is part of the
    /// contract: floating-point addition is not associative, so a different
    /// traversal gives a different last bit, and this value is hashed.
    ///
    /// Positive for a closed, outward-oriented surface. Negative means the
    /// winding is inverted. Meaningless for an open surface, which is why
    /// [`validate`] reports watertightness alongside it.
    #[must_use]
    pub fn signed_volume(&self) -> f64 {
        let mut total = 0.0;
        for i in 0..self.triangle_count() {
            let [a, b, c] = self.triangle(i);
            // The signed volume of the tetrahedron (origin, a, b, c), times six.
            total += a.dot(b.cross(c));
        }
        total / 6.0
    }

    /// Total surface area, summed in ascending triangle index order.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let mut total = 0.0;
        for i in 0..self.triangle_count() {
            total += self.double_area(i);
        }
        total / 2.0
    }
}

impl Hashable for TriMesh {
    /// Hashes geometry, topology and provenance, in index order.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("TriMesh");
        self.meta.hash_canonical(h);
        h.usize(self.vertices.len());
        for v in &self.vertices {
            v.hash_canonical(h);
        }
        h.usize(self.triangles.len());
        for t in &self.triangles {
            // u32, not usize: the whole reason indices are u32 is so this line
            // produces identical bytes on wasm32 and native.
            h.u64(u64::from(t[0]));
            h.u64(u64::from(t[1]));
            h.u64(u64::from(t[2]));
        }
        h.end();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A unit cube at the origin, outward-oriented, twelve triangles.
    pub(crate) fn unit_cube() -> TriMesh {
        let v = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(0.0, 1.0, 1.0),
        ];
        let t = vec![
            [0, 2, 1],
            [0, 3, 2], // z = 0, normal -z
            [4, 5, 6],
            [4, 6, 7], // z = 1, normal +z
            [0, 1, 5],
            [0, 5, 4], // y = 0, normal -y
            [2, 3, 7],
            [2, 7, 6], // y = 1, normal +y
            [0, 4, 7],
            [0, 7, 3], // x = 0, normal -x
            [1, 2, 6],
            [1, 6, 5], // x = 1, normal +x
        ];
        TriMesh::new(v, t, MeshMeta::synthetic()).expect("valid cube")
    }

    #[test]
    fn cube_has_the_right_measurements() {
        let m = unit_cube();
        assert_eq!(m.vertex_count(), 8);
        assert_eq!(m.triangle_count(), 12);
        assert!(!m.is_empty());
        assert_eq!(m.bounds(), Aabb3::new(Vec3::ZERO, Vec3::ONE));
        assert_eq!(m.signed_volume(), 1.0, "unit cube encloses exactly 1");
        assert_eq!(m.surface_area(), 6.0, "six unit faces");
    }

    #[test]
    fn face_normals_are_computed_and_point_outward() {
        let m = unit_cube();
        // Triangle 0 is on the z = 0 face and must face -z.
        assert_eq!(m.face_normal(0), Some(-Vec3::Z));
        assert_eq!(m.face_normal(2), Some(Vec3::Z));
        assert_eq!(m.face_normal(4), Some(-Vec3::Y));
        assert_eq!(m.face_normal(10), Some(Vec3::X));
        for i in 0..m.triangle_count() {
            let n = m.face_normal(i).expect("non-degenerate");
            assert!((n.length() - 1.0).abs() < 1e-15);
        }
    }

    #[test]
    fn inverted_winding_gives_negative_volume() {
        let m = unit_cube();
        let flipped: Vec<[u32; 3]> = m.triangles().iter().map(|t| [t[0], t[2], t[1]]).collect();
        let inverted = TriMesh::new(m.vertices().to_vec(), flipped, MeshMeta::synthetic())
            .expect("still a valid mesh, just inside out");
        assert_eq!(inverted.signed_volume(), -1.0);
        assert_eq!(inverted.surface_area(), 6.0, "area is unsigned");
    }

    #[test]
    fn degenerate_triangles_load_rather_than_erroring() {
        // A validation finding, not a parse error: real exports are full of them.
        let v = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
        let repeated = TriMesh::new(v.clone(), vec![[0, 1, 1]], MeshMeta::synthetic())
            .expect("a repeated index must load");
        assert_eq!(repeated.face_normal(0), None);
        assert_eq!(repeated.double_area(0), 0.0);

        let collinear = TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::new(2.0, 0.0, 0.0)],
            vec![[0, 1, 2]],
            MeshMeta::synthetic(),
        )
        .expect("collinear vertices must load");
        assert_eq!(collinear.face_normal(0), None);
    }

    #[test]
    fn non_finite_coordinates_are_rejected_with_the_offending_vertex() {
        for (axis, bad) in [
            (0, Vec3::new(f64::NAN, 0.0, 0.0)),
            (1, Vec3::new(0.0, f64::INFINITY, 0.0)),
            (2, Vec3::new(0.0, 0.0, f64::NEG_INFINITY)),
        ] {
            let err = TriMesh::new(vec![Vec3::ZERO, bad], vec![], MeshMeta::synthetic())
                .expect_err("must reject");
            match err {
                MeshError::NonFiniteCoordinate {
                    vertex, axis: a, ..
                } => {
                    assert_eq!(vertex, 1);
                    assert_eq!(a, axis);
                }
                other => panic!("wrong error: {other}"),
            }
            assert!(err.to_string().contains("vertex 1"));
        }
    }

    #[test]
    fn coordinates_outside_the_predicate_range_are_rejected() {
        let too_big = Vec3::new(ORIENT3D_COORDS.max * 10.0, 0.0, 0.0);
        let err = TriMesh::new(vec![too_big], vec![], MeshMeta::synthetic()).expect_err("reject");
        assert!(matches!(
            err,
            MeshError::CoordinateOutOfRange {
                vertex: 0,
                axis: 0,
                ..
            }
        ));
        assert!(err.to_string().contains("orient3d"));

        let too_small = Vec3::new(0.0, ORIENT3D_COORDS.min / 10.0, 0.0);
        let err = TriMesh::new(vec![too_small], vec![], MeshMeta::synthetic()).expect_err("reject");
        assert!(matches!(
            err,
            MeshError::CoordinateOutOfRange { axis: 1, .. }
        ));

        // Zero is always admissible, and ordinary part-scale coordinates are fine.
        TriMesh::new(
            vec![Vec3::ZERO, Vec3::splat(150.0)],
            vec![],
            MeshMeta::synthetic(),
        )
        .expect("ordinary coordinates must load");
    }

    #[test]
    fn out_of_range_indices_are_rejected() {
        let err = TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2], [0, 1, 3]],
            MeshMeta::synthetic(),
        )
        .expect_err("must reject");
        match err {
            MeshError::IndexOutOfRange {
                triangle,
                index,
                vertex_count,
            } => {
                assert_eq!((triangle, index, vertex_count), (1, 3, 3));
            }
            other => panic!("wrong error: {other}"),
        }
    }

    #[test]
    fn hashing_covers_geometry_topology_and_units() {
        let m = unit_cube();
        assert_eq!(m.canonical_digest(), unit_cube().canonical_digest());

        // A moved vertex changes the hash.
        let mut moved = m.vertices().to_vec();
        moved[0] = Vec3::new(0.5, 0.0, 0.0);
        let m2 = TriMesh::new(moved, m.triangles().to_vec(), MeshMeta::synthetic()).expect("valid");
        assert_ne!(m.canonical_digest(), m2.canonical_digest());

        // A different winding changes the hash even though the geometry is the
        // same set of points.
        let rewound: Vec<[u32; 3]> = m.triangles().iter().map(|t| [t[0], t[2], t[1]]).collect();
        let m3 =
            TriMesh::new(m.vertices().to_vec(), rewound, MeshMeta::synthetic()).expect("valid");
        assert_ne!(m.canonical_digest(), m3.canonical_digest());

        // And so does the source unit, since it changes what the numbers mean.
        let mut meta = MeshMeta::synthetic();
        meta.source_unit = Unit::Inch;
        let m4 = TriMesh::new(m.vertices().to_vec(), m.triangles().to_vec(), meta).expect("valid");
        assert_ne!(m.canonical_digest(), m4.canonical_digest());
    }

    #[test]
    fn centroid_is_the_mean_and_does_not_overflow() {
        let m = unit_cube();
        let [a, b, c] = m.triangle(0);
        let expected = Vec3::new(
            (a.x + b.x + c.x) / 3.0,
            (a.y + b.y + c.y) / 3.0,
            (a.z + b.z + c.z) / 3.0,
        );
        let got = m.centroid(0);
        assert!((got - expected).length() < 1e-15);

        // Summing first would overflow to infinity here.
        let huge = f64::MAX / 2.0;
        let big = TriMesh::new(
            vec![Vec3::splat(huge), Vec3::splat(huge), Vec3::splat(huge)],
            vec![[0, 1, 2]],
            MeshMeta::synthetic(),
        );
        // ORIENT3D_COORDS rejects it, which is the more useful outcome; assert
        // that rather than pretending the centroid case arises.
        assert!(big.is_err());
    }

    #[test]
    fn empty_mesh_is_representable() {
        let m = TriMesh::new(Vec::new(), Vec::new(), MeshMeta::synthetic()).expect("valid");
        assert!(m.is_empty());
        assert_eq!(m.triangle_count(), 0);
        assert_eq!(m.signed_volume(), 0.0);
        assert_eq!(m.surface_area(), 0.0);
        assert_eq!(m.bounds(), Aabb3::EMPTY);
    }

    #[test]
    fn meta_reports_the_scale_that_was_applied() {
        let mut meta = MeshMeta::synthetic();
        meta.source_unit = Unit::Inch;
        assert_eq!(meta.scale_applied(), 25.4);
        assert_eq!(MeshMeta::synthetic().scale_applied(), 1.0);
    }
}
