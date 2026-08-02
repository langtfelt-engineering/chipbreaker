// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Exact triangle-triangle intersection, and the self-intersection check.
//!
//! # Why this is opt-in
//!
//! Finding every intersecting pair costs `O(n log n)` with a large constant: a
//! BVH query per triangle, then an exact test per candidate pair. On a
//! million-triangle model that is minutes, not milliseconds, so it sits behind
//! `--check-self-intersect` rather than running on every validation.
//!
//! # The test
//!
//! Two non-coplanar triangles intersect iff **some edge of one crosses the
//! other**. That is complete: when two triangle interiors meet, the intersection
//! is a segment, and each of its endpoints lies on an edge of one triangle or
//! the other. So six segment-triangle tests decide the general case, and each
//! reduces to predicates that already exist:
//!
//! - the segment's endpoints must lie on opposite sides of the triangle's plane
//!   (two `orient3d` calls), and
//! - the segment's *line* must pass through the triangle, which is the same
//!   three-edge-function test [`crate::mesh::bvh`] uses for ray casting.
//!
//! The coplanar case falls out separately: if every vertex of one triangle lies
//! on the other's plane, the problem is two-dimensional and is decided by a
//! separating-axis test using [`orient2d`] in the dominant coordinate plane.
//!
//! Everything is exact. A float "do these overlap" test on nearly-coplanar
//! triangles is precisely the kind of thing that reports a different answer on
//! two machines, and a self-intersection report that varies by machine is worse
//! than none.
//!
//! # Touching counts
//!
//! For triangles that share no vertex, touching at a point or along an edge *is*
//! a self-intersection: a valid solid does not have two unrelated faces resting
//! against each other. So zero-valued predicates are treated as contact rather
//! than as separation. Adjacent triangles — those sharing a vertex index — are
//! excluded before the test runs, since their shared edge is not a defect.

use crate::math::{Aabb3, Vec2, Vec3};
use crate::mesh::TriMesh;
use crate::mesh::bvh::Bvh;
use crate::predicates::{Orientation, orient2d, orient3d};

/// True if every non-zero orientation among `signs` shares a sign.
///
/// Zeros are permitted and mean "on the boundary", which for this test counts as
/// contact rather than separation.
#[inline]
fn consistent(signs: [Orientation; 3]) -> bool {
    let mut seen = Orientation::Zero;
    for s in signs {
        if s == Orientation::Zero {
            continue;
        }
        if seen == Orientation::Zero {
            seen = s;
        } else if seen != s {
            return false;
        }
    }
    true
}

/// True if all three signs are non-zero and equal — the strict separation test.
#[inline]
fn strictly_same_side(signs: [Orientation; 3]) -> bool {
    signs[0] != Orientation::Zero && signs[1] == signs[0] && signs[2] == signs[0]
}

/// True if the segment `a`-`b` meets triangle `tri`, boundary included.
///
/// Assumes the segment is not coplanar with the triangle; the caller handles
/// that case.
fn segment_meets_triangle(a: Vec3, b: Vec3, tri: [Vec3; 3]) -> bool {
    let sa = orient3d(tri[0], tri[1], tri[2], a);
    let sb = orient3d(tri[0], tri[1], tri[2], b);
    // Both strictly on the same side: the segment never reaches the plane.
    if sa != Orientation::Zero && sa == sb {
        return false;
    }
    // Both exactly on the plane: coplanar, decided elsewhere.
    if sa == Orientation::Zero && sb == Orientation::Zero {
        return false;
    }
    // The line through a and b meets the plane; does it do so inside the
    // triangle? This is the ray caster's edge-function test.
    consistent([
        orient3d(a, b, tri[0], tri[1]),
        orient3d(a, b, tri[1], tri[2]),
        orient3d(a, b, tri[2], tri[0]),
    ])
}

/// Projects a point onto the coordinate plane that the triangle's normal is
/// least aligned with, which is the projection that cannot be degenerate.
#[inline]
fn project(v: Vec3, drop_axis: usize) -> Vec2 {
    match drop_axis {
        0 => Vec2::new(v.y, v.z),
        1 => Vec2::new(v.z, v.x),
        _ => Vec2::new(v.x, v.y),
    }
}

/// Separating-axis overlap test for two coplanar triangles, in 2D and exact.
fn coplanar_overlap(t1: [Vec3; 3], t2: [Vec3; 3]) -> bool {
    // Drop the axis the normal is most aligned with, so the projected triangles
    // keep positive area.
    let n = (t1[1] - t1[0]).cross(t1[2] - t1[0]).abs();
    let drop_axis = if n.x >= n.y && n.x >= n.z {
        0
    } else if n.y >= n.z {
        1
    } else {
        2
    };

    let mut a: [Vec2; 3] = core::array::from_fn(|i| project(t1[i], drop_axis));
    let mut b: [Vec2; 3] = core::array::from_fn(|i| project(t2[i], drop_axis));
    // Normalise both to counter-clockwise so "outside" has one meaning.
    if orient2d(a[0], a[1], a[2]) == Orientation::Negative {
        a.swap(1, 2);
    }
    if orient2d(b[0], b[1], b[2]) == Orientation::Negative {
        b.swap(1, 2);
    }

    // If every vertex of one triangle is strictly right of some directed edge of
    // the other, a separating line exists and they do not overlap. For convex
    // shapes the edges are the only candidate axes, so this is exact and
    // complete.
    for (p, q) in [(a, b), (b, a)] {
        for k in 0..3 {
            let (e0, e1) = (p[k], p[(k + 1) % 3]);
            if q.iter()
                .all(|v| orient2d(e0, e1, *v) == Orientation::Negative)
            {
                return false;
            }
        }
    }
    true
}

/// True if two triangles intersect, treating contact as intersection.
///
/// Callers must exclude pairs that share a vertex; see the module documentation.
#[must_use]
pub fn triangles_intersect(t1: [Vec3; 3], t2: [Vec3; 3]) -> bool {
    let side_of_t2 = [
        orient3d(t2[0], t2[1], t2[2], t1[0]),
        orient3d(t2[0], t2[1], t2[2], t1[1]),
        orient3d(t2[0], t2[1], t2[2], t1[2]),
    ];
    if strictly_same_side(side_of_t2) {
        return false;
    }
    let side_of_t1 = [
        orient3d(t1[0], t1[1], t1[2], t2[0]),
        orient3d(t1[0], t1[1], t1[2], t2[1]),
        orient3d(t1[0], t1[1], t1[2], t2[2]),
    ];
    if strictly_same_side(side_of_t1) {
        return false;
    }

    // Coplanar: every vertex of each lies on the other's plane.
    if side_of_t2.iter().all(|s| *s == Orientation::Zero)
        && side_of_t1.iter().all(|s| *s == Orientation::Zero)
    {
        return coplanar_overlap(t1, t2);
    }

    // General case: six segment-triangle tests.
    for k in 0..3 {
        if segment_meets_triangle(t1[k], t1[(k + 1) % 3], t2) {
            return true;
        }
        if segment_meets_triangle(t2[k], t2[(k + 1) % 3], t1) {
            return true;
        }
    }
    false
}

/// Every pair of non-adjacent triangles that intersect, ascending.
///
/// Pairs sharing a vertex index are excluded: two triangles meeting along their
/// shared edge is what a manifold surface *is*, not a defect.
///
/// Degenerate triangles are excluded too. A zero-area triangle has no interior
/// to intersect, it is already reported as a
/// [`crate::mesh::validate::FindingKind::DegenerateTriangle`], and including it
/// would bury the real findings under noise.
#[must_use]
pub fn self_intersections(mesh: &TriMesh, bvh: &Bvh) -> Vec<(u32, u32)> {
    let mut pairs = Vec::new();
    let mut candidates = Vec::new();

    let degenerate: Vec<bool> = (0..mesh.triangle_count())
        .map(|i| {
            let t = mesh.triangles()[i as usize];
            t[0] == t[1] || t[1] == t[2] || t[2] == t[0] || mesh.face_normal(i).is_none()
        })
        .collect();

    for i in 0..mesh.triangle_count() {
        if degenerate[i as usize] {
            continue;
        }
        let tri_i = mesh.triangle(i);
        bvh.query_aabb(&Aabb3::from_points(&tri_i), &mut candidates);
        let idx_i = mesh.triangles()[i as usize];
        for &j in &candidates {
            // Each pair once, in ascending order.
            if j <= i || degenerate[j as usize] {
                continue;
            }
            let idx_j = mesh.triangles()[j as usize];
            if idx_i.iter().any(|a| idx_j.contains(a)) {
                continue;
            }
            if triangles_intersect(tri_i, mesh.triangle(j)) {
                pairs.push((i, j));
            }
        }
    }
    pairs.sort_unstable();
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshMeta, shapes};

    fn tri(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [Vec3; 3] {
        [
            Vec3::from_array(a),
            Vec3::from_array(b),
            Vec3::from_array(c),
        ]
    }

    #[test]
    fn two_triangles_that_cross_are_detected() {
        // A horizontal triangle and a vertical one passing through it.
        let flat = tri([0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0]);
        let upright = tri([1.0, 1.0, -2.0], [1.0, 1.0, 2.0], [2.0, 2.0, 0.0]);
        assert!(triangles_intersect(flat, upright));
        assert!(triangles_intersect(upright, flat), "the test is symmetric");
    }

    #[test]
    fn separated_triangles_are_not_detected() {
        let a = tri([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        // Parallel, well above.
        let b = tri([0.0, 0.0, 5.0], [1.0, 0.0, 5.0], [0.0, 1.0, 5.0]);
        assert!(!triangles_intersect(a, b));
        // Same plane, disjoint in 2D.
        let c = tri([10.0, 10.0, 0.0], [11.0, 10.0, 0.0], [10.0, 11.0, 0.0]);
        assert!(!triangles_intersect(a, c));
        // Crossing planes, but the triangles themselves miss each other.
        let d = tri([9.0, 9.0, -1.0], [9.0, 9.0, 1.0], [10.0, 10.0, 0.0]);
        assert!(!triangles_intersect(a, d));
    }

    #[test]
    fn coplanar_overlap_is_decided_exactly() {
        let a = tri([0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0]);
        // Overlapping, same plane.
        let b = tri([1.0, 1.0, 0.0], [5.0, 1.0, 0.0], [1.0, 5.0, 0.0]);
        assert!(triangles_intersect(a, b));
        // Sharing only a touching corner, same plane: contact counts.
        let c = tri([4.0, 0.0, 0.0], [8.0, 0.0, 0.0], [4.0, 4.0, 0.0]);
        assert!(triangles_intersect(a, c));
        // Same plane, clearly apart.
        let d = tri([100.0, 0.0, 0.0], [104.0, 0.0, 0.0], [100.0, 4.0, 0.0]);
        assert!(!triangles_intersect(a, d));
        // One inside the other.
        let e = tri([1.0, 1.0, 0.0], [2.0, 1.0, 0.0], [1.0, 2.0, 0.0]);
        assert!(triangles_intersect(a, e));
    }

    #[test]
    fn coplanar_detection_works_in_every_orientation() {
        // The dominant-axis projection must pick a non-degenerate plane, so the
        // same configuration has to be found however it is oriented.
        for axis in 0..3 {
            let permute = |v: [f64; 3]| match axis {
                0 => [v[2], v[0], v[1]],
                1 => [v[1], v[2], v[0]],
                _ => v,
            };
            let a = tri(
                permute([0.0, 0.0, 0.0]),
                permute([4.0, 0.0, 0.0]),
                permute([0.0, 4.0, 0.0]),
            );
            let b = tri(
                permute([1.0, 1.0, 0.0]),
                permute([5.0, 1.0, 0.0]),
                permute([1.0, 5.0, 0.0]),
            );
            assert!(triangles_intersect(a, b), "axis {axis}");
        }
    }

    #[test]
    fn touching_at_a_point_counts_as_intersection() {
        // Two unrelated faces resting against each other is a defect, even
        // though they do not overlap.
        let a = tri([0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0]);
        let b = tri([1.0, 1.0, 0.0], [3.0, 3.0, 3.0], [0.0, 3.0, 3.0]);
        assert!(triangles_intersect(a, b), "vertex resting on a face");
    }

    #[test]
    fn a_closed_solid_has_no_self_intersections() {
        for (label, mesh) in [
            ("cube", shapes::cube(10.0)),
            ("sphere", shapes::icosphere(5.0, 2)),
            ("cylinder", shapes::cylinder(4.0, 9.0, 32)),
            ("torus", shapes::torus(6.0, 2.0, 24, 12)),
            ("lattice", shapes::lattice_block(3)),
        ] {
            let bvh = Bvh::build(&mesh);
            let found = self_intersections(&mesh, &bvh);
            assert!(found.is_empty(), "{label}: {found:?}");
        }
    }

    #[test]
    fn an_interpenetrating_pair_is_found() {
        // Two cubes overlapping in space: a classic self-intersection.
        let a = shapes::cube(10.0);
        let b = shapes::box_solid(Vec3::splat(5.0), Vec3::splat(15.0));
        let mut v = a.vertices().to_vec();
        let mut t = a.triangles().to_vec();
        let offset = v.len() as u32;
        v.extend_from_slice(b.vertices());
        t.extend(
            b.triangles()
                .iter()
                .map(|x| [x[0] + offset, x[1] + offset, x[2] + offset]),
        );
        let mesh = TriMesh::new(v, t, MeshMeta::synthetic()).expect("valid");

        let bvh = Bvh::build(&mesh);
        let found = self_intersections(&mesh, &bvh);
        assert!(!found.is_empty(), "overlapping cubes must self-intersect");
        // Every reported pair must be one triangle from each cube.
        for (i, j) in &found {
            assert!(*i < 12 && *j >= 12, "pair ({i}, {j}) is within one cube");
        }
    }

    #[test]
    fn adjacent_triangles_are_never_reported() {
        // Two triangles sharing an edge is what a manifold surface is.
        let mesh = TriMesh::new(
            vec![
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [1, 3, 2]],
            MeshMeta::synthetic(),
        )
        .expect("valid");
        let bvh = Bvh::build(&mesh);
        assert!(self_intersections(&mesh, &bvh).is_empty());
    }

    #[test]
    fn degenerate_triangles_are_skipped() {
        // A zero-area triangle has no interior to intersect, and is already
        // reported as degenerate. Including it would bury the real findings.
        let mesh = TriMesh::new(
            vec![
                Vec3::ZERO,
                Vec3::new(4.0, 0.0, 0.0),
                Vec3::new(0.0, 4.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(2.0, 2.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 4]],
            MeshMeta::synthetic(),
        )
        .expect("valid");
        let bvh = Bvh::build(&mesh);
        assert!(self_intersections(&mesh, &bvh).is_empty());
    }

    #[test]
    fn results_are_sorted_and_reproducible() {
        let a = shapes::cube(10.0);
        let b = shapes::box_solid(Vec3::splat(5.0), Vec3::splat(15.0));
        let mut v = a.vertices().to_vec();
        let mut t = a.triangles().to_vec();
        let offset = v.len() as u32;
        v.extend_from_slice(b.vertices());
        t.extend(
            b.triangles()
                .iter()
                .map(|x| [x[0] + offset, x[1] + offset, x[2] + offset]),
        );
        let mesh = TriMesh::new(v, t, MeshMeta::synthetic()).expect("valid");
        let bvh = Bvh::build(&mesh);

        let first = self_intersections(&mesh, &bvh);
        let second = self_intersections(&mesh, &bvh);
        assert_eq!(first, second);
        let mut sorted = first.clone();
        sorted.sort_unstable();
        assert_eq!(first, sorted, "pairs must come out sorted");
        for (i, j) in &first {
            assert!(i < j, "pairs must be ordered within themselves");
        }
    }

    #[test]
    fn query_aabb_finds_every_overlapping_triangle() {
        // The BVH query must not miss candidates, or self-intersections go
        // unreported.
        let mesh = shapes::icosphere(5.0, 2);
        let bvh = Bvh::build(&mesh);
        let mut out = Vec::new();
        for i in 0..mesh.triangle_count() {
            let bounds = Aabb3::from_points(&mesh.triangle(i));
            bvh.query_aabb(&bounds, &mut out);
            assert!(out.contains(&i), "triangle {i} did not find itself");
            // A brute-force sweep must not find anything the query missed.
            for j in 0..mesh.triangle_count() {
                if Aabb3::from_points(&mesh.triangle(j)).intersects(&bounds) {
                    assert!(out.contains(&j), "query missed {j} overlapping {i}");
                }
            }
        }
        // And an empty query returns nothing.
        bvh.query_aabb(&Aabb3::EMPTY, &mut out);
        assert!(out.is_empty());
    }
}
