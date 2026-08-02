// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Deterministic generators for analytic solids.
//!
//! These exist for three reasons, in order of importance:
//!
//! 1. **They have known closed-form volume and area**, so the mesh pipeline can
//!    be checked against arithmetic rather than against its own output.
//! 2. **They are reproducible**, so the committed corpus can be regenerated and
//!    diffed rather than being an opaque blob somebody once made in a CAD
//!    package.
//! 3. **[`lattice_block`] is adversarial on purpose** — every vertex lands on a
//!    round coordinate, so a lattice-aligned ray cast passes precisely through
//!    edges and vertices. That is the configuration that finds tie-break bugs,
//!    and it is built deliberately rather than hoped for.
//!
//! All trigonometry goes through [`crate::transcendental`], so a sphere
//! tessellated on Windows is bit-identical to one tessellated under `wasmtime`.
//! That is not incidental: these meshes feed golden hashes.

use crate::math::Vec3;
use crate::mesh::{MeshMeta, TriMesh};
use crate::transcendental::sin_cos;

use core::f64::consts::PI;

/// Builds a mesh, panicking on failure.
///
/// The generators below produce finite, in-range coordinates by construction, so
/// a failure here is a bug in this module rather than a runtime condition.
fn build(vertices: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> TriMesh {
    TriMesh::new(vertices, triangles, MeshMeta::synthetic())
        .unwrap_or_else(|e| panic!("shape generator produced an invalid mesh: {e}"))
}

/// An axis-aligned box from `min` to `max`, outward-oriented, twelve triangles.
#[must_use]
pub fn box_solid(min: Vec3, max: Vec3) -> TriMesh {
    let v = vec![
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, max.z),
        Vec3::new(min.x, max.y, max.z),
    ];
    let t = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    build(v, t)
}

/// A cube of side `size` with one corner at the origin.
#[must_use]
pub fn cube(size: f64) -> TriMesh {
    box_solid(Vec3::ZERO, Vec3::splat(size))
}

/// An icosphere of the given radius, centred at the origin.
///
/// `subdivisions` of 0 gives the bare icosahedron (20 triangles); each level
/// quadruples the triangle count.
///
/// Built by subdividing an icosahedron and projecting onto the sphere, which
/// needs only `sqrt` — correctly rounded per IEEE-754, hence identical on every
/// target without recourse to [`crate::transcendental`]. The uniform triangle
/// size is also what makes it a good BVH stress case, unlike a UV sphere whose
/// poles are pathological.
///
/// # Panics
/// Panics if `subdivisions` is large enough to overflow the `u32` index space.
#[must_use]
pub fn icosphere(radius: f64, subdivisions: u32) -> TriMesh {
    // Golden ratio, from sqrt(5): exact enough and deterministic.
    let phi = (1.0 + 5.0f64.sqrt()) / 2.0;
    let mut vertices: Vec<Vec3> = vec![
        Vec3::new(-1.0, phi, 0.0),
        Vec3::new(1.0, phi, 0.0),
        Vec3::new(-1.0, -phi, 0.0),
        Vec3::new(1.0, -phi, 0.0),
        Vec3::new(0.0, -1.0, phi),
        Vec3::new(0.0, 1.0, phi),
        Vec3::new(0.0, -1.0, -phi),
        Vec3::new(0.0, 1.0, -phi),
        Vec3::new(phi, 0.0, -1.0),
        Vec3::new(phi, 0.0, 1.0),
        Vec3::new(-phi, 0.0, -1.0),
        Vec3::new(-phi, 0.0, 1.0),
    ];
    let mut triangles: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    for _ in 0..subdivisions {
        // A BTreeMap keyed by the ordered edge, so midpoint numbering is a
        // function of the topology rather than of a hash seed.
        let mut midpoints: std::collections::BTreeMap<(u32, u32), u32> =
            std::collections::BTreeMap::new();
        let mut next = Vec::with_capacity(triangles.len() * 4);
        for t in &triangles {
            let mut mid = [0u32; 3];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = if a < b { (a, b) } else { (b, a) };
                let index = *midpoints.entry(key).or_insert_with(|| {
                    // Halve each term rather than summing then halving: the same
                    // overflow-safety argument as Aabb3::center.
                    let m = vertices[a as usize] / 2.0 + vertices[b as usize] / 2.0;
                    vertices.push(m);
                    u32::try_from(vertices.len() - 1).expect("index space exhausted")
                });
                mid[k] = index;
            }
            next.push([t[0], mid[0], mid[2]]);
            next.push([t[1], mid[1], mid[0]]);
            next.push([t[2], mid[2], mid[1]]);
            next.push([mid[0], mid[1], mid[2]]);
        }
        triangles = next;
    }

    // Project onto the sphere. A vertex is never zero here, so `normalize`
    // always succeeds; the `map_or` keeps the panic-free contract anyway.
    let projected = vertices
        .into_iter()
        .map(|v| v.normalize().map_or(Vec3::ZERO, |n| n * radius))
        .collect();
    build(projected, triangles)
}

/// A closed cylinder along `+z`, radius `r`, height `h`, base at the origin.
///
/// `segments` is the number of facets around the circumference; at least 3.
///
/// # Panics
/// Panics if `segments` is below 3.
#[must_use]
pub fn cylinder(r: f64, h: f64, segments: u32) -> TriMesh {
    assert!(segments >= 3, "a cylinder needs at least three segments");
    let n = segments;
    let mut v = Vec::with_capacity((2 * n + 2) as usize);
    for i in 0..n {
        let (s, c) = sin_cos(2.0 * PI * f64::from(i) / f64::from(n));
        v.push(Vec3::new(r * c, r * s, 0.0));
    }
    for i in 0..n {
        let (s, c) = sin_cos(2.0 * PI * f64::from(i) / f64::from(n));
        v.push(Vec3::new(r * c, r * s, h));
    }
    let bottom_centre = 2 * n;
    let top_centre = 2 * n + 1;
    v.push(Vec3::new(0.0, 0.0, 0.0));
    v.push(Vec3::new(0.0, 0.0, h));

    let mut t = Vec::with_capacity((4 * n) as usize);
    for i in 0..n {
        let j = (i + 1) % n;
        // Side quad, outward.
        t.push([i, j, n + j]);
        t.push([i, n + j, n + i]);
        // Bottom fan, facing -z.
        t.push([bottom_centre, j, i]);
        // Top fan, facing +z.
        t.push([top_centre, n + i, n + j]);
    }
    build(v, t)
}

/// A closed cone along `+z`, base radius `r` at the origin, apex at height `h`.
///
/// # Panics
/// Panics if `segments` is below 3.
#[must_use]
pub fn cone(r: f64, h: f64, segments: u32) -> TriMesh {
    assert!(segments >= 3, "a cone needs at least three segments");
    let n = segments;
    let mut v = Vec::with_capacity((n + 2) as usize);
    for i in 0..n {
        let (s, c) = sin_cos(2.0 * PI * f64::from(i) / f64::from(n));
        v.push(Vec3::new(r * c, r * s, 0.0));
    }
    let centre = n;
    let apex = n + 1;
    v.push(Vec3::new(0.0, 0.0, 0.0));
    v.push(Vec3::new(0.0, 0.0, h));

    let mut t = Vec::with_capacity((2 * n) as usize);
    for i in 0..n {
        let j = (i + 1) % n;
        t.push([i, j, apex]);
        t.push([centre, j, i]);
    }
    build(v, t)
}

/// A torus in the `xy` plane: `major` is the ring radius, `minor` the tube
/// radius. Genus 1, which makes it the corpus's Euler-characteristic check.
///
/// # Panics
/// Panics if either segment count is below 3.
#[must_use]
pub fn torus(major: f64, minor: f64, major_segments: u32, minor_segments: u32) -> TriMesh {
    assert!(
        major_segments >= 3 && minor_segments >= 3,
        "too few segments"
    );
    let (nu, nv) = (major_segments, minor_segments);
    let mut v = Vec::with_capacity((nu * nv) as usize);
    for i in 0..nu {
        let (su, cu) = sin_cos(2.0 * PI * f64::from(i) / f64::from(nu));
        for j in 0..nv {
            let (sv, cv) = sin_cos(2.0 * PI * f64::from(j) / f64::from(nv));
            let radial = major + minor * cv;
            v.push(Vec3::new(radial * cu, radial * su, minor * sv));
        }
    }
    let index = |i: u32, j: u32| (i % nu) * nv + (j % nv);
    let mut t = Vec::with_capacity((2 * nu * nv) as usize);
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (
                index(i, j),
                index(i + 1, j),
                index(i + 1, j + 1),
                index(i, j + 1),
            );
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    build(v, t)
}

/// An `n` x `n` x `n` box whose faces are subdivided into unit quads, so that
/// **every vertex sits on an integer coordinate**.
///
/// This is the adversarial case for [`crate::mesh::bvh`]. A ray cast on the
/// integer lattice passes exactly through vertices and along edges of this mesh,
/// which is precisely the configuration where a naive ray-triangle test reports
/// two hits or none and leaks material. If the Simulation of Simplicity tie-break
/// is wrong, this mesh finds it.
///
/// # Panics
/// Panics if `n` is zero.
#[must_use]
pub fn lattice_block(n: u32) -> TriMesh {
    assert!(n >= 1, "a lattice block needs at least one cell");
    let side = f64::from(n);
    let mut vertices: Vec<Vec3> = Vec::new();
    // A BTreeMap keyed by integer coordinates: deterministic, and it welds the
    // shared vertices along face seams for free.
    let mut index_of: std::collections::BTreeMap<(u32, u32, u32), u32> =
        std::collections::BTreeMap::new();
    let mut vertex = |x: u32, y: u32, z: u32, vertices: &mut Vec<Vec3>| -> u32 {
        *index_of.entry((x, y, z)).or_insert_with(|| {
            vertices.push(Vec3::new(f64::from(x), f64::from(y), f64::from(z)));
            u32::try_from(vertices.len() - 1).expect("index space exhausted")
        })
    };

    let mut triangles: Vec<[u32; 3]> = Vec::new();
    // For each of the six faces, emit an n x n grid of quads wound outward.
    for (axis, at_max) in [
        (0u8, false),
        (0, true),
        (1, false),
        (1, true),
        (2, false),
        (2, true),
    ] {
        let fixed = if at_max { n } else { 0 };
        for a in 0..n {
            for b in 0..n {
                let corner = |da: u32, db: u32| -> (u32, u32, u32) {
                    match axis {
                        0 => (fixed, a + da, b + db),
                        1 => (a + da, fixed, b + db),
                        _ => (a + da, b + db, fixed),
                    }
                };
                let p00 = corner(0, 0);
                let p10 = corner(1, 0);
                let p11 = corner(1, 1);
                let p01 = corner(0, 1);
                let i00 = vertex(p00.0, p00.1, p00.2, &mut vertices);
                let i10 = vertex(p10.0, p10.1, p10.2, &mut vertices);
                let i11 = vertex(p11.0, p11.1, p11.2, &mut vertices);
                let i01 = vertex(p01.0, p01.1, p01.2, &mut vertices);
                // Wind so the normal points away from the block's centre.
                // `at_max` faces and `axis == 1` each flip the handedness, so the
                // two flips cancel — hence the exclusive-or.
                if at_max ^ (axis == 1) {
                    triangles.push([i00, i10, i11]);
                    triangles.push([i00, i11, i01]);
                } else {
                    triangles.push([i00, i11, i10]);
                    triangles.push([i00, i01, i11]);
                }
            }
        }
    }
    debug_assert!(side > 0.0);
    build(vertices, triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::validate::validate;

    /// Every generated solid must be closed, manifold and outward-oriented.
    fn assert_solid(m: &TriMesh, label: &str) {
        let r = validate(m);
        assert!(r.is_manifold, "{label}: not manifold: {:?}", r.findings);
        assert!(r.is_watertight, "{label}: not watertight: {:?}", r.findings);
        assert!(
            r.is_orientation_consistent,
            "{label}: inconsistent winding: {:?}",
            r.findings
        );
        assert!(
            r.signed_volume > 0.0,
            "{label}: inside out ({})",
            r.signed_volume
        );
        assert!(r.is_solid(), "{label}: not a solid");
    }

    #[test]
    fn every_shape_is_a_closed_outward_solid() {
        assert_solid(&cube(1.0), "cube");
        assert_solid(
            &box_solid(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(4.0, 5.0, 6.0)),
            "box",
        );
        for s in 0..=3 {
            assert_solid(&icosphere(1.0, s), &format!("icosphere/{s}"));
        }
        for seg in [3u32, 8, 32, 64] {
            assert_solid(&cylinder(1.0, 2.0, seg), &format!("cylinder/{seg}"));
            assert_solid(&cone(1.0, 2.0, seg), &format!("cone/{seg}"));
        }
        assert_solid(&torus(3.0, 1.0, 16, 8), "torus");
        for n in 1..=4 {
            assert_solid(&lattice_block(n), &format!("lattice/{n}"));
        }
    }

    #[test]
    fn volumes_converge_to_the_closed_form() {
        assert_eq!(cube(2.0).signed_volume(), 8.0);

        // An inscribed polyhedron always under-estimates, converging from below.
        // The interesting property is the *rate*: each subdivision quarters the
        // triangle edge length, so the volume error should fall by roughly 4x.
        // Asserting the rate catches a subtly wrong subdivision that a single
        // absolute tolerance would wave through.
        let exact_sphere = 4.0 / 3.0 * PI;
        let mut previous = 0.0;
        let mut errors = Vec::new();
        for s in 0..=4 {
            let v = icosphere(1.0, s).signed_volume();
            assert!(
                v < exact_sphere,
                "an inscribed sphere cannot exceed the true volume"
            );
            assert!(v > previous, "refinement must improve the estimate");
            previous = v;
            errors.push(exact_sphere - v);
        }
        for w in errors.windows(2) {
            let ratio = w[0] / w[1];
            assert!(
                (3.0..5.5).contains(&ratio),
                "each subdivision should reduce the error about fourfold, got {ratio}"
            );
        }
        assert!(
            (previous - exact_sphere).abs() / exact_sphere < 0.0025,
            "4 subdivisions lands within 0.25%, got {previous} vs {exact_sphere}"
        );

        // Inscribed prism: exact formula is (n/2) r^2 sin(2 pi / n) h.
        for n in [8u32, 64, 256] {
            let v = cylinder(1.0, 2.0, n).signed_volume();
            let exact = f64::from(n) / 2.0 * sin_cos(2.0 * PI / f64::from(n)).0 * 2.0;
            assert!((v - exact).abs() < 1e-9, "cylinder/{n}: {v} vs {exact}");
        }

        let cone_v = cone(1.0, 3.0, 512).signed_volume();
        let cone_exact = PI * 3.0 / 3.0;
        assert!(
            (cone_v - cone_exact).abs() / cone_exact < 0.001,
            "{cone_v} vs {cone_exact}"
        );

        // 2 pi^2 R r^2. Inscribed, so it under-estimates by about 0.2% at this
        // tessellation — the same order as the sphere, and for the same reason.
        let torus_v = torus(3.0, 1.0, 128, 64).signed_volume();
        let torus_exact = 2.0 * PI * PI * 3.0 * 1.0;
        assert!(torus_v < torus_exact, "an inscribed torus cannot exceed the true volume");
        assert!(
            (torus_v - torus_exact).abs() / torus_exact < 0.0025,
            "{torus_v} vs {torus_exact}"
        );
    }

    #[test]
    fn areas_converge_to_the_closed_form() {
        assert_eq!(cube(2.0).surface_area(), 24.0);
        let sphere = icosphere(1.0, 4).surface_area();
        let exact = 4.0 * PI;
        assert!(
            (sphere - exact).abs() / exact < 0.002,
            "{sphere} vs {exact}"
        );
        assert!(sphere < exact, "an inscribed sphere has less area");
    }

    #[test]
    fn a_torus_has_genus_one() {
        // The corpus's topology check: V - E + F = 0 for a torus, so g = 1.
        let r = validate(&torus(3.0, 1.0, 16, 8));
        assert_eq!(r.components.len(), 1);
        assert_eq!(r.components[0].euler_characteristic, 0);
        assert_eq!(r.components[0].genus, Some(1));
    }

    #[test]
    fn a_sphere_has_genus_zero() {
        for s in 0..=2 {
            let r = validate(&icosphere(1.0, s));
            assert_eq!(r.components[0].euler_characteristic, 2, "subdivision {s}");
            assert_eq!(r.components[0].genus, Some(0));
        }
    }

    #[test]
    fn the_lattice_block_really_is_on_the_lattice() {
        // The property that makes it adversarial. Every coordinate must be an
        // exact integer, so integer-aligned rays hit edges and vertices head on.
        let m = lattice_block(3);
        for v in m.vertices() {
            for c in v.to_array() {
                assert_eq!(c, c.round(), "vertex coordinate {c} is not an integer");
                assert!((0.0..=3.0).contains(&c));
            }
        }
        assert_eq!(m.signed_volume(), 27.0);
        assert_eq!(m.surface_area(), 54.0);
        // 6 faces x 9 quads x 2 triangles.
        assert_eq!(m.triangle_count(), 108);
        // The face grids share their seam vertices, so this is a welded surface.
        let r = validate(&m);
        assert!(r.is_watertight, "{:?}", r.findings);
        assert_eq!(r.components.len(), 1);
        assert_eq!(r.components[0].genus, Some(0));
    }

    #[test]
    fn shape_generation_is_reproducible() {
        use crate::golden::Hashable;
        // These feed golden hashes, so a second call must be byte-identical.
        assert_eq!(
            icosphere(1.0, 3).canonical_digest(),
            icosphere(1.0, 3).canonical_digest()
        );
        assert_eq!(
            torus(3.0, 1.0, 16, 8).canonical_digest(),
            torus(3.0, 1.0, 16, 8).canonical_digest()
        );
        assert_eq!(
            lattice_block(3).canonical_digest(),
            lattice_block(3).canonical_digest()
        );
    }

    #[test]
    fn subdivision_multiplies_triangles_by_four() {
        let mut expected = 20;
        for s in 0..=4 {
            assert_eq!(
                icosphere(1.0, s).triangle_count(),
                expected,
                "subdivision {s}"
            );
            expected *= 4;
        }
        // And a subdivided icosphere stays welded:
        // V - E + F = 2 requires shared midpoints.
        let r = validate(&icosphere(1.0, 2));
        assert!(r.is_watertight);
    }
}
