// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The leak-free ray casting contract.
//!
//! This is the test Unit 5 depends on, and the one worth the most of anybody's
//! attention in Unit 2.
//!
//! # What it proves
//!
//! For a closed, manifold, consistently oriented surface, **every** line through
//! it must produce an even number of crossings, and walking those crossings in
//! order must keep a running depth counter non-negative and return it to zero.
//!
//! If one ray fails that, material leaks: every interval past the bad crossing
//! inverts, solid becomes void, and the simulated stock grows a spike or a
//! tunnel. It happens intermittently, on customer data, and it is very hard to
//! reproduce after the fact. So the tolerance here is **zero** — one leaking ray
//! is a failure, not a statistic.
//!
//! # Why depth rather than strict alternation
//!
//! A single closed shell alternates enter, exit, enter, exit. A scene with a
//! *nested* shell does not: a ray through a sphere inside a sphere enters twice
//! before it leaves anything. The invariant that holds in both cases is that the
//! depth counter never goes negative and ends at zero. Unit 5 must track depth
//! rather than a boolean for exactly this reason.
//!
//! # The lattices
//!
//! Three, deliberately:
//!
//! - **offset** — rays at cell centres, off every vertex and edge. The easy case;
//!   if this fails, something is very wrong.
//! - **aligned** — rays on exactly the integer lattice. Against
//!   [`shapes::lattice_block`], whose vertices are all integers, these pass
//!   precisely through vertices and along edges. This is the case that finds
//!   tie-break bugs.
//! - **diagonal** — non-axis-aligned directions, which exercise the BVH slab
//!   test's zero-component handling and the general position path.

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::bvh::{Bvh, RayStats};
use chipbreaker_core::mesh::validate::validate;
use chipbreaker_core::mesh::{TriMesh, shapes};

/// Rays cast per mesh per lattice. The specification asks for at least 10,000
/// per closed mesh; three lattices of 64x64 gives 12,288.
const LATTICE: u32 = 64;

/// How a ray failed the parity contract.
#[derive(Debug)]
struct Leak {
    origin: Vec3,
    direction: Vec3,
    crossings: usize,
    reason: String,
}

/// Checks one ray, returning a description if it leaks.
fn check_ray(mesh: &TriMesh, bvh: &Bvh, ray: &Ray, stats: &mut RayStats) -> Option<Leak> {
    let mut hits = Vec::new();
    let s = match bvh.intersect_ray_all_into(mesh, ray, &mut hits) {
        Ok(s) => s,
        Err(e) => {
            return Some(Leak {
                origin: ray.origin,
                direction: ray.direction,
                crossings: 0,
                reason: format!("ray rejected: {e}"),
            });
        }
    };
    stats.merge(&s);

    if hits.len() % 2 != 0 {
        return Some(Leak {
            origin: ray.origin,
            direction: ray.direction,
            crossings: hits.len(),
            reason: format!(
                "odd crossing count; material leaks past t = {:?}",
                hits.last().map(|h| h.t)
            ),
        });
    }

    // Depth is checked between *distinct* points on the ray, not between
    // individual crossings.
    //
    // Where the ray grazes a silhouette — through the rim vertex of a cone, say —
    // it touches the surface at a single point and picks up two coincident
    // crossings, one entering and one leaving, at exactly the same `t`. Their
    // relative order is not defined by geometry: there is only one point. Sorting
    // breaks the tie by triangle index, which is arbitrary with respect to the
    // crossing sense, so demanding non-negative depth *within* such a group would
    // fail on a configuration that is perfectly correct — the enclosed interval
    // is zero length and contributes nothing.
    //
    // What must hold is that depth is non-negative between distinct points, and
    // that is what is checked.
    let mut depth = 0i32;
    let mut i = 0usize;
    while i < hits.len() {
        let t = hits[i].t;
        let mut group_delta = 0i32;
        while i < hits.len() && hits[i].t == t {
            group_delta += if hits[i].entering { 1 } else { -1 };
            i += 1;
        }
        depth += group_delta;
        if depth < 0 {
            return Some(Leak {
                origin: ray.origin,
                direction: ray.direction,
                crossings: hits.len(),
                reason: format!(
                    "depth went negative after the crossings at t = {t}; the ray \
                     left material it had not entered"
                ),
            });
        }
    }
    if depth != 0 {
        return Some(Leak {
            origin: ray.origin,
            direction: ray.direction,
            crossings: hits.len(),
            reason: format!("final depth {depth}, expected 0"),
        });
    }
    None
}

/// Sweeps three lattices of rays across a mesh and returns any leaks.
fn sweep(mesh: &TriMesh, label: &str) -> (Vec<Leak>, u64, RayStats) {
    let report = validate(mesh);
    assert!(
        report.is_solid(),
        "{label} is not a closed outward solid, so the parity contract does not \
         apply to it: {:?}",
        report.findings
    );

    let bvh = Bvh::build(mesh);
    let b = mesh.bounds();
    let extent = b.extent();
    let mut leaks = Vec::new();
    let mut cast = 0u64;
    let mut stats = RayStats::default();

    for mode in ["offset", "aligned", "diagonal"] {
        for i in 0..LATTICE {
            for j in 0..LATTICE {
                let (u, v) = (f64::from(i), f64::from(j));
                let n = f64::from(LATTICE);
                let ray = match mode {
                    // Cell centres: comfortably off every feature.
                    "offset" => Ray::new(
                        Vec3::new(
                            b.min.x + extent.x * (u + 0.5) / n,
                            b.min.y + extent.y * (v + 0.5) / n,
                            b.min.z - extent.z - 1.0,
                        ),
                        Vec3::Z,
                    ),
                    // Exactly on the integer lattice. Against `lattice_block`
                    // these pass through vertices and along edges.
                    "aligned" => Ray::new(
                        Vec3::new(
                            (b.min.x + extent.x * u / n).round(),
                            (b.min.y + extent.y * v / n).round(),
                            b.min.z - extent.z - 1.0,
                        ),
                        Vec3::Z,
                    ),
                    // Oblique, to exercise the general-position path and the
                    // slab test's non-zero components.
                    _ => Ray::new(
                        Vec3::new(
                            b.min.x + extent.x * (u + 0.5) / n,
                            b.min.y - extent.y - 1.0,
                            b.min.z + extent.z * (v + 0.5) / n,
                        ),
                        Vec3::new(0.3, 1.0, 0.17),
                    ),
                };
                cast += 1;
                if let Some(leak) = check_ray(mesh, &bvh, &ray, &mut stats) {
                    leaks.push(leak);
                }
            }
        }
    }
    (leaks, cast, stats)
}

fn assert_no_leaks(mesh: &TriMesh, label: &str) -> (u64, RayStats) {
    let (leaks, cast, stats) = sweep(mesh, label);
    assert!(
        leaks.is_empty(),
        "{label}: {} of {cast} rays leaked. First five:\n{}",
        leaks.len(),
        leaks
            .iter()
            .take(5)
            .map(|l| format!(
                "  origin {:?} dir {:?}: {} crossings, {}",
                l.origin, l.direction, l.crossings, l.reason
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
    assert!(cast >= 10_000, "{label}: only {cast} rays cast");
    (cast, stats)
}

#[test]
fn analytic_solids_never_leak() {
    let meshes: Vec<(&str, TriMesh)> = vec![
        ("cube", shapes::cube(10.0)),
        (
            "box",
            shapes::box_solid(Vec3::new(-3.0, -5.0, -7.0), Vec3::new(4.0, 6.0, 8.0)),
        ),
        ("icosphere/1", shapes::icosphere(5.0, 1)),
        ("icosphere/3", shapes::icosphere(5.0, 3)),
        ("cylinder/8", shapes::cylinder(4.0, 9.0, 8)),
        ("cylinder/64", shapes::cylinder(4.0, 9.0, 64)),
        ("cone/32", shapes::cone(4.0, 9.0, 32)),
        ("torus", shapes::torus(6.0, 2.0, 32, 16)),
    ];
    let mut total = 0u64;
    for (label, mesh) in &meshes {
        let (cast, _) = assert_no_leaks(mesh, label);
        total += cast;
    }
    eprintln!(
        "{total} rays cast across {} analytic solids, zero leaks",
        meshes.len()
    );
}

#[test]
fn the_lattice_aligned_adversarial_mesh_never_leaks() {
    // The case built specifically to break a naive tie-break: every vertex is on
    // an integer coordinate, and the "aligned" lattice sends rays straight
    // through vertices and along edges.
    for n in [1u32, 2, 3, 5] {
        let mesh = shapes::lattice_block(n);
        let label = format!("lattice_block/{n}");
        let (cast, stats) = assert_no_leaks(&mesh, &label);
        assert!(
            stats.sos_resolutions > 0,
            "{label}: no degenerate edge functions occurred, so this mesh is not \
             actually adversarial and the test proves nothing"
        );
        eprintln!(
            "{label}: {cast} rays, {} triangle tests, {:.2}% exact path, \
             {} SoS resolutions, {} coplanar rejects",
            stats.triangle_tests,
            stats.exact_fraction() * 100.0,
            stats.sos_resolutions,
            stats.coplanar_rejected
        );
    }
}

#[test]
fn nested_shells_keep_depth_non_negative() {
    // Two concentric spheres. The crossing sequence is enter, enter, exit, exit
    // rather than alternating, which is why U5 must count depth.
    let outer = shapes::icosphere(10.0, 2);
    let inner = shapes::icosphere(4.0, 2);
    let mut vertices = outer.vertices().to_vec();
    let mut triangles = outer.triangles().to_vec();
    let offset = vertices.len() as u32;
    vertices.extend_from_slice(inner.vertices());
    triangles.extend(
        inner
            .triangles()
            .iter()
            .map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]),
    );
    let mesh = TriMesh::new(vertices, triangles, outer.meta().clone()).expect("valid");
    assert_no_leaks(&mesh, "nested spheres");
}

#[test]
fn a_ray_along_an_edge_of_the_lattice_block_is_handled() {
    // The most degenerate configuration the SoS cascade has to survive: the ray
    // line coincides with a mesh edge, so both perturbation coefficients vanish
    // and the pathological index fallback decides.
    let mesh = shapes::lattice_block(2);
    let bvh = Bvh::build(&mesh);
    let mut leaks = 0;
    // Every vertical edge of the block runs along x, y integer.
    for x in 0..=2 {
        for y in 0..=2 {
            let ray = Ray::new(Vec3::new(f64::from(x), f64::from(y), -5.0), Vec3::Z);
            let mut stats = RayStats::default();
            if let Some(l) = check_ray(&mesh, &bvh, &ray, &mut stats) {
                eprintln!("edge ray ({x},{y}) leaked: {}", l.reason);
                leaks += 1;
            }
        }
    }
    assert_eq!(leaks, 0, "rays along mesh edges must not leak");
}

#[test]
fn the_exact_fallback_rate_is_reported_for_generic_and_adversarial_meshes() {
    // Not an assertion about performance so much as a measurement U5 and U9 need:
    // Unit 1 measured orient3d at roughly 17x the filtered path, so how often the
    // exact path fires sets the budget.
    let generic = shapes::icosphere(5.0, 3);
    let (_, generic_stats) = assert_no_leaks(&generic, "icosphere/3");
    let adversarial = shapes::lattice_block(4);
    let (_, adversarial_stats) = assert_no_leaks(&adversarial, "lattice_block/4");

    eprintln!(
        "exact-fallback rate: generic {:.4}%, lattice-aligned {:.4}%",
        generic_stats.exact_fraction() * 100.0,
        adversarial_stats.exact_fraction() * 100.0
    );
    assert!(
        adversarial_stats.exact_fraction() > generic_stats.exact_fraction(),
        "the adversarial mesh must take the exact path more often than a generic \
         one, or it is not adversarial: {:.6} vs {:.6}",
        adversarial_stats.exact_fraction(),
        generic_stats.exact_fraction()
    );
    // The generic case must stay cheap, or U5's budget is wrong.
    assert!(
        generic_stats.exact_fraction() < 0.05,
        "generic meshes should almost always take the fast path, got {:.4}%",
        generic_stats.exact_fraction() * 100.0
    );
}
