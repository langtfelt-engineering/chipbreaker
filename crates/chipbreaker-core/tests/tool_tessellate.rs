// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Tessellation, checked against the closed forms and against Unit 2's mesh
//! ray caster.
//!
//! # The differential test is the point
//!
//! Sections 6 and 8 are two independent descriptions of the same solid: a
//! closed-form volume, and a ray caster that works from the profile directly.
//! Unit 2 is a third, entirely separate one: a BVH over triangles, using exact
//! predicates and Simulation of Simplicity. Tessellating the tool and casting
//! the same rays through both is the strongest check available on either, and it
//! is a check neither could perform alone — the two implementations share no
//! code, no data structure, and no numerical approach.
//!
//! Where they must agree is bounded by the tessellation tolerance, and the test
//! says so explicitly rather than picking a tolerance that happens to pass.

use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::validate::validate;
use chipbreaker_core::tool::catalog::{
    Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill, flat_end_mill,
};
use chipbreaker_core::tool::profile::Profile;
use chipbreaker_core::tool::tessellate::{MIN_ANGULAR_DIVISIONS, angular_divisions, arc_chords};
use chipbreaker_core::transcendental as t;

fn corpus() -> Vec<(&'static str, Profile)> {
    let shank = Shank::plain(6.0, 40.0);
    vec![
        ("flat", flat_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "flat-necked",
            flat_end_mill(10.0, 20.0, &Shank::plain(6.0, 40.0)).expect("valid"),
        ),
        ("ball", ball_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "bull",
            bull_end_mill(6.0, 1.5, 20.0, &shank).expect("valid"),
        ),
        (
            "chamfer",
            chamfer_mill(8.0, 1.0, 90.0, 20.0, &Shank::plain(8.0, 40.0)).expect("valid"),
        ),
        ("drill", drill(6.0, 118.0, 25.0, &shank).expect("valid")),
        (
            "barrel",
            barrel_end_mill(12.0, 60.0, 40.0, &Shank::plain(12.0, 60.0)).expect("valid"),
        ),
    ]
}

#[test]
fn the_subdivision_counts_follow_the_sagitta_formula() {
    // A chord subtending phi on a circle of radius rho deviates by
    // rho (1 - cos(phi/2)). Every count returned must actually satisfy that.
    for rho in [0.5f64, 1.0, 4.0, 60.0] {
        for tolerance in [1.0f64, 0.1, 0.01, 0.001] {
            let sweep = core::f64::consts::FRAC_PI_2;
            let n = arc_chords(rho, sweep, tolerance);
            let sagitta = rho * (1.0 - t::cos(sweep / (2.0 * n as f64)));
            assert!(
                sagitta <= tolerance * (1.0 + 1e-12),
                "rho {rho}, tolerance {tolerance}: {n} chords leave a sagitta of {sagitta}"
            );

            let m = angular_divisions(rho, tolerance);
            let angular = rho * (1.0 - t::cos(core::f64::consts::PI / m as f64));
            assert!(
                angular <= tolerance * (1.0 + 1e-12) || m == MIN_ANGULAR_DIVISIONS,
                "rho {rho}, tolerance {tolerance}: {m} divisions leave {angular}"
            );
        }
    }
}

#[test]
fn a_tolerance_that_is_not_a_length_is_refused() {
    let profile = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 40.0)).expect("valid");
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(profile.tessellate(bad).is_err(), "{bad} is not a tolerance");
    }
}

#[test]
fn every_tessellated_tool_is_a_closed_manifold_solid() {
    for (name, profile) in corpus() {
        let (mesh, report) = profile.tessellate(0.02).expect("valid tolerance");
        let check = validate(&mesh);
        assert!(
            check.is_solid(),
            "{name}: manifold {}, watertight {}, oriented {}, volume {} \
             (mesh from {} stations x {} divisions)",
            check.is_manifold,
            check.is_watertight,
            check.is_orientation_consistent,
            check.signed_volume,
            report.stations,
            report.divisions
        );
        assert!(
            check.fatal_findings().next().is_none(),
            "{name}: {:?}",
            check.findings
        );
        assert_eq!(
            check.components.len(),
            1,
            "{name}: a tool is one connected piece"
        );
    }
}

#[test]
fn the_mesh_is_inscribed_and_converges_to_the_closed_form_volume() {
    for (name, profile) in corpus() {
        let exact = profile.volume();
        let mut previous_error = f64::INFINITY;
        for tolerance in [0.2f64, 0.05, 0.0125] {
            let (mesh, _) = profile.tessellate(tolerance).expect("valid");
            let measured = mesh.signed_volume();
            assert!(
                measured <= exact * (1.0 + 1e-9),
                "{name} at tolerance {tolerance}: the mesh must be inscribed, but its \
                 volume {measured} exceeds the true {exact}"
            );
            let error = (exact - measured) / exact;
            assert!(
                error < previous_error,
                "{name}: tightening the tolerance to {tolerance} did not improve the \
                 volume ({error} vs {previous_error})"
            );
            previous_error = error;
        }
        assert!(
            previous_error < 0.01,
            "{name}: still {previous_error} short at the finest tolerance"
        );
    }
}

#[test]
fn tessellation_is_bit_identical_between_runs() {
    for (name, profile) in corpus() {
        let hash = |p: &Profile| {
            let (mesh, _) = p.tessellate(0.05).expect("valid");
            let mut h = CanonicalHash::new();
            h.add(&mesh);
            h.finish().to_hex()
        };
        assert_eq!(hash(&profile), hash(&profile), "{name}");
    }
}

#[test]
fn the_analytic_ray_caster_agrees_with_unit_twos_mesh_ray_caster() {
    // Two independent implementations of "where is this solid along this ray":
    // the profile-based intersection from section 8, and the BVH over triangles
    // from Unit 2, which shares none of its code.
    const TOLERANCE: f64 = 0.01;

    let mut compared = 0usize;
    let mut worst = 0.0f64;
    let mut total_difference = 0.0f64;
    let mut worst_case = String::new();

    for (name, profile) in corpus() {
        let (mesh, _) = profile.tessellate(TOLERANCE).expect("valid");
        let bvh = Bvh::build(&mesh);
        let cylinder = profile.bounding_cylinder();
        let radius = cylinder.radius * 1.2 + 1.0;

        let mut hits = Vec::new();
        for i in 0..20 {
            for j in 0..20 {
                // Cell centres, offset off the axis and off any exact station.
                let y = -radius + 2.0 * radius * (f64::from(i) + 0.5) / 20.0;
                let z = cylinder.z_min
                    + 0.013
                    + (cylinder.z_max - cylinder.z_min) * (f64::from(j) + 0.5) / 20.0;
                let ray =
                    Ray::new_normalized(Vec3::new(-radius - 1.0, y, z), Vec3::new(1.0, 0.0, 0.0))
                        .expect("unit");

                let analytic = profile.intersect_ray(&ray);
                let stats = bvh
                    .intersect_ray_all_into(&mesh, &ray, &mut hits)
                    .expect("the ray is well conditioned");
                let _ = stats;

                let mesh_measure: f64 = hits
                    .chunks_exact(2)
                    .map(|pair| (pair[1].t - pair[0].t).max(0.0))
                    .sum();
                let analytic_measure = analytic.measure();

                // Away from the silhouette the mesh is short of the solid by
                // O(tolerance) on each of the two crossings a span makes. Near
                // it the bound is a square root, not a linear one: the chord of
                // a circle is `2 sqrt(r^2 - y^2)`, so at `y` close to `r` a
                // radius reduced by `d` shortens the chord by about
                // `2 sqrt(2 r d)` — for a 3 mm cutter at a 0.01 mm tolerance
                // that is 0.49, fifty times the tolerance, and it is correct
                // behaviour rather than an error. Asserting a linear bound here
                // would be asserting something untrue about geometry.
                let radius = cylinder.radius;
                let slack = 4.0 * TOLERANCE * (analytic.len().max(1) as f64)
                    + 2.0 * (2.0 * radius * TOLERANCE).sqrt()
                    + 1e-9;

                let difference = (analytic_measure - mesh_measure).abs();
                if difference > worst {
                    worst = difference;
                    worst_case = format!(
                        "{name} at y {y:.3} z {z:.3}: analytic {analytic_measure:.6}, \
                         mesh {mesh_measure:.6}"
                    );
                }
                assert!(
                    difference <= slack,
                    "{name} at y {y:.3} z {z:.3}: analytic says {analytic_measure}, \
                     the tessellated mesh says {mesh_measure}, which is {difference} \
                     apart against a {slack} allowance for a {TOLERANCE} tessellation"
                );
                total_difference += difference;
                compared += 1;
            }
        }
    }
    assert!(compared > 2000, "only {compared} rays compared");
    // The sharp statement is about the average: the square-root blow-up above
    // applies only to the handful of rays within O(tolerance) of a silhouette,
    // so across the bundle the two casters agree to within a few tolerances.
    let mean = total_difference / compared as f64;
    eprintln!(
        "{compared} rays compared, mean difference {mean:.6}, worst {worst:.6} ({worst_case})"
    );
    assert!(
        mean < 8.0 * TOLERANCE,
        "the two ray casters differ by {mean} on average, well above the {TOLERANCE} \
         tessellation tolerance; that is a disagreement, not a discretisation"
    );
}

#[test]
fn a_finer_tessellation_brings_the_two_ray_casters_closer() {
    let profile = bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 40.0)).expect("valid");
    let mut previous = f64::INFINITY;
    for tolerance in [0.2f64, 0.05, 0.0125, 0.003] {
        let (mesh, _) = profile.tessellate(tolerance).expect("valid");
        let bvh = Bvh::build(&mesh);
        let mut hits = Vec::new();
        let mut total = 0.0f64;
        let mut rays = 0usize;

        for i in 0..40 {
            let y = -6.0 + 12.0 * (f64::from(i) + 0.5) / 40.0;
            let ray = Ray::new_normalized(Vec3::new(-50.0, y, 1.0), Vec3::new(1.0, 0.0, 0.0))
                .expect("unit");
            let analytic = profile.intersect_ray(&ray).measure();
            bvh.intersect_ray_all_into(&mesh, &ray, &mut hits)
                .expect("well conditioned");
            let meshed: f64 = hits
                .chunks_exact(2)
                .map(|pair| (pair[1].t - pair[0].t).max(0.0))
                .sum();
            total += (analytic - meshed).abs();
            rays += 1;
        }
        let mean = total / rays as f64;
        eprintln!("tolerance {tolerance:>7}: mean disagreement {mean:.6}");
        assert!(
            mean < previous,
            "tightening to {tolerance} did not bring them closer: {mean} vs {previous}"
        );
        previous = mean;
    }
    assert!(previous < 0.02, "still {previous} apart at the finest");
}
