// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Case B against the dense reference, and the moving maximum on its own.
//!
//! The specification warned that sweeping a profile takes the **upper envelope**
//! of its radius over the window, not a translation of the chain, and that this
//! is where a subtle error would hide. So the envelope is tested directly on a
//! tool that would expose the difference, as well as through the sweep.

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::spans::Spans;
use chipbreaker_core::sweep::{LinearMove, plunge, reference};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, bull_end_mill, drill, flat_end_mill};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};

fn shank(d: f64) -> Shank {
    Shank::plain(d, 60.0)
}

fn tools() -> Vec<(&'static str, Profile)> {
    vec![
        (
            "flat 6",
            flat_end_mill(6.0, 20.0, &shank(6.0)).expect("valid"),
        ),
        (
            "ball 8",
            ball_end_mill(8.0, 20.0, &shank(8.0)).expect("valid"),
        ),
        (
            "drill 6",
            drill(6.0, 118.0, 20.0, &shank(6.0)).expect("valid"),
        ),
        // The tool that separates a moving maximum from a chain translation: its
        // shank necks in below the cutting diameter, so its radius is NOT
        // monotonic in z.
        (
            "bull 10 r2 (necked shank)",
            bull_end_mill(10.0, 2.0, 16.0, &Shank::plain(8.0, 60.0)).expect("valid"),
        ),
    ]
}

fn probe_rays() -> Vec<Ray> {
    let mut rays = Vec::new();
    let n = 11;
    for i in 0..n {
        for j in 0..n {
            let a = -9.0 + 18.0 * f64::from(i) / f64::from(n - 1) + 0.137;
            let b = -9.0 + 18.0 * f64::from(j) / f64::from(n - 1) + 0.041;
            rays.push(Ray {
                origin: Vec3::new(a, b, -30.0),
                direction: Vec3::new(0.0, 0.0, 1.0),
            });
            rays.push(Ray {
                origin: Vec3::new(-40.0, a, b + 14.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
            });
            rays.push(Ray {
                origin: Vec3::new(a, -40.0, b + 14.0),
                direction: Vec3::new(0.0, 1.0, 0.0),
            });
        }
    }
    rays
}

fn uncovered(a: &Spans, b: &Spans) -> f64 {
    a.subtract(b).measure()
}

#[test]
fn the_moving_maximum_is_not_a_chain_translation() {
    // The claim the specification flagged. `bull-10-r2` has radii 0, 3, 5, 5, 4,
    // 4: it reaches 5 mm at the cutting edge and necks back to 4 mm on the
    // shank. Over a window spanning both, the swept radius must be the LARGER,
    // because the wide part passes through that height during the plunge.
    let profile = bull_end_mill(10.0, 2.0, 16.0, &Shank::plain(8.0, 60.0)).expect("valid");

    // A window covering only the necked shank, well above the cutter.
    let neck = plunge::max_radius_over_z(&profile, 30.0, 40.0);
    assert!(
        (neck - 4.0).abs() < 1.0e-9,
        "the shank alone should be 4 mm, got {neck}"
    );

    // A window reaching down to the cutting diameter must report 5, not 4.
    let spanning = plunge::max_radius_over_z(&profile, 10.0, 40.0);
    assert!(
        (spanning - 5.0).abs() < 1.0e-9,
        "a window spanning cutter and shank must take the MAXIMUM, 5 mm, not the \
         value at either end. Got {spanning}, which is what a chain translation \
         would give."
    );
    assert!(
        spanning > neck,
        "the envelope must exceed the shank radius over a spanning window"
    );
}

#[test]
fn the_moving_maximum_catches_an_arcs_interior_bulge() {
    // A ball nose: over a window inside the ball, the maximum is at the window's
    // upper edge, but over a window containing the ball's equator the maximum is
    // the full radius even though neither edge reaches it.
    let profile = ball_end_mill(8.0, 20.0, &shank(8.0)).expect("valid");
    // The ball's equator sits at z = 4 with r = 4.
    let across = plunge::max_radius_over_z(&profile, 3.0, 5.0);
    assert!(
        (across - 4.0).abs() < 1.0e-9,
        "a window straddling the ball's equator must reach the full 4 mm; got {across}"
    );
    let below = plunge::max_radius_over_z(&profile, 0.0, 1.0);
    assert!(
        below < 4.0 && below > 0.0,
        "a window low on the ball should be part way out, got {below}"
    );
}

#[test]
fn plunges_match_the_dense_reference() {
    let mut scratch = RaycastScratch::default();
    let mut stats = RaycastStats::default();

    for (name, profile) in tools() {
        assert!(
            plunge::is_radially_convex(&profile),
            "{name} should be radially convex"
        );
        // Once per profile, as the cut path does.
        let convex = plunge::is_radially_convex(&profile);
        for (label, motion) in [
            (
                "down",
                LinearMove {
                    start: Vec3::new(0.0, 0.0, 12.0),
                    end: Vec3::new(0.0, 0.0, -3.0),
                },
            ),
            (
                "up (a retract)",
                LinearMove {
                    start: Vec3::new(0.0, 0.0, -3.0),
                    end: Vec3::new(0.0, 0.0, 12.0),
                },
            ),
            (
                "off the origin",
                LinearMove {
                    start: Vec3::new(1.5, -2.25, 9.0),
                    end: Vec3::new(1.5, -2.25, 0.5),
                },
            ),
        ] {
            let mut worst_missing = 0.0f64;
            let mut worst_extra = 0.0f64;
            let mut compared = 0u32;
            let mut analytic = Spans::new();

            for ray in probe_rays() {
                let handled = plunge::swept_spans_into(
                    &profile,
                    &motion,
                    &ray,
                    convex,
                    &mut scratch,
                    &mut analytic,
                    &mut stats,
                );
                assert!(handled, "{name}/{label}: an axis ray must be handled");
                let dense = reference::swept_spans(&profile, &motion, 2048, &ray);
                if analytic.is_empty() && dense.is_empty() {
                    continue;
                }
                compared += 1;
                worst_missing = worst_missing.max(uncovered(&dense, &analytic));
                worst_extra = worst_extra.max(uncovered(&analytic, &dense));
            }

            assert!(compared > 20, "{name}/{label}: only {compared} rays met it");
            assert!(
                worst_missing < 1.0e-6,
                "{name}/{label}: the reference found {worst_missing} mm the \
                 envelope missed. The moving maximum is under-reporting."
            );
            assert!(
                worst_extra < 0.02,
                "{name}/{label}: the envelope exceeds a 2048-step reference by \
                 {worst_extra} mm"
            );
        }
    }
}

#[test]
fn a_plunge_removes_the_same_material_through_the_cut_path() {
    use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
    use chipbreaker_core::mesh::shapes;
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri};

    let stock = || {
        TriDexelField::build(
            &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 30.0, 12.0)),
            &TriBuildOptions {
                spacing_xyz: None,
                spacing: 0.4,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds")
        .0
    };

    let profile = drill(6.0, 118.0, 25.0, &shank(6.0)).expect("valid");
    let motion = LinearMove {
        start: Vec3::new(15.0, 15.0, 13.0),
        end: Vec3::new(15.0, 15.0, -1.0),
    };

    let mut analytic = stock();
    let mut scratch = CutScratch::new(&profile);
    let stats = cut_tri(
        &mut analytic,
        &profile,
        &motion,
        SweepMethod::Analytic { tolerance: 1.0e-3 },
        &mut scratch,
    );

    let mut dense = stock();
    let mut scratch = CutScratch::new(&profile);
    cut_tri(
        &mut dense,
        &profile,
        &motion,
        SweepMethod::Reference { steps: 2048 },
        &mut scratch,
    );

    assert_eq!(stats.substeps, 0, "a plunge must not sub-step");
    let relative = (analytic.volume() - dense.volume()).abs() / dense.volume();
    assert!(
        relative < 1.0e-5,
        "analytic {} against reference {}: {relative:e} apart",
        analytic.volume(),
        dense.volume()
    );
}

#[test]
fn a_ray_that_is_neither_along_nor_across_is_declined_rather_than_guessed() {
    // The honest failure. A diagonal ray does not occur in a tri-dexel field,
    // but the function must say so rather than return a plausible wrong answer,
    // because the caller's fallback depends on being told.
    let profile = flat_end_mill(6.0, 20.0, &shank(6.0)).expect("valid");
    let motion = LinearMove {
        start: Vec3::new(0.0, 0.0, 10.0),
        end: Vec3::new(0.0, 0.0, 0.0),
    };
    let diagonal = Ray {
        origin: Vec3::new(-20.0, 0.0, -20.0),
        direction: Vec3::new(0.6, 0.0, 0.8),
    };
    let mut out = Spans::new();
    let handled = plunge::swept_spans_into(
        &profile,
        &motion,
        &diagonal,
        plunge::is_radially_convex(&profile),
        &mut RaycastScratch::default(),
        &mut out,
        &mut RaycastStats::default(),
    );
    assert!(!handled, "a diagonal ray must be declined, not guessed at");
}
