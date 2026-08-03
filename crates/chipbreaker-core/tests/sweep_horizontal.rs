// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Case A against the dense reference.
//!
//! Two independent computations of the same geometry: the three-piece
//! decomposition, and sub-stepping the static tool until it converges. The
//! reference is a **subset** of the true sweep and converges upward, so the
//! analytic answer must contain it and the gap must close as the step count
//! grows. A gap that does not close is a bug in the decomposition; an analytic
//! result that fails to contain the reference is a bug too, and a worse one.

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::spans::Spans;
use chipbreaker_core::sweep::{LinearMove, horizontal, reference};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};

fn flat(diameter: f64) -> Profile {
    flat_end_mill(diameter, 18.0, &Shank::plain(diameter, 45.0)).expect("valid")
}

fn ball(diameter: f64) -> Profile {
    ball_end_mill(diameter, 18.0, &Shank::plain(diameter, 45.0)).expect("valid")
}

/// Rays from three bundles over a region, so every orientation is exercised.
fn probe_rays() -> Vec<Ray> {
    let mut rays = Vec::new();
    let n = 9;
    for i in 0..n {
        for j in 0..n {
            let a = -12.0 + 24.0 * f64::from(i) / f64::from(n - 1);
            let b = -12.0 + 24.0 * f64::from(j) / f64::from(n - 1);
            // Deliberately off any round number, so a ray never lands exactly on
            // a tool axis or a motion line by accident and the test measures
            // ordinary geometry rather than a pile of tangencies.
            let a = a + 0.137;
            let b = b + 0.041;
            rays.push(Ray {
                origin: Vec3::new(a, b, -20.0),
                direction: Vec3::new(0.0, 0.0, 1.0),
            });
            rays.push(Ray {
                origin: Vec3::new(-30.0, a, b + 9.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
            });
            rays.push(Ray {
                origin: Vec3::new(a, -30.0, b + 9.0),
                direction: Vec3::new(0.0, 1.0, 0.0),
            });
        }
    }
    rays
}

/// Total length of `a` not covered by `b`.
fn uncovered(a: &Spans, b: &Spans) -> f64 {
    a.subtract(b).measure()
}

#[test]
fn the_three_piece_decomposition_matches_the_dense_reference() {
    let cases: [(&str, Profile, LinearMove); 4] = [
        (
            "flat mill along X",
            flat(6.0),
            LinearMove {
                start: Vec3::new(-8.0, 0.0, 0.0),
                end: Vec3::new(8.0, 0.0, 0.0),
            },
        ),
        (
            "flat mill diagonally",
            flat(6.0),
            LinearMove {
                start: Vec3::new(-7.0, -5.0, 1.0),
                end: Vec3::new(6.0, 4.0, 1.0),
            },
        ),
        (
            "ball mill along Y",
            ball(8.0),
            LinearMove {
                start: Vec3::new(0.0, -9.0, 2.0),
                end: Vec3::new(0.0, 7.0, 2.0),
            },
        ),
        (
            "ball mill on an awkward bearing",
            ball(8.0),
            LinearMove {
                start: Vec3::new(-6.3, -4.1, -1.5),
                end: Vec3::new(5.7, 6.9, -1.5),
            },
        ),
    ];

    for (name, profile, motion) in &cases {
        let rays = probe_rays();
        let mut worst_missing = 0.0f64;
        let mut worst_extra = 0.0f64;
        let mut compared = 0u32;

        for ray in &rays {
            let analytic = horizontal::swept_spans(profile, motion, ray);
            let dense = reference::swept_spans(profile, motion, 512, ray);
            if analytic.is_empty() && dense.is_empty() {
                continue;
            }
            compared += 1;
            // The reference is a subset of the truth, so anything it found the
            // analytic answer must also have found. This direction catches a
            // decomposition that has LOST a piece.
            worst_missing = worst_missing.max(uncovered(&dense, &analytic));
            // And the other direction bounds how much the analytic answer has
            // that the reference has not yet reached. It must be small and must
            // shrink with the step count, which the next test checks.
            worst_extra = worst_extra.max(uncovered(&analytic, &dense));
        }

        assert!(compared > 20, "{name}: only {compared} rays met the sweep");
        assert!(
            worst_missing < 1.0e-9,
            "{name}: the reference found {worst_missing} mm of material the \
             three-piece decomposition missed. A piece of the sweep is lost."
        );
        assert!(
            worst_extra < 0.05,
            "{name}: the analytic result exceeds a 512-step reference by \
             {worst_extra} mm, which is far more than sub-stepping should leave"
        );
    }
}

#[test]
fn the_reference_converges_upward_to_the_analytic_answer() {
    // The gap must CLOSE as the reference refines. If it plateaus, the analytic
    // answer is wrong by a fixed amount and the first test's tolerance was
    // simply generous enough to hide it.
    let profile = flat(6.0);
    let motion = LinearMove {
        start: Vec3::new(-7.0, -3.0, 0.5),
        end: Vec3::new(6.0, 5.0, 0.5),
    };
    let rays = probe_rays();

    let mut gaps = Vec::new();
    for steps in [4u32, 16, 64, 256] {
        let mut worst = 0.0f64;
        for ray in &rays {
            let analytic = horizontal::swept_spans(&profile, &motion, ray);
            let dense = reference::swept_spans(&profile, &motion, steps, ray);
            worst = worst.max(uncovered(&analytic, &dense));
            // At every step count the reference must remain a subset.
            assert!(
                uncovered(&dense, &analytic) < 1.0e-9,
                "at {steps} steps the reference found material the analytic \
                 answer missed"
            );
        }
        gaps.push((steps, worst));
    }

    for pair in gaps.windows(2) {
        assert!(
            pair[1].1 <= pair[0].1 + 1.0e-12,
            "the gap grew from {:?} to {:?}: refining the reference should only \
             ever close it",
            pair[0],
            pair[1]
        );
    }
    let (_, finest) = gaps.last().copied().expect("measured");
    assert!(
        finest < 0.02,
        "the gap plateaued at {finest} mm, so the analytic answer differs from \
         the truth by a fixed amount rather than by sampling: {gaps:?}"
    );
}

#[test]
fn a_ray_running_along_the_motion_is_handled_rather_than_dropped() {
    // The degenerate case that is not rare: an X bundle meeting a move along X
    // takes it on every single ray, because the mapped cross-section direction
    // collapses to a point.
    let profile = flat(6.0);
    let motion = LinearMove {
        start: Vec3::new(-8.0, 0.0, 0.0),
        end: Vec3::new(8.0, 0.0, 0.0),
    };
    // Straight down the motion line, at a height the tool occupies.
    let ray = Ray {
        origin: Vec3::new(-40.0, 0.31, 4.0),
        direction: Vec3::new(1.0, 0.0, 0.0),
    };
    let analytic = horizontal::swept_spans(&profile, &motion, &ray);
    let dense = reference::swept_spans(&profile, &motion, 2048, &ray);

    assert!(!analytic.is_empty(), "the ray runs the length of the sweep");
    assert!(
        uncovered(&dense, &analytic) < 1.0e-9,
        "the along-motion ray lost material the reference found"
    );
    // The swept length here is the motion plus a tool diameter, since the ray
    // passes within the tool radius of the axis at both ends.
    let expected = 16.0 + 2.0 * (9.0f64 - 0.31 * 0.31).sqrt();
    assert!(
        (analytic.measure() - expected).abs() < 1.0e-9,
        "expected {expected} mm along the motion, got {}",
        analytic.measure()
    );
}

#[test]
fn a_ray_that_misses_the_sweep_entirely_returns_nothing() {
    let profile = flat(6.0);
    let motion = LinearMove {
        start: Vec3::new(-8.0, 0.0, 0.0),
        end: Vec3::new(8.0, 0.0, 0.0),
    };
    // Well outside the swept radius, and above the top of the tool.
    for ray in [
        Ray {
            origin: Vec3::new(0.0, 40.0, -20.0),
            direction: Vec3::new(0.0, 0.0, 1.0),
        },
        Ray {
            origin: Vec3::new(-40.0, 0.0, 100.0),
            direction: Vec3::new(1.0, 0.0, 0.0),
        },
    ] {
        assert!(
            horizontal::swept_spans(&profile, &motion, &ray).is_empty(),
            "a ray clear of the sweep must return nothing"
        );
    }
}

#[test]
fn a_tangential_graze_removes_nothing_rather_than_a_sliver() {
    // Unit 3's EPS_TANGENT policy meeting real motion. A ray that touches the
    // swept surface exactly must produce no interval at all: a sliver of a
    // nanometre, multiplied across millions of rays, is what shows up later as
    // a visible artefact on a wall.
    let profile = flat(6.0);
    let motion = LinearMove {
        start: Vec3::new(-8.0, 0.0, 2.0),
        end: Vec3::new(8.0, 0.0, 2.0),
    };
    // Exactly the swept radius away, so the ray grazes the flank.
    let ray = Ray {
        origin: Vec3::new(0.0, 3.0, -20.0),
        direction: Vec3::new(0.0, 0.0, 1.0),
    };
    let spans = horizontal::swept_spans(&profile, &motion, &ray);
    assert!(
        spans.measure() < 1.0e-9,
        "a tangential graze produced {} mm of material: {spans:?}",
        spans.measure()
    );
}

#[test]
fn the_analytic_cut_path_agrees_with_the_reference_cut_path() {
    // The decomposition is exercised through `cut`, not just through the span
    // function, because that is how the product uses it. Both paths cut the same
    // stock with the same motion and must land within sampling of each other.
    use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
    use chipbreaker_core::mesh::shapes;
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri};

    let stock = || {
        TriDexelField::build(
            &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
            &TriBuildOptions {
                spacing: 0.5,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds")
        .0
    };

    let profile = flat(6.0);
    let motion = LinearMove {
        start: Vec3::new(3.0, 7.0, 4.0),
        end: Vec3::new(35.0, 22.0, 4.0),
    };

    let mut analytic = stock();
    let mut scratch = CutScratch::new(&profile);
    let a = cut_tri(
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
        SweepMethod::Reference { steps: 1024 },
        &mut scratch,
    );

    // Exact, so no sub-steps were taken at all.
    assert_eq!(a.substeps, 0, "a horizontal move must not sub-step");
    assert_eq!(
        a.worst_bound_mm, 0.0,
        "an exact case has no deviation bound"
    );

    let relative = (analytic.volume() - dense.volume()).abs() / dense.volume();
    assert!(
        relative < 1.0e-4,
        "analytic {} against a 1024-step reference {}: {relative:e} apart",
        analytic.volume(),
        dense.volume()
    );
    // And the analytic path removes at least as much, since the reference
    // under-reports by construction.
    assert!(
        analytic.volume() <= dense.volume() + 1.0e-9,
        "the analytic cut removed less than a reference that under-reports"
    );
}
