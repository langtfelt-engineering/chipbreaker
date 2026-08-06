// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Case A′ against a dense reference.
//!
//! The reference sub-steps the arc: the tool at many positions along the sweep,
//! unioned. It is a **subset** of the true swept volume and converges upward, so
//! the analytic answer must contain it and the gap must close as the step count
//! grows. Same discipline as Unit 7's linear cases, and it is what will catch a
//! wedge that clips too much or a seam that leaves a sliver.

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::spans::Spans;
use chipbreaker_core::sweep::arc::{ArcMove, swept_spans_into};
use chipbreaker_core::sweep::{plunge, spans_in_tool_at};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};

const PI: f64 = core::f64::consts::PI;

fn flat(d: f64) -> Profile {
    flat_end_mill(d, 18.0, &Shank::plain(d, 45.0)).expect("valid")
}
fn ball(d: f64) -> Profile {
    ball_end_mill(d, 18.0, &Shank::plain(d, 45.0)).expect("valid")
}

/// The dense reference: the static tool at many positions along the sweep.
fn reference(profile: &Profile, arc: &ArcMove, steps: u32, ray: &Ray) -> Spans {
    let mut scratch = RaycastScratch::default();
    let mut stats = RaycastStats::default();
    let mut out = Spans::new();
    let mut piece = Spans::new();
    let mut merged = Spans::new();
    for k in 0..=steps {
        let s = f64::from(k) / f64::from(steps);
        spans_in_tool_at(
            profile,
            arc.at(s),
            ray,
            &mut scratch,
            &mut piece,
            &mut stats,
        );
        if !piece.is_empty() {
            out.union_into(&piece, &mut merged);
            core::mem::swap(&mut out, &mut merged);
        }
    }
    out
}

fn analytic(profile: &Profile, arc: &ArcMove, ray: &Ray) -> Spans {
    let mut out = Spans::new();
    let handled = swept_spans_into(
        profile,
        arc,
        ray,
        plunge::is_radially_convex(profile),
        &mut RaycastScratch::default(),
        &mut out,
        &mut RaycastStats::default(),
    );
    assert!(handled, "an axis ray must be handled");
    out
}

/// Rays from all three bundles over the swept region.
fn probe_rays() -> Vec<Ray> {
    let mut rays = Vec::new();
    let n = 13;
    for i in 0..n {
        for j in 0..n {
            let a = -18.0 + 36.0 * f64::from(i) / f64::from(n - 1) + 0.137;
            let b = -18.0 + 36.0 * f64::from(j) / f64::from(n - 1) + 0.041;
            rays.push(Ray {
                origin: Vec3::new(a, b, -20.0),
                direction: Vec3::new(0.0, 0.0, 1.0),
            });
            rays.push(Ray {
                origin: Vec3::new(-40.0, a, b.abs() * 0.4 + 1.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
            });
            rays.push(Ray {
                origin: Vec3::new(a, -40.0, b.abs() * 0.4 + 1.0),
                direction: Vec3::new(0.0, 1.0, 0.0),
            });
        }
    }
    rays
}

fn uncovered(a: &Spans, b: &Spans) -> f64 {
    a.subtract(b).measure()
}

fn arc_at(radius: f64, start: f64, sweep: f64) -> ArcMove {
    ArcMove {
        center: Vec3::new(1.5, -2.25, 0.0),
        radius,
        start_angle: start,
        sweep,
        z: 0.0,
        rise: 0.0,
        plane: chipbreaker_core::toolpath::ArcPlane::Xy,
    }
}

#[test]
fn horizontal_arcs_match_the_dense_reference() {
    let cases: [(&str, Profile, ArcMove); 6] = [
        (
            "quarter turn, flat 6",
            flat(6.0),
            arc_at(10.0, 0.3, PI / 2.0),
        ),
        ("clockwise, flat 6", flat(6.0), arc_at(10.0, 2.0, -1.7)),
        ("major arc, ball 8", ball(8.0), arc_at(9.0, -0.4, 4.5)),
        (
            "full circle, flat 6",
            flat(6.0),
            arc_at(11.0, 0.7, 2.0 * PI),
        ),
        ("two turns, ball 8", ball(8.0), arc_at(8.0, 0.1, 4.0 * PI)),
        // The tight sub-case: the arc radius is under the tool radius, so the
        // annulus reaches the axis and becomes disc-like.
        ("tight arc, flat 12", flat(12.0), arc_at(2.5, 0.2, 2.4)),
    ];

    for (name, profile, arc) in &cases {
        let mut worst_missing = 0.0f64;
        let mut worst_extra = 0.0f64;
        let mut compared = 0u32;
        for ray in probe_rays() {
            let a = analytic(profile, arc, &ray);
            let d = reference(profile, arc, 900, &ray);
            if a.is_empty() && d.is_empty() {
                continue;
            }
            compared += 1;
            worst_missing = worst_missing.max(uncovered(&d, &a));
            worst_extra = worst_extra.max(uncovered(&a, &d));
        }
        assert!(compared > 40, "{name}: only {compared} rays met the sweep");
        assert!(
            worst_missing < 1.0e-6,
            "{name}: the reference found {worst_missing} mm the decomposition missed. \
             Either the wedge clips too much or a piece is lost."
        );
        assert!(
            worst_extra < 0.05,
            "{name}: the decomposition exceeds a 900-step reference by {worst_extra} mm"
        );
    }
}

#[test]
fn the_reference_converges_upward_to_the_analytic_answer() {
    // If the gap plateaus, the decomposition is wrong by a fixed amount and the
    // tolerance above was merely generous.
    let profile = flat(6.0);
    let arc = arc_at(10.0, 0.4, 2.2);
    let rays = probe_rays();

    let mut gaps = Vec::new();
    for steps in [8u32, 32, 128, 512] {
        let mut worst = 0.0f64;
        for ray in &rays {
            let a = analytic(&profile, &arc, ray);
            let d = reference(&profile, &arc, steps, ray);
            worst = worst.max(uncovered(&a, &d));
            assert!(
                uncovered(&d, &a) < 1.0e-6,
                "at {steps} steps the reference found material the decomposition missed"
            );
        }
        gaps.push((steps, worst));
    }
    for pair in gaps.windows(2) {
        assert!(
            pair[1].1 <= pair[0].1 + 1.0e-12,
            "the gap grew from {:?} to {:?}",
            pair[0],
            pair[1]
        );
    }
    let (_, finest) = gaps.last().copied().expect("measured");
    assert!(
        finest < 0.02,
        "the gap plateaued at {finest} mm, so the decomposition is off by a fixed \
         amount rather than by sampling: {gaps:?}"
    );
}

#[test]
fn a_full_circle_is_not_deleted_by_its_zero_chord() {
    // Unit 4 found that a full circle has start equal to end, so its chord is
    // zero. A handler that dispatches on the chord treats it as no motion and
    // removes nothing at all.
    let profile = flat(6.0);
    let full = arc_at(10.0, 0.7, 2.0 * PI);
    assert!(
        (full.at(0.0) - full.at(1.0)).length() < 1.0e-9,
        "a full circle's start and end must coincide, or this tests nothing"
    );

    // A ray through the ring, well away from the start bearing.
    let ray = Ray {
        origin: Vec3::new(1.5 - 10.0, -2.25, -20.0),
        direction: Vec3::new(0.0, 0.0, 1.0),
    };
    let spans = analytic(&profile, &full, &ray);
    assert!(
        spans.measure() > 1.0,
        "a full circle must sweep the whole ring; got {} mm",
        spans.measure()
    );
    // And the wedge test is vacuous for a full turn, so every bearing is in.
    for bearing in [0.0, 1.0, 3.0, -2.0, 6.0] {
        assert!(
            full.wedge_contains(bearing),
            "bearing {bearing} should be swept"
        );
    }
}

#[test]
fn a_multi_turn_helix_sweeps_the_same_ring_as_one_turn() {
    // Beyond 2 PI the wedge is vacuous, so two turns and one turn sweep the same
    // set. Anything else means the sweep sign or the wrap is being mishandled.
    let profile = flat(6.0);
    let one = arc_at(10.0, 0.25, 2.0 * PI);
    let two = arc_at(10.0, 0.25, 4.0 * PI);
    let back = arc_at(10.0, 0.25, -4.0 * PI);
    for ray in probe_rays().into_iter().take(60) {
        let a = analytic(&profile, &one, &ray);
        let b = analytic(&profile, &two, &ray);
        let c = analytic(&profile, &back, &ray);
        assert_eq!(a.as_slice(), b.as_slice(), "two turns must equal one");
        assert_eq!(
            a.as_slice(),
            c.as_slice(),
            "and so must two turns backwards"
        );
    }
}

#[test]
fn the_tight_arc_sub_case_reaches_the_axis() {
    // `R <= max rho`: the inner radius goes negative, so the annulus closes into
    // a disc and the centre is inside the swept volume.
    let profile = flat(12.0);
    let arc = arc_at(2.5, 0.0, 2.0 * PI);
    assert!(
        arc.radius < profile.max_radius(),
        "this case needs the arc radius under the tool radius"
    );
    // A vertical ray straight down the arc's own axis.
    let ray = Ray {
        origin: Vec3::new(1.5, -2.25, -20.0),
        direction: Vec3::new(0.0, 0.0, 1.0),
    };
    let spans = analytic(&profile, &arc, &ray);
    assert!(
        spans.measure() > 1.0,
        "the centre of a tight arc must be inside the sweep; got {} mm",
        spans.measure()
    );
    // And it agrees with the reference there.
    let dense = reference(&profile, &arc, 900, &ray);
    assert!(
        uncovered(&dense, &spans) < 1.0e-9,
        "the tight arc lost material the reference found"
    );
}

#[test]
fn a_tangential_arc_removes_nothing() {
    // Curved tangential contact touches at a point rather than along a line, so
    // any interval at all would be spurious. This is the failure that
    // accumulates into a visible artefact across millions of rays.
    let profile = flat(6.0);
    let arc = arc_at(10.0, 0.0, 2.0 * PI);
    // Exactly the outer swept radius away, in plan, on a horizontal ray that
    // grazes the ring's outside.
    let outer = 10.0 + 3.0;
    let ray = Ray {
        origin: Vec3::new(-40.0, -2.25 + outer, 4.0),
        direction: Vec3::new(1.0, 0.0, 0.0),
    };
    let spans = analytic(&profile, &arc, &ray);
    assert!(
        spans.measure() < 1.0e-9,
        "a tangential graze produced {} mm: {spans:?}",
        spans.measure()
    );
}

#[test]
fn a_ray_clear_of_the_ring_returns_nothing() {
    let profile = flat(6.0);
    let arc = arc_at(10.0, 0.0, PI);
    for ray in [
        // Outside the outer radius.
        Ray {
            origin: Vec3::new(1.5 + 40.0, -2.25, -20.0),
            direction: Vec3::new(0.0, 0.0, 1.0),
        },
        // Inside the hole, where a full annulus has no material.
        Ray {
            origin: Vec3::new(1.5, -2.25, -20.0),
            direction: Vec3::new(0.0, 0.0, 1.0),
        },
        // Above the top of the tool.
        Ray {
            origin: Vec3::new(-40.0, -2.25, 200.0),
            direction: Vec3::new(1.0, 0.0, 0.0),
        },
    ] {
        assert!(
            analytic(&profile, &arc, &ray).is_empty(),
            "a ray clear of the swept ring must return nothing"
        );
    }
}

// --- Case B': helices -------------------------------------------------------

fn helix(radius: f64, start: f64, sweep: f64, rise: f64) -> ArcMove {
    ArcMove {
        center: Vec3::new(1.5, -2.25, 0.0),
        radius,
        start_angle: start,
        sweep,
        z: 0.0,
        rise,
        plane: chipbreaker_core::toolpath::ArcPlane::Xy,
    }
}

#[test]
fn the_helix_bound_is_conservative_against_the_true_path() {
    // The bound claims: no point of the true path is farther than
    // `deviation_bound(N)` from the nearest of N+1 evenly spaced samples.
    //
    // The tool translates rigidly along the path, so the tip's deviation IS the
    // swept volume's deviation -- which is why measuring the path is enough and
    // is far more direct than measuring the volume.
    for arc in [
        helix(10.0, 0.3, 2.4, 6.0),
        helix(4.0, -1.0, -3.1, -2.5),
        helix(12.0, 0.0, 4.0 * PI, 9.0),
        helix(0.5, 0.7, 1.2, 8.0),
    ] {
        for steps in [4u32, 16, 64, 256] {
            let bound = arc.deviation_bound(steps);
            // Walk the path finely and find the worst distance to a sample.
            let mut worst = 0.0f64;
            let fine = steps * 97;
            for k in 0..=fine {
                let s = f64::from(k) / f64::from(fine);
                let p = arc.at(s);
                let mut nearest = f64::INFINITY;
                for j in 0..=steps {
                    let q = arc.at(f64::from(j) / f64::from(steps));
                    nearest = nearest.min((p - q).length());
                }
                worst = worst.max(nearest);
            }
            assert!(
                worst <= bound * (1.0 + 1.0e-9),
                "R={} sweep={} rise={} at {steps} steps: measured deviation {worst} \
                 exceeds the claimed bound {bound}. The bound is unsound.",
                arc.radius,
                arc.sweep,
                arc.rise
            );
            // And not absurdly slack: the bound is the midpoint distance, which
            // the fine walk should very nearly reach.
            assert!(
                worst >= bound * 0.9,
                "R={} at {steps} steps: bound {bound} is {:.1}x the measured {worst}, \
                 which is looser than the derivation should allow",
                bound / worst.max(1.0e-300),
                arc.radius
            );
        }
    }
}

#[test]
fn a_chord_based_bound_would_be_unsound_for_a_helix() {
    // Why the bound uses the helical path length rather than the chord. On an
    // ordinary helix the chord under-states the path by a fifth, so a bound
    // derived from it would claim an accuracy the sweep does not have.
    let arc = helix(10.0, 0.0, 2.4, 6.0);
    let chord = (arc.at(1.0) - arc.at(0.0)).length();
    let path = arc.path_length();
    assert!(
        chord < path * 0.85,
        "this case is meant to have a chord well under the path: {chord} against {path}"
    );
    for steps in [8u32, 32] {
        let honest = arc.deviation_bound(steps);
        let from_chord = chord / (2.0 * f64::from(steps));
        assert!(
            from_chord < honest,
            "a chord-based bound {from_chord} should be OPTIMISTIC against the true \
             {honest}, which is exactly why it must not be used"
        );
    }
}

#[test]
fn a_helix_is_declined_by_the_closed_form_and_falls_through() {
    // Case A′ collapses only when the axial term is absent. A helix must say so
    // rather than return the level answer.
    let profile = flat(6.0);
    let level = helix(10.0, 0.2, 1.5, 0.0);
    let rising = helix(10.0, 0.2, 1.5, 5.0);
    assert!(!level.is_helix());
    assert!(rising.is_helix());

    let ray = Ray {
        origin: Vec3::new(1.5 + 10.0, -2.25, -20.0),
        direction: Vec3::new(0.0, 0.0, 1.0),
    };
    let mut out = Spans::new();
    let convex = plunge::is_radially_convex(&profile);
    assert!(
        swept_spans_into(
            &profile,
            &level,
            &ray,
            convex,
            &mut RaycastScratch::default(),
            &mut out,
            &mut RaycastStats::default()
        ),
        "a level arc is closed form"
    );
    assert!(
        !swept_spans_into(
            &profile,
            &rising,
            &ray,
            convex,
            &mut RaycastScratch::default(),
            &mut out,
            &mut RaycastStats::default()
        ),
        "a helix must be declined, not answered with the level result"
    );
}

#[test]
fn a_zero_radius_helix_is_a_plunge() {
    // The degenerate case: with no radius the path is a straight descent, so the
    // swept volume must match Case B's plunge exactly.
    use chipbreaker_core::sweep::LinearMove;
    let profile = ball(6.0);
    let spin = helix(0.0, 0.0, 4.0 * PI, -8.0);
    let drop = LinearMove {
        start: Vec3::new(1.5, -2.25, 0.0),
        end: Vec3::new(1.5, -2.25, -8.0),
    };

    for ray in probe_rays().into_iter().take(90) {
        let mut helical = Spans::new();
        chipbreaker_core::sweep::reference::arc_spans_into(
            &profile,
            &spin,
            64,
            &ray,
            &mut RaycastScratch::default(),
            &mut helical,
            &mut RaycastStats::default(),
        );
        let mut straight = Spans::new();
        assert!(plunge::swept_spans_into(
            &profile,
            &drop,
            &ray,
            plunge::is_radially_convex(&profile),
            &mut RaycastScratch::default(),
            &mut straight,
            &mut RaycastStats::default(),
        ));
        assert!(
            helical.subtract(&straight).measure() < 1.0e-9,
            "a zero-radius helix found material the plunge did not: {helical:?} vs \
             {straight:?}"
        );
    }
}
