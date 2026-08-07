// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Relations an arc cut must satisfy regardless of what the right answer is.
//!
//! # A mid-sweep split rounds twice, not once
//!
//! `sweep_metamorphic.rs` establishes that splitting a *linear* move mid-segment
//! cannot be bit-identical, because the split point is rounded and the second
//! piece's direction vector is therefore computed from different inputs.
//!
//! An arc is worse, and predictably so. Splitting at fraction `f` produces a
//! second piece whose start angle is `start + sweep * f` — a **rounded angle** —
//! and every point on that piece then comes out of `sin`/`cos` of it. So there
//! are two independent roundings where a line had one, and they do not commute:
//! the rounded angle enters a transcendental before it reaches a coordinate.
//!
//! The contract asserted here is therefore the same shape as the linear one but
//! with the tolerance measured rather than assumed, and stated in ULP.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::Motion;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri_motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;
use chipbreaker_core::transcendental as t;

const PI: f64 = core::f64::consts::PI;
const SPACING: f64 = 0.4;
const METHOD: SweepMethod = SweepMethod::Analytic {
    tolerance: SPACING / 10.0,
};

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill() -> Profile {
    flat_end_mill(6.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn ball() -> Profile {
    ball_end_mill(6.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

/// Cuts a list of motions into fresh stock, returning the field and the volume
/// removed.
fn cut(profile: &Profile, motions: &[Motion]) -> (TriDexelField, f64) {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let before = field.volume();
    for motion in motions {
        cut_tri_motion(&mut field, profile, motion, METHOD, &mut scratch);
    }
    let removed = before - field.volume();
    (field, removed)
}

/// Whether the two fields have the same span structure, and the worst endpoint
/// disagreement in ULP.
fn compare(a: &TriDexelField, b: &TriDexelField) -> (bool, i64) {
    let mut same_structure = true;
    let mut worst_ulp = 0i64;
    for axis in AXES {
        let (Some(x), Some(y)) = (a.bundle(axis), b.bundle(axis)) else {
            continue;
        };
        let rays = u32::try_from(x.arena().rays()).expect("small");
        for ray in 0..rays {
            let (p, q) = (x.arena().get(ray), y.arena().get(ray));
            if p.len() != q.len() {
                same_structure = false;
                continue;
            }
            for (u, v) in p.iter().zip(q) {
                for (m, n) in [(u.t0, v.t0), (u.t1, v.t1)] {
                    #[allow(
                        clippy::cast_possible_wrap,
                        reason = "both are ordinary positive lengths"
                    )]
                    let d = (m.to_bits() as i64 - n.to_bits() as i64).abs();
                    worst_ulp = worst_ulp.max(d);
                }
            }
        }
    }
    (same_structure, worst_ulp)
}

fn level(sweep: f64) -> ArcMove {
    ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 9.0,
        start_angle: 0.37,
        sweep,
        z: 6.0,
        plane: ArcPlane::Xy,
        rise: 0.0,
    }
}

/// The two halves of `arc`, split at fraction `f` of its sweep.
///
/// The second piece's start angle is `start + sweep * f`, computed here exactly
/// as a caller splitting a program would compute it — rounded, and then fed to
/// `sin`/`cos`. That is the whole point of the exercise.
fn split_at(arc: &ArcMove, f: f64) -> [Motion; 2] {
    let first = ArcMove {
        sweep: arc.sweep * f,
        rise: arc.rise * f,
        ..*arc
    };
    let second = ArcMove {
        start_angle: arc.start_angle + arc.sweep * f,
        sweep: arc.sweep * (1.0 - f),
        z: arc.z + arc.rise * f,
        rise: arc.rise * (1.0 - f),
        ..*arc
    };
    [Motion::Arc(first), Motion::Arc(second)]
}

#[test]
fn splitting_an_arc_mid_sweep_agrees_to_within_a_few_ulp() {
    let profile = mill();
    let arc = level(2.1);
    let (whole, removed_whole) = cut(&profile, &[Motion::Arc(arc)]);

    for fraction in [0.5, 0.25, 1.0 / 3.0, 0.87] {
        let (split, removed_split) = cut(&profile, &split_at(&arc, fraction));

        // Bit-identical by absorption, exactly as in the linear case: an endpoint
        // shift of order 1e-15 mm cannot survive into an accumulator of order
        // 1000 mm^3. It is not evidence that the two agreed exactly -- the ULP
        // assertion below is what says by how much.
        assert_eq!(
            removed_whole.to_bits(),
            removed_split.to_bits(),
            "split at {fraction}: removed volume {removed_whole} against \
             {removed_split}"
        );
        let (structure, ulp) = compare(&whole, &split);
        assert!(
            structure,
            "split at {fraction} changed the span structure, so a sliver was left \
             at the join"
        );
        assert!(
            ulp <= 64,
            "split at {fraction}: endpoints differ by {ulp} ULP. Two roundings \
             through a transcendental buy a wider tolerance than a line's one, \
             but not this much"
        );
    }
}

#[test]
fn a_full_circle_split_into_quarters_agrees() {
    // Four pieces rather than two, so the rounded start angles compound.
    let profile = mill();
    let arc = level(2.0 * PI);
    let (whole, removed_whole) = cut(&profile, &[Motion::Arc(arc)]);

    let quarters: Vec<Motion> = (0..4)
        .map(|k| {
            Motion::Arc(ArcMove {
                start_angle: arc.start_angle + arc.sweep * (f64::from(k) / 4.0),
                sweep: arc.sweep / 4.0,
                ..arc
            })
        })
        .collect();
    let (split, removed_split) = cut(&profile, &quarters);

    assert_eq!(
        removed_whole.to_bits(),
        removed_split.to_bits(),
        "a circle cut in quarters removes {removed_split}, whole {removed_whole}"
    );
    let (structure, ulp) = compare(&whole, &split);
    assert!(structure, "quartering changed the span structure");
    assert!(ulp <= 64, "quartering moved endpoints by {ulp} ULP");
}

#[test]
fn reversing_an_arc_removes_the_same_material() {
    // Same locus, traversed the other way: start at the far end and negate the
    // sweep. Nothing about the swept solid depends on direction, so any
    // disagreement is a sign error in the wedge or in the half-plane test.
    let profile = ball();
    let forward = level(1.7);
    let backward = ArcMove {
        start_angle: forward.start_angle + forward.sweep,
        sweep: -forward.sweep,
        ..forward
    };

    let (a, removed_a) = cut(&profile, &[Motion::Arc(forward)]);
    let (b, removed_b) = cut(&profile, &[Motion::Arc(backward)]);

    let (structure, ulp) = compare(&a, &b);
    assert!(
        structure,
        "reversing the arc changed the span structure, which a direction-free \
         solid cannot do"
    );
    assert!(ulp <= 64, "reversing moved endpoints by {ulp} ULP");
    let relative = (removed_a - removed_b).abs() / removed_a;
    assert!(
        relative < 1.0e-12,
        "reversed removed {removed_b}, forward {removed_a}"
    );
}

#[test]
fn cutting_the_same_arc_twice_equals_cutting_it_once() {
    // Subtraction is idempotent, so the second pass must be a no-op down to the
    // bit. A cut that grew on repetition would mean the swept solid was being
    // computed against the current field rather than against the motion.
    let profile = mill();
    let arc = Motion::Arc(level(2.4));
    let (once, _) = cut(&profile, std::slice::from_ref(&arc));
    let (twice, _) = cut(&profile, &[arc, arc]);
    assert_eq!(
        digest(&once),
        digest(&twice),
        "cutting an arc twice differed from cutting it once"
    );
}

#[test]
fn a_thousand_arc_cuts_do_not_accumulate() {
    // The drift test, for arcs. If any part of the arc path read the field and
    // wrote back something slightly different -- a re-normalised endpoint, a
    // merge that nudged a boundary -- a thousand repeats would show it and one
    // would not.
    let profile = mill();
    let arc = Motion::Arc(level(2.0 * PI));
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);

    cut_tri_motion(&mut field, &profile, &arc, METHOD, &mut scratch);
    let after_first = digest(&field);
    let volume_after_first = field.volume();

    for _ in 0..999 {
        cut_tri_motion(&mut field, &profile, &arc, METHOD, &mut scratch);
    }

    assert_eq!(
        digest(&field),
        after_first,
        "1000 identical arc cuts drifted away from 1"
    );
    assert_eq!(
        field.volume().to_bits(),
        volume_after_first.to_bits(),
        "the volume moved over 1000 repeats without the field changing, which \
         would mean the measurement is not a function of the field"
    );
}

#[test]
fn a_helix_split_mid_sweep_agrees_to_within_a_few_ulp() {
    // A helix splits in the angle AND in the height, and it is sub-stepped, so
    // the two pieces do not even sample the path at the same places as the
    // whole. Bit-identity is out of reach and so is anything close to it: the
    // two are different approximations, not two computations of one thing.
    //
    // **The tolerance is derived, not chosen.** Both schedules meet the same
    // 0.04 mm deviation bound, so the swept solids can differ by that much
    // anywhere on their boundary. The solid here is `H (R r Theta + pi r^2)` =
    // 255 pi = 801 mm^3, with roughly 440 mm^2 of wall (outer radius 10 and
    // inner radius 4, over a 5 mm descent). Displacing 440 mm^2 by 0.04 mm is
    // 17.6 mm^3 -- about **2.2%**, which is what the bound alone permits.
    //
    // Measured: 0.021%, a hundred times inside that. The two step schedules
    // agree far better than they are required to, which is worth knowing but is
    // not something to assert on. The threshold below is 0.1%: twenty times
    // inside the bound, five times outside the measurement, so it catches a real
    // regression without breaking on a re-planned step count.
    let profile = mill();
    let helix = ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 7.0,
        start_angle: 0.2,
        sweep: 2.0 * PI,
        z: 10.0,
        plane: ArcPlane::Xy,
        rise: -5.0,
    };
    let (whole, removed_whole) = cut(&profile, &[Motion::Arc(helix)]);

    for fraction in [0.5, 0.4] {
        let (split, removed_split) = cut(&profile, &split_at(&helix, fraction));
        let relative = (removed_whole - removed_split).abs() / removed_whole;
        assert!(
            relative < 1.0e-3,
            "split at {fraction}: removed {removed_split} against {removed_whole}, \
             a relative difference of {relative:.3e}. That is past the 1e-3 \
             threshold, which the shared sub-step bound leaves room for twenty \
             times over"
        );
        let (structure, _) = compare(&whole, &split);
        assert!(
            structure,
            "split at {fraction} changed the span structure of a helix, meaning a \
             sliver survived at the join"
        );
    }
}

#[test]
fn an_arc_entirely_outside_the_stock_changes_nothing_at_all() {
    let profile = mill();
    let away = ArcMove {
        center: Vec3::new(200.0, 150.0, 0.0),
        radius: 9.0,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 6.0,
        plane: ArcPlane::Xy,
        rise: 0.0,
    };
    let untouched = digest(&stock());
    let (field, removed) = cut(&profile, &[Motion::Arc(away)]);
    assert_eq!(digest(&field), untouched);
    assert_eq!(removed.to_bits(), 0.0f64.to_bits());
}

#[test]
fn an_arc_of_zero_sweep_is_the_static_tool() {
    // The degenerate end of the wedge. Every point's bearing is outside a zero
    // wedge, so the whole solid comes from the endpoint pieces -- and both
    // endpoints are the same place.
    let profile = mill();
    let arc = ArcMove {
        sweep: 0.0,
        ..level(0.0)
    };
    let (from_arc, removed_arc) = cut(&profile, &[Motion::Arc(arc)]);

    // The same tool, parked, expressed as a zero-length linear move.
    let point = arc.at(0.0);
    let parked = Motion::Linear(chipbreaker_core::sweep::LinearMove {
        start: point,
        end: point,
    });
    let (from_point, removed_point) = cut(&profile, &[parked]);

    assert_eq!(
        digest(&from_arc),
        digest(&from_point),
        "a zero-sweep arc is a parked tool and must cut like one"
    );
    assert_eq!(removed_arc.to_bits(), removed_point.to_bits());
    assert!(
        removed_arc > 0.0,
        "the parked tool should still cut something"
    );
}

#[test]
fn the_start_angle_is_taken_modulo_a_full_turn() {
    // Adding 2 pi to the start angle is the same arc. It is NOT the same
    // floating-point computation -- `sin(x + 2 pi)` is not `sin(x)` bitwise --
    // so this bounds how far that argument reduction can move a cut surface.
    let profile = mill();
    let arc = level(1.3);
    let shifted = ArcMove {
        start_angle: arc.start_angle + 2.0 * PI,
        ..arc
    };
    let (a, _) = cut(&profile, &[Motion::Arc(arc)]);
    let (b, _) = cut(&profile, &[Motion::Arc(shifted)]);

    let (structure, ulp) = compare(&a, &b);
    assert!(structure, "a full-turn shift changed the span structure");
    assert!(
        ulp <= 4096,
        "a full-turn shift of the start angle moved endpoints by {ulp} ULP. The \
         angle itself loses about {} ULP to the addition, and the tool radius \
         scales that up",
        ((arc.start_angle + 2.0 * PI) - 2.0 * PI - arc.start_angle).abs() / f64::EPSILON
    );
    // And the tangent sanity check that the shift really is a no-op in exact
    // arithmetic, so a failure above is rounding and not a modelling error.
    let (s0, c0) = t::sin_cos(arc.start_angle);
    let (s1, c1) = t::sin_cos(shifted.start_angle);
    assert!((s0 - s1).abs() < 1.0e-15 && (c0 - c1).abs() < 1.0e-15);
}
