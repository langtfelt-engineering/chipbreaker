// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Removed volume against closed forms.
//!
//! # Why volume is the right metric *here*, and only here
//!
//! ADR 0005 rules that volume is a construction-time diagnostic and deviation is
//! the accuracy metric, for three measured reasons: it is non-monotone under
//! refinement, it floors out against tessellation, and it carries a cell
//! quantisation bias.
//!
//! **This file does not contradict that.** It is not measuring surface fidelity
//! and it is not making an accuracy claim. It is checking that the *quantity
//! removed* matches a solid whose volume is known in closed form — a
//! conservation check on the subtraction, not a statement about where the cut
//! surface lies. A sign error in the prism, a lost end cap, or a double-counted
//! overlap all change this number by a lot; none of them would be caught by
//! deviation alone, because deviation only looks at where the field sampled the
//! surface.
//!
//! The geometries are chosen so the cell size divides the stock extents exactly.
//! That makes the quantisation bias of ADR 0005 reason 3 vanish — with no slack
//! there is nothing to over-count — so what remains is genuine sampling error,
//! which is what the tolerances are set from.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::LinearMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

const PI: f64 = core::f64::consts::PI;

/// Stock: 40 x 20 x 10 mm, its lower corner at the origin.
///
/// Every extent is divided exactly by the 0.25 mm cells, so `Lattice::pad` is
/// zero on all three bundles and no cell claims area outside the stock.
fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 20.0, 10.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: 0.25,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill(radius: f64) -> Profile {
    flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid")
}

/// Removed volume per bundle, in `AXES` order.
fn removed(motion: &LinearMove, profile: &Profile) -> [f64; 3] {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let before: Vec<f64> = field.volumes().iter().map(|v| v.unwrap_or(0.0)).collect();
    cut_tri(
        &mut field,
        profile,
        motion,
        SweepMethod::Analytic { tolerance: 1.0e-4 },
        &mut scratch,
    );
    let after = field.volumes();
    [
        before[0] - after[0].unwrap_or(0.0),
        before[1] - after[1].unwrap_or(0.0),
        before[2] - after[2].unwrap_or(0.0),
    ]
}

/// Asserts every bundle's removed volume against a closed form.
///
/// Each bundle is checked **separately**. Averaging them would hide a bundle
/// that had gone wrong on its own, which is exactly the failure a per-bundle cut
/// contract makes possible.
fn assert_removed(name: &str, motion: &LinearMove, profile: &Profile, expected: f64, tol: f64) {
    assert_removed_per_bundle(name, motion, profile, expected, [tol; 3]);
}

/// As [`assert_removed`], with a tolerance per bundle.
///
/// Needed because the three bundles are **not** equally good at the same solid,
/// and one shared tolerance would have to be the worst of the three. A cylinder
/// seen from the side is a rectangle the lattice tiles exactly; seen end-on it is
/// a disc, and counting cells inside a disc is the Gauss circle problem from
/// a single bundle.
fn assert_removed_per_bundle(
    name: &str,
    motion: &LinearMove,
    profile: &Profile,
    expected: f64,
    tol: [f64; 3],
) {
    let measured = removed(motion, profile);
    for ((axis, value), tol) in AXES.iter().zip(measured).zip(tol) {
        let relative = (value - expected).abs() / expected;
        assert!(
            relative < tol,
            "{name} on the {} bundle: removed {value} mm^3 against a closed form of {expected} mm^3, {relative:.4e} apart (tolerance {tol:.1e})",
            axis.as_str()
        );
    }
}

#[test]
fn a_full_width_slot_removes_a_rectangular_prism() {
    // The tool passes clean through the block in `z` and runs off both ends in
    // `x`, so the removed solid is exactly a rectangular prism: the half-cylinder
    // caps fall outside the stock entirely.
    //
    // 40 long, 6 wide (the tool diameter), 10 deep.
    let radius = 3.0;
    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(-10.0, 10.0, -1.0),
        end: Vec3::new(50.0, 10.0, -1.0),
    };
    assert_removed(
        "full-width slot",
        &motion,
        &profile,
        40.0 * 2.0 * radius * 10.0,
        2.0e-3,
    );
}

#[test]
fn a_blind_slot_removes_a_prism_plus_two_half_cylinders() {
    // Both ends inside the stock, so the caps count. The swept cross-section is a
    // stadium: a rectangle `2R` by `L` plus two half-discs that together make one
    // full disc.
    //
    // This is the shape the three-piece decomposition exists for, and the caps
    // are the two pieces a naive implementation would drop.
    let radius = 3.0;
    let length = 20.0;
    let depth = 10.0;
    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(10.0, 10.0, -1.0),
        end: Vec3::new(10.0 + length, 10.0, -1.0),
    };
    let expected = (2.0 * radius * length + PI * radius * radius) * depth;
    assert_removed("blind slot", &motion, &profile, expected, 3.0e-3);
}

#[test]
fn a_face_pass_removes_a_stadium_of_the_commanded_depth() {
    // The same stadium, but the tool bottom sits inside the stock rather than
    // below it, so the depth is set by where the tip is rather than by the stock.
    let radius = 4.0;
    let length = 16.0;
    let depth = 3.0;
    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(12.0, 10.0, 10.0 - depth),
        end: Vec3::new(12.0 + length, 10.0, 10.0 - depth),
    };
    let expected = (2.0 * radius * length + PI * radius * radius) * depth;
    assert_removed("face pass", &motion, &profile, expected, 3.0e-3);
}

#[test]
fn a_straight_plunge_removes_a_cylinder() {
    // Case B's closed form. A flat mill entering from above leaves a
    // flat-bottomed hole, so the removed solid is a plain cylinder.
    //
    // The X and Y bundles see that cylinder from the side, where its silhouette
    // is a rectangle the lattice tiles exactly, so they are held tight. The Z
    // bundle sees it end-on, where the removed area is the count of cells whose
    // centre lies inside the disc -- the Gauss circle problem, measured at Unit
    // 5 and bounded there by roughly (h/R)^1.37. At h/R = 1/12 that bound is
    // 1.5%, and the measured shortfall is 0.97%.
    //
    // Rather than loosen the tolerance to absorb that, the next test asserts the
    // Z bundle against the lattice count EXACTLY, which is a stronger statement
    // than any tolerance.
    let radius = 3.0;
    let depth = 6.0;
    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(20.0, 10.0, 12.0),
        end: Vec3::new(20.0, 10.0, 10.0 - depth),
    };
    assert_removed_per_bundle(
        "straight plunge",
        &motion,
        &profile,
        PI * radius * radius * depth,
        // X and Y sum the disc's CHORDS across the transverse cells, which is a
        // midpoint rule on 2*sqrt(R^2 - y^2). Square-root endpoints make that
        // h^1.5 rather than h^2 -- the same behaviour measured on a lying
        // cylinder, where the fitted exponent was 1.46. Measured here: 2.6e-3.
        // Z counts whole cells inside the disc and gets the Gauss envelope.
        [5.0e-3, 5.0e-3, 1.6e-2],
    );
}

#[test]
fn the_plunge_shortfall_on_the_z_bundle_is_exactly_the_lattice_point_count() {
    // Not a defect and not a tolerance: an identity.
    //
    // A flat mill plunging along +Z removes, from a Z-bundle ray, a full column
    // or nothing at all -- the chord is a hard indicator. So the removed volume
    // is exactly h^2 * depth * (cells whose centre lies inside the disc), and the
    // gap to pi R^2 depth is exactly the disc's lattice-point counting error.
    //
    // Asserting the identity rather than the gap means a real regression in the
    // plunge path cannot hide inside a percent of slack.
    let radius = 3.0;
    let depth = 6.0;
    let spacing = 0.25;
    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(20.0, 10.0, 12.0),
        end: Vec3::new(20.0, 10.0, 10.0 - depth),
    };

    let mut field = stock();
    let lattice = field
        .bundle(chipbreaker_core::math::Axis::Z)
        .expect("built")
        .lattice()
        .clone();
    let before = field.volumes()[2].expect("built");
    let mut scratch = CutScratch::new(&profile);
    cut_tri(
        &mut field,
        &profile,
        &motion,
        SweepMethod::Analytic { tolerance: 1.0e-4 },
        &mut scratch,
    );
    let measured = before - field.volumes()[2].expect("built");

    // Count the cells independently, from the lattice alone.
    let [nx, ny] = lattice.counts();
    let mut inside = 0u64;
    for i in 0..nx {
        for j in 0..ny {
            let p = lattice.origin_of(i, j);
            let dx = p.x - motion.start.x;
            let dy = p.y - motion.start.y;
            if dx * dx + dy * dy <= radius * radius {
                inside += 1;
            }
        }
    }
    #[allow(clippy::cast_precision_loss, reason = "a small count")]
    let predicted = inside as f64 * spacing * spacing * depth;

    assert!(
        (measured - predicted).abs() < 1.0e-9,
        "the Z bundle removed {measured} mm^3; a lattice-point count of the disc predicts {predicted} mm^3. A plunge's removed volume must be exactly h^2 * depth * (cells inside the disc)."
    );
    // And that really is short of the true cylinder, by the bounded amount.
    let truth = PI * radius * radius * depth;
    let shortfall = (truth - measured) / truth;
    assert!(
        shortfall > 0.0 && shortfall < 1.6e-2,
        "the shortfall should be positive and inside the Gauss circle envelope, got {shortfall:.4e}"
    );
}

#[test]
fn a_ramp_entry_matches_an_independent_quadrature() {
    // Case C, and the one geometry here without a tidy closed form.
    //
    // The removed depth at a point is still exact in closed form, so the
    // reference is a quadrature of that rather than of the dexel field -- an
    // independent computation that shares no code with the thing under test,
    // which is what a closed form would have bought.
    //
    // A flat mill ramping from the top surface down to `depth` over `length`
    // removes, at a point `(x, y)` within the tool radius of the path,
    // everything from the deepest the flat bottom reached above it up to the top
    // surface. The bottom is deepest at the LARGEST path parameter that still
    // covers the point, clamped to the path's end.
    let radius = 3.0;
    let length = 20.0;
    let depth = 4.0;
    let top = 10.0;
    let (x0, y0) = (10.0, 10.0);

    let profile = mill(radius);
    let motion = LinearMove {
        start: Vec3::new(x0, y0, top),
        end: Vec3::new(x0 + length, y0, top - depth),
    };
    assert_eq!(
        motion.case(),
        chipbreaker_core::sweep::SweepCase::Ramp,
        "this geometry is meant to exercise the ramp path"
    );

    // Quadrature of the exact depth function, on a grid fine enough that its own
    // error is well under the tolerance being asserted.
    let n = 2000u32;
    let (lo_x, hi_x) = (x0 - radius, x0 + length + radius);
    let (lo_y, hi_y) = (y0 - radius, y0 + radius);
    let dx = (hi_x - lo_x) / f64::from(n);
    let dy = (hi_y - lo_y) / f64::from(n);
    let mut expected = 0.0;
    for i in 0..n {
        let x = lo_x + (f64::from(i) + 0.5) * dx;
        for j in 0..n {
            let y = lo_y + (f64::from(j) + 0.5) * dy;
            let across = (y - y0).abs();
            if across >= radius {
                continue;
            }
            // Half-width of the tool's footprint at this offset from the path.
            let half = (radius * radius - across * across).sqrt();
            // Path parameters that cover this point, clamped to the path.
            let last = (x + half).min(x0 + length);
            if last < x0 || x - half > x0 + length {
                continue;
            }
            // Deepest the flat bottom reached over this point.
            let s = ((last - x0) / length).clamp(0.0, 1.0);
            let bottom = top - depth * s;
            expected += (top - bottom).max(0.0) * dx * dy;
        }
    }

    // A looser tolerance than the exact cases, and deliberately so: this motion
    // is sub-stepped rather than closed-form, so it carries the sweep's own
    // deviation bound on top of the lattice sampling.
    assert_removed("ramp entry", &motion, &profile, expected, 1.5e-2);
}

#[test]
fn a_deeper_cut_removes_proportionally_more() {
    // A cheap invariant that would catch a depth used in the wrong frame -- tip
    // versus top of stock -- which a single absolute check might not, because one
    // constant offset can be absorbed by a tolerance.
    let radius = 3.0;
    let profile = mill(radius);
    let mut previous = 0.0;
    for depth in [1.0, 2.0, 4.0, 8.0] {
        let motion = LinearMove {
            start: Vec3::new(20.0, 10.0, 12.0),
            end: Vec3::new(20.0, 10.0, 10.0 - depth),
        };
        // The X bundle, whose silhouette here is a rectangle rather than a
        // disc, so this measures the depth and not the Gauss circle error.
        let measured = removed(&motion, &profile)[0];
        let expected = PI * radius * radius * depth;
        assert!(
            (measured - expected).abs() / expected < 5.0e-3,
            "plunge to {depth} mm removed {measured} against {expected}"
        );
        assert!(measured > previous, "deeper must remove more");
        previous = measured;
    }
}
