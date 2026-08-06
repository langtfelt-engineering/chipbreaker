// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Removed volume for arcs and helices, against closed forms.
//!
//! Companion to `sweep_volume.rs`, and under the same reading of ADR 0005: this
//! is a conservation check on the subtraction, not an accuracy claim. A lost end
//! cap, a wedge tested with the wrong sign, or an annulus computed as a disc all
//! move this number a great deal, and none of them would show up in a deviation
//! measurement — which only looks at where the field sampled the cut surface.
//!
//! # The swept cross-section, which is where both closed forms come from
//!
//! A cylindrical tool of radius `r` whose centre travels a circular arc of
//! radius `R > r` and angular extent `alpha` sweeps the Minkowski sum of that arc
//! with a disc: an annular sector plus a half-disc cap at each end.
//!
//! ```text
//! A(alpha) = 2 * alpha * R * r  +  pi * r^2            (0 <= alpha, no self-overlap)
//! A(2 pi)  = pi ((R+r)^2 - (R-r)^2) = 4 pi R r          (the full annulus)
//! ```
//!
//! `A(0) = pi r^2` — one disc — which is the check that the caps are counted
//! once and not twice.
//!
//! # The helix, by Cavalieri
//!
//! Descending helix, tip from `z0` down by `H` over angular extent `Theta`. The
//! tool occupies everything above its tip, so a horizontal level `zeta` is
//! covered by exactly those path parameters whose tip is at or below it — an
//! angular extent that runs linearly from `0` at the bottom to `Theta` at the
//! top. So, substituting `alpha` for `zeta`,
//!
//! ```text
//! V = integral A(alpha(zeta)) d zeta
//!   = (H / Theta) * integral_0^Theta (2 alpha R r + pi r^2) d alpha
//!   = H * (R r Theta + pi r^2)
//! ```
//!
//! Two sanity checks fall straight out. `R = 0` gives `H pi r^2`, a cylinder, and
//! `Theta = 0` gives the same — both are a plunge, which is what they are.
//!
//! This is the one place a helix has a closed form at all. The swept *volume*
//! integrates cleanly even though the swept *solid* does not decompose, which is
//! exactly why volume is worth checking here and deviation is measured elsewhere.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::Motion;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri_motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

const PI: f64 = core::f64::consts::PI;

/// Cell size. Divides every stock extent exactly, so `Lattice::pad` is zero and
/// ADR 0005's quantisation bias is out of the measurement.
const SPACING: f64 = 0.25;

/// Stock: 40 x 30 x 10 mm, lower corner at the origin.
fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// A cylindrical cutter: radius `r` at every height, shank included.
///
/// Constant radius matters. The closed forms above take `rho` to be one number,
/// and a necked or tapered tool would make `A` depend on the level as well as
/// on the angle.
fn mill(radius: f64) -> Profile {
    flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid")
}

fn removed(motion: &Motion, profile: &Profile, tolerance: f64) -> [f64; 3] {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let before = field.volumes();
    cut_tri_motion(
        &mut field,
        profile,
        motion,
        SweepMethod::Analytic { tolerance },
        &mut scratch,
    );
    let after = field.volumes();
    let mut out = [0.0; 3];
    for axis in AXES {
        let i = axis.index();
        out[i] = match (before[i], after[i]) {
            (Some(b), Some(a)) => b - a,
            _ => 0.0,
        };
    }
    out
}

fn assert_close(name: &str, bundle: usize, got: f64, expected: f64, relative: f64) {
    let error = (got - expected).abs() / expected;
    assert!(
        error <= relative,
        "{name}, bundle {bundle}: removed {got:.6} mm3, closed form {expected:.6} mm3, \
         relative error {:.4}% exceeds {:.4}%",
        error * 100.0,
        relative * 100.0
    );
}

#[test]
fn a_circular_pocket_removes_an_annular_prism() {
    // Full turn, so the swept cross-section is the whole annulus and the caps
    // fall away. Centre away from the stock walls so nothing clips.
    let (centre, big, small, depth) = (Vec3::new(20.0, 15.0, 0.0), 5.0, 3.0, 3.0);
    let profile = mill(small);
    let motion = Motion::Arc(ArcMove {
        center: centre,
        radius: big,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 10.0 - depth,
        plane: ArcPlane::Xy,
        rise: 0.0,
    });
    // pi ((R+r)^2 - (R-r)^2) * depth = 4 pi R r depth.
    let expected = 4.0 * PI * big * small * depth;
    let got = removed(&motion, &profile, SPACING / 10.0);
    for (bundle, value) in got.iter().enumerate() {
        // Each bundle samples the annulus its own way and none of them is the
        // truth; 2% is what a 0.25 mm cell affords against a shape with this
        // much curved boundary. The three are not expected to agree.
        assert_close("circular pocket", bundle, *value, expected, 0.02);
    }
}

#[test]
fn the_circular_pocket_on_the_z_bundle_is_exactly_the_lattice_point_count() {
    // The same identity the plunge test found, one dimension out: a Z ray is
    // either wholly in the annulus or wholly out, and every ray inside loses
    // exactly `depth`. So the Z bundle's volume is `h^2 * depth * (cell centres
    // in the annulus)` -- exactly, not nearly, and this asserts it bit for bit.
    //
    // No cell centre can land on either boundary circle, which is what makes an
    // exact assertion safe rather than lucky. With `pad = 0` a centre sits at
    // `0.125 + 0.25 k`, so its offset from an integer centre is an odd eighth;
    // `d^2 = 4` would need `odd^2 + odd^2 = 256` and `d^2 = 64` would need
    // `odd^2 + odd^2 = 4096`. An odd square is `1 mod 8`, so the left side is
    // `2 mod 8` and neither right side is. No solutions.
    let (cx, cy, big, small, depth) = (20.0, 15.0, 5.0, 3.0, 3.0);
    let profile = mill(small);
    let motion = Motion::Arc(ArcMove {
        center: Vec3::new(cx, cy, 0.0),
        radius: big,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 10.0 - depth,
        plane: ArcPlane::Xy,
        rise: 0.0,
    });

    let mut inside = 0u64;
    let cells_x = (40.0 / SPACING) as usize;
    let cells_y = (30.0 / SPACING) as usize;
    for i in 0..cells_x {
        for j in 0..cells_y {
            let x = (i as f64 + 0.5) * SPACING - cx;
            let y = (j as f64 + 0.5) * SPACING - cy;
            let d = (x * x + y * y).sqrt();
            if (d - big).abs() <= small {
                inside += 1;
            }
        }
    }

    let expected = SPACING * SPACING * depth * inside as f64;
    let got = removed(&motion, &profile, SPACING / 10.0)[2];
    assert_eq!(
        got.to_bits(),
        expected.to_bits(),
        "Z bundle removed {got} mm3, but {inside} cell centres in the annulus at \
         {depth} mm deep is exactly {expected} mm3"
    );

    // And the continuous annulus, for scale: the shortfall is the Gauss circle
    // error of two discs, not a defect.
    let continuous = 4.0 * PI * big * small * depth;
    let shortfall = (continuous - got).abs() / continuous;
    assert!(
        shortfall < 0.01,
        "the lattice count should track the continuous annulus to within a \
         percent, got {:.4}%",
        shortfall * 100.0
    );
}

#[test]
fn an_arc_slot_removes_an_annular_sector_plus_two_caps() {
    // A quarter turn, so both end caps are present and separate. This is the
    // case that a decomposition dropping the endpoint tools would get wrong by
    // `pi r^2 * depth`, which here is 84.8 mm3 out of 226 -- 37%, impossible to
    // miss.
    let (centre, big, small, depth) = (Vec3::new(20.0, 15.0, 0.0), 5.0, 3.0, 3.0);
    let alpha = PI / 2.0;
    let profile = mill(small);
    let motion = Motion::Arc(ArcMove {
        center: centre,
        radius: big,
        start_angle: 0.3,
        sweep: alpha,
        z: 10.0 - depth,
        plane: ArcPlane::Xy,
        rise: 0.0,
    });
    let expected = (2.0 * alpha * big * small + PI * small * small) * depth;
    let got = removed(&motion, &profile, SPACING / 10.0);
    for (bundle, value) in got.iter().enumerate() {
        assert_close("arc slot", bundle, *value, expected, 0.02);
    }
}

#[test]
fn a_helical_bore_matches_the_cavalieri_integral() {
    // Tip starts exactly at the stock top, so there is no slab above it and the
    // integral runs over the whole descent. Half a turn, which keeps the swept
    // cross-section free of self-overlap so `A(alpha)` is the plain formula.
    let (centre, big, small) = (Vec3::new(20.0, 15.0, 0.0), 5.0, 3.0);
    let (theta, rise) = (PI, 4.0);
    let profile = mill(small);
    let motion = Motion::Arc(ArcMove {
        center: centre,
        radius: big,
        start_angle: 0.0,
        sweep: theta,
        z: 10.0,
        plane: ArcPlane::Xy,
        rise: -rise,
    });
    // H (R r Theta + pi r^2).
    let expected = rise * (big * small * theta + PI * small * small);
    let got = removed(&motion, &profile, SPACING / 10.0);
    for (bundle, value) in got.iter().enumerate() {
        // Looser than the level cases: a helix is sub-stepped, so the sweep
        // deviation is on top of the sampling error, and the cut surface is a
        // ruled helicoid that a lattice samples worse than a vertical wall.
        assert_close("helical bore", bundle, *value, expected, 0.03);
    }
}

#[test]
fn a_helix_of_zero_radius_is_a_plunge() {
    // `V = H (R r Theta + pi r^2)` with `R = 0` is a cylinder, and it had better
    // be, because the geometry is a plunge with a pointless angle attached. Runs
    // the helix code path against an answer nobody can argue with.
    let small = 3.0;
    let rise = 6.0;
    let profile = mill(small);
    let motion = Motion::Arc(ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 0.0,
        start_angle: 0.0,
        sweep: 3.0 * PI,
        z: 10.0,
        plane: ArcPlane::Xy,
        rise: -rise,
    });
    let expected = PI * small * small * rise;
    let got = removed(&motion, &profile, SPACING / 10.0);
    for (bundle, value) in got.iter().enumerate() {
        assert_close("degenerate helix", bundle, *value, expected, 0.02);
    }
}

#[test]
fn a_deeper_arc_removes_proportionally_more() {
    // The annular prism is linear in depth, and a cut whose depth handling was
    // off by a cell would show as a constant offset rather than a ratio.
    let (centre, big, small) = (Vec3::new(20.0, 15.0, 0.0), 5.0, 3.0);
    let profile = mill(small);
    let at_depth = |depth: f64| {
        let motion = Motion::Arc(ArcMove {
            center: centre,
            radius: big,
            start_angle: 0.0,
            sweep: 2.0 * PI,
            z: 10.0 - depth,
            plane: ArcPlane::Xy,
            rise: 0.0,
        });
        removed(&motion, &profile, SPACING / 10.0)[2]
    };
    let shallow = at_depth(2.0);
    let deep = at_depth(6.0);
    let ratio = deep / shallow;
    assert!(
        (ratio - 3.0).abs() < 1.0e-12,
        "three times the depth should remove three times the volume on the Z \
         bundle, where the annulus is identical and only the depth changes; \
         got {ratio}"
    );
}
