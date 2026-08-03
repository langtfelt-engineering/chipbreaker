// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Arc resolution: the richest source of bugs in this unit.
//!
//! Two of these are required by the definition of done — the `I`/`J`/`K` and `R`
//! forms of one arc must resolve identically, and the same arc must behave
//! correctly in all three planes.

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{ArcForm, ArcPlane};
use chipbreaker_gcode::arcs::{
    ArcRequest, DEFAULT_ARC_TOLERANCE, RADIUS_FORM_MARGIN, Turn, resolve,
};
use chipbreaker_gcode::diag::{Diagnostics, GcodeError, Site};

use core::f64::consts::{FRAC_PI_2, PI, TAU};

fn request(start: Vec3, end: Vec3, plane: ArcPlane, turn: Turn) -> ArcRequest {
    ArcRequest {
        start,
        end,
        plane,
        turn,
        centre: None,
        radius_word: None,
        extra_turns: 0,
        tolerance: DEFAULT_ARC_TOLERANCE,
        site: Site::new(0, 1, 1),
    }
}

fn ok(request: &ArcRequest) -> (chipbreaker_core::toolpath::ArcData, Diagnostics) {
    let mut diagnostics = Diagnostics::new();
    let arc = resolve(request, &mut diagnostics).expect("should resolve");
    (arc, diagnostics)
}

fn err(request: &ArcRequest) -> GcodeError {
    let mut diagnostics = Diagnostics::new();
    resolve(request, &mut diagnostics).expect_err("should be refused")
}

#[test]
fn a_quarter_arc_in_g17_sweeps_the_right_way() {
    // G3 from (10,0) to (0,10) about the origin is a quarter turn
    // counter-clockwise seen from +Z, so a positive quarter about the normal.
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 10.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    let (arc, _) = ok(&r);
    assert!((arc.sweep - FRAC_PI_2).abs() < 1e-12, "{}", arc.sweep);
    assert!((arc.radius - 10.0).abs() < 1e-12);

    // The same endpoints as G2 take the long way round: three quarters, negative.
    r.turn = Turn::Clockwise;
    let (arc, _) = ok(&r);
    assert!(
        (arc.sweep + 3.0 * FRAC_PI_2).abs() < 1e-12,
        "G2 over the same endpoints is -270 degrees, got {}",
        arc.sweep
    );
}

/// The strongest agreement the two arc forms can be held to.
///
/// **Not zero, and it cannot be.** The `I`/`J`/`K` form is *given* the centre;
/// the `R` form *derives* it, through a square root and two divisions. Those are
/// different computations of the same quantity, so they land within a rounding
/// of each other rather than on the same bits. For a quarter circle of radius 10
/// the observed difference is `8.9e-16`, about half an ULP at that scale.
///
/// This matters beyond the test. The specification asks that the two forms
/// "produce identical IR", and byte-identical is what a golden hash compares —
/// so the golden IR of an arc written with `R` will differ from the same arc
/// written with `I`/`J`/`K`, in the last bit or two of the centre. The corpus
/// pairs them and compares within this bound instead, and says so.
///
/// Expressed relative to the coordinate magnitude, since an absolute bound would
/// mean something different at 10 mm and at 1000 mm.
///
/// # Where 32 comes from
///
/// Measured, not guessed. `cargo run -p chipbreaker-gcode --example
/// form_agreement` sweeps 39 angles at four scales and reports the worst
/// disagreement: **11.15 ULP** in the centre and 3.82 in the sweep. Thirty-two
/// is that with margin.
///
/// The first version of this constant was 8, taken from a single quarter-circle
/// observation of 0.5 ULP. That is the same error made twice in Unit 3 --
/// picking a threshold from one sample instead of from a distribution -- and it
/// is why the example exists rather than a comment claiming a number.
const FORM_AGREEMENT_ULPS: f64 = 32.0;

fn agree_to_ulps(a: f64, b: f64, scale: f64) -> bool {
    let ulp = scale.abs().max(1.0) * f64::EPSILON;
    (a - b).abs() <= FORM_AGREEMENT_ULPS * ulp
}

#[test]
fn the_ijk_and_r_forms_of_one_arc_resolve_to_within_a_rounding() {
    // Required by the definition of done, and the cheapest test there is for a
    // sign error in either path. A quarter circle is a minor arc, so positive R.
    //
    // "To within a rounding" rather than "identically": see
    // `FORM_AGREEMENT_ULPS`. The two forms compute the centre differently and
    // exact agreement is not available.
    for turn in [Turn::Clockwise, Turn::CounterClockwise] {
        let mut by_centre = request(
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            ArcPlane::Xy,
            turn,
        );
        by_centre.centre = Some(Vec3::new(0.0, 0.0, 0.0));

        let mut by_radius = by_centre;
        by_radius.centre = None;
        // G2 over these endpoints is the *major* arc, so its R is negative.
        by_radius.radius_word = Some(if turn == Turn::CounterClockwise {
            10.0
        } else {
            -10.0
        });

        let (a, _) = ok(&by_centre);
        let (b, _) = ok(&by_radius);

        for (u, v) in a.center.to_array().iter().zip(b.center.to_array()) {
            assert!(
                agree_to_ulps(*u, v, 10.0),
                "{turn:?}: centres {:?} and {:?} differ by more than a rounding",
                a.center,
                b.center
            );
        }
        assert!(
            agree_to_ulps(a.sweep, b.sweep, TAU),
            "{turn:?}: sweeps {} and {}",
            a.sweep,
            b.sweep
        );
        assert!(agree_to_ulps(a.radius, b.radius, 10.0));
        assert_eq!(a.plane, b.plane);
        // Only the provenance differs, and deliberately so.
        assert_eq!(a.form, ArcForm::CentreOffsets);
        assert_eq!(b.form, ArcForm::Radius);
    }
}

#[test]
fn the_sign_of_r_chooses_the_minor_or_the_major_arc() {
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 10.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );

    r.radius_word = Some(10.0);
    let (minor, _) = ok(&r);
    assert!(
        (minor.sweep - FRAC_PI_2).abs() < 1e-12,
        "positive R is the minor arc: {}",
        minor.sweep
    );

    r.radius_word = Some(-10.0);
    let (major, _) = ok(&r);
    assert!(
        (major.sweep - 3.0 * FRAC_PI_2).abs() < 1e-12,
        "negative R is the major arc: {}",
        major.sweep
    );
    assert!(major.sweep.abs() > PI, "and it is more than a half turn");
}

#[test]
fn g18_handedness_matches_the_documented_reference() {
    // The trap named in the specification. RS-274 orders G18 as Z then X, so
    // its right-handed normal is +Y and G2 is clockwise *seen from +Y*.
    //
    // Pinned to a case that can be checked without re-deriving the convention:
    // a positive rotation about +Y carries +Z toward +X. So travelling from the
    // +X axis to the +Z axis is the negative direction about +Y, and a G2 that
    // does it in a quarter turn must have sweep -PI/2.
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 10.0),
        ArcPlane::Zx,
        Turn::Clockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    let (arc, _) = ok(&r);
    assert!(
        (arc.sweep + FRAC_PI_2).abs() < 1e-12,
        "G2 from +X to +Z in G18 is a negative quarter about +Y, got {}",
        arc.sweep
    );

    // And G3 over the same endpoints takes the long way, positive.
    r.turn = Turn::CounterClockwise;
    let (arc, _) = ok(&r);
    assert!((arc.sweep - 3.0 * FRAC_PI_2).abs() < 1e-12, "{}", arc.sweep);
}

#[test]
fn the_same_arc_behaves_the_same_in_every_plane() {
    // Coordinates permuted to match each plane's axis order, so all three are
    // the *same* arc seen from a different axis. Their sweeps must be identical.
    let cases = [
        (
            ArcPlane::Xy,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
        ),
        (
            ArcPlane::Zx,
            Vec3::new(0.0, 0.0, 10.0),
            Vec3::new(10.0, 0.0, 0.0),
        ),
        (
            ArcPlane::Yz,
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 10.0),
        ),
    ];
    for (plane, start, end) in cases {
        let mut r = request(start, end, plane, Turn::CounterClockwise);
        r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
        let (arc, _) = ok(&r);
        assert!(
            (arc.sweep - FRAC_PI_2).abs() < 1e-12,
            "{plane:?}: expected +PI/2, got {}",
            arc.sweep
        );
        assert!((arc.radius - 10.0).abs() < 1e-12);
    }
}

#[test]
fn coincident_endpoints_are_a_full_circle_with_ijk_and_an_error_with_r() {
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );

    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    let (arc, _) = ok(&r);
    assert!(
        (arc.sweep - TAU).abs() < 1e-12,
        "a full turn, not no motion: {}",
        arc.sweep
    );

    // With R it names no particular circle: every circle of that radius through
    // the point qualifies.
    r.centre = None;
    r.radius_word = Some(10.0);
    assert!(
        matches!(err(&r), GcodeError::FullCircleWithRadiusWord { .. }),
        "an R-form full circle is meaningless"
    );
}

#[test]
fn an_r_arc_near_a_half_turn_is_refused_rather_than_guessed() {
    // The centre sits at sqrt(R^2 - h^2) from the chord midpoint. As h -> R that
    // derivative goes to infinity, so a micron of endpoint rounding moves the
    // centre by millimetres. The information is not in the file.
    let radius = 10.0;
    let half_chord = radius * (1.0 - RADIUS_FORM_MARGIN / 2.0);
    let mut r = request(
        Vec3::new(-half_chord, 0.0, 0.0),
        Vec3::new(half_chord, 0.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.radius_word = Some(radius);
    match err(&r) {
        GcodeError::ArcIllConditioned { .. } => {}
        other => panic!("{other:?}"),
    }
    // The message has to say what to do instead.
    assert!(err(&r).to_string().contains("I/J/K"));

    // Comfortably inside the margin it resolves without complaint.
    let mut fine = r;
    fine.start = Vec3::new(-5.0, 0.0, 0.0);
    fine.end = Vec3::new(5.0, 0.0, 0.0);
    let (arc, _) = ok(&fine);
    assert!(arc.radius > 0.0);
}

#[test]
fn a_radius_too_small_to_reach_is_refused() {
    let mut r = request(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(30.0, 0.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.radius_word = Some(5.0);
    assert!(matches!(err(&r), GcodeError::ArcRadiusTooSmall { .. }));
}

#[test]
fn a_radius_mismatch_inside_tolerance_recentres_and_records_the_residual() {
    // CAM rounds coordinates, so this is the ordinary case rather than an error.
    // U13 needs the residual to tell a deviation caused by geometry from one
    // caused by rounding.
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        // 10.004 from the origin rather than 10: a 4 micron mismatch.
        Vec3::new(0.0, 10.004, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    let (arc, diagnostics) = ok(&r);

    assert_eq!(diagnostics.count_of("arc-recentred"), 1);
    assert!(
        (arc.radius_residual.abs() - 0.004).abs() < 1e-9,
        "residual {}",
        arc.radius_residual
    );
    // After recentring the two distances agree.
    let to_start = (r.start - arc.center).length();
    let to_end = (r.end - arc.center).length();
    assert!(
        (to_start - to_end).abs() < 1e-12,
        "{to_start} vs {to_end} after recentring"
    );
}

#[test]
fn a_radius_mismatch_beyond_tolerance_is_refused_and_says_by_how_much() {
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 10.5, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    match err(&r) {
        GcodeError::ArcRadiusMismatch {
            start_radius,
            end_radius,
            tolerance,
            ..
        } => {
            assert!((start_radius - 10.0).abs() < 1e-9);
            assert!((end_radius - 10.5).abs() < 1e-9);
            assert!((tolerance - DEFAULT_ARC_TOLERANCE).abs() < 1e-15);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn extra_turns_add_whole_revolutions_in_the_right_direction() {
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 10.0, 0.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    r.extra_turns = 2;
    let (arc, _) = ok(&r);
    assert!(
        (arc.sweep - (FRAC_PI_2 + 2.0 * TAU)).abs() < 1e-12,
        "{}",
        arc.sweep
    );
    assert_eq!(arc.turns(), 2);

    // Clockwise, the extra turns are negative too.
    r.turn = Turn::Clockwise;
    let (arc, _) = ok(&r);
    assert!(arc.sweep < 0.0);
    assert_eq!(arc.turns(), -2);
}

#[test]
fn a_helix_keeps_its_centre_on_the_start_plane() {
    // The out-of-plane coordinate of the centre is taken from the start, so the
    // centre is a point on the axis of the helix at its beginning rather than
    // floating somewhere between the endpoints.
    let mut r = request(
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(0.0, 10.0, 5.0),
        ArcPlane::Xy,
        Turn::CounterClockwise,
    );
    r.centre = Some(Vec3::new(0.0, 0.0, 0.0));
    let (arc, _) = ok(&r);
    assert!((arc.center.z - 0.0).abs() < 1e-12, "{:?}", arc.center);
    assert!((arc.sweep - FRAC_PI_2).abs() < 1e-12);
}
