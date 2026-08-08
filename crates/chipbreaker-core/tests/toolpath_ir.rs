// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The invariants everything downstream is entitled to assume.
//!
//! These are not tests of a parser — no G-code appears here. They are tests of
//! the contract itself, written so that the contract fails loudly if a later
//! unit weakens it.

use std::collections::BTreeMap;

use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{
    ArcData, ArcForm, ArcPlane, FeedMode, FeedSpec, MotionKind, MotionSegment, OffsetEpoch,
    PathEvent, PathEventKind, Provenance, RapidPath, TOOLPATH_SCHEMA_VERSION, Toolpath,
    ToolpathError, ToolpathHeader, WorkOffsetId,
};

fn header() -> ToolpathHeader {
    let mut offsets = BTreeMap::new();
    offsets.insert(
        WorkOffsetId::from_gcode(54, 0).expect("G54"),
        vec![OffsetEpoch {
            // A full-precision value, per the fixture rule in CONTRIBUTING.md:
            // round numbers survive any serializer ever written.
            value: Vec3::new(-250.5, -100.25, -2.048_155_585_660_824_2),
            from_segment: 0,
        }],
    );
    ToolpathHeader {
        schema_version: TOOLPATH_SCHEMA_VERSION,
        program: "test".to_owned(),
        offsets,
        rapid_path: RapidPath::Linear,
        arc_tolerance: 0.01,
        path_tolerance: None,
        block_skip_executed: true,
        unmodelled_retracts: 0,
    }
}

fn linear(from: Vec3, to: Vec3, line: u32) -> MotionSegment {
    MotionSegment {
        kind: MotionKind::Linear,
        start: from,
        end: to,
        arc: None,
        orientation: None,
        tool: 1,
        feed: FeedSpec {
            value: 500.0,
            mode: FeedMode::UnitsPerMinute,
            spindle_rpm: Some(8000.0),
        },
        source: Provenance::new(0, line, line),
    }
}

#[test]
fn a_contiguous_path_is_accepted() {
    let segments = vec![
        linear(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 1),
        linear(Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 0.0), 2),
    ];
    let path = Toolpath::new(header(), segments, vec![]).expect("contiguous");
    assert_eq!(path.segment_count(), 2);
    assert!((path.length() - 20.0).abs() < 1e-12);
}

#[test]
fn a_gap_of_one_ulp_is_rejected() {
    // The contract says exactly equal, with no tolerance. In machine
    // coordinates there is no legitimate way for this to fail, so an
    // approximate check would only hide a bug — which is the whole reason the
    // IR is in machine coordinates at all. See ADR 0003.
    let joint = 10.0f64;
    let nudged = f64::from_bits(joint.to_bits() + 1);
    let segments = vec![
        linear(Vec3::new(0.0, 0.0, 0.0), Vec3::new(joint, 0.0, 0.0), 1),
        linear(Vec3::new(nudged, 0.0, 0.0), Vec3::new(20.0, 0.0, 0.0), 2),
    ];
    let err = Toolpath::new(header(), segments, vec![]).expect_err("one ULP is still a gap");
    match err {
        ToolpathError::Discontinuous { index, .. } => assert_eq!(index, 1),
        other => panic!("{other:?}"),
    }
    // And the message has to be diagnosable, not just correct.
    assert!(err.to_string().contains("continuous by construction"));
}

#[test]
fn a_non_finite_coordinate_never_reaches_the_ir() {
    // `Orientation::from_determinant` panics on NaN in release by design, so a
    // NaN that reaches the predicates aborts the process. The boundary is here.
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let segments = vec![linear(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(bad, 0.0, 0.0),
            1,
        )];
        let err = Toolpath::new(header(), segments, vec![]).expect_err("{bad} is not a coordinate");
        assert!(matches!(err, ToolpathError::NotFinite { .. }), "{err:?}");
    }
}

#[test]
fn a_non_finite_arc_field_is_rejected_too() {
    let mut segment = linear(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 1);
    segment.kind = MotionKind::Arc;
    segment.arc = Some(ArcData {
        center: Vec3::new(5.0, 0.0, 0.0),
        plane: ArcPlane::Xy,
        sweep: f64::NAN,
        radius: 5.0,
        form: ArcForm::CentreOffsets,
        radius_residual: 0.0,
    });
    let err = Toolpath::new(header(), vec![segment], vec![]).expect_err("NaN sweep");
    assert!(matches!(err, ToolpathError::NotFinite { .. }), "{err:?}");
}

#[test]
fn events_must_name_a_real_segment_and_stay_in_order() {
    let segments = vec![linear(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        1,
    )];

    let beyond = vec![PathEvent {
        at_segment: 7,
        kind: PathEventKind::Stop,
        source: Provenance::new(0, 1, 0),
    }];
    assert!(matches!(
        Toolpath::new(header(), segments.clone(), beyond).expect_err("no segment 7"),
        ToolpathError::EventOutOfRange { .. }
    ));

    let backwards = vec![
        PathEvent {
            at_segment: 1,
            kind: PathEventKind::Stop,
            source: Provenance::new(0, 2, 1),
        },
        PathEvent {
            at_segment: 0,
            kind: PathEventKind::ProgramEnd,
            source: Provenance::new(0, 3, 2),
        },
    ];
    assert!(matches!(
        Toolpath::new(header(), segments, backwards).expect_err("out of order"),
        ToolpathError::EventsOutOfOrder { .. }
    ));
}

#[test]
fn an_event_may_sit_after_the_last_segment() {
    // M30 lives there, so `at_segment == len` has to be legal.
    let segments = vec![linear(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        1,
    )];
    let events = vec![PathEvent {
        at_segment: 1,
        kind: PathEventKind::ProgramEnd,
        source: Provenance::new(0, 9, 8),
    }];
    assert!(Toolpath::new(header(), segments, events).is_ok());
}

#[test]
fn work_offset_ids_map_g54_to_one_not_to_fifty_four() {
    assert_eq!(
        WorkOffsetId::from_gcode(54, 0).map(WorkOffsetId::index),
        Some(1)
    );
    assert_eq!(
        WorkOffsetId::from_gcode(59, 0).map(WorkOffsetId::index),
        Some(6)
    );
    assert_eq!(
        WorkOffsetId::from_gcode(59, 3).map(WorkOffsetId::index),
        Some(9)
    );
    assert_eq!(
        WorkOffsetId::from_gcode(53, 0),
        None,
        "G53 is not an offset"
    );
    assert_eq!(WorkOffsetId::from_gcode(59, 4), None);

    // Round-trips, so a report never names a different offset from the one used.
    for (major, minor) in [(54, 0), (55, 0), (59, 0), (59, 1), (59, 3)] {
        let id = WorkOffsetId::from_gcode(major, minor).expect("valid");
        let expected = if minor == 0 {
            format!("G{major}")
        } else {
            format!("G{major}.{minor}")
        };
        assert_eq!(id.as_gcode(), expected);
    }
}

#[test]
fn the_zx_plane_normal_is_positive_y() {
    // G18 is the one that catches people: the plane is conventionally called
    // "XZ" but RS-274 orders it Z then X, precisely so the right-handed normal
    // is +Y. Reading it as X,Z gives -Y and turns every G2 into a G3.
    assert_eq!(ArcPlane::Zx.normal(), Vec3::new(0.0, 1.0, 0.0));
    assert_eq!(ArcPlane::Zx.axes(), [2, 0, 1]);
    assert_eq!(ArcPlane::Xy.normal(), Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(ArcPlane::Yz.normal(), Vec3::new(1.0, 0.0, 0.0));

    // The three normals must form a right-handed set, which is the property the
    // axis orders exist to guarantee.
    let x = ArcPlane::Yz.normal();
    let y = ArcPlane::Zx.normal();
    let z = ArcPlane::Xy.normal();
    assert_eq!(x.cross(y), z, "x cross y must be z");
    assert_eq!(y.cross(z), x, "y cross z must be x");
    assert_eq!(z.cross(x), y, "z cross x must be y");
}

#[test]
fn a_helix_is_longer_than_its_arc() {
    // The true length is the hypotenuse of the planar arc and the rise.
    // Treating it as the planar length understates a ramp's cutting time.
    let arc = ArcData {
        center: Vec3::new(0.0, 0.0, 0.0),
        plane: ArcPlane::Xy,
        sweep: core::f64::consts::TAU,
        radius: 10.0,
        form: ArcForm::CentreOffsets,
        radius_residual: 0.0,
    };
    let mut segment = linear(Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 5.0), 1);
    segment.kind = MotionKind::Helix;
    segment.arc = Some(arc);

    let planar = core::f64::consts::TAU * 10.0;
    let expected = (planar * planar + 25.0f64).sqrt();
    assert!(
        (segment.length() - expected).abs() < 1e-9,
        "{} vs {expected}",
        segment.length()
    );
    assert!(segment.length() > planar);
}

#[test]
fn a_full_circle_is_not_degenerate_even_though_it_returns_to_its_start() {
    let mut segment = linear(Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 1);
    segment.kind = MotionKind::Arc;
    segment.arc = Some(ArcData {
        center: Vec3::new(0.0, 0.0, 0.0),
        plane: ArcPlane::Xy,
        sweep: core::f64::consts::TAU,
        radius: 10.0,
        form: ArcForm::CentreOffsets,
        radius_residual: 0.0,
    });
    assert!(
        !segment.is_degenerate(),
        "a full circle starts and ends at the same point but is very much motion"
    );
    assert!((segment.length() - core::f64::consts::TAU * 10.0).abs() < 1e-9);
    assert_eq!(segment.chord(), 0.0, "its chord, however, is zero");
}

#[test]
fn arc_bounds_contain_the_whole_circle_and_are_never_too_small() {
    // Deliberately conservative: too large costs a little work, too small loses
    // material.
    let mut segment = linear(Vec3::new(10.0, 0.0, 3.0), Vec3::new(0.0, 10.0, 3.0), 1);
    segment.kind = MotionKind::Arc;
    segment.arc = Some(ArcData {
        center: Vec3::new(0.0, 0.0, 3.0),
        plane: ArcPlane::Xy,
        sweep: core::f64::consts::FRAC_PI_2,
        radius: 10.0,
        form: ArcForm::CentreOffsets,
        radius_residual: 0.0,
    });
    let bounds = segment.bounds();
    assert!((bounds.min.x + 10.0).abs() < 1e-12 && (bounds.max.x - 10.0).abs() < 1e-12);
    assert!((bounds.min.y + 10.0).abs() < 1e-12 && (bounds.max.y - 10.0).abs() < 1e-12);
    // Out of plane, the endpoints bound it: no spurious expansion in z.
    assert!((bounds.min.z - 3.0).abs() < 1e-12 && (bounds.max.z - 3.0).abs() < 1e-12);
}

#[test]
fn feed_duration_depends_on_the_mode_and_says_when_it_cannot_answer() {
    let distance = 100.0;

    let per_minute = FeedSpec {
        value: 500.0,
        mode: FeedMode::UnitsPerMinute,
        spindle_rpm: None,
    };
    assert_eq!(per_minute.duration_minutes(distance), Some(0.2));

    // G95 needs the spindle speed, and says so rather than guessing.
    let per_rev = FeedSpec {
        value: 0.1,
        mode: FeedMode::UnitsPerRevolution,
        spindle_rpm: None,
    };
    assert_eq!(per_rev.duration_minutes(distance), None);
    let per_rev_with_spindle = FeedSpec {
        spindle_rpm: Some(2000.0),
        ..per_rev
    };
    assert_eq!(per_rev_with_spindle.duration_minutes(distance), Some(0.5));

    // G93: the block takes 1/F minutes whatever its length, so distance is
    // irrelevant. That is the entire point of inverse time.
    let inverse = FeedSpec {
        value: 4.0,
        mode: FeedMode::InverseTime,
        spindle_rpm: None,
    };
    assert_eq!(inverse.duration_minutes(distance), Some(0.25));
    assert_eq!(inverse.duration_minutes(1.0), Some(0.25));

    // A rapid has no commanded rate at all.
    assert_eq!(FeedSpec::rapid().duration_minutes(distance), None);
    assert!(FeedSpec::rapid().is_rapid());
}

#[test]
fn provenance_distinguishes_the_three_motions_one_cycle_line_expands_to() {
    let base = Provenance::new(0, 42, 7);
    assert!(!base.is_from_cycle());
    let plunge = base.in_cycle(1);
    assert!(plunge.is_from_cycle());
    assert_eq!(plunge.line, 42, "the line is still the line");
    assert_eq!(plunge.cycle_step, 1);
    assert_ne!(
        base.in_cycle(0),
        base.in_cycle(1),
        "a gouge report that says line 42 three times makes the user work it out"
    );
}

#[test]
fn hashing_is_stable_and_reflects_every_field() {
    let path = Toolpath::new(
        header(),
        vec![linear(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            1,
        )],
        vec![PathEvent {
            at_segment: 0,
            kind: PathEventKind::ToolChange { tool: 3 },
            source: Provenance::new(0, 1, 0),
        }],
    )
    .expect("valid");

    let digest = |p: &Toolpath| {
        let mut h = CanonicalHash::new();
        h.add(p);
        h.finish().to_hex()
    };
    assert_eq!(digest(&path), digest(&path), "hashing is a function");

    // Every field that changes the meaning must change the hash. The rapid-path
    // policy is the interesting one: it is a header field that changes what the
    // segments *mean* without changing any of their numbers.
    let mut other = path.clone();
    other.header.rapid_path = RapidPath::Dogleg;
    assert_ne!(digest(&path), digest(&other));

    let mut retooled = path.clone();
    retooled.segments[0].tool = 2;
    assert_ne!(digest(&path), digest(&retooled));

    let mut moved = path.clone();
    moved.segments[0].source = Provenance::new(0, 999, 0);
    assert_ne!(
        digest(&path),
        digest(&moved),
        "provenance is part of the answer, not decoration"
    );
}

#[test]
fn orientation_is_reserved_and_empty_in_this_unit() {
    let path = Toolpath::new(
        header(),
        vec![linear(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            1,
        )],
        vec![],
    )
    .expect("valid");
    assert!(
        path.segments.iter().all(|s| s.orientation.is_none()),
        "5-axis work populates this; until then it must be uniformly None so that the \
         golden hashes it will move are unambiguous"
    );
}
