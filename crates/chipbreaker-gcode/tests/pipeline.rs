// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The coordinate resolution pipeline, verified per stage against numbers
//! computed by hand.
//!
//! Every coordinate in every downstream unit comes out of this, so each stage is
//! checked on its own before the stages are checked together, and the three
//! landmines — `G53`, `G28`/`G30`, and a mid-program `G10 L2` — each get a case.

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{MotionKind, PathEventKind, Toolpath, WorkOffsetId};
use chipbreaker_gcode::diag::{Diagnostics, GcodeError};
use chipbreaker_gcode::modal::Units;
use chipbreaker_gcode::resolve::{ParseOptions, ParseStats, parse};

fn run(text: &str) -> (Toolpath, Diagnostics, ParseStats) {
    parse(text, "test", &ParseOptions::default(), None).expect("should parse")
}

fn run_with(text: &str, options: &ParseOptions) -> (Toolpath, Diagnostics, ParseStats) {
    parse(text, "test", options, None).expect("should parse")
}

fn fails(text: &str) -> GcodeError {
    parse(text, "test", &ParseOptions::default(), None).expect_err("should be refused")
}

fn ends_at(path: &Toolpath) -> Vec3 {
    path.segments.last().expect("a segment").end
}

// --- stage by stage --------------------------------------------------------

#[test]
fn stage_one_units_are_applied_exactly_once() {
    // G20 with X1. is 25.4 mm, not 25.4 twice over and not 1.
    let (path, _, _) = run("G21 G90 G0 X0. Y0. Z0.\nG20 G0 X1.\n");
    assert!(
        (ends_at(&path).x - 25.4).abs() < 1e-12,
        "{:?}",
        ends_at(&path)
    );

    // And back again: the change is modal, so the next block is inches too.
    let (path, _, _) = run("G20 G90 G0 X1.\nG0 X2.\n");
    assert!((ends_at(&path).x - 50.8).abs() < 1e-12);
}

#[test]
fn stage_two_incremental_adds_to_where_the_tool_is() {
    let (path, _, _) = run("G90 G0 X10. Y20.\nG91 G0 X5. Y-5.\nG0 X5.\n");
    let end = ends_at(&path);
    assert!(
        (end.x - 20.0).abs() < 1e-12 && (end.y - 15.0).abs() < 1e-12,
        "{end:?}"
    );
}

#[test]
fn stage_two_incremental_needs_no_frame_reasoning() {
    // A delta is a delta in every frame. The same G91 move must land in the
    // same place whether or not a work offset is active, which is the property
    // the resolver's module header relies on.
    let with_offset = run("G10 L2 P2 X100. Y50.\nG55 G90 G0 X0. Y0.\nG91 G0 X7. Y3.\n");
    let plain = run("G90 G0 X100. Y50.\nG91 G0 X7. Y3.\n");
    assert_eq!(
        ends_at(&with_offset.0),
        ends_at(&plain.0),
        "an incremental move must not depend on the active offset"
    );
}

#[test]
fn stage_three_absolute_coordinates_traverse_the_work_offset() {
    // G54 at machine (-250, -100, -300): programmed X0 Y0 Z0 is that point.
    let (path, _, _) = run("G10 L2 P1 X-250. Y-100. Z-300.\nG54 G90 G0 X0. Y0. Z0.\nG0 X10.\n");
    let end = ends_at(&path);
    assert!(
        (end.x + 240.0).abs() < 1e-12 && (end.y + 100.0).abs() < 1e-12,
        "{end:?}"
    );
}

#[test]
fn stage_four_g92_shifts_persistently_and_survives_an_offset_change() {
    // G92 makes the current position read as the programmed value, and it is a
    // shift rather than a move: nothing travels.
    let (path, _, _) = run("G10 L2 P1 X-100. Y0. Z0.\n\
         G54 G90 G0 X10. Y0.\n\
         G92 X0.\n\
         G0 X5.\n");
    // After G92 X0 at machine -90, programmed 0 means machine -90, so
    // programmed 5 means machine -85.
    let end = ends_at(&path);
    assert!((end.x + 85.0).abs() < 1e-12, "{end:?}");

    // And it is cancellable.
    let (path, _, _) = run("G10 L2 P1 X-100. Y0. Z0.\n\
         G54 G90 G0 X10. Y0.\n\
         G92 X0.\n\
         G92.1\n\
         G0 X5.\n");
    assert!(
        (ends_at(&path).x + 95.0).abs() < 1e-12,
        "{:?}",
        ends_at(&path)
    );
}

// --- landmine one: G53 -----------------------------------------------------

#[test]
fn g53_bypasses_the_work_offset_and_the_g92_shift_for_one_block_only() {
    let (path, _, _) = run("G10 L2 P1 X-250. Y-100. Z0.\n\
         G54 G90 G0 X0. Y0.\n\
         G53 G0 X-10. Y-10.\n\
         G0 X0. Y0.\n");
    // Three segments: to the G54 origin, to machine (-10,-10), back to the G54
    // origin. The middle one is in machine coordinates.
    assert_eq!(path.segments.len(), 3);
    assert!(
        (path.segments[1].end.x + 10.0).abs() < 1e-12
            && (path.segments[1].end.y + 10.0).abs() < 1e-12,
        "G53 is machine coordinates: {:?}",
        path.segments[1].end
    );
    // And it is non-modal: the block after it is back in G54.
    assert!(
        (path.segments[2].end.x + 250.0).abs() < 1e-12,
        "G53 must not persist: {:?}",
        path.segments[2].end
    );
}

// --- landmine two: G28 / G30 -----------------------------------------------

#[test]
fn g28_is_two_moves_via_the_intermediate_point_not_one() {
    // Collapsing these into a single straight move to the reference point is
    // how a simulation reports clearance through a fixture the real machine
    // would have hit.
    let (path, _, stats) = run("G90 G0 X50. Y50. Z-20.\nG28 Z0.\n");
    assert_eq!(
        stats.segments, 3,
        "one move to position, then two for the G28"
    );
    // The intermediate point takes Z from the block and keeps X and Y.
    let intermediate = path.segments[1].end;
    assert!(
        (intermediate.x - 50.0).abs() < 1e-12 && (intermediate.z - 0.0).abs() < 1e-12,
        "intermediate {intermediate:?}"
    );
    // Then the reference point.
    assert_eq!(path.segments[2].end, Vec3::ZERO);
    assert!(path.segments.iter().all(|s| s.kind == MotionKind::Rapid));
}

// --- landmine three: mid-program G10 L2 ------------------------------------

#[test]
fn a_mid_program_g10_leaves_earlier_geometry_correct_and_records_both_epochs() {
    // The segments are in machine coordinates, so rewriting an offset partway
    // cannot disturb what came before. But a report rendering into a workpiece
    // frame must use the value in force *then*, so the header keeps both.
    let (path, _, _) = run("G10 L2 P1 X-100. Y0. Z0.\n\
         G54 G90 G0 X10. Y0.\n\
         G10 L2 P1 X-200. Y0. Z0.\n\
         G0 X10.\n");

    // First move: offset -100, programmed 10 -> machine -90.
    assert!((path.segments[0].end.x + 90.0).abs() < 1e-12);
    // Second: offset -200, programmed 10 -> machine -190. Earlier geometry
    // untouched.
    assert!((path.segments[1].end.x + 190.0).abs() < 1e-12);
    assert!(
        (path.segments[0].end.x + 90.0).abs() < 1e-12,
        "the first segment must not have moved"
    );

    let g54 = WorkOffsetId::from_gcode(54, 0).expect("G54");
    let epochs = path.header.offsets.get(&g54).expect("G54 in the table");
    assert!(
        epochs.len() >= 3,
        "power-up, then two G10 rewrites: {epochs:?}"
    );
    assert!(
        epochs
            .windows(2)
            .all(|w| w[0].from_segment <= w[1].from_segment),
        "epochs must be ordered by segment"
    );
    // And the rewrite is announced.
    assert!(
        path.events
            .iter()
            .any(|e| matches!(e.kind, PathEventKind::WorkOffsetRedefined { .. })),
        "a G10 rewrite must be visible in the events"
    );
}

// --- everything else in this increment -------------------------------------

#[test]
fn segments_are_exactly_contiguous() {
    let (path, _, _) = run("G90 G21 G54 G0 X0. Y0. Z5.\n\
         G1 Z-1. F300.\n\
         G1 X20.\n\
         G2 X30. Y10. I0. J10.\n\
         G1 Y30.\n\
         G0 Z5.\n");
    for pair in path.segments.windows(2) {
        assert_eq!(
            pair[0].end, pair[1].start,
            "contiguity is exact, with no tolerance"
        );
    }
}

#[test]
fn a_zero_length_move_is_dropped_and_counted() {
    let (path, diagnostics, stats) = run("G90 G0 X10.\nG0 X10.\nG0 X20.\n");
    assert_eq!(stats.segments, 2);
    assert_eq!(stats.dropped_zero_length, 1);
    assert_eq!(diagnostics.count_of("zero-length-move"), 1);
    assert!(path.segments.iter().all(|s| s.chord() > 0.0));
}

#[test]
fn a_full_circle_is_kept_even_though_its_chord_is_zero() {
    // The finding from the IR work, now exercised through the parser: start and
    // end coincide but the sweep does not vanish.
    let (path, _, stats) = run("G90 G21 G17 G0 X10. Y0.\nG1 F100.\nG2 X10. Y0. I-10. J0.\n");
    assert_eq!(stats.dropped_zero_length, 0, "a full circle is not nothing");
    let circle = path.segments.last().expect("the arc");
    assert_eq!(circle.kind, MotionKind::Arc);
    assert!((circle.chord() - 0.0).abs() < 1e-12);
    assert!(
        (circle.length() - core::f64::consts::TAU * 10.0).abs() < 1e-9,
        "{}",
        circle.length()
    );
}

#[test]
fn a_helix_is_recognised_as_one() {
    let (path, _, _) = run("G90 G21 G17 G0 X10. Y0. Z0.\nG1 F100.\nG3 X10. Y0. Z5. I-10. J0.\n");
    let helix = path.segments.last().expect("the helix");
    assert_eq!(helix.kind, MotionKind::Helix);
    assert!(helix.length() > core::f64::consts::TAU * 10.0);
}

#[test]
fn an_axis_word_without_a_decimal_point_is_refused_unless_told_otherwise() {
    // X10 on a legacy control is 0.010 mm. A factor of a thousand that parses.
    match fails("G90 G0 X10\n") {
        GcodeError::MissingDecimalPoint { word, .. } => assert_eq!(word, "X10"),
        other => panic!("{other:?}"),
    }
    // X0 is exempt: zero is zero in any increment.
    let (path, _, _) = run("G90 G0 X0. Y0.\nG0 X0 Y5.\n");
    assert!((ends_at(&path).y - 5.0).abs() < 1e-12);

    // And the escape hatch means what it says.
    let options = ParseOptions {
        legacy_increment: Some(0.001),
        ..ParseOptions::default()
    };
    let (path, _, _) = run_with("G90 G0 X10\n", &options);
    assert!(
        (ends_at(&path).x - 0.01).abs() < 1e-12,
        "ten increments of a thousandth: {:?}",
        ends_at(&path)
    );
}

#[test]
fn a_feed_move_before_any_f_word_is_refused() {
    assert!(matches!(
        fails("G90 G1 X10.\n"),
        GcodeError::NoFeedRate { .. }
    ));
    // A rapid needs none.
    assert!(run("G90 G0 X10.\n").0.segments.len() == 1);
}

#[test]
fn inverse_time_feed_is_not_scaled_by_the_unit_factor() {
    // G93's F is a reciprocal duration, not a distance rate. Multiplying it by
    // 25.4 would be nonsense.
    let (path, _, _) = run("G20 G93 G90 G1 X1. F4.\n");
    let feed = path.segments[0].feed;
    assert_eq!(feed.mode, chipbreaker_core::toolpath::FeedMode::InverseTime);
    assert!((feed.value - 4.0).abs() < 1e-12, "{}", feed.value);
    assert_eq!(feed.duration_minutes(999.0), Some(0.25));

    // Whereas G94 in inches is scaled.
    let (path, _, _) = run("G20 G94 G90 G1 X1. F10.\n");
    assert!((path.segments[0].feed.value - 254.0).abs() < 1e-12);
}

#[test]
fn the_rapid_path_policy_is_recorded_in_the_header() {
    let (path, _, _) = run("G90 G0 X10.\n");
    assert_eq!(
        path.header.rapid_path,
        chipbreaker_core::toolpath::RapidPath::Linear
    );
    let options = ParseOptions {
        rapid_path: chipbreaker_core::toolpath::RapidPath::Dogleg,
        ..ParseOptions::default()
    };
    let (path, _, _) = run_with("G90 G0 X10.\n", &options);
    assert_eq!(
        path.header.rapid_path,
        chipbreaker_core::toolpath::RapidPath::Dogleg,
        "a collision report is only as trustworthy as the path it was computed against"
    );
}

#[test]
fn provenance_names_the_line_that_produced_each_segment() {
    let (path, _, _) = run("G90 G21\nG0 X10.\nG1 X20. F100.\nG0 X30.\n");
    let lines: Vec<u32> = path.segments.iter().map(|s| s.source.line).collect();
    assert_eq!(lines, vec![2, 3, 4]);
    assert!(path.segments.iter().all(|s| !s.source.is_from_cycle()));
}

#[test]
fn a_units_change_mid_program_is_warned_about() {
    let (_, diagnostics, _) = run("G21 G90 G0 X10.\nG20 G0 X1.\n");
    assert_eq!(diagnostics.count_of("units-changed"), 1);
}

#[test]
fn block_skip_is_obeyed_and_recorded() {
    let executed = ParseOptions::default();
    let (path, _, _) = run_with("G90 G0 X10.\n/G0 X20.\nG0 X30.\n", &executed);
    assert_eq!(path.segments.len(), 3);

    let skipped = ParseOptions {
        execute_block_skip: false,
        ..ParseOptions::default()
    };
    let (path, diagnostics, stats) = run_with("G90 G0 X10.\n/G0 X20.\nG0 X30.\n", &skipped);
    assert_eq!(path.segments.len(), 2);
    assert_eq!(stats.skipped_blocks, 1);
    assert_eq!(diagnostics.count_of("block-skip"), 1);
    assert!(!path.header.block_skip_executed);
}

#[test]
fn a_canned_cycle_expands_rather_than_vanishing() {
    // This began life asserting that cycles were *refused*, while Increment B
    // was still ahead. The point was the same either way: a program whose
    // drilling silently disappeared would verify as clean.
    let (path, _, stats) = run("G90 G21 G0 X0. Y0. Z5.
G81 X10. Y10. Z-5. R2. F100.
G80
");
    assert!(stats.cycle_segments > 0, "the cycle must produce motion");
    assert!(
        path.segments
            .iter()
            .any(|s| s.kind == MotionKind::Linear && (s.end.z + 5.0).abs() < 1e-12),
        "and it must reach the programmed depth"
    );
}

#[test]
fn tool_and_spindle_and_stops_become_events() {
    let (path, _, _) = run("T3 M6\nS8000 M3\nG90 G0 X10.\nM5\nM30\n");
    let kinds: Vec<&str> = path.events.iter().map(|e| e.kind.as_str()).collect();
    assert!(kinds.contains(&"tool-change"));
    assert!(kinds.contains(&"spindle"));
    assert!(kinds.contains(&"program-end"));
    assert_eq!(path.segments[0].tool, 3, "the segment carries the tool");
    assert!(
        path.events
            .windows(2)
            .all(|w| w[0].at_segment <= w[1].at_segment),
        "events stay in segment order"
    );
}

#[test]
fn an_unmodelled_m_code_is_recorded_rather_than_refused() {
    let (path, diagnostics, _) = run("M55\nG90 G0 X10.\n");
    assert_eq!(diagnostics.count_of("unmodelled-m-code"), 1);
    assert!(
        path.events
            .iter()
            .any(|e| matches!(e.kind, PathEventKind::UnmodelledMCode { code: 55 }))
    );
}

#[test]
fn strict_promotes_the_first_warning_to_an_error() {
    let lenient = ParseOptions::default();
    assert!(parse("M55\nG90 G0 X10.\n", "t", &lenient, None).is_ok());

    let strict = ParseOptions {
        strict: true,
        ..ParseOptions::default()
    };
    let err = parse("M55\nG90 G0 X10.\n", "t", &strict, None).expect_err("strict");
    assert!(matches!(err, GcodeError::Strict { .. }));
    assert!(err.to_string().contains("--strict"));
}

#[test]
fn the_default_units_option_is_honoured_before_the_program_says_otherwise() {
    let inches = ParseOptions {
        default_units: Units::Inches,
        ..ParseOptions::default()
    };
    let (path, _, _) = run_with("G90 G0 X1.\n", &inches);
    assert!((ends_at(&path).x - 25.4).abs() < 1e-12);
}

#[test]
fn no_nan_ever_reaches_the_ir() {
    // Belt and braces: the IR rejects them at construction, and nothing here
    // should be able to produce one in the first place.
    let (path, _, _) = run("G90 G21 G54 G0 X0. Y0. Z5.\n\
         G1 Z-1. F300.\n\
         G2 X10. Y10. I10. J0.\n\
         G3 X0. Y0. I-10. J0.\n");
    for segment in &path.segments {
        assert!(segment.start.is_finite() && segment.end.is_finite());
        if let Some(arc) = &segment.arc {
            assert!(arc.is_finite());
        }
    }
}
