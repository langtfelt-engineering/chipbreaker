// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Canned cycles, checked against the longhand a programmer would have written.
//!
//! # Why this is the strongest test in the unit
//!
//! A cycle is a compression of motion that every control expands slightly
//! differently. Testing an expansion against *itself* — against a golden file
//! produced by the same code — proves only that it has not changed. Testing it
//! against the longhand proves it is right, because longhand has nowhere to hide
//! a dialect assumption: `G0 Z2.` means one thing and only one thing.
//!
//! Geometry is compared rather than whole segments, because provenance
//! deliberately differs: an expanded motion carries a `cycle_step` and a
//! longhand one does not. That difference is the point of `cycle_step`, so a
//! test that demanded full equality would be demanding the feature away.

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{MotionKind, Toolpath};
use chipbreaker_gcode::diag::GcodeError;
use chipbreaker_gcode::resolve::{ParseOptions, parse};

fn run(text: &str) -> Toolpath {
    parse(text, "test", &ParseOptions::default(), None)
        .unwrap_or_else(|e| panic!("should parse: {e}"))
        .0
}

fn fails(text: &str) -> GcodeError {
    parse(text, "test", &ParseOptions::default(), None).expect_err("should be refused")
}

/// The geometry of a path: what a cycle and its longhand must share.
fn geometry(path: &Toolpath) -> Vec<(MotionKind, Vec3, Vec3)> {
    path.segments
        .iter()
        .map(|s| (s.kind, s.start, s.end))
        .collect()
}

/// Asserts a cycle expands to exactly its longhand equivalent.
fn same_geometry(name: &str, cycle: &str, longhand: &str) {
    let by_cycle = run(cycle);
    let by_hand = run(longhand);
    let (a, b) = (geometry(&by_cycle), geometry(&by_hand));
    assert_eq!(
        a.len(),
        b.len(),
        "{name}: the cycle produced {} motions and the longhand {}\n\
         cycle:    {a:#?}\n\
         longhand: {b:#?}",
        a.len(),
        b.len()
    );
    for (i, (from_cycle, from_hand)) in a.iter().zip(&b).enumerate() {
        assert_eq!(
            from_cycle, from_hand,
            "{name}: motion {i} differs\n  cycle    {from_cycle:?}\n  longhand {from_hand:?}"
        );
    }
}

const PREAMBLE: &str = "G90 G21 G17 G54 G0 X0. Y0. Z10.\nF100.\n";

#[test]
fn g81_expands_to_its_longhand() {
    same_geometry(
        "G81 G98",
        &format!("{PREAMBLE}G98 G81 X20. Y30. Z-5. R2.\nG80\n"),
        &format!("{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z10.\n"),
    );
}

#[test]
fn g99_retracts_to_the_r_plane_and_g98_to_the_initial_z() {
    // The difference that changes every intermediate retract in a pattern, and
    // therefore whether the tool clears a clamp.
    same_geometry(
        "G81 G99",
        &format!("{PREAMBLE}G99 G81 X20. Y30. Z-5. R2.\nG80\n"),
        &format!("{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z2.\n"),
    );

    // And across two holes the difference compounds.
    same_geometry(
        "G81 G99 two holes",
        &format!("{PREAMBLE}G99 G81 X20. Y30. Z-5. R2.\nX40.\nG80\n"),
        &format!(
            "{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z2.\n\
             G0 X40. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z2.\n"
        ),
    );
    same_geometry(
        "G81 G98 two holes",
        &format!("{PREAMBLE}G98 G81 X20. Y30. Z-5. R2.\nX40.\nG80\n"),
        &format!(
            "{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z10.\n\
             G0 X40. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z10.\n"
        ),
    );
}

#[test]
fn g82_matches_g81_geometrically_and_adds_a_dwell_event() {
    // A dwell removes no material, so the geometry is a G81's.
    same_geometry(
        "G82",
        &format!("{PREAMBLE}G98 G82 X20. Y30. Z-5. R2. P0.5\nG80\n"),
        &format!("{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z10.\n"),
    );
    let path = run(&format!("{PREAMBLE}G98 G82 X20. Y30. Z-5. R2. P0.5\nG80\n"));
    assert!(
        path.events.iter().any(|e| e.kind.as_str() == "dwell"),
        "the dwell must still be recorded, even though it moves nothing"
    );
}

#[test]
fn g83_pecks_with_a_full_retract_between_each() {
    // From R at 2 down to -5 in 3 mm pecks: -1, -4, -5.
    same_geometry(
        "G83",
        &format!("{PREAMBLE}G98 G83 X20. Y30. Z-5. R2. Q3.\nG80\n"),
        &format!(
            "{PREAMBLE}\
             G0 X20. Y30.\n\
             G0 Z2.\n\
             G1 Z-1.\n\
             G0 Z2.\n\
             G0 Z-1.\n\
             G1 Z-4.\n\
             G0 Z2.\n\
             G0 Z-4.\n\
             G1 Z-5.\n\
             G0 Z10.\n"
        ),
    );
}

#[test]
fn g73_pecks_without_a_retract_because_the_clearance_is_a_machine_parameter() {
    // Documented in the module header: the chip-break retract goes into space
    // already cut and removes nothing, and inventing its size would put a rapid
    // in the IR that the machine may not make.
    same_geometry(
        "G73",
        &format!("{PREAMBLE}G98 G73 X20. Y30. Z-5. R2. Q3.\nG80\n"),
        &format!(
            "{PREAMBLE}\
             G0 X20. Y30.\n\
             G0 Z2.\n\
             G1 Z-1.\n\
             G1 Z-4.\n\
             G1 Z-5.\n\
             G0 Z10.\n"
        ),
    );
}

#[test]
fn g85_and_the_tapping_cycles_retract_at_feed() {
    for code in ["G85", "G84", "G74"] {
        same_geometry(
            code,
            &format!("{PREAMBLE}G98 {code} X20. Y30. Z-5. R2.\nG80\n"),
            &format!("{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG1 Z10.\n"),
        );
    }
}

#[test]
fn g86_retracts_rapid_after_boring() {
    same_geometry(
        "G86",
        &format!("{PREAMBLE}G98 G86 X20. Y30. Z-5. R2.\nG80\n"),
        &format!("{PREAMBLE}G0 X20. Y30.\nG0 Z2.\nG1 Z-5.\nG0 Z10.\n"),
    );
}

#[test]
fn a_cycle_persists_until_g80_and_fires_on_any_block_with_axis_words() {
    // Three holes from one cycle line and two bare position lines.
    let path = run(&format!(
        "{PREAMBLE}G99 G81 X10. Y0. Z-5. R2.\nX20.\nX30.\nG80\nG0 X0.\n"
    ));
    let holes: Vec<f64> = path
        .segments
        .iter()
        .filter(|s| s.kind == MotionKind::Linear)
        .map(|s| s.end.x)
        .collect();
    assert_eq!(holes, vec![10.0, 20.0, 30.0], "three holes");
}

#[test]
fn a_block_with_only_a_feed_word_does_not_fire_the_cycle() {
    // The distinction that stops a program's F line drilling a spurious hole.
    let with_f = run(&format!(
        "{PREAMBLE}G99 G81 X10. Y0. Z-5. R2.\nF200.\nG80\n"
    ));
    let without = run(&format!("{PREAMBLE}G99 G81 X10. Y0. Z-5. R2.\nG80\n"));
    assert_eq!(
        geometry(&with_f).len(),
        geometry(&without).len(),
        "an F word alone must not fire the cycle again"
    );
}

#[test]
fn g80_cancels_and_a_later_axis_word_is_an_ordinary_move() {
    let path = run(&format!(
        "{PREAMBLE}G99 G81 X10. Y0. Z-5. R2.\nG80\nG0 X50.\n"
    ));
    let last = path.segments.last().expect("a segment");
    assert_eq!(last.kind, MotionKind::Rapid);
    assert!((last.end.x - 50.0).abs() < 1e-12);
    assert!(
        !last.source.is_from_cycle(),
        "after G80 nothing is part of a cycle"
    );
}

#[test]
fn a_repeat_count_with_g91_produces_a_bolt_pattern_from_one_line() {
    same_geometry(
        "G81 L3 under G91",
        &format!("{PREAMBLE}G99 G91 G81 X10. Y0. Z-7. R-8. L3\nG80\nG90\n"),
        &format!(
            "{PREAMBLE}G91\n\
             G0 X10. Y0.\nG0 Z-8.\nG1 Z-7.\nG0 Z7.\n\
             G0 X10. Y0.\nG0 Z0.\nG1 Z-7.\nG0 Z7.\n\
             G0 X10. Y0.\nG0 Z0.\nG1 Z-7.\nG0 Z7.\n\
             G90\n"
        ),
    );
}

#[test]
fn l_zero_means_do_not_execute() {
    // A real case, and one that reads exactly like a typo.
    //
    // Compared against the preamble rather than against emptiness: the preamble
    // itself commands a move, so "no segments at all" would be asserting that
    // L0 undoes the lines before it.
    let with_l0 = run(&format!("{PREAMBLE}G99 G81 X10. Y0. Z-5. R2. L0\nG80\n"));
    let preamble_only = run(PREAMBLE);
    assert_eq!(
        geometry(&with_l0),
        geometry(&preamble_only),
        "L0 must add nothing at all beyond the preamble"
    );
}

#[test]
fn the_r_word_is_measured_from_the_initial_z_under_g91() {
    // Absolute R would put the retract plane somewhere else entirely.
    let incremental = run(&format!(
        "{PREAMBLE}G99 G91 G81 X10. Y0. Z-7. R-8.\nG80\nG90\n"
    ));
    let retract = incremental.segments.last().expect("a retract").end.z;
    assert!(
        (retract - 2.0).abs() < 1e-12,
        "initial Z 10 minus 8 is an R plane at 2, got {retract}"
    );
}

#[test]
fn a_cycle_expansion_is_traceable_to_its_line_and_its_step() {
    // `cycle_step` earns itself here: one G81 line becomes several motions, and
    // a gouge report that says "line 3" three times makes the user work out
    // which was at fault.
    let path = run(&format!("{PREAMBLE}G98 G81 X20. Y30. Z-5. R2.\nG80\n"));
    let expanded: Vec<_> = path
        .segments
        .iter()
        .filter(|s| s.source.is_from_cycle())
        .collect();
    assert_eq!(expanded.len(), 4, "position, plunge to R, feed, retract");

    let lines: Vec<u32> = expanded.iter().map(|s| s.source.line).collect();
    assert!(
        lines.windows(2).all(|w| w[0] == w[1]),
        "all four name the same line: {lines:?}"
    );
    let steps: Vec<u32> = expanded.iter().map(|s| s.source.cycle_step).collect();
    assert_eq!(steps, vec![0, 1, 2, 3], "and are distinguishable by step");
}

#[test]
fn a_cycle_without_a_feed_rate_is_refused() {
    assert!(matches!(
        fails("G90 G21 G54 G0 X0. Y0. Z10.\nG98 G81 X20. Z-5. R2.\nG80\n"),
        GcodeError::NoFeedRate { .. }
    ));
}

#[test]
fn every_cycle_leaves_the_path_contiguous() {
    for cycle in ["G81", "G82 P0.2", "G83 Q3.", "G73 Q3.", "G85", "G86", "G84"] {
        let path = run(&format!(
            "{PREAMBLE}G98 {cycle} X20. Y30. Z-5. R2.\nX40.\nG80\n"
        ));
        for pair in path.segments.windows(2) {
            assert_eq!(
                pair[0].end, pair[1].start,
                "{cycle}: contiguity is exact even through an expansion"
            );
        }
    }
}

// --- subprograms ----------------------------------------------------------

#[test]
fn a_subprogram_runs_its_body_and_returns() {
    // The body is the same geometry whether called or written out.
    same_geometry(
        "M98 once",
        "G90 G21 G0 X0. Y0. Z10.\nF100.\nM98 P100\nG0 X50.\nM30\nO100\nG1 X10.\nG1 Y10.\nM99\n",
        "G90 G21 G0 X0. Y0. Z10.\nF100.\nG1 X10.\nG1 Y10.\nG0 X50.\nM30\n",
    );
}

#[test]
fn a_repeat_count_runs_the_body_that_many_times() {
    let path =
        run("G90 G91.1 G21 G0 X0. Y0. Z10.\nF100.\nG91\nM98 P100 L3\nM30\nO100\nG1 X10.\nM99\n");
    let feeds: Vec<f64> = path
        .segments
        .iter()
        .filter(|s| s.kind == MotionKind::Linear)
        .map(|s| s.end.x)
        .collect();
    assert_eq!(feeds, vec![10.0, 20.0, 30.0], "three passes, each +10");
}

#[test]
fn a_subprogram_may_call_another() {
    let path = run("G90 G21 G0 X0. Y0. Z10.\nF100.\nM98 P100\nM30\n\
         O100\nG1 X10.\nM98 P200\nM99\n\
         O200\nG1 Y20.\nM99\n");
    let ends: Vec<(f64, f64)> = path
        .segments
        .iter()
        .filter(|s| s.kind == MotionKind::Linear)
        .map(|s| (s.end.x, s.end.y))
        .collect();
    assert_eq!(ends, vec![(10.0, 0.0), (10.0, 20.0)]);
}

#[test]
fn runaway_nesting_is_refused_by_name() {
    // A subprogram that calls itself would otherwise recurse until the stack
    // gave out, which is a crash rather than a diagnosis.
    let err = fails("G90 G21 G0 X0. Y0.\nM98 P100\nM30\nO100\nM98 P100\nM99\n");
    match err {
        GcodeError::SubprogramTooDeep { limit, .. } => assert!(limit > 0),
        other => panic!("{other:?}"),
    }
}

#[test]
fn calling_a_subprogram_that_does_not_exist_is_refused_by_name() {
    match fails("G90 G21 G0 X0. Y0.\nM98 P999\nM30\n") {
        GcodeError::UnknownSubprogram { number, .. } => assert_eq!(number, 999),
        other => panic!("{other:?}"),
    }
}

#[test]
fn execution_stops_at_m30_rather_than_falling_into_the_subprogram_bodies() {
    // Subprogram bodies sit after M30 in a Fanuc file. Running off the end into
    // them would drill every hole a second time.
    let path = run("G90 G21 G0 X0. Y0. Z10.\nF100.\nG1 X5.\nM30\nO100\nG1 X999.\nM99\n");
    assert!(
        path.segments.iter().all(|s| s.end.x < 100.0),
        "the body after M30 must not run: {:?}",
        geometry(&path)
    );
}

// --- G73's chip-break retract ---------------------------------------------

#[test]
fn g73_without_a_clearance_omits_the_retract_and_says_so_structurally() {
    // The omission is exact for material removal -- the retract goes into space
    // already cut -- but it is NOT invisible to a collision check, and a warning
    // in a list is too easy for a downstream unit to ignore. So it is counted in
    // the header, where a certification step can refuse against it.
    let path = run(&format!("{PREAMBLE}G98 G73 X20. Y30. Z-5. R2. Q3.\nG80\n"));
    assert_eq!(
        path.header.unmodelled_retracts, 1,
        "the omission must be structural, not merely a warning"
    );
    // Geometrically a straight plunge, broken at the peck boundaries.
    let plunges: Vec<f64> = path
        .segments
        .iter()
        .filter(|s| s.kind == MotionKind::Linear)
        .map(|s| s.end.z)
        .collect();
    assert_eq!(plunges, vec![-1.0, -4.0, -5.0]);
    assert!(
        path.segments
            .iter()
            .all(|s| s.kind != MotionKind::Rapid || s.end.z >= 2.0 || s.start.z >= 2.0),
        "no intermediate rapid should appear inside the hole"
    );
}

#[test]
fn g73_with_a_clearance_emits_the_real_oscillation_and_the_counter_stays_zero() {
    let path = run(&format!("{PREAMBLE}G98 G73 X20. Y30. Z-5. R2. Q3.\nG80\n"));
    let supplied = parse(
        &format!("{PREAMBLE}G98 G73 X20. Y30. Z-5. R2. Q3.\nG80\n"),
        "test",
        &ParseOptions {
            chip_break_clearance: Some(0.5),
            ..ParseOptions::default()
        },
        None,
    )
    .expect("parses")
    .0;

    assert_eq!(
        supplied.header.unmodelled_retracts, 0,
        "supplying the parameter means nothing is omitted"
    );
    assert!(
        supplied.segments.len() > path.segments.len(),
        "the oscillation is extra motion: {} against {}",
        supplied.segments.len(),
        path.segments.len()
    );

    // Up by the clearance from each peck bottom, then back down to it.
    let inside: Vec<(f64, f64)> = supplied
        .segments
        .iter()
        .filter(|s| s.kind == MotionKind::Rapid && s.start.z < 2.0 && s.end.z < 2.0)
        .map(|s| (s.start.z, s.end.z))
        .collect();
    assert_eq!(
        inside,
        vec![(-1.0, -0.5), (-0.5, -1.0), (-4.0, -3.5), (-3.5, -4.0)]
    );

    // And the material removed is unchanged, which is the whole point: the
    // difference is a collision-checking one, not a geometric one.
    assert_eq!(
        path.segments.last().expect("a segment").end,
        supplied.segments.last().expect("a segment").end
    );
}

#[test]
fn only_g73_counts_as_an_unmodelled_retract() {
    // G83's retract is unambiguous and is modelled, so it must not be counted.
    for cycle in ["G81", "G83 Q3.", "G85"] {
        let path = run(&format!("{PREAMBLE}G98 {cycle} X20. Y30. Z-5. R2.\nG80\n"));
        assert_eq!(path.header.unmodelled_retracts, 0, "{cycle} omits nothing");
    }
}

#[test]
fn a_g73_without_a_q_word_is_a_plain_drill_and_omits_nothing() {
    // No peck depth means no pecking, so there is no retract to be missing.
    let path = run(&format!("{PREAMBLE}G98 G73 X20. Y30. Z-5. R2.\nG80\n"));
    assert_eq!(path.header.unmodelled_retracts, 0);
}
