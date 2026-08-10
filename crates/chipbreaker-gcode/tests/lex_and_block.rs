// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Stages one and two, tested apart from the rest.
//!
//! Keeping the lexer and the block assembler separately testable is the whole
//! reason they are separate stages: against a real NC file the question is
//! nearly always "which stage got this wrong", and that is only answerable if
//! each can be run on its own.

use chipbreaker_gcode::block::{ModalGroup, assemble, g_group, m_group, render_code};
use chipbreaker_gcode::diag::{Diagnostics, ForeignDialect, GcodeError};
use chipbreaker_gcode::lex::lex;

fn lex_ok(text: &str) -> (Vec<chipbreaker_gcode::lex::RawBlock>, Diagnostics) {
    let mut diagnostics = Diagnostics::new();
    let blocks = lex(text, 0, &mut diagnostics).expect("should lex");
    (blocks, diagnostics)
}

fn lex_err(text: &str) -> GcodeError {
    let mut diagnostics = Diagnostics::new();
    lex(text, 0, &mut diagnostics).expect_err("should be refused")
}

#[test]
fn an_ordinary_block_lexes_into_words() {
    let (blocks, _) = lex_ok("N10 G1 X10.5 Y-3.25 F500.\n");
    assert_eq!(blocks.len(), 1);
    let words = &blocks[0].words;
    assert_eq!(words.len(), 5);
    assert_eq!(words[0].letter, 'N');
    assert_eq!(words[1].letter, 'G');
    assert_eq!(words[2].letter, 'X');
    assert!((words[2].value - 10.5).abs() < 1e-12);
    assert!((words[3].value + 3.25).abs() < 1e-12);
    assert_eq!(blocks[0].line, 1);
}

#[test]
fn a_word_may_be_separated_from_its_number_by_spaces() {
    // Legal, and it happens in hand-written programs.
    let (blocks, _) = lex_ok("G 1 X 10.\n");
    assert_eq!(blocks[0].words.len(), 2);
    assert!((blocks[0].words[1].value - 10.0).abs() < 1e-12);
}

#[test]
fn code_keys_are_integers_so_g59_point_1_is_never_a_float_comparison() {
    let (blocks, _) = lex_ok("G59.1 G1 G91.1\n");
    let keys: Vec<u32> = blocks[0].words.iter().map(|w| w.code_key()).collect();
    assert_eq!(keys, vec![591, 10, 911]);
}

#[test]
fn comments_are_captured_in_both_forms() {
    let (blocks, diagnostics) = lex_ok("G1 X10. (rough pass) Y2.\nG0 Z5. ; retract\n");
    assert_eq!(blocks[0].comments, vec!["rough pass"]);
    assert_eq!(blocks[0].words.len(), 3, "the comment does not eat Y2.");
    assert_eq!(blocks[1].comments, vec!["retract"]);
    assert!(diagnostics.is_empty());
}

#[test]
fn an_unbalanced_comment_warns_rather_than_aborting() {
    // Illegal, and common enough in the wild that refusing the file would be
    // refusing files that run correctly on the machine.
    let (blocks, diagnostics) = lex_ok("G1 X10. (never closed\nG1 Y20.)\nG1 Z1.\n");
    assert_eq!(diagnostics.count_of("unbalanced-comment"), 1);
    // Everything up to the closing paren is comment, so line 2 has no words.
    assert!(blocks[1].words.is_empty());
    assert_eq!(blocks[2].words.len(), 2, "and lexing recovers afterwards");
}

#[test]
fn a_nested_paren_inside_a_comment_warns() {
    let (_, diagnostics) = lex_ok("G1 X10. (outer (inner) )\n");
    assert_eq!(diagnostics.count_of("nested-comment"), 1);
}

#[test]
fn block_skip_is_recorded_only_at_the_start_of_a_line() {
    let (blocks, _) = lex_ok("/G1 X10.\nG1 Y10.\n");
    assert!(blocks[0].block_skip);
    assert!(!blocks[1].block_skip);
}

#[test]
fn tape_markers_and_blank_lines_produce_empty_blocks() {
    let (blocks, _) = lex_ok("%\nO1000\n\n%\n");
    assert!(blocks[0].is_empty());
    assert_eq!(blocks[1].words.len(), 1, "O1000 is a program number");
    assert!(blocks[2].is_empty());
    assert!(blocks[3].is_empty());
}

#[test]
fn the_decimal_point_is_recorded_because_its_absence_changes_the_meaning() {
    // X10 on a legacy control is 0.010 mm, not 10 mm. The lexer records the
    // fact; the policy decision belongs further up.
    let (blocks, _) = lex_ok("X10 Y10.\n");
    assert!(!blocks[0].words[0].had_decimal);
    assert!(blocks[0].words[1].had_decimal);
}

#[test]
fn siemens_and_heidenhain_are_refused_by_name() {
    for (text, expected) in [
        ("N10 CYCLE81(10,0,2,-15)\n", ForeignDialect::Siemens840d),
        ("R1=45.0\n", ForeignDialect::Siemens840d),
        ("0 BEGIN PGM TEST MM\n", ForeignDialect::HeidenhainKlartext),
        (
            "3 TOOL CALL 5 Z S2000\n",
            ForeignDialect::HeidenhainKlartext,
        ),
    ] {
        match lex_err(text) {
            GcodeError::ForeignLanguage { dialect, .. } => assert_eq!(dialect, expected),
            other => panic!("{text:?}: {other:?}"),
        }
    }
}

#[test]
fn a_foreign_file_is_named_even_when_a_syntax_error_comes_first() {
    // The behaviour that made this worth fixing: a real Siemens program whose
    // first *unparseable* line arrives before its first *recognisable* one.
    //
    // `DEF REAL DEPTH` is the giveaway, and it is on line 5. Line 3 is a
    // Siemens frame assignment that lexes as an `A` word with no number, so
    // interleaved detection reported a syntax error on line 3 and the file was
    // never identified. The visitor sees "this is not the language I read"
    // instead of a complaint about a line they cannot fix.
    let text = "\
;SIEMENS 840D
N10 G17 G54
N20 TRANS X=A
N30 G0 X0 Y0
N40 DEF REAL DEPTH
N50 CYCLE81(10,0,2,-15)
";
    match lex_err(text) {
        GcodeError::ForeignLanguage {
            dialect, evidence, ..
        } => {
            assert_eq!(dialect, ForeignDialect::Siemens840d);
            assert_eq!(evidence, "DEF REAL");
        }
        other => panic!("a Siemens file must be named as one, got {other:?}"),
    }
}

#[test]
fn the_first_marker_in_the_file_wins_rather_than_the_first_that_lexes() {
    // Two markers, and the earlier one is the evidence. A file identified by
    // its last line would report evidence a reader cannot easily find.
    let text = "N10 TOOL CALL 5 Z S2000\nN20 CYCL DEF 200\n";
    match lex_err(text) {
        GcodeError::ForeignLanguage {
            dialect, evidence, ..
        } => {
            assert_eq!(dialect, ForeignDialect::HeidenhainKlartext);
            assert_eq!(evidence, "TOOL CALL");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_bare_r_word_is_still_an_arc_radius_not_a_siemens_parameter() {
    // The `=` is what distinguishes them, and getting this wrong would refuse
    // every arc written in R form.
    let (blocks, _) = lex_ok("G2 X10. Y10. R5.\n");
    assert_eq!(blocks[0].words.len(), 4);
}

#[test]
fn macro_programming_is_refused_and_names_the_construct() {
    for (text, needle) in [
        ("#100 = 5.0\n", "# variable"),
        ("IF [#1 GT 5] GOTO 100\n", "IF"),
        ("WHILE [#2 LT 10] DO 1\n", "WHILE"),
    ] {
        match lex_err(text) {
            GcodeError::MacroProgramming { construct, .. } => {
                assert!(
                    construct.contains(needle),
                    "expected {needle:?} in {construct:?}"
                );
            }
            other => panic!("{text:?}: {other:?}"),
        }
    }
}

#[test]
fn o_words_are_refused_but_uppercase_program_numbers_are_not() {
    match lex_err("o100 sub\n") {
        GcodeError::OWord { word, .. } => assert!(word.starts_with("o100")),
        other => panic!("{other:?}"),
    }
    let (blocks, _) = lex_ok("O100\n");
    assert_eq!(blocks[0].words[0].letter, 'O');
}

#[test]
fn every_error_names_a_line_and_a_column() {
    // A user cannot act on "unsupported construct"; they can act on a line.
    let err = lex_err("G1 X10.\nG1 Y20.\n#500 = 3\n");
    let site = err.site().expect("errors from lexing have a site");
    assert_eq!(site.line, 3);
    assert_eq!(site.column, 1);
    assert!(err.to_string().contains("line 3"));
}

#[test]
fn a_number_that_is_not_a_number_is_refused() {
    for text in ["X.\n", "X+\n", "X\n"] {
        assert!(
            matches!(lex_err(text), GcodeError::NotANumber { .. }),
            "{text:?}"
        );
    }
}

// --- stage two ------------------------------------------------------------

fn block_of(text: &str) -> chipbreaker_gcode::block::Block {
    let (blocks, _) = lex_ok(text);
    assemble(&blocks[0]).expect("should assemble")
}

fn block_err(text: &str) -> GcodeError {
    let (blocks, _) = lex_ok(text);
    assemble(&blocks[0]).expect_err("should be refused")
}

#[test]
fn words_sort_into_slots() {
    let block = block_of("G1 X10. Y-2. Z3. F500. S8000 T4 H4 I1. J2.\n");
    assert_eq!(block.g_codes, vec![10]);
    assert!(block.has_axis_words());
    assert!((block.axes[0].as_ref().expect("X").value - 10.0).abs() < 1e-12);
    assert!((block.ijk[1].as_ref().expect("J").value - 2.0).abs() < 1e-12);
    assert!((block.f.as_ref().expect("F").value - 500.0).abs() < 1e-12);
    assert_eq!(
        block
            .t
            .as_ref()
            .and_then(chipbreaker_gcode::lex::Word::as_u32),
        Some(4)
    );
}

#[test]
fn two_motion_codes_in_one_block_is_an_error_not_last_one_wins() {
    // Which motion a real control performs depends on the control, so there is
    // no safe reading.
    match block_err("G0 G1 X10.\n") {
        GcodeError::ModalGroupConflict { group, codes, .. } => {
            assert_eq!(group, "motion codes");
            assert_eq!(codes, vec!["G0".to_owned(), "G1".to_owned()]);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn conflicts_are_detected_in_every_group_not_only_motion() {
    for text in [
        "G17 G18\n",
        "G90 G91\n",
        "G20 G21\n",
        "G54 G55\n",
        "G98 G99\n",
        "G93 G94\n",
        "G43 G49\n",
        "M3 M5\n",
    ] {
        assert!(
            matches!(block_err(text), GcodeError::ModalGroupConflict { .. }),
            "{text:?} should conflict"
        );
    }
}

#[test]
fn codes_from_different_groups_share_a_block_happily() {
    // The commonest line in any program is several groups at once.
    let block = block_of("G90 G54 G17 G21 G0 X0. Y0.\n");
    assert_eq!(block.g_codes.len(), 5);
    assert_eq!(block.g_in(ModalGroup::Motion), Some(0));
    assert_eq!(block.g_in(ModalGroup::Distance), Some(900));
    assert_eq!(block.g_in(ModalGroup::WorkOffset), Some(540));
}

#[test]
fn g53_and_a_motion_code_coexist_because_g53_is_non_modal() {
    let block = block_of("G53 G0 X0. Y0. Z0.\n");
    assert_eq!(block.g_in(ModalGroup::NonModal), Some(530));
    assert_eq!(block.g_in(ModalGroup::Motion), Some(0));
}

#[test]
fn cutter_compensation_is_refused_the_moment_it_is_armed() {
    // G41 on its own arms the control for every move that follows, so the
    // refusal cannot wait until a move appears.
    for (text, code) in [("G41 D1\n", 41), ("G42 D1\n", 42)] {
        match block_err(text) {
            GcodeError::CutterCompensation { code: found, .. } => assert_eq!(found, code),
            other => panic!("{other:?}"),
        }
    }
    // And the message has to say what to do about it.
    let message = block_err("G41 D1\n").to_string();
    assert!(message.contains("G40"), "{message}");
    assert!(message.contains("tool radius"), "{message}");

    // G40 -- cancelling it -- is perfectly fine.
    assert_eq!(block_of("G40\n").g_in(ModalGroup::CutterComp), Some(400));
}

#[test]
fn an_unimplemented_g_code_is_named_rather_than_ignored() {
    match block_err("G65 P1000\n") {
        GcodeError::UnsupportedCode { code, .. } => assert_eq!(code, "G65"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn g54_point_1_is_refused_rather_than_silently_treated_as_g54() {
    // Fanuc's extended offsets are addressed by a P word and are a different
    // mechanism. A range like `540..=593` would have swallowed this.
    match block_err("G54.1 P3\n") {
        GcodeError::UnsupportedCode { code, .. } => assert_eq!(code, "G54.1"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn an_unknown_m_code_is_kept_rather_than_refused() {
    // Shops use M-codes for machine-specific functions. Refusing a program for
    // using M55 would refuse most real programs.
    let block = block_of("M55\n");
    assert_eq!(block.m_codes, vec![550]);
    assert_eq!(m_group(550), None);
}

#[test]
fn group_membership_matches_the_standard() {
    assert_eq!(g_group(0), Some(ModalGroup::Motion));
    assert_eq!(g_group(810), Some(ModalGroup::Motion), "G81 is a cycle");
    assert_eq!(
        g_group(800),
        Some(ModalGroup::Motion),
        "so is G80 cancelling"
    );
    assert_eq!(g_group(170), Some(ModalGroup::Plane));
    assert_eq!(g_group(901), Some(ModalGroup::ArcDistance), "G90.1");
    assert_eq!(g_group(900), Some(ModalGroup::Distance), "G90");
    assert_eq!(g_group(530), Some(ModalGroup::NonModal), "G53");
    assert_eq!(g_group(650), None, "G65 is macro call, unsupported");
}

#[test]
fn codes_render_the_way_they_were_written() {
    assert_eq!(render_code('G', 0), "G0");
    assert_eq!(render_code('G', 10), "G1");
    assert_eq!(render_code('G', 591), "G59.1");
    assert_eq!(render_code('M', 300), "M30");
}

#[test]
fn a_repeated_word_is_last_one_wins_unlike_a_modal_conflict() {
    // Unambiguous, unlike two motion codes: every control takes the last.
    let block = block_of("G1 X10. X20.\n");
    assert!((block.axes[0].as_ref().expect("X").value - 20.0).abs() < 1e-12);
}

#[test]
fn line_numbers_are_labels_and_carry_no_ordering() {
    let (blocks, _) = lex_ok("N50 G1 X10.\nN10 G1 X20.\nN50 G1 X30.\n");
    let numbers: Vec<f64> = blocks
        .iter()
        .map(|b| b.word('N').expect("N").value)
        .collect();
    assert_eq!(numbers, vec![50.0, 10.0, 50.0]);
    // Out of order and repeated, and none of that means anything: the file
    // order is the order.
    for (i, block) in blocks.iter().enumerate() {
        assert_eq!(block.line, u32::try_from(i + 1).expect("small"));
    }
}
