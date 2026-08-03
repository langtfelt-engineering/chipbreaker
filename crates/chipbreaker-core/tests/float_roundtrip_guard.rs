// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! `serde_json` must parse floats bit-exactly, and this test fails loudly if it
//! stops doing so.
//!
//! # Why this exists separately from the tool library's own tests
//!
//! Unit 3 found that `serde_json`'s default float parser is not correctly
//! rounded: it reads `2.0481555856608242` as `2.048155585660824`, one ULP low,
//! where Rust's own `str::parse` gets it right. The workspace enables the
//! `float_roundtrip` feature to fix it.
//!
//! That fix is a single line in the workspace `Cargo.toml`, which makes it a
//! single line somebody can remove — while tidying features, or while adding a
//! crate that declares `serde_json` directly instead of inheriting the workspace
//! entry. The tool library's tests would catch it, but only because a tapered
//! mill happens to be in their fixture. This test depends on nothing but
//! `serde_json` itself, so it keeps working however the rest of the engine is
//! rearranged.
//!
//! A one-ULP loss on read means a saved simulation describes a different solid
//! from the one that was written. That is the guarantee this project exists to
//! make, so it gets a test that names it.

/// Values whose shortest round-trip form needs all seventeen significant digits.
///
/// Round numbers survive any parser ever written, which is precisely why the
/// original defect went unnoticed. Each of these is a real quantity from the
/// engine rather than a contrived bit pattern.
const SEVENTEEN_DIGIT_VALUES: &[(&str, f64)] = &[
    // Top radius of a 3 degree tapered mill: 1 + 20 tan(3 deg).
    ("tapered mill top radius", 2.048_155_585_660_824_2),
    // Height of a 118 degree drill point on a 6 mm drill: 3 / tan(59 deg).
    ("118 degree drill point", 1.802_581_857_082_680_8),
    // Widest point of a 12 mm barrel on a 200 mm arc: sqrt(200^2 - 194^2).
    ("barrel widest point", 48.620_983_124_572_874),
    // Scallop left by a 6 mm ball nose at 0.7 mm stepover.
    ("ball nose scallop", 0.020_486_616_912_083_644),
    // A negative one, since sign handling is a separate code path.
    ("negative", -2.048_155_585_660_824_2),
];

#[test]
fn serde_json_parses_floats_bit_exactly() {
    for (name, value) in SEVENTEEN_DIGIT_VALUES {
        // Guard the guard: if a value stops needing seventeen digits, this test
        // silently becomes a test of nothing.
        let text = format!("{value:?}");
        let digits = text
            .trim_start_matches('-')
            .chars()
            .filter(char::is_ascii_digit)
            .count();
        assert!(
            digits >= 17,
            "{name}: {text} carries only {digits} significant digits, so it \
             cannot detect a one-ULP parsing loss; choose another value"
        );

        let parsed: f64 = serde_json::from_str(&text).expect("valid JSON number");
        assert_eq!(
            parsed.to_bits(),
            value.to_bits(),
            "{name}: serde_json read {text} as {parsed:?}, a difference of {} ULP. \
             The workspace `serde_json` entry has lost its `float_roundtrip` \
             feature, or a crate declares `serde_json` directly without it.",
            parsed.to_bits().abs_diff(value.to_bits())
        );
    }
}

#[test]
fn serde_json_agrees_with_rusts_own_parser() {
    // A second formulation of the same requirement, in terms of the thing that
    // is unambiguously correct: `str::parse` is correctly rounded.
    for (name, value) in SEVENTEEN_DIGIT_VALUES {
        let text = format!("{value:?}");
        let by_std: f64 = text.parse().expect("a number");
        let by_serde: f64 = serde_json::from_str(&text).expect("valid JSON number");
        assert_eq!(
            by_serde.to_bits(),
            by_std.to_bits(),
            "{name}: serde_json and str::parse disagree about {text}"
        );
    }
}

#[test]
fn a_value_survives_a_json_document_not_merely_a_bare_number() {
    // The bare-number path and the document path are different code in
    // serde_json, and the engine uses the latter.
    let value = 2.048_155_585_660_824_2_f64;
    let document = format!(r#"{{"schema":"probe","point":[{value:?}, 0.0]}}"#);
    let parsed: serde_json::Value = serde_json::from_str(&document).expect("valid");
    let back = parsed["point"][0].as_f64().expect("a number");
    assert_eq!(
        back.to_bits(),
        value.to_bits(),
        "a float nested in a document must survive as exactly as a bare one"
    );
}
