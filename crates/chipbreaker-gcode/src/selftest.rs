// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The parser's contribution to the cross-platform parity guarantee.
//!
//! # Why the parser needs to be in the hash at all
//!
//! It looks like text handling, and text handling is the same everywhere. But
//! the resolver reaches `atan2`, `sin_cos` and the root solver on every arc, and
//! it does floating-point arithmetic on every coordinate in the pipeline. A
//! one-ULP disagreement between targets moves a segment endpoint, and a moved
//! endpoint is material removed somewhere else.
//!
//! This was learned the expensive way: for a long stretch every green `wasm
//! parity` job was covering the older suites only, because the new code was not
//! in the suite. The hash said nothing about the thing being built.
//!
//! # What is hashed
//!
//! The **resolved toolpaths themselves**, segment by segment, rather than a
//! count or a total. A count survives a coordinate moving; the segments do not.

use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::selftest::{Failure, SuiteResult};

use crate::resolve::{ParseOptions, parse};

/// Programs exercising every path through the parser that touches arithmetic.
///
/// Written inline rather than read from the corpus directory: the self-test runs
/// under `wasmtime` with no filesystem, and a suite that cannot run on the
/// target it exists to check would be worse than none.
const PROGRAMS: &[(&str, &str)] = &[
    (
        "linear",
        "G21 G90 G17 G54\nG0 X0. Y0. Z10.\nG1 Z-1. F250.\nG1 X20.5 Y-3.25\nG0 Z10.\nM30\n",
    ),
    (
        "arc-ijk",
        "G21 G90 G17\nG0 X10. Y0.\nG1 Z-1. F250.\nG3 X0. Y10. I-10. J0.\nG2 X10. Y0. I0. J-10.\nM30\n",
    ),
    (
        "arc-r-both-signs",
        "G21 G90 G17\nG0 X10. Y0.\nG1 Z-1. F250.\nG3 X0. Y10. R10.\nG3 X10. Y0. R-10.\nM30\n",
    ),
    (
        "arc-planes",
        "G21 G90\nG18 G0 X0. Z10.\nG1 F250.\nG3 X10. Z0. I0. K-10.\n\
         G19 G0 Y10. Z0.\nG3 Y0. Z10. J-10. K0.\nG17\nM30\n",
    ),
    (
        "helix",
        "G21 G90 G17\nG0 X10. Y0. Z0.\nG1 F250.\nG3 X10. Y0. Z-5. I-10. J0.\nM30\n",
    ),
    (
        "cycles",
        "G21 G90 G17\nG0 X0. Y0. Z10.\nF250.\n\
         G98 G81 X20. Y30. Z-5. R2.\nX40.\n\
         G99 G83 X60. Y30. Z-7.5 R2. Q2.5\n\
         G80\nM30\n",
    ),
    (
        "offsets",
        "G21 G90\nG10 L2 P1 X-250.5 Y-100.25 Z-2.0481555856608242\n\
         G54 G0 X0. Y0. Z5.\nG1 Z-1. F250.\nG53 G0 X-10. Y-10.\n\
         G92 X0.\nG0 X5.\nG92.1\nM30\n",
    ),
    (
        "inches-and-inverse-time",
        "G20 G90 G0 X1. Y1.\nG93 G1 X2. F4.\nG94 G21 G1 X30. F500.\nM30\n",
    ),
    (
        "subprogram",
        "G21 G90 G0 X0. Y0. Z10.\nF250.\nM98 P100 L2\nM30\n\
         O100\nG91 G1 X10. Y5.\nG90\nM99\n",
    ),
];

/// How many programs the suite parses.
pub const PROGRAM_COUNT: usize = PROGRAMS.len();

/// The suites this crate contributes to [`chipbreaker_core::selftest::run_with`].
#[must_use]
pub fn suites() -> Vec<SuiteResult> {
    vec![parser_suite()]
}

fn parser_suite() -> SuiteResult {
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("gcode");
    let mut cases = 0usize;

    for (name, text) in PROGRAMS {
        h.str(name);
        match parse(text, name, &ParseOptions::default(), None) {
            Ok((path, diagnostics, stats)) => {
                // The segments themselves, not a summary of them.
                h.add(&path);
                h.usize(diagnostics.len());
                h.u64(u64::from(stats.segments));
                cases += 1 + path.segments.len();

                // Hashing catches a *change*; a defect present from the first
                // run would hash consistently and still be wrong. So the
                // invariants are asserted as well.
                for pair in path.segments.windows(2) {
                    if pair[0].end != pair[1].start {
                        failures.push(Failure {
                            case: format!("{name}: contiguity"),
                            detail: format!(
                                "{:?} does not join {:?}",
                                pair[0].end.to_array(),
                                pair[1].start.to_array()
                            ),
                        });
                    }
                }
                for segment in &path.segments {
                    if !segment.start.is_finite() || !segment.end.is_finite() {
                        failures.push(Failure {
                            case: format!("{name}: finite"),
                            detail: "a non-finite coordinate reached the IR".to_owned(),
                        });
                    }
                }
            }
            Err(error) => {
                failures.push(Failure {
                    case: (*name).to_owned(),
                    detail: error.to_string(),
                });
            }
        }
    }

    // The two arc forms describe one arc. Their hashes differ legitimately, so
    // what is checked is the geometry, to the measured 32 ULP bound.
    if let (Ok((ijk, _, _)), Ok((radius, _, _))) = (
        parse(PROGRAMS[1].1, "ijk", &ParseOptions::default(), None),
        parse(PROGRAMS[2].1, "r", &ParseOptions::default(), None),
    ) {
        let centres = |p: &chipbreaker_core::toolpath::Toolpath| {
            p.segments
                .iter()
                .filter_map(|s| s.arc.map(|a| a.center))
                .collect::<Vec<_>>()
        };
        let bound = 32.0 * 10.0 * f64::EPSILON;
        for (a, b) in centres(&ijk).iter().zip(centres(&radius)) {
            cases += 1;
            for (u, v) in a.to_array().iter().zip(b.to_array()) {
                if (u - v).abs() > bound {
                    failures.push(Failure {
                        case: "arc forms agree".to_owned(),
                        detail: format!("{:?} against {:?}", a.to_array(), b.to_array()),
                    });
                }
            }
        }
    }
    h.end();

    SuiteResult {
        name: "gcode",
        description: "NC programs resolved to toolpath IR, hashed segment by segment",
        cases,
        failures,
        digest: h.finish(),
    }
}
