// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The combined self-test digest: the one the CLI prints and CI compares.
//!
//! It lives here rather than in the core because it covers the core's suites
//! *and* the parser's, and the core cannot see a crate that depends on it.
//! `selftest-core-results` pins the core's own subset separately, so a change
//! can be attributed to one side or the other.

use chipbreaker_core::golden::{CanonicalHash, Hashable, check_golden};

#[test]
fn the_combined_selftest_digest_is_unchanged() {
    let report = chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites());
    assert!(
        report.passed(),
        "refusing to pin the digest of a failing self-test: {:?}",
        report
            .suites
            .iter()
            .filter(|s| !s.passed())
            .map(|s| s.name)
            .collect::<Vec<_>>()
    );
    let mut h = CanonicalHash::new();
    report.hash_canonical(&mut h);
    if let Err(e) = check_golden("selftest-results", &h.finish()) {
        panic!("{e}");
    }
}

#[test]
fn each_parser_suite_digest_is_unchanged() {
    for suite in chipbreaker_gcode::selftest::suites() {
        let mut h = CanonicalHash::new();
        suite.hash_canonical(&mut h);
        if let Err(e) = check_golden(&format!("selftest-suite-{}", suite.name), &h.finish()) {
            panic!("{e}");
        }
    }
}
