// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Golden-file checks: the committed digests that pin what this unit computes.
//!
//! These are the tripwires. Every other test in the repository asserts a
//! *property* — that a predicate agrees with exact arithmetic, that a set
//! identity holds. A golden hash asserts something narrower and, for a
//! determinism guarantee, more useful: that the exact bits have not moved.
//!
//! A golden mismatch means one of two things, and the commit message has to say
//! which:
//!
//! 1. A deliberate change to what we compute. Accept the new hash and explain
//!    the change.
//! 2. A determinism bug. Do not accept it.
//!
//! To accept:
//!
//! ```sh
//! CHIPBREAKER_ACCEPT_GOLDEN=1 cargo test -p chipbreaker-core --test golden_hashes
//! ```
//!
//! `selftest-results` is deliberately the same digest the CLI prints and the CI
//! parity job compares against the `wasmtime` run. One number, pinned in one
//! place, checked three ways.

use chipbreaker_core::golden::{CanonicalHash, GoldenStore, Hashable, check_golden};
use chipbreaker_core::math::{Aabb3, Mat3, Mat4, Ray, Vec2, Vec3};
use chipbreaker_core::predicates::ADAPTIVE;
use chipbreaker_core::predicates::corpus::{PredicateKind, degenerate_corpus};
use chipbreaker_core::selftest;
use chipbreaker_core::spans::{Span, Spans};

/// Asserts a digest against its committed golden file.
fn assert_golden(name: &str, hash: &CanonicalHash) {
    if let Err(e) = check_golden(name, &hash.finish()) {
        panic!("{e}");
    }
}

#[test]
fn selftest_results_digest_is_unchanged() {
    // The headline number: what `chipbreaker selftest` prints, what CI compares
    // between native and WASM, and what every later unit will extend.
    let report = selftest::run();
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
    assert_golden("selftest-results", &h);
}

#[test]
fn each_selftest_suite_digest_is_unchanged() {
    // Pinned individually as well as in aggregate, so a mismatch names the suite
    // that moved instead of just saying "something changed".
    for suite in selftest::run().suites {
        let mut h = CanonicalHash::new();
        suite.hash_canonical(&mut h);
        assert_golden(
            &format!("selftest-suite-{}", suite.name.replace('.', "-")),
            &h,
        );
    }
}

#[test]
fn predicate_corpus_digest_is_unchanged() {
    // Covers the corpus inputs, the stored expectations, and the answers the
    // adaptive predicates actually give. Editing a coordinate, an expectation or
    // the predicate backend all move this hash.
    let cases = degenerate_corpus();
    let mut h = CanonicalHash::new();
    h.begin("predicate-corpus");
    h.usize(cases.len());
    for kind in PredicateKind::ALL {
        h.begin(kind.name());
        let range = kind.coord_range();
        // The published exact range is part of the contract, so a change to it
        // must show up here too.
        h.f64(range.min).f64(range.max).u64(u64::from(range.degree));
        for case in cases.iter().filter(|c| c.kind == kind) {
            case.hash_canonical(&mut h);
            case.evaluate(&ADAPTIVE).hash_canonical(&mut h);
        }
        h.end();
    }
    h.end();
    assert_golden("predicate-corpus", &h);
}

#[test]
fn span_algebra_digest_is_unchanged() {
    // A fixed worked example rather than random data: readable, and a mismatch
    // points at a specific operation.
    let a = Spans::from_unsorted(vec![
        Span::new(0.0, 10.0),
        Span::new(20.0, 25.0),
        Span::new(30.0, 31.5),
    ]);
    let b = Spans::from_unsorted(vec![
        Span::new(5.0, 22.0),
        Span::new(24.0, 24.5),
        Span::new(40.0, 50.0),
    ]);
    let bounds = Span::new(-5.0, 60.0);

    let mut h = CanonicalHash::new();
    h.begin("span-algebra");
    for (label, set) in [
        ("a", a.clone()),
        ("b", b.clone()),
        ("union", a.union(&b)),
        ("intersect", a.intersect(&b)),
        ("a-minus-b", a.subtract(&b)),
        ("b-minus-a", b.subtract(&a)),
        ("complement-a", a.complement_within(bounds)),
        ("clipped", a.clipped_to(Span::new(3.0, 30.5))),
    ] {
        h.begin(label);
        set.hash_canonical(&mut h);
        // The measure is hashed too: it is the value U12's removed-material
        // totals are built on, and a reassociated sum would change its last bit
        // without changing the span boundaries at all.
        h.f64(set.measure());
        h.usize(set.len());
        h.end();
    }
    h.end();
    assert_golden("span-algebra", &h);
}

#[test]
fn math_types_digest_is_unchanged() {
    // Pins the canonical encoding of every math type and the results of the
    // operations most likely to drift: inversion, transforms, normalization.
    let m = Mat4::from_mat3_translation(
        Mat3::from_rows_array([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]]),
        Vec3::new(10.0, -20.0, 30.5),
    );
    let p = Vec3::new(0.1, -2.5, 7.0 / 3.0);
    let b = Aabb3::from_points(&[p, Vec3::ONE, Vec3::new(-4.0, 0.25, 12.0)]);

    let mut h = CanonicalHash::new();
    h.begin("math-types");
    h.add(&Vec2::new(0.1, -0.2));
    h.add(&p);
    h.add(&m);
    h.add(&m.upper_left3());
    h.add(&b);
    h.add(&Ray::new(p, Vec3::Z));
    h.f64(m.determinant());
    h.f64(b.surface_area())
        .f64(b.volume())
        .add(&b.center())
        .add(&b.extent());
    h.usize(b.longest_axis().index());
    h.f64(p.length())
        .f64(p.length_squared())
        .f64(p.dot(Vec3::ONE));
    h.add(&p.cross(Vec3::ONE));
    match p.normalize() {
        Some(n) => {
            h.bool(true).add(&n);
        }
        None => {
            h.bool(false);
        }
    }
    match m.inverse() {
        Some(inv) => {
            h.bool(true).add(&inv).add(&inv.transform_point(p));
        }
        None => {
            h.bool(false);
        }
    }
    h.add(&m.transform_point(p)).add(&m.transform_direction(p));
    h.end();
    assert_golden("math-types", &h);
}

#[test]
fn a_deliberately_wrong_digest_is_rejected() {
    // Proves the harness can fail. A golden check that always passes is worse
    // than no golden check, because it looks like coverage.
    //
    // This uses its own store in a scratch directory rather than
    // `check_golden`, for two reasons: under CHIPBREAKER_ACCEPT_GOLDEN=1 the
    // ambient store would *write* the wrong digest instead of rejecting it, and
    // since cargo runs tests in parallel it would race the real checks and
    // corrupt a committed golden file. Which is exactly what it did the first
    // time this test was written.
    let dir = std::env::temp_dir().join(format!(
        "chipbreaker-golden-negative-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);

    let right = Vec3::ONE.canonical_digest();
    let wrong = Vec3::new(1.0, 2.0, 3.0).canonical_digest();
    assert_ne!(right, wrong);

    GoldenStore::new(&dir, true)
        .check("negative-control", &right)
        .expect("accept mode writes the file");

    let comparing = GoldenStore::new(&dir, false);
    comparing
        .check("negative-control", &right)
        .expect("the matching digest still compares equal");

    let err = comparing
        .check("negative-control", &wrong)
        .expect_err("a golden check must reject a digest that does not match");
    let rendered = err.to_string();
    assert!(
        rendered.contains("negative-control"),
        "the error must name the test"
    );
    assert!(rendered.contains(&right.to_hex()), "and the expected hash");
    assert!(rendered.contains(&wrong.to_hex()), "and the actual hash");
    assert!(
        rendered.contains("CHIPBREAKER_ACCEPT_GOLDEN"),
        "and how to accept an intended change"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn every_golden_file_is_lf_terminated_hex() {
    // A CRLF here would make the comparison platform-dependent, which is the
    // reason `*.hash` is marked binary in .gitattributes. Belt and braces.
    let dir = chipbreaker_core::golden::golden_dir();
    let entries = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("golden directory {} is unreadable: {e}", dir.display()));
    let mut seen = 0usize;
    for entry in entries {
        let path = entry.expect("readable entry").path();
        if path.extension().is_none_or(|e| e != "hash") {
            continue;
        }
        let bytes = std::fs::read(&path).expect("readable golden file");
        assert_eq!(
            bytes.len(),
            65,
            "{} is not 64 hex digits plus LF",
            path.display()
        );
        assert_eq!(bytes[64], b'\n', "{} does not end with LF", path.display());
        assert!(
            !bytes.contains(&b'\r'),
            "{} contains CR; the checkout has mangled it",
            path.display()
        );
        assert!(
            bytes[..64]
                .iter()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()),
            "{} is not lower-case hex",
            path.display()
        );
        seen += 1;
    }
    // Under CHIPBREAKER_ACCEPT_GOLDEN the other tests in this binary are writing
    // these files concurrently, so the directory listing is a moving target and
    // a count assertion is a race. The per-file format checks above are still
    // meaningful; only the total is not.
    assert!(
        seen >= 9 || chipbreaker_core::golden::accepting(),
        "expected at least nine golden files, found {seen}"
    );
}
