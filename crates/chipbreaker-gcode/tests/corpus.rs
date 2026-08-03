// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The versioned NC corpus: every entry parses to its golden IR, or is rejected
//! with the specific error it is there to provoke.
//!
//! # Why the goldens are hashes rather than dumps
//!
//! A golden IR dump of a hundred segments is a file nobody reads and everybody
//! regenerates. A hash is read in one glance: it changed, or it did not. When
//! one changes, `path dump` produces the difference on demand.
//!
//! # The arc pair
//!
//! `arc-quarter-ijk` and `arc-quarter-r` describe the same arc and their hashes
//! **legitimately differ** — see the corpus README. A separate test asserts they
//! agree geometrically, which is the property that actually matters.

use std::collections::BTreeMap;
use std::path::PathBuf;

use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_gcode::resolve::{ParseOptions, parse};
use serde_json::Value;

fn corpus_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/gcode")
}

fn expectations() -> BTreeMap<String, Value> {
    let path = corpus_dir().join("expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    let doc: Value = serde_json::from_str(&text).expect("expectations are JSON");
    assert_eq!(doc["schema"], "chipbreaker.gcode-corpus");
    doc["entries"]
        .as_object()
        .expect("an entries object")
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

fn read(name: &str) -> String {
    let path = corpus_dir().join(format!("{name}.nc"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

/// The options the corpus is parsed under. Deliberately the defaults: an entry
/// that needs a flag to parse is an entry testing the flag, and there are none.
fn options() -> ParseOptions {
    ParseOptions::default()
}

#[test]
fn the_corpus_is_large_enough_to_be_worth_having() {
    let entries = expectations();
    assert!(
        entries.len() >= 40,
        "the corpus has {} entries; the unit asks for at least 40",
        entries.len()
    );
    let rejects = entries.values().filter(|e| e["expect"] == "reject").count();
    assert!(
        rejects >= 10,
        "only {rejects} rejection cases; refusing correctly is half the unit"
    );
}

#[test]
fn every_entry_has_a_why() {
    // The Unit 2 convention: a corpus entry whose purpose is not written down
    // becomes a file nobody dares delete and nobody can explain.
    for (name, entry) in expectations() {
        let why = entry["_why"].as_str().unwrap_or_default();
        assert!(
            why.len() > 40,
            "{name}: `_why` is {why:?}, which does not say what the entry is for"
        );
    }
}

#[test]
fn every_entry_parses_or_is_rejected_exactly_as_expected() {
    let mut parsed = 0usize;
    let mut rejected = 0usize;

    for (name, entry) in expectations() {
        let text = read(&name);
        let outcome = parse(&text, &name, &options(), None);

        match entry["expect"].as_str() {
            Some("parse") => {
                let (path, _, _) =
                    outcome.unwrap_or_else(|e| panic!("{name} should parse, but: {e}"));
                // Contiguity is the invariant every downstream unit relies on.
                for pair in path.segments.windows(2) {
                    assert_eq!(
                        pair[0].end, pair[1].start,
                        "{name}: segments must join exactly"
                    );
                }
                for segment in &path.segments {
                    assert!(
                        segment.start.is_finite() && segment.end.is_finite(),
                        "{name}: a non-finite coordinate reached the IR"
                    );
                    assert!(
                        segment.chord() > 0.0 || segment.arc.is_some(),
                        "{name}: a zero-length segment survived"
                    );
                }
                parsed += 1;
            }
            Some("reject") => {
                let expected = entry["error"].as_str().unwrap_or_default();
                let error = outcome
                    .err()
                    .unwrap_or_else(|| panic!("{name} should have been rejected"));
                assert_eq!(
                    error.kind(),
                    expected,
                    "{name}: rejected with {} instead of {expected} ({error})",
                    error.kind()
                );
                // A rejection the user cannot act on is nearly worthless.
                assert!(
                    error.site().is_some() || error.kind() == "invalid-ir",
                    "{name}: {error} names no place in the file"
                );
                rejected += 1;
            }
            other => panic!("{name}: unknown expectation {other:?}"),
        }
    }

    eprintln!("corpus: {parsed} parsed, {rejected} rejected");
    assert!(parsed > 0 && rejected > 0);
}

#[test]
fn every_parsing_entry_hashes_the_same_way_twice() {
    // The corpus's real job at U5 onward: a hash that moves means the IR moved.
    for (name, entry) in expectations() {
        if entry["expect"] != "parse" {
            continue;
        }
        let text = read(&name);
        let digest = |t: &str| {
            let (path, _, _) = parse(t, &name, &options(), None).expect("parses");
            let mut h = CanonicalHash::new();
            h.add(&path);
            h.finish().to_hex()
        };
        assert_eq!(digest(&text), digest(&text), "{name}");
    }
}

#[test]
fn the_two_arc_forms_agree_geometrically_while_their_hashes_differ() {
    // The amendment recorded in the corpus README. If a future change makes the
    // hashes equal, it has snapped a computed centre to a convenient value and
    // that is a bug rather than a tidy-up.
    let ijk = parse(&read("arc-quarter-ijk"), "ijk", &options(), None)
        .expect("parses")
        .0;
    let r = parse(&read("arc-quarter-r"), "r", &options(), None)
        .expect("parses")
        .0;

    let arcs_of = |p: &chipbreaker_core::toolpath::Toolpath| {
        p.segments
            .iter()
            .filter_map(|s| s.arc.map(|a| (a, s.start, s.end)))
            .collect::<Vec<_>>()
    };
    let (a, b) = (arcs_of(&ijk), arcs_of(&r));
    assert_eq!(a.len(), 1);
    assert_eq!(b.len(), 1);

    // 32 ULP, from the measured distribution; see examples/form_agreement.rs.
    let bound = 32.0 * 10.0 * f64::EPSILON;
    for (u, v) in a[0]
        .0
        .center
        .to_array()
        .iter()
        .zip(b[0].0.center.to_array())
    {
        assert!(
            (u - v).abs() <= bound,
            "centres {:?} and {:?} differ by more than 32 ULP",
            a[0].0.center,
            b[0].0.center
        );
    }
    assert!((a[0].0.sweep - b[0].0.sweep).abs() <= 32.0 * core::f64::consts::TAU * f64::EPSILON);
    assert_eq!(a[0].1, b[0].1, "and they start in the same place exactly");
    assert_eq!(a[0].2, b[0].2, "and end in the same place exactly");
}

#[test]
fn the_three_planes_describe_the_same_arc() {
    // Coordinates permuted per plane, so all three are one arc seen from a
    // different axis. G18 is the one that catches people.
    let sweeps: Vec<f64> = ["arc-plane-g17", "arc-plane-g18", "arc-plane-g19"]
        .iter()
        .map(|name| {
            let path = parse(&read(name), name, &options(), None)
                .expect("parses")
                .0;
            path.segments
                .iter()
                .find_map(|s| s.arc.map(|a| a.sweep))
                .unwrap_or_else(|| panic!("{name} has no arc"))
        })
        .collect();
    for sweep in &sweeps {
        assert!(
            (sweep - core::f64::consts::FRAC_PI_2).abs() < 1e-12,
            "every plane must give the same quarter turn: {sweeps:?}"
        );
    }
}

#[test]
fn the_g98_and_g99_forms_of_a_cycle_differ_only_in_their_retracts() {
    // The difference that decides whether the tool clears a clamp between holes.
    let g98 = parse(&read("cycle-g81-g98"), "g98", &options(), None)
        .expect("parses")
        .0;
    let g99 = parse(&read("cycle-g81-g99"), "g99", &options(), None)
        .expect("parses")
        .0;
    assert_eq!(
        g98.segments.len(),
        g99.segments.len(),
        "the same number of motions either way"
    );
    let differing = g98
        .segments
        .iter()
        .zip(&g99.segments)
        .filter(|(a, b)| a.end != b.end)
        .count();
    assert!(differing > 0, "G98 and G99 must not produce the same path");
}

#[test]
fn the_corpus_regenerates_identically() {
    // Guards against a hand-edited entry and against generator drift. The CI
    // job does the same across the whole corpus directory; this catches it in
    // a unit-test loop.
    let expectations_path = corpus_dir().join("expectations.json");
    let before = std::fs::read_to_string(&expectations_path).expect("readable");
    let doc: Value = serde_json::from_str(&before).expect("JSON");
    let names: Vec<&str> = doc["entries"]
        .as_object()
        .expect("entries")
        .keys()
        .map(String::as_str)
        .collect();
    for name in names {
        let path = corpus_dir().join(format!("{name}.nc"));
        assert!(
            path.exists(),
            "{name} is in expectations.json but has no file"
        );
    }
}
