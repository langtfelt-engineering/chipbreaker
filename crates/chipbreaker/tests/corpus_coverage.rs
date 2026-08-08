// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does the regeneration gate actually cover every generator?
//!
//! # The gap this closes
//!
//! CI regenerates the corpus and fails if anything differs, which is what makes
//! a committed corpus evidence rather than a snapshot somebody once took. That
//! gate is only as good as its list of generators, and the list is hand-written
//! in the workflow file.
//!
//! A generator that exists but is not on the list is invisible to it. That is
//! not hypothetical: a Python generator's output drifted from its source for a
//! week, and the reason the drift survived a local check was that the local
//! check ran the Cargo generators and not the Python one. The workflow did have
//! it, so CI caught it — but nothing guaranteed the workflow would keep having
//! it, and the next generator added is the one that gets forgotten.
//!
//! So this test asserts the invariant directly: **every generator in the tree is
//! named in the regeneration step.** Adding a generator without wiring it up
//! fails here, at the moment it is added, rather than the first time its output
//! goes stale.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("the workspace root")
}

fn workflow() -> String {
    std::fs::read_to_string(root().join(".github/workflows/ci.yml")).expect("the CI workflow")
}

/// Every corpus generator in the tree, by the name CI would invoke it under.
fn generators() -> BTreeSet<String> {
    let mut out = BTreeSet::new();

    let examples = root().join("crates/chipbreaker-core/examples");
    for e in std::fs::read_dir(&examples).expect("examples directory") {
        let name = e.expect("a directory entry").file_name();
        let name = name.to_str().expect("utf-8");
        if let Some(stem) = name.strip_suffix(".rs")
            && stem.starts_with("generate_")
        {
            out.insert(stem.to_owned());
        }
    }

    let scripts = root().join("scripts");
    for e in std::fs::read_dir(&scripts).expect("scripts directory") {
        let name = e.expect("a directory entry").file_name();
        let name = name.to_str().expect("utf-8").to_owned();
        // A corpus generator by name. Benchmarks and helpers in the same
        // directory are not corpus generators and are not expected here.
        if name.ends_with(".py") && (name.contains("corpus") || name.contains("expectations")) {
            out.insert(name);
        }
    }

    out
}

#[test]
fn every_generator_is_named_in_the_regeneration_gate() {
    let ci = workflow();
    let gens = generators();
    assert!(
        gens.len() >= 8,
        "found only {} generators, so this test is not looking where it thinks",
        gens.len()
    );

    let missing: Vec<&String> = gens.iter().filter(|g| !ci.contains(g.as_str())).collect();
    assert!(
        missing.is_empty(),
        "these generators exist but the corpus regeneration job never runs them, so \
         their committed output can drift without CI noticing: {missing:?}"
    );
    eprintln!("{} generators, all wired into the gate", gens.len());
}

#[test]
fn the_coverage_check_would_notice_an_unwired_generator() {
    // The mutation check. `contains` on a whole workflow file passes easily --
    // any substring anywhere counts -- so this proves the assertion can fail:
    // a plausibly-named generator that does not exist must not be found.
    let ci = workflow();
    for invented in [
        "generate_collision_corpus_that_does_not_exist",
        "fixture_corpus_nonexistent.py",
    ] {
        assert!(
            !ci.contains(invented),
            "the workflow mentions {invented}, which was chosen because it does not \
             exist; the test above proves nothing if arbitrary names match"
        );
    }
}

#[test]
fn every_committed_corpus_expectation_has_a_generator_or_is_declared_hand_written() {
    // The other direction. A corpus file with no generator cannot drift *from* a
    // generator, so the regeneration gate is silent about it -- and that silence
    // is how a stale reference lived in the mesh corpus for a week.
    //
    // Hand-writing one is sometimes right: the mesh corpus states, correctly,
    // that generating expectations from validator output would assert only
    // self-consistency. What is not right is a file being hand-written by
    // accident. So each one must either be generated or say in its own text
    // that it is hand-written, and be validated against the real artifact
    // elsewhere.
    let corpus = root().join("tests/corpus");
    let ci = workflow();
    let mut checked = 0usize;
    for dir in std::fs::read_dir(&corpus).expect("the corpus directory") {
        let dir = dir.expect("a directory entry").path();
        if !dir.is_dir() {
            continue;
        }
        for f in std::fs::read_dir(&dir).expect("a corpus subdirectory") {
            let path = f.expect("a directory entry").path();
            if path.extension().and_then(|e| e.to_str()) != Some("json") {
                continue;
            }
            let rel = path
                .strip_prefix(root())
                .expect("under the root")
                .to_str()
                .expect("utf-8")
                .replace('\\', "/");
            let generated = ci.contains(&rel);
            let text = std::fs::read_to_string(&path).expect("readable");
            let declared = text.contains("HAND-WRITTEN");
            assert!(
                generated || declared,
                "{rel} is neither regenerated by CI nor marked HAND-WRITTEN, so nothing \
                 checks that it still describes reality"
            );
            checked += 1;
        }
    }
    assert!(checked >= 8, "only {checked} corpus files examined");
    eprintln!("{checked} corpus files, each generated or declared hand-written");
}
