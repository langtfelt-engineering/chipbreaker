// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does every subsystem that computes something appear in the self-test?
//!
//! # Why this is an invariant and not a habit
//!
//! A verification report carries `engine_selftest` to identify the *behaviour*
//! of the build that produced it, and promises that the same manifest digest
//! implies the same findings. That promise is only as wide as the suites behind
//! the digest.
//!
//! Collision detection shipped without a suite. Two builds that disagreed about
//! collisions therefore carried the same `engine_selftest`, and a diff of two
//! such reports showed the collisions changing under an **identical** manifest —
//! which is precisely the thing a manifest exists to make impossible. It was a
//! hole in the assurance claim rather than in a feature, and it was found by
//! running the diff rather than by anybody reasoning about it.
//!
//! So the rule is mechanical now: a module that computes gets a suite, or it
//! gets an exemption with a written reason. A habit would have caught this one
//! only if somebody remembered.
//!
//! # What the exemption list is for
//!
//! Not every module has behaviour a digest could pin. A file of plain data
//! types, a module that only re-exports, a corpus definition consumed by tests —
//! hashing those would add cases without adding coverage. Each is listed with a
//! reason, and the reason is the point: an exemption nobody had to justify is a
//! hole with a comment next to it.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("the workspace root")
}

/// Modules with no behaviour a self-test digest could meaningfully pin.
///
/// Each entry carries why. Anything that computes a number, a geometry or a
/// decision does **not** belong here, however small.
const EXEMPT: &[(&str, &str)] = &[
    (
        "lib",
        "the crate root: re-exports and lints, no behaviour of its own",
    ),
    (
        "eps",
        "named tolerance constants; the values are pinned where they are used",
    ),
    (
        "budget",
        "a memory estimate for the caller's benefit, not part of any result",
    ),
    (
        "crash",
        "a corpus definition consumed by tests; its own tests pin it",
    ),
    (
        "defect",
        "a corpus definition consumed by tests; its own tests pin it",
    ),
    (
        "golden",
        "the hashing machinery itself, covered by hash.selfcheck",
    ),
    (
        "selftest",
        "the harness that produces the digest; it cannot pin itself",
    ),
    (
        "findings",
        "the module root: re-exports over submodules that are covered",
    ),
];

/// Which self-test suite covers a module, where the names differ.
///
/// A module and its suite usually share a name. These do not, and mapping them
/// explicitly is better than loosening the match until everything passes.
const COVERED_BY: &[(&str, &str)] = &[
    ("math", "math.kernels"),
    ("spans", "spans.algebra"),
    ("predicates", "predicates.corpus"),
    ("dexel", "dexel.field"),
    ("toolpath", "gcode"),
    ("transcendental", "transcendental"),
    ("roots", "roots"),
    ("mesh", "mesh"),
    ("tool", "tool"),
    ("sweep", "sweep"),
    ("contour", "contour"),
    ("deviation", "deviation"),
    ("findings::collide", "collision"),
    ("findings::detect", "collision"),
    ("findings::cluster", "deviation"),
    ("findings::attribute", "deviation"),
    ("findings::report", "deviation"),
    ("findings::diff", "deviation"),
    ("findings::verdict", "collision"),
];

/// Every top-level module of `chipbreaker-core`, by file or directory name.
fn core_modules() -> BTreeSet<String> {
    let src = root().join("crates/chipbreaker-core/src");
    let mut out = BTreeSet::new();
    for e in std::fs::read_dir(&src).expect("the core source directory") {
        let p = e.expect("a directory entry").path();
        let name = if p.is_dir() {
            p.file_name().and_then(|s| s.to_str()).map(str::to_owned)
        } else if p.extension().and_then(|s| s.to_str()) == Some("rs") {
            p.file_stem().and_then(|s| s.to_str()).map(str::to_owned)
        } else {
            None
        };
        if let Some(n) = name {
            out.insert(n);
        }
    }
    out
}

fn suite_names() -> BTreeSet<String> {
    chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites())
        .suites
        .iter()
        .map(|s| (*s.name).to_owned())
        .collect()
}

#[test]
fn every_computing_module_is_covered_by_a_suite() {
    let modules = core_modules();
    let suites = suite_names();
    assert!(
        modules.len() >= 15,
        "found only {} core modules, so this test is not looking where it thinks",
        modules.len()
    );

    let exempt: BTreeSet<&str> = EXEMPT.iter().map(|(m, _)| *m).collect();
    let mut uncovered = Vec::new();
    for m in &modules {
        if exempt.contains(m.as_str()) {
            continue;
        }
        let suite = COVERED_BY
            .iter()
            .find(|(module, _)| module == m)
            .map(|(_, s)| *s);
        match suite {
            Some(s) if suites.contains(s) => {}
            Some(s) => uncovered.push(format!(
                "{m}: mapped to suite {s:?}, which the self-test does not produce"
            )),
            None => uncovered.push(format!(
                "{m}: no self-test suite and no exemption. A build identity is only \
                 as strong as the suites behind it; see CONTRIBUTING.md"
            )),
        }
    }
    assert!(
        uncovered.is_empty(),
        "{} core module(s) are invisible to `engine_selftest`:\n  {}",
        uncovered.len(),
        uncovered.join("\n  ")
    );
    eprintln!(
        "{} core modules: {} covered, {} exempt",
        modules.len(),
        modules.len() - exempt.len(),
        exempt.len()
    );
}

#[test]
fn every_exemption_carries_a_reason() {
    // An exemption nobody had to justify is a hole with a comment next to it.
    for (module, reason) in EXEMPT {
        assert!(
            reason.len() >= 20,
            "the exemption for {module:?} says only {reason:?}, which is not a reason"
        );
    }
    // And an exemption for a module that no longer exists is stale: it would
    // silently start covering for a *new* module of the same name.
    let modules = core_modules();
    for (module, _) in EXEMPT {
        assert!(
            modules.contains(*module),
            "{module:?} is exempted but no such core module exists; a stale \
             exemption would silently cover a future module of that name"
        );
    }
}

#[test]
fn the_coverage_check_would_notice_an_uncovered_module() {
    // The mutation check. The assertion above passes trivially if every module
    // is either mapped or exempt by construction, so prove the mapping is real:
    // a module name that exists in neither list must be reported.
    let exempt: BTreeSet<&str> = EXEMPT.iter().map(|(m, _)| *m).collect();
    let mapped: BTreeSet<&str> = COVERED_BY.iter().map(|(m, _)| *m).collect();
    let invented = "a_module_that_does_not_exist";
    assert!(
        !exempt.contains(invented) && !mapped.contains(invented),
        "the lists match arbitrary names, so the check above proves nothing"
    );
    // And the suite names really are checked against the running self-test,
    // rather than only against the table.
    let suites = suite_names();
    assert!(
        !suites.contains("a_suite_that_does_not_exist"),
        "the suite set matches arbitrary names"
    );
    assert!(
        suites.contains("collision"),
        "the collision suite is missing, which is the case this rule exists for"
    );
}
