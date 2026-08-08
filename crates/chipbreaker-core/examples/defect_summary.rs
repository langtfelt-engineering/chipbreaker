// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! What the injected-defect corpus contains.
//!
//! A spread check rather than a test: the corpus is only an oracle if it asks
//! questions across the whole space, and a count alone would hide a corpus that
//! was two hundred variations of one case.

#![allow(missing_docs, reason = "an example binary, not API")]

use std::collections::BTreeMap;

use chipbreaker_core::defect::{DEPTHS, DefectKind, Locale, corpus};

fn main() {
    let cases = corpus();
    println!("{} cases\n", cases.len());

    let spacing = 0.4;
    let mut by_kind: BTreeMap<&str, usize> = BTreeMap::new();
    let mut by_locale: BTreeMap<&str, usize> = BTreeMap::new();
    let mut by_facing: BTreeMap<&str, usize> = BTreeMap::new();
    let mut sub_cell = 0usize;
    let mut above_two = 0usize;
    let mut gouges = 0usize;

    for c in &cases {
        *by_kind.entry(c.kind.as_str()).or_default() += 1;
        *by_locale.entry(c.locale.as_str()).or_default() += 1;
        *by_facing.entry(c.facing.as_str()).or_default() += 1;
        let cells = c.cells(spacing);
        if cells < 1.0 {
            sub_cell += 1;
        }
        if cells >= 2.0 {
            above_two += 1;
        }
        if c.kind.is_gouge() {
            gouges += 1;
        }
    }

    println!("by kind:");
    for (k, n) in &by_kind {
        println!("  {k:<22}{n:>4}");
    }
    println!("\nby locale:");
    for (k, n) in &by_locale {
        println!("  {k:<22}{n:>4}");
    }
    println!("\nby facing:");
    for (k, n) in &by_facing {
        println!("  {k:<22}{n:>4}");
    }
    println!(
        "\nsign: {gouges} gouges, {} excess stock",
        cases.len() - gouges
    );
    println!("depth at h = {spacing} mm: {sub_cell} sub-cell, {above_two} at or above 2 cells");
    println!("depth ladder: {DEPTHS:?}");

    // Every combination that is plausible should actually appear, or the spread
    // claim is not true.
    let mut missing = Vec::new();
    for kind in DefectKind::all() {
        for locale in Locale::all() {
            if !cases.iter().any(|c| c.kind == kind && c.locale == locale) {
                missing.push(format!("{}/{}", kind.as_str(), locale.as_str()));
            }
        }
    }
    println!("\ncombinations with no case: {}", missing.len());
    for m in &missing {
        println!("  {m}");
    }
}
