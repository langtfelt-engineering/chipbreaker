// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Regenerates `tests/corpus/defect/expectations.json`.
//!
//! # Why a generated corpus needs a committed file at all
//!
//! [`chipbreaker_core::defect::corpus`] builds its 295 cases from code, so there
//! is nothing to commit in the usual sense. That is exactly the problem. Twice
//! now a change to the generator has quietly emptied cases of their defect —
//! `mid-face` anchored on the stock surface, and `rapid-clips-stock` clearing
//! above it — and both times the corpus went on reporting 295 cases while
//! several of them asked nothing. Neither showed up as a diff, because there was
//! no file to diff.
//!
//! So the file records **every case's identity and its geometry**: the kind, the
//! locale, the facing, the depth, the anchor, which segment was perturbed, and a
//! digest over the motions themselves. A change to `program()` then appears as a
//! reviewable diff naming the cases it moved, rather than as a recall figure
//! that shifts by a few percent for no stated reason.
//!
//! # What it deliberately does not record
//!
//! Any simulated result. The cases are *questions*, and pinning the engine's
//! current answers to them here would make the corpus agree with whatever the
//! engine does — which is the failure mode `tests/defect_injection.rs` exists to
//! prevent, moved into a file. The answers are asserted, not recorded:
//! `defect_injection` requires each case to inject what it claims, and
//! `deviation_recall` measures how many are found.

#![allow(missing_docs, reason = "an example binary, not API")]

use std::collections::BTreeMap;

use chipbreaker_core::defect::{DEPTHS, DefectCase, STOCK, corpus};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::sweep::Motion;

/// A canonical digest over a case's motions.
///
/// Binary, never text: the coordinates are `f64` and a decimal rendering would
/// hash the formatter's rounding rather than the geometry. The same rule as
/// every other golden in the project.
fn motions_digest(motions: &[Motion]) -> String {
    let mut h = CanonicalHash::new();
    h.begin("Motions");
    for m in motions {
        match m {
            Motion::Linear(l) => {
                h.str("linear");
                h.f64_slice(&l.start.to_array());
                h.f64_slice(&l.end.to_array());
            }
            Motion::Arc(a) => {
                h.str("arc");
                h.f64_slice(&a.center.to_array());
                h.f64(a.radius);
                h.f64(a.start_angle);
                h.f64(a.sweep);
                h.f64(a.z);
                h.f64(a.rise);
                h.str(a.plane.as_str());
            }
        }
    }
    h.end();
    h.finish().to_hex()
}

fn case_json(c: &DefectCase) -> String {
    format!(
        "    {{\n\
         \x20     \"id\": \"{}\",\n\
         \x20     \"kind\": \"{}\",\n\
         \x20     \"locale\": \"{}\",\n\
         \x20     \"facing\": \"{}\",\n\
         \x20     \"depth_mm\": {:.17e},\n\
         \x20     \"at\": [{:.17e}, {:.17e}, {:.17e}],\n\
         \x20     \"segment\": {},\n\
         \x20     \"clean_motions\": {},\n\
         \x20     \"motions\": {},\n\
         \x20     \"tool_diameter_delta_mm\": {:.17e},\n\
         \x20     \"tool_length_delta_mm\": {:.17e},\n\
         \x20     \"clean_digest\": \"{}\",\n\
         \x20     \"dirty_digest\": \"{}\"\n\
         \x20   }}",
        c.id,
        c.kind.as_str(),
        c.locale.as_str(),
        c.facing.as_str(),
        c.depth_mm,
        c.at.x,
        c.at.y,
        c.at.z,
        c.segment
            .map_or_else(|| "null".to_owned(), |s| s.to_string()),
        c.clean.len(),
        c.motions.len(),
        c.tool_diameter_delta_mm,
        c.tool_length_delta_mm,
        motions_digest(&c.clean),
        motions_digest(&c.motions),
    )
}

fn main() {
    let cases = corpus();

    // The spread, recorded so that a generator change which halves one kind is
    // visible in the header rather than only in 150 lines of case diff.
    let mut by_kind: BTreeMap<&str, usize> = BTreeMap::new();
    let mut by_locale: BTreeMap<&str, usize> = BTreeMap::new();
    let mut gouges = 0usize;
    for c in &cases {
        *by_kind.entry(c.kind.as_str()).or_default() += 1;
        *by_locale.entry(c.locale.as_str()).or_default() += 1;
        if c.kind.is_gouge() {
            gouges += 1;
        }
    }

    println!("{{");
    println!("  \"schema\": \"chipbreaker.defect-corpus\",");
    println!("  \"version\": 1,");
    println!(
        "  \"stock_mm\": [{:.17e}, {:.17e}, {:.17e}],",
        STOCK[0], STOCK[1], STOCK[2]
    );
    let depths: Vec<String> = DEPTHS.iter().map(|d| format!("{d:.17e}")).collect();
    println!("  \"depth_ladder_mm\": [{}],", depths.join(", "));
    println!("  \"case_count\": {},", cases.len());
    println!("  \"gouge_count\": {gouges},");
    println!("  \"excess_count\": {},", cases.len() - gouges);
    let kinds: Vec<String> = by_kind
        .iter()
        .map(|(k, n)| format!("\"{k}\": {n}"))
        .collect();
    println!("  \"by_kind\": {{{}}},", kinds.join(", "));
    let locales: Vec<String> = by_locale
        .iter()
        .map(|(k, n)| format!("\"{k}\": {n}"))
        .collect();
    println!("  \"by_locale\": {{{}}},", locales.join(", "));
    println!("  \"cases\": [");
    let rendered: Vec<String> = cases.iter().map(case_json).collect();
    println!("{}", rendered.join(",\n"));
    println!("  ]");
    println!("}}");
}
