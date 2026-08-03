// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! What fraction of real toolpath segments are arcs?
//!
//! `MotionSegment` is 192 bytes, of which `ArcData` is 56 carried **inline**.
//! Every linear move therefore pays 29% for a field it does not use. Boxing or
//! side-tabling the arc data would recover that on linear-dominated programs and
//! cost an indirection on arcs.
//!
//! U5 holds the IR alongside the dexel field, so the question is worth an actual
//! number rather than an assumption. Measured now, to be decided at U10.
//!
//! Run with: `cargo run --release -p chipbreaker-gcode --example arc_fraction`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::toolpath::MotionKind;
use chipbreaker_gcode::resolve::{ParseOptions, parse};
use std::path::PathBuf;

fn corpus_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/gcode")
}

fn main() {
    let mut total = 0usize;
    let mut arcs = 0usize;
    let mut files = 0usize;
    let mut worst: (f64, String) = (0.0, String::new());

    let mut entries: Vec<PathBuf> = std::fs::read_dir(corpus_dir())
        .expect("corpus directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "nc"))
        .collect();
    // Sorted, because an unordered walk over the filesystem would make the
    // reported worst-case depend on the directory's iteration order.
    entries.sort();

    for path in entries {
        let text = std::fs::read_to_string(&path).expect("readable");
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("?")
            .to_owned();
        let Ok((toolpath, _, _)) = parse(&text, &name, &ParseOptions::default(), None) else {
            continue;
        };
        if toolpath.segments.is_empty() {
            continue;
        }
        files += 1;
        let here_arcs = toolpath
            .segments
            .iter()
            .filter(|s| matches!(s.kind, MotionKind::Arc | MotionKind::Helix))
            .count();
        let fraction = here_arcs as f64 / toolpath.segments.len() as f64;
        if fraction > worst.0 {
            worst = (fraction, name);
        }
        arcs += here_arcs;
        total += toolpath.segments.len();
    }

    let fraction = arcs as f64 / total as f64;
    println!("{files} corpus programs, {total} segments, {arcs} of them arcs or helices");
    println!("arc fraction: {:.2}%", fraction * 100.0);
    println!(
        "worst single program: {:.1}% ({})",
        worst.0 * 100.0,
        worst.1
    );
    println!();

    // What boxing would save, per million segments.
    let segment = size_of::<chipbreaker_core::toolpath::MotionSegment>();
    let arc = size_of::<chipbreaker_core::toolpath::ArcData>();
    let boxed = segment - arc + size_of::<usize>();
    let inline_mb = (1_000_000 * segment) as f64 / (1024.0 * 1024.0);
    let boxed_mb = (1_000_000 * boxed) as f64 / (1024.0 * 1024.0)
        + (fraction * 1_000_000.0 * arc as f64) / (1024.0 * 1024.0);
    println!("per million segments at this arc fraction:");
    println!("  inline ArcData  {inline_mb:>8.1} MB   ({segment} B/segment)");
    println!("  boxed ArcData   {boxed_mb:>8.1} MB   ({boxed} B/segment + {arc} B per arc)");
    println!(
        "  saving          {:>8.1} MB   ({:.0}%)",
        inline_mb - boxed_mb,
        (inline_mb - boxed_mb) / inline_mb * 100.0
    );
    println!();
    println!("NOTE: the corpus is hand-written landmine cases, so its arc fraction is");
    println!("far higher than real CAM output, where long raster passes dominate. Treat");
    println!("this as an upper bound on the arc share and therefore a LOWER bound on the");
    println!("saving.");
}
