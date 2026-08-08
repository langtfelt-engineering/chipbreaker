// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! How much of a real toolpath is each sweep case?
//!
//! The sweep decomposes linear motion into three cases:
//!
//! - **A**, `dz = 0`: horizontal. Reduces to the tool at each end plus a prism.
//! - **B**, `dxy = 0`: pure plunge. The swept volume is itself a solid of
//!   revolution, so the stationary ray cast handles it whole.
//! - **C**, both non-zero: a general ramp. Neither reduction applies.
//!
//! Case C is the expensive one, and whether it deserves a closed form or
//! error-bounded sub-stepping depends entirely on how often it occurs. Measuring
//! that before choosing is the point of this file.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-gcode --example motion_classes`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::toolpath::MotionKind;
use chipbreaker_gcode::resolve::{ParseOptions, parse};
use std::path::PathBuf;

/// Below this, a displacement counts as zero for classification.
///
/// Deliberately tiny. A move of a nanometre in `z` is a horizontal move that
/// picked up a rounding artefact somewhere upstream, not a ramp — but the
/// threshold has to be far below any real machining displacement or it would
/// silently reclassify shallow finishing ramps, which are exactly the Case C
/// work that matters.
const ZERO: f64 = 1.0e-9;

fn corpus_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/gcode")
}

#[derive(Default, Clone, Copy)]
struct Counts {
    a_horizontal: u64,
    b_plunge: u64,
    c_ramp: u64,
    degenerate: u64,
    arc: u64,
}

impl Counts {
    fn linear_total(self) -> u64 {
        self.a_horizontal + self.b_plunge + self.c_ramp + self.degenerate
    }

    fn add(&mut self, other: Self) {
        self.a_horizontal += other.a_horizontal;
        self.b_plunge += other.b_plunge;
        self.c_ramp += other.c_ramp;
        self.degenerate += other.degenerate;
        self.arc += other.arc;
    }
}

fn main() {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(corpus_dir())
        .expect("corpus directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "nc"))
        .collect();
    // Sorted: an unordered walk would make the per-file table depend on the
    // filesystem's iteration order.
    paths.sort();

    let mut total = Counts::default();
    let mut files = 0u32;
    let mut worst_c: (f64, String) = (0.0, String::new());

    println!(
        "  {:<28} {:>8} {:>8} {:>8} {:>8} {:>7}",
        "program", "A horiz", "B plunge", "C ramp", "arcs", "C %"
    );
    for path in paths {
        let Ok(text) = std::fs::read_to_string(&path) else {
            continue;
        };
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

        let mut counts = Counts::default();
        for segment in &toolpath.segments {
            if matches!(segment.kind, MotionKind::Arc | MotionKind::Helix) {
                counts.arc += 1;
                continue;
            }
            let delta = segment.end - segment.start;
            let horizontal = (delta.x * delta.x + delta.y * delta.y).sqrt();
            let vertical = delta.z.abs();
            match (horizontal > ZERO, vertical > ZERO) {
                (false, false) => counts.degenerate += 1,
                (true, false) => counts.a_horizontal += 1,
                (false, true) => counts.b_plunge += 1,
                (true, true) => counts.c_ramp += 1,
            }
        }

        let linear = counts.linear_total().max(1);
        #[allow(clippy::cast_precision_loss, reason = "counts are small")]
        let share = counts.c_ramp as f64 / linear as f64 * 100.0;
        if share > worst_c.0 {
            worst_c = (share, name.clone());
        }
        println!(
            "  {name:<28} {:>8} {:>8} {:>8} {:>8} {share:>6.1}%",
            counts.a_horizontal, counts.b_plunge, counts.c_ramp, counts.arc
        );
        total.add(counts);
    }

    let linear = total.linear_total().max(1);
    #[allow(clippy::cast_precision_loss, reason = "counts are small")]
    let pct = |n: u64| n as f64 / linear as f64 * 100.0;

    println!();
    println!(
        "--- {files} programs, {linear} linear segments, {} arcs ---",
        total.arc
    );
    println!(
        "  A  horizontal (dz = 0)      {:>8}   {:>6.2}%",
        total.a_horizontal,
        pct(total.a_horizontal)
    );
    println!(
        "  B  plunge     (dxy = 0)     {:>8}   {:>6.2}%",
        total.b_plunge,
        pct(total.b_plunge)
    );
    println!(
        "  C  ramp       (both)        {:>8}   {:>6.2}%",
        total.c_ramp,
        pct(total.c_ramp)
    );
    println!(
        "     degenerate (no motion)   {:>8}   {:>6.2}%",
        total.degenerate,
        pct(total.degenerate)
    );
    println!();
    println!(
        "  worst single program for Case C: {:.1}% ({})",
        worst_c.0, worst_c.1
    );
    println!();
    println!("NOTE: the corpus is hand-written landmine cases, not CAM output. Real");
    println!("finishing work is overwhelmingly Case A -- long raster passes at constant");
    println!("depth -- so this OVERSTATES the Case C share. Treat it as an upper bound.");
}
