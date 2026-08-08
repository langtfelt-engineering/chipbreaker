// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! What a million IR segments cost in memory.
//!
//! A whole toolpath is held beside the dexel field, and a million-segment
//! program is an ordinary finishing pass. This is the number that decides
//! whether that is a footnote or a design constraint, so it is measured rather
//! than estimated from the struct definition -- padding and the `Option<ArcData>`
//! make the two differ.
//!
//! Run with: `cargo run --release -p chipbreaker-gcode --example ir_memory`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::toolpath::{ArcData, MotionSegment, PathEvent, Toolpath};

fn main() {
    let segment = size_of::<MotionSegment>();
    let arc = size_of::<ArcData>();
    let event = size_of::<PathEvent>();

    println!("{:<34} {:>8}", "MotionSegment", segment);
    println!("{:<34} {:>8}", "  of which ArcData (inline)", arc);
    println!("{:<34} {:>8}", "PathEvent", event);
    println!(
        "{:<34} {:>8}",
        "Toolpath (header + 3 Vec)",
        size_of::<Toolpath>()
    );
    println!();

    for count in [100_000usize, 1_000_000, 10_000_000] {
        let bytes = count * segment;
        println!(
            "{count:>10} segments  {:>8.1} MB",
            bytes as f64 / (1024.0 * 1024.0)
        );
    }
    println!();
    println!(
        "A million segments cost {:.0} MB. The arc payload is carried inline in every",
        (1_000_000 * segment) as f64 / (1024.0 * 1024.0)
    );
    println!("segment rather than boxed, so a program of pure linear moves pays for arcs");
    println!("it does not have -- which is the right trade while cutting wants arcs resident,");
    println!("and the first thing to revisit if this number becomes a problem.");
}
