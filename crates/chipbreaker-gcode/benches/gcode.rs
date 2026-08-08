// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Parser throughput, and the number that sets a field's memory budget.
//!
//! # The measurement that matters most is not a rate
//!
//! It is **IR bytes per segment**. A whole toolpath is held in memory beside
//! its dexel field, and a million-segment program is an ordinary size for a
//! finishing pass. If a segment costs 200 bytes that is 200 MB before any stock
//! exists; if it costs 500, the budget has to be rethought rather than
//! discovered.
//!
//! # On the large file
//!
//! The 100k-line benchmark input is **synthetic**. It is generated here from a
//! realistic raster surfacing pattern rather than taken from a CAM post,
//! because a real post's output is somebody's copyrighted part program and could
//! not be committed. Its statistics — line length, the ratio of rapids to feeds,
//! the mix of arcs — are chosen to look like CAM output, but nobody should read
//! the resulting rate as "Chipbreaker parses Mastercam at N lines a second".

use chipbreaker_gcode::resolve::{ParseOptions, parse};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::fmt::Write as _;
use std::hint::black_box;

/// A raster surfacing pass: the commonest shape of real CAM output.
///
/// Long runs of short `G1` moves in alternating directions, with a rapid at the
/// end of each pass, and an arc every so often where a corner was rounded.
fn surfacing_program(lines: usize) -> String {
    let mut text = String::with_capacity(lines * 24);
    text.push_str("%\nO1000 (synthetic surfacing pass)\nG21 G17 G90 G94 G54\n");
    text.push_str("T1 M6\nS12000 M3\nG43 H1\nG0 X0. Y0. Z5.\nG1 Z-0.5 F1200.\n");

    let mut y = 0.0f64;
    let mut written = 8usize;
    let mut pass = 0usize;
    let mut last_x = 0.0f64;
    while written < lines {
        let forward = pass.is_multiple_of(2);
        for step in 0..40u32 {
            if written >= lines {
                break;
            }
            let x = if forward {
                f64::from(step) * 2.5
            } else {
                100.0 - f64::from(step) * 2.5
            };
            // A coordinate needing full precision every so often, so the
            // benchmark is not measuring the easy case exclusively.
            let value = if step.is_multiple_of(7) {
                x + 0.048_155_585_660_824_2
            } else {
                x
            };
            let _ = writeln!(text, "X{value:.6} Y{y:.4}");
            last_x = value;
            written += 1;
        }
        if written < lines {
            // The step-over arc joins the end of one pass to the start of the
            // next at the SAME x. An earlier version sent it to the far end of
            // the pass, which asked for a 2.6 mm chord on a 0.6 mm radius --
            // rejected, correctly, as a radius too small to reach. The parser
            // caught the benchmark's own bad geometry before it was measured.
            let _ = writeln!(text, "G3 X{last_x:.6} Y{:.4} R0.4", y + 0.6);
            // G2/G3 is modal, so the next bare `X.. Y..` line would fire
            // another arc -- with no centre and no radius, which every control
            // rejects and so does this parser. The G1 is not decoration. This
            // is the second time the parser caught bad geometry in its own
            // benchmark input before measuring it.
            text.push_str(
                "G1
",
            );
            written += 2;
        }
        y += 1.2;
        pass += 1;
    }
    text.push_str("G0 Z25.\nM5\nM30\n%\n");
    text
}

/// A drilling program, which is where cycle expansion dominates.
fn drilling_program(holes: usize) -> String {
    let mut text = String::with_capacity(holes * 20);
    text.push_str("%\nO2000 (synthetic drilling)\nG21 G17 G90 G94 G54\n");
    text.push_str("T2 M6\nS4000 M3\nG0 X0. Y0. Z10.\nF250.\nG99 G83 X0. Y0. Z-12.5 R2. Q3.\n");
    for i in 0..holes {
        let _ = writeln!(
            text,
            "X{:.3} Y{:.3}",
            f64::from(u32::try_from(i % 40).unwrap_or(0)) * 7.5,
            f64::from(u32::try_from(i / 40).unwrap_or(0)) * 7.5
        );
    }
    text.push_str("G80\nG0 Z25.\nM5\nM30\n%\n");
    text
}

fn bench_end_to_end(c: &mut Criterion) {
    let mut group = c.benchmark_group("gcode/end-to-end");
    for lines in [1_000usize, 10_000, 100_000] {
        let text = surfacing_program(lines);
        let actual = text.lines().count();
        group.throughput(Throughput::Elements(actual as u64));
        group.bench_with_input(BenchmarkId::from_parameter(actual), &text, |b, text| {
            b.iter(|| {
                let (path, _, _) =
                    parse(black_box(text), "bench", &ParseOptions::default(), None).expect("valid");
                path.segments.len()
            });
        });
    }
    group.finish();
}

fn bench_stages(c: &mut Criterion) {
    // Each stage on the same input, so the shares are comparable.
    let text = surfacing_program(20_000);
    let lines = text.lines().count() as u64;

    let mut group = c.benchmark_group("gcode/stages");
    group.throughput(Throughput::Elements(lines));

    group.bench_function("lex", |b| {
        b.iter(|| {
            let mut diagnostics = chipbreaker_gcode::diag::Diagnostics::new();
            chipbreaker_gcode::lex::lex(black_box(&text), 0, &mut diagnostics)
                .expect("valid")
                .len()
        });
    });

    let mut diagnostics = chipbreaker_gcode::diag::Diagnostics::new();
    let raw = chipbreaker_gcode::lex::lex(&text, 0, &mut diagnostics).expect("valid");
    group.bench_function("assemble", |b| {
        b.iter(|| {
            let mut total = 0usize;
            for line in black_box(&raw) {
                if line.is_empty() {
                    continue;
                }
                total += chipbreaker_gcode::block::assemble(line)
                    .expect("valid")
                    .g_codes
                    .len();
            }
            total
        });
    });

    group.bench_function("lex+assemble+resolve", |b| {
        b.iter(|| {
            parse(black_box(&text), "bench", &ParseOptions::default(), None)
                .expect("valid")
                .0
                .segments
                .len()
        });
    });
    group.finish();
}

fn bench_arc_forms(c: &mut Criterion) {
    // The I/J/K path is given a centre; the R path derives one through a square
    // root and two divisions. Whether that costs anything is worth knowing
    // before cutting leans on either.
    let ijk: String = std::iter::once("G21 G90 G17 G0 X10. Y0.\nF500.\n".to_owned())
        .chain((0..5_000).map(|_| "G3 X0. Y10. I-10. J0.\nG3 X10. Y0. I0. J-10.\n".to_owned()))
        .collect();
    let radius: String = std::iter::once("G21 G90 G17 G0 X10. Y0.\nF500.\n".to_owned())
        .chain((0..5_000).map(|_| "G3 X0. Y10. R10.\nG3 X10. Y0. R10.\n".to_owned()))
        .collect();

    let mut group = c.benchmark_group("gcode/arc-form");
    group.throughput(Throughput::Elements(10_000));
    for (name, text) in [("ijk", &ijk), ("r", &radius)] {
        group.bench_with_input(BenchmarkId::from_parameter(name), text, |b, text| {
            b.iter(|| {
                parse(black_box(text), "bench", &ParseOptions::default(), None)
                    .expect("valid")
                    .0
                    .segments
                    .len()
            });
        });
    }
    group.finish();
}

fn bench_cycle_expansion(c: &mut Criterion) {
    let text = drilling_program(5_000);
    let lines = text.lines().count() as u64;
    let mut group = c.benchmark_group("gcode/cycles");
    group.throughput(Throughput::Elements(lines));
    group.bench_function("g83 expansion", |b| {
        b.iter(|| {
            parse(black_box(&text), "bench", &ParseOptions::default(), None)
                .expect("valid")
                .0
                .segments
                .len()
        });
    });
    group.finish();
}

criterion_group!(
    benches,
    bench_end_to_end,
    bench_stages,
    bench_arc_forms,
    bench_cycle_expansion
);
criterion_main!(benches);
