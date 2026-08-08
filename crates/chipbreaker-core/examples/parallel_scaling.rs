// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The scaling curve, on a balanced job and a badly balanced one.
//!
//! Two jobs, because they measure different things. A finishing raster spreads
//! its work across the field and would scale even under a static split; a deep
//! pocket clusters all of it into a corner, and that is where dynamic
//! assignment earns its place. Reporting only the first would flatter the design.
//!
//! Efficiency is `T1 / (N * TN)`: what fraction of the ideal speedup was
//! achieved. The sequential path is timed separately, so the "overhead when
//! unused" column says what the parallel machinery costs a customer who never
//! asks for it.

#![allow(missing_docs, reason = "an example binary, not API")]

use std::time::Instant;

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::parallel::{DEFAULT_CHUNK, Schedule, cut_all_parallel};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

const SPACING: f64 = 0.35;
const METHOD: SweepMethod = SweepMethod::Analytic {
    tolerance: SPACING / 10.0,
};

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(80.0, 60.0, 20.0)),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill() -> Profile {
    flat_end_mill(6.0, 30.0, &Shank::plain(6.0, 60.0)).expect("valid")
}

fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

/// Work spread across the whole field.
fn balanced() -> Vec<Motion> {
    let mut out = Vec::new();
    let mut y = 5.0;
    let mut left = true;
    while y <= 55.0 {
        let (a, b) = if left { (5.0, 75.0) } else { (75.0, 5.0) };
        out.push(line([a, y, 16.0], [b, y, 16.0]));
        out.push(line([b, y, 16.0], [b, y + 0.2, 16.0]));
        y += 0.2;
        left = !left;
    }
    out
}

/// Work clustered into one corner: most chunks see nothing at all.
fn clustered() -> Vec<Motion> {
    let mut out = Vec::new();
    let mut z = 19.0;
    while z >= 4.0 {
        let mut y = 6.0;
        while y <= 18.0 {
            out.push(line([6.0, y, z], [20.0, y, z]));
            y += 0.6;
        }
        z -= 0.5;
    }
    out
}

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

fn run_sequential(profile: &Profile, motions: &[Motion]) -> (f64, String) {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let started = Instant::now();
    cut_all(
        &mut field,
        profile,
        motions,
        METHOD,
        &mut scratch,
        DEFAULT_BATCH,
    );
    (started.elapsed().as_secs_f64(), digest(&field))
}

fn run_parallel_batched(
    profile: &Profile,
    motions: &[Motion],
    threads: usize,
    batch: usize,
) -> (f64, String) {
    let mut field = stock();
    let started = Instant::now();
    cut_all_parallel(
        &mut field,
        profile,
        motions,
        METHOD,
        batch,
        Schedule {
            threads,
            chunk: DEFAULT_CHUNK,
            chaos: None,
        },
    );
    (started.elapsed().as_secs_f64(), digest(&field))
}

fn run_parallel(profile: &Profile, motions: &[Motion], threads: usize) -> (f64, String) {
    let mut field = stock();
    let started = Instant::now();
    cut_all_parallel(
        &mut field,
        profile,
        motions,
        METHOD,
        DEFAULT_BATCH,
        Schedule {
            threads,
            chunk: DEFAULT_CHUNK,
            chaos: None,
        },
    );
    (started.elapsed().as_secs_f64(), digest(&field))
}

fn scaling(name: &str, profile: &Profile, motions: &[Motion]) {
    println!("\n=== {name} ({} motions) ===", motions.len());
    let (t_seq, want) = run_sequential(profile, motions);
    println!("sequential (batch::cut_all)   {:>8.3} s", t_seq);

    let (t1, got1) = run_parallel(profile, motions, 1);
    assert_eq!(got1, want, "one worker diverged from sequential");
    println!(
        "parallel, 1 worker            {:>8.3} s   overhead when unused {:+.1}%",
        t1,
        (t1 / t_seq - 1.0) * 100.0
    );
    println!(
        "\n{:>8}{:>10}{:>10}{:>12}",
        "threads", "seconds", "speedup", "efficiency"
    );
    for threads in [1usize, 2, 4, 8, 16] {
        let (t, got) = run_parallel(profile, motions, threads);
        assert_eq!(got, want, "{threads} workers diverged from sequential");
        #[allow(clippy::cast_precision_loss, reason = "a small thread count")]
        let n = threads as f64;
        println!(
            "{threads:>8}{t:>10.3}{:>10.2}x{:>11.1}%",
            t1 / t,
            t1 / t / n * 100.0
        );
    }
    println!("every thread count produced the same field digest as sequential.");
}

fn main() {
    let available = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
    println!("host reports {available} available cores (not hashed; host detail)");
    println!("stock 80 x 60 x 20 mm at {SPACING} mm, chunk {DEFAULT_CHUNK} rays");

    let profile = mill();
    scaling("balanced raster", &profile, &balanced());
    scaling("clustered pocket", &profile, &clustered());

    // Batch size. It is invisible to the answer (ADR 0006), but it decides how
    // many times the thread scope is entered -- once per bundle per batch -- so
    // a job cut into many small batches pays the spawn cost many times.
    println!(
        "
=== batch size, balanced raster, 8 workers ==="
    );
    println!("{:>8}{:>10}{:>10}", "batch", "seconds", "vs 32");
    let raster = balanced();
    let (base, _) = run_parallel_batched(&profile, &raster, 8, 32);
    for batch in [32usize, 128, 512, 2048] {
        let (t, _) = run_parallel_batched(&profile, &raster, 8, batch);
        println!("{batch:>8}{t:>10.3}{:>9.2}x", base / t);
    }

    // Chunk size, on the clustered job where it matters most.
    println!("\n=== chunk size, clustered pocket, 8 workers ===");
    println!("{:>8}{:>10}", "chunk", "seconds");
    let motions = clustered();
    for chunk in [16usize, 64, 256, 1024, 4096] {
        let mut field = stock();
        let started = Instant::now();
        cut_all_parallel(
            &mut field,
            &profile,
            &motions,
            METHOD,
            DEFAULT_BATCH,
            Schedule {
                threads: 8,
                chunk,
                chaos: None,
            },
        );
        println!("{chunk:>8}{:>10.3}", started.elapsed().as_secs_f64());
    }
}
