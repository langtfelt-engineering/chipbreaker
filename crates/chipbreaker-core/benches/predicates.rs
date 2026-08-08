// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// `criterion_group!` expands to a public function it does not document, and the
// workspace denies missing docs. Scoped to the benches, where the lint buys us
// nothing anyway.
#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Throughput of the exact geometric predicates.
//!
//! The number that matters is not the absolute throughput but the **ratio**
//! between the two input families:
//!
//! - **generic** — inputs where the floating-point filter's error bound is
//!   satisfied on the first try, so the predicate answers in a handful of
//!   multiplies and never allocates.
//! - **degenerate** — exactly and near-exactly degenerate inputs, where the
//!   filter cannot decide and the predicate escalates through the adaptive
//!   stages to exact expansion arithmetic.
//!
//! That ratio is how much degeneracy costs, and it feeds a real decision in the
//! extractor:
//! dual contouring evaluates predicates on grid-aligned data, which is
//! degenerate far more often than random data is. If the exact path costs two
//! orders of magnitude more than the filtered one, that has to be designed
//! around rather than discovered.

use chipbreaker_core::math::{Vec2, Vec3};
use chipbreaker_core::predicates::{ADAPTIVE, Predicates};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::hint::black_box;

/// Fixed, so successive benchmark runs measure the same work. A benchmark whose
/// inputs vary run to run is a random number generator with a chart.
const BENCH_SEED: u64 = 0x0000_C41B_0000_0030;

/// Inputs per batch. Large enough to swamp timer overhead, small enough that the
/// batch stays in cache.
const BATCH: usize = 1024;

fn p2(rng: &mut StdRng) -> Vec2 {
    Vec2::new(rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0))
}

fn p3(rng: &mut StdRng) -> Vec3 {
    Vec3::new(
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
        rng.random_range(-1.0..1.0),
    )
}

/// Points in general position: the filter resolves every one of these.
fn generic_2d(rng: &mut StdRng) -> Vec<[Vec2; 3]> {
    (0..BATCH).map(|_| [p2(rng), p2(rng), p2(rng)]).collect()
}

/// Points where the third lies on, or a few ULPs from, the line through the
/// first two — the case the filter cannot resolve.
fn degenerate_2d(rng: &mut StdRng) -> Vec<[Vec2; 3]> {
    (0..BATCH)
        .map(|_| {
            let a = p2(rng);
            let b = p2(rng);
            let t: f64 = rng.random_range(1.5..3.0);
            let mut c = Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            for _ in 0..rng.random_range(0u32..3) {
                c.y = c.y.next_up();
            }
            [a, b, c]
        })
        .collect()
}

fn generic_3d(rng: &mut StdRng) -> Vec<[Vec3; 4]> {
    (0..BATCH)
        .map(|_| [p3(rng), p3(rng), p3(rng), p3(rng)])
        .collect()
}

fn degenerate_3d(rng: &mut StdRng) -> Vec<[Vec3; 4]> {
    (0..BATCH)
        .map(|_| {
            let (a, b, c) = (p3(rng), p3(rng), p3(rng));
            let (u, v): (f64, f64) = (rng.random_range(-1.0..2.0), rng.random_range(-1.0..2.0));
            let mut d = Vec3::new(
                a.x + (b.x - a.x) * u + (c.x - a.x) * v,
                a.y + (b.y - a.y) * u + (c.y - a.y) * v,
                a.z + (b.z - a.z) * u + (c.z - a.z) * v,
            );
            for _ in 0..rng.random_range(0u32..3) {
                d.z = d.z.next_up();
            }
            [a, b, c, d]
        })
        .collect()
}

fn bench_predicates(c: &mut Criterion) {
    let mut rng = StdRng::seed_from_u64(BENCH_SEED);
    let generic2 = generic_2d(&mut rng);
    let degenerate2 = degenerate_2d(&mut rng);
    let generic3 = generic_3d(&mut rng);
    let degenerate3 = degenerate_3d(&mut rng);

    let mut group = c.benchmark_group("orient2d");
    group.throughput(Throughput::Elements(BATCH as u64));
    for (name, data) in [("generic", &generic2), ("degenerate", &degenerate2)] {
        group.bench_with_input(BenchmarkId::from_parameter(name), data, |bencher, data| {
            bencher.iter(|| {
                let mut acc = 0i32;
                for [a, b, c] in data {
                    acc += i32::from(
                        ADAPTIVE
                            .orient2d(black_box(*a), black_box(*b), black_box(*c))
                            .as_i8(),
                    );
                }
                black_box(acc)
            });
        });
    }
    group.finish();

    let mut group = c.benchmark_group("orient3d");
    group.throughput(Throughput::Elements(BATCH as u64));
    for (name, data) in [("generic", &generic3), ("degenerate", &degenerate3)] {
        group.bench_with_input(BenchmarkId::from_parameter(name), data, |bencher, data| {
            bencher.iter(|| {
                let mut acc = 0i32;
                for [a, b, c, d] in data {
                    acc += i32::from(
                        ADAPTIVE
                            .orient3d(black_box(*a), black_box(*b), black_box(*c), black_box(*d))
                            .as_i8(),
                    );
                }
                black_box(acc)
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_predicates);
criterion_main!(benches);
