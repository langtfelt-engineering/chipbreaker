// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// `criterion_group!` expands to a public function it does not document, and the
// workspace denies missing docs. Scoped to the benches, where the lint buys us
// nothing anyway.
#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Throughput of the span set algebra.
//!
//! This is the hot loop of the entire product: every cut is a
//! [`Spans::subtract`] on a dexel ray, and a realistic job performs tens of
//! millions of them. The merge-scan is `O(|a| + |b|)` by construction, so what
//! these benchmarks are really checking is that the constant factor is small and
//! that it stays linear as the sets grow.
//!
//! Both the allocating and the scratch-buffer forms are measured. The difference
//! between them is the allocation cost a real sweep avoids by keeping one
//! scratch `Spans` per ray, and it is worth knowing rather than assuming.

use chipbreaker_core::spans::{Span, Spans};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::hint::black_box;

/// Fixed, so successive runs measure the same work.
const BENCH_SEED: u64 = 0x0000_C41B_0000_0031;

/// Span counts to measure at. Covers a lightly-cut ray, a heavily-cut one, and
/// the pathological case of a ray crossing a thousand thin features.
const SIZES: [usize; 3] = [10, 100, 1000];

/// A set of exactly `n` disjoint spans on a grid coarse enough that none of them
/// merge.
fn build(rng: &mut StdRng, n: usize) -> Spans {
    let mut out = Spans::with_capacity(n);
    let mut t = 0.0f64;
    for _ in 0..n {
        let len = rng.random_range(0.5f64..3.0);
        let gap = rng.random_range(0.5f64..3.0);
        out.push_merge(Span::new(t, t + len));
        t += len + gap;
    }
    assert_eq!(out.len(), n, "the generator must not merge its own spans");
    out
}

/// A second set offset by half a period, so it interleaves with the first rather
/// than nesting inside it — the arrangement that makes the merge-scan do the
/// most work.
fn build_offset(rng: &mut StdRng, n: usize) -> Spans {
    let base = build(rng, n);
    let shifted: Vec<Span> = base.iter().map(|s| s.translated(1.25)).collect();
    Spans::from_unsorted(shifted)
}

fn bench_set_operations(c: &mut Criterion) {
    let mut rng = StdRng::seed_from_u64(BENCH_SEED);

    for size in SIZES {
        let a = build(&mut rng, size);
        let b = build_offset(&mut rng, size);

        let mut group = c.benchmark_group(format!("spans/{size}"));
        // One "element" is one span in the left operand, so the reported
        // throughput is directly comparable across sizes; a linear algorithm
        // holds it roughly constant.
        group.throughput(Throughput::Elements(size as u64));

        group.bench_function(BenchmarkId::new("union", size), |bencher| {
            bencher.iter(|| black_box(black_box(&a).union(black_box(&b))));
        });
        group.bench_function(BenchmarkId::new("intersect", size), |bencher| {
            bencher.iter(|| black_box(black_box(&a).intersect(black_box(&b))));
        });
        group.bench_function(BenchmarkId::new("subtract", size), |bencher| {
            bencher.iter(|| black_box(black_box(&a).subtract(black_box(&b))));
        });

        // The form a real toolpath sweep uses: no allocation after the first.
        group.bench_function(BenchmarkId::new("subtract_into", size), |bencher| {
            let mut scratch = Spans::with_capacity(2 * size);
            bencher.iter(|| {
                black_box(&a).subtract_into(black_box(&b), &mut scratch);
                black_box(scratch.len())
            });
        });

        group.bench_function(BenchmarkId::new("measure", size), |bencher| {
            bencher.iter(|| black_box(black_box(&a).measure()));
        });

        group.bench_function(BenchmarkId::new("contains", size), |bencher| {
            let probe = a.hull().map_or(0.0, |h| h.midpoint());
            bencher.iter(|| black_box(black_box(&a).contains(black_box(probe))));
        });

        group.finish();
    }
}

fn bench_push_merge(c: &mut Criterion) {
    let mut rng = StdRng::seed_from_u64(BENCH_SEED);
    let mut group = c.benchmark_group("spans/push_merge");

    for size in SIZES {
        // Pre-generate the spans so the benchmark measures insertion, not
        // random number generation.
        let ascending: Vec<Span> = {
            let mut t = 0.0f64;
            (0..size)
                .map(|_| {
                    let len = rng.random_range(0.5f64..3.0);
                    let gap = rng.random_range(0.5f64..3.0);
                    let s = Span::new(t, t + len);
                    t += len + gap;
                    s
                })
                .collect()
        };

        group.throughput(Throughput::Elements(size as u64));

        // The hot path: appending at or after the current maximum, which is what
        // a front-to-back sweep along a dexel ray does. O(1) per span.
        group.bench_function(BenchmarkId::new("ascending", size), |bencher| {
            bencher.iter(|| {
                let mut set = Spans::with_capacity(size);
                for s in &ascending {
                    set.push_merge(black_box(*s));
                }
                black_box(set.len())
            });
        });

        // The slow path: every insert lands before the end and triggers a full
        // re-normalization. Measured so the cost of getting the order wrong is a
        // known number rather than a surprise.
        let descending: Vec<Span> = ascending.iter().rev().copied().collect();
        group.bench_function(BenchmarkId::new("descending", size), |bencher| {
            bencher.iter(|| {
                let mut set = Spans::with_capacity(size);
                for s in &descending {
                    set.push_merge(black_box(*s));
                }
                black_box(set.len())
            });
        });

        // Bulk construction from an unsorted pile: what a caller should use
        // instead of repeated out-of-order pushes.
        group.bench_function(BenchmarkId::new("from_unsorted", size), |bencher| {
            bencher.iter(|| black_box(Spans::from_unsorted(descending.clone())));
        });
    }

    group.finish();
}

criterion_group!(benches, bench_set_operations, bench_push_merge);
criterion_main!(benches);
