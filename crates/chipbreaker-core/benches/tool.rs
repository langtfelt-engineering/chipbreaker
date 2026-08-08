// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// `criterion_group!` expands to a public function it does not document, and the
// workspace denies missing docs. Scoped to the benches, where the lint buys us
// nothing anyway.
#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Root solving and ray-versus-tool throughput.
//!
//! # The number that decides a field's budget
//!
//! Cutting subtracts the tool from a tri-dexel field by casting three orthogonal
//! bundles of rays at it, millions of times per simulation. The cost of one such
//! ray is therefore a hard constraint on the whole product, and it is set almost
//! entirely by which surfaces the tool's profile generates:
//!
//! - a flat end mill is cylinders and discs, so every element is a **quadratic**;
//! - a ball nose adds a sphere, still quadratic;
//! - a bull nose or a barrel adds a **torus**, which is a quartic.
//!
//! The flat-versus-bull ratio measured here is the price of a corner radius, and
//! it is the number to quote when someone asks why a toroidal cutter simulates
//! more slowly than a square one.
//!
//! # Why the quartic is benchmarked on its own as well
//!
//! Section 8 abandoned Ferrari's closed form for a derivative-and-bracket
//! method, which is unconditionally correct and unambiguously slower: it solves
//! a cubic and then runs safeguarded Newton on each bracket. That trade was made
//! on correctness grounds — Ferrari lost every significant digit when `|b/a|`
//! was large, which is the *normal* case for a ray meeting a torus away from the
//! origin — but the cost of it should be visible rather than folded into a
//! composite figure.
//!
//! # What it measured, and the one number that is a problem
//!
//! On an x86-64 laptop, per solve:
//!
//! | degree | time | relative |
//! |---|---:|---:|
//! | quadratic | 11.5 ns | 1x |
//! | cubic | 94 ns | 8x |
//! | quartic | 560 ns | **49x** |
//!
//! And per coherent ray, by tool:
//!
//! | tool | throughput | relative |
//! |---|---:|---:|
//! | flat | 13.6 M/s | 1x |
//! | drill | 7.8 M/s | 1.7x |
//! | ball | 7.0 M/s | 1.9x |
//! | bull | 3.1 M/s | 4.4x |
//! | barrel | 0.72 M/s | **19x** |
//!
//! The barrel is the number to worry about. It is not a surprise once the two
//! tables are read together — a barrel's whole cutting length is one torus, so
//! nearly every ray pays for two quartic solves, where a bull nose pays only for
//! the rays that clip its corner — but 19x is a real cost and a field feels it.
//!
//! **This is deliberate and it is recoverable.** The quartic is slow because it
//! is always solved the safe way. The obvious recovery is to try Ferrari first
//! and check the residual of each root it produces, falling back to bracketing
//! only when the check fails — which restores closed-form speed on the
//! well-conditioned majority while keeping the guarantee on the cases that
//! motivated the change. That is not done here because it is an optimisation
//! that can only be justified against a measurement, and this is the
//! measurement. It belongs with parallel performance work, not with tool geometry.

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::roots::{solve_cubic, solve_quadratic, solve_quartic};
use chipbreaker_core::spans::Spans;
use chipbreaker_core::tool::catalog::{
    Shank, ball_end_mill, barrel_end_mill, bull_end_mill, drill, flat_end_mill,
};
use chipbreaker_core::tool::profile::Profile;
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::hint::black_box;

/// Fixed, so successive runs measure the same work.
const BENCH_SEED: u64 = 0x0000_C41B_0000_0041;

/// The catalogue forms, and the highest polynomial degree each one costs.
fn tools() -> Vec<(&'static str, Profile, u32)> {
    let shank = Shank::plain(6.0, 50.0);
    vec![
        (
            "flat (quadratic)",
            flat_end_mill(6.0, 20.0, &shank).expect("valid"),
            2,
        ),
        (
            "ball (quadratic)",
            ball_end_mill(6.0, 20.0, &shank).expect("valid"),
            2,
        ),
        (
            "drill (quadratic)",
            drill(6.0, 118.0, 30.0, &shank).expect("valid"),
            2,
        ),
        (
            "bull (quartic)",
            bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
            4,
        ),
        (
            "barrel (quartic)",
            barrel_end_mill(12.0, 60.0, 40.0, &Shank::plain(12.0, 70.0)).expect("valid"),
            4,
        ),
    ]
}

/// One benchmark corpus: the same root sets expanded to each degree, so the
/// three timings differ only in the degree and not in the difficulty.
struct PolynomialCorpus {
    quadratics: Vec<[f64; 3]>,
    cubics: Vec<[f64; 4]>,
    quartics: Vec<[f64; 5]>,
}

/// Polynomials of each degree, built from known roots so that the repeated-root
/// path is exercised rather than avoided.
fn polynomial_corpus(count: usize) -> PolynomialCorpus {
    let mut rng = StdRng::seed_from_u64(BENCH_SEED);
    let mut quadratics = Vec::with_capacity(count);
    let mut cubics = Vec::with_capacity(count);
    let mut quartics = Vec::with_capacity(count);

    let expand = |roots: &[f64]| -> Vec<f64> {
        let mut c = vec![1.0];
        for &r in roots {
            let mut next = vec![0.0; c.len() + 1];
            for (i, &v) in c.iter().enumerate() {
                next[i] += v;
                next[i + 1] -= v * r;
            }
            c = next;
        }
        c
    };

    for i in 0..count {
        let mut pick = || f64::from(rng.random_range(-32i32..=32)) / 4.0;
        let (a, b, c, d) = (pick(), pick(), pick(), pick());
        // Every fourth polynomial has a repeated root, which is the slow path:
        // it leaves the closed form for the critical points of the derivative.
        let repeat = i % 4 == 0;

        let q = expand(&[a, if repeat { a } else { b }]);
        quadratics.push([q[0], q[1], q[2]]);
        let k = expand(&[a, if repeat { a } else { b }, c]);
        cubics.push([k[0], k[1], k[2], k[3]]);
        let p = expand(&[a, if repeat { a } else { b }, c, d]);
        quartics.push([p[0], p[1], p[2], p[3], p[4]]);
    }
    PolynomialCorpus {
        quadratics,
        cubics,
        quartics,
    }
}

fn bench_roots(c: &mut Criterion) {
    const COUNT: usize = 1_000;
    let corpus = polynomial_corpus(COUNT);
    let (quadratics, cubics, quartics) = (&corpus.quadratics, &corpus.cubics, &corpus.quartics);

    let mut group = c.benchmark_group("roots");
    group.throughput(Throughput::Elements(COUNT as u64));

    group.bench_function("quadratic", |bencher| {
        bencher.iter(|| {
            let mut total = 0usize;
            for p in quadratics {
                total += black_box(solve_quadratic(p[0], p[1], p[2])).len();
            }
            total
        });
    });
    group.bench_function("cubic", |bencher| {
        bencher.iter(|| {
            let mut total = 0usize;
            for p in cubics {
                total += black_box(solve_cubic(p[0], p[1], p[2], p[3])).len();
            }
            total
        });
    });
    // The one that pays for section 8's correctness decision.
    group.bench_function("quartic", |bencher| {
        bencher.iter(|| {
            let mut total = 0usize;
            for p in quartics {
                total += black_box(solve_quartic(p[0], p[1], p[2], p[3], p[4])).len();
            }
            total
        });
    });
    group.finish();
}

/// A bundle of coherent rays, which is what a field actually casts. Incoherent rays
/// would measure a workload this engine never has.
fn coherent_rays(profile: &Profile, n: usize) -> Vec<Ray> {
    let cylinder = profile.bounding_cylinder();
    let radius = cylinder.radius * 1.25 + 1.0;
    let mut rays = Vec::with_capacity(n * n);
    for i in 0..n {
        let y = -radius + 2.0 * radius * (i as f64 + 0.5) / n as f64;
        for j in 0..n {
            let z = cylinder.z_min - 0.5
                + (cylinder.z_max - cylinder.z_min + 1.0) * (j as f64 + 0.5) / n as f64;
            if let Some(ray) =
                Ray::new_normalized(Vec3::new(-radius - 1.0, y, z), Vec3::new(1.0, 0.0, 0.0))
            {
                rays.push(ray);
            }
        }
    }
    rays
}

fn bench_raycast(c: &mut Criterion) {
    const SIDE: usize = 48;
    let mut group = c.benchmark_group("tool/raycast");
    group.throughput(Throughput::Elements((SIDE * SIDE) as u64));

    for (name, profile, degree) in tools() {
        let rays = coherent_rays(&profile, SIDE);
        let mut scratch = RaycastScratch::with_capacity(profile.len());
        let mut spans = Spans::new();
        let mut stats = RaycastStats::default();
        group.bench_with_input(BenchmarkId::from_parameter(name), &degree, |bencher, _| {
            bencher.iter(|| {
                let mut measure = 0.0f64;
                for ray in &rays {
                    profile.intersect_ray_into(
                        black_box(ray),
                        &mut scratch,
                        &mut spans,
                        &mut stats,
                    );
                    measure += spans.measure();
                }
                measure
            });
        });
    }
    group.finish();
}

/// The allocation-free form against the convenient one.
///
/// `intersect_ray` allocates a `Spans` and a scratch buffer per call.
/// `intersect_ray_into` reuses the caller's. A field calls this millions of times
/// per simulation, so the difference is the whole argument for the scratch
/// parameter existing at all — and if it turns out not to matter, the API should
/// lose it.
fn bench_scratch_reuse(c: &mut Criterion) {
    const SIDE: usize = 32;
    let profile = bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid");
    let rays = coherent_rays(&profile, SIDE);

    let mut group = c.benchmark_group("tool/allocation");
    group.throughput(Throughput::Elements((SIDE * SIDE) as u64));

    group.bench_function("allocating", |bencher| {
        bencher.iter(|| {
            let mut measure = 0.0f64;
            for ray in &rays {
                measure += profile.intersect_ray(black_box(ray)).measure();
            }
            measure
        });
    });
    group.bench_function("reusing scratch", |bencher| {
        let mut scratch = RaycastScratch::with_capacity(profile.len());
        let mut spans = Spans::new();
        let mut stats = RaycastStats::default();
        bencher.iter(|| {
            let mut measure = 0.0f64;
            for ray in &rays {
                profile.intersect_ray_into(black_box(ray), &mut scratch, &mut spans, &mut stats);
                measure += spans.measure();
            }
            measure
        });
    });
    group.finish();
}

fn bench_properties(c: &mut Criterion) {
    let mut group = c.benchmark_group("tool/properties");
    for (name, profile, _) in tools() {
        group.bench_with_input(BenchmarkId::new("volume", name), &profile, |b, p| {
            b.iter(|| black_box(p.volume()));
        });
        group.bench_with_input(BenchmarkId::new("contains", name), &profile, |b, p| {
            b.iter(|| black_box(p.contains_rz(2.0, 10.0)));
        });
    }
    group.finish();
}

fn bench_tessellate(c: &mut Criterion) {
    let profile = bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid");
    let mut group = c.benchmark_group("tool/tessellate");
    for tolerance in [0.1f64, 0.01, 0.001] {
        group.bench_with_input(
            BenchmarkId::from_parameter(tolerance),
            &tolerance,
            |b, &t| {
                b.iter(|| black_box(profile.tessellate(t).expect("valid").0.triangle_count()));
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_roots,
    bench_raycast,
    bench_scratch_reuse,
    bench_properties,
    bench_tessellate
);
criterion_main!(benches);
