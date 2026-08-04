// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Cutting throughput, and the rejection rate that decides it.
//!
//! # The rejection rate dominates everything else
//!
//! A finishing segment touches a vanishing fraction of a four-million-ray field.
//! What decides whether a 500,000-segment job takes minutes or days is not the
//! inner loop but how cheaply the other 99.99% of rays are skipped. So the
//! headline number here is not segments per second, it is the fraction of rays
//! never examined — and the two are the same measurement seen from either end.
//!
//! # Why each case is timed separately
//!
//! Case A is exact and needs three ray casts. Case B is exact and needs one, or
//! none at all for a ray along the plunge. Case C sub-steps, so its cost is set
//! by the tolerance rather than by the geometry. Averaging them would produce a
//! number that describes no real job, because the mix differs completely between
//! a drilling program and a finishing pass.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::spans::Spans;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri};
use chipbreaker_core::sweep::{LinearMove, horizontal, plunge, reference};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

fn mill() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

/// Stock at a given cell size. 100 x 60 x 20 mm, a plausible plate.
fn stock(spacing: f64) -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 60.0, 20.0)),
        &TriBuildOptions {
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn horizontal_move() -> LinearMove {
    LinearMove {
        start: Vec3::new(5.0, 30.0, 14.0),
        end: Vec3::new(95.0, 30.0, 14.0),
    }
}

fn plunge_move() -> LinearMove {
    LinearMove {
        start: Vec3::new(50.0, 30.0, 22.0),
        end: Vec3::new(50.0, 30.0, 12.0),
    }
}

fn ramp_move() -> LinearMove {
    LinearMove {
        start: Vec3::new(10.0, 20.0, 20.0),
        end: Vec3::new(80.0, 45.0, 13.0),
    }
}

/// Whole cuts, per case, at three resolutions.
fn cut_by_case(c: &mut Criterion) {
    let profile = mill();
    let mut group = c.benchmark_group("sweep/cut");
    for spacing in [1.0, 0.5, 0.25] {
        let field = stock(spacing);
        let rays = field.rays() as u64;
        for (name, motion) in [
            ("A-horizontal", horizontal_move()),
            ("B-plunge", plunge_move()),
            ("C-ramp", ramp_move()),
        ] {
            group.throughput(Throughput::Elements(rays));
            group.bench_with_input(
                BenchmarkId::new(name, format!("{spacing}mm/{rays}rays")),
                &motion,
                |b, motion| {
                    b.iter_batched(
                        || (stock(spacing), CutScratch::new(&profile)),
                        |(mut field, mut scratch)| {
                            black_box(cut_tri(
                                &mut field,
                                &profile,
                                motion,
                                SweepMethod::Analytic {
                                    tolerance: spacing / 10.0,
                                },
                                &mut scratch,
                            ))
                        },
                        criterion::BatchSize::LargeInput,
                    );
                },
            );
        }
    }
    group.finish();
}

/// The exact path against the reference, on the same motion.
///
/// The number that says what the closed forms bought.
fn analytic_against_reference(c: &mut Criterion) {
    let profile = mill();
    let spacing = 0.5;
    let motion = horizontal_move();
    let field = stock(spacing);
    let rays = field.rays() as u64;

    let mut group = c.benchmark_group("sweep/method");
    group.throughput(Throughput::Elements(rays));
    for (name, method) in [
        (
            "analytic",
            SweepMethod::Analytic {
                tolerance: spacing / 10.0,
            },
        ),
        ("reference-64", SweepMethod::Reference { steps: 64 }),
        ("reference-512", SweepMethod::Reference { steps: 512 }),
    ] {
        group.bench_function(name, |b| {
            b.iter_batched(
                || (stock(spacing), CutScratch::new(&profile)),
                |(mut field, mut scratch)| {
                    black_box(cut_tri(&mut field, &profile, &motion, method, &mut scratch))
                },
                criterion::BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

/// Swept spans for one ray, with no field around them.
///
/// Isolates the geometry from the traversal, so a regression in Case A's prism
/// is not hidden by the rejection test doing its job.
fn spans_per_ray(c: &mut Criterion) {
    let profile = mill();
    let mut scratch = RaycastScratch::default();
    let mut stats = RaycastStats::default();
    let mut out = Spans::new();
    // Computed once, as the cut path does: it is a property of the profile,
    // and computing it per ray cost 180x Case A's whole span computation.
    let convex = plunge::is_radially_convex(&profile);
    let ray = Ray {
        origin: Vec3::new(50.0, 30.0, -10.0),
        direction: Vec3::new(0.0, 0.0, 1.0),
    };

    let mut group = c.benchmark_group("sweep/spans");
    group.throughput(Throughput::Elements(1));
    let motion = horizontal_move();
    group.bench_function("A-horizontal", |b| {
        b.iter(|| {
            horizontal::swept_spans_into(
                &profile,
                &motion,
                black_box(&ray),
                &mut scratch,
                &mut out,
                &mut stats,
            );
            black_box(out.len())
        });
    });
    let motion = plunge_move();
    group.bench_function("B-plunge", |b| {
        b.iter(|| {
            black_box(plunge::swept_spans_into(
                &profile,
                &motion,
                black_box(&ray),
                convex,
                &mut scratch,
                &mut out,
                &mut stats,
            ))
        });
    });
    for steps in [16u32, 64, 256] {
        let motion = ramp_move();
        group.bench_with_input(
            BenchmarkId::new("C-ramp-substeps", steps),
            &steps,
            |b, &steps| {
                b.iter(|| {
                    reference::swept_spans_into(
                        &profile,
                        &motion,
                        steps,
                        black_box(&ray),
                        &mut scratch,
                        &mut out,
                        &mut stats,
                    );
                    black_box(out.len())
                });
            },
        );
    }
    group.finish();
}

/// A many-segment job, which is what a customer actually runs.
///
/// A raster finishing pass: parallel horizontal passes with a plunge between
/// each, which is the mix real finishing work has and the corpus does not.
fn raster_job(c: &mut Criterion) {
    let profile = mill();
    let spacing = 0.5;

    let mut moves = Vec::new();
    let mut y = 5.0;
    let mut forward = true;
    while y < 55.0 {
        let (a, b) = if forward { (5.0, 95.0) } else { (95.0, 5.0) };
        moves.push(LinearMove {
            start: Vec3::new(a, y, 16.0),
            end: Vec3::new(b, y, 16.0),
        });
        moves.push(LinearMove {
            start: Vec3::new(b, y, 16.0),
            end: Vec3::new(b, y + 3.0, 16.0),
        });
        y += 3.0;
        forward = !forward;
    }

    let mut group = c.benchmark_group("sweep/job");
    group.throughput(Throughput::Elements(moves.len() as u64));
    group.bench_function("raster-finishing", |b| {
        b.iter_batched(
            || (stock(spacing), CutScratch::new(&profile)),
            |(mut field, mut scratch)| {
                for motion in &moves {
                    cut_tri(
                        &mut field,
                        &profile,
                        motion,
                        SweepMethod::Analytic {
                            tolerance: spacing / 10.0,
                        },
                        &mut scratch,
                    );
                }
                black_box(field.volume())
            },
            criterion::BatchSize::LargeInput,
        );
    });
    group.finish();
}

criterion_group!(
    benches,
    cut_by_case,
    analytic_against_reference,
    spans_per_ray,
    raster_job
);
criterion_main!(benches);
