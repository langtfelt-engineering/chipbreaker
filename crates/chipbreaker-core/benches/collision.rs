// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! What collision checking costs on top of cutting.
//!
//! # The number that decides whether anybody turns it on
//!
//! Collision checking is optional, and an optional check that doubles a
//! simulation's runtime gets switched off — after which it finds nothing at all.
//! So the figure worth measuring is not the absolute cost but the **overhead
//! against cutting alone**, which is what a user actually trades away.
//!
//! The comparison is deliberately like-for-like: the same field, the same
//! program, the same tool, differing only in whether the non-cutting geometry is
//! tested before each move. Both arms cut, because the checking arm has to cut
//! too — its whole design is to interleave.
//!
//! # Why fixtures scale separately
//!
//! A fixture is another field to test against, so the cost is linear in their
//! number, and it is worth confirming that rather than assuming it: a
//! shop with a tombstone and eight clamps is the case where this either stays
//! usable or does not.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{HolderStage, Shank, flat_end_mill};
use chipbreaker_core::toolpath::{MotionKind, Provenance};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

const SPACING: f64 = 0.6;
const STOCK: Vec3 = Vec3 {
    x: 80.0,
    y: 50.0,
    z: 30.0,
};

fn stock_mesh() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), STOCK)
}

fn field() -> TriDexelField {
    TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// A stub cutter under an ER32 chuck, so the non-cutting geometry is real and
/// large enough that testing it is not free.
fn tool() -> Profile {
    flat_end_mill(
        6.0,
        12.0,
        &Shank::with_holder(
            6.0,
            26.0,
            [
                HolderStage::cylinder(50.8, 28.0),
                HolderStage::cylinder(61.912_499_999_999_994, 50.0),
            ],
        ),
    )
    .expect("valid")
}

/// A raster of passes, which is what a facing job looks like.
fn raster(z: f64) -> Vec<Motion> {
    let mut out = Vec::new();
    let mut y = 8.0;
    let mut forward = true;
    while y <= 42.0 {
        let (a, b) = if forward { (6.0, 74.0) } else { (74.0, 6.0) };
        out.push(Motion::Linear(LinearMove {
            start: Vec3::new(a, y, z),
            end: Vec3::new(b, y, z),
        }));
        y += 4.0;
        forward = !forward;
    }
    out
}

fn method() -> SweepMethod {
    SweepMethod::Analytic {
        tolerance: SPACING / 10.0,
    }
}

fn params(clearance_mm: f64) -> CollideParams {
    CollideParams {
        clearance_mm,
        grid_mm: 2.0 * SPACING,
        method: method(),
    }
}

fn clamps(n: usize) -> Vec<(String, TriDexelField)> {
    (0..n)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "a handful of fixtures")]
            let x = 84.0 + (i as f64) * 20.0;
            let mesh = shapes::box_solid(Vec3::new(x, 18.0, 0.0), Vec3::new(x + 14.0, 32.0, 40.0));
            let f = TriDexelField::build(
                &mesh,
                &TriBuildOptions {
                    spacing: SPACING,
                    ..TriBuildOptions::default()
                },
            )
            .expect("builds")
            .0;
            (format!("clamp-{i}"), f)
        })
        .collect()
}

/// Cutting alone against cutting with the collision check interleaved.
fn overhead(c: &mut Criterion) {
    let profile = tool();
    let motions = raster(10.0);
    let kinds = vec![MotionKind::Linear; motions.len()];
    let provenance: Vec<Provenance> = (0..motions.len())
        .map(|i| Provenance::new(0, u32::try_from(i).unwrap_or(0), 0))
        .collect();

    let mut group = c.benchmark_group("collision/overhead");
    group.sample_size(10);
    group.throughput(Throughput::Elements(motions.len() as u64));

    group.bench_function("cut_only", |b| {
        b.iter(|| {
            let mut f = field();
            let mut scratch = CutScratch::new(&profile);
            cut_all(
                &mut f,
                &profile,
                &motions,
                method(),
                &mut scratch,
                DEFAULT_BATCH,
            );
            black_box(f.total_spans())
        });
    });

    group.bench_function("cut_and_check", |b| {
        b.iter(|| {
            let mut f = field();
            let mut scratch = CutScratch::new(&profile);
            let found = collide_with_stock(
                &mut f,
                &profile,
                &motions,
                &kinds,
                &provenance,
                0,
                &[],
                &params(0.0),
                &mut scratch,
            )
            .expect("the tool has a chuck");
            black_box(found.len())
        });
    });
    group.finish();
}

/// Clearance reporting on top of collision checking.
fn clearance(c: &mut Criterion) {
    let profile = tool();
    let motions = raster(10.0);
    let kinds = vec![MotionKind::Linear; motions.len()];
    let provenance: Vec<Provenance> = (0..motions.len())
        .map(|i| Provenance::new(0, u32::try_from(i).unwrap_or(0), 0))
        .collect();
    let fixtures = clamps(1);

    let mut group = c.benchmark_group("collision/clearance");
    group.sample_size(10);
    for mm in [0.0, 2.0] {
        group.bench_with_input(BenchmarkId::from_parameter(mm), &mm, |b, &mm| {
            b.iter(|| {
                let mut f = field();
                let mut scratch = CutScratch::new(&profile);
                let found = collide_with_stock(
                    &mut f,
                    &profile,
                    &motions,
                    &kinds,
                    &provenance,
                    0,
                    &fixtures,
                    &params(mm),
                    &mut scratch,
                )
                .expect("the tool has a chuck");
                black_box(found.len())
            });
        });
    }
    group.finish();
}

/// How the cost grows with the number of fixtures.
fn fixture_count(c: &mut Criterion) {
    let profile = tool();
    let motions = raster(10.0);
    let kinds = vec![MotionKind::Linear; motions.len()];
    let provenance: Vec<Provenance> = (0..motions.len())
        .map(|i| Provenance::new(0, u32::try_from(i).unwrap_or(0), 0))
        .collect();

    let mut group = c.benchmark_group("collision/fixtures");
    group.sample_size(10);
    for n in [0usize, 1, 4, 8] {
        let fixtures = clamps(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &fixtures, |b, fx| {
            b.iter(|| {
                let mut f = field();
                let mut scratch = CutScratch::new(&profile);
                let found = collide_with_stock(
                    &mut f,
                    &profile,
                    &motions,
                    &kinds,
                    &provenance,
                    0,
                    fx,
                    &params(0.0),
                    &mut scratch,
                )
                .expect("the tool has a chuck");
                black_box(found.len())
            });
        });
    }
    group.finish();
}

criterion_group!(benches, overhead, clearance, fixture_count);
criterion_main!(benches);
