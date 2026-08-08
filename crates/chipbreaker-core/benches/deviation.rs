// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! What a comparison costs, and which of its three queries dominates.
//!
//! # Per sample, not per part
//!
//! `compare` visits every span endpoint of all three bundles, so its cost scales
//! with the **surface area** of the cut result rather than with the part's
//! volume or the program's length. A number per sample is therefore the one that
//! transfers between jobs, and the sample count is reported beside it so a part
//! can be sized from its own field.
//!
//! # Three queries, and they are not equal
//!
//! Each sample runs three things against the nominal's hierarchy:
//!
//! 1. **A closest-point query**, branch and bound over the BVH. The metric.
//! 2. **Two ray casts**, along `+normal` and `-normal`. The perpendicular
//!    diagnostic.
//! 3. **One more ray cast**, gathering every crossing, for the containment
//!    parity that decides the sign.
//!
//! They are timed apart because they scale differently and because a guess about
//! which dominates turned out to be wrong. The reasoning was that the parity
//! test is the one query that cannot stop early — a nearest-point search
//! abandons a subtree once its bound exceeds the running best, and a nearest-hit
//! cast stops at the first crossing, but parity has to find them all. It was the
//! cheapest of the three, and stayed so.
//!
//! What actually dominates is the **pair of casts for the perpendicular
//! diagnostic**, at 62% of a comparison: more than the metric it is a diagnostic
//! for. It is not being removed — it is what makes the step-edge artefact
//! visible rather than silent — but the number is what would decide whether a
//! `--no-perpendicular` flag is worth exposing. See BENCHMARKS.md.

use chipbreaker_core::deviation::{compare, facet_size};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

const STOCK: Vec3 = Vec3 {
    x: 40.0,
    y: 30.0,
    z: 12.0,
};

fn stock_mesh() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), STOCK)
}

/// A raster of passes, which is what a real facing or pocketing job looks like
/// and what makes the cut surface large enough to be worth timing.
fn cut_field(spacing: f64) -> TriDexelField {
    let mut field = TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let profile = flat_end_mill(6.0, 30.0, &Shank::plain(6.0, 60.0)).expect("valid");
    let mut scratch = CutScratch::new(&profile);
    let mut motions = Vec::new();
    let mut y = 6.0;
    let mut forward = true;
    while y <= 24.0 {
        let (a, b) = if forward { (5.0, 35.0) } else { (35.0, 5.0) };
        motions.push(Motion::Linear(LinearMove {
            start: Vec3::new(a, y, 8.0),
            end: Vec3::new(b, y, 8.0),
        }));
        y += 4.5;
        forward = !forward;
    }
    cut_all(
        &mut field,
        &profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: spacing / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// The nominal, extracted from a clean run of the same program.
fn nominal(field: &TriDexelField) -> TriMesh {
    use chipbreaker_core::contour::{ContourOptions, extract};
    extract(field, &ContourOptions::default())
        .expect("extracts")
        .0
}

/// End to end, per sample, against grid size.
fn comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("deviation/compare");
    group.sample_size(10);
    for spacing in [0.8, 0.5, 0.35] {
        let field = cut_field(spacing);
        let part = nominal(&field);
        let d = compare(&field, &part, Some(&stock_mesh()));
        eprintln!(
            "h={spacing}: {} samples against {} triangles, worst gouge {:.4} mm, \
             worst excess {:.4} mm, nominal facets {:.4} mm",
            d.samples.len(),
            part.triangle_count(),
            d.worst_gouge_mm,
            d.worst_excess_mm,
            facet_size(&part),
        );
        group.throughput(Throughput::Elements(d.samples.len() as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(spacing),
            &(&field, &part),
            |b, (f, p)| {
                b.iter(|| black_box(compare(f, p, None).worst_gouge_mm));
            },
        );
    }
    group.finish();
}

/// The three queries, one sample at a time, on the same hierarchy.
///
/// Deliberately not `compare` with parts switched off: this times the queries
/// themselves, so the numbers can be added up and checked against the end-to-end
/// figure above rather than trusted.
fn queries(c: &mut Criterion) {
    let field = cut_field(0.5);
    let part = nominal(&field);
    let bvh = Bvh::build(&part);

    // A spread of points over the cut surface, taken from the field itself so
    // they sit where real samples sit: on faces, in corners, and at the
    // grazing endpoints that make a hierarchy work hardest.
    use chipbreaker_core::dexel::tri::AXES;
    let mut points: Vec<(Vec3, Vec3)> = Vec::new();
    for axis in AXES {
        let Some(bundle) = field.bundle(axis) else {
            continue;
        };
        let lattice = bundle.lattice().clone();
        let direction = axis.direction();
        let rays = u32::try_from(bundle.arena().rays()).expect("small");
        for ray in (0..rays).step_by(37) {
            let (i, j) = lattice.coords(ray);
            let origin = lattice.origin_of(i, j);
            for span in bundle.arena().get(ray) {
                let at = Vec3::new(
                    origin.x + direction.x * span.t1,
                    origin.y + direction.y * span.t1,
                    origin.z + direction.z * span.t1,
                );
                points.push((at, span.n1.decode()));
            }
        }
    }
    eprintln!("queries: {} sample points", points.len());

    let mut group = c.benchmark_group("deviation/query");
    group.throughput(Throughput::Elements(points.len() as u64));

    group.bench_function("closest_point", |b| {
        b.iter(|| {
            let mut total = 0.0f64;
            for (at, _) in &points {
                if let Some((q, _)) = bvh.closest_point(&part, *at) {
                    total += q.x;
                }
            }
            black_box(total)
        });
    });

    group.bench_function("nearest_hit_both_ways", |b| {
        b.iter(|| {
            let mut total = 0.0f64;
            for (at, n) in &points {
                for direction in [*n, Vec3::new(-n.x, -n.y, -n.z)] {
                    if let Ok(Some(h)) = bvh.intersect_ray(
                        &part,
                        &Ray {
                            origin: *at,
                            direction,
                        },
                    ) {
                        total += h.t;
                    }
                }
            }
            black_box(total)
        });
    });

    group.bench_function("all_crossings_for_parity", |b| {
        let mut hits = Vec::new();
        b.iter(|| {
            let mut total = 0usize;
            for (at, n) in &points {
                let query = Ray {
                    origin: *at,
                    direction: *n,
                };
                if bvh.intersect_ray_all_into(&part, &query, &mut hits).is_ok() {
                    total += hits.len();
                }
            }
            black_box(total)
        });
    });
    group.finish();
}

/// What the tessellation floor costs, which is once per mesh rather than per
/// sample — and is therefore expected to be invisible.
///
/// Timed anyway, because it walks every edge of the nominal and builds a map to
/// do it, and "expected to be invisible" is the kind of claim that turns out to
/// be a third of the runtime.
fn floor(c: &mut Criterion) {
    let field = cut_field(0.5);
    let part = nominal(&field);
    let mut group = c.benchmark_group("deviation/facet_size");
    group.throughput(Throughput::Elements(u64::from(part.triangle_count())));
    group.bench_function("nominal", |b| {
        b.iter(|| black_box(facet_size(&part)));
    });
    group.finish();
}

criterion_group!(benches, comparison, queries, floor);
criterion_main!(benches);
