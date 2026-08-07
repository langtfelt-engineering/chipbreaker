// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Extraction throughput, and what dominates it.
//!
//! # Triangles per second is the headline, but not the constraint
//!
//! Extraction visits every grid corner once whether or not there is a surface
//! near it, so its cost is set by the field's *volume* while its output is set
//! by the surface's *area*. On a large field most of the work classifies corners
//! that produce nothing, which is why the corner classification and the QEF are
//! timed apart: they scale with different things and a single throughput number
//! hides that.

use chipbreaker_core::contour::qef::Qef;
use chipbreaker_core::contour::{ContourOptions, extract};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

fn field(spacing: f64) -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 20.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// Throughput against grid size, in triangles produced per second.
fn extraction(c: &mut Criterion) {
    let mut group = c.benchmark_group("contour/extract");
    group.sample_size(20);
    for spacing in [1.0, 0.5, 0.25] {
        let f = field(spacing);
        let (mesh, stats) = extract(&f, &ContourOptions::default()).expect("extracts");
        let triangles = mesh.triangle_count() as u64;
        eprintln!(
            "h={spacing}: {} corners, {} triangles, {} cells with vertices, \
             {} split, {} disagreements",
            stats.corners,
            triangles,
            stats.cells_with_vertices,
            stats.cells_with_multiple_vertices,
            stats.corner_disagreements
        );
        group.throughput(Throughput::Elements(triangles));
        group.bench_with_input(BenchmarkId::from_parameter(spacing), &f, |b, f| {
            b.iter(|| black_box(extract(f, &ContourOptions::default()).expect("extracts").1));
        });
    }
    group.finish();
}

/// With and without normals: what the QEF costs over the centroid.
fn normals_cost(c: &mut Criterion) {
    let f = field(0.4);
    let mut group = c.benchmark_group("contour/normals");
    group.sample_size(20);
    for use_normals in [true, false] {
        group.bench_function(if use_normals { "qef" } else { "centroid" }, |b| {
            b.iter(|| {
                black_box(
                    extract(
                        &f,
                        &ContourOptions {
                            use_normals,
                            ..ContourOptions::default()
                        },
                    )
                    .expect("extracts")
                    .1,
                )
            });
        });
    }
    group.finish();
}

/// The solver alone, per system, at each rank.
///
/// Timed apart from extraction because it is the only part whose cost depends on
/// the geometry rather than on the grid: a flat, an edge and a corner run the
/// same eigensolver but take different branches out of it.
fn qef_solves(c: &mut Criterion) {
    let flat = {
        let mut q = Qef::new();
        for p in [
            Vec3::new(0.0, 0.0, 4.0),
            Vec3::new(1.0, 0.0, 4.0),
            Vec3::new(0.0, 1.0, 4.0),
        ] {
            q.add(p, Vec3::new(0.0, 0.0, 1.0));
        }
        q
    };
    let edge = {
        let mut q = Qef::new();
        q.add(Vec3::new(1.0, 0.0, 5.0), Vec3::new(1.0, 0.0, 0.0));
        q.add(Vec3::new(0.0, 2.0, 7.0), Vec3::new(0.0, 1.0, 0.0));
        q
    };
    let corner = {
        let mut q = Qef::new();
        q.add(Vec3::new(1.0, 9.0, 9.0), Vec3::new(1.0, 0.0, 0.0));
        q.add(Vec3::new(9.0, 2.0, 9.0), Vec3::new(0.0, 1.0, 0.0));
        q.add(Vec3::new(9.0, 9.0, 3.0), Vec3::new(0.0, 0.0, 1.0));
        q
    };

    let mut group = c.benchmark_group("contour/qef");
    group.throughput(Throughput::Elements(1));
    for (name, system) in [("flat", &flat), ("edge", &edge), ("corner", &corner)] {
        group.bench_function(name, |b| b.iter(|| black_box(system.solve())));
    }
    group.finish();
}

criterion_group!(benches, extraction, normals_cost, qef_solves);
criterion_main!(benches);
