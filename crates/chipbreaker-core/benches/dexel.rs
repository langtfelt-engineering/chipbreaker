// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Dexel field construction and storage.
//!
//! Three questions, each of which someone has already had to guess at once.
//!
//! **Does the arena actually beat `Vec<Spans>`?** ADR 0001 Part 1 argued it
//! would, from structure rather than measurement, and left the benchmark as an
//! obligation. This discharges it. The argument was 24 bytes of header
//! per ray plus one allocation each, against two allocations for the whole
//! field.
//!
//! **What does building cost per ray?** The number that says whether a 2000x2000
//! field is a coffee break or an overnight job, and the budget every later unit
//! spends against.
//!
//! **How much does the half-cell offset buy?** The mesh benchmarks measured 15.8x on a
//! synthetic sweep. Measured again here on the real construction path, because a
//! number that load-bearing should not rest on one measurement in one place.

use chipbreaker_core::dexel::{Arena, BuildOptions, DexelField, io as dexel_io};
use chipbreaker_core::math::{Aabb3, Axis, Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::spans::{Span, Spans};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

/// Cell sizes to build at, in millimetres.
const SPACINGS: [f64; 3] = [1.0, 0.5, 0.25];

fn stock() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 40.0, 20.0))
}

fn part() -> TriMesh {
    shapes::torus(20.0, 6.0, 128, 64)
}

/// Building a field: the cost every later unit budgets against.
fn build(c: &mut Criterion) {
    let mut group = c.benchmark_group("dexel/build");
    for mesh_name in ["box", "torus"] {
        let mesh = if mesh_name == "box" { stock() } else { part() };
        for spacing in SPACINGS {
            let options = BuildOptions {
                spacing_xyz: None,
                spacing,
                ..BuildOptions::default()
            };
            let (field, _) = DexelField::build(&mesh, &options).expect("builds");
            let rays = field.lattice().ray_count() as u64;
            group.throughput(Throughput::Elements(rays));
            group.bench_with_input(
                BenchmarkId::new(mesh_name, format!("{spacing}mm/{rays}rays")),
                &options,
                |b, options| {
                    b.iter(|| black_box(DexelField::build(black_box(&mesh), options).is_ok()));
                },
            );
        }
    }
    group.finish();
}

/// The arena against the `Vec<Spans>` it replaced.
///
/// ADR 0001 Part 1's outstanding obligation. Both sides do the same work: fill
/// every ray with one span, then read every ray back. The difference is entirely
/// in where the spans live.
fn arena_versus_per_ray_vec(c: &mut Criterion) {
    let mut group = c.benchmark_group("dexel/storage");
    for rays in [10_000usize, 100_000, 1_000_000] {
        group.throughput(Throughput::Elements(rays as u64));

        group.bench_with_input(BenchmarkId::new("arena/fill", rays), &rays, |b, &rays| {
            b.iter(|| {
                let mut arena = Arena::new(rays);
                for ray in 0..rays as u32 {
                    arena.set(ray, &[Span::new(0.0, 1.0)]);
                }
                black_box(arena.total_spans())
            });
        });

        group.bench_with_input(BenchmarkId::new("vec/fill", rays), &rays, |b, &rays| {
            b.iter(|| {
                let mut store: Vec<Spans> = Vec::with_capacity(rays);
                for _ in 0..rays {
                    store.push(Spans::from_span(Span::new(0.0, 1.0)));
                }
                black_box(store.iter().map(Spans::len).sum::<usize>())
            });
        });

        // Reading is the operation cutting performs millions of times, and it is where
        // locality shows up: the arena's spans are contiguous, the Vec's are
        // scattered across as many allocations as there are rays.
        let mut arena = Arena::new(rays);
        for ray in 0..rays as u32 {
            arena.set(ray, &[Span::new(0.0, 1.0)]);
        }
        let store: Vec<Spans> = (0..rays)
            .map(|_| Spans::from_span(Span::new(0.0, 1.0)))
            .collect();

        group.bench_with_input(BenchmarkId::new("arena/scan", rays), &rays, |b, &rays| {
            b.iter(|| {
                let mut total = 0.0;
                for ray in 0..rays as u32 {
                    for span in arena.get(ray) {
                        total += span.length();
                    }
                }
                black_box(total)
            });
        });

        group.bench_with_input(BenchmarkId::new("vec/scan", rays), &rays, |b, _| {
            b.iter(|| {
                let mut total = 0.0;
                for spans in &store {
                    for span in spans.iter() {
                        total += span.length();
                    }
                }
                black_box(total)
            });
        });
    }
    group.finish();
}

/// The half-cell offset, measured on the construction path.
///
/// ADR 0001 Part 2 rests on 2.52 ms against 39.83 ms from a synthetic sweep at
/// the mesh benchmarks. A number that decides a required invariant deserves a second
/// measurement somewhere else, so here it is against the same lattice block, cast
/// the way construction casts.
fn cell_centres_versus_corners(c: &mut Criterion) {
    let mesh = shapes::lattice_block(9);
    let bvh = Bvh::build(&mesh);
    let bounds = mesh.bounds();
    let spacing = 1.0;
    let side = 64u32;

    let mut group = c.benchmark_group("dexel/ray-origins");
    group.throughput(Throughput::Elements(u64::from(side) * u64::from(side)));

    for (name, offset) in [("cell-centres", 0.5), ("integer-lattice", 0.0)] {
        group.bench_function(name, |b| {
            b.iter(|| {
                let mut hits = Vec::new();
                let mut crossings = 0usize;
                for i in 0..side {
                    for j in 0..side {
                        let ray = Ray {
                            origin: Vec3::new(
                                bounds.min.x + (f64::from(i) + offset) * spacing,
                                bounds.min.y + (f64::from(j) + offset) * spacing,
                                bounds.min.z - spacing,
                            ),
                            direction: Vec3::new(0.0, 0.0, 1.0),
                        };
                        if bvh.intersect_ray_all_into(&mesh, &ray, &mut hits).is_ok() {
                            crossings += hits.len();
                        }
                    }
                }
                black_box(crossings)
            });
        });
    }
    group.finish();
}

/// Writing and reading `.dexel`.
///
/// The format is raw IEEE bits precisely so that this is a memory copy rather
/// than millions of decimal conversions (ADR 0004). Measured so the claim is a
/// number rather than an expectation.
fn serialization(c: &mut Criterion) {
    let mesh = part();
    let (field, _) = DexelField::build(
        &mesh,
        &BuildOptions {
            spacing_xyz: None,
            spacing: 0.25,
            ..BuildOptions::default()
        },
    )
    .expect("builds");
    let bytes = dexel_io::to_bytes(&field).expect("writes");

    let mut group = c.benchmark_group("dexel/io");
    group.throughput(Throughput::Bytes(bytes.len() as u64));
    group.bench_function("write", |b| {
        b.iter(|| black_box(dexel_io::to_bytes(black_box(&field)).map(|v| v.len())));
    });
    group.bench_function("read", |b| {
        b.iter(|| black_box(dexel_io::from_bytes(black_box(&bytes)).is_ok()));
    });
    group.finish();
}

/// Measuring a field's volume, which is the traversal cutting and extraction repeat.
fn volume(c: &mut Criterion) {
    let mut group = c.benchmark_group("dexel/volume");
    for spacing in SPACINGS {
        let (field, _) = DexelField::build(
            &part(),
            &BuildOptions {
                spacing_xyz: None,
                spacing,
                ..BuildOptions::default()
            },
        )
        .expect("builds");
        let rays = field.lattice().ray_count() as u64;
        group.throughput(Throughput::Elements(rays));
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{spacing}mm/{rays}rays")),
            &field,
            |b, field| b.iter(|| black_box(field.volume())),
        );
    }
    group.finish();
}

/// A sanity check that the lattice itself costs nothing worth measuring.
///
/// Included because `origin_of` runs once per ray on the innermost loop, and a
/// future "improvement" that made it allocate would be invisible in the build
/// benchmark's noise.
fn lattice(c: &mut Criterion) {
    use chipbreaker_core::dexel::Lattice;
    let lattice = Lattice::new(
        Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 25.0)),
        0.1,
        Axis::Z,
    )
    .expect("valid");
    let rays = lattice.ray_count() as u64;
    let mut group = c.benchmark_group("dexel/lattice");
    group.throughput(Throughput::Elements(rays));
    group.bench_function("origin_of", |b| {
        b.iter(|| {
            let mut acc = 0.0;
            for ray in 0..rays as u32 {
                let (i, j) = lattice.coords(ray);
                acc += lattice.origin_of(i, j).x;
            }
            black_box(acc)
        });
    });
    group.finish();
}

criterion_group!(
    benches,
    build,
    arena_versus_per_ray_vec,
    cell_centres_versus_corners,
    serialization,
    volume,
    lattice,
    tri,
    tessellation
);
criterion_main!(benches);

/// Three bundles: build, serialize, and measure deviation.
///
/// The numbers cutting budgets against. Note that three bundles is **not** three
/// times one: they cover `(WD + DH + HW) / h^2` rays between them, so the cost
/// tracks half the bounding-box surface area rather than any single face.
fn tri(c: &mut Criterion) {
    use chipbreaker_core::dexel::deviation::{measure, sample_mesh_budget};
    use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};

    let mesh = part();
    let mut group = c.benchmark_group("tridexel");

    for spacing in [1.0, 0.5, 0.25] {
        let options = TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        };
        let (field, stats) = TriDexelField::build(&mesh, &options).expect("builds");
        group.throughput(Throughput::Elements(stats.rays));
        group.bench_with_input(
            BenchmarkId::new("build", format!("{spacing}mm/{}rays", stats.rays)),
            &options,
            |b, options| {
                b.iter(|| black_box(TriDexelField::build(black_box(&mesh), options).is_ok()));
            },
        );

        let bytes = dexel_io::tri_to_bytes(&field).expect("writes");
        group.throughput(Throughput::Bytes(bytes.len() as u64));
        group.bench_with_input(
            BenchmarkId::new("write", format!("{spacing}mm")),
            &field,
            |b, field| b.iter(|| black_box(dexel_io::tri_to_bytes(field).map(|v| v.len()))),
        );
        group.bench_with_input(
            BenchmarkId::new("read", format!("{spacing}mm")),
            &bytes,
            |b, bytes| b.iter(|| black_box(dexel_io::tri_from_bytes(bytes).is_ok())),
        );
    }

    // Deviation, which is the accuracy metric and therefore the measurement a
    // customer will actually run. Its cost is what decides whether it can be a
    // default or has to be opt-in.
    let (field, _) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: 0.5,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds");
    let (samples, _) = sample_mesh_budget(&mesh, 20_000);
    group.throughput(Throughput::Elements(samples.len() as u64));
    group.bench_function("deviation/20k-samples", |b| {
        b.iter(|| black_box(measure(black_box(&field), black_box(&samples)).best_max));
    });
    group.finish();
}

/// The tessellation estimate, which `dexel build` runs on every invocation.
fn tessellation(c: &mut Criterion) {
    use chipbreaker_core::dexel::tessellation;
    let mut group = c.benchmark_group("tridexel/tessellation");
    for (name, mesh) in [
        ("sphere-4", shapes::icosphere(10.0, 4)),
        ("sphere-5", shapes::icosphere(10.0, 5)),
    ] {
        group.throughput(Throughput::Elements(u64::from(mesh.triangle_count())));
        group.bench_function(name, |b| {
            b.iter(|| black_box(tessellation::estimate(black_box(&mesh)).percentile_sagitta_mm));
        });
    }
    group.finish();
}
