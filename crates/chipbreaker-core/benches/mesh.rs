// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

// `criterion_group!` expands to a public function it does not document, and the
// workspace denies missing docs. Scoped to the benches, where the lint buys us
// nothing anyway.
#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! Mesh pipeline throughput: parsing, welding, validation, BVH build and ray
//! queries.
//!
//! The two numbers that matter most for U5 are at the bottom:
//!
//! - **coherent versus incoherent ray throughput.** U5 casts millions of
//!   parallel rays on a lattice, not random ones, so the coherent figure is the
//!   one that sets its budget. The ratio also says how much the BVH's locality
//!   is worth.
//! - **generic versus lattice-aligned ray cost.** Unit 1 measured `orient3d` at
//!   roughly 17x the filtered path; the parity suite measures the exact-fallback
//!   *rate* at 3.9% generic against 65.8% lattice-aligned. This benchmark turns
//!   those two facts into a single wall-clock ratio, which is what a scheduling
//!   decision actually needs.

use chipbreaker_core::eps::EPS_WELD;
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::io::{obj, stl};
use chipbreaker_core::mesh::units::Unit;
use chipbreaker_core::mesh::validate::{check_self_intersections, validate};
use chipbreaker_core::mesh::weld::weld;
use chipbreaker_core::mesh::{TriMesh, shapes};
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::hint::black_box;

/// Fixed, so successive runs measure the same work.
const BENCH_SEED: u64 = 0x0000_C41B_0000_0040;

/// Lattice block sizes giving roughly 10k, 100k and 1M triangles: a block of
/// side `n` has `12 n^2` triangles.
const SIZES: [(u32, &str); 3] = [(29, "10k"), (91, "100k"), (289, "1M")];

/// A soup of loose triangles, as STL delivers them: the input welding must cope
/// with.
fn soup(mesh: &TriMesh) -> TriMesh {
    let mut v = Vec::with_capacity(mesh.triangle_count() as usize * 3);
    let mut t = Vec::with_capacity(mesh.triangle_count() as usize);
    for i in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(i);
        let base = v.len() as u32;
        v.extend_from_slice(&[a, b, c]);
        t.push([base, base + 1, base + 2]);
    }
    TriMesh::new(v, t, mesh.meta().clone()).expect("valid soup")
}

fn bench_parse(c: &mut Criterion) {
    let mesh = shapes::icosphere(10.0, 5); // 20,480 triangles
    let binary = stl::write_binary(&mesh);
    let ascii = stl::write_ascii(&mesh, "bench");
    let wavefront = obj::write(&mesh);

    let mut group = c.benchmark_group("mesh/parse");
    for (name, bytes) in [
        ("stl-binary", binary.len()),
        ("stl-ascii", ascii.len()),
        ("obj", wavefront.len()),
    ] {
        group.throughput(Throughput::Bytes(bytes as u64));
        match name {
            "stl-binary" => {
                group.bench_function(name, |b| {
                    b.iter(|| black_box(stl::read_binary(black_box(&binary), Unit::Millimetre)));
                });
            }
            "stl-ascii" => {
                group.bench_function(name, |b| {
                    b.iter(|| black_box(stl::read_ascii(black_box(&ascii), Unit::Millimetre)));
                });
            }
            _ => {
                group.bench_function(name, |b| {
                    b.iter(|| black_box(obj::read(black_box(&wavefront), Unit::Millimetre)));
                });
            }
        }
    }
    group.finish();
}

fn bench_weld(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh/weld");
    // 1M triangles takes seconds per iteration; the default 100 samples would
    // run for an hour.
    group.sample_size(10);
    for (n, label) in SIZES {
        let mesh = soup(&shapes::lattice_block(n));
        group.throughput(Throughput::Elements(u64::from(mesh.triangle_count())));
        group.bench_function(BenchmarkId::from_parameter(label), |b| {
            b.iter(|| black_box(weld(black_box(&mesh), EPS_WELD)));
        });
    }
    group.finish();
}

fn bench_validate(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh/validate");
    group.sample_size(10);
    for (n, label) in SIZES {
        let mesh = shapes::lattice_block(n);
        group.throughput(Throughput::Elements(u64::from(mesh.triangle_count())));
        group.bench_function(BenchmarkId::new("topology", label), |b| {
            b.iter(|| black_box(validate(black_box(&mesh))));
        });
    }
    // Self-intersection is opt-in precisely because of this ratio; only the
    // smallest size is run, because the others take minutes.
    let mesh = shapes::lattice_block(SIZES[0].0);
    group.throughput(Throughput::Elements(u64::from(mesh.triangle_count())));
    group.bench_function(BenchmarkId::new("with-self-intersect", SIZES[0].1), |b| {
        b.iter(|| {
            let mut report = validate(black_box(&mesh));
            check_self_intersections(black_box(&mesh), &mut report);
            black_box(report.triangles)
        });
    });
    group.finish();
}

fn bench_bvh_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh/bvh-build");
    group.sample_size(10);
    for (n, label) in SIZES {
        let mesh = shapes::lattice_block(n);
        group.throughput(Throughput::Elements(u64::from(mesh.triangle_count())));
        group.bench_function(BenchmarkId::from_parameter(label), |b| {
            b.iter(|| black_box(Bvh::build(black_box(&mesh))));
        });
    }
    group.finish();
}

/// A batch of parallel rays on a lattice — U5's actual access pattern.
fn coherent_rays(mesh: &TriMesh, count: u32) -> Vec<Ray> {
    let b = mesh.bounds();
    let e = b.extent();
    let side = f64::from(count);
    (0..count)
        .flat_map(|i| {
            (0..count).map(move |j| {
                Ray::new(
                    Vec3::new(
                        b.min.x + e.x * (f64::from(i) + 0.5) / side,
                        b.min.y + e.y * (f64::from(j) + 0.5) / side,
                        b.min.z - e.z - 1.0,
                    ),
                    Vec3::Z,
                )
            })
        })
        .collect()
}

/// The same rays, but snapped to the integer lattice so they strike vertices and
/// edges head on and force the exact fallback.
fn aligned_rays(mesh: &TriMesh, count: u32) -> Vec<Ray> {
    coherent_rays(mesh, count)
        .into_iter()
        .map(|r| {
            Ray::new(
                Vec3::new(r.origin.x.round(), r.origin.y.round(), r.origin.z),
                r.direction,
            )
        })
        .collect()
}

/// Random origins and directions: the worst case for cache locality.
fn incoherent_rays(mesh: &TriMesh, count: usize) -> Vec<Ray> {
    let mut rng = StdRng::seed_from_u64(BENCH_SEED);
    let b = mesh.bounds();
    let centre = b.center();
    let radius = b.extent().length();
    (0..count)
        .map(|_| {
            let origin = centre
                + Vec3::new(
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                ) * radius;
            let direction = (centre - origin)
                + Vec3::new(
                    rng.random_range(-0.5..0.5),
                    rng.random_range(-0.5..0.5),
                    rng.random_range(-0.5..0.5),
                ) * radius;
            Ray::new(origin, direction.normalize().unwrap_or(Vec3::Z))
        })
        .collect()
}

fn bench_ray_queries(c: &mut Criterion) {
    // A mesh big enough that the hierarchy matters, small enough to stay in
    // cache so the numbers are about the algorithm rather than about DRAM.
    let generic = shapes::icosphere(50.0, 4); // 5,120 triangles
    let lattice = shapes::lattice_block(21); // 5,292 triangles — near-identical size
    let generic_bvh = Bvh::build(&generic);
    let lattice_bvh = Bvh::build(&lattice);

    let mut group = c.benchmark_group("mesh/rays");
    group.sample_size(20);

    for (label, mesh, bvh, rays) in [
        (
            "coherent-generic",
            &generic,
            &generic_bvh,
            coherent_rays(&generic, 64),
        ),
        (
            "incoherent-generic",
            &generic,
            &generic_bvh,
            incoherent_rays(&generic, 4096),
        ),
        (
            "coherent-lattice-offset",
            &lattice,
            &lattice_bvh,
            coherent_rays(&lattice, 64),
        ),
        (
            // The adversarial case: every ray strikes vertices and edges, so
            // most triangle tests take the exact path.
            "coherent-lattice-aligned",
            &lattice,
            &lattice_bvh,
            aligned_rays(&lattice, 64),
        ),
    ] {
        group.throughput(Throughput::Elements(rays.len() as u64));
        group.bench_function(label, |b| {
            let mut scratch = Vec::with_capacity(16);
            b.iter(|| {
                let mut crossings = 0usize;
                for ray in &rays {
                    if bvh
                        .intersect_ray_all_into(mesh, black_box(ray), &mut scratch)
                        .is_ok()
                    {
                        crossings += scratch.len();
                    }
                }
                black_box(crossings)
            });
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_parse,
    bench_weld,
    bench_validate,
    bench_bvh_build,
    bench_ray_queries
);
criterion_main!(benches);
