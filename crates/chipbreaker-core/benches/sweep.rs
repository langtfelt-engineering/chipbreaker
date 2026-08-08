// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

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
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{cut_all, split_runs};
use chipbreaker_core::sweep::cut::{CutScratch, CutStats, SweepMethod, cut_tri, cut_tri_motion};
use chipbreaker_core::sweep::{LinearMove, Motion, horizontal, plunge, reference};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, bull_end_mill, flat_end_mill};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};
use chipbreaker_core::toolpath::ArcPlane;
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
            spacing_xyz: None,
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

const PI: f64 = core::f64::consts::PI;

fn level_arc(sweep: f64, z: f64) -> Motion {
    Motion::Arc(ArcMove {
        center: Vec3::new(50.0, 30.0, 0.0),
        radius: 18.0,
        start_angle: 0.0,
        sweep,
        z,
        plane: ArcPlane::Xy,
        rise: 0.0,
    })
}

fn helix_move() -> Motion {
    Motion::Arc(ArcMove {
        center: Vec3::new(50.0, 30.0, 0.0),
        radius: 8.0,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 20.0,
        plane: ArcPlane::Xy,
        rise: -6.0,
    })
}

/// Arc throughput, split by the sub-case each bundle takes.
///
/// The three bundles do genuinely different work for the same arc, and averaging
/// them would describe nothing. Along the arc's axis the bearing is constant, so
/// the middle piece is a vertical cast at radius `|d - R|` -- the raycaster's
/// raycaster, unchanged. Across it the height is constant, so the condition is
/// an annulus, and a line meets one in the difference of two disc chords with no
/// cast at all. A helix takes neither and sub-steps.
fn arc_by_sub_case(c: &mut Criterion) {
    let profile = mill();
    let spacing = 0.5;
    let field = stock(spacing);
    let method = SweepMethod::Analytic {
        tolerance: spacing / 10.0,
    };
    let rays: u64 = field.bundles().map(|(_, b)| b.arena().rays() as u64).sum();

    let mut group = c.benchmark_group("sweep/arc-case");
    group.throughput(Throughput::Elements(rays));
    for (name, motion) in [
        ("A-prime-full-circle", level_arc(2.0 * PI, 14.0)),
        ("A-prime-quarter", level_arc(PI / 2.0, 14.0)),
        ("B-prime-helix", helix_move()),
    ] {
        group.bench_function(name, |b| {
            b.iter_batched(
                || (stock(spacing), CutScratch::new(&profile)),
                |(mut f, mut scratch)| {
                    black_box(cut_tri_motion(
                        &mut f,
                        &profile,
                        &motion,
                        method,
                        &mut scratch,
                    ))
                },
                criterion::BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

/// Native arcs against the chords a post would emit, at several tolerances.
///
/// The accuracy side is in `tests/sweep_linearise.rs`, which finds no crossover:
/// dexel endpoints are continuous along the ray, so refining chords keeps paying
/// down to a thousandth of a cell. What is timed here is the price, and the
/// price is the argument for native arcs.
fn native_against_linearised(c: &mut Criterion) {
    let profile = mill();
    let spacing = 0.5;
    let method = SweepMethod::Analytic {
        tolerance: spacing / 10.0,
    };
    let Motion::Arc(arc) = level_arc(2.0 * PI, 14.0) else {
        unreachable!()
    };

    let mut group = c.benchmark_group("sweep/native-vs-linearised");
    group.bench_function("native", |b| {
        b.iter_batched(
            || (stock(spacing), CutScratch::new(&profile)),
            |(mut f, mut scratch)| {
                black_box(cut_tri_motion(
                    &mut f,
                    &profile,
                    &Motion::Arc(arc),
                    method,
                    &mut scratch,
                ))
            },
            criterion::BatchSize::LargeInput,
        );
    });
    for tolerance in [0.25, 0.0625, 0.015_625, 0.003_906_25] {
        let chords: Vec<Motion> = arc
            .linearise(tolerance)
            .into_iter()
            .map(Motion::Linear)
            .collect();
        let label = format!("{}mm-{}chords", tolerance, chords.len());
        group.bench_with_input(
            BenchmarkId::new("linearised", label),
            &chords,
            |b, chords| {
                b.iter_batched(
                    || (stock(spacing), CutScratch::new(&profile)),
                    |(mut f, mut scratch)| {
                        black_box(cut_all(&mut f, &profile, chords, method, &mut scratch, 32))
                    },
                    criterion::BatchSize::LargeInput,
                );
            },
        );
    }
    group.finish();
}

/// A surface-following pass of **short** segments, which is what a posted 3D
/// program actually contains and what batching is for.
///
/// Long full-width raster passes are the wrong workload to measure batching on,
/// and measuring them first was the mistake. Each such move's box already spans
/// the part, so the union of thirty-two of them is barely larger than one and
/// the rejection rate does not move at all. A posted finishing program is the
/// opposite: thousands of segments a few tenths long, each with a tiny box, so a
/// batch's union is still small and one test really does stand in for thirty-two.
fn fine_motions(count: usize) -> Vec<Motion> {
    let mut out = Vec::with_capacity(count);
    let (mut x, mut y) = (20.0f64, 20.0f64);
    let mut forward = true;
    for k in 0..count {
        let step = 0.4;
        let next_x = if forward { x + step } else { x - step };
        // A gentle Z wander, so the segments are not all coplanar and the box
        // test has something to reject in the third axis too.
        let z = 16.0 + 0.3 * ((k % 17) as f64 / 17.0);
        out.push(Motion::Linear(LinearMove {
            start: Vec3::new(x, y, z),
            end: Vec3::new(next_x, y, z),
        }));
        x = next_x;
        if !(15.0..=65.0).contains(&x) {
            forward = !forward;
            x = x.clamp(15.0, 65.0);
            y += 0.4;
        }
    }
    out
}

/// Batching speedup against batch size, and the rejection rate behind it.
///
/// Size 1 is the unbatched path by another name, so the ratio to it is the
/// speedup. The rejection rate is printed rather than timed, because it is what
/// explains the curve: a batch rejects a ray against the **union** of its boxes,
/// so once a batch spans more than the region its segments work in, the union
/// swells, rejection collapses, and the speedup turns over.
///
/// Note that `rays_rejected` is unchanged by batching **by construction** -- a
/// union rejection is charged as one rejection per motion, so the ratio stays
/// comparable with the unbatched path. What batching changes is how many box
/// tests were run to reach that number, which is why the timing is the
/// measurement and the rate is only the context.
fn batching(c: &mut Criterion) {
    let profile = mill();
    let spacing = 0.5;
    let motions = fine_motions(4000);
    let method = SweepMethod::Analytic {
        tolerance: spacing / 10.0,
    };
    let tools = vec![1u32; motions.len()];

    for size in [1usize, 8, 32, 128, 512] {
        let mut field = stock(spacing);
        let mut scratch = CutScratch::new(&profile);
        let stats: CutStats = cut_all(&mut field, &profile, &motions, method, &mut scratch, size);
        eprintln!(
            "batch {size:>4}: {:>6.3}% of rays rejected, {} runs, {} ray-cuts tested",
            stats.rejection_rate() * 100.0,
            split_runs(&motions, &tools, size).len(),
            stats.rays_tested,
        );
    }

    let mut group = c.benchmark_group("sweep/batching");
    group.sample_size(20);
    group.throughput(Throughput::Elements(motions.len() as u64));
    for size in [1usize, 4, 16, 64, 256, 1024] {
        group.bench_with_input(BenchmarkId::from_parameter(size), &size, |b, &size| {
            b.iter_batched(
                || (stock(spacing), CutScratch::new(&profile)),
                |(mut f, mut scratch)| {
                    black_box(cut_all(
                        &mut f,
                        &profile,
                        &motions,
                        method,
                        &mut scratch,
                        size,
                    ))
                },
                criterion::BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

/// The quartic, on the tool that actually reaches the torus branch.
///
/// Arcs were predicted to drive this up, because the middle piece
/// was expected to need an offset profile. It does not: the bundles split the
/// problem and no profile is constructed. So a corner-radius mill costs the same
/// going round a corner as down a slot, and this times both to show it.
///
/// A ball nose would be the wrong tool for this measurement. Its tip arc is
/// centred on the tool axis, so the raycaster takes the sphere branch -- a
/// quadratic. Only an arc at a non-zero radius sweeps a torus.
fn quartic_cost(c: &mut Criterion) {
    let bull = bull_end_mill(10.0, 2.0, 24.0, &Shank::plain(8.0, 60.0)).expect("valid");
    let spacing = 0.5;
    let method = SweepMethod::Analytic {
        tolerance: spacing / 10.0,
    };
    let slot = Motion::Linear(LinearMove {
        start: Vec3::new(10.0, 30.0, 14.0),
        end: Vec3::new(90.0, 30.0, 14.0),
    });

    let mut group = c.benchmark_group("sweep/torus-tool");
    for (name, motion) in [("slot", slot), ("arc", level_arc(2.0 * PI, 14.0))] {
        group.bench_function(name, |b| {
            b.iter_batched(
                || (stock(spacing), CutScratch::new(&bull)),
                |(mut f, mut scratch)| {
                    black_box(cut_tri_motion(&mut f, &bull, &motion, method, &mut scratch))
                },
                criterion::BatchSize::LargeInput,
            );
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    cut_by_case,
    analytic_against_reference,
    spans_per_ray,
    raster_job,
    arc_by_sub_case,
    native_against_linearised,
    batching,
    quartic_cost
);
criterion_main!(benches);
