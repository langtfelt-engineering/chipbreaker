// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! What a multi-setup job costs.
//!
//! # Per-tool or amortised, which is the question a shop actually asks
//!
//! Collision checking measured 53% over cutting alone for one tool. A real job
//! changes tools several times, and whether that 53% is paid **once** or **per
//! tool** is the difference between a check somebody leaves on and one they
//! switch off.
//!
//! The two arms here are the same total cutting work, split one way and the
//! other: one tool over N passes against N tools over one pass each. If the
//! overhead is amortised the two agree; if it is per-tool the second is dearer
//! by roughly the number of tools.
//!
//! # Re-fixturing should cost almost nothing, and that is worth confirming
//!
//! The axis-aligned path is a relabelling — no rays are cast and no
//! intersections are solved. It should therefore scale with the number of
//! *spans*, not with the volume of the field, and be far cheaper than the build
//! it replaces. Measuring it against a fresh build of the same geometry says
//! whether that holds.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::math::{Mat4, Vec3};
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::refixture::refixture_exact;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{HolderStage, Shank, flat_end_mill};
use chipbreaker_core::toolpath::{MotionKind, Provenance};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::hint::black_box;

const SPACING: f64 = 0.7;
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

/// A cutter of the given diameter under an ER32 chuck, so every tool carries
/// real non-cutting geometry.
fn tool(diameter: f64) -> Profile {
    flat_end_mill(
        diameter,
        24.0,
        &Shank::with_holder(
            diameter,
            40.0,
            [
                HolderStage::cylinder(50.8, 28.0),
                HolderStage::cylinder(61.912_499_999_999_994, 50.0),
            ],
        ),
    )
    .expect("valid")
}

/// One pass across the block at height `z`, offset in y.
fn pass(y: f64, z: f64) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(6.0, y, z),
        end: Vec3::new(74.0, y, z),
    })
}

fn method() -> SweepMethod {
    SweepMethod::Analytic {
        tolerance: SPACING / 10.0,
    }
}

fn params() -> CollideParams {
    CollideParams {
        clearance_mm: 0.0,
        grid_mm: 2.0 * SPACING,
        method: method(),
    }
}

/// Checks `passes` motions with one tool, or splits them across `tools`.
fn check(tools: usize, passes: usize) -> usize {
    let mut f = field();
    let mut found = 0usize;
    let per = passes / tools.max(1);
    for t in 0..tools {
        #[allow(clippy::cast_precision_loss, reason = "a handful of tools")]
        let profile = tool(6.0 + (t as f64));
        let motions: Vec<Motion> = (0..per)
            .map(|i| {
                #[allow(clippy::cast_precision_loss, reason = "a handful of passes")]
                let k = (t * per + i) as f64;
                pass(8.0 + k * 3.0, 22.0)
            })
            .collect();
        let kinds = vec![MotionKind::Linear; motions.len()];
        let provenance: Vec<Provenance> = (0..motions.len())
            .map(|i| Provenance::new(0, u32::try_from(i).unwrap_or(0), 0))
            .collect();
        let mut scratch = CutScratch::new(&profile);
        let c = collide_with_stock(
            &mut f,
            &profile,
            &motions,
            &kinds,
            &provenance,
            0,
            &[],
            &params(),
            &mut scratch,
        )
        .expect("every tool here has a chuck");
        found += c.len();
    }
    found
}

/// The same cutting work, split across one tool or several.
///
/// A flat line means the overhead is amortised across the job; a line rising
/// with the tool count means it is paid per tool.
fn per_tool_or_amortised(c: &mut Criterion) {
    let mut group = c.benchmark_group("job/tools");
    group.sample_size(10);
    for tools in [1usize, 2, 4] {
        group.bench_with_input(BenchmarkId::from_parameter(tools), &tools, |b, &n| {
            b.iter(|| black_box(check(n, 12)));
        });
    }
    group.finish();
}

/// Cutting alone, for the overhead ratio at this fixture's size.
fn cutting_only(c: &mut Criterion) {
    let profile = tool(6.0);
    let motions: Vec<Motion> = (0..12)
        .map(|i| {
            #[allow(clippy::cast_precision_loss, reason = "a handful of passes")]
            let k = i as f64;
            pass(8.0 + k * 3.0, 22.0)
        })
        .collect();
    let mut group = c.benchmark_group("job/cut_only");
    group.sample_size(10);
    group.bench_function("twelve_passes", |b| {
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
    group.finish();
}

/// Crossing a setup boundary, against building the same field from scratch.
fn refixture_cost(c: &mut Criterion) {
    let quarter = Mat4::from_rows_array([
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let source = field();

    let mut group = c.benchmark_group("job/refixture");
    group.sample_size(20);
    group.bench_function("axis_aligned_move", |b| {
        b.iter(|| {
            let moved = refixture_exact(&source, &quarter).expect("axis-aligned");
            black_box(moved.total_spans())
        });
    });
    // The alternative this replaces: building the field again in the new
    // orientation. Not what the engine does -- it is here as the yardstick.
    group.bench_function("rebuild_from_mesh", |b| {
        b.iter(|| black_box(field().total_spans()));
    });
    group.finish();
}

criterion_group!(benches, cutting_only, per_tool_or_amortised, refixture_cost);
criterion_main!(benches);
