// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

#![allow(
    missing_docs,
    reason = "criterion_group! generates an undocumented public fn"
)]

//! What it costs to turn a field of deviations into a list of findings.
//!
//! # Three stages with three different shapes
//!
//! **Clustering** scales with the number of samples *above tolerance*, not with
//! the field. A clean part clusters instantly however large it is, and a part
//! that is wrong everywhere is the expensive case — which is the right way
//! round, because a part that is wrong everywhere is one somebody is about to
//! spend a long time on anyway.
//!
//! **Attribution** scales with findings times segments, and the box rejection is
//! what keeps the second factor from mattering. It is measured with the
//! rejection in place because that is how it runs.
//!
//! **Diffing** scales with findings alone and is a map lookup per finding, so it
//! is here mostly to confirm it stays that way: `report-diff` is meant to run in
//! somebody's CI on every push, and a diff that got slower than the verification
//! it compares would be a strange thing to ship.

use chipbreaker_core::deviation::{Deviation, compare};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::cluster::{ClusterParams, cluster};
use chipbreaker_core::findings::report::{Manifest, Report, semantics_from};
use chipbreaker_core::findings::verdict::{self, Verdict};
use chipbreaker_core::findings::{attribute_finding, diff, identify};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::toolpath::Provenance;
use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use std::hint::black_box;

const STOCK: Vec3 = Vec3 {
    x: 40.0,
    y: 30.0,
    z: 12.0,
};
const SPACING: f64 = 0.4;

fn stock_mesh() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), STOCK)
}

fn mill() -> Profile {
    flat_end_mill(6.0, 30.0, &Shank::plain(6.0, 60.0)).expect("valid")
}

/// A raster of passes at `z`, which is what a facing job looks like.
fn raster(z: f64) -> Vec<Motion> {
    let mut out = Vec::new();
    let mut y = 6.0;
    let mut forward = true;
    while y <= 24.0 {
        let (a, b) = if forward { (5.0, 35.0) } else { (35.0, 5.0) };
        out.push(Motion::Linear(LinearMove {
            start: Vec3::new(a, y, z),
            end: Vec3::new(b, y, z),
        }));
        y += 4.5;
        forward = !forward;
    }
    out
}

fn cut(motions: &[Motion]) -> TriDexelField {
    let mut field = TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let profile = mill();
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// The deviation field of a part cut `depth` too deep against its own nominal.
fn samples(depth: f64) -> (Vec<Deviation>, TriMesh) {
    use chipbreaker_core::contour::{ContourOptions, extract};
    let nominal = extract(&cut(&raster(8.0)), &ContourOptions::default())
        .expect("extracts")
        .0;
    let field = cut(&raster(8.0 - depth));
    let d = compare(&field, &nominal, Some(&stock_mesh()));
    (d.samples, nominal)
}

/// Clustering against how much of the part is out of tolerance.
fn clustering(c: &mut Criterion) {
    let mut group = c.benchmark_group("findings/cluster");
    group.sample_size(20);
    for depth in [0.0, 0.5, 2.0] {
        let (s, _) = samples(depth);
        let params = ClusterParams::for_spacing(SPACING, SPACING / 2.0);
        let above = s
            .iter()
            .filter(|d| d.signed_mm.abs() > params.tolerance_mm)
            .count();
        let found = cluster(&s, &params, SPACING).len();
        eprintln!(
            "depth {depth} mm: {} samples, {above} above tolerance, {found} findings",
            s.len()
        );
        // Throughput in samples above tolerance, which is what the work is
        // proportional to -- quoting it per total sample would flatter a clean
        // part and say nothing about a bad one.
        group.throughput(Throughput::Elements(above.max(1) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(depth), &s, |b, s| {
            b.iter(|| black_box(cluster(s, &params, SPACING).len()));
        });
    }
    group.finish();
}

/// Attribution, with the box rejection that makes it affordable.
fn attribution(c: &mut Criterion) {
    let (s, _) = samples(2.0);
    let params = ClusterParams::for_spacing(SPACING, SPACING / 2.0);
    let findings = identify(cluster(&s, &params, SPACING), params.radius_mm);
    let motions = raster(6.0);
    let profile = mill();
    let bounds: Vec<_> = motions.iter().map(|m| m.swept_bounds(&profile)).collect();
    let provenance: Vec<Provenance> = (0..motions.len())
        .map(|i| Provenance::new(0, u32::try_from(i).unwrap_or(0), 0))
        .collect();
    eprintln!(
        "attribution: {} findings against {} segments",
        findings.len(),
        motions.len()
    );

    let mut group = c.benchmark_group("findings/attribute");
    group.sample_size(20);
    group.throughput(Throughput::Elements(findings.len().max(1) as u64));
    group.bench_function("all_findings", |b| {
        let mut scratch = CutScratch::new(&profile);
        b.iter(|| {
            let mut n = 0usize;
            for f in &findings {
                let a = attribute_finding(
                    &profile,
                    &motions,
                    &bounds,
                    &provenance,
                    SweepMethod::Analytic {
                        tolerance: SPACING / 10.0,
                    },
                    &mut scratch,
                    &f.probes,
                );
                n += a.segments.len();
            }
            black_box(n)
        });
    });
    group.finish();
}

/// Diffing two reports, against the number of findings.
fn diffing(c: &mut Criterion) {
    let build = |depth: f64| -> Report {
        let (s, _) = samples(depth);
        let params = ClusterParams::for_spacing(SPACING, SPACING / 2.0);
        let findings = identify(cluster(&s, &params, SPACING), params.radius_mm);
        Report {
            manifest: Manifest {
                inputs: Vec::new(),
                spacing_mm: [SPACING; 3],
                tolerance_mm: SPACING / 2.0,
                cluster_radius_mm: params.radius_mm,
                engine_version: "bench".to_owned(),
                engine_selftest: "bench".to_owned(),
                boundaries: Vec::new(),
            },
            semantics: semantics_from(
                &chipbreaker_core::deviation::DeviationField::default(),
                [SPACING; 3],
                SPACING / 2.0,
                None,
            ),
            verdict: Verdict::new().with(verdict::GATE_GOUGE, Report::gouge_gate(&findings)),
            findings,
            collisions: Vec::new(),
            rapid_path: None,
        }
    };
    let a = build(2.0);
    let b = build(2.5);
    eprintln!(
        "diff: {} findings against {}",
        a.findings.len(),
        b.findings.len()
    );

    let mut group = c.benchmark_group("findings/diff");
    group.throughput(Throughput::Elements(a.findings.len().max(1) as u64));
    group.bench_function("two_reports", |bn| {
        bn.iter(|| black_box(diff::diff(&a, &b).changes.len()));
    });
    group.finish();
}

/// Scale, on a part with many separate findings.
///
/// The raster fixtures above produce **one** finding, because a raster gouged
/// uniformly is one connected region — which is realistic and useless for
/// measuring how the list scales. So this builds the pathological case
/// directly: isolated deviations scattered far enough apart that none of them
/// join, giving one finding per sample.
///
/// A real part with ten thousand *separate* findings would be one nobody should
/// be machining, but a report generator has to survive one, and a customer's CI
/// running `report-diff` over it should not be the slow part of their pipeline.
fn at_scale(c: &mut Criterion) {
    let params = ClusterParams {
        // Tight, so nothing merges and the count is exactly the sample count.
        radius_mm: 0.05,
        tolerance_mm: 0.1,
    };
    let mut group = c.benchmark_group("findings/scale");
    group.sample_size(10);

    for n in [1_000usize, 10_000] {
        // A lattice of isolated samples, one millimetre apart, which is twenty
        // times the radius.
        // Integer cube root by search. `f64::cbrt` is a disallowed method --
        // the determinism rules apply to benchmarks too, and rightly: a
        // benchmark that computes its fixture differently on two targets is
        // measuring two different things.
        let side = (1usize..).find(|k| k * k * k >= n).unwrap_or(1);
        let mut s = Vec::with_capacity(n);
        for i in 0..n {
            let (x, y, z) = (i % side, (i / side) % side, i / (side * side));
            let at = Vec3::new(x as f64, y as f64, z as f64);
            s.push(Deviation {
                at,
                normal: Vec3::new(0.0, 0.0, 1.0),
                signed_mm: -1.0,
                perpendicular_mm: -1.0,
                nearest_on_nominal: at,
                axis: 2,
            });
        }
        let found = cluster(&s, &params, SPACING).len();
        eprintln!("{n} isolated samples -> {found} findings");
        assert_eq!(found, n, "the fixture must not merge anything");

        group.throughput(Throughput::Elements(n as u64));
        group.bench_with_input(BenchmarkId::new("cluster", n), &s, |b, s| {
            b.iter(|| black_box(cluster(s, &params, SPACING).len()));
        });
        group.bench_with_input(BenchmarkId::new("identify", n), &s, |b, s| {
            b.iter(|| {
                let cs = cluster(s, &params, SPACING);
                black_box(identify(cs, params.radius_mm).len())
            });
        });
    }
    group.finish();
}

criterion_group!(benches, clustering, attribution, diffing, at_scale);
criterion_main!(benches);
