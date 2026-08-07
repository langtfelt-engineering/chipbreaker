// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Does each corpus case actually inject the defect it claims to?
//!
//! The corpus is the oracle for recall, so a case that perturbs nothing is worse
//! than a missing case: it sits in the denominator and can never be found, and
//! the resulting recall figure reads as a limit of the detector when it is a
//! property of the corpus.
//!
//! The measurement here does not go anywhere near `deviation::compare`. It runs
//! the clean program and the dirty one into two fields on the same lattice, and
//! measures the **Hausdorff distance between the two span sets, ray by ray**.
//! That is one-dimensional and exact: two sets of intervals on the same line,
//! with no meshes, no normals and no containment test involved. If it reports
//! zero, the two programs cut the same solid and the case injects nothing,
//! whatever its name says.

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::defect::{DefectCase, STOCK, corpus};
use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::spans::Span;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

const SPACING: f64 = 0.4;

fn stock_mesh() -> TriMesh {
    shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(STOCK[0], STOCK[1], STOCK[2]),
    )
}

fn mill(diameter: f64) -> Profile {
    flat_end_mill(diameter, 30.0, &Shank::plain(diameter, 60.0)).expect("valid")
}

fn cut(motions: &[Motion], profile: &Profile) -> TriDexelField {
    let mut field = TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let mut scratch = CutScratch::new(profile);
    cut_all(
        &mut field,
        profile,
        motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// The dirty run for a case, matching what the recall harness does.
fn dirty(case: &DefectCase) -> TriDexelField {
    if case.tool_length_delta_mm > 0.0 {
        let lowered: Vec<Motion> = case
            .motions
            .iter()
            .map(|m| match m {
                Motion::Linear(l) => Motion::Linear(LinearMove {
                    start: Vec3::new(l.start.x, l.start.y, l.start.z - case.tool_length_delta_mm),
                    end: Vec3::new(l.end.x, l.end.y, l.end.z - case.tool_length_delta_mm),
                }),
                other => *other,
            })
            .collect();
        cut(&lowered, &mill(6.0))
    } else {
        cut(&case.motions, &mill(6.0 + case.tool_diameter_delta_mm))
    }
}

/// Distance from `t` to the nearest point of a set of intervals.
fn distance_to(spans: &[Span], t: f64) -> f64 {
    let mut best = f64::INFINITY;
    for s in spans {
        let d = if t < s.t0 {
            s.t0 - t
        } else if t > s.t1 {
            t - s.t1
        } else {
            0.0
        };
        best = best.min(d);
    }
    best
}

/// One-sided Hausdorff: the furthest any bound of `a` is from the set `b`.
fn one_sided(a: &[Span], b: &[Span]) -> f64 {
    if a.is_empty() {
        return 0.0;
    }
    if b.is_empty() {
        // Everything of `a` vanished. Its own extent is the displacement.
        return a
            .iter()
            .map(|s| s.t1 - s.t0)
            .fold(0.0f64, f64::max)
            .max(0.0);
    }
    let mut worst = 0.0f64;
    for s in a {
        worst = worst.max(distance_to(b, s.t0)).max(distance_to(b, s.t1));
    }
    worst
}

/// The largest distance by which the two fields' surfaces differ, along rays.
///
/// Both fields are built from the same stock on the same lattice, so ray `r` of
/// bundle `axis` is the same line in both and the comparison is between two
/// interval sets on it.
fn injected_mm(clean: &TriDexelField, dirty: &TriDexelField) -> f64 {
    let mut worst = 0.0f64;
    for axis in AXES {
        let (Some(a), Some(b)) = (clean.bundle(axis), dirty.bundle(axis)) else {
            continue;
        };
        let rays = u32::try_from(a.arena().rays()).expect("small");
        for r in 0..rays {
            let sa = a.arena().get(r);
            let sb = b.arena().get(r);
            worst = worst.max(one_sided(sa, sb)).max(one_sided(sb, sa));
        }
    }
    worst
}

fn main() {
    let cases = corpus();
    let profile = mill(6.0);

    let mut vacuous: Vec<(String, f64, f64)> = Vec::new();
    let mut weak: Vec<(String, f64, f64)> = Vec::new();
    let mut checked = 0usize;

    for (index, case) in cases.iter().enumerate() {
        // The full corpus builds two fields each, which is minutes. Every third
        // case keeps every kind, locale and depth band represented while staying
        // inside a coffee break.
        if index % 3 != 0 {
            continue;
        }
        checked += 1;
        let clean = cut(&case.clean, &profile);
        let got = injected_mm(&clean, &dirty(case));
        let want = case.depth_mm.abs();
        // Relative to the claim, not against an absolute floor. The first
        // version of this classifier used `SPACING / 4`, which flagged six cases
        // injecting exactly the 0.08 mm they claimed -- shallow is not the same
        // as vacuous, and a corpus needs its shallow end most of all.
        if got <= 1.0e-9 {
            vacuous.push((case.id.clone(), want, got));
        } else if got < 0.5 * want {
            weak.push((case.id.clone(), want, got));
        }
    }

    println!("{checked} of {} cases measured\n", cases.len());
    println!("injecting nothing measurable ({}):", vacuous.len());
    for (id, want, got) in &vacuous {
        println!("  {id:<44} claims {want:.2} mm, injects {got:.4} mm");
    }
    println!(
        "\ninjecting less than half what they claim ({}):",
        weak.len()
    );
    for (id, want, got) in &weak {
        println!("  {id:<44} claims {want:.2} mm, injects {got:.4} mm");
    }
    #[allow(clippy::cast_precision_loss, reason = "small counts")]
    let rate = (vacuous.len() + weak.len()) as f64 / checked.max(1) as f64;
    println!(
        "\n{:.1}% of the corpus does not inject what it claims",
        rate * 100.0
    );
}
