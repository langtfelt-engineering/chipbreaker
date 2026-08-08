// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Native arcs against the chords a CAM post would emit.
//!
//! # What is being compared, and why not net volume
//!
//! The obvious comparison — removed volume, native minus linearised — is the
//! wrong one. Chords cut *inside* the arc on the outer wall and *outside* it on
//! the inner wall of an annulus, and those two errors have opposite sign. A net
//! volume comparison lets them cancel, so a linearisation that is visibly wrong
//! in both directions can report a small difference.
//!
//! So the metric here is the **symmetric difference** of the two fields: material
//! present in one and absent in the other, either way round, summed over every
//! ray of every bundle. Nothing cancels. It is zero only if the two cuts agree
//! everywhere the lattice can see.
//!
//! # The crossover that is not there
//!
//! The question is the tolerance at which refining the chords stops buying
//! anything, on the reasoning that the disagreement must eventually disappear
//! under the lattice's own sampling error. **There is no such tolerance**, and
//! `there_is_no_crossover_below_the_cell_size` measures it out to a 1024th of a
//! cell: the disagreement is still halving with every halving of the tolerance,
//! with no sign of a floor.
//!
//! The reason is a property of the representation, not of these arcs. A dexel
//! stores its span endpoints as **continuous** positions along the ray. The cell
//! size quantises which rays exist; it does not quantise where the surface sits
//! along one. So a chord error of a micrometre moves every endpoint it touches
//! by a micrometre, and the field records it.
//!
//! The practical reading is the reverse of the expected one. A 0.25 mm lattice
//! can still tell a 1 um chord tolerance from a 10 um one. What limits
//! refinement is cost alone: the sagitta is second order in the angle, so the
//! chord count runs as the inverse square root of the tolerance, forever.
//!
//! # What this finding does NOT say
//!
//! It is about linearisation as a **validation device** -- our chords, compared
//! against our arc -- and not about a customer's post-processor tolerance.
//!
//! If a post has already emitted chords, those chords are what the machine
//! executes. Simulating them is then simply correct, and there is no error to
//! measure: the part really is the chorded one. The tolerance question only
//! arises when *we* choose to linearise something the controller would have
//! interpolated natively, which is exactly what `--no-arc-native` does and
//! nothing else does.
//!
//! Stated the other way round: this measures how well our two computations of
//! one arc agree, not how well a chorded program matches an ideal curve.
//!
//! Run with `--nocapture` to see the tables.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::spans::Spans;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri_motion};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

const PI: f64 = core::f64::consts::PI;
const SPACING: f64 = 0.25;
const SWEEP_TOL: f64 = SPACING / 10.0;

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill(radius: f64) -> Profile {
    flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid")
}

/// Volume present in one field and not the other, either way round.
///
/// # Panics
/// Panics if the two fields do not have the same bundles and ray counts.
fn symmetric_difference(a: &TriDexelField, b: &TriDexelField) -> f64 {
    let mut total = 0.0;
    for axis in AXES {
        let (Some(left), Some(right)) = (a.bundle(axis), b.bundle(axis)) else {
            continue;
        };
        let cell_area = left.lattice().cell_area();
        assert_eq!(
            left.arena().rays(),
            right.arena().rays(),
            "the two fields must be built from the same stock"
        );
        let (mut l, mut r) = (Spans::new(), Spans::new());
        let (mut only_left, mut only_right) = (Spans::new(), Spans::new());
        for ray in 0..left.arena().rays() {
            let ray = u32::try_from(ray).expect("u32 indices");
            left.arena().read_into(ray, &mut l);
            right.arena().read_into(ray, &mut r);
            if l.as_slice() == r.as_slice() {
                continue;
            }
            l.subtract_into(&r, &mut only_left);
            r.subtract_into(&l, &mut only_right);
            total += (only_left.measure() + only_right.measure()) * cell_area;
        }
    }
    total
}

fn cut_native(motion: &Motion, profile: &Profile) -> TriDexelField {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    cut_tri_motion(
        &mut field,
        profile,
        motion,
        SweepMethod::Analytic {
            tolerance: SWEEP_TOL,
        },
        &mut scratch,
    );
    field
}

fn cut_linearised(chords: &[LinearMove], profile: &Profile) -> TriDexelField {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let motions: Vec<Motion> = chords.iter().copied().map(Motion::Linear).collect();
    cut_all(
        &mut field,
        profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: SWEEP_TOL,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// Total material the cut removed, summed over bundles.
fn removed_total(before: &TriDexelField, after: &TriDexelField) -> f64 {
    let (b, a) = (before.volumes(), after.volumes());
    let mut sum = 0.0;
    for axis in AXES {
        let i = axis.index();
        if let (Some(b), Some(a)) = (b[i], a[i]) {
            sum += b - a;
        }
    }
    sum
}

/// The ladder of chord tolerances every case walks.
const LADDER: [f64; 9] = [
    2.0, 1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125,
];

/// Walks a ladder, prints the table, and returns
/// `(tolerance, chords, symmetric difference)` for each rung.
fn converge(label: &str, arc: &ArcMove, profile: &Profile, ladder: &[f64]) -> Vec<(f64, u32, f64)> {
    let motion = Motion::Arc(*arc);
    let native = cut_native(&motion, profile);
    let baseline = stock();
    let native_removed = removed_total(&baseline, &native);

    println!("\n{label}");
    println!(
        "  native: {native_removed:.6} mm3 removed, {}",
        if arc.is_level_xy() {
            "closed form".to_owned()
        } else {
            let (steps, bound) = arc.substeps_for_error(SWEEP_TOL);
            format!("sub-stepped {steps} times, bound {bound:.6} mm")
        }
    );
    println!("   chord tol      chords   sym.diff mm3   as % of removed");

    let mut rows = Vec::new();
    for &tolerance in ladder {
        let chords = arc.chords_for_error(tolerance);
        let field = cut_linearised(&arc.linearise(tolerance), profile);
        let difference = symmetric_difference(&native, &field);
        println!(
            "  {tolerance:>9.7}   {chords:>7}   {difference:>12.6}   {:>13.4}",
            difference / native_removed * 100.0
        );
        rows.push((tolerance, chords, difference));
    }
    rows
}

/// The finest rung's disagreement, which every coarser rung is judged against.
fn floor_of(rows: &[(f64, u32, f64)]) -> f64 {
    rows.last().expect("the ladder is not empty").2
}

#[test]
fn a_level_arc_converges_to_its_chords() {
    let profile = mill(3.0);
    let arc = ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 8.0,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 7.0,
        plane: ArcPlane::Xy,
        rise: 0.0,
    };
    let rows = converge(
        "level full circle, R=8 r=3 depth=3",
        &arc,
        &profile,
        &LADDER,
    );

    // Convergence, stated as a ratio rather than a monotone chain. Volume in a
    // sampled field is not monotone under refinement (ADR 0005 reason 1), and
    // asserting a strictly decreasing column would be asserting something known
    // to be false.
    let coarse = rows[0].2;
    let fine = floor_of(&rows);
    assert!(
        fine * 20.0 < coarse,
        "eight halvings of the chord tolerance should cut the disagreement by \
         far more than 20x; went from {coarse:.6} to {fine:.6} mm3"
    );

    // Chord count against the sagitta law: halving the tolerance multiplies the
    // count by sqrt(2), because the sagitta is second order in the angle. This
    // is what makes linearisation expensive to refine and native arcs worth
    // having.
    for pair in rows.windows(2) {
        let growth = f64::from(pair[1].1) / f64::from(pair[0].1);
        assert!(
            (1.2..=1.7).contains(&growth),
            "halving the tolerance should grow the chord count by about sqrt(2); \
             {} -> {} is {growth:.3}x",
            pair[0].1,
            pair[1].1
        );
    }
}

#[test]
fn an_arc_sector_converges_to_its_chords() {
    let profile = mill(2.0);
    let arc = ArcMove {
        center: Vec3::new(18.0, 14.0, 0.0),
        radius: 10.0,
        start_angle: 0.4,
        sweep: -1.9,
        z: 8.0,
        plane: ArcPlane::Xy,
        rise: 0.0,
    };
    // Clockwise and partial, so the wedge and the end caps both matter and a
    // sign error in the sweep would show as a wildly wrong baseline.
    let rows = converge(
        "clockwise sector, R=10 r=2 depth=2",
        &arc,
        &profile,
        &LADDER,
    );
    assert!(floor_of(&rows) * 20.0 < rows[0].2);
}

#[test]
fn a_helix_converges_to_its_chords() {
    // Native here is itself sub-stepped, so this compares two approximations
    // rather than an approximation against a closed form. They still have to
    // agree, and the fact that they do is the check that the helix sub-stepping
    // and the chord decomposition are not making the same mistake.
    let profile = mill(3.0);
    let arc = ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 5.0,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 10.0,
        plane: ArcPlane::Xy,
        rise: -5.0,
    };
    let rows = converge(
        "helical bore, R=5 r=3 rise=-5 one turn",
        &arc,
        &profile,
        &LADDER,
    );
    assert!(
        floor_of(&rows) * 5.0 < rows[0].2,
        "the helix should converge too, if less sharply: native is sub-stepped \
         and carries its own error, so the two never meet exactly"
    );
}

/// Five more halvings, to look for a floor the shorter ladder cannot reach.
const DEEP_LADDER: [f64; 14] = [
    2.0,
    1.0,
    0.5,
    0.25,
    0.125,
    0.0625,
    0.03125,
    0.015625,
    0.0078125,
    0.00390625,
    0.001_953_125,
    0.000_976_562_5,
    0.000_488_281_25,
    0.000_244_140_625,
];

#[test]
fn there_is_no_crossover_below_the_cell_size() {
    // This test was written to find a crossover: the chord tolerance past which
    // refining buys nothing because the lattice cannot see the difference. There
    // is no such tolerance, and the reason is worth stating.
    //
    // A dexel stores its span endpoints as **continuous** positions along the
    // ray. The cell size quantises only *which rays exist*, never *where the
    // surface is along one*. So a chord error of a micrometre still moves every
    // endpoint it touches by a micrometre, and the symmetric difference keeps
    // falling in proportion, with no floor -- measured here to a four-thousandth
    // of a cell.
    //
    // The practical consequence is the opposite of what the plan expected: a
    // simulation on a 0.25 mm lattice can still tell a 1 um post-processor
    // tolerance from a 10 um one. Coarsening the lattice does not license
    // coarsening the post.
    let profile = mill(3.0);
    let arc = ArcMove {
        center: Vec3::new(20.0, 15.0, 0.0),
        radius: 8.0,
        start_angle: 0.0,
        sweep: 2.0 * PI,
        z: 7.0,
        plane: ArcPlane::Xy,
        rise: 0.0,
    };
    let rows = converge(
        "no-crossover probe: R=8 r=3 depth=3 on a 0.25 mm lattice",
        &arc,
        &profile,
        &DEEP_LADDER,
    );

    // The law: symmetric difference is proportional to chord tolerance. Fitted
    // as the ratio across the last decade, where the coarse-end nonlinearity
    // (chords too few for the sagitta formula to be the whole story) is gone.
    let deep = &rows[rows.len() - 5..];
    for pair in deep.windows(2) {
        let ratio = pair[0].2 / pair[1].2;
        assert!(
            (1.5..=2.6).contains(&ratio),
            "halving the chord tolerance should roughly halve the disagreement,              with no floor in sight; {} mm -> {} mm gave {ratio:.3}x",
            pair[0].0,
            pair[1].0
        );
    }

    let finest = rows.last().expect("non-empty");
    println!(
        "  NO CROSSOVER. At {} mm -- a {:.0}th of the {SPACING} mm cell -- the disagreement is still {:.6} mm3 and still halving.",
        finest.0,
        SPACING / finest.0,
        finest.2
    );
    println!(
        "  Dexel endpoints are continuous along the ray, so the lattice never becomes the limiting error. Cost is the only limit: {} chords.",
        finest.1
    );

    // And the cost side, which is the actual argument for native arcs: the chord
    // count grows as the inverse square root of the tolerance, so buying the
    // last factor of two in accuracy costs 40% more segments, forever.
    let coarse_chords = f64::from(rows[8].1);
    let fine_chords = f64::from(finest.1);
    let accuracy_gain = rows[8].2 / finest.2;
    println!(
        "  {:.0}x less disagreement cost {:.1}x the segments ({} -> {}).",
        accuracy_gain,
        fine_chords / coarse_chords,
        rows[8].1,
        finest.1
    );
    assert!(
        fine_chords > coarse_chords,
        "refining must cost segments, or the ladder is not doing anything"
    );
}
