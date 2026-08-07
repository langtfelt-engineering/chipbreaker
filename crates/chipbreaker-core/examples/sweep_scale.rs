// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Wall time for a 500,000-segment job, with arcs.
//!
//! Not a criterion benchmark. Criterion measures a small operation many times;
//! this measures one large operation once, because the question is whether a
//! real program finishes in minutes or in days, and that is answered by running
//! one.
//!
//! The program is built to look like posted 3D work rather than like a
//! benchmark: short segments, a wandering depth, arcs where a contour turns, and
//! a retract-reposition-plunge every few hundred moves. A synthetic path of
//! identical parallel passes would flatter the box test badly.
//!
//! Run with `cargo run --release --example sweep_scale`. Debug is roughly
//! twenty times slower and the number means nothing.

#![allow(missing_docs, reason = "an example binary, not API")]

use std::time::Instant;

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

const SEGMENTS: usize = 500_000;
const SPACING: f64 = 0.5;
const PI: f64 = core::f64::consts::PI;

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 60.0, 20.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

/// A program of `count` segments: short moves, arcs at the turns, and a
/// lift-reposition-plunge every 400 moves.
///
/// `ramped` decides whether each cutting move changes depth. It is the whole
/// difference between an all-exact job and one that sub-steps, and both are
/// worth a number: a 2.5D program really is level, and a 3D surfacing program
/// really is not.
fn program(count: usize, ramped: bool) -> Vec<Motion> {
    let mut out = Vec::with_capacity(count);
    let (mut x, mut y) = (12.0f64, 12.0f64);
    let mut forward = true;
    let mut z = 16.0;

    while out.len() < count {
        // A short cutting move.
        let step = 0.4;
        let next_x = if forward { x + step } else { x - step };
        // Depth wanders on a slow cycle. When it wanders *within* a move the
        // move is a ramp and sub-steps; when it only wanders between moves,
        // every move is horizontal and exact.
        let next_z = if ramped {
            15.5 + 0.5 * ((out.len() % 97) as f64 / 97.0)
        } else {
            z
        };
        out.push(Motion::Linear(LinearMove {
            start: Vec3::new(x, y, z),
            end: Vec3::new(next_x, y, next_z),
        }));
        x = next_x;
        z = next_z;

        if !(10.0..=90.0).contains(&x) {
            // The turn, as a real contour makes it: a half-circle stepover.
            x = x.clamp(10.0, 90.0);
            out.push(Motion::Arc(ArcMove {
                center: Vec3::new(x, y + 0.2, 0.0),
                radius: 0.2,
                start_angle: if forward { -PI / 2.0 } else { PI / 2.0 },
                sweep: if forward { PI } else { -PI },
                z,
                plane: ArcPlane::Xy,
                rise: 0.0,
            }));
            y += 0.4;
            forward = !forward;
            if y > 50.0 {
                y = 12.0;
            }
        }

        if out.len() % 400 == 0 {
            // Lift, reposition, plunge. Rapids over air are what the box test
            // is supposed to make free, and leaving them out would flatter it.
            out.push(Motion::Linear(LinearMove {
                start: Vec3::new(x, y, z),
                end: Vec3::new(x, y, 25.0),
            }));
            out.push(Motion::Linear(LinearMove {
                start: Vec3::new(x, y, 25.0),
                end: Vec3::new(x + 1.0, y + 1.0, 25.0),
            }));
            out.push(Motion::Linear(LinearMove {
                start: Vec3::new(x + 1.0, y + 1.0, 25.0),
                end: Vec3::new(x + 1.0, y + 1.0, z),
            }));
            x = (x + 1.0).clamp(10.0, 90.0);
            y = (y + 1.0).min(50.0);
        }
    }
    out.truncate(count);
    out
}

fn main() {
    println!("stock     100 x 60 x 20 mm at {SPACING} mm");
    println!("batch     {DEFAULT_BATCH}");
    for ramped in [false, true] {
        run(ramped);
    }
}

fn run(ramped: bool) {
    let profile = mill();

    let built = Instant::now();
    let motions = program(SEGMENTS, ramped);
    let build_time = built.elapsed();

    let mut arcs = 0usize;
    for motion in &motions {
        if matches!(motion, Motion::Arc(_)) {
            arcs += 1;
        }
    }

    let mut field = stock();
    let rays: usize = field.bundles().map(|(_, b)| b.arena().rays()).sum();
    let mut scratch = CutScratch::new(&profile);

    println!();
    println!(
        "=== {} ===",
        if ramped {
            "3D surfacing: every cutting move changes depth, so every one sub-steps"
        } else {
            "2.5D: every cutting move is level, so every one is exact"
        }
    );
    println!(
        "program   {} segments ({arcs} arcs), built in {build_time:.2?}, {rays} rays",
        motions.len()
    );
    println!("cutting...");

    let started = Instant::now();
    let stats = cut_all(
        &mut field,
        &profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    let elapsed = started.elapsed();

    #[allow(clippy::cast_precision_loss, reason = "reporting a rate")]
    let per_second = motions.len() as f64 / elapsed.as_secs_f64();
    println!();
    println!("wall      {elapsed:.2?} for {} segments", motions.len());
    println!("rate      {per_second:.0} segments/second");
    println!(
        "rays      {} tested, {} rejected ({:.3}% rejection), {} changed",
        stats.rays_tested,
        stats.rays_rejected,
        stats.rejection_rate() * 100.0,
        stats.rays_changed
    );
    println!(
        "sweep     {} ray-cuts exact, {} sub-stepped over {} steps, worst bound {:.6} mm",
        stats.rays_exact, stats.rays_substepped, stats.substeps, stats.worst_bound_mm
    );
    println!(
        "quartics  {} solves over {} casts",
        stats.raycast.quartics, stats.raycast.rays
    );
    println!("removed   {:?} mm^3 per bundle", stats.removed_mm3);
}
