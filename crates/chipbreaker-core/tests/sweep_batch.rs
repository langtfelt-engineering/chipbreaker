// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Batching must not change the answer.
//!
//! Batching inverts the loop: unbatched walks motions outside and rays inside,
//! batched does the reverse. The field cannot care -- rays do not interact, and
//! each ray still sees its motions in order -- but the **statistics** can, since
//! `removed_mm3` is a float sum and a sum reordered is a different sum.
//!
//! So these tests compare **bit patterns**, not tolerances. A comparison with an
//! epsilon here would pass while hiding exactly the defect worth catching.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all, split_runs};
use chipbreaker_core::sweep::cut::{CutScratch, CutStats, SweepMethod, cut_tri_motion};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

const SPACING: f64 = 0.5;
const METHOD: SweepMethod = SweepMethod::Analytic {
    tolerance: SPACING / 10.0,
};

fn flat() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn ball() -> Profile {
    ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

/// Cuts one motion at a time, accumulating exactly as the CLI does.
fn unbatched(profile: &Profile, motions: &[Motion]) -> (TriDexelField, CutStats) {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let mut total = CutStats::default();
    for motion in motions {
        let stats = cut_tri_motion(&mut field, profile, motion, METHOD, &mut scratch);
        total.rays_tested += stats.rays_tested;
        total.rays_rejected += stats.rays_rejected;
        total.rays_changed += stats.rays_changed;
        for slot in 0..3 {
            total.removed_mm3[slot] += stats.removed_mm3[slot];
        }
        total.substeps += stats.substeps;
        total.rays_exact += stats.rays_exact;
        total.rays_substepped += stats.rays_substepped;
        total.worst_bound_mm = total.worst_bound_mm.max(stats.worst_bound_mm);
        total.raycast.merge(&stats.raycast);
    }
    (field, total)
}

/// Cuts in batches of `size`.
fn batched(profile: &Profile, motions: &[Motion], size: usize) -> (TriDexelField, CutStats) {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let stats = cut_all(&mut field, profile, motions, METHOD, &mut scratch, size);
    (field, stats)
}

fn linear(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

fn level_arc(cx: f64, cy: f64, r: f64, from: f64, sweep: f64, z: f64) -> Motion {
    Motion::Arc(ArcMove {
        center: Vec3::new(cx, cy, 0.0),
        radius: r,
        start_angle: from,
        sweep,
        z,
        plane: ArcPlane::Xy,
        rise: 0.0,
    })
}

/// A raster with plunges, retracts and repositions -- the shape of real work,
/// and the one whose consecutive segments batching is meant to exploit.
fn raster() -> Vec<Motion> {
    let mut out = Vec::new();
    let mut y = 6.0;
    let mut left = true;
    while y <= 24.0 {
        let (a, b) = if left { (8.0, 32.0) } else { (32.0, 8.0) };
        out.push(linear([a, y, 12.0], [a, y, 7.0]));
        out.push(linear([a, y, 7.0], [b, y, 7.0]));
        out.push(linear([b, y, 7.0], [b, y, 12.0]));
        y += 3.0;
        left = !left;
    }
    out
}

/// Arcs and helices mixed in, so the batched path is exercised on the sub-cases
/// that dispatch differently -- a level arc takes the closed form, a helix
/// sub-steps, and `split_runs` is supposed to keep them apart.
fn arcs_and_ramps() -> Vec<Motion> {
    vec![
        linear([20.0, 15.0, 12.0], [20.0, 15.0, 6.0]),
        level_arc(20.0, 15.0, 8.0, 0.0, std::f64::consts::PI, 6.0),
        level_arc(
            20.0,
            15.0,
            8.0,
            std::f64::consts::PI,
            std::f64::consts::PI,
            6.0,
        ),
        linear([12.0, 15.0, 6.0], [28.0, 22.0, 3.0]),
        Motion::Arc(ArcMove {
            center: Vec3::new(14.0, 10.0, 0.0),
            radius: 5.0,
            start_angle: 0.0,
            sweep: 2.0 * std::f64::consts::PI,
            z: 9.0,
            plane: ArcPlane::Xy,
            rise: -4.0,
        }),
        linear([28.0, 22.0, 3.0], [28.0, 22.0, 12.0]),
    ]
}

fn assert_identical(label: &str, size: usize, motions: &[Motion], profile: &Profile) {
    let (plain_field, plain) = unbatched(profile, motions);
    let (batch_field, batch) = batched(profile, motions, size);

    assert_eq!(
        digest(&plain_field),
        digest(&batch_field),
        "{label}: batch size {size} produced a different field"
    );
    for slot in 0..3 {
        assert_eq!(
            plain.removed_mm3[slot].to_bits(),
            batch.removed_mm3[slot].to_bits(),
            "{label}: batch size {size}, bundle {slot} removed volume differs by \
             {} mm3 -- a reordered float sum, not a geometry change",
            batch.removed_mm3[slot] - plain.removed_mm3[slot]
        );
    }
    assert_eq!(plain.rays_tested, batch.rays_tested, "{label}: rays tested");
    assert_eq!(
        plain.rays_rejected, batch.rays_rejected,
        "{label}: rays rejected"
    );
    assert_eq!(
        plain.rays_changed, batch.rays_changed,
        "{label}: rays changed"
    );
    assert_eq!(plain.substeps, batch.substeps, "{label}: substeps");
    assert_eq!(plain.rays_exact, batch.rays_exact, "{label}: rays exact");
    assert_eq!(
        plain.rays_substepped, batch.rays_substepped,
        "{label}: rays substepped"
    );
    assert_eq!(
        plain.worst_bound_mm.to_bits(),
        batch.worst_bound_mm.to_bits(),
        "{label}: worst bound"
    );
    assert_eq!(plain.raycast, batch.raycast, "{label}: predicate counters");
}

#[test]
fn batched_raster_is_bit_identical_at_every_size() {
    let profile = flat();
    let motions = raster();
    // Swept, because a bug that only shows when a batch boundary lands
    // mid-motion-group is exactly the bug this is for. Size 1 is the degenerate
    // case that must reduce to unbatched.
    for size in [1, 2, 3, 5, 8, 16, DEFAULT_BATCH, 1024] {
        assert_identical("raster", size, &motions, &profile);
    }
}

#[test]
fn batched_arcs_and_ramps_are_bit_identical() {
    let profile = ball();
    let motions = arcs_and_ramps();
    for size in [1, 2, 3, 4, 8, DEFAULT_BATCH] {
        assert_identical("arcs", size, &motions, &profile);
    }
}

#[test]
fn batched_single_motion_matches_unbatched() {
    let profile = flat();
    let motions = vec![linear([-5.0, 15.0, -1.0], [45.0, 15.0, -1.0])];
    assert_identical("one move", DEFAULT_BATCH, &motions, &profile);
}

#[test]
fn split_runs_never_spans_a_tool_change() {
    let motions = raster();
    let mut tools = vec![7u32; motions.len()];
    for tool in tools.iter_mut().skip(4) {
        *tool = 9;
    }
    let runs = split_runs(&motions, &tools, 64);
    for (lo, hi) in &runs {
        let tool = tools[*lo];
        assert!(
            tools[*lo..*hi].iter().all(|t| *t == tool),
            "run {lo}..{hi} mixes tools"
        );
    }
    // Contiguous and complete: no motion dropped, none cut twice.
    assert_eq!(runs.first().expect("non-empty").0, 0);
    assert_eq!(runs.last().expect("non-empty").1, motions.len());
    for pair in runs.windows(2) {
        assert_eq!(pair[0].1, pair[1].0, "runs must tile the motion list");
    }
}

#[test]
fn split_runs_never_mixes_exact_with_substepped() {
    let motions = arcs_and_ramps();
    let tools = vec![1u32; motions.len()];
    for (lo, hi) in split_runs(&motions, &tools, 64) {
        let exact = motions[lo].case().is_exact();
        assert!(
            motions[lo..hi].iter().all(|m| m.case().is_exact() == exact),
            "run {lo}..{hi} mixes exact and sub-stepped motions, so its reported \
             deviation bound would not belong to the motions that earned it"
        );
    }
}

#[test]
fn split_runs_respects_the_size_cap() {
    let motions = raster();
    let tools = vec![1u32; motions.len()];
    for size in [1usize, 2, 5, 32] {
        for (lo, hi) in split_runs(&motions, &tools, size) {
            assert!(hi - lo <= size, "run {lo}..{hi} exceeds size {size}");
        }
    }
}
