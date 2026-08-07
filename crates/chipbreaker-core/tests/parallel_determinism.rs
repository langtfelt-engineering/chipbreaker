// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! **Identical output at every thread count, on every schedule.**
//!
//! This is the product's core differentiator and no incumbent publishes it. A
//! single scheduling-dependent bit ends the claim, so these tests compare bit
//! patterns and digests, never tolerances.
//!
//! # Thread count alone is a weak perturbation
//!
//! On a machine with spare cores, 4 workers and 8 workers may produce nearly the
//! same interleaving, and both pass while a real ordering dependency sits
//! undetected. So the primary test is an **adversarial schedule**: chunk order
//! shuffled from a seed, chunk sizes varied, workers oversubscribed past the
//! core count so the OS interleaves aggressively. Thread count is the sanity
//! check underneath it.
//!
//! # The sequential path is the reference, always
//!
//! `batch::cut_all` is the Unit 7 to 9 code and remains the definition of the
//! right answer. Any divergence is a bug in the parallel version and never a new
//! baseline.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, CutStats, SweepMethod};
use chipbreaker_core::sweep::parallel::{Schedule, cut_all_parallel};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

const SPACING: f64 = 0.5;
const METHOD: SweepMethod = SweepMethod::Analytic {
    tolerance: SPACING / 10.0,
};

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 12.0)),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill() -> Profile {
    flat_end_mill(6.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn ball() -> Profile {
    ball_end_mill(6.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

/// A raster with plunges and retracts: work spread evenly across the field.
fn balanced() -> Vec<Motion> {
    let mut out = Vec::new();
    let mut y = 6.0;
    let mut left = true;
    while y <= 24.0 {
        let (a, b) = if left { (6.0, 34.0) } else { (34.0, 6.0) };
        out.push(line([a, y, 14.0], [a, y, 8.0]));
        out.push(line([a, y, 8.0], [b, y, 8.0]));
        out.push(line([b, y, 8.0], [b, y, 14.0]));
        y += 2.0;
        left = !left;
    }
    out
}

/// A deep pocket in one corner: work clustered into a few chunks, which is where
/// static partitioning falls over and stealing earns its place.
fn clustered() -> Vec<Motion> {
    let mut out = Vec::new();
    let mut z = 11.0;
    while z >= 3.0 {
        let mut y = 6.0;
        while y <= 12.0 {
            out.push(line([6.0, y, z], [14.0, y, z]));
            y += 1.0;
        }
        z -= 1.0;
    }
    out
}

/// Arcs and a helix, so the sub-stepped paths are exercised too.
fn with_arcs() -> Vec<Motion> {
    vec![
        line([20.0, 15.0, 14.0], [20.0, 15.0, 8.0]),
        Motion::Arc(ArcMove {
            center: Vec3::new(20.0, 15.0, 0.0),
            radius: 8.0,
            start_angle: 0.0,
            sweep: 2.0 * core::f64::consts::PI,
            z: 8.0,
            plane: ArcPlane::Xy,
            rise: 0.0,
        }),
        Motion::Arc(ArcMove {
            center: Vec3::new(14.0, 10.0, 0.0),
            radius: 5.0,
            start_angle: 0.0,
            sweep: 2.0 * core::f64::consts::PI,
            z: 12.0,
            plane: ArcPlane::Xy,
            rise: -5.0,
        }),
        line([12.0, 20.0, 10.0], [30.0, 24.0, 6.0]),
    ]
}

/// The sequential reference: the Unit 7 to 9 code, unchanged.
fn reference(profile: &Profile, motions: &[Motion]) -> (String, CutStats) {
    let mut field = stock();
    let mut scratch = CutScratch::new(profile);
    let stats = cut_all(
        &mut field,
        profile,
        motions,
        METHOD,
        &mut scratch,
        DEFAULT_BATCH,
    );
    (digest(&field), stats)
}

fn parallel(profile: &Profile, motions: &[Motion], schedule: Schedule) -> (String, CutStats) {
    let mut field = stock();
    let stats = cut_all_parallel(
        &mut field,
        profile,
        motions,
        METHOD,
        DEFAULT_BATCH,
        schedule,
    );
    (digest(&field), stats)
}

fn assert_same(label: &str, want: &(String, CutStats), got: &(String, CutStats)) {
    assert_eq!(
        want.0, got.0,
        "{label}: the FIELD differs from the sequential reference. Scheduling \
         reached the geometry, which ends the determinism claim outright."
    );
    for slot in 0..3 {
        assert_eq!(
            want.1.removed_mm3[slot].to_bits(),
            got.1.removed_mm3[slot].to_bits(),
            "{label}: bundle {slot} removed volume differs by {} mm3. The field \
             matched, so this is a reordered float sum rather than a geometry \
             change -- the reduction is not replaying the sequential order.",
            got.1.removed_mm3[slot] - want.1.removed_mm3[slot]
        );
    }
    assert_eq!(
        want.1.rays_tested, got.1.rays_tested,
        "{label}: rays tested"
    );
    assert_eq!(
        want.1.rays_rejected, got.1.rays_rejected,
        "{label}: rays rejected"
    );
    assert_eq!(
        want.1.rays_changed, got.1.rays_changed,
        "{label}: rays changed"
    );
    assert_eq!(want.1.substeps, got.1.substeps, "{label}: substeps");
    assert_eq!(want.1.rays_exact, got.1.rays_exact, "{label}: rays exact");
    assert_eq!(
        want.1.rays_substepped, got.1.rays_substepped,
        "{label}: rays substepped"
    );
    assert_eq!(
        want.1.worst_bound_mm.to_bits(),
        got.1.worst_bound_mm.to_bits(),
        "{label}: worst bound"
    );
    assert_eq!(want.1.raycast, got.1.raycast, "{label}: predicate counters");
}

#[test]
fn every_thread_count_matches_the_sequential_reference() {
    for (name, profile, motions) in [
        ("balanced", mill(), balanced()),
        ("clustered", mill(), clustered()),
        ("arcs", ball(), with_arcs()),
    ] {
        let want = reference(&profile, &motions);
        // 24 is past the core count on most machines, so the OS is forced to
        // interleave rather than simply running each worker to completion.
        for threads in [1usize, 2, 3, 4, 8, 16, 24] {
            let got = parallel(
                &profile,
                &motions,
                Schedule {
                    threads,
                    ..Schedule::default()
                },
            );
            assert_same(&format!("{name} at {threads} threads"), &want, &got);
        }
    }
}

#[test]
fn every_chunk_size_matches_the_sequential_reference() {
    // Chunk size changes which rays share a worker and therefore the grouping a
    // naive reduction would produce. It must be invisible.
    let (profile, motions) = (mill(), balanced());
    let want = reference(&profile, &motions);
    for chunk in [1usize, 7, 64, 256, 4096, 1_000_000] {
        let got = parallel(
            &profile,
            &motions,
            Schedule {
                threads: 4,
                chunk,
                chaos: None,
            },
        );
        assert_same(&format!("chunk {chunk}"), &want, &got);
    }
}

#[test]
fn adversarial_schedules_match_the_sequential_reference() {
    // **The test that actually defends the claim.** Varying the thread count
    // perturbs the schedule weakly; shuffling the chunk order from a seed
    // perturbs it as hard as the design allows, because chunk order is exactly
    // what a work-stealing queue is free to change.
    let cases = [
        ("balanced", mill(), balanced()),
        ("clustered", mill(), clustered()),
        ("arcs", ball(), with_arcs()),
    ];
    let mut configurations = 0usize;
    for (name, profile, motions) in cases {
        let want = reference(&profile, &motions);
        for seed in 0..24u64 {
            // Vary the chunk size with the seed as well, so no two runs share a
            // partition either.
            let chunk = [3usize, 17, 64, 333][(seed % 4) as usize];
            let threads = [2usize, 3, 5, 8][((seed / 4) % 4) as usize];
            let got = parallel(
                &profile,
                &motions,
                Schedule {
                    threads,
                    chunk,
                    chaos: Some(seed),
                },
            );
            assert_same(
                &format!("{name} seed {seed} chunk {chunk} threads {threads}"),
                &want,
                &got,
            );
            configurations += 1;
        }
    }
    assert!(
        configurations >= 72,
        "only {configurations} adversarial configurations ran"
    );
}

#[test]
fn batch_size_is_invisible_to_the_parallel_path() {
    // The parallel path raises the batch size for scheduling reasons, which is
    // only safe because batch size cannot reach the answer. Checked directly
    // rather than inherited from Unit 8, since the reduction here is a different
    // piece of code doing the same job.
    let (profile, motions) = (mill(), balanced());
    let want = reference(&profile, &motions);
    for batch in [1usize, 7, 32, 512, 100_000] {
        let mut field = stock();
        let stats = cut_all_parallel(
            &mut field,
            &profile,
            &motions,
            METHOD,
            batch,
            Schedule {
                threads: 4,
                ..Schedule::default()
            },
        );
        assert_same(&format!("batch {batch}"), &want, &(digest(&field), stats));
    }
}

#[test]
fn repeated_parallel_runs_agree_with_each_other() {
    // Determinism across runs of the *same* configuration, which catches a
    // dependence on timing rather than on partition.
    let (profile, motions) = (mill(), clustered());
    let schedule = Schedule {
        threads: 8,
        chunk: 31,
        chaos: None,
    };
    let first = parallel(&profile, &motions, schedule);
    for round in 0..12 {
        let again = parallel(&profile, &motions, schedule);
        assert_same(&format!("round {round}"), &first, &again);
    }
}

#[test]
fn the_chaos_shuffle_really_reorders() {
    // A guard on the guard. If `chaos_order` returned the identity, every
    // adversarial test above would pass while testing nothing.
    use chipbreaker_core::sweep::parallel::chaos_order;
    let plain = chaos_order(64, None);
    assert_eq!(plain, (0..64).collect::<Vec<_>>());
    let mut differed = 0;
    for seed in 0..8u64 {
        let shuffled = chaos_order(64, Some(seed));
        let mut sorted = shuffled.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, plain, "seed {seed} lost or duplicated a chunk");
        if shuffled != plain {
            differed += 1;
        }
        // And reproducible from the seed.
        assert_eq!(shuffled, chaos_order(64, Some(seed)));
    }
    assert_eq!(differed, 8, "some seeds produced the identity order");
}

#[test]
fn an_empty_motion_list_is_a_no_op() {
    let profile = mill();
    let before = digest(&stock());
    let mut field = stock();
    let stats = cut_all_parallel(
        &mut field,
        &profile,
        &[],
        METHOD,
        DEFAULT_BATCH,
        Schedule::default(),
    );
    assert_eq!(digest(&field), before);
    assert_eq!(stats, CutStats::default());
}
