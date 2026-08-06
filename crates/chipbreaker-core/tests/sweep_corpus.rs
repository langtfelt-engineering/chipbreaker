// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The sweep corpus: ten cut fields pinned by digest.
//!
//! Goldens are digests, as at Units 5 and 6, because a cut `.tdx` field cannot
//! be diffed. What is pinned alongside decides what a failure *means*:
//!
//! - digest moves, volumes hold → a serialization change
//! - digest and volumes both move → a geometry change
//! - sub-steps move → a **dispatch** change: a case that used to take its closed
//!   form has started sub-stepping, or the reverse
//! - spill moves → the arena's growth under cutting has changed
//!
//! That last pair is why this corpus exists at all rather than leaning on the
//! differential tests. Those check that a case is *correct*; only a pinned
//! sub-step count catches a case that is still correct but has quietly stopped
//! being exact.

use std::collections::BTreeMap;
use std::path::PathBuf;

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::LinearMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri, distribution, spilled};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, bull_end_mill, drill, flat_end_mill};
use serde_json::Value;

const SPACING: f64 = 0.5;

fn expectations() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/sweep/expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    assert!(!text.trim().is_empty(), "the sweep corpus is empty");
    serde_json::from_str(&text).expect("valid JSON")
}

fn flat() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}
fn ball() -> Profile {
    ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}
fn bull() -> Profile {
    bull_end_mill(10.0, 2.0, 16.0, &Shank::plain(8.0, 60.0)).expect("valid")
}
fn twist() -> Profile {
    drill(6.0, 118.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
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

fn horizontal(y: f64, z: f64) -> LinearMove {
    LinearMove {
        start: Vec3::new(-5.0, y, z),
        end: Vec3::new(45.0, y, z),
    }
}

fn m(sx: f64, sy: f64, sz: f64, ex: f64, ey: f64, ez: f64) -> LinearMove {
    LinearMove {
        start: Vec3::new(sx, sy, sz),
        end: Vec3::new(ex, ey, ez),
    }
}

/// Rebuilds a case from its id, mirroring the generator.
fn recipe(id: &str) -> (Profile, Vec<LinearMove>) {
    match id {
        "case-a-slot-flat" => (flat(), vec![horizontal(15.0, -1.0)]),
        "case-a-diagonal-ball" => (ball(), vec![m(4.0, 5.0, 4.0, 36.0, 26.0, 4.0)]),
        "case-a-along-x-ray-degenerate" => (flat(), vec![horizontal(15.0, 5.0)]),
        "case-b-plunge-drill" => (twist(), vec![m(20.0, 15.0, 12.0, 20.0, 15.0, 2.0)]),
        "case-b-plunge-necked-shank" => (bull(), vec![m(20.0, 15.0, 14.0, 20.0, 15.0, 1.0)]),
        "case-b-retract" => (
            flat(),
            vec![
                m(20.0, 15.0, 12.0, 20.0, 15.0, 3.0),
                m(20.0, 15.0, 3.0, 20.0, 15.0, 12.0),
            ],
        ),
        "case-c-ramp-entry" => (flat(), vec![m(6.0, 15.0, 10.0, 30.0, 15.0, 5.0)]),
        "case-c-ramp-diagonal" => (ball(), vec![m(5.0, 6.0, 11.0, 34.0, 25.0, 4.0)]),
        "mixed-pocket-rib" => (flat(), vec![horizontal(10.0, -1.0), horizontal(20.0, -1.0)]),
        "mixed-raster-with-plunges" => (
            flat(),
            vec![
                m(8.0, 8.0, 12.0, 8.0, 8.0, 6.0),
                m(8.0, 8.0, 6.0, 32.0, 8.0, 6.0),
                m(32.0, 8.0, 6.0, 32.0, 16.0, 6.0),
                m(32.0, 16.0, 6.0, 8.0, 16.0, 6.0),
                m(8.0, 16.0, 6.0, 8.0, 16.0, 12.0),
            ],
        ),
        other => panic!(
            "the corpus has a case `{other}` this test cannot rebuild. Add it here as \
             well as in the generator, or the golden pins nothing."
        ),
    }
}

/// Runs a case, returning the field and what the cut cost.
fn replay(id: &str) -> (TriDexelField, [f64; 3], u64, f64, u64, u64) {
    let (profile, moves) = recipe(id);
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);
    let before = field.volumes();
    let (mut substeps, mut bound, mut tested, mut rejected) = (0u64, 0.0f64, 0u64, 0u64);
    for motion in &moves {
        let s = cut_tri(
            &mut field,
            &profile,
            motion,
            SweepMethod::Analytic {
                tolerance: SPACING / 10.0,
            },
            &mut scratch,
        );
        substeps += s.substeps;
        bound = bound.max(s.worst_bound_mm);
        tested += s.rays_tested;
        rejected += s.rays_rejected;
    }
    let after = field.volumes();
    let removed = [
        before[0].unwrap_or(0.0) - after[0].unwrap_or(0.0),
        before[1].unwrap_or(0.0) - after[1].unwrap_or(0.0),
        before[2].unwrap_or(0.0) - after[2].unwrap_or(0.0),
    ];
    (field, removed, substeps, bound, tested, rejected)
}

#[test]
fn every_corpus_cut_matches_its_golden() {
    let data = expectations();
    let cases = data["cases"].as_array().expect("cases");
    assert!(cases.len() >= 10, "the corpus has shrunk: {}", cases.len());

    for case in cases {
        let id = case["id"].as_str().expect("id");
        let (field, removed, substeps, bound, tested, rejected) = replay(id);

        let mut h = CanonicalHash::new();
        h.add(&field);
        assert_eq!(
            h.finish().to_hex(),
            case["digest"].as_str().expect("digest"),
            "{id}: the field digest moved. Check removed volume and sub-steps below \
             to tell a geometry change from a serialization or dispatch one."
        );

        // Removed volume per bundle, on the BITS. Not an agreement assertion
        // between bundles -- ADR 0005 forbids that -- but each must reproduce
        // its own number exactly.
        let golden = case["removed_mm3"].as_array().expect("removed");
        for ((axis, value), expected) in AXES.iter().zip(removed).zip(golden) {
            let expected = expected.as_f64().expect("a number");
            assert_eq!(
                value.to_bits(),
                expected.to_bits(),
                "{id}/{}: removed {value} mm^3 against golden {expected}",
                axis.as_str()
            );
        }

        assert_eq!(
            substeps,
            case["substeps"].as_u64().expect("substeps"),
            "{id}: the sub-step count moved. A case that used to take its closed form \
             has started sub-stepping, or the reverse -- either way the dispatch in \
             `cut_bundle` has changed."
        );
        let expected_bound = case["worst_bound_mm"].as_f64().expect("bound");
        assert_eq!(bound.to_bits(), expected_bound.to_bits(), "{id}: bound");
        assert_eq!(
            tested,
            case["rays_tested"].as_u64().expect("tested"),
            "{id}: rays tested"
        );
        assert_eq!(
            rejected,
            case["rays_rejected"].as_u64().expect("rejected"),
            "{id}: rays rejected -- the box rejection has changed what it skips"
        );
    }
}

#[test]
fn the_closed_form_cases_take_no_sub_steps_at_all() {
    // The property that makes Cases A and B worth having. If one of these ever
    // reports a sub-step, it has silently fallen through to the bounded path and
    // is merely accurate rather than exact.
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let kinds: Vec<&str> = case["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .map(|k| k.as_str().expect("a name"))
            .collect();
        if kinds.iter().any(|k| *k == "ramp") {
            continue;
        }
        let (_, _, substeps, bound, _, _) = replay(id);
        assert_eq!(
            substeps, 0,
            "{id} is {kinds:?} and must be exact, but took {substeps} sub-steps"
        );
        assert_eq!(bound, 0.0, "{id}: an exact case has no deviation bound");
    }
}

#[test]
fn the_ramps_report_a_bound_beside_their_step_count() {
    // The other half: a sub-stepped case must never report a count without the
    // bound it achieved, and that bound must be under what was asked for.
    let data = expectations();
    let mut ramps = 0;
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let kinds: Vec<&str> = case["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .map(|k| k.as_str().expect("a name"))
            .collect();
        if !kinds.iter().any(|k| *k == "ramp") {
            continue;
        }
        ramps += 1;
        let (_, _, substeps, bound, _, _) = replay(id);
        assert!(substeps > 0, "{id}: a ramp must sub-step");
        assert!(bound > 0.0, "{id}: a sub-stepped case must report a bound");
        assert!(
            bound <= SPACING / 10.0,
            "{id}: bound {bound} exceeds the requested {}",
            SPACING / 10.0
        );
    }
    assert!(ramps >= 2, "the corpus should exercise more than one ramp");
}

#[test]
fn the_corpus_records_what_cutting_does_to_the_arena() {
    // Unit 7 rebuilt the spill path on the evidence that cutting makes rays
    // split and that spill is per bundle rather than per ray. Pinning the
    // distribution means a change to that behaviour cannot pass unnoticed.
    let data = expectations();
    let mut saw_spill = false;
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let (field, ..) = replay(id);

        let golden: BTreeMap<usize, usize> = case["span_distribution"]
            .as_array()
            .expect("distribution")
            .iter()
            .map(|pair| {
                let p = pair.as_array().expect("pair");
                (
                    usize::try_from(p[0].as_u64().expect("spans")).expect("small"),
                    usize::try_from(p[1].as_u64().expect("rays")).expect("small"),
                )
            })
            .collect();
        assert_eq!(distribution(&field), golden, "{id}: span distribution");
        assert_eq!(
            spilled(&field) as u64,
            case["spilled_rays"].as_u64().expect("spilled"),
            "{id}: spilled rays"
        );
        if spilled(&field) > 0 {
            saw_spill = true;
        }
    }
    assert!(
        saw_spill,
        "no corpus case spills, so the rebuilt spill path is untested here. The rib \
         case exists to reach it."
    );
}

#[test]
fn the_corpus_covers_the_cases_that_matter() {
    let data = expectations();
    let ids: Vec<&str> = data["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .map(|c| c["id"].as_str().expect("id"))
        .collect();
    for required in [
        // The three-piece decomposition, on and off the axes.
        "case-a-slot-flat",
        "case-a-diagonal-ball",
        // The degenerate ray an X bundle hits on every ray of an X move.
        "case-a-along-x-ray-degenerate",
        // The moving maximum, on the tool that separates it from a translation.
        "case-b-plunge-necked-shank",
        // Bounded sub-stepping, with its bound.
        "case-c-ramp-entry",
        // The geometry that rebuilt the arena.
        "mixed-pocket-rib",
    ] {
        assert!(
            ids.contains(&required),
            "the corpus has lost `{required}`, which one of this unit's findings rests on"
        );
    }
}
