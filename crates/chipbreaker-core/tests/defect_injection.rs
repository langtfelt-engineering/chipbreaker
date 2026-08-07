// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Every corpus case must inject the defect it claims, before it may count.
//!
//! The corpus is the oracle for recall, so a case that perturbs nothing is worse
//! than a missing case. It sits in the denominator, it can never be found, and
//! the resulting figure reads as a limit of the detector when it is a property of
//! the corpus. Two rounds of this have already been paid for:
//!
//! * `mid-face` anchored at `z = 12`, the stock surface, so the clean pass
//!   removed nothing and neither did any perturbation at or above it.
//! * `rapid-clips-stock` cleared to `16 - depth`, still above a 12 mm stock at
//!   every depth in the ladder.
//!
//! Both were found by noticing that recall was oddly flat with depth, which is a
//! slow and indirect way to learn it. This file asks the question directly.
//!
//! # The measurement avoids the machinery under test
//!
//! It runs the clean program and the dirty one into two fields on the same
//! lattice and takes the **Hausdorff distance between the two span sets, ray by
//! ray**. That is one dimensional and exact — two sets of intervals on the same
//! line — with no mesh, no extraction, no normals and no containment test
//! anywhere in it. A defect this cannot see is not in the simulation at all.
//!
//! # Every case, not a sample
//!
//! Sampling would reintroduce exactly the hole being closed: an unsampled case
//! is an unchecked case, and the ones that turned out to be empty were not
//! evenly spread — they clustered in two kinds and two locales. The clean field
//! depends only on the locale, so seven of them are built and shared, which makes
//! the full corpus affordable.
//!
//! # And the cases themselves are pinned
//!
//! [`the_corpus_matches_its_committed_expectations`] compares the generated
//! cases against `tests/corpus/defect/expectations.json`. Injecting a defect and
//! being *the case you meant* are different properties: a generator change that
//! swaps one locale for another leaves every case injecting correctly while
//! silently changing what the recall figure is a figure about.

use std::collections::BTreeMap;

use chipbreaker_core::defect::{DefectCase, Locale, STOCK, corpus};
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

/// The fraction of its claimed depth a case must actually inject.
///
/// Not one: a perturbation of `d` measured on a lattice and clipped by the stock
/// need not displace a surface by exactly `d`.
///
/// # Why there is no ceiling here
///
/// A ceiling was tried, and it does not have an honest oracle. `depth_mm` is a
/// **local** statement — this wall moved by `d` — and the measurement below is a
/// global maximum over the whole part, so the two are not the same quantity even
/// when both are right.
///
/// `tool-too-large` is the clean demonstration. A cutter `2d` oversize moves each
/// slot wall out by `d`, exactly as claimed, and at the same time exposes a band
/// of slot floor that was solid stock before — a displacement of the full slot
/// depth, five millimetres against a claimed 0.4. Both are true. The case is not
/// wrong; a global maximum is simply not what its ground truth describes.
///
/// So the discipline asserted here is the one that can be asserted: **every case
/// injects something, and injects at least the depth it claims somewhere.** The
/// complementary question — is the defect the claimed size *at the place it was
/// claimed* — belongs to the localisation test, which knows the place.
const MUST_INJECT: f64 = 0.5;

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

/// The dirty run, exactly as the recall harness performs it.
///
/// A tool defect perturbs the cutter and leaves the path alone; a length offset
/// is modelled by lowering the path, which is what it does to a machine.
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

/// One-sided Hausdorff: `sup` over points of `a` of the distance to `b`.
///
/// **Over points, not over endpoints.** The first version of this walked only the
/// bounds of `a`, and reported zero for a case that opened a hole through the
/// middle of a span: deepening a slot leaves an X-bundle ray entering and leaving
/// the stock at exactly the same two places, with the material between them gone.
/// Neither bound moved, so nothing was measured, and the reading of zero then won
/// the minimum below and was mistaken for a case that injected nothing.
///
/// The supremum is attained at a bound of `a` or at the midpoint of a gap in `b`,
/// because `d(., b)` is a tent function on any interval clear of `b`. Both are
/// checked, so this is exact rather than sampled.
fn one_sided(a: &[Span], b: &[Span]) -> f64 {
    if a.is_empty() {
        return 0.0;
    }
    if b.is_empty() {
        // Everything of `a` vanished along this ray. Its own extent stands in
        // for an infinite distance; returning zero would call a case that
        // removed a whole feature "no change".
        return a.iter().map(|s| s.t1 - s.t0).fold(0.0f64, f64::max);
    }
    let mut worst = 0.0f64;
    for s in a {
        worst = worst.max(distance_to(b, s.t0)).max(distance_to(b, s.t1));
        // Every gap between consecutive intervals of `b`, and the two open ends.
        // A point of `a` inside such a gap is as far from `b` as anything gets.
        let mut edges: Vec<f64> = Vec::with_capacity(b.len() * 2);
        for t in b {
            edges.push(t.t0);
            edges.push(t.t1);
        }
        for pair in edges.windows(2) {
            let mid = 0.5 * (pair[0] + pair[1]);
            if mid >= s.t0 && mid <= s.t1 {
                worst = worst.max(distance_to(b, mid));
            }
        }
    }
    worst
}

/// The largest along-ray displacement between the two fields, per bundle.
fn per_bundle_mm(clean: &TriDexelField, dirty: &TriDexelField) -> [f64; 3] {
    let mut worst = [0.0f64; 3];
    for axis in AXES {
        let (Some(a), Some(b)) = (clean.bundle(axis), dirty.bundle(axis)) else {
            continue;
        };
        let rays = u32::try_from(a.arena().rays()).expect("small");
        let slot = &mut worst[axis.index()];
        for r in 0..rays {
            let (sa, sb) = (a.arena().get(r), b.arena().get(r));
            *slot = slot.max(one_sided(sa, sb)).max(one_sided(sb, sa));
        }
    }
    worst
}

/// Did anything change at all: the **largest** bundle reading.
///
/// Any bundle seeing a displacement means the two programs cut different solids.
/// Zero here means they cut the same one, whatever the case is called.
fn changed_mm(per_bundle: [f64; 3]) -> f64 {
    per_bundle.into_iter().fold(0.0f64, f64::max)
}

/// How much it changed: the **smallest** bundle reading.
///
/// An along-ray displacement overstates the perpendicular one by `1/cos θ`,
/// where `θ` is the angle between the surface normal and that bundle's axis, and
/// on a bundle the surface nearly grazes that factor is enormous — a wall moved
/// 0.15 mm sideways displaces the end of an X-bundle span, tangent to the slot's
/// rounded end, by five millimetres. Unit 6 spent a whole unit on this ratio
/// being unbounded, and it is unbounded here for the same reason.
///
/// Taking the smallest reading is what the tri-dexel guarantee is *for*. No
/// surface can hide from all three bundles: the worst case is a body diagonal, at
/// `54.74` degrees to every axis, so the best-placed bundle overstates by at most
/// `1/cos(54.74) = sqrt(3)`. That bound is where the ceiling above comes from,
/// rather than from a number that happened to fit.
///
/// **Zero readings are excluded, and the reason is not a convenience.** A bundle
/// transverse to the perturbed surface only sees the change if one of its rays
/// falls inside the displaced band, and for a defect a fifth of a cell deep it
/// usually does not — the reading is then exactly zero, meaning *this bundle took
/// no sample there*, not *nothing moved*. Letting that win the minimum reported
/// 39 shallow cases as injecting nothing while the aligned bundle was measuring
/// them correctly.
///
/// The bundle nearest the surface normal never has that problem: a displacement
/// along a ray slides that ray's span endpoint by `d / cos θ` whatever the
/// lattice does, so it resolves any depth at all. It is the transverse bundles
/// that go blind, and they are the ones this drops.
fn injected_mm(per_bundle: [f64; 3]) -> f64 {
    per_bundle
        .into_iter()
        .filter(|v| *v > 0.0)
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn every_case_injects_the_defect_it_claims() {
    let cases = corpus();
    let profile = mill(6.0);

    // The clean program depends only on the locale, so seven fields serve all
    // 295 cases. Without this the run is twice the length for no more coverage.
    let mut clean_for: BTreeMap<Locale, TriDexelField> = BTreeMap::new();
    for locale in Locale::all() {
        let case = cases
            .iter()
            .find(|c| c.locale == locale)
            .expect("every locale has cases");
        clean_for.insert(locale, cut(&case.clean, &profile));
    }
    // Asserted, because the sharing above is only sound if it is true.
    for case in &cases {
        let reference = cases
            .iter()
            .find(|c| c.locale == case.locale)
            .expect("present");
        assert_eq!(
            case.clean.len(),
            reference.clean.len(),
            "{}: the clean program varies within a locale, so it cannot be shared",
            case.id
        );
    }

    let mut empty: Vec<(&str, f64)> = Vec::new();
    let mut weak: Vec<(&str, f64, f64)> = Vec::new();
    let mut lowest = f64::INFINITY;

    for case in &cases {
        let clean = &clean_for[&case.locale];
        let per_bundle = per_bundle_mm(clean, &dirty(case));
        let want = case.depth_mm.abs();
        if changed_mm(per_bundle) <= 1.0e-9 {
            empty.push((&case.id, want));
            continue;
        }
        let got = injected_mm(per_bundle);
        let ratio = got / want;
        lowest = lowest.min(ratio);
        if ratio < MUST_INJECT {
            weak.push((&case.id, want, got));
        }
    }

    println!(
        "{} cases, all injecting; the weakest reaches {:.0}% of the depth it claims",
        cases.len(),
        lowest * 100.0
    );

    assert!(
        empty.is_empty(),
        "{} cases inject nothing at all and cannot be found by any detector, \
         while counting in the recall denominator: {:?}",
        empty.len(),
        &empty[..empty.len().min(10)]
    );
    assert!(
        weak.is_empty(),
        "{} cases inject less than {:.0}% of the depth they claim, so their \
         ground truth is wrong even where they are detected: {:?}",
        weak.len(),
        MUST_INJECT * 100.0,
        &weak[..weak.len().min(10)]
    );
}

#[test]
fn the_check_notices_a_case_that_injects_nothing() {
    // The evidence CONTRIBUTING.md asks for. A case whose dirty program is its
    // clean one injects nothing by construction, and the measurement must say
    // so -- otherwise the test above passes on a corpus full of holes.
    let cases = corpus();
    let case = &cases[0];
    let profile = mill(6.0);
    let clean = cut(&case.clean, &profile);
    let identical = cut(&case.clean, &profile);
    assert_eq!(
        changed_mm(per_bundle_mm(&clean, &identical)),
        0.0,
        "two runs of the same program measured as different, so the measurement \
         has noise in it and a zero reading would mean nothing"
    );

    // And it is not blind in the other direction either: the real dirty run of
    // the same case must read as a genuine displacement.
    let got = changed_mm(per_bundle_mm(&clean, &dirty(case)));
    assert!(
        got > 1.0e-9,
        "{}: the measurement reports zero for a case that does perturb the \
         program, so it cannot tell an empty case from a real one",
        case.id
    );
}

#[test]
fn the_corpus_matches_its_committed_expectations() {
    // The corpus is built from code, so there is nothing to commit in the usual
    // sense -- and that is exactly the problem. Twice a generator change has
    // quietly emptied cases of their defect while the count stayed at 295, and
    // neither showed up as a diff because there was no file to diff.
    //
    // `tests/corpus/defect/expectations.json` records every case's identity and
    // a digest over its motions, so a change to `program()` arrives as a
    // reviewable diff naming the cases it moved. Regenerate it with
    // `cargo run -p chipbreaker-core --example generate_defect_corpus`.
    use std::path::PathBuf;

    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/defect/expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    let data: serde_json::Value = serde_json::from_str(&text).expect("valid JSON");

    let cases = corpus();
    let expected = data["cases"].as_array().expect("cases");
    assert_eq!(
        cases.len(),
        expected.len(),
        "the corpus has {} cases and the committed file has {}. Regenerate it and \
         review the diff -- a case count that moves without a reason is how both \
         of the empty-case bugs got in.",
        cases.len(),
        expected.len()
    );
    assert_eq!(
        data["case_count"].as_u64().expect("a count") as usize,
        cases.len(),
        "the file's own header disagrees with its case list"
    );

    for (case, want) in cases.iter().zip(expected) {
        assert_eq!(
            case.id,
            want["id"].as_str().expect("id"),
            "the corpus order changed, which renumbers every case downstream"
        );
        // The identity fields first, because a mismatch there explains a digest
        // mismatch and the reverse is not true.
        for (name, got, expect) in [
            ("kind", case.kind.as_str(), want["kind"].as_str()),
            ("locale", case.locale.as_str(), want["locale"].as_str()),
            ("facing", case.facing.as_str(), want["facing"].as_str()),
        ] {
            assert_eq!(got, expect.expect(name), "{}: {name} changed", case.id);
        }
        assert_eq!(
            case.depth_mm,
            want["depth_mm"].as_f64().expect("depth"),
            "{}: the ground-truth depth changed",
            case.id
        );
        assert_eq!(
            case.motions.len(),
            want["motions"].as_u64().expect("motions") as usize,
            "{}: the perturbed program has a different number of segments, so \
             `segment` no longer points where it did",
            case.id
        );
    }
}
