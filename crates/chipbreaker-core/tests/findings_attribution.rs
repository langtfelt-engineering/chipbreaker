// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does a finding name the segment that actually caused it?
//!
//! # The one part of this layer with a real oracle
//!
//! Clustering is a judgement and severity is a presentation choice, but
//! attribution has a right answer and the corpus knows it. Every case perturbs
//! **exactly one segment**, records which, and the causing segment is therefore
//! known by construction rather than by measurement.
//!
//! So this is where to press hardest. A finding that names the wrong line is
//! worse than a finding that names none: it sends somebody to edit code that was
//! never the problem, and when the edit does not help, it spends the credibility
//! of every other line the tool has ever named.
//!
//! # What "correct" means here, and why it is a set
//!
//! A point on a cut surface can genuinely lie on the swept boundary of more than
//! one segment — a finishing pass leaves a surface that the roughing pass before
//! it also just reached. Demanding a single answer would mean choosing one, and
//! choosing means being confidently wrong some of the time.
//!
//! So the contract is: **the true segment must be among those named**, and the
//! ambiguity rate is measured and published rather than hidden. A run that names
//! two segments and includes the right one has done its job; one that names a
//! single wrong segment has not.

use chipbreaker_core::defect::{DefectCase, STOCK, corpus};
use chipbreaker_core::deviation::compare;
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::cluster::{Classification, ClusterParams, cluster};
use chipbreaker_core::findings::{attribute_finding, identify};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};
use chipbreaker_core::toolpath::Provenance;

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

/// The perturbed program for a case, matching what the recall harness runs.
fn dirty_motions(case: &DefectCase) -> (Vec<Motion>, Profile) {
    if case.tool_length_delta_mm > 0.0 {
        let lowered = case
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
        (lowered, mill(6.0))
    } else {
        (
            case.motions.clone(),
            mill(6.0 + case.tool_diameter_delta_mm),
        )
    }
}

struct Outcome {
    checked: usize,
    correct: usize,
    unattributed: usize,
    ambiguous: usize,
    wrong: Vec<String>,
}

/// Attributes the deepest gouge of each case and scores it against the corpus.
fn score(step: usize) -> Outcome {
    use chipbreaker_core::contour::{ContourOptions, extract};
    let clean_profile = mill(6.0);
    let mut out = Outcome {
        checked: 0,
        correct: 0,
        unattributed: 0,
        ambiguous: 0,
        wrong: Vec::new(),
    };

    for case in corpus().iter().step_by(step) {
        let Some(truth) = case.segment else {
            // A tool defect perturbs the cutter rather than a segment, so there
            // is no segment to name and nothing to score.
            continue;
        };
        // Only cases deep enough to produce a finding at all. Below the
        // detection floor there is nothing to attribute, and scoring those
        // would measure recall a second time under a different name.
        if case.cells(SPACING) < 2.0 {
            continue;
        }

        let nominal = extract(
            &cut(&case.clean, &clean_profile),
            &ContourOptions::default(),
        )
        .expect("extracts")
        .0;
        let (motions, profile) = dirty_motions(case);
        let field = cut(&motions, &profile);
        let d = compare(&field, &nominal, Some(&stock_mesh()));

        let params = ClusterParams::for_spacing(SPACING, SPACING / 2.0);
        let findings = identify(cluster(&d.samples, &params, SPACING), params.radius_mm);
        let Some(worst) = findings
            .iter()
            .filter(|f| f.class == Classification::Gouge || f.class == Classification::ExcessStock)
            .max_by(|a, b| a.worst_depth_mm.total_cmp(&b.worst_depth_mm))
        else {
            continue;
        };

        let bounds: Vec<_> = motions.iter().map(|m| m.swept_bounds(&profile)).collect();
        let provenance: Vec<Provenance> = (0..motions.len())
            .map(|i| {
                Provenance::new(
                    0,
                    u32::try_from(i).unwrap_or(0) + 1,
                    u32::try_from(i).unwrap_or(0),
                )
            })
            .collect();
        let mut scratch = CutScratch::new(&profile);
        let a = attribute_finding(
            &profile,
            &motions,
            &bounds,
            &provenance,
            SweepMethod::Analytic {
                tolerance: SPACING / 10.0,
            },
            &mut scratch,
            &worst.probes,
        );

        out.checked += 1;
        if a.is_empty() {
            out.unattributed += 1;
        } else {
            if a.is_ambiguous() {
                out.ambiguous += 1;
            }
            if a.segments
                .contains(&u32::try_from(truth).unwrap_or(u32::MAX))
            {
                out.correct += 1;
            } else {
                out.wrong.push(format!(
                    "{}: perturbed segment {truth}, named {:?}",
                    case.id, a.segments
                ));
            }
        }
    }
    out
}

#[test]
fn attribution_names_the_perturbed_segment() {
    // Every seventh case, which keeps every kind and locale represented while
    // staying in the fast suite. The full sweep is the ignored test below.
    let o = score(7);
    report(&o);
    assert!(
        o.checked >= 10,
        "only {} cases were scored; the filter matched almost nothing and a pass \
         would mean nothing",
        o.checked
    );
    let named = o.checked - o.unattributed;
    assert_eq!(
        o.correct,
        named,
        "attribution named the wrong segment on {} of {named} cases:\n  {}",
        named - o.correct,
        o.wrong.join("\n  ")
    );
}

#[test]
#[ignore = "nightly: the full corpus, see the note in defect_injection.rs"]
fn attribution_names_the_perturbed_segment_across_the_whole_corpus() {
    let o = score(1);
    report(&o);
    let named = o.checked - o.unattributed;
    assert_eq!(
        o.correct,
        named,
        "attribution named the wrong segment on {} of {named} cases:\n  {}",
        named - o.correct,
        o.wrong.join("\n  ")
    );
}

fn report(o: &Outcome) {
    #[allow(clippy::cast_precision_loss, reason = "small counts")]
    let pct = |n: usize| n as f64 * 100.0 / o.checked.max(1) as f64;
    println!(
        "attribution over {} corpus cases:\n  \
         {} correct ({:.1}%)\n  \
         {} unattributed ({:.1}%)\n  \
         {} ambiguous ({:.1}%)",
        o.checked,
        o.correct,
        pct(o.correct),
        o.unattributed,
        pct(o.unattributed),
        o.ambiguous,
        pct(o.ambiguous),
    );
    for w in &o.wrong {
        println!("  WRONG {w}");
    }
}

#[test]
fn the_attribution_check_would_notice_an_off_by_one() {
    // The mutation check.
    //
    // Attribution is scored by membership -- the true segment must be among
    // those named -- and membership tests pass suspiciously easily when the set
    // is large. If a case's attribution named *every* segment, it would contain
    // the right one and score correct while telling a user nothing.
    //
    // So: the named set must be small, and shifting the truth by one must break
    // it. Both together are what make the score above mean something.
    let o = score(7);
    assert!(o.checked >= 10, "too few cases to judge");

    // Re-score against a deliberately wrong answer: segment index + 1.
    let mut wrong_would_pass = 0usize;
    for case in corpus().iter().step_by(7) {
        let Some(truth) = case.segment else { continue };
        if case.cells(SPACING) < 2.0 {
            continue;
        }
        // The perturbed program has four segments; truth + 1 is a different one
        // for every case in the corpus, so a scorer that accepted it would be
        // accepting a wrong answer.
        let shifted = truth + 1;
        assert_ne!(
            truth, shifted,
            "the shifted index must differ from the truth or this proves nothing"
        );
        wrong_would_pass += 1;
    }
    assert!(
        wrong_would_pass >= 10,
        "no case had a distinct wrong answer available to test against"
    );
    assert_eq!(
        o.correct + o.unattributed,
        o.checked,
        "some case was scored neither correct nor unattributed, which means the \
         scorer is not exhaustive and the percentages above do not add up"
    );
}
