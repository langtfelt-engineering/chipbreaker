// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The crash corpus: zero missed collisions, and no invented ones.
//!
//! # Why the two numbers are not treated alike
//!
//! A **missed** collision is a spindle. There is no partial credit and no
//! acceptable rate, so it is asserted at zero.
//!
//! An **invented** one is measured and printed rather than merely asserted away,
//! because the easy cure for false positives is to make the check less sensitive
//! — and that trade, made quietly to get a test green, is exactly how a checker
//! stops finding real crashes. Printing the rate means a change that buys
//! quietness with blindness is visible in the output of the test that permitted
//! it.
//!
//! Both halves of the corpus are needed for either number to mean anything. A
//! checker that reported every move would score zero misses; one that reported
//! nothing would score zero false positives. Only the pair constrains it.

use chipbreaker_core::crash::{CrashCase, CrashKind, corpus};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::Collision;
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::tool::profile::ElementRole;
use chipbreaker_core::toolpath::{MotionKind, Provenance};

const SPACING: f64 = 0.8;

fn stock_field() -> TriDexelField {
    let mesh = shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(
            chipbreaker_core::crash::STOCK[0],
            chipbreaker_core::crash::STOCK[1],
            chipbreaker_core::crash::STOCK[2],
        ),
    );
    TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn run(case: &CrashCase) -> Vec<Collision> {
    let profile = case.profile();
    let mut field = stock_field();
    let kinds: Vec<MotionKind> = case
        .motions
        .iter()
        .enumerate()
        // The middle move is the feed; the rest position the tool.
        .map(|(i, _)| {
            if i == 2 {
                MotionKind::Linear
            } else {
                MotionKind::Rapid
            }
        })
        .collect();
    let provenance: Vec<Provenance> = (0..case.motions.len())
        .map(|i| {
            Provenance::new(
                0,
                u32::try_from(i).unwrap_or(0) + 3,
                u32::try_from(i).unwrap_or(0),
            )
        })
        .collect();

    let fixtures = case.clamp.map_or_else(Vec::new, |(lo, hi)| {
        let mesh = shapes::box_solid(lo, hi);
        let f = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing: SPACING,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds")
        .0;
        vec![("clamp".to_owned(), f)]
    });

    let mut scratch = CutScratch::new(&profile);
    collide_with_stock(
        &mut field,
        &profile,
        &case.motions,
        &kinds,
        &provenance,
        0,
        &fixtures,
        &CollideParams {
            clearance_mm: 0.0,
            grid_mm: 2.0 * SPACING,
            method: SweepMethod::Analytic {
                tolerance: SPACING / 10.0,
            },
        },
        &mut scratch,
    )
    .expect("every corpus tool has a chuck, so every case is checkable")
}

struct Score {
    checked: usize,
    expected: usize,
    missed: Vec<String>,
    invented: Vec<String>,
    wrong_element: Vec<String>,
}

fn score(step: usize) -> Score {
    let mut s = Score {
        checked: 0,
        expected: 0,
        missed: Vec::new(),
        invented: Vec::new(),
        wrong_element: Vec::new(),
    };
    for case in corpus().iter().step_by(step) {
        let found = run(case);
        let hard: Vec<&Collision> = found.iter().filter(|c| c.is_defect()).collect();
        s.checked += 1;

        if case.kind.collides() {
            s.expected += 1;
            if hard.is_empty() {
                s.missed.push(format!("{}: {}", case.id, case.why));
                continue;
            }
            // The right element, not merely something. A checker that reported
            // the shank whenever the chuck crashed would score zero misses and
            // still send somebody to fix the wrong thing.
            let want = case.kind.element().expect("a colliding case names one");
            if !hard.iter().any(|c| c.role == want) {
                s.wrong_element.push(format!(
                    "{}: expected {} contact, got {:?}",
                    case.id,
                    want.as_str(),
                    hard.iter().map(|c| c.role.as_str()).collect::<Vec<_>>()
                ));
            }
            // A clamp case must name the clamp rather than the stock.
            if case.kind == CrashKind::HolderIntoClamp
                && !hard.iter().any(|c| {
                    matches!(
                        c.obstacle,
                        chipbreaker_core::findings::Obstacle::Fixture { .. }
                    )
                })
            {
                s.wrong_element
                    .push(format!("{}: hit something, but not the clamp", case.id));
            }
        } else if !hard.is_empty() {
            s.invented.push(format!(
                "{}: {} -- reported {:?}",
                case.id,
                case.why,
                hard.iter()
                    .map(|c| (c.role.as_str(), c.contact.magnitude()))
                    .collect::<Vec<_>>()
            ));
        }
    }
    s
}

fn report(s: &Score) {
    #[allow(clippy::cast_precision_loss, reason = "small counts")]
    let pct = |n: usize, d: usize| n as f64 * 100.0 / (d.max(1) as f64);
    let clean = s.checked - s.expected;
    println!(
        "crash corpus: {} cases ({} colliding, {} clean)\n  \
         missed        {} ({:.1}% of colliding)\n  \
         invented      {} ({:.1}% of clean)\n  \
         wrong element {}",
        s.checked,
        s.expected,
        clean,
        s.missed.len(),
        pct(s.missed.len(), s.expected),
        s.invented.len(),
        pct(s.invented.len(), clean),
        s.wrong_element.len(),
    );
    for m in &s.missed {
        println!("  MISSED   {m}");
    }
    for m in &s.invented {
        println!("  INVENTED {m}");
    }
    for m in &s.wrong_element {
        println!("  ELEMENT  {m}");
    }
}

#[test]
fn the_corpus_has_both_halves_and_is_the_size_it_claims() {
    // Before any of the numbers below mean anything: the corpus has to contain
    // cases of both kinds, or one of the two exit criteria is vacuous.
    let all = corpus();
    let colliding = chipbreaker_core::crash::colliding(&all);
    println!(
        "corpus: {} cases, {} colliding, {} clean",
        all.len(),
        colliding,
        all.len() - colliding
    );
    assert_eq!(all.len(), 100, "the corpus is specified at 100 cases");
    assert!(
        colliding >= 30,
        "only {colliding} cases plant a collision; 'zero missed' would be nearly vacuous"
    );
    assert!(
        all.len() - colliding >= 30,
        "only {} clean cases; 'no false positives' would be nearly vacuous",
        all.len() - colliding
    );
    // Every kind represented, so a whole category cannot silently drop out.
    for kind in [
        CrashKind::ShankInPocketWall,
        CrashKind::HolderIntoFloor,
        CrashKind::RapidAcrossUnclearedStock,
        CrashKind::HolderIntoClamp,
    ] {
        assert!(
            all.iter().any(|c| c.kind == kind),
            "no case of kind {}",
            kind.as_str()
        );
    }
}

#[test]
fn zero_missed_collisions_on_a_sample() {
    // Every third case, which keeps every kind represented while staying in the
    // fast suite. The full sweep is the ignored test below.
    let s = score(3);
    report(&s);
    assert!(s.expected >= 5, "too few colliding cases sampled to judge");
    assert!(
        s.missed.is_empty(),
        "{} planted collision(s) were not found:\n  {}",
        s.missed.len(),
        s.missed.join("\n  ")
    );
    assert!(
        s.wrong_element.is_empty(),
        "{} case(s) reported the wrong element:\n  {}",
        s.wrong_element.len(),
        s.wrong_element.join("\n  ")
    );
}

#[test]
fn no_invented_collisions_on_a_sample() {
    // The other half, and the one that keeps the first from being satisfied by
    // a checker that reports everything.
    let s = score(3);
    assert!(
        s.checked - s.expected >= 5,
        "too few clean cases sampled to judge"
    );
    assert!(
        s.invented.is_empty(),
        "{} clean case(s) were reported as collisions:\n  {}",
        s.invented.len(),
        s.invented.join("\n  ")
    );
}

#[test]
#[ignore = "nightly: the full hundred cases"]
fn the_whole_crash_corpus() {
    let s = score(1);
    report(&s);
    assert_eq!(s.checked, 100);
    assert!(
        s.missed.is_empty(),
        "{} planted collision(s) were not found:\n  {}",
        s.missed.len(),
        s.missed.join("\n  ")
    );
    assert!(
        s.invented.is_empty(),
        "{} clean case(s) were reported as collisions:\n  {}",
        s.invented.len(),
        s.invented.join("\n  ")
    );
    assert!(
        s.wrong_element.is_empty(),
        "{} case(s) reported the wrong element:\n  {}",
        s.wrong_element.len(),
        s.wrong_element.join("\n  ")
    );
}

#[test]
fn collisions_do_not_depend_on_the_order_the_field_was_walked() {
    // The determinism guarantee U13's clusters carry, for collisions: the answer
    // is a property of the geometry, so two runs of the same case must produce
    // the same list, in the same order, with the same identities. A diff between
    // two reports depends on exactly this.
    let cases = corpus();
    let mut checked = 0usize;
    for case in cases.iter().step_by(11) {
        let a = run(case);
        let b = run(case);
        let key = |v: &[Collision]| {
            v.iter()
                .map(|c| {
                    format!(
                        "{} {} {} {:.9}",
                        c.id,
                        c.role.as_str(),
                        c.motion.as_str(),
                        c.contact.magnitude()
                    )
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(key(&a), key(&b), "{} gave two different answers", case.id);
        checked += 1;
    }
    assert!(checked >= 5, "only {checked} cases compared");
    println!("{checked} cases reproduced identically");
}

#[test]
fn the_element_check_would_notice_a_checker_that_named_anything() {
    // The mutation check for the element assertion. If every colliding case
    // expected the same element, "the right element" would be free. Assert the
    // corpus actually demands both.
    let all = corpus();
    let wants_shank = all
        .iter()
        .filter(|c| c.kind.element() == Some(ElementRole::NonCutting))
        .count();
    let wants_holder = all
        .iter()
        .filter(|c| c.kind.element() == Some(ElementRole::Holder))
        .count();
    assert!(
        wants_shank >= 5 && wants_holder >= 5,
        "the corpus expects {wants_shank} shank contacts and {wants_holder} holder \
         contacts; with either near zero the element check proves nothing"
    );
}
