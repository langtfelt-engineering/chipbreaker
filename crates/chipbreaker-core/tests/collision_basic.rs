// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does collision detection find a holder in the stock, and only then?
//!
//! # The two failures this file is built around
//!
//! **Missing a collision** is the failure that scraps a spindle, and it is the
//! easy one to commit by accident: check against the final field and every
//! collision with material a later pass removes disappears silently.
//!
//! **Inventing one** is the failure that gets the tool switched off. A holder
//! following its cutter down into a pocket the cutter just opened is doing
//! exactly what it should, and reporting it would make the checker useless on
//! every real program.
//!
//! The same fixture produces both, which is what makes the pair worth having:
//! one program plunges the holder into full stock, one does not, and the
//! geometry differs by nothing except how deep the tool goes.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::detect::{
    CollideParams, Unchecked, collide_with_stock, cutting_only, holder_present,
};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{HolderStage, Shank, flat_end_mill};
use chipbreaker_core::tool::profile::ElementRole;
use chipbreaker_core::toolpath::{MotionKind, Provenance};

const SPACING: f64 = 0.5;
/// A tall block, so the holder has something to hit well above the floor.
const STOCK: Vec3 = Vec3 {
    x: 60.0,
    y: 40.0,
    z: 40.0,
};

fn stock_mesh() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), STOCK)
}

fn field() -> TriDexelField {
    TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// A stub cutter under a bulky chuck: 10 mm of flute, then a 6 mm shank to
/// z = 20, then a 50.8 mm nut. Anything deeper than 20 mm puts the nut in the
/// material.
fn stub_in_chuck() -> Profile {
    flat_end_mill(
        6.0,
        10.0,
        &Shank::with_holder(
            6.0,
            20.0,
            [
                HolderStage::cylinder(50.8, 28.0),
                HolderStage::cylinder(61.912499999999994, 50.0),
            ],
        ),
    )
    .expect("valid")
}

/// The same cutter with 80 mm of shank, so the chuck stays clear.
fn long_reach_in_chuck() -> Profile {
    flat_end_mill(
        6.0,
        10.0,
        &Shank::with_holder(
            6.0,
            80.0,
            [
                HolderStage::cylinder(50.8, 28.0),
                HolderStage::cylinder(61.912499999999994, 50.0),
            ],
        ),
    )
    .expect("valid")
}

fn params() -> CollideParams {
    CollideParams {
        clearance_mm: 0.0,
        grid_mm: 2.0,
        method: SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
    }
}

/// Plunges to `z`, feeds across in x, retracts.
fn plunge_and_cut(z: f64) -> (Vec<Motion>, Vec<MotionKind>, Vec<Provenance>) {
    let motions = vec![
        Motion::Linear(LinearMove {
            start: Vec3::new(30.0, 20.0, 60.0),
            end: Vec3::new(30.0, 20.0, z),
        }),
        Motion::Linear(LinearMove {
            start: Vec3::new(30.0, 20.0, z),
            end: Vec3::new(45.0, 20.0, z),
        }),
        Motion::Linear(LinearMove {
            start: Vec3::new(45.0, 20.0, z),
            end: Vec3::new(45.0, 20.0, 60.0),
        }),
    ];
    let kinds = vec![MotionKind::Rapid, MotionKind::Linear, MotionKind::Rapid];
    let provenance = (0..3)
        .map(|i| Provenance::new(0, i + 3, i))
        .collect::<Vec<_>>();
    (motions, kinds, provenance)
}

fn run(profile: &Profile, z: f64) -> Vec<chipbreaker_core::findings::Collision> {
    let (motions, kinds, provenance) = plunge_and_cut(z);
    let mut f = field();
    let mut scratch = CutScratch::new(profile);
    collide_with_stock(
        &mut f,
        profile,
        &motions,
        &kinds,
        &provenance,
        0,
        &[],
        &params(),
        &mut scratch,
    )
    .expect("the tool has a holder, so the check runs")
}

#[test]
fn a_holder_driven_into_full_stock_is_found() {
    // 30 mm below a 40 mm top face, with only 20 mm of shank: the nut is 10 mm
    // into solid material.
    let found = run(&stub_in_chuck(), 10.0);
    eprintln!("stub at z=10: {} contacts", found.len());
    for c in &found {
        eprintln!(
            "  {} {} {} at {:?} {:.3} mm",
            c.id,
            c.role.as_str(),
            c.motion.as_str(),
            c.at.to_array(),
            c.contact.magnitude()
        );
    }
    assert!(
        !found.is_empty(),
        "a 50.8 mm chuck 10 mm into a solid block must be reported"
    );
    assert!(
        found.iter().any(|c| c.role == ElementRole::Holder),
        "the holder is what entered the stock; reporting only the shank would \
         understate it: {:?}",
        found.iter().map(|c| c.role.as_str()).collect::<Vec<_>>()
    );
    assert!(
        found.iter().all(|c| c.is_defect()),
        "contact with solid material is a collision, not a near miss"
    );
}

#[test]
fn the_same_cut_with_enough_reach_keeps_the_chuck_out() {
    // **The mutation check for the test above, and the more important half.**
    // Identical program, identical chuck, 60 mm more shank.
    //
    // The *shank* is buried in both cases and correctly so: a 10 mm flute
    // cutting a 30 mm deep slot puts 20 mm of shank below the top face, whatever
    // is above it. That is a real rub and the checker is right to say so — it
    // was this test expecting silence that was wrong, not the detector.
    //
    // What separates the two tools is the **holder**, so that is what this
    // asserts. Demanding an empty list would have meant quietly weakening the
    // shank check to make a tidier test pass.
    let found = run(&long_reach_in_chuck(), 10.0);
    eprintln!("long reach at z=10: {} contacts", found.len());
    for c in &found {
        eprintln!(
            "  {} {} {:.3} mm at {:?}",
            c.id,
            c.role.as_str(),
            c.contact.magnitude(),
            c.at.to_array()
        );
    }
    assert!(
        !found.iter().any(|c| c.role == ElementRole::Holder),
        "a chuck 60 mm clear of the top face must not be reported as hitting it"
    );
    assert!(
        found.iter().any(|c| c.role == ElementRole::NonCutting),
        "the shank genuinely is below the top face here; if this stops being \
         reported, the check has gone blind rather than got tidier"
    );
}

#[test]
fn a_shallow_cut_does_not_bury_the_chuck() {
    // The other direction: the stub tool is fine as long as it stays shallow.
    // Cutting 2 mm below the face leaves the nut 8 mm above it.
    let found = run(&stub_in_chuck(), 38.0);
    assert!(
        found.is_empty(),
        "a 2 mm deep cut leaves the chuck above the stock: {:?}",
        found
            .iter()
            .map(|c| (c.role.as_str(), c.at.to_array()))
            .collect::<Vec<_>>()
    );
}

#[test]
fn checking_happens_before_the_cut_not_after() {
    // **The temporal property, as a test.**
    //
    // A collision must be judged against the material present when the motion
    // runs. Detection that ran against the *final* field would test every motion
    // against the least material the job ever contains, and would miss this: the
    // plunge buries the chuck in stock that the plunge itself then removes.
    //
    // So: run the same program, and separately cut the field to completion and
    // confirm the material at the contact point is gone by the end. If the
    // checker were reading the end state it would see air there and report
    // nothing.
    let profile = stub_in_chuck();
    let found = run(&profile, 10.0);
    assert!(!found.is_empty(), "the fixture must produce a collision");

    let (motions, _, _) = plunge_and_cut(10.0);
    let mut after = field();
    let mut scratch = CutScratch::new(&profile);
    chipbreaker_core::sweep::batch::cut_all(
        &mut after,
        &profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        chipbreaker_core::sweep::batch::DEFAULT_BATCH,
    );

    // The tool axis at the plunge. By the end of the program this column has
    // been cut away by the chuck itself, so an end-state check sees nothing.
    let start = field();
    let axis_x = 30.0;
    let occupied = |f: &TriDexelField| -> f64 {
        let bundle = f.bundle(chipbreaker_core::math::Axis::Z).expect("z bundle");
        let lattice = bundle.lattice().clone();
        let mut total = 0.0;
        for r in 0..u32::try_from(bundle.arena().rays()).unwrap_or(0) {
            let (i, j) = lattice.coords(r);
            let o = lattice.origin_of(i, j);
            if (o.x - axis_x).abs() < 3.0 && (o.y - 20.0).abs() < 3.0 {
                total += bundle
                    .arena()
                    .get(r)
                    .iter()
                    .map(|s| s.length())
                    .sum::<f64>();
            }
        }
        total
    };
    let before_mm = occupied(&start);
    let after_mm = occupied(&after);
    eprintln!("material on the tool axis: {before_mm:.1} mm before, {after_mm:.1} mm after");
    assert!(
        after_mm < before_mm * 0.5,
        "the fixture must remove most of the material at the contact, or an \
         end-state check would find it too and this proves nothing"
    );
}

#[test]
fn a_tool_without_a_holder_is_unchecked_rather_than_clear() {
    // Silence here would read as safety. It has to be an explicit refusal.
    let bare = flat_end_mill(6.0, 10.0, &Shank::plain(6.0, 20.0)).expect("valid");
    assert!(!holder_present(&bare));

    let (motions, kinds, provenance) = plunge_and_cut(10.0);
    let mut f = field();
    let mut scratch = CutScratch::new(&bare);
    let e = collide_with_stock(
        &mut f,
        &bare,
        &motions,
        &kinds,
        &provenance,
        0,
        &[],
        &params(),
        &mut scratch,
    )
    .expect_err("a tool with no holder cannot be checked");
    assert_eq!(e, Unchecked::NoHolder);
    assert!(
        e.to_string().contains("holder"),
        "the refusal must name what is missing: {e}"
    );
}

#[test]
fn unmodelled_retracts_make_the_answer_unchecked() {
    // A program expanded without motion the machine will make cannot be
    // certified clean, however clean the motion we do have turns out to be.
    let profile = long_reach_in_chuck();
    let (motions, kinds, provenance) = plunge_and_cut(38.0);
    let mut f = field();
    let mut scratch = CutScratch::new(&profile);
    let e = collide_with_stock(
        &mut f,
        &profile,
        &motions,
        &kinds,
        &provenance,
        3,
        &[],
        &params(),
        &mut scratch,
    )
    .expect_err("three unmodelled retracts must block certification");
    assert_eq!(e, Unchecked::UnmodelledRetracts(3));
    assert!(e.to_string().contains('3'));

    // Mutation check: the identical run with none is checkable, so the refusal
    // is about the retracts and not about the fixture.
    let mut f2 = field();
    collide_with_stock(
        &mut f2,
        &profile,
        &motions,
        &kinds,
        &provenance,
        0,
        &[],
        &params(),
        &mut scratch,
    )
    .expect("with no unmodelled retracts the same run is checkable");
}

#[test]
fn the_cutting_only_profile_is_the_tool_without_its_shank() {
    let full = stub_in_chuck();
    let cut = cutting_only(&full).expect("a held tool has non-cutting geometry");
    assert!(
        cut.elements()
            .iter()
            .all(|e| e.role == ElementRole::Cutting),
        "the cutting-only profile must contain no shank or holder"
    );
    assert!(cut.elements().len() < full.elements().len());
    // It reaches the top of the flutes and no further, which is what makes the
    // subtraction isolate exactly the non-cutting part.
    let top = cut.top().y;
    assert!(
        (top - 10.0).abs() < 1e-9,
        "the cutting geometry ends at the flute top, got {top}"
    );
}

/// A clamp standing beside the block, as its own field.
///
/// A fixture is a field like any other and is simply never cut: a clamp does not
/// get out of the way, which is exactly what makes it dangerous.
///
/// Tall on purpose. The tool feeds at z = 38 with its flutes below z = 48, so a
/// clamp that stopped at the height of the stock would sit entirely under the
/// shank and be hit by nothing -- which is a fixture the test would pass against
/// without exercising anything.
fn clamp_at(x: f64) -> (String, TriDexelField) {
    let mesh = shapes::box_solid(Vec3::new(x, 14.0, 0.0), Vec3::new(x + 12.0, 26.0, 60.0));
    let f = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    ("clamp".to_owned(), f)
}

fn run_with(
    profile: &Profile,
    z: f64,
    fixtures: &[(String, TriDexelField)],
    clearance_mm: f64,
) -> Vec<chipbreaker_core::findings::Collision> {
    let (motions, kinds, provenance) = plunge_and_cut(z);
    let mut f = field();
    let mut scratch = CutScratch::new(profile);
    let p = CollideParams {
        clearance_mm,
        ..params()
    };
    collide_with_stock(
        &mut f,
        profile,
        &motions,
        &kinds,
        &provenance,
        0,
        fixtures,
        &p,
        &mut scratch,
    )
    .expect("checkable")
}

#[test]
fn a_holder_that_hits_a_clamp_is_reported_against_that_clamp() {
    // The feed runs to x = 45 with a 50.8 mm chuck, so a clamp at x = 48 is in
    // its path even though the stock there is untouched.
    let found = run_with(&long_reach_in_chuck(), 38.0, &[clamp_at(44.0)], 0.0);
    eprintln!("with a clamp at x=48: {} contacts", found.len());
    for c in &found {
        eprintln!(
            "  {} {} vs {} {:.3} mm",
            c.id,
            c.role.as_str(),
            c.obstacle.kind(),
            c.contact.magnitude()
        );
    }
    let against_clamp: Vec<_> = found
        .iter()
        .filter(|c| {
            matches!(
                &c.obstacle,
                chipbreaker_core::findings::Obstacle::Fixture { .. }
            )
        })
        .collect();
    assert!(
        !against_clamp.is_empty(),
        "a shank sweeping to x=48 must be reported against a clamp at x=44"
    );
    assert!(
        against_clamp.iter().all(|c| c.is_defect()),
        "contact with a clamp is a collision"
    );
    // The obstacle has to be named, not merely counted: "something was hit" does
    // not tell anybody which fixture to move.
    match &against_clamp[0].obstacle {
        chipbreaker_core::findings::Obstacle::Fixture { name, index } => {
            assert_eq!(name, "clamp");
            assert_eq!(*index, 0);
        }
        other => panic!("expected a fixture, got {other:?}"),
    }
}

#[test]
fn the_same_program_without_the_clamp_is_clean() {
    // The mutation check: the collision above must come from the clamp and not
    // from the program, which is fine on its own.
    let found = run_with(&long_reach_in_chuck(), 38.0, &[], 0.0);
    assert!(
        found.is_empty(),
        "a 2 mm deep cut with a long-reach tool and no fixtures must be clean: {:?}",
        found
            .iter()
            .map(|c| (c.role.as_str(), c.obstacle.kind()))
            .collect::<Vec<_>>()
    );
}

#[test]
fn clearance_reports_a_near_miss_and_only_within_the_threshold() {
    // The shank's envelope reaches x = 45 + 3 = 48, so a clamp at 49.6 clears
    // it by 1.6 mm: too far to be a collision, close enough to be worth saying.
    let far = clamp_at(49.6);

    let none = run_with(
        &long_reach_in_chuck(),
        38.0,
        std::slice::from_ref(&far),
        0.0,
    );
    assert!(
        none.is_empty(),
        "with clearance reporting off, a gap must produce nothing: {:?}",
        none.iter().map(|c| c.contact.as_str()).collect::<Vec<_>>()
    );

    let wide = run_with(
        &long_reach_in_chuck(),
        38.0,
        std::slice::from_ref(&far),
        5.0,
    );
    let misses: Vec<_> = wide.iter().filter(|c| !c.is_defect()).collect();
    eprintln!("clearance 5.0: {} near miss(es)", misses.len());
    for c in &misses {
        eprintln!("  {} gap {:.4} mm", c.id, c.contact.magnitude());
    }
    assert!(
        !misses.is_empty(),
        "a clamp 1.6 mm outside the envelope must be reported at a 5 mm threshold"
    );
    assert!(
        misses.iter().all(|c| !c.is_defect()),
        "a near miss is not a defect and must not fail the gate"
    );

    // And the threshold has to bite: below the actual gap, nothing fires.
    let narrow = run_with(&long_reach_in_chuck(), 38.0, &[far], 0.5);
    assert!(
        narrow.iter().all(|c| c.is_defect()),
        "a 0.5 mm threshold must not report a gap wider than that: {:?}",
        narrow
            .iter()
            .map(|c| (c.contact.as_str(), c.contact.magnitude()))
            .collect::<Vec<_>>()
    );
}

#[test]
fn collisions_are_independent_of_fixture_order() {
    // Two clamps, presented both ways round. The result is a property of the
    // geometry, so the list must match -- including the identities, which is
    // what a diff between two runs depends on.
    let a = clamp_at(44.0);
    let b = clamp_at(-14.0);
    let one = run_with(&long_reach_in_chuck(), 38.0, &[a.clone(), b.clone()], 0.0);
    let two = run_with(&long_reach_in_chuck(), 38.0, &[b, a], 0.0);
    assert!(!one.is_empty(), "the fixture must produce contacts");
    let key = |v: &[chipbreaker_core::findings::Collision]| {
        let mut k: Vec<String> = v
            .iter()
            .map(|c| {
                format!(
                    "{} {} {:.6}",
                    c.role.as_str(),
                    c.obstacle.kind(),
                    c.contact.magnitude()
                )
            })
            .collect();
        k.sort();
        k
    };
    assert_eq!(
        key(&one),
        key(&two),
        "swapping the order the fixtures were listed changed the collisions found"
    );
}
