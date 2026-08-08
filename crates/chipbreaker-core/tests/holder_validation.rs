// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Holder geometry has to be real before anything is allowed to trust it.
//!
//! # Why this comes before collision detection rather than after
//!
//! Until now the shank existed so that the profile closed. Nothing depended on
//! its dimensions being right, so nothing would have noticed if they were not.
//!
//! Collision detection changes that completely: it is a claim about where the
//! holder is, and a claim computed from unvalidated geometry is a **confidently
//! wrong answer** — the failure mode this project refuses everywhere else, and
//! the one it would be inflicting on itself. A holder entered 10 mm too narrow
//! does not make the checker fail loudly; it makes it report "clear" for a move
//! that takes the spindle off.
//!
//! So a stack that could not exist is rejected at construction, by name, and
//! never repaired. Repairing it would mean guessing which of the two numbers the
//! user meant, and then reporting a collision result for a tool nobody owns.

use chipbreaker_core::tool::catalog::{CatalogError, HolderStage, Shank, flat_end_mill};
use chipbreaker_core::tool::profile::ElementRole;

fn er16(shank_diameter: f64) -> Shank {
    Shank::with_holder(
        shank_diameter,
        50.0,
        [
            HolderStage::cylinder(28.000000000000004, 21.0),
            HolderStage::cylinder(34.0, 41.0),
        ],
    )
}

#[test]
fn a_holder_narrower_than_its_shank_is_refused() {
    // A collet chuck cannot grip a shank wider than its own bore. The error is
    // always data entry, and it always understates the holder -- which is the
    // dangerous direction, because a holder modelled too small collides with
    // nothing.
    let shank = Shank::with_holder(20.0, 50.0, [HolderStage::cylinder(12.0, 30.0)]);
    let e = flat_end_mill(6.0, 20.0, &shank).expect_err("a 12 mm holder cannot grip a 20 mm shank");
    match e {
        CatalogError::HolderNarrowerThanShank {
            holder_diameter,
            shank_diameter,
        } => {
            assert!((holder_diameter - 12.0).abs() < 1e-12);
            assert!((shank_diameter - 20.0).abs() < 1e-12);
        }
        other => panic!("expected HolderNarrowerThanShank, got {other:?}"),
    }
    // The message has to name both numbers: "invalid holder" sends somebody
    // looking through a library for a fault they cannot see.
    let text = e.to_string();
    assert!(
        text.contains("12") && text.contains("20"),
        "the refusal must name both diameters: {text}"
    );
}

#[test]
fn the_narrowness_check_would_notice_if_it_were_inverted() {
    // The mutation check. If the comparison ran the other way, a *correct*
    // holder would be refused -- so assert that the legitimate case builds.
    // Without this, the test above passes just as well against a check that
    // rejects everything.
    let ok = flat_end_mill(6.0, 20.0, &er16(6.0)).expect("a 28 mm chuck grips a 6 mm shank");
    assert!(
        ok.top_of_role(ElementRole::Holder).is_some(),
        "the holder geometry must survive into the profile"
    );
    // And the boundary: equal diameters are legal, since a chuck bore matching
    // its shank is the ordinary case rather than an edge one.
    flat_end_mill(
        6.0,
        20.0,
        &Shank::with_holder(28.0, 50.0, [HolderStage::cylinder(28.0, 20.0)]),
    )
    .expect("a shank exactly filling its bore is legal");
}

#[test]
fn a_stage_that_does_not_advance_up_the_axis_is_refused() {
    // A zero-length stage revolves to nothing; a negative one folds the holder
    // back down through itself. Neither is a holder, and accepting either would
    // put an element in the stack with no well-defined outside.
    for bad in [0.0, -5.0] {
        let shank = Shank::with_holder(6.0, 50.0, [HolderStage::cylinder(28.0, bad)]);
        let e =
            flat_end_mill(6.0, 20.0, &shank).expect_err("a stage of length {bad} must be refused");
        assert!(
            matches!(e, CatalogError::NotPositive { .. }),
            "expected a NotPositive length for {bad}, got {e:?}"
        );
    }
    // Mutation check: a positive length is accepted, so the loop above is not
    // passing because everything is refused.
    flat_end_mill(
        6.0,
        20.0,
        &Shank::with_holder(6.0, 50.0, [HolderStage::cylinder(28.0, 1.0)]),
    )
    .expect("a positive stage length is legal");
}

#[test]
fn a_stack_that_narrows_going_up_is_accepted() {
    // **Deliberately not an error**, and worth pinning so nobody tightens it
    // later on the theory that holders widen monotonically. An ER collet nut is
    // routinely wider than the chuck body immediately above it, so requiring a
    // monotone stack would refuse correct tooling -- a worse failure than
    // accepting an odd one, since the self-intersection check already catches a
    // stack that folds through itself.
    let nut_wider = Shank::with_holder(
        6.0,
        50.0,
        [
            HolderStage::cylinder(50.0, 26.0), // the nut
            HolderStage::cylinder(48.0, 40.0), // the body behind it, slightly slimmer
        ],
    );
    let p = flat_end_mill(6.0, 20.0, &nut_wider).expect("a real ER chuck profile");
    assert!(p.top_of_role(ElementRole::Holder).is_some());
}

#[test]
fn a_tool_without_holder_geometry_is_distinguishable_from_one_with_it() {
    // The predicate collision checking gates on. It has to be a property of the
    // profile rather than of the constructor, because a tool read back from a
    // library file has no `Shank` left -- only elements and their roles.
    let bare = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    assert_eq!(
        bare.top_of_role(ElementRole::Holder),
        None,
        "a plain shank must report no holder, so the collision gate can say \
         `unchecked` rather than inventing a clear result"
    );
    assert!(bare.top_of_role(ElementRole::NonCutting).is_some());

    let held = flat_end_mill(6.0, 20.0, &er16(6.0)).expect("valid");
    let top = held.top_of_role(ElementRole::Holder).expect("a holder");
    assert!(
        (top - 112.0).abs() < 1e-9,
        "the ER16 stack should reach 50 + 21 + 41 = 112 mm, got {top}"
    );
}

#[test]
fn the_gauge_line_may_not_fall_inside_the_holder() {
    // Gauge length is measured tip to spindle face. A gauge length shorter than
    // the assembled tool would put the spindle nose somewhere inside the chuck,
    // which is not a tool but does have a perfectly plausible-looking profile.
    use chipbreaker_core::tool::{Tool, ToolId};
    let held = flat_end_mill(6.0, 20.0, &er16(6.0)).expect("valid");
    let too_short = Tool::new(
        1,
        ToolId::new("t").expect("valid"),
        "gauge line inside the chuck",
        held.clone(),
        100.0,
    );
    assert!(
        too_short.is_err(),
        "a 100 mm gauge length on a 112 mm assembly must be refused"
    );
    // Mutation check: the same tool with a gauge length above the stack is fine,
    // so the refusal is about the number and not about the tool.
    Tool::new(1, ToolId::new("t").expect("valid"), "fine", held, 140.0)
        .expect("a gauge length above the holder is legal");
}
