// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The first test of cutting: it does not accumulate error.
//!
//! `dexel::tri` claims that a cut is exact along each ray — interval arithmetic
//! on exact intersection parameters, not a resampling — so after a thousand cuts
//! a bundle still holds exactly the true remaining solid sampled on its lattice,
//! with only the fixed transverse error set by `h`.
//!
//! This is the test that would catch the claim being false, and it is first
//! because if it fails the subtraction is resampling and the whole unit is the
//! wrong shape. The chained-equals-monolithic test rests on the same
//! property, so a failure here surfaces eight units early.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri, distribution, spilled};
use chipbreaker_core::sweep::{LinearMove, SweepCase};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

/// A flat end mill of the given radius, with enough flute to clear the stock.
fn flat_mill(radius: f64, length: f64) -> Profile {
    flat_end_mill(
        2.0 * radius,
        length,
        &Shank::plain(2.0 * radius, length + 20.0),
    )
    .expect("a valid flat mill")
}

fn stock() -> TriDexelField {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 20.0, 8.0));
    TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: 0.5,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

#[test]
fn cutting_the_same_material_repeatedly_does_not_drift() {
    // THE no-accumulation test. A thousand overlapping passes over the same
    // region: after the first few the material is gone, so every later pass
    // removes nothing and the field must stop changing EXACTLY rather than
    // creeping.
    //
    // If subtraction were a resampling — snapping endpoints to a grid, or
    // rebuilding spans from a reconstructed surface — each pass would nudge the
    // remaining material and the digest would keep moving. It does not.
    let profile = flat_mill(3.0, 12.0);
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);
    let method = SweepMethod::Reference { steps: 8 };

    let digest = |f: &TriDexelField| {
        let mut h = CanonicalHash::new();
        h.add(f);
        h.finish().to_hex()
    };

    let pass = LinearMove {
        start: Vec3::new(5.0, 10.0, 4.0),
        end: Vec3::new(35.0, 10.0, 4.0),
    };

    let mut settled_at = None;
    let mut previous = digest(&field);
    let mut volumes = Vec::new();
    for k in 0..1000u32 {
        cut_tri(&mut field, &profile, &pass, method, &mut scratch);
        let now = digest(&field);
        if now == previous && settled_at.is_none() && k > 0 {
            settled_at = Some(k);
        }
        if settled_at.is_some() {
            assert_eq!(
                now, previous,
                "the field changed at pass {k}, after it had already settled. \
                 Repeated identical cuts must be idempotent: a change here means \
                 subtraction is resampling rather than intersecting."
            );
        }
        previous = now;
        if k % 100 == 0 {
            volumes.push(field.volume());
        }
    }

    let settled = settled_at.expect("repeated identical cuts must reach a fixed point");
    assert!(
        settled <= 2,
        "an identical cut should be idempotent after the first application, settled at {settled}"
    );

    // And the volume is flat, on the bits, across the whole run.
    for (k, v) in volumes.iter().enumerate() {
        assert_eq!(
            v.to_bits(),
            volumes[0].to_bits(),
            "volume moved at sample {k}: {v} against {}",
            volumes[0]
        );
    }
}

#[test]
fn error_against_an_independent_reference_is_flat_in_the_number_of_operations() {
    // The stronger form: N *different* overlapping passes that sweep the same
    // region, against a reference that removes the same material in ONE cut.
    //
    // If error accumulated, the gap between the chained result and the
    // monolithic one would grow with N. It must not grow at all -- both are
    // exact along each ray, so both land on the same intersection parameters.
    let profile = flat_mill(2.0, 10.0);
    let method = SweepMethod::Reference { steps: 16 };

    let full = LinearMove {
        start: Vec3::new(4.0, 10.0, 3.0),
        end: Vec3::new(36.0, 10.0, 3.0),
    };

    let mut errors = Vec::new();
    for pieces in [1u32, 2, 4, 8, 16, 32, 64, 128] {
        let mut chained = stock();
        let mut scratch = CutScratch::new(&profile);
        // Split the same motion into `pieces` overlapping sub-moves. Overlapping
        // rather than abutting, so consecutive cuts genuinely revisit material
        // and any drift would compound.
        for k in 0..pieces {
            let a = f64::from(k) / f64::from(pieces);
            let b = (f64::from(k + 1) / f64::from(pieces)).min(1.0);
            let overlap = 0.25 / f64::from(pieces);
            let piece = LinearMove {
                start: full.at((a - overlap).max(0.0)),
                end: full.at((b + overlap).min(1.0)),
            };
            cut_tri(&mut chained, &profile, &piece, method, &mut scratch);
        }

        // The reference: enough sub-steps that the whole motion is resolved
        // finely, applied once.
        let mut monolithic = stock();
        let mut scratch = CutScratch::new(&profile);
        cut_tri(
            &mut monolithic,
            &profile,
            &full,
            SweepMethod::Reference { steps: 512 },
            &mut scratch,
        );

        let error = (chained.volume() - monolithic.volume()).abs() / monolithic.volume().max(1.0);
        errors.push((pieces, error));
    }

    // The error must not GROW with the number of operations. It may fall, since
    // more pieces means the union covers the sweep more finely.
    let worst_late = errors
        .iter()
        .filter(|(n, _)| *n >= 16)
        .map(|(_, e)| *e)
        .fold(0.0f64, f64::max);
    let early = errors[0].1;
    assert!(
        worst_late <= early.max(1e-6) * 1.5,
        "error grew with the operation count, which means cutting accumulates: {errors:?}"
    );
}

// --- the arena, re-measured after cutting ----------------------------------

#[test]
fn a_through_pocket_puts_two_spans_on_transverse_rays() {
    // INLINE_CAPACITY = 2 was sized on stock at rest, where the distribution
    // is nearly degenerate: one span on every ray. Cutting splits spans, so the
    // number that matters is this one, taken AFTER a cut.
    //
    // A slot cut clean through the block leaves material either side of it, so
    // every transverse ray crossing the slot carries two spans by construction.
    let profile = flat_mill(3.0, 12.0);
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);

    let before = distribution(&field);
    assert_eq!(
        before.keys().copied().max(),
        Some(1),
        "stock at rest should be one span per filled ray: {before:?}"
    );

    // A slot straight through, deeper than the stock.
    cut_tri(
        &mut field,
        &profile,
        &LinearMove {
            start: Vec3::new(-5.0, 10.0, -1.0),
            end: Vec3::new(45.0, 10.0, -1.0),
        },
        SweepMethod::Reference { steps: 32 },
        &mut scratch,
    );

    let after = distribution(&field);
    assert!(
        after.contains_key(&2),
        "a through slot must leave two-span rays: {after:?}"
    );
    assert_eq!(spilled(&field), 0, "a slot must not spill past capacity 2");
}

#[test]
fn a_boss_puts_three_spans_on_a_ray_and_that_spills() {
    // The case the specification predicted, and the one that decides whether
    // INLINE_CAPACITY = 2 survives. Two parallel slots leave a standing rib
    // between them, so a transverse ray crosses material-gap-material-gap-
    // material: three spans, one past the inline capacity.
    let profile = flat_mill(2.0, 12.0);
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);

    for y in [7.0, 13.0] {
        cut_tri(
            &mut field,
            &profile,
            &LinearMove {
                start: Vec3::new(-5.0, y, -1.0),
                end: Vec3::new(45.0, y, -1.0),
            },
            SweepMethod::Reference { steps: 32 },
            &mut scratch,
        );
    }

    let after = distribution(&field);
    let three_or_more: usize = after
        .iter()
        .filter(|(k, _)| **k >= 3)
        .map(|(_, v)| *v)
        .sum();
    assert!(
        three_or_more > 0,
        "two slots either side of a rib must leave three-span rays: {after:?}"
    );
    // And those rays are exactly the ones that spill.
    assert_eq!(
        spilled(&field),
        three_or_more,
        "every ray past the inline capacity should be in the spill map"
    );
}

#[test]
fn the_case_classification_matches_the_geometry() {
    let cases = [
        (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            SweepCase::Stationary,
        ),
        (
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(10.0, 0.0, 5.0),
            SweepCase::Horizontal,
        ),
        (
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            SweepCase::Plunge,
        ),
        (
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(10.0, 3.0, 1.0),
            SweepCase::Ramp,
        ),
    ];
    for (start, end, expected) in cases {
        assert_eq!(
            LinearMove { start, end }.case(),
            expected,
            "{start:?} -> {end:?}"
        );
    }
    let _ = AXES;
}
