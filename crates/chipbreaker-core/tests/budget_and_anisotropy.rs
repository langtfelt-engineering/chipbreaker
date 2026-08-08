// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The memory ceiling, and what anisotropic spacing does to the guarantee.
//!
//! # The prediction has to be exact, not close
//!
//! A footprint computed from extents, spacings and two type sizes is a pure
//! function, so "within a few percent" would mean the model is wrong somewhere
//! and the slack is hiding it. The first version of `bytes_per_ray` counted the
//! arena's spill index unconditionally and over-predicted every case by exactly
//! 8.00%, which looked like a safety margin and was a wrong model. These tests
//! assert equality.
//!
//! # Anisotropy must not weaken the guarantee silently
//!
//! `sample_distance_bound` generalises `h * sqrt(3/2)` to
//! `sqrt((hx^2 + hy^2 + hz^2) / 2)`. That is a quadratic mean, so it is driven
//! by the **largest** spacing: coarsening one axis degrades the worst case for
//! every surface, not only for those facing it. `auto_spacing` therefore holds
//! the bound fixed and minimises memory subject to it, and the tests below check
//! both halves of that -- the bound really is unchanged, and the memory really
//! does fall where the part allows it.

use chipbreaker_core::budget::{
    Budget, BudgetError, Spacing, auto_spacing, auto_spacing as pick, bytes_per_ray, ray_counts,
};
use chipbreaker_core::dexel::tri::{SAMPLE_DISTANCE_CONSTANT, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;

/// Stock shapes that span the aspect ratios a shop actually sees.
const PARTS: [(&str, [f64; 3]); 4] = [
    ("cube", [40.0, 40.0, 40.0]),
    ("block", [100.0, 60.0, 20.0]),
    ("plate", [200.0, 200.0, 6.0]),
    ("bar", [300.0, 20.0, 20.0]),
];

fn build(extents: [f64; 3], spacing: Spacing) -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(extents[0], extents[1], extents[2]),
        ),
        &TriBuildOptions {
            spacing: spacing.x,
            spacing_xyz: Some(spacing),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

#[test]
fn the_prediction_is_exact_for_an_uncut_field() {
    for (name, extents) in PARTS {
        for h in [1.0, 0.5, 0.31] {
            let spacing = Spacing::uniform(h);
            let field = build(extents, spacing);
            let measured = field.bytes() as u64;
            let predicted = Budget::predict(extents, spacing, 0, false).field_bytes;
            assert_eq!(
                predicted, measured,
                "{name} at {h} mm: predicted {predicted} against measured {measured}. \
                 This is a pure function of the extents and two type sizes, so any \
                 difference at all means the model is wrong rather than approximate."
            );
        }
    }
}

#[test]
fn the_prediction_is_exact_under_anisotropy_too() {
    for (name, extents) in PARTS {
        let spacing = Spacing {
            x: 0.7,
            y: 0.4,
            z: 0.25,
        };
        let field = build(extents, spacing);
        assert_eq!(
            Budget::predict(extents, spacing, 0, false).field_bytes,
            field.bytes() as u64,
            "{name}: the anisotropic prediction must be exact too"
        );
    }
}

#[test]
fn bytes_per_ray_matches_what_a_bundle_actually_holds() {
    // The constant the whole prediction rests on, checked against a real field
    // rather than against the struct definitions it was derived from.
    let extents = [10.0, 10.0, 10.0];
    let spacing = Spacing::uniform(0.5);
    let field = build(extents, spacing);
    let rays: u64 = ray_counts(extents, spacing).iter().sum();
    assert_eq!(
        field.bytes() as u64 / rays,
        bytes_per_ray() as u64,
        "a ray costs {} bytes in practice against {} predicted",
        field.bytes() as u64 / rays,
        bytes_per_ray()
    );
}

#[test]
fn an_over_budget_job_refuses_before_allocating_and_names_a_spacing_that_fits() {
    let extents = [200.0, 200.0, 200.0];
    let budget = Budget::bytes(32 * 1024 * 1024);
    let err = budget
        .check(extents, Spacing::uniform(0.05), 0, false)
        .expect_err("must refuse");
    let BudgetError::TooLarge { suggestion, .. } = &err else {
        panic!("wrong variant: {err:?}");
    };
    let fit = suggestion.expect("something coarser should fit");
    assert!(
        budget.check(extents, fit, 0, false).is_ok(),
        "the suggested {fit:?} does not actually fit, which is worse than no \
         suggestion"
    );
    // And it must be a number a person would type, not the raw bisection.
    let text = err.to_string();
    assert!(
        text.split("mm fits")
            .next()
            .is_some_and(|t| { t.rsplit(' ').next().is_some_and(|n| n.trim().len() <= 8) }),
        "the suggested spacing should be rounded for a human: {text}"
    );
}

#[test]
fn the_refusal_names_every_contributor_separately() {
    // A user whose problem is a three-million-segment program must not be told
    // to coarsen a lattice that was never the issue.
    let err = Budget::bytes(4096)
        .check([80.0, 80.0, 80.0], Spacing::uniform(0.4), 3_000_000, true)
        .expect_err("must refuse");
    let text = err.to_string();
    for part in [
        "field",
        "spill headroom",
        "extraction window",
        "toolpath IR",
    ] {
        assert!(text.contains(part), "the breakdown omits {part}: {text}");
    }
}

#[test]
fn growth_under_cutting_refuses_with_the_operation_and_segment() {
    let err = Budget::bytes(1024)
        .check_growth(4096, "cutting", 41_332)
        .expect_err("must refuse");
    let text = err.to_string();
    assert!(text.contains("cutting"), "{text}");
    assert!(
        text.contains("41332"),
        "the segment index is the answer to 'how far did it get': {text}"
    );
    assert!(
        text.contains("splits spans"),
        "the message should say WHY a field that fitted can stop fitting: {text}"
    );
}

#[test]
fn the_anisotropic_bound_reduces_to_unit_6_when_isotropic() {
    for h in [0.05, 0.25, 1.6] {
        let got = Spacing::uniform(h).sample_distance_bound();
        let expected = h * SAMPLE_DISTANCE_CONSTANT;
        assert!(
            (got - expected).abs() <= 1.0e-15 * expected,
            "h={h}: {got} against the isotropic {expected}"
        );
    }
}

#[test]
fn auto_res_holds_the_bound_exactly() {
    // The ruling this unit was built to: `--auto-res` may not weaken the
    // accuracy `--res` promised. Checked on every part shape, including the ones
    // where it has nothing to gain.
    for (name, extents) in PARTS {
        for reference in [0.5, 0.1] {
            let chosen = auto_spacing(extents, reference);
            let target = Spacing::uniform(reference).sample_distance_bound();
            let got = chosen.sample_distance_bound();
            assert!(
                (got - target).abs() < 1.0e-9,
                "{name} at {reference} mm: auto-res moved the bound from {target} \
                 to {got}. Memory may be traded for anything except the guarantee."
            );
        }
    }
}

#[test]
fn auto_res_never_costs_memory_and_helps_where_the_part_is_anisotropic() {
    // The other half. It is a constrained minimisation, so it can never do worse
    // than isotropic -- isotropic is in its feasible set.
    let mut saw_a_real_win = false;
    for (name, extents) in PARTS {
        let reference = 0.1;
        let iso = Spacing::uniform(reference);
        let auto = pick(extents, reference);
        let m_iso = Budget::predict(extents, iso, 0, false).field_bytes;
        let m_auto = Budget::predict(extents, auto, 0, false).field_bytes;
        #[allow(clippy::cast_precision_loss, reason = "a ratio of byte counts")]
        let ratio = m_iso as f64 / m_auto as f64;
        assert!(
            ratio > 0.99,
            "{name}: auto-res used MORE memory than isotropic ({ratio:.4}x), which \
             cannot happen for a constrained minimum whose feasible set contains \
             the isotropic point"
        );
        if ratio > 1.10 {
            saw_a_real_win = true;
        }
    }
    assert!(
        saw_a_real_win,
        "no part shape saw a win above 10%, so anisotropy is buying nothing and \
         the optimiser is not doing its job"
    );
}

#[test]
fn a_cube_gets_no_benefit_and_that_is_correct() {
    // The problem is symmetric under a cube, so the constrained optimum is
    // isotropic. A rule that produced anisotropy here would be one that had
    // stopped holding the bound.
    let auto = auto_spacing([40.0, 40.0, 40.0], 0.2);
    let spread = auto.x.max(auto.y).max(auto.z) / auto.x.min(auto.y).min(auto.z);
    assert!(
        spread < 1.10,
        "a cube should come back essentially isotropic; got {auto:?}, spread {spread:.4}"
    );
}

#[test]
fn registration_holds_under_every_spacing_combination() {
    // The invariant extraction made load-bearing. Anisotropy is registration-safe by
    // construction -- each axis still draws from one shared ordinate set -- but
    // "by construction" is what this project checks rather than asserts.
    let combinations = [
        Spacing::uniform(0.5),
        Spacing {
            x: 0.5,
            y: 0.25,
            z: 1.0,
        },
        Spacing {
            x: 0.31,
            y: 0.7,
            z: 0.13,
        },
        Spacing {
            x: 1.6,
            y: 1.6,
            z: 0.2,
        },
    ];
    for (name, extents) in PARTS {
        for spacing in combinations {
            let field = build(extents, spacing);
            field
                .check_registration()
                .unwrap_or_else(|e| panic!("{name} at {spacing:?}: {e}"));
        }
    }
}

#[test]
fn an_anisotropic_field_still_contours_soundly() {
    // Dual contouring assumes the three bundles are the three edge directions of
    // one grid. Anisotropy changes the cell's shape, not that fact -- but the
    // vertex clamp and the virtual ring both read a spacing, and both were wrong
    // before this unit fixed them, so the exit criterion is re-checked here.
    use chipbreaker_core::contour::{ContourOptions, extract};
    use chipbreaker_core::mesh::validate::validate;

    for spacing in [
        Spacing::uniform(0.5),
        Spacing {
            x: 0.8,
            y: 0.4,
            z: 0.25,
        },
    ] {
        let field = build([16.0, 12.0, 8.0], spacing);
        let (mesh, _) = extract(&field, &ContourOptions::default()).expect("extracts");
        let report = validate(&mesh);
        assert!(report.is_manifold, "{spacing:?}: not manifold");
        assert!(report.is_watertight, "{spacing:?}: not watertight");
        assert!(
            report.signed_volume > 0.0,
            "{spacing:?}: inside out, volume {}",
            report.signed_volume
        );
    }
}

#[test]
fn an_anisotropic_field_stays_within_its_own_bound() {
    // The measured half of the derivation. A box's faces are axis aligned, so
    // the worst sample distance is attained where a face is furthest from a ray,
    // and it must not exceed the bound the spacings advertise.
    use chipbreaker_core::dexel::deviation::{measure, sample_mesh};

    for spacing in [
        Spacing::uniform(0.5),
        Spacing {
            x: 0.8,
            y: 0.4,
            z: 0.25,
        },
        Spacing {
            x: 0.25,
            y: 1.0,
            z: 0.5,
        },
    ] {
        let extents = [16.0, 12.0, 8.0];
        let mesh = shapes::box_solid(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(extents[0], extents[1], extents[2]),
        );
        let field = build(extents, spacing);
        let samples = sample_mesh(&mesh, 0.15);
        let report = measure(&field, &samples);
        let bound = spacing.sample_distance_bound();
        assert!(
            report.best_max <= bound,
            "{spacing:?}: measured worst sample distance {:.6} mm exceeds the \
             derived bound {bound:.6} mm. Either the derivation is wrong or the \
             lattice is not the one it describes.",
            report.best_max
        );
    }
}
