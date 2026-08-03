// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Convergence: the claims the measurements actually support.
//!
//! The unit specification asked for a fitted exponent per solid, asserted to be
//! superlinear at `> 1.5`. Measuring it showed that a fitted exponent is the
//! wrong instrument for two of the six cases, for two different reasons:
//!
//! - The axis-parallel cylinder's error is the Gauss circle problem, which is
//!   erratic. A least-squares fit through six samples of it returns a number
//!   that changes with the sample grid — 1.91 on one, 1.57 on another. Neither
//!   describes anything.
//! - The smooth solids converge so fast that their **signed** error crosses zero
//!   inside the tested range. The magnitude then dips wherever the crossing
//!   happens to fall, which reads as superconvergence and is cancellation. The
//!   cone and the R=10 torus are both non-monotone for this reason.
//!
//! So what is asserted here is an **envelope**: `error <= (h/R)^p`, with the
//! constant pinned at one rather than fitted. That is a superlinearity claim,
//! it is robust to the wobble, and it is the shape of the theory in both
//! regimes. The fitted exponents are still measured and reported by
//! `examples/dexel_convergence.rs`, clearly marked as information.
//!
//! See `dexel::convergence` for the full derivation.

use chipbreaker_core::dexel::convergence::{
    Case, Convergence, ErrorModel, GAUSS_CIRCLE_EXPONENT, measure, standard_cases,
};
use chipbreaker_core::dexel::{BuildOptions, DexelField};
use chipbreaker_core::math::Mat4;
use chipbreaker_core::mesh::shapes;

/// Superlinear. The specification's `1.5`, kept as the exponent of the envelope
/// rather than as a target for a fit.
const SUPERLINEAR: f64 = 1.5;

/// Coarse enough to run in a debug build on every commit. The finer grid, and
/// the `h <= R/200` bound that needs it, live in the nightly test below.
fn quick_ratios() -> Vec<f64> {
    vec![0.1, 0.05, 0.025]
}

fn envelope_for(result: &Convergence) -> f64 {
    match result.model {
        ErrorModel::Quadrature => SUPERLINEAR,
        ErrorModel::LatticeCount => GAUSS_CIRCLE_EXPONENT,
    }
}

#[test]
fn every_solid_stays_inside_its_superlinear_envelope() {
    for case in standard_cases() {
        let result = measure(&case, &quick_ratios());
        let p = envelope_for(&result);
        let c = result.envelope_constant(p);
        assert!(
            c <= 1.0,
            "{}: error <= C * (h/R)^{p} needs C = {c:.4}, which exceeds 1. Either \
             convergence has regressed or the case has been given the wrong ErrorModel.",
            result.name
        );
    }
}

#[test]
fn the_upright_cylinder_error_is_exactly_lattice_point_counting() {
    // The load-bearing measurement of this module, and the one that identifies
    // the model rather than merely fitting it.
    //
    // A cylinder whose axis runs along the bundle has a chord that is the full
    // height inside the silhouette and zero outside. So its dexel volume must be
    // EXACTLY `h^2 * H * (rays whose centre lies inside the disc)`. If that
    // identity holds, the volume error is the error in counting lattice points
    // inside a disc -- the Gauss circle problem -- and the specification's 1.37
    // exponent is not an approximation but 2 - 131/208 exactly.
    let radius = 10.0;
    let height = 20.0;
    let mesh = shapes::cylinder(radius, height, 256);

    for spacing in [1.0, 0.5, 0.25] {
        let (field, _) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing,
                ..BuildOptions::default()
            },
        )
        .expect("builds");

        // Count independently, from the lattice alone, touching no span.
        let lattice = field.lattice();
        let [nx, ny] = lattice.counts();
        let mut inside = 0u64;
        for i in 0..nx {
            for j in 0..ny {
                let p = lattice.origin_of(i, j);
                if p.x * p.x + p.y * p.y <= radius * radius {
                    inside += 1;
                }
            }
        }

        // The mesh is a 256-gon, so its silhouette is very slightly inside the
        // ideal circle and a handful of rays near the boundary can fall the
        // other way. The identity is about the STRUCTURE of the error, so a few
        // boundary rays out of tens of thousands is the expected agreement.
        let predicted = inside as f64 * spacing * spacing * height;
        let relative = (field.volume() - predicted).abs() / predicted;
        assert!(
            relative < 2e-3,
            "at h = {spacing}: the field measured {} but a lattice-point count \
             predicts {predicted} ({relative:e} apart). The volume of an \
             axis-parallel cylinder must be h^2 * H * (points inside the disc); \
             if it is not, the Gauss circle identification is wrong.",
            field.volume()
        );
    }
}

#[test]
fn refining_the_lattice_is_not_always_an_improvement() {
    // Pinned as a test because it is the finding, not a defect, and because a
    // future reader who sees it will otherwise try to fix it.
    //
    // The Gauss circle error does not decay monotonically. On the axis-parallel
    // cylinder there is a refinement that quadruples the ray count and makes the
    // answer worse. A single-axis field therefore cannot promise a customer that
    // a finer simulation is a safer one -- which is the argument for Unit 6.
    let case = standard_cases()
        .into_iter()
        .find(|c| c.model == ErrorModel::LatticeCount)
        .expect("the upright cylinder is in the standard set");
    let result = measure(&case, &[0.1, 0.05, 0.025, 0.0125, 0.00625]);
    assert!(
        !result.is_monotone(),
        "the lattice-count case came out monotone across {:?}. Either the grid no \
         longer straddles a Gauss circle fluctuation, or something has changed \
         about how the silhouette is sampled. Investigate before relaxing this.",
        result.samples.iter().map(|s| s.ratio).collect::<Vec<_>>()
    );
}

#[test]
fn the_same_cylinder_lying_down_converges_predictably() {
    // The other half of the contrast, and the cleanest statement in the project
    // of why three bundles are needed. Same solid, same lattice, rotated: the
    // chord becomes 2*sqrt(R^2 - x^2), the sum becomes a midpoint quadrature of
    // a continuous profile, and the error becomes monotone.
    let case = standard_cases()
        .into_iter()
        .find(|c| c.name.contains("ACROSS"))
        .expect("the lying cylinder is in the standard set");
    let result = measure(&case, &[0.1, 0.05, 0.025, 0.0125]);
    assert!(
        result.is_monotone(),
        "the lying cylinder must converge monotonically: {:?}",
        result
            .samples
            .iter()
            .map(|s| s.mesh_error())
            .collect::<Vec<_>>()
    );
    let p = result.exponent().expect("monotone, so fittable");
    assert!(
        p > 1.3,
        "expected roughly h^1.5 from a square-root chord profile, measured {p:.3}"
    );
}

#[test]
fn accuracy_depends_on_the_ratio_and_not_on_the_spacing() {
    // The claim the whole module rests on: a sphere at h/R = 1/40 has the same
    // relative error whether it is 2.5 mm across or 20 mm across. If this ever
    // stops holding, "accuracy depends on feature-size over cell-size" is no
    // longer true and the U6 argument needs rebuilding from scratch.
    let mut errors = Vec::new();
    for radius in [2.5, 5.0, 10.0, 20.0] {
        let case = Case {
            name: "sphere",
            radius,
            // A leaked static is the only way to give `mesh: fn() -> TriMesh` a
            // radius; measuring the four separately would be the same code four
            // times over. The mesh is built once per case either way.
            mesh: match radius as u32 {
                2 => || shapes::icosphere(2.5, 4),
                5 => || shapes::icosphere(5.0, 4),
                10 => || shapes::icosphere(10.0, 4),
                _ => || shapes::icosphere(20.0, 4),
            },
            analytic: None,
            placement: Mat4::IDENTITY,
            model: ErrorModel::Quadrature,
        };
        let result = measure(&case, &[0.025]);
        errors.push(result.samples[0].mesh_error());
    }
    let lo = errors.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = errors.iter().copied().fold(0.0, f64::max);
    assert!(
        (hi - lo) / hi < 1e-6,
        "the same h/R must give the same relative error at every scale: {errors:?}"
    );
}

// --- nightly ---------------------------------------------------------------

#[test]
#[ignore = "the full grid reaches 50M rays; run in release with --ignored"]
fn the_absolute_bound_holds_where_the_cells_are_fine() {
    // The customer-facing accuracy claim, and it is stated WITH the ratio it was
    // measured at. An absolute accuracy number without the ratio is not a claim
    // about anything, because the same 0.05 mm lattice is fine for a 20 mm boss
    // and hopeless for a 0.1 mm fillet.
    for case in standard_cases() {
        let result = measure(
            &case,
            &chipbreaker_core::dexel::convergence::standard_ratios(),
        );
        let finest = result
            .finest_within(1.0 / 200.0)
            .expect("the standard grid reaches h/R = 1/320");
        assert!(
            finest.mesh_error() < 1e-3,
            "{}: at h/R = {:.5} the error is {:.4}%, over the 0.1% bound",
            result.name,
            finest.ratio,
            finest.mesh_error() * 100.0
        );
        // And the envelope on the full grid, not just the quick one.
        let p = envelope_for(&result);
        assert!(result.envelope_constant(p) <= 1.0, "{}", result.name);
    }
}
