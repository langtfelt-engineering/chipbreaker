// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The convergence table.
//!
//! Two error columns, because a dexel volume differs from the truth for two
//! independent reasons and one number would hide which. Against the **mesh**
//! isolates dexel sampling error and is what the tests assert on; against the
//! **analytic solid** adds tessellation error and is the number a customer
//! cares about. See `dexel::convergence` for the full argument, including why
//! the axis-parallel cylinder gets an envelope rather than a fitted rate.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-core --example dexel_convergence`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::convergence::{
    ErrorModel, GAUSS_CIRCLE_EXPONENT, measure, standard_cases, standard_ratios,
};

fn main() {
    let ratios = standard_ratios();
    let mut summary = Vec::new();

    for case in standard_cases() {
        let result = measure(&case, &ratios);
        let model = match result.model {
            ErrorModel::Quadrature => "quadrature (chord vanishes continuously)",
            ErrorModel::LatticeCount => "lattice-point counting (chord is a hard indicator)",
        };
        println!("=== {} ===", result.name);
        println!("  error model: {model}");
        println!(
            "  {:>8}  {:>9}  {:>11}  {:>12}  {:>12}  {:>8}",
            "h/R", "h (mm)", "rays", "vs mesh", "vs analytic", "order"
        );
        let orders = result.observed_orders();
        for (k, sample) in result.samples.iter().enumerate() {
            let analytic = sample
                .analytic_error()
                .map_or_else(|| "          --".to_owned(), |e| format!("{:>11.3e}", e));
            // The order shown on a row is the order observed getting TO it from
            // the row above, so the first row has none.
            let order = if k == 0 {
                "      --".to_owned()
            } else {
                format!("{:>8.2}", orders[k - 1])
            };
            println!(
                "  {:>8.5}  {:>9.4}  {:>11}  {:>11.3e}  {analytic}  {order}",
                sample.ratio,
                sample.spacing,
                sample.rays,
                sample.signed_mesh_error(),
            );
        }

        match result.model {
            ErrorModel::Quadrature => {
                let p = result.exponent().expect("a monotone sequence fits");
                println!("  fitted exponent p in error ~ (h/R)^p:  {p:.3}");
                summary.push((result.name.clone(), format!("p = {p:.3}")));
            }
            ErrorModel::LatticeCount => {
                // Deliberately no fitted exponent. Six samples of the Gauss
                // circle error will produce a number, and the number is noise;
                // an earlier pass at this measurement reported 1.91 and a rerun
                // on a different grid gave 1.57.
                let c = result.envelope_constant(GAUSS_CIRCLE_EXPONENT);
                println!("  no fitted rate: the Gauss circle error is erratic. Envelope instead:");
                println!(
                    "    error <= {c:.4} * (h/R)^{GAUSS_CIRCLE_EXPONENT:.5}   \
                     [{GAUSS_CIRCLE_EXPONENT:.5} = 2 - 131/208]"
                );
                summary.push((result.name.clone(), format!("envelope C = {c:.4}")));
            }
        }
        println!(
            "  monotone under refinement: {}",
            if result.is_monotone() {
                "yes"
            } else {
                "NO -- a finer lattice made the answer worse at least once"
            }
        );
        if let Some(sample) = result.finest_within(1.0 / 200.0) {
            println!(
                "  at h <= R/200 (h/R = {:.5}): {:.5}% against the mesh",
                sample.ratio,
                sample.mesh_error() * 100.0
            );
        } else {
            println!("  no sample reached h <= R/200");
        }
        println!();
    }

    println!("--- summary ---");
    for (name, claim) in &summary {
        println!("  {claim:>20}   {name}");
    }
    println!();
    println!("The two cylinder rows are the point, and not for the reason one would");
    println!("guess. The upright cylinder is not merely less accurate than the one lying");
    println!("down -- its error obeys a different law. Its chord is the full height");
    println!("inside the silhouette and zero outside, so the volume is exactly");
    println!("h^2 * H * (lattice points inside the disc), and the error is exactly the");
    println!("Gauss circle problem. That error is erratic: going from h/R = 1/80 to");
    println!("1/160 quadruples the ray count and MORE THAN DOUBLES the error.");
    println!();
    println!("The lying cylinder is a midpoint quadrature of 2*sqrt(R^2 - x^2). Monotone,");
    println!("predictable, about h^1.5.");
    println!();
    println!("So the fix for the upright cylinder is not a finer lattice -- refinement is");
    println!("not even reliably an improvement. It is a bundle along another axis, where");
    println!("that same vertical wall becomes a horizontal surface the rays meet");
    println!("analytically. That is the whole of Unit 6.");
}
