// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! How far apart are the `I`/`J`/`K` and `R` arc forms, really?
//!
//! The specification asks that the two forms produce identical IR. They cannot:
//! one is *given* the centre and the other *derives* it through a square root
//! and two divisions, so they agree to within a rounding rather than on the same
//! bits. This measures the rounding across a sweep of angles and scales, so that
//! the bound asserted in the tests is a measured number rather than a guess.
//!
//! Run with: `cargo run -p chipbreaker-gcode --example form_agreement`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::ArcPlane;
use chipbreaker_core::transcendental as t;
use chipbreaker_gcode::arcs::{ArcRequest, DEFAULT_ARC_TOLERANCE, Turn, resolve};
use chipbreaker_gcode::diag::{Diagnostics, Site};

fn main() {
    println!(
        "{:>10} {:>10} {:>14} {:>14}",
        "scale", "cases", "worst centre", "worst sweep"
    );
    let mut overall_centre = 0.0f64;
    let mut overall_sweep = 0.0f64;

    for scale in [1.0f64, 10.0, 100.0, 1000.0] {
        let mut worst_centre = 0.0f64;
        let mut worst_sweep = 0.0f64;
        let mut cases = 0usize;

        for k in 1..40 {
            // Sweeps from just above zero to just below a half turn. Above that
            // the R form needs a negative radius, and near it the form is
            // refused as ill-conditioned.
            let angle = core::f64::consts::PI * f64::from(k) / 40.0;
            let (sin, cos) = t::sin_cos(angle);
            let base = ArcRequest {
                start: Vec3::new(scale, 0.0, 0.0),
                end: Vec3::new(scale * cos, scale * sin, 0.0),
                plane: ArcPlane::Xy,
                turn: Turn::CounterClockwise,
                centre: Some(Vec3::new(0.0, 0.0, 0.0)),
                radius_word: None,
                extra_turns: 0,
                tolerance: DEFAULT_ARC_TOLERANCE,
                site: Site::new(0, 1, 1),
            };
            let mut by_radius = base;
            by_radius.centre = None;
            by_radius.radius_word = Some(scale);

            let mut diagnostics = Diagnostics::new();
            let (Ok(a), Ok(b)) = (
                resolve(&base, &mut diagnostics),
                resolve(&by_radius, &mut diagnostics),
            ) else {
                continue;
            };
            cases += 1;

            let ulp = scale * f64::EPSILON;
            for (u, v) in a.center.to_array().iter().zip(b.center.to_array()) {
                worst_centre = worst_centre.max((u - v).abs() / ulp);
            }
            let sweep_ulp = core::f64::consts::TAU * f64::EPSILON;
            worst_sweep = worst_sweep.max((a.sweep - b.sweep).abs() / sweep_ulp);
        }

        println!("{scale:>10} {cases:>10} {worst_centre:>13.2}u {worst_sweep:>13.2}u");
        overall_centre = overall_centre.max(worst_centre);
        overall_sweep = overall_sweep.max(worst_sweep);
    }

    println!();
    println!(
        "worst across all scales: centre {overall_centre:.2} ULP, sweep {overall_sweep:.2} ULP"
    );
    println!("(the tests assert 32 ULP, which is this with margin)");
}
