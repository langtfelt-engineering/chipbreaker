// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Memory for three bundles, and whether tri-dexel volume inherits the
//! single-axis non-monotonicity.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-core --example tri_budget`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::dexel::{Arena, Lattice};
use chipbreaker_core::math::{Aabb3, Vec3};
use chipbreaker_core::mesh::shapes;

const SPACINGS: [f64; 4] = [0.4, 0.2, 0.1, 0.05];

/// Ray count and bytes for all three bundles over a box, from the lattice alone.
///
/// Computed rather than built: 0.05 mm over 100 mm is four million rays per
/// bundle and the answer does not depend on what the rays find, since inline
/// slots are allocated per ray whatever is on them. The spill check below
/// confirms the assumption on real geometry.
fn budget(bounds: Aabb3, spacing: f64) -> Option<(usize, usize)> {
    let mut rays = 0usize;
    let mut bytes = 0usize;
    for axis in AXES {
        let lattice = Lattice::new(bounds, spacing, axis).ok()?;
        rays += lattice.ray_count();
        bytes += Arena::new(lattice.ray_count()).bytes();
    }
    Some((rays, bytes))
}

fn memory() {
    println!("=== MEMORY: three bundles ===");
    println!();
    println!("NOT 3x a single bundle, except for a cube. The three bundles cover");
    println!("(WD + DH + HW) / h^2 rays between them -- HALF THE BOUNDING-BOX SURFACE");
    println!("AREA over h^2, not three times one face. The U5 report's `x3 for U6`");
    println!("column was the simplification, and it is wrong for anything but a cube.");
    println!();

    let parts: [(&str, [f64; 3]); 4] = [
        ("plate 100 x 100 x 10", [100.0, 100.0, 10.0]),
        ("block 100 x 100 x 50", [100.0, 100.0, 50.0]),
        ("cube  100 x 100 x 100", [100.0, 100.0, 100.0]),
        ("bar   100 x 100 x 200", [100.0, 100.0, 200.0]),
    ];

    for (name, e) in parts {
        let bounds = Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(e[0], e[1], e[2]));
        let cm3 = e[0] * e[1] * e[2] / 1000.0;
        println!("  {name}  ({cm3:.1} cm^3 of workspace)");
        println!(
            "    {:>8}  {:>14}  {:>11}  {:>14}  {:>16}",
            "h (mm)", "rays (all 3)", "MiB", "KiB per cm^3", "vs one bundle"
        );
        for spacing in SPACINGS {
            match budget(bounds, spacing) {
                Some((rays, bytes)) => {
                    // The Z bundle alone, for the ratio.
                    let one = Lattice::new(bounds, spacing, chipbreaker_core::math::Axis::Z)
                        .map_or(0, |l| Arena::new(l.ray_count()).bytes());
                    println!(
                        "    {spacing:>8}  {rays:>14}  {:>11.1}  {:>14.1}  {:>15.2}x",
                        bytes as f64 / (1024.0 * 1024.0),
                        bytes as f64 / 1024.0 / cm3,
                        bytes as f64 / one as f64,
                    );
                }
                None => println!("    {spacing:>8}  refused: more rays than a u32 can address"),
            }
        }
        println!();
    }

    println!("  Per cm^3, three bundles cost (1/W + 1/D + 1/H) / h^2 times the bytes");
    println!("  per ray. A single bundle costs 1/H alone, which is why a thin plate is");
    println!("  so expensive per cm^3 with one bundle: the thin direction is in the");
    println!("  denominator by itself. With three, the thin direction ALSO supplies the");
    println!("  two cheap bundles, so the plate-to-bar spread narrows sharply.");
    println!();

    // The spill assumption the table rests on.
    println!("  Spill check on real geometry (the only allocation not in the table):");
    println!(
        "    {:<24} {:>12} {:>12} {:>10}",
        "mesh", "rays", "spans", "spilled"
    );
    for (name, mesh, spacing) in [
        (
            "box at rest",
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 40.0, 20.0)),
            0.25,
        ),
        ("torus", shapes::torus(15.0, 4.0, 96, 48), 0.25),
        ("lattice block", shapes::lattice_block(9), 0.25),
    ] {
        let (field, _) = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing_xyz: None,
                spacing,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds");
        let mut spilled = 0;
        for (_, bundle) in field.bundles() {
            spilled += bundle.arena().spilled_rays();
        }
        println!(
            "    {name:<24} {:>12} {:>12} {:>10}",
            field.rays(),
            field.total_spans(),
            spilled
        );
    }
    println!();

    // And the measured figure against the computed one, so the table is not
    // an extrapolation.
    println!("  Measured against computed, on a real build:");
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 50.0));
    for spacing in [0.4, 0.2] {
        let (field, _) = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing_xyz: None,
                spacing,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds");
        let computed = budget(mesh.bounds(), spacing).expect("valid").1;
        println!(
            "    h={spacing}: measured {:.1} MiB, computed {:.1} MiB",
            field.bytes() as f64 / (1024.0 * 1024.0),
            computed as f64 / (1024.0 * 1024.0),
        );
    }
    println!();
}

/// Does averaging three bundles cancel the single-axis oscillation?
///
/// The Unit 6 plan asked out of curiosity. The answer matters either way: if
/// three independent oscillating terms cancel, that is worth recording; if they
/// do not, it strengthens ADR 0005.
fn volume_monotonicity() {
    println!("=== VOLUME MONOTONICITY: does averaging three bundles cancel? ===");
    println!();
    println!("Unit 5's upright cylinder is the case: its volume is exactly a count of");
    println!("lattice points inside a disc, so the error is the Gauss circle problem");
    println!("and it oscillates. Going from h/R = 1/80 to 1/160 quadrupled the rays");
    println!("and MORE THAN DOUBLED the error on a single bundle.");
    println!();

    let radius = 10.0;
    let height = 20.0;
    let mesh = shapes::cylinder(radius, height, 256);
    let truth = mesh.signed_volume().abs();

    println!(
        "  {:>8}  {:>12}  {:>12}  {:>12}  {:>12}",
        "h/R", "X err", "Y err", "Z err", "MEAN err"
    );
    let mut single_monotone = true;
    let mut mean_monotone = true;
    let mut previous_single = f64::INFINITY;
    let mut previous_mean = f64::INFINITY;

    for k in 0..6 {
        let ratio = 1.0 / (10.0 * f64::from(1u32 << k));
        let spacing = ratio * radius;
        let (field, _) = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing_xyz: None,
                spacing,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds");
        let volumes = field.volumes();
        let err = |v: Option<f64>| v.map_or(f64::NAN, |x| (x - truth).abs() / truth);
        let mean_err = (field.volume() - truth).abs() / truth;

        println!(
            "  {ratio:>8.5}  {:>12.3e}  {:>12.3e}  {:>12.3e}  {:>12.3e}",
            err(volumes[0]),
            err(volumes[1]),
            err(volumes[2]),
            mean_err
        );
        // The Z bundle is the axis-parallel one -- the Gauss circle case.
        let single = err(volumes[2]);
        if single > previous_single {
            single_monotone = false;
        }
        if mean_err > previous_mean {
            mean_monotone = false;
        }
        previous_single = single;
        previous_mean = mean_err;
    }
    println!();
    println!("  single bundle (Z, axis-parallel) monotone: {single_monotone}");
    println!("  three-bundle mean monotone:                {mean_monotone}");
    println!();
    if mean_monotone && !single_monotone {
        println!("  The three oscillating terms DO partially cancel: averaging recovered");
        println!("  monotonicity that a single bundle did not have. Pleasant, and worth");
        println!("  recording -- but not a licence to assert on volume. It is one solid");
        println!("  on one grid, and the cancellation is luck rather than structure:");
        println!("  nothing forces three independent errors to have opposite signs.");
    } else if !mean_monotone {
        println!("  The mean is ALSO non-monotone. Three independent oscillating terms do");
        println!("  not cancel, which is what ADR 0005 assumed and now has evidence for.");
        println!("  Averaging a diagnostic three times gives a better diagnostic, not a");
        println!("  metric.");
    } else {
        println!("  Both monotone on this grid, which says little either way: the");
        println!("  interesting case is where the single bundle oscillates.");
    }
    println!();
    println!("  Either way ADR 0005 stands. The cell-quantisation bias alone (every cell");
    println!("  claims a full h^2, so a spacing that does not divide the stock over-counts");
    println!("  by the covered-to-true area ratio) is enough to disqualify volume, and it");
    println!("  is arithmetic rather than luck.");
}

fn main() {
    memory();
    volume_monotonicity();
}
