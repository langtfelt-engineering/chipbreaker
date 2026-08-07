// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Per-cubic-centimetre memory, isotropic against auto-selected, at equal
//! accuracy.
//!
//! Unit 6 published a per-cm³ table and observed it varying about 5× between a
//! plate and a bar: a plate pays full resolution in the direction where it has
//! almost no extent. Unit 10's question is how much of that is recoverable
//! **without weakening the sampling guarantee**, which is the constraint
//! `auto_spacing` works under.
//!
//! The answer is: some, and much less than the raw 5× spread suggests. The
//! spread is a property of the part's shape, and only the part of it that
//! survives holding `sqrt((hx^2+hy^2+hz^2)/2)` fixed is available to claim.

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::budget::{Budget, Spacing, auto_spacing};

/// The four shapes Unit 6 tabulated.
const PARTS: [(&str, [f64; 3]); 4] = [
    ("cube 40x40x40", [40.0, 40.0, 40.0]),
    ("block 100x60x20", [100.0, 60.0, 20.0]),
    ("plate 200x200x6", [200.0, 200.0, 6.0]),
    ("bar 300x20x20", [300.0, 20.0, 20.0]),
];

fn kib_per_cm3(extents: [f64; 3], spacing: Spacing) -> f64 {
    let bytes = Budget::predict(extents, spacing, 0, false).field_bytes;
    let cm3 = extents[0] * extents[1] * extents[2] / 1000.0;
    #[allow(clippy::cast_precision_loss, reason = "reporting a rate")]
    {
        bytes as f64 / 1024.0 / cm3
    }
}

fn main() {
    let reference = 0.1;
    println!("Per-cm^3 field memory at a fixed sample-distance bound.");
    println!(
        "Reference: --res {reference} mm, bound {:.6} mm. Auto-selection holds that \
         bound exactly.\n",
        Spacing::uniform(reference).sample_distance_bound()
    );
    println!(
        "{:<20}{:>13}{:>13}{:>9}   auto spacings (mm)",
        "part", "iso KiB/cm3", "auto KiB/cm3", "saving"
    );

    let mut worst = f64::INFINITY;
    let mut best = 0.0f64;
    for (name, extents) in PARTS {
        let iso = Spacing::uniform(reference);
        let auto = auto_spacing(extents, reference);
        let a = kib_per_cm3(extents, iso);
        let b = kib_per_cm3(extents, auto);
        let saving = a / b;
        worst = worst.min(saving);
        best = best.max(saving);
        println!(
            "{name:<20}{a:>13.1}{b:>13.1}{saving:>8.2}x   {:.4} {:.4} {:.4}",
            auto.x, auto.y, auto.z
        );
        assert!(
            (auto.sample_distance_bound() - iso.sample_distance_bound()).abs() < 1.0e-9,
            "the bound moved, which the ruling forbids"
        );
    }

    println!();
    println!("Bound held identical in every row (asserted, not merely reported).");
    println!("Saving ranges {worst:.2}x to {best:.2}x.");

    // The Unit 6 observation, restated with the constraint applied.
    let spread_iso = {
        let values: Vec<f64> = PARTS
            .iter()
            .map(|(_, e)| kib_per_cm3(*e, Spacing::uniform(reference)))
            .collect();
        values.iter().fold(0.0f64, |a, v| a.max(*v))
            / values.iter().fold(f64::INFINITY, |a, v| a.min(*v))
    };
    println!(
        "\nUnit 6 observed per-cm^3 varying about 5x across these shapes; measured \
         here at {spread_iso:.2}x isotropic."
    );
    println!(
        "That spread is the part's geometry, not waste. Only the portion that \
         survives holding the bound fixed is recoverable, and that is the {best:.2}x \
         above."
    );
}
