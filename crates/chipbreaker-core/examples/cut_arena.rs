// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The span-count distribution after cutting, which is the number that decides
//! `INLINE_CAPACITY`.
//!
//! The arena was sized at 2 on **stock at rest**, where the distribution is nearly
//! degenerate: one span on every filled ray, and only a genuine internal cavity
//! reaches two. Cutting splits spans, so that measurement was of the wrong
//! population and this one replaces it.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-core --example cut_arena`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::INLINE_CAPACITY;
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::LinearMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri, distribution, spilled};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

fn mill(diameter: f64) -> Profile {
    flat_end_mill(diameter, 20.0, &Shank::plain(diameter, 45.0)).expect("valid")
}

fn stock(spacing: f64) -> TriDexelField {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 40.0, 12.0));
    TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn report(name: &str, field: &TriDexelField) {
    let d = distribution(field);
    let rays: usize = d.values().sum();
    let spill = spilled(field);
    println!("  --- {name} ---");
    for (spans, count) in &d {
        #[allow(clippy::cast_precision_loss, reason = "a percentage of counts")]
        let share = *count as f64 / rays.max(1) as f64 * 100.0;
        let bar = "#".repeat(((share / 2.0).round() as usize).min(50));
        println!("    {spans:>3} span(s)  {count:>9} rays  {share:>6.2}%  {bar}");
    }
    for cap in [1usize, 2, 3, 4, 6] {
        let within: usize = d.iter().filter(|(k, _)| **k <= cap).map(|(_, v)| *v).sum();
        #[allow(clippy::cast_precision_loss, reason = "a percentage of counts")]
        let pct = within as f64 / rays.max(1) as f64 * 100.0;
        let marker = if cap == INLINE_CAPACITY {
            "  <-- current"
        } else {
            ""
        };
        println!("    capacity {cap}: covers {pct:>7.3}% of rays{marker}");
    }
    println!("    spilled rays: {spill}");
    print!("    per bundle:");
    for (axis, bundle) in field.bundles() {
        let d = bundle.arena().distribution();
        let worst = d.keys().copied().max().unwrap_or(0);
        print!(
            "   {}: {} rays, max {} spans, {} spilled",
            axis.as_str(),
            bundle.arena().rays(),
            worst,
            bundle.arena().spilled_rays()
        );
    }
    println!();
    println!();
}

fn main() {
    let spacing = 0.4;
    println!("Span-count distribution across all three bundles, before and after cutting.");
    println!("Cell size {spacing} mm; stock 60 x 40 x 12 mm.");
    println!();

    let field = stock(spacing);
    report(
        "stock at rest (the population the arena was sized on)",
        &field,
    );

    // One pocket: a slot through the middle. Two spans on transverse rays.
    let profile = mill(8.0);
    let mut one_slot = stock(spacing);
    let mut scratch = CutScratch::new(&profile);
    cut_tri(
        &mut one_slot,
        &profile,
        &LinearMove {
            start: Vec3::new(-6.0, 20.0, 4.0),
            end: Vec3::new(66.0, 20.0, 4.0),
        },
        SweepMethod::Reference { steps: 48 },
        &mut scratch,
    );
    report("one slot (a pocket)", &one_slot);

    // A rib: two slots either side of standing material. Three spans.
    let mut rib = stock(spacing);
    for y in [14.0, 26.0] {
        cut_tri(
            &mut rib,
            &profile,
            &LinearMove {
                start: Vec3::new(-6.0, y, -1.0),
                end: Vec3::new(66.0, y, -1.0),
            },
            SweepMethod::Reference { steps: 48 },
            &mut scratch,
        );
    }
    report("two through slots leaving a rib (a boss)", &rib);

    // A comb: five slots, which is the shape that stresses the arena hardest --
    // a ray across them crosses material and air alternately.
    let comb_tool = mill(4.0);
    let mut comb = stock(spacing);
    let mut scratch = CutScratch::new(&comb_tool);
    for k in 0..5 {
        let y = 6.0 + f64::from(k) * 7.0;
        cut_tri(
            &mut comb,
            &comb_tool,
            &LinearMove {
                start: Vec3::new(-4.0, y, -1.0),
                end: Vec3::new(64.0, y, -1.0),
            },
            SweepMethod::Reference { steps: 48 },
            &mut scratch,
        );
    }
    report("five through slots (a comb)", &comb);

    println!("The comb is the adversarial case, not the typical one: a transverse ray");
    println!("crossing five slots carries six spans. Real work sits between the pocket");
    println!("and the rib, but the arena has to survive the comb without being wrong,");
    println!("and the spill map is what makes that a cost rather than a limit.");
}
