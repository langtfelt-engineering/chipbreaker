// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Per-ray raycast cost, with the confounds made visible.
//!
//! The benchmark reports throughput per ray *cast*, which includes rays that
//! miss. Tools differ in how much of their bundle hits and in how much of the
//! hitting geometry is a torus, so a bare ratio between two tools can be an
//! artefact of bundle geometry rather than a statement about the solver. This
//! prints the hit rate and the torus share alongside the timing so the ratio can
//! be read honestly.
//!
//! Run with: `cargo run --release -p chipbreaker-core --example raycast_cost`

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::spans::Spans;
use chipbreaker_core::tool::catalog::{
    Shank, ball_end_mill, barrel_end_mill, bull_end_mill, drill, flat_end_mill,
};
use chipbreaker_core::tool::profile::{Profile, ProfileElement};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};
use std::time::Instant;

const SIDE: usize = 48;
const REPEATS: usize = 40;

/// Fraction of the profile's length that is a torus, which is what costs a
/// quartic. An arc centred on the axis is a sphere and only costs a quadratic.
fn torus_share(profile: &Profile) -> f64 {
    let mut torus = 0.0;
    let mut total = 0.0;
    for e in profile.elements() {
        let length = e.element.length();
        total += length;
        if let ProfileElement::Arc { center, .. } = e.element
            && center.x.abs() > 1.0e-9
        {
            torus += length;
        }
    }
    if total > 0.0 { torus / total } else { 0.0 }
}

fn main() {
    let shank = Shank::plain(6.0, 50.0);
    let tools: Vec<(&str, Profile)> = vec![
        ("flat 6mm", flat_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "drill 6mm/118",
            drill(6.0, 118.0, 30.0, &shank).expect("valid"),
        ),
        ("ball 6mm", ball_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "bull 10mm r2",
            bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "bull 10mm r0.5",
            bull_end_mill(10.0, 0.5, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "barrel 12mm R60",
            barrel_end_mill(12.0, 60.0, 40.0, &Shank::plain(12.0, 70.0)).expect("valid"),
        ),
        (
            "barrel 12mm R200",
            barrel_end_mill(12.0, 200.0, 60.0, &Shank::plain(12.0, 90.0)).expect("valid"),
        ),
    ];

    println!(
        "{:<18} {:>10} {:>10} {:>9} {:>10} {:>9}",
        "tool", "ns/ray", "ns/hit", "hit rate", "torus", "vs flat"
    );
    let mut flat_per_ray = 0.0f64;

    for (name, profile) in &tools {
        let cylinder = profile.bounding_cylinder();
        let radius = cylinder.radius * 1.25 + 1.0;
        let mut rays = Vec::with_capacity(SIDE * SIDE);
        for i in 0..SIDE {
            let y = -radius + 2.0 * radius * (i as f64 + 0.5) / SIDE as f64;
            for j in 0..SIDE {
                let z = cylinder.z_min - 0.5
                    + (cylinder.z_max - cylinder.z_min + 1.0) * (j as f64 + 0.5) / SIDE as f64;
                if let Some(ray) =
                    Ray::new_normalized(Vec3::new(-radius - 1.0, y, z), Vec3::new(1.0, 0.0, 0.0))
                {
                    rays.push(ray);
                }
            }
        }

        let mut scratch = RaycastScratch::with_capacity(profile.len());
        let mut spans = Spans::new();
        let mut stats = RaycastStats::default();

        // Warm up, and count how many rays actually meet the tool.
        let mut hits = 0u64;
        for ray in &rays {
            profile.intersect_ray_into(ray, &mut scratch, &mut spans, &mut stats);
            if !spans.is_empty() {
                hits += 1;
            }
        }

        let start = Instant::now();
        let mut sink = 0.0f64;
        for _ in 0..REPEATS {
            for ray in &rays {
                profile.intersect_ray_into(ray, &mut scratch, &mut spans, &mut stats);
                sink += spans.measure();
            }
        }
        let elapsed = start.elapsed().as_secs_f64();
        assert!(sink >= 0.0);

        let casts = (REPEATS * rays.len()) as f64;
        let per_ray = elapsed / casts * 1.0e9;
        let hit_rate = hits as f64 / rays.len() as f64;
        let per_hit = per_ray / hit_rate.max(1.0e-9);
        if *name == "flat 6mm" {
            flat_per_ray = per_ray;
        }

        println!(
            "{name:<18} {per_ray:>10.1} {per_hit:>10.1} {:>8.1}% {:>9.1}% {:>8.2}x",
            hit_rate * 100.0,
            torus_share(profile) * 100.0,
            per_ray / flat_per_ray
        );
    }
}
