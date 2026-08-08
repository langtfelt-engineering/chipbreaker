// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! What `facet_size` reports for shapes of known chord error.
//!
//! The estimator claims to return the chord error of the mesh: how far the
//! smooth surface it stands for departs from the flat facets. For a sphere and a
//! torus that number has a closed form, so the claim is checkable rather than
//! plausible.

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::deviation::facet_size;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;

fn main() {
    println!("{:<28}{:>12}{:>14}", "mesh", "facet_size", "closed form");

    // A box: planes, represented exactly, whatever the triangle size.
    let b = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 12.0));
    println!(
        "{:<28}{:>12.4}{:>14}",
        "box 40x30x12",
        facet_size(&b),
        "0 (exact)"
    );

    for level in 0..4u32 {
        let r = 20.0;
        let m = shapes::icosphere(r, level);
        // Each level quadruples the faces, so the typical edge subtends about
        // half the angle of the level below. The bare icosahedron subtends 41.8
        // degrees between adjacent normals.
        let approx = 41.81_f64.to_radians() / f64::from(1u32 << level);
        let sagitta = r * (1.0 - chipbreaker_core::transcendental::cos(approx / 2.0));
        println!(
            "{:<28}{:>12.4}{:>14.4}",
            format!("icosphere r=20 level {level}"),
            facet_size(&m),
            sagitta
        );
    }

    for segments in [8u32, 12, 16, 24, 48, 96] {
        let (major, minor) = (40.0, 10.0);
        let m = shapes::torus(major, minor, segments, 24);
        // Both directions, because whichever is coarser governs -- and at 96
        // segments it is the 24 rings around the minor radius, not the major.
        let sagitta = |radius: f64, count: u32| {
            let half = core::f64::consts::PI / f64::from(count);
            radius * (1.0 - chipbreaker_core::transcendental::cos(half))
        };
        let sagitta = sagitta(major, segments).max(sagitta(minor, 24));
        println!(
            "{:<28}{:>12.4}{:>14.4}",
            format!("torus R=40 r=10, {segments} seg"),
            facet_size(&m),
            sagitta
        );
    }
}
