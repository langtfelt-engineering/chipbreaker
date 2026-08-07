// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Does the tessellation floor measure what a mesh cannot tell you?
//!
//! ADR 0005: any accuracy metric floors against the fidelity of its input. The
//! floor is only useful if it tracks the input's real error, and the first
//! version did not — it returned the square root of the mean triangle area,
//! which refused to compare anything against a box. A box's faces are planes and
//! twelve triangles represent them exactly; no refinement would improve them. So
//! every prismatic part, which is most machined parts, was refused a comparison
//! it could have answered exactly.
//!
//! What is measured now is the **chord error**: how far the smooth surface a
//! mesh stands for departs from the flat facets standing in for it. That has a
//! closed form on a sphere and a torus, so the estimator can be checked against
//! arithmetic rather than against its own output.
//!
//! `examples/facet_probe.rs` prints the whole table; this pins the properties.

use chipbreaker_core::deviation::facet_size;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;

#[test]
fn a_box_has_no_chord_error_however_large_its_triangles() {
    // The case the first version got wrong, and the reason it mattered.
    for (lo, hi) in [
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 12.0)),
        (Vec3::new(-1.0, -1.0, -1.0), Vec3::new(400.0, 300.0, 120.0)),
        (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.4, 0.3, 0.2)),
    ] {
        let mesh = shapes::box_solid(lo, hi);
        assert_eq!(
            facet_size(&mesh),
            0.0,
            "a box from {lo:?} to {hi:?} is made of planes; its triangles \
             represent them exactly at any size"
        );
    }
}

#[test]
fn refining_a_curved_mesh_lowers_the_floor_it_imposes() {
    // The property a floor has to have. If it did not fall under refinement it
    // would not be measuring tessellation at all.
    let mut previous = f64::INFINITY;
    for level in 1..4u32 {
        let mesh = shapes::icosphere(20.0, level);
        let f = facet_size(&mesh);
        assert!(
            f > 0.0 && f < previous,
            "icosphere level {level} reported {f:.6} mm against {previous:.6} at \
             the level below; refining must lower the floor"
        );
        previous = f;
    }
}

#[test]
fn the_estimate_tracks_the_closed_form_chord_error_within_a_factor_of_two() {
    // **A factor of two, both ways**, which is looser than what is measured. The
    // estimate sits at a steady 1.23 times the closed form across three
    // tessellations of the same torus, and at 1.5 to 1.6 on an icosphere: a
    // consistent, slightly conservative offset rather than noise. The bracket is
    // wide because the constant is a property of how a shape is triangulated,
    // and a new generator would land somewhere else in the same range.
    //
    // Wide enough to be robust and narrow enough to catch a rewrite that changes
    // what is being measured. It is deliberately not asserted as a *bound*: a
    // triangulated torus is not a regular polygon, the formula is exact only for
    // one, and claiming a bound here would be the more comfortable statement and
    // the wrong one.
    let (major, minor, rings) = (40.0, 10.0, 24u32);
    for segments in [16u32, 24, 48, 96] {
        let mesh = shapes::torus(major, minor, segments, rings);
        let f = facet_size(&mesh);

        let sagitta = |radius: f64, count: u32| {
            let half = core::f64::consts::PI / f64::from(count);
            radius * (1.0 - chipbreaker_core::transcendental::cos(half))
        };
        let truth = sagitta(major, segments).max(sagitta(minor, rings));

        println!(
            "torus {segments} seg: estimate {f:.4} mm, closed form {truth:.4} mm, \
             ratio {:.2}",
            f / truth
        );
        assert!(
            f >= truth / 2.0 && f <= truth * 2.0,
            "torus with {segments} segments: the estimate {f:.4} mm is outside a \
             factor of two of the closed-form chord error {truth:.4} mm, so it is \
             no longer tracking the same quantity"
        );
    }
}

#[test]
fn a_coarsely_faceted_solid_is_not_distinguishable_from_a_faceted_design() {
    // **The stated limitation**, pinned so that it is a decision rather than a
    // surprise.
    //
    // The estimator ignores edges above 30 degrees, because a 90 degree corner is
    // a feature the part really has and no refinement will soften it. That
    // threshold corresponds to about twelve segments per revolution. Below it,
    // a coarsely tessellated cylinder and a deliberate dodecagonal boss are the
    // same mesh, and nothing in the file says which was meant.
    //
    // The choice is to believe the part. Treating a genuine 45 degree corner as
    // sampling error would inflate the floor on every chamfered part and refuse
    // comparisons that are exactly answerable -- the same failure the mean-area
    // version had, wearing the opposite hat.
    //
    // The cost is real and is recorded here: a bare icosahedron standing in for
    // a sphere departs from it by 1.3 mm and reports nothing.
    let coarse = shapes::icosphere(20.0, 0);
    assert_eq!(
        facet_size(&coarse),
        0.0,
        "a bare icosahedron's facets meet at 41.8 degrees, above the feature \
         threshold, so it reads as a twenty-sided solid rather than a coarse \
         sphere. If this ever changes, the threshold moved and every chamfered \
         part's floor moved with it."
    );
}
