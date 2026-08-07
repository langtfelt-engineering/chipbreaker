// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! How far the extracted surface sits from the surface it represents.
//!
//! # This is Unit 9 measuring its own error, which nothing before it bounded
//!
//! Unit 6's two constants describe the *field*: `h * sqrt(3/2)` is how far a
//! ray endpoint can lie from the true surface along its own ray, and `h/sqrt(3)`
//! bounds a **nearest-neighbour** reconstruction. Neither bounds this. Dual
//! contouring does not reconstruct by nearest neighbour; it interpolates between
//! exact endpoints and then solves a least-squares system over their planes, and
//! that should do materially better than either constant on a smooth surface.
//!
//! So the figure is measured rather than inherited, and it is reported in two
//! parts, because one number over the whole surface would be dominated by the
//! edges and say nothing about the 95% of a part that is not an edge:
//!
//! - **Smooth regions**, away from any sharp edge. Here the QEF is rank 1 and
//!   the vertex sits on an interpolated plane.
//! - **Sharp edges**, where two or three faces meet. Here the vertex is pulled
//!   onto the intersection, and the error is a different quantity with a
//!   different bound.
//!
//! # The comparison that justifies four bytes an endpoint, in two halves
//!
//! The same field is extracted twice — once with normals and once with them
//! discarded — and the edge is measured both ways. It is the test that decides
//! whether Unit 9 section 1b was worth its memory, and it belongs here rather
//! than in a one-off measurement because the answer has to keep being true.
//!
//! It is run on **two** geometries, and the reason is a defect that lived for
//! five units. A field built from a mesh takes every endpoint normal from the
//! triangle its ray crossed; a field that has been *cut* takes them from the
//! tool. Only the first was ever implemented, so every cut face in the engine
//! carried `(0, 0, -1)` and this test — run on an uncut box — passed throughout
//! without being able to notice.
//!
//! | | `..._only_when_normals_are_stored` | `..._on_cut_geometry_too` |
//! |---|---|---|
//! | shape | an uncut block | a slot cut through a block |
//! | normals from | construction | `tool::normal` |
//! | with normals | exact | exact |
//! | without | 0.167 mm worst | 0.125 mm worst |
//!
//! Both are published. Either alone overstates what the four bytes are known to
//! buy: the first because it never exercised the sweep, the second because a
//! part is mostly not cut faces. See ADR 0010.

use chipbreaker_core::contour::{ContourOptions, extract};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};

fn field_from(mesh: &TriMesh, spacing: f64) -> TriDexelField {
    TriDexelField::build(
        mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// Signed distance from `p` to a sphere of radius `r` about `c`.
fn sphere_distance(p: Vec3, c: Vec3, r: f64) -> f64 {
    let d = Vec3::new(p.x - c.x, p.y - c.y, p.z - c.z);
    (d.x * d.x + d.y * d.y + d.z * d.z).sqrt() - r
}

/// Distance from `p` to the surface of an axis-aligned box.
///
/// The exact unsigned distance, inside and out, so a vertex that has drifted
/// into the solid is measured as honestly as one that has drifted out.
fn box_surface_distance(p: Vec3, lo: Vec3, hi: Vec3) -> f64 {
    let q = [
        (lo.x - p.x).max(p.x - hi.x),
        (lo.y - p.y).max(p.y - hi.y),
        (lo.z - p.z).max(p.z - hi.z),
    ];
    let outside = [q[0].max(0.0), q[1].max(0.0), q[2].max(0.0)];
    let outside_len =
        (outside[0] * outside[0] + outside[1] * outside[1] + outside[2] * outside[2]).sqrt();
    let inside = q[0].max(q[1]).max(q[2]).min(0.0);
    // Outside the box the distance is the length of the positive part; inside it
    // is the distance to the nearest face, which is the largest (least negative)
    // component.
    if outside_len > 0.0 {
        outside_len
    } else {
        -inside
    }
}

/// The worst and the root-mean-square of a set of deviations.
fn worst_and_rms(values: &[f64]) -> (f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0);
    }
    let worst = values.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    let sum: f64 = values.iter().map(|v| v * v).sum();
    #[allow(clippy::cast_precision_loss, reason = "a sample count")]
    let rms = (sum / values.len() as f64).sqrt();
    (worst, rms)
}

#[test]
fn a_sphere_is_reconstructed_well_inside_the_unit_6_bounds() {
    // A sphere has no sharp features at all, so every vertex is in the smooth
    // regime and the whole surface is one measurement.
    let (centre, radius) = (Vec3::new(0.0, 0.0, 0.0), 8.0);
    let spacing = 0.4;
    let mesh = shapes::icosphere(radius, 4);
    let field = field_from(&mesh, spacing);
    let (extracted, _) = extract(&field, &ContourOptions::default()).expect("extracts");

    let deviations: Vec<f64> = extracted
        .vertices()
        .iter()
        .map(|v| sphere_distance(*v, centre, radius))
        .collect();
    let (worst, rms) = worst_and_rms(&deviations);

    // The two Unit 6 constants, for scale.
    let lateral = spacing * (3.0f64 / 2.0).sqrt();
    let perpendicular = spacing / 3.0f64.sqrt();
    println!(
        "sphere r={radius} at h={spacing}: worst {worst:.6} mm, rms {rms:.6} mm; \
         h*sqrt(3/2) = {lateral:.6}, h/sqrt(3) = {perpendicular:.6}"
    );
    println!(
        "  worst is {:.3} h, rms is {:.3} h",
        worst / spacing,
        rms / spacing
    );

    // The tessellation is a subdivided icosahedron, so the "analytic" sphere is
    // itself approximated by the source mesh; that floor is included here and is
    // why the bound is not tighter.
    assert!(
        worst < perpendicular,
        "worst deviation {worst:.6} mm should beat the nearest-neighbour bound \
         h/sqrt(3) = {perpendicular:.6} mm, since interpolating between exact \
         endpoints is strictly better than taking the nearer one"
    );
    assert!(
        rms < perpendicular / 3.0,
        "rms {rms:.6} mm is not comfortably inside the bound"
    );
}

#[test]
fn a_box_is_reconstructed_with_its_faces_and_edges_measured_apart() {
    // A box is all flats and sharp edges, so it separates the two regimes
    // cleanly: a vertex is "near an edge" if it is within a cell of one.
    let (lo, hi) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(16.0, 12.0, 8.0));
    let spacing = 0.4;
    let mesh = shapes::box_solid(lo, hi);
    let field = field_from(&mesh, spacing);
    let (extracted, stats) = extract(&field, &ContourOptions::default()).expect("extracts");

    let near_edge = |p: Vec3| -> bool {
        // How many of the three axes are within a cell of a face. Two or three
        // means an edge or a corner.
        let close = [
            (p.x - lo.x).abs().min((p.x - hi.x).abs()) < spacing,
            (p.y - lo.y).abs().min((p.y - hi.y).abs()) < spacing,
            (p.z - lo.z).abs().min((p.z - hi.z).abs()) < spacing,
        ];
        close.iter().filter(|c| **c).count() >= 2
    };

    let mut smooth = Vec::new();
    let mut sharp = Vec::new();
    for v in extracted.vertices() {
        let d = box_surface_distance(*v, lo, hi);
        if near_edge(*v) {
            sharp.push(d);
        } else {
            smooth.push(d);
        }
    }
    let (smooth_worst, smooth_rms) = worst_and_rms(&smooth);
    let (sharp_worst, sharp_rms) = worst_and_rms(&sharp);
    println!(
        "box at h={spacing}: smooth worst {smooth_worst:.6} rms {smooth_rms:.6} \
         over {} vertices; sharp worst {sharp_worst:.6} rms {sharp_rms:.6} over \
         {} vertices",
        smooth.len(),
        sharp.len()
    );
    println!(
        "  ranks: {} flat, {} edge, {} corner",
        stats.rank_histogram[1], stats.rank_histogram[2], stats.rank_histogram[3]
    );

    assert!(
        !smooth.is_empty() && !sharp.is_empty(),
        "both regimes needed"
    );
    // A planar face should be reconstructed almost exactly: the crossings lie on
    // the plane and the QEF has nothing to trade off.
    assert!(
        smooth_worst < spacing / 4.0,
        "a flat face should be reconstructed to well under a quarter cell, got \
         {smooth_worst:.6} mm at h={spacing}"
    );
    // An edge is harder and is allowed more, but must still be inside a cell.
    assert!(
        sharp_worst < spacing,
        "an edge vertex strayed {sharp_worst:.6} mm, more than the {spacing} mm cell"
    );
    assert!(
        stats.rank_histogram[2] + stats.rank_histogram[3] > 0,
        "a box has twelve edges and eight corners; the QEF found none"
    );
}

#[test]
fn sharp_edges_survive_only_when_normals_are_stored() {
    // **Half of the measurement that justifies four bytes an endpoint**, and the
    // half that was mistaken for the whole of it for five units.
    //
    // Every normal here comes from **construction**: the field is built from a
    // mesh and never cut, so each endpoint takes the triangle normal of the
    // facet its ray crossed. That path was always correct. The other path --
    // the analytic tool surface normal during a cut -- was not implemented at
    // all until Unit 12, so this test passed throughout on a field that could
    // not exercise the defect.
    //
    // `sharp_edges_survive_on_cut_geometry_too` is the other half, and both
    // numbers are published because only together do they say what four bytes
    // an endpoint actually buys.
    let (lo, hi) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(16.0, 12.0, 8.0));
    let spacing = 0.5;
    let mesh = shapes::box_solid(lo, hi);
    let field = field_from(&mesh, spacing);

    let measure = |use_normals: bool| -> (f64, f64, [u64; 4]) {
        let (extracted, stats) = extract(
            &field,
            &ContourOptions {
                use_normals,
                ..ContourOptions::default()
            },
        )
        .expect("extracts");
        // Only the vertices near an edge: away from one, both methods are
        // equally good and including them would dilute the comparison.
        let mut edge_error = Vec::new();
        for p in extracted.vertices() {
            let close = [
                (p.x - lo.x).abs().min((p.x - hi.x).abs()) < spacing,
                (p.y - lo.y).abs().min((p.y - hi.y).abs()) < spacing,
                (p.z - lo.z).abs().min((p.z - hi.z).abs()) < spacing,
            ];
            if close.iter().filter(|c| **c).count() >= 2 {
                edge_error.push(box_surface_distance(*p, lo, hi));
            }
        }
        let (worst, rms) = worst_and_rms(&edge_error);
        (worst, rms, stats.rank_histogram)
    };

    let (with_worst, with_rms, with_ranks) = measure(true);
    let (without_worst, without_rms, without_ranks) = measure(false);

    println!("edge fidelity on a {spacing} mm grid, box edges only:");
    println!("  with normals:    worst {with_worst:.6} mm, rms {with_rms:.6} mm");
    println!("  without normals: worst {without_worst:.6} mm, rms {without_rms:.6} mm");
    println!("  rank histogram with normals:    {with_ranks:?}");
    println!("  rank histogram without normals: {without_ranks:?}");
    // Printed as a ratio only when there is a ratio to print. Dividing by a
    // floor of 1e-12 to avoid an infinity yields "126361907901x", which is not a
    // measurement of anything.
    if with_rms > 0.0 {
        println!(
            "  improvement: {:.2}x on worst, {:.2}x on rms",
            without_worst / with_worst,
            without_rms / with_rms
        );
    } else {
        println!("  improvement: with normals the edge is reconstructed exactly");
    }

    // Without normals every system is rank 0 -- no planes at all -- so the
    // solver returns the centroid every time. That is the definition of surface
    // nets, and it is what makes the control a control.
    assert_eq!(
        without_ranks[1] + without_ranks[2] + without_ranks[3],
        0,
        "discarding normals must leave the QEF with no constrained directions; \
         got ranks {without_ranks:?}"
    );
    assert!(
        with_ranks[2] + with_ranks[3] > 0,
        "with normals a box must produce edge and corner vertices"
    );
    assert!(
        with_rms < without_rms,
        "storing normals must improve edge fidelity, or section 1b bought \
         nothing: with {with_rms:.6} mm rms against without {without_rms:.6} mm"
    );
}

#[test]
fn sharp_edges_survive_on_cut_geometry_too() {
    // **The other half of the four-byte measurement, on the geometry that
    // matters.**
    //
    // The uncut test above takes every normal from construction, and construction
    // was never the broken path. A cut face's normal comes from the tool, and
    // until Unit 12 the sweep set none at all -- so the claim that four bytes buy
    // sharp features had, for five units, been demonstrated only where it was
    // never in doubt.
    //
    // The shape here is a through slot in a block. Its edges are the ones a
    // machinist measures: the two top rims where the slot meets the face, and the
    // two bottom fillets where the walls meet the floor. Every one of them is a
    // **cut** edge, so every normal in play comes from `tool::normal`.
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::sweep::{LinearMove, Motion};
    use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

    let (lo, hi) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(24.0, 18.0, 10.0));
    let spacing = 0.5;
    let floor = 6.0;
    let (centre_y, radius) = (9.0, 3.0);

    let mut field = field_from(&shapes::box_solid(lo, hi), spacing);
    let profile =
        flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid");
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        &[Motion::Linear(LinearMove {
            // Right through, so the slot has no rounded ends to complicate the
            // planes: two walls, a floor, and four straight edges.
            start: Vec3::new(-4.0, centre_y, floor),
            end: Vec3::new(28.0, centre_y, floor),
        })],
        SweepMethod::Analytic {
            tolerance: spacing / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );

    // Distance to the exact surface of the slotted block, which is a plane in
    // every direction that matters here.
    let wall = |y: f64| {
        (y - (centre_y - radius))
            .abs()
            .min((y - (centre_y + radius)).abs())
    };
    let surface = |p: Vec3| -> f64 {
        // Inside the slot's width the surface is the floor; outside it, the top.
        if (p.y - centre_y).abs() <= radius {
            (p.z - floor).abs().min(wall(p.y))
        } else {
            (p.z - hi.z).abs().min(wall(p.y))
        }
    };

    let measure = |use_normals: bool| -> (f64, f64, [u64; 4]) {
        let (extracted, stats) = extract(
            &field,
            &ContourOptions {
                use_normals,
                ..ContourOptions::default()
            },
        )
        .expect("extracts");
        // Only vertices on one of the four slot edges: within a cell of a wall
        // and within a cell of either the floor or the top face. Away from an
        // edge both methods do equally well, and including those would dilute
        // the comparison exactly as it would on the box.
        let mut edge_error = Vec::new();
        for p in extracted.vertices() {
            let near_wall = wall(p.y) < spacing;
            let near_step = (p.z - floor).abs() < spacing || (p.z - hi.z).abs() < spacing;
            // Away from the ends of the block, so the stock's own construction
            // edges cannot creep into a measurement about cut ones.
            let interior = p.x > lo.x + 2.0 && p.x < hi.x - 2.0;
            if near_wall && near_step && interior {
                edge_error.push(surface(*p));
            }
        }
        let (worst, rms) = worst_and_rms(&edge_error);
        assert!(
            edge_error.len() > 100,
            "only {} vertices landed on a cut edge; the filter matched almost \
             nothing and the comparison would mean nothing",
            edge_error.len()
        );
        (worst, rms, stats.rank_histogram)
    };

    let (with_worst, with_rms, with_ranks) = measure(true);
    let (without_worst, without_rms, without_ranks) = measure(false);

    println!("edge fidelity on a {spacing} mm grid, CUT slot edges only:");
    println!("  with normals:    worst {with_worst:.6} mm, rms {with_rms:.6} mm");
    println!("  without normals: worst {without_worst:.6} mm, rms {without_rms:.6} mm");
    println!("  rank histogram with normals:    {with_ranks:?}");
    println!("  rank histogram without normals: {without_ranks:?}");
    // Printed as a ratio only when there is a ratio to print. Dividing by a
    // floor of 1e-12 to avoid an infinity yields "126361907901x", which is not a
    // measurement of anything.
    if with_rms > 0.0 {
        println!(
            "  improvement: {:.2}x on worst, {:.2}x on rms",
            without_worst / with_worst,
            without_rms / with_rms
        );
    } else {
        println!("  improvement: with normals the edge is reconstructed exactly");
    }

    assert_eq!(
        without_ranks[1] + without_ranks[2] + without_ranks[3],
        0,
        "discarding normals must leave the QEF with no constrained directions; \
         got ranks {without_ranks:?}"
    );
    assert!(
        with_ranks[2] + with_ranks[3] > 0,
        "a slot has four long edges and the corners where they meet the ends; \
         with normals the QEF found none: {with_ranks:?}"
    );
    assert!(
        with_rms < without_rms,
        "on cut geometry, storing normals must improve edge fidelity: with \
         {with_rms:.6} mm rms against without {without_rms:.6} mm. This is the \
         assertion the uncut version could not make, because an uncut field \
         takes every normal from construction and never exercises the sweep."
    );
}

#[test]
fn extraction_of_an_uncut_field_round_trips_to_the_source_mesh() {
    // Not identity -- the field is a sampling, and a round trip through it
    // cannot return the original triangles. What it can do is stay within the
    // sampling error, and that catches whole classes of indexing mistake: a
    // transposed axis, an off-by-one in the corner grid, or a bundle read with
    // another bundle's lattice would all move the surface far more than `h`.
    let (lo, hi) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(14.0, 10.0, 6.0));
    let spacing = 0.35;
    let source = shapes::box_solid(lo, hi);
    let field = field_from(&source, spacing);
    let (extracted, _) = extract(&field, &ContourOptions::default()).expect("extracts");

    let deviations: Vec<f64> = extracted
        .vertices()
        .iter()
        .map(|v| box_surface_distance(*v, lo, hi))
        .collect();
    let (worst, rms) = worst_and_rms(&deviations);
    let bound = spacing * (3.0f64 / 2.0).sqrt();
    println!(
        "round trip at h={spacing}: worst {worst:.6} mm, rms {rms:.6} mm, Unit 6 \
         lateral bound {bound:.6} mm"
    );
    assert!(
        worst < bound,
        "a round trip strayed {worst:.6} mm, past the Unit 6 lateral bound of \
         {bound:.6} mm -- that is an indexing error, not sampling"
    );

    // And the volume, which an axis transposition would preserve but a
    // mis-scaled lattice would not.
    let expected = (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z);
    let got = extracted.signed_volume();
    let relative = (got - expected).abs() / expected;
    println!(
        "  volume {got:.4} against {expected:.4}, {:.4}% out",
        relative * 100.0
    );
    assert!(
        relative < 0.02,
        "extracted volume {got:.4} against {expected:.4}: {:.3}% out",
        relative * 100.0
    );
}

#[test]
fn refining_the_grid_reduces_the_deviation_until_it_meets_the_tessellation() {
    // **Deviation floors against tessellation, exactly as volume does.**
    //
    // ADR 0005 rejected volume as an accuracy metric partly because it floors
    // out against the source tessellation. Deviation is the metric instead, and
    // it is monotone where volume is not -- but it has the same floor, and this
    // test found it by failing.
    //
    // Measured on a subdivision-4 icosphere of radius 8, whose facets are about
    // 0.4 mm across:
    //
    // ```text
    // h = 0.8   rms 0.012246 mm
    // h = 0.4   rms 0.003464 mm
    // h = 0.2   rms 0.005297 mm   <- rising
    // ```
    //
    // Nothing is wrong at h = 0.2. The grid has become finer than the source
    // mesh's own facets, so it is faithfully reproducing a faceted polyhedron
    // while the test measures against an ideal sphere. The deviation it reports
    // is the tessellation's, not the field's.
    //
    // The lesson generalises and is recorded once, in ADR 0005: **any accuracy
    // metric floors against the fidelity of its input**, and past that point it
    // measures the mesher rather than the pipeline. Volume floors that way,
    // deviation floors that way, and Unit 12's comparison against a nominal part
    // will floor that way twice -- once for each mesh it is given.
    //
    // A subdivision-5 sphere has facets near 0.2 mm, which keeps all three rungs
    // above the floor.
    let (centre, radius) = (Vec3::new(0.0, 0.0, 0.0), 8.0);
    let mesh = shapes::icosphere(radius, 5);
    let mut previous = f64::INFINITY;
    for spacing in [0.8, 0.4, 0.2] {
        let field = field_from(&mesh, spacing);
        let (extracted, _) = extract(&field, &ContourOptions::default()).expect("extracts");
        let deviations: Vec<f64> = extracted
            .vertices()
            .iter()
            .map(|v| sphere_distance(*v, centre, radius))
            .collect();
        let (worst, rms) = worst_and_rms(&deviations);
        println!("h={spacing}: worst {worst:.6} mm, rms {rms:.6} mm");
        assert!(
            rms < previous,
            "refining from the previous step did not reduce the rms deviation:              {rms:.6} against {previous:.6}. If this fires at the finest rung,              check whether the grid has passed the tessellation -- see the note              above before assuming it is a regression."
        );
        previous = rms;
    }
}
