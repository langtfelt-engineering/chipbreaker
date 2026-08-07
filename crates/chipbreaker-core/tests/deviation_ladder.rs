// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Three rungs, in order, before the deviation field is trusted at all.
//!
//! Comparing a field against a mesh extracted from that same field conflates
//! extraction error with comparison error, so a failure cannot be localised.
//! These rungs separate them, and each is only meaningful once the one below it
//! is clean.
//!
//! 1. **Field against the mesh it was built from.** Span endpoints are *exact*
//!    ray-surface intersections, so every sample must be exactly zero. No
//!    extraction is involved, which isolates the comparison arithmetic
//!    completely: this either passes or hands over the bug.
//! 2. **Field against `extract(field)`.** Must land inside Unit 9's measured
//!    extraction error, which is sub-cell. A failure here is in the field-to-mesh
//!    interface -- containment, sidedness, placement -- and not in rung 1's
//!    arithmetic.
//! 3. **Field against an independently modelled nominal.** The real test, and
//!    only meaningful once the first two are clean.

use chipbreaker_core::contour::{ContourOptions, extract};
use chipbreaker_core::deviation::compare;
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};

const SPACING: f64 = 0.4;

fn field_from(mesh: &TriMesh) -> TriDexelField {
    TriDexelField::build(
        mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

#[test]
fn rung_1_a_field_against_its_own_source_mesh_is_exactly_zero() {
    // The purest form of "the same solid on both sides". A span endpoint is an
    // exact ray-surface intersection with this very mesh, so the signed distance
    // from it to that mesh is zero by construction -- not nearly zero, zero.
    // Anything else is the comparison arithmetic being wrong, with no extraction
    // or resolution effect able to explain it away.
    for (name, mesh) in [
        ("box", shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0))),
        ("sphere", shapes::icosphere(8.0, 3)),
        ("torus", shapes::torus(8.0, 3.0, 48, 24)),
    ] {
        let field = field_from(&mesh);
        let d = compare(&field, &mesh, Some(&mesh));
        let worst = d.worst_gouge_mm.max(d.worst_excess_mm);
        println!(
            "{name}: {} samples, worst gouge {:.6}, worst excess {:.6}, rms {:.6}",
            d.samples.len(),
            d.worst_gouge_mm,
            d.worst_excess_mm,
            d.rms_mm
        );
        assert!(
            d.samples.len() > 1000,
            "{name}: too few samples to mean anything"
        );
        assert!(
            worst < 1.0e-6,
            "{name}: a field compared against the very mesh its endpoints lie on \
             reported {worst:.6} mm. Endpoints are exact ray-surface \
             intersections, so this is the comparison arithmetic and nothing else."
        );
    }
}

#[test]
fn rung_2_a_field_against_its_own_extraction_is_sub_cell() {
    // Only meaningful if rung 1 passes. A failure here is in the field-to-mesh
    // interface rather than in the arithmetic.
    for (name, mesh) in [
        ("box", shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0))),
        ("sphere", shapes::icosphere(8.0, 3)),
    ] {
        let field = field_from(&mesh);
        let extracted = extract(&field, &ContourOptions::default()).expect("extracts").0;
        let d = compare(&field, &extracted, Some(&mesh));
        let worst = d.worst_gouge_mm.max(d.worst_excess_mm);
        println!(
            "{name}: worst gouge {:.6}, worst excess {:.6}, rms {:.6}, cell {SPACING}",
            d.worst_gouge_mm, d.worst_excess_mm, d.rms_mm
        );
        assert!(
            worst < SPACING,
            "{name}: {worst:.4} mm against a {SPACING} mm cell. Unit 9 measured \
             extraction as exact on flats and inside one cell on edges, so this \
             is the field-to-mesh interface -- containment, sidedness or \
             placement -- not a resolution limit."
        );
    }
}

#[test]
fn rung_2b_a_cut_field_against_its_own_extraction_is_sub_cell() {
    // Rungs 1 and 2 pass on uncut shapes, so the arithmetic and the field-to-mesh
    // interface are both sound. The failing case differs in one way: the field
    // has been CUT. This isolates that.
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::sweep::{LinearMove, Motion};
    use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 12.0));
    let profile = flat_end_mill(6.0, 30.0, &Shank::plain(6.0, 60.0)).expect("valid");
    let motions = vec![
        Motion::Linear(LinearMove {
            start: Vec3::new(6.0, 15.0, 16.0),
            end: Vec3::new(6.0, 15.0, 7.0),
        }),
        Motion::Linear(LinearMove {
            start: Vec3::new(6.0, 15.0, 7.0),
            end: Vec3::new(34.0, 15.0, 7.0),
        }),
        Motion::Linear(LinearMove {
            start: Vec3::new(34.0, 15.0, 7.0),
            end: Vec3::new(34.0, 15.0, 16.0),
        }),
    ];
    let mut field = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );

    let extracted = extract(&field, &ContourOptions::default()).expect("extracts").0;
    let d = compare(&field, &extracted, Some(&mesh));
    let worst = d.worst_gouge_mm.max(d.worst_excess_mm);
    let over = d.samples.iter().filter(|s| s.signed_mm.abs() > SPACING).count();
    println!(
        "cut field: {} samples, {over} above one cell, worst gouge {:.4}, worst \
         excess {:.4}, rms {:.4}",
        d.samples.len(),
        d.worst_gouge_mm,
        d.worst_excess_mm,
        d.rms_mm
    );
    // Print the worst few, with where they are, so a failure localises itself.
    let mut sorted: Vec<_> = d.samples.iter().collect();
    sorted.sort_by(|a, b| {
        b.signed_mm
            .abs()
            .partial_cmp(&a.signed_mm.abs())
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    for s in sorted.iter().take(5) {
        println!(
            "  {:+.4} mm at ({:.2}, {:.2}, {:.2}) normal ({:.2}, {:.2}, {:.2}) axis {}",
            s.signed_mm, s.at.x, s.at.y, s.at.z, s.normal.x, s.normal.y, s.normal.z, s.axis
        );
    }
    assert!(
        worst < SPACING,
        "a CUT field against its own extraction reported {worst:.4} mm at a \
         {SPACING} mm cell, while the uncut cases were exact. The defect is \
         specific to cut geometry."
    );
}
