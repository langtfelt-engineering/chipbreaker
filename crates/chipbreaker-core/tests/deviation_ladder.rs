// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! A ladder, climbed in order, before the deviation field is trusted at all.
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
//! 3. **The same, on a field that has been CUT** (rung 2b). The rung that found
//!    the defect. Rungs 1 and 2 were clean on uncut shapes, so cutting was the
//!    one thing left that differed.
//! 4. **Are the cut faces' normals geometry at all?** (rung 2c). Narrowing the
//!    one above: a slot's walls face four ways, so a single shared normal cannot
//!    have come from the shape. They did share one, and the sweep had never set
//!    any.
//! 5. **Field against an independently modelled nominal** (rung 3). The real
//!    test, and only meaningful once the rest are clean.
//!
//! # Rung 3's nominal is written out by hand, and checked before it is believed
//!
//! Every other rung compares the engine against itself on purpose, to keep a
//! failure localised. Rung 3 gives that up: [`slotted_block`] is coordinates and
//! nothing else, so anything it reports is real.
//!
//! A fixture that ambitious needs its own check, or a mistake in it sends the
//! search into the engine — the most expensive possible place to look for a
//! mistake that is not there. [`the_hand_built_nominal_is_the_solid_it_claims_to_be`]
//! validates it for manifoldness and winding and against its closed-form volume,
//! and it earned its keep immediately: the first version came out inside out,
//! with the volume exactly right and negative.
//!
//! Rung 3 then reports **exactly zero**, which is the correct answer — a flat
//! mill driven straight through a block sweeps precisely the slotted solid — and
//! is indistinguishable from a comparison that has fallen asleep. So
//! [`rung_3_reports_a_nominal_that_is_wrong`] runs it again against a nominal a
//! millimetre out and requires the millimetre back, with the right sign.

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

#[test]
fn rung_2c_cut_faces_all_point_the_same_way_which_cannot_be_geometry() {
    // Narrowing rung 2b. The grazing explanation predicts SOME endpoints are
    // wrong; a simpler cause predicts they all are.
    //
    // Counting placeholders is the obvious check and it does not work, because
    // `PLACEHOLDER` IS `+Z` -- the Unit 9 encoding has no reserved pattern on
    // purpose -- so every up-facing endpoint of an uncut box already counts as
    // one. The direct question is better: a slot has walls facing four different
    // directions, so if every one of its cut endpoints shares a single normal,
    // that normal cannot have come from the geometry.
    use chipbreaker_core::dexel::tri::AXES;
    use chipbreaker_core::math::Axis;
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::sweep::{LinearMove, Motion};
    use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 12.0));
    let mut field = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let profile = flat_end_mill(6.0, 30.0, &Shank::plain(6.0, 60.0)).expect("valid");
    let motions = vec![Motion::Linear(LinearMove {
        start: Vec3::new(6.0, 15.0, 7.0),
        end: Vec3::new(34.0, 15.0, 7.0),
    })];
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

    // The X bundle at the slot's floor height sees the slot's two END walls,
    // which face -X and +X. Their endpoint normals must differ from each other.
    let bundle = field.bundle(Axis::X).expect("x bundle");
    let lattice = bundle.lattice().clone();
    let mut wall_normals = Vec::new();
    let rays = u32::try_from(bundle.arena().rays()).expect("small");
    for r in 0..rays {
        let (i, j) = lattice.coords(r);
        let o = lattice.origin_of(i, j);
        // A ray ABOVE the slot floor and on its centre line, so it crosses the
        // slot and meets both end walls. Below the floor the material is
        // continuous and there are no wall endpoints to look at, which the first
        // attempt at this filter got wrong.
        if (o.y - 15.0).abs() > 0.3 || o.z < 8.0 || o.z > 10.0 {
            continue;
        }
        for s in bundle.arena().get(r) {
            for n in [s.n0, s.n1] {
                wall_normals.push(n.decode());
            }
        }
    }
    assert!(
        wall_normals.len() >= 8,
        "expected endpoints on both slot end walls, got {}",
        wall_normals.len()
    );
    println!("X-bundle endpoint normals ({}):", wall_normals.len());
    for n in wall_normals.iter().take(6) {
        println!("  ({:.3}, {:.3}, {:.3})", n.x, n.y, n.z);
    }

    // Every surface an X-bundle ray meets here faces +/-X: the two outer stock
    // faces and the slot's two end walls. A Z-facing normal on any of them is
    // impossible geometrically.
    //
    // The first version of this test asked whether ALL the normals matched, and
    // passed -- vacuously, because the outer faces carry correct normals from
    // construction and only the CUT ones are wrong. A passing test that cannot
    // fail is worse than a missing one, and that is now the second time in this
    // unit.
    let z_facing = wall_normals.iter().filter(|n| n.z.abs() > 0.5).count();
    assert_eq!(
        z_facing,
        0,
        "{z_facing} of {} endpoints on an X-bundle ray point along Z, including          the slot's two end walls, which face opposite directions and share one          normal. That is not geometry -- it is the placeholder normal negated by          the subtraction. Unit 9 said the normal is free at both sites an          endpoint is born, the triangle normal during construction and the          analytic TOOL SURFACE normal during a cut. Only the first was ever          implemented: `sweep` and `tool::raycast` set no normal at all, so every          cut face in the engine carries (0, 0, -1).",
        wall_normals.len()
    );
    let _ = AXES;
}

/// A block with a rectangular channel milled right through it, modelled by hand.
///
/// **The point is that nothing in the engine produced this.** Rungs 1 and 2
/// compare a field against a mesh derived from that same field, which removes
/// the engine's own error from both sides on purpose so that a failure localises.
/// Rung 3 gives up that convenience: the nominal here is written out as
/// coordinates, so anything the comparison reports is real.
///
/// The cross-section is a twelve-sided polygon in `(y, z)`, extruded along `x`.
/// Twelve rather than eight because the cap has to be triangulated without
/// T-junctions: splitting the slab under the channel at the channel's own walls
/// means every interior edge is shared by exactly two triangles, which is what
/// makes the result manifold. Getting that wrong produces a mesh that looks right
/// and leaks under a ray cast.
fn slotted_block(size: Vec3, slot_y: (f64, f64), floor: f64) -> TriMesh {
    use chipbreaker_core::mesh::MeshMeta;
    let (ya, yb) = slot_y;
    let (x, y, z) = (size.x, size.y, size.z);

    // Counter-clockwise in `(y, z)`, with `y` to the right and `z` up.
    let section = [
        (0.0, 0.0),
        (ya, 0.0),
        (yb, 0.0),
        (y, 0.0),
        (y, floor),
        (y, z),
        (yb, z),
        (yb, floor),
        (ya, floor),
        (ya, z),
        (0.0, z),
        (0.0, floor),
    ];
    let n = section.len();

    // Two copies of the section, at each end of the extrusion.
    let mut vertices = Vec::with_capacity(2 * n);
    for (vy, vz) in section {
        vertices.push(Vec3::new(0.0, vy, vz));
    }
    for (vy, vz) in section {
        vertices.push(Vec3::new(x, vy, vz));
    }

    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let idx = |i: usize| u32::try_from(i).expect("small");

    // The sides, one quad per edge of the section.
    for i in 0..n {
        let j = (i + 1) % n;
        let (a, b, c, d) = (idx(i), idx(j), idx(j + n), idx(i + n));
        triangles.push([a, b, c]);
        triangles.push([a, c, d]);
    }

    // The two caps. Five rectangles, chosen so that every edge they introduce is
    // shared by exactly two of them or lies on the section's boundary.
    let quads = [
        [0, 1, 8, 11],  // under the channel, left of it
        [1, 2, 7, 8],   // under the channel
        [2, 3, 4, 7],   // under the channel, right of it
        [11, 8, 9, 10], // the left rail
        [7, 4, 5, 6],   // the right rail
    ];
    for q in quads {
        // The section is counter-clockwise in `(y, z)`, which viewed from `+X`
        // -- looking back along `-X`, where `+y` runs left -- is clockwise. So
        // the near cap at `x = 0` takes the reversed order and the far cap takes
        // the section's own. Written the other way round the mesh comes out
        // inside out, with the volume exactly right and negative, which is what
        // the fixture check below is for.
        triangles.push([idx(q[2]), idx(q[1]), idx(q[0])]);
        triangles.push([idx(q[3]), idx(q[2]), idx(q[0])]);
        triangles.push([idx(q[0] + n), idx(q[1] + n), idx(q[2] + n)]);
        triangles.push([idx(q[0] + n), idx(q[2] + n), idx(q[3] + n)]);
    }

    TriMesh::new(vertices, triangles, MeshMeta::synthetic()).expect("a valid hand-built mesh")
}

#[test]
fn the_hand_built_nominal_is_the_solid_it_claims_to_be() {
    // Checked before it is trusted. A nominal that is subtly wrong would fail
    // rung 3 and send the search into the engine, which is the most expensive
    // possible way to find a mistake in a test fixture.
    use chipbreaker_core::mesh::validate::validate;
    let (size, slot, floor) = (Vec3::new(24.0, 18.0, 10.0), (6.0, 12.0), 6.0);
    let mesh = slotted_block(size, slot, floor);
    let report = validate(&mesh);

    let expected = size.x * (size.y * size.z - (slot.1 - slot.0) * (size.z - floor));
    println!(
        "hand-built nominal: {} vertices, {} triangles, volume {:.6} against {:.6} \
         by arithmetic",
        mesh.vertex_count(),
        mesh.triangle_count(),
        report.signed_volume,
        expected
    );
    assert!(report.is_manifold, "the hand-built nominal is not manifold");
    assert!(report.is_watertight, "the hand-built nominal leaks");
    assert!(
        report.is_orientation_consistent,
        "the hand-built nominal has inconsistent winding"
    );
    assert!(
        (report.signed_volume - expected).abs() < 1.0e-9,
        "the hand-built nominal encloses {:.9} where the arithmetic says {expected:.9}; \
         a sign error in the winding shows up here as a negative volume and a \
         mis-placed vertex as a small difference",
        report.signed_volume
    );
}

#[test]
fn rung_3_a_cut_field_against_an_independently_modelled_nominal() {
    // **The real test**, and only meaningful because the three below it are
    // clean. Nothing the engine produced appears on the nominal side.
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::sweep::{LinearMove, Motion};
    use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

    let size = Vec3::new(24.0, 18.0, 10.0);
    let (radius, centre_y, floor) = (3.0, 9.0, 6.0);
    let nominal = slotted_block(size, (centre_y - radius, centre_y + radius), floor);

    let mut field = field_from(&shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), size));
    let profile =
        flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid");
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        // Starting and ending clear of the block, so the channel goes right
        // through and the nominal has no rounded ends to model.
        &[Motion::Linear(LinearMove {
            start: Vec3::new(-6.0, centre_y, floor),
            end: Vec3::new(size.x + 6.0, centre_y, floor),
        })],
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );

    let d = compare(&field, &nominal, Some(&shapes::box_solid(Vec3::ZERO, size)));
    let over = d.samples.iter().filter(|s| s.signed_mm.abs() > SPACING).count();
    println!(
        "rung 3: {} samples, {over} above one cell, worst gouge {:.4} mm, worst \
         excess {:.4} mm, rms {:.4} mm, worst projection gap {:.4} mm",
        d.samples.len(),
        d.worst_gouge_mm,
        d.worst_excess_mm,
        d.rms_mm,
        d.worst_projection_gap_mm
    );

    assert!(
        d.samples.len() > 10_000,
        "too few samples to mean anything: {}",
        d.samples.len()
    );
    // A correctly simulated part against its true nominal. The residual is the
    // engine's own reconstruction error and nothing else, so the bound is the one
    // rung 2 established: half a cell, where dual contouring places a vertex on a
    // flat face.
    let bound = SPACING / 2.0;
    assert!(
        d.worst_gouge_mm <= bound,
        "rung 3 reported a {:.4} mm gouge against a hand-modelled nominal, beyond \
         the {bound} mm the lattice accounts for. Rungs 1, 2 and 2c are clean, so \
         this is neither the comparison arithmetic nor the normals: it is the \
         simulation disagreeing with the part it was asked to make.",
        d.worst_gouge_mm
    );
    assert!(
        d.worst_excess_mm <= bound,
        "rung 3 reported {:.4} mm of excess stock against a hand-modelled nominal, \
         beyond the {bound} mm the lattice accounts for",
        d.worst_excess_mm
    );
}

#[test]
fn rung_3_reports_a_nominal_that_is_wrong() {
    // Rung 3 comes back exactly zero, and a test that reports zero has to prove
    // it could have reported something else. The zero is not luck: a flat mill
    // driven straight through a block sweeps exactly the slotted solid, and every
    // span endpoint is an exact intersection with it, so the two surfaces
    // genuinely coincide. But "genuinely coincide" and "the comparison is asleep"
    // look identical from the outside.
    //
    // So the same field is compared against a nominal whose channel floor is a
    // millimetre too low. The part is then a millimetre of excess stock across
    // the whole channel, and rung 3 must say so, with that sign.
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::sweep::{LinearMove, Motion};
    use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

    let size = Vec3::new(24.0, 18.0, 10.0);
    let (radius, centre_y, floor) = (3.0, 9.0, 6.0);
    let error = 1.0;
    let wrong = slotted_block(size, (centre_y - radius, centre_y + radius), floor - error);

    let mut field = field_from(&shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), size));
    let profile =
        flat_end_mill(2.0 * radius, 30.0, &Shank::plain(2.0 * radius, 60.0)).expect("valid");
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        &[Motion::Linear(LinearMove {
            start: Vec3::new(-6.0, centre_y, floor),
            end: Vec3::new(size.x + 6.0, centre_y, floor),
        })],
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );

    let d = compare(&field, &wrong, Some(&shapes::box_solid(Vec3::ZERO, size)));
    println!(
        "rung 3, nominal {error} mm too deep: worst gouge {:.4} mm, worst excess \
         {:.4} mm, rms {:.4} mm",
        d.worst_gouge_mm, d.worst_excess_mm, d.rms_mm
    );

    assert!(
        (d.worst_excess_mm - error).abs() < SPACING / 2.0,
        "a channel machined {error} mm shallower than the nominal must read as \
         {error} mm of excess stock; it read {:.4}",
        d.worst_excess_mm
    );
    assert!(
        d.worst_gouge_mm < SPACING / 2.0,
        "a part with material left standing is not gouged anywhere, but rung 3 \
         reported a {:.4} mm gouge. The sign is inverted.",
        d.worst_gouge_mm
    );
}
