// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! **Zero non-manifold outputs. No exceptions.**
//!
//! This is Unit 9's exit criterion and it is not a quality target. Unit 12
//! compares the extracted mesh against the nominal part; a hole in it becomes a
//! phantom gouge, and a phantom gouge is a customer stopping a machine for
//! nothing. So every case here asserts the full validator, not a subset.
//!
//! The two deliberately awkward shapes — a thin wall and a near-tangential cut —
//! are the ones plain dual contouring gets wrong. They are here because the
//! manifold criterion has to be the algorithm rather than a patch, and a test
//! suite that only contained comfortable shapes would not notice the difference.

use chipbreaker_core::contour::{ContourOptions, extract};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::validate::validate;
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri_motion};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};

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

fn mill(diameter: f64) -> Profile {
    flat_end_mill(diameter, 30.0, &Shank::plain(diameter, 60.0)).expect("valid")
}

fn ball(diameter: f64) -> Profile {
    ball_end_mill(diameter, 30.0, &Shank::plain(diameter, 60.0)).expect("valid")
}

fn cut(field: &mut TriDexelField, profile: &Profile, motions: &[Motion], spacing: f64) {
    let mut scratch = CutScratch::new(profile);
    for m in motions {
        cut_tri_motion(
            field,
            profile,
            m,
            SweepMethod::Analytic {
                tolerance: spacing / 10.0,
            },
            &mut scratch,
        );
    }
}

fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

/// The whole exit criterion, in one place.
fn assert_sound(label: &str, field: &TriDexelField, options: &ContourOptions) -> TriMesh {
    let (mesh, stats) = extract(field, options).unwrap_or_else(|e| panic!("{label}: {e}"));
    assert!(
        mesh.triangle_count() > 0,
        "{label}: produced no triangles at all, which passes every check below \
         vacuously"
    );
    let report = validate(&mesh);
    assert!(
        report.is_manifold,
        "{label}: NOT MANIFOLD. {} finding(s), first: {:?}. Stats: {stats:?}",
        report.findings.len(),
        report.findings.first()
    );
    assert!(
        report.is_watertight,
        "{label}: NOT WATERTIGHT -- a hole here becomes a phantom gouge at Unit \
         12. {} finding(s), first: {:?}",
        report.findings.len(),
        report.findings.first()
    );
    assert!(
        report.is_orientation_consistent,
        "{label}: orientation is inconsistent between neighbouring triangles"
    );
    assert!(
        report.signed_volume > 0.0,
        "{label}: signed volume {} is negative, so the mesh is INSIDE OUT. The \
         likeliest cause is the normal sign convention on cut faces.",
        report.signed_volume
    );
    mesh
}

#[test]
fn an_uncut_box_extracts_soundly() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0));
    let field = field_from(&mesh, 0.5);
    assert_sound("uncut box", &field, &ContourOptions::default());
}

#[test]
fn an_uncut_sphere_extracts_soundly() {
    // Smooth and curved everywhere, so every cell is a flat or a gentle bend and
    // the QEF spends its time in the rank-1 regime.
    let mesh = shapes::icosphere(8.0, 3);
    let field = field_from(&mesh, 0.4);
    assert_sound("uncut sphere", &field, &ContourOptions::default());
}

#[test]
fn an_uncut_torus_extracts_soundly() {
    // Genus 1: the Euler characteristic test below depends on this shape
    // surviving extraction with its hole intact.
    let mesh = shapes::torus(8.0, 3.0, 64, 32);
    let field = field_from(&mesh, 0.4);
    assert_sound("uncut torus", &field, &ContourOptions::default());
}

#[test]
fn a_slotted_block_extracts_soundly() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0));
    let mut field = field_from(&mesh, 0.4);
    let profile = mill(6.0);
    cut(
        &mut field,
        &profile,
        &[line([-5.0, 10.0, 6.0], [35.0, 10.0, 6.0])],
        0.4,
    );
    assert_sound("slotted block", &field, &ContourOptions::default());
}

#[test]
fn a_pocketed_block_extracts_soundly() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 24.0, 10.0));
    let mut field = field_from(&mesh, 0.4);
    let profile = mill(6.0);
    let mut motions = Vec::new();
    let mut y = 8.0;
    while y <= 16.0 {
        motions.push(line([8.0, y, 6.0], [22.0, y, 6.0]));
        y += 2.0;
    }
    cut(&mut field, &profile, &motions, 0.4);
    assert_sound("pocketed block", &field, &ContourOptions::default());
}

/// Two solids joined into one mesh, so a field can be built across both.
fn merged(a: &TriMesh, b: &TriMesh) -> TriMesh {
    let mut vertices = a.vertices().to_vec();
    let mut triangles = a.triangles().to_vec();
    let offset = u32::try_from(vertices.len()).expect("small");
    vertices.extend_from_slice(b.vertices());
    for t in b.triangles() {
        triangles.push([t[0] + offset, t[1] + offset, t[2] + offset]);
    }
    TriMesh::new(vertices, triangles, MeshMeta::synthetic()).expect("valid")
}

#[test]
fn diagonal_contact_forces_cells_to_split() {
    // **Where plain dual contouring fails, and the reason the manifold criterion
    // has to be the algorithm rather than a patch.**
    //
    // A first version of this test used a thin *wall* and found no splits, which
    // was the test being wrong rather than the code. A wall thinner than a cell
    // does not produce disconnected inside-corners: at most one row of corners
    // falls inside it, so the cell sees one component or none.
    //
    // What splits a cell is a sub-cell *gap*: solid on both sides, air between,
    // all inside one cell. Its inside corners fall into two groups with no path
    // between them through the cell's own edges. Plain DC would put one vertex
    // there, and that vertex would have to serve both faces of the gap -- giving
    // an edge four incident triangles.
    // **Two attempts at this were wrong, and both were instructive.**
    //
    // A gap narrower than a cell does not split anything: either a corner lands
    // inside the gap, in which case each side simply has its own surface, or no
    // corner does, in which case both corners are solid and the gap is invisible
    // -- below the resolution, and counted as a multi-crossing edge rather than
    // reconstructed.
    //
    // What splits a cell is a configuration where two inside corners are not
    // adjacent along any cell edge. The simplest is diagonal contact: two solids
    // meeting corner to corner. The cell at the junction has inside corners at
    // opposite ends of a face diagonal and outside corners between them, so no
    // path joins them through the cell's own edges.
    let spacing = 0.5;
    let left = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(9.9, 7.9, 6.0));
    let right = shapes::box_solid(Vec3::new(10.1, 8.1, 0.0), Vec3::new(20.0, 16.0, 6.0));
    let field = field_from(&merged(&left, &right), spacing);

    let (_, stats) = extract(&field, &ContourOptions::default()).expect("extracts");
    assert!(
        stats.cells_with_multiple_vertices > 0,
        "diagonal contact in a {spacing} mm grid must split at least one cell,          got none -- the manifold criterion would then be inert and this test          worthless. Stats: {stats:?}"
    );
    assert_sound("diagonal contact", &field, &ContourOptions::default());
}

#[test]
fn a_thin_wall_extracts_soundly() {
    // A wall about one and a half cells thick, left between two slots. It does
    // not split cells -- see the test above for what does -- but it is the shape
    // most likely to produce a degenerate sliver, so it earns its own soundness
    // check.
    let spacing = 0.5;
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 24.0, 10.0));
    let mut field = field_from(&mesh, spacing);
    let profile = mill(6.0);
    cut(
        &mut field,
        &profile,
        &[
            line([-5.0, 8.6, -1.0], [35.0, 8.6, -1.0]),
            line([-5.0, 15.4, -1.0], [35.0, 15.4, -1.0]),
        ],
        spacing,
    );
    assert_sound("thin wall", &field, &ContourOptions::default());
}

#[test]
fn a_near_tangential_cut_extracts_soundly() {
    // The other case plain DC gets wrong: a tool grazing a surface leaves cells
    // with a sliver of material and sign configurations that barely change.
    let spacing = 0.4;
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0));
    let mut field = field_from(&mesh, spacing);
    let profile = ball(8.0);
    // A ball nose skimming the top face, cutting a few hundredths deep.
    cut(
        &mut field,
        &profile,
        &[
            line([-5.0, 6.0, 9.97], [35.0, 6.0, 9.97]),
            line([-5.0, 10.0, 9.99], [35.0, 10.0, 9.99]),
            line([-5.0, 14.0, 10.01], [35.0, 14.0, 10.01]),
        ],
        spacing,
    );
    assert_sound("near-tangential cut", &field, &ContourOptions::default());
}

#[test]
fn a_deep_bore_extracts_soundly() {
    let spacing = 0.4;
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(24.0, 24.0, 16.0));
    let mut field = field_from(&mesh, spacing);
    let profile = mill(8.0);
    cut(
        &mut field,
        &profile,
        &[line([12.0, 12.0, 18.0], [12.0, 12.0, 4.0])],
        spacing,
    );
    assert_sound("deep bore", &field, &ContourOptions::default());
}

#[test]
fn a_through_hole_leaves_a_genus_one_solid() {
    // Euler characteristic, on a shape whose genus is known by construction: a
    // block with one hole right through it is a torus topologically, so
    // `V - E + F = 0`.
    let spacing = 0.4;
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(24.0, 24.0, 10.0));
    let mut field = field_from(&mesh, spacing);
    let profile = mill(8.0);
    cut(
        &mut field,
        &profile,
        &[line([12.0, 12.0, 12.0], [12.0, 12.0, -2.0])],
        spacing,
    );
    let extracted = assert_sound("through hole", &field, &ContourOptions::default());
    let report = validate(&extracted);
    assert_eq!(
        report.components.len(),
        1,
        "a block with one hole is still one component"
    );
    assert_eq!(
        report.components[0].euler_characteristic, 0,
        "a solid with one through hole has genus 1 and so characteristic 0; got \
         {}. A characteristic of 2 means the hole did not go through.",
        report.components[0].euler_characteristic
    );
}

#[test]
fn extraction_without_normals_is_still_sound() {
    // The surface-nets control has to be a valid mesh too, or the sharp-feature
    // comparison would be comparing against something broken.
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0));
    let field = field_from(&mesh, 0.5);
    assert_sound(
        "no normals",
        &field,
        &ContourOptions {
            use_normals: false,
            ..ContourOptions::default()
        },
    );
}

#[test]
fn extraction_is_deterministic() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0));
    let mut field = field_from(&mesh, 0.5);
    cut(
        &mut field,
        &mill(6.0),
        &[line([-2.0, 8.0, 8.0], [22.0, 8.0, 8.0])],
        0.5,
    );
    let (first, stats) = extract(&field, &ContourOptions::default()).expect("extracts");
    for _ in 0..8 {
        let (again, s) = extract(&field, &ContourOptions::default()).expect("extracts");
        assert_eq!(s, stats, "the statistics moved between runs");
        assert_eq!(again.triangle_count(), first.triangle_count());
        for (a, b) in first.vertices().iter().zip(again.vertices()) {
            assert_eq!(a.x.to_bits(), b.x.to_bits(), "vertex x moved");
            assert_eq!(a.y.to_bits(), b.y.to_bits(), "vertex y moved");
            assert_eq!(a.z.to_bits(), b.z.to_bits(), "vertex z moved");
        }
        assert_eq!(again.triangles(), first.triangles(), "connectivity moved");
    }
}

#[test]
fn a_field_missing_a_bundle_is_refused_rather_than_guessed() {
    use chipbreaker_core::dexel::tri::AxisSet;
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 16.0, 12.0));
    let (field, _) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: 0.5,
            axes: AxisSet::parse("z").expect("valid"),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds");
    let err = extract(&field, &ContourOptions::default()).expect_err("must refuse");
    let text = err.to_string();
    assert!(
        text.contains("bundle"),
        "the error should name the missing bundle, got {text}"
    );
}
