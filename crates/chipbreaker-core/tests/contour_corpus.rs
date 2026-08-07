// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The contour corpus: seven extractions pinned by digest, in both modes.
//!
//! Goldens are digests, as everywhere else, because an extracted mesh is
//! thousands of triangles and cannot be diffed. What is pinned beside the digest
//! decides what a failure *means*:
//!
//! - digest moves, counts hold → vertex positions moved: a QEF or solver change
//! - counts move too → connectivity changed: the sign or component logic
//! - Euler characteristic moves → **topology** changed, which is the serious one
//! - rank histogram moves alone → the singular threshold, not the geometry
//! - corner disagreement or multi-crossing counts move → the change is upstream
//!   of this module, in the field
//!
//! The soundness flags are asserted rather than merely compared. A corpus that
//! recorded a non-manifold result and then faithfully reproduced it would be
//! pinning the bug.

use std::path::PathBuf;

use chipbreaker_core::contour::{ContourOptions, extract};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::validate::validate;
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri_motion};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};
use serde_json::Value;

fn expectations() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/contour/expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    assert!(!text.trim().is_empty(), "the contour corpus is empty");
    serde_json::from_str(&text).expect("valid JSON")
}

fn mill(d: f64) -> Profile {
    flat_end_mill(d, 30.0, &Shank::plain(d, 60.0)).expect("valid")
}

fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

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

/// Rebuilds a case from its id, mirroring the generator exactly.
fn field_for(id: &str, spacing: f64) -> TriDexelField {
    let build = |mesh: &TriMesh| {
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
    };
    let cut = |field: &mut TriDexelField, profile: &Profile, motions: &[Motion]| {
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
    };

    match id {
        "box-uncut" => build(&shapes::box_solid(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(16.0, 12.0, 8.0),
        )),
        "sphere-uncut" => build(&shapes::icosphere(7.0, 3)),
        "torus-uncut" => build(&shapes::torus(7.0, 2.5, 48, 24)),
        "slot-cut" => {
            let mut f = build(&shapes::box_solid(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(24.0, 18.0, 10.0),
            ));
            cut(
                &mut f,
                &mill(6.0),
                &[line([-4.0, 9.0, 6.0], [28.0, 9.0, 6.0])],
            );
            f
        }
        "bore-through" => {
            let mut f = build(&shapes::box_solid(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(20.0, 20.0, 10.0),
            ));
            cut(
                &mut f,
                &mill(8.0),
                &[line([10.0, 10.0, 12.0], [10.0, 10.0, -2.0])],
            );
            f
        }
        "tangential-skim" => {
            let mut f = build(&shapes::box_solid(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(24.0, 16.0, 10.0),
            ));
            let ball = ball_end_mill(8.0, 30.0, &Shank::plain(8.0, 60.0)).expect("valid");
            cut(
                &mut f,
                &ball,
                &[
                    line([-4.0, 5.0, 9.97], [28.0, 5.0, 9.97]),
                    line([-4.0, 11.0, 10.01], [28.0, 11.0, 10.01]),
                ],
            );
            f
        }
        "diagonal-contact" => {
            let left = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(9.9, 7.9, 6.0));
            let right = shapes::box_solid(Vec3::new(10.1, 8.1, 0.0), Vec3::new(20.0, 16.0, 6.0));
            build(&merged(&left, &right))
        }
        other => panic!(
            "the corpus has a case {other} this test cannot rebuild. Add it here as well as \
             in the generator, or the golden pins nothing."
        ),
    }
}

#[test]
fn every_corpus_extraction_matches_its_golden() {
    let data = expectations();
    let cases = data["cases"].as_array().expect("cases");
    assert!(cases.len() >= 7, "the corpus has shrunk: {}", cases.len());

    for case in cases {
        let id = case["id"].as_str().expect("id");
        let spacing = case["spacing_mm"].as_f64().expect("spacing");
        let field = field_for(id, spacing);

        for mode in case["modes"].as_array().expect("modes") {
            let use_normals = mode["normals"].as_bool().expect("normals");
            let label = format!("{id}/normals={use_normals}");
            let (mesh, stats) = extract(
                &field,
                &ContourOptions {
                    use_normals,
                    ..ContourOptions::default()
                },
            )
            .unwrap_or_else(|e| panic!("{label}: {e}"));
            let report = validate(&mesh);

            // Asserted, not compared. A corpus that recorded a hole and then
            // reproduced it faithfully would be pinning the bug.
            assert!(report.is_manifold, "{label}: NOT MANIFOLD");
            assert!(report.is_watertight, "{label}: NOT WATERTIGHT");
            assert!(
                report.is_orientation_consistent,
                "{label}: orientation inconsistent"
            );
            assert!(
                report.signed_volume > 0.0,
                "{label}: signed volume {} is negative, so the mesh is inside out",
                report.signed_volume
            );

            let mut h = CanonicalHash::new();
            h.add(&mesh);
            assert_eq!(
                h.finish().to_hex(),
                mode["digest"].as_str().expect("digest"),
                "{label}: the mesh digest moved. Compare the counts and the Euler \
                 characteristic below to tell a vertex move from a connectivity or \
                 topology change."
            );

            assert_eq!(
                mesh.vertex_count() as u64,
                mode["vertices"].as_u64().expect("vertices"),
                "{label}: vertex count"
            );
            assert_eq!(
                mesh.triangle_count() as u64,
                mode["triangles"].as_u64().expect("triangles"),
                "{label}: triangle count"
            );

            let expected_euler: Vec<i64> = mode["euler_characteristic"]
                .as_array()
                .expect("euler")
                .iter()
                .map(|v| v.as_i64().expect("an integer"))
                .collect();
            let got_euler: Vec<i64> = report
                .components
                .iter()
                .map(|c| c.euler_characteristic)
                .collect();
            assert_eq!(
                got_euler, expected_euler,
                "{label}: the Euler characteristic moved, which means the TOPOLOGY \
                 changed -- a hole gained or lost, or a component split"
            );

            let expected_ranks: Vec<u64> = mode["rank_histogram"]
                .as_array()
                .expect("ranks")
                .iter()
                .map(|v| v.as_u64().expect("a count"))
                .collect();
            assert_eq!(
                stats.rank_histogram.to_vec(),
                expected_ranks,
                "{label}: the rank histogram moved. With the counts unchanged this is \
                 the singular threshold reclassifying flats and edges, not a geometry \
                 change."
            );

            for (name, got, expected) in [
                ("corners", stats.corners, mode["corners"].as_u64()),
                (
                    "corner disagreements",
                    stats.corner_disagreements,
                    mode["corner_disagreements"].as_u64(),
                ),
                (
                    "crossing edges",
                    stats.crossing_edges,
                    mode["crossing_edges"].as_u64(),
                ),
                (
                    "multi-crossing edges",
                    stats.multi_crossing_edges,
                    mode["multi_crossing_edges"].as_u64(),
                ),
                (
                    "split cells",
                    stats.cells_with_multiple_vertices,
                    mode["cells_with_multiple_vertices"].as_u64(),
                ),
                (
                    "clamped vertices",
                    stats.clamped_vertices,
                    mode["clamped_vertices"].as_u64(),
                ),
            ] {
                assert_eq!(
                    got,
                    expected.expect("a count"),
                    "{label}: {name}. The corner and crossing counts are properties of \
                     the FIELD, so a change in them points upstream of contouring."
                );
            }
        }
    }
}

#[test]
fn the_corpus_covers_the_configurations_that_matter() {
    let data = expectations();
    let ids: Vec<&str> = data["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .map(|c| c["id"].as_str().expect("id"))
        .collect();
    for required in [
        // Reconstructed exactly; any deviation is a regression.
        "box-uncut",
        // Genus 1 by construction, so a lost hole is visible.
        "torus-uncut",
        // Cut faces, whose normals are the negated tool normals.
        "slot-cut",
        // Turns genus 0 into genus 1 by machining rather than by modelling.
        "bore-through",
        // The configuration plain DC gets wrong.
        "diagonal-contact",
    ] {
        assert!(
            ids.contains(&required),
            "the corpus has lost `{required}`, which one of this unit's findings rests on"
        );
    }
}

#[test]
fn discarding_normals_flattens_every_sharp_feature() {
    // The control's defining property, pinned in the corpus rather than only in
    // a one-off measurement: without normals the QEF has no planes, so every
    // system is rank 0 and no vertex is ever classified as an edge or a corner.
    // If this ever stops being true, the sharp-feature comparison has silently
    // stopped comparing two different things.
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        for mode in case["modes"].as_array().expect("modes") {
            if mode["normals"].as_bool().expect("normals") {
                continue;
            }
            let ranks: Vec<u64> = mode["rank_histogram"]
                .as_array()
                .expect("ranks")
                .iter()
                .map(|v| v.as_u64().expect("a count"))
                .collect();
            assert_eq!(
                ranks[1] + ranks[2] + ranks[3],
                0,
                "{id}: without normals every system must be rank 0, got {ranks:?}"
            );
        }
    }
}

#[test]
fn the_sharp_cases_do_find_edges_and_corners_with_normals() {
    // The other half: a shape with edges must report them, or the corpus would
    // pass with a QEF that had quietly stopped working.
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        if !matches!(
            id,
            "box-uncut" | "slot-cut" | "bore-through" | "diagonal-contact"
        ) {
            continue;
        }
        let mode = &case["modes"].as_array().expect("modes")[0];
        assert!(mode["normals"].as_bool().expect("normals"));
        let ranks: Vec<u64> = mode["rank_histogram"]
            .as_array()
            .expect("ranks")
            .iter()
            .map(|v| v.as_u64().expect("a count"))
            .collect();
        assert!(
            ranks[2] > 0 && ranks[3] > 0,
            "{id} has sharp edges and corners; the QEF found {ranks:?}"
        );
    }
}

#[test]
fn the_smooth_case_finds_no_sharp_feature_at_all() {
    // A sphere and a torus have no edge anywhere, so a single rank-2 vertex
    // would mean the singular threshold is inventing creases out of the
    // encoding's own noise.
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        if !matches!(id, "sphere-uncut" | "torus-uncut") {
            continue;
        }
        let mode = &case["modes"].as_array().expect("modes")[0];
        let ranks: Vec<u64> = mode["rank_histogram"]
            .as_array()
            .expect("ranks")
            .iter()
            .map(|v| v.as_u64().expect("a count"))
            .collect();
        assert_eq!(
            ranks[2] + ranks[3],
            0,
            "{id} is smooth everywhere, but the QEF reported {} edge and {} corner \
             vertices. That is the singular threshold reading quantisation noise as \
             geometry.",
            ranks[2],
            ranks[3]
        );
    }
}
