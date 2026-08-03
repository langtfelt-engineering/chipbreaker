// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The tri-dexel corpus: seven fields pinned by digest, deviation and coverage.
//!
//! Goldens are digests, as at Unit 5, because a `.tdx` blob cannot be diffed.
//! Deviation and worst-cosine are pinned alongside because they are the unit's
//! assertion metric (ADR 0005) — a digest that moves while they hold still is a
//! serialization change; one where they move too is a change in where the rays
//! are, which wants a very different investigation.
//!
//! Regenerate with `examples/generate_tri_corpus.rs`.

use std::path::PathBuf;

use chipbreaker_core::dexel::deviation::{coverage, measure, sample_mesh_budget};
use chipbreaker_core::dexel::io as dexel_io;
use chipbreaker_core::dexel::tri::{
    AXES, AxisSet, TriBuildOptions, TriDexelField, WORST_CASE_COSINE,
};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Mat4, Vec3};
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use serde_json::Value;

fn expectations() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/tridexel/expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    assert!(!text.trim().is_empty(), "the tri-dexel corpus is empty");
    serde_json::from_str(&text).expect("valid JSON")
}

fn octahedron() -> TriMesh {
    let r = 10.0;
    TriMesh::new(
        vec![
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(-r, 0.0, 0.0),
            Vec3::new(0.0, r, 0.0),
            Vec3::new(0.0, -r, 0.0),
            Vec3::new(0.0, 0.0, r),
            Vec3::new(0.0, 0.0, -r),
        ],
        vec![
            [0, 2, 4],
            [2, 1, 4],
            [1, 3, 4],
            [3, 0, 4],
            [2, 0, 5],
            [1, 2, 5],
            [3, 1, 5],
            [0, 3, 5],
        ],
        MeshMeta::synthetic(),
    )
    .expect("valid")
}

/// Rebuilds a case from its id, mirroring the generator.
fn rebuild(id: &str) -> (TriMesh, TriDexelField) {
    let (mesh, spacing, axes): (TriMesh, f64, &str) = match id {
        "box-non-dividing-spacing" => (
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
            1.6,
            "xyz",
        ),
        "octahedron-body-diagonal-faces" => (octahedron(), 0.5, "xyz"),
        "sphere" => (shapes::icosphere(12.0, 4), 0.4, "xyz"),
        "cylinder-upright" => (shapes::cylinder(8.0, 24.0, 128), 0.5, "xyz"),
        "torus" => (shapes::torus(15.0, 4.0, 96, 48), 0.5, "xyz"),
        "lattice-block-integer-vertices" => (shapes::lattice_block(5), 0.5, "xyz"),
        "sphere-two-bundles-only" => (shapes::icosphere(12.0, 4), 0.4, "xz"),
        other => panic!(
            "the corpus has a case `{other}` this test cannot rebuild. Add it here as \
             well as in the generator, or the golden pins nothing."
        ),
    };
    let (field, _) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing,
            axes: AxisSet::parse(axes).expect("valid"),
            placement: Mat4::IDENTITY,
            margin: 0.0,
        },
    )
    .expect("builds");
    (mesh, field)
}

#[test]
fn every_corpus_field_matches_its_golden() {
    let data = expectations();
    let cases = data["cases"].as_array().expect("cases");
    assert!(cases.len() >= 7, "the corpus has shrunk: {}", cases.len());

    for case in cases {
        let id = case["id"].as_str().expect("id");
        let (_, field) = rebuild(id);

        let mut h = CanonicalHash::new();
        h.add(&field);
        assert_eq!(
            h.finish().to_hex(),
            case["digest"].as_str().expect("digest"),
            "{id}: the field digest moved. Check the deviation and volume columns \
             below to tell a serialization change from a geometry change."
        );

        // Per-bundle volumes on the BITS. Not an agreement assertion between
        // bundles -- ADR 0005 forbids that -- but each bundle must reproduce its
        // own number exactly.
        let golden = case["volumes_mm3"].as_array().expect("volumes");
        for (axis, expected) in AXES.iter().zip(golden) {
            match (field.bundle(*axis), expected.as_f64()) {
                (Some(bundle), Some(v)) => assert_eq!(
                    bundle.volume().to_bits(),
                    v.to_bits(),
                    "{id}/{axis:?}: volume {} vs golden {v}",
                    bundle.volume()
                ),
                (None, None) => {}
                (a, b) => panic!(
                    "{id}/{axis:?}: bundle present {} but golden {b:?}",
                    a.is_some()
                ),
            }
        }
        assert_eq!(
            field.bytes() as u64,
            case["bytes"].as_u64().expect("bytes"),
            "{id}: memory"
        );
    }
}

#[test]
fn every_corpus_field_round_trips_to_the_same_bytes() {
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let (_, field) = rebuild(id);
        let bytes = dexel_io::tri_to_bytes(&field).expect("writes");

        assert_eq!(
            bytes.len() as u64,
            case["file_bytes"].as_u64().expect("file_bytes"),
            "{id}: file size"
        );
        let mut h = CanonicalHash::new();
        h.bytes(&bytes);
        assert_eq!(
            h.finish().to_hex(),
            case["file_digest"].as_str().expect("file_digest"),
            "{id}: the bytes on disk changed. If the field digest did NOT also change, \
             this is a serialization change and TDX_FORMAT_VERSION should have moved."
        );

        let reloaded = dexel_io::tri_from_bytes(&bytes).expect("reads");
        let mut before = CanonicalHash::new();
        before.add(&field);
        let mut after = CanonicalHash::new();
        after.add(&reloaded);
        assert_eq!(before.finish(), after.finish(), "{id}: round trip");
    }
}

#[test]
fn deviation_and_coverage_match_their_goldens() {
    // The assertion metric, pinned. A tolerance is used rather than bits because
    // the sampler walks the mesh in float arithmetic and this is a measured
    // quantity, not a serialized one -- but it is tight enough that any real
    // change in where the rays sit will move it.
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let (mesh, field) = rebuild(id);
        let (samples, _) = sample_mesh_budget(&mesh, 4_000);
        let report = measure(&field, &samples);

        let expected = case["deviation_best_max_mm"].as_f64().expect("deviation");
        assert!(
            (report.best_max - expected).abs() <= 1e-9 * expected.max(1.0),
            "{id}: best-of-three deviation {} vs golden {expected}",
            report.best_max
        );

        let (cover, _) = coverage(&mesh, field.axes());
        let expected_cos = case["worst_cosine"].as_f64().expect("cosine");
        assert!(
            (cover - expected_cos).abs() < 1e-12,
            "{id}: worst cosine {cover} vs golden {expected_cos}"
        );
    }
}

#[test]
fn every_complete_field_honours_the_sampling_guarantee() {
    let data = expectations();
    let mut complete = 0;
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let (mesh, field) = rebuild(id);
        let (cover, _) = coverage(&mesh, field.axes());
        if field.is_complete() {
            complete += 1;
            assert!(
                cover >= WORST_CASE_COSINE - 1e-12,
                "{id}: a complete field sampled a surface at only {cover}, below the \
                 1/sqrt(3) bound"
            );
        } else {
            // The negative case earns its place: it shows the guarantee is a
            // property of having three bundles, not of the code in general.
            assert!(
                cover < WORST_CASE_COSINE,
                "{id}: a two-bundle field is not entitled to the bound, and this one \
                 met it at {cover} -- so the case no longer demonstrates anything"
            );
        }
    }
    assert!(complete >= 6, "the corpus should be mostly complete fields");
}

#[test]
fn the_corpus_covers_the_cases_that_matter() {
    let data = expectations();
    let ids: Vec<&str> = data["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .map(|c| c["id"].as_str().expect("id"))
        .collect();
    for required in [
        // The U5 lattice bug this unit found.
        "box-non-dividing-spacing",
        // The worst case a closed solid can present: the bound is attained.
        "octahedron-body-diagonal-faces",
        // The anisotropy that justifies three bundles.
        "cylinder-upright",
        // The offset invariant, on every axis.
        "lattice-block-integer-vertices",
        // Proof that the guarantee needs all three.
        "sphere-two-bundles-only",
    ] {
        assert!(
            ids.contains(&required),
            "the corpus has lost `{required}`, which one of this unit's findings rests on"
        );
    }
}
