// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The dexel corpus: ten fields pinned by digest.
//!
//! Goldens here are **digests, not files**. A `.dexel` blob cannot be diffed, so
//! a committed file would give a test that fails with "1.4 MB differs" and no
//! way to see what changed. The digest is what is under contract anyway — it is
//! the value the cross-platform parity check compares.
//!
//! Volume, occupancy and span distribution are pinned alongside so that a
//! digest change can be attributed rather than merely noticed. A digest that
//! moves while the volume and distribution hold still is a serialization change;
//! one where the volume moves too is a geometry change. Those want very
//! different investigations.
//!
//! Regenerate with `examples/generate_dexel_corpus.rs`. The CI job that reruns
//! every generator and diffs the result is what stops this file from being
//! hand-edited into agreement.

use std::collections::BTreeMap;
use std::path::PathBuf;

use chipbreaker_core::dexel::{BuildOptions, DexelField, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use serde_json::Value;

/// Ninety degrees about X, so an upright axis lies down.
const LIE_DOWN: Mat4 = Mat4 {
    m: [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
};

fn expectations() -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/dexel/expectations.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    assert!(
        !text.trim().is_empty(),
        "the dexel corpus is empty. A generator once truncated a tracked corpus \
         file to zero bytes and it was committed that way; this is the guard."
    );
    serde_json::from_str(&text).expect("valid JSON")
}

fn nested_shells() -> TriMesh {
    let shell = shapes::icosphere(10.0, 3);
    let hole = shapes::icosphere(5.0, 3);
    let offset = shell.vertex_count();
    let mut vertices = shell.vertices().to_vec();
    let mut triangles = shell.triangles().to_vec();
    vertices.extend_from_slice(hole.vertices());
    triangles.extend(
        hole.triangles()
            .iter()
            .map(|t| [t[0] + offset, t[2] + offset, t[1] + offset]),
    );
    TriMesh::new(vertices, triangles, MeshMeta::synthetic()).expect("valid")
}

/// Rebuilds a case from its id, mirroring the generator.
fn rebuild(id: &str) -> (DexelField, u64) {
    let (mesh, spacing, axis, placement): (TriMesh, f64, Axis, Mat4) = match id {
        "box-at-rest" => (
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
            0.5,
            Axis::Z,
            Mat4::IDENTITY,
        ),
        "lattice-block-integer-vertices" => {
            (shapes::lattice_block(5), 0.5, Axis::Z, Mat4::IDENTITY)
        }
        "sphere" => (shapes::icosphere(12.0, 4), 0.4, Axis::Z, Mat4::IDENTITY),
        "nested-shells-cavity" => (nested_shells(), 0.5, Axis::Z, Mat4::IDENTITY),
        "torus-hole-along-the-bundle" => (
            shapes::torus(15.0, 4.0, 96, 48),
            0.5,
            Axis::Z,
            Mat4::IDENTITY,
        ),
        "torus-hole-across-the-bundle" => {
            (shapes::torus(15.0, 4.0, 96, 48), 0.5, Axis::Z, LIE_DOWN)
        }
        "cylinder-axis-along-bundle" => (
            shapes::cylinder(10.0, 20.0, 128),
            0.5,
            Axis::Z,
            Mat4::IDENTITY,
        ),
        "cylinder-axis-across-bundle" => {
            (shapes::cylinder(10.0, 20.0, 128), 0.5, Axis::Z, LIE_DOWN)
        }
        "sphere-bundle-along-x" => (shapes::icosphere(12.0, 4), 0.4, Axis::X, Mat4::IDENTITY),
        "sphere-placed-off-origin" => (
            shapes::icosphere(12.0, 4),
            0.4,
            Axis::Z,
            Mat4::from_translation(Vec3::new(1.0 / 3.0, -0.078_125, 7.5)),
        ),
        other => panic!(
            "the corpus has a case `{other}` that this test does not know how to \
             rebuild. Add it here as well as in the generator, or the golden is \
             pinning nothing."
        ),
    };
    let options = BuildOptions {
        spacing,
        axis,
        placement,
        margin: 0.0,
    };
    let (field, stats) = DexelField::build(&mesh, &options).expect("builds");
    (field, stats.rays)
}

#[test]
fn every_corpus_field_matches_its_golden() {
    let data = expectations();
    let cases = data["cases"].as_array().expect("cases is an array");
    assert!(
        cases.len() >= 10,
        "the corpus has shrunk: {} cases",
        cases.len()
    );

    for case in cases {
        let id = case["id"].as_str().expect("id");
        let (field, rays) = rebuild(id);

        let mut h = CanonicalHash::new();
        h.add(&field);
        let digest = h.finish().to_hex();
        assert_eq!(
            digest,
            case["digest"].as_str().expect("digest"),
            "{id}: the field digest moved. Compare the volume and distribution \
             below to tell a serialization change from a geometry change."
        );

        assert_eq!(rays, case["rays"].as_u64().expect("rays"), "{id}: rays");
        assert_eq!(
            field.total_spans() as u64,
            case["total_spans"].as_u64().expect("total_spans"),
            "{id}: spans"
        );
        assert_eq!(
            field.arena().spilled_rays() as u64,
            case["spilled_rays"].as_u64().expect("spilled_rays"),
            "{id}: spilled rays"
        );

        // The volume is pinned on the BITS. A tolerance here would let a
        // one-ULP drift through, which is exactly the class of change the
        // determinism contract exists to catch.
        let expected: f64 = case["volume_mm3"].as_f64().expect("volume");
        assert_eq!(
            field.volume().to_bits(),
            expected.to_bits(),
            "{id}: volume {} vs golden {expected}",
            field.volume()
        );

        let golden: BTreeMap<usize, usize> = case["span_distribution"]
            .as_array()
            .expect("distribution")
            .iter()
            .map(|pair| {
                let p = pair.as_array().expect("pair");
                (
                    usize::try_from(p[0].as_u64().expect("spans")).expect("small"),
                    usize::try_from(p[1].as_u64().expect("rays")).expect("small"),
                )
            })
            .collect();
        assert_eq!(field.arena().distribution(), golden, "{id}: distribution");
    }
}

#[test]
fn every_corpus_field_round_trips_to_the_same_bytes() {
    let data = expectations();
    for case in data["cases"].as_array().expect("cases") {
        let id = case["id"].as_str().expect("id");
        let (field, _) = rebuild(id);
        let bytes = dexel_io::to_bytes(&field).expect("writes");

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
            "{id}: the bytes on disk changed. If the field digest did NOT also \
             change, this is a serialization change and ADR 0004's format version \
             should have been bumped."
        );

        let reloaded = dexel_io::from_bytes(&bytes).expect("reads");
        let mut before = CanonicalHash::new();
        before.add(&field);
        let mut after = CanonicalHash::new();
        after.add(&reloaded);
        assert_eq!(before.finish(), after.finish(), "{id}: round trip");
    }
}

#[test]
fn the_corpus_covers_the_cases_that_matter() {
    // A corpus can rot by staying green while losing the case it was built for.
    // These are the shapes each of this unit's findings rests on.
    let data = expectations();
    let ids: Vec<&str> = data["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .map(|c| c["id"].as_str().expect("id"))
        .collect();

    for required in [
        // The half-cell offset invariant: integer vertices everywhere.
        "lattice-block-integer-vertices",
        // The arena's tail: the only measured shape reaching two spans.
        "nested-shells-cavity",
        // The pair that corrected "holes give two spans".
        "torus-hole-along-the-bundle",
        "torus-hole-across-the-bundle",
        // The pair that is the argument for Unit 6.
        "cylinder-axis-along-bundle",
        "cylinder-axis-across-bundle",
        // Axis::cyclic under contract on a non-default axis.
        "sphere-bundle-along-x",
    ] {
        assert!(
            ids.contains(&required),
            "the corpus has lost `{required}`, which one of this unit's findings \
             rests on. See the note on that case in the generator."
        );
    }
}

#[test]
fn no_corpus_field_needed_a_coplanar_rejection() {
    // ADR 0001 Part 2 makes a coplanar rejection a hard error, so a field that
    // builds at all had none. Stated as its own test because the lattice block
    // is in the corpus precisely to exercise it: every one of its vertices is an
    // integer, which is the geometry that would trigger rejections the moment
    // ray origins moved to cell corners.
    let (field, _) = rebuild("lattice-block-integer-vertices");
    assert!(
        field.total_spans() > 0,
        "the lattice block found no material"
    );
}
