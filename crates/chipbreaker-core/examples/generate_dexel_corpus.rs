// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Regenerates `tests/corpus/dexel/expectations.json`.
//!
//! Goldens for dexel fields are **digests, not files**. A `.dexel` file is a
//! binary blob that cannot be diffed, so committing one as a golden would give a
//! test that fails with "1.4 MB differs" and no way to see what changed. The
//! digest is the thing under contract anyway: ADR 0004 requires bit-identical
//! round-tripping, and the digest is what the cross-platform parity check
//! compares.
//!
//! The file also records the volume, the occupancy and the span distribution, so
//! that a change to the digest can be attributed. A digest that moves while the
//! volume and distribution stay put is a serialization change; one where the
//! volume moves too is a geometry change.
//!
//! Redirect into the corpus file only after checking it ran:
//! `cargo run -q -p chipbreaker-core --example generate_dexel_corpus > out.json`
//! then move it into place. Writing straight into the tracked file is how a
//! zero-byte corpus got committed once already.

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::{BuildOptions, DexelField, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};

struct Case {
    id: &'static str,
    note: &'static str,
    mesh: fn() -> TriMesh,
    spacing: f64,
    axis: Axis,
    placement: Mat4,
}

/// Ninety degrees about X, so an upright axis lies down.
const LIE_DOWN: Mat4 = Mat4 {
    m: [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
};

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

fn cases() -> Vec<Case> {
    vec![
        Case {
            id: "box-at-rest",
            note: "stock before anything is cut: one span on every ray",
            mesh: || shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "lattice-block-integer-vertices",
            note: "every vertex an integer: the mesh that would trigger coplanar \
                   rejections if ray origins ever moved to cell corners",
            mesh: || shapes::lattice_block(5),
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "sphere",
            note: "a large empty fraction, and span endpoints that need all 17 digits",
            mesh: || shapes::icosphere(12.0, 4),
            spacing: 0.4,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "nested-shells-cavity",
            note: "a genuine internal cavity: the only measured case that reaches two \
                   spans on a ray",
            mesh: nested_shells,
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "torus-hole-along-the-bundle",
            note: "the hole shows as EMPTY rays, not two-span rays",
            mesh: || shapes::torus(15.0, 4.0, 96, 48),
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "torus-hole-across-the-bundle",
            note: "the same torus lying down: NOW it gives two-span rays",
            mesh: || shapes::torus(15.0, 4.0, 96, 48),
            spacing: 0.5,
            axis: Axis::Z,
            placement: LIE_DOWN,
        },
        Case {
            id: "cylinder-axis-along-bundle",
            note: "the vertical-wall case: volume is a lattice-point count of the disc",
            mesh: || shapes::cylinder(10.0, 20.0, 128),
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "cylinder-axis-across-bundle",
            note: "the same cylinder lying down: a smooth quadrature instead",
            mesh: || shapes::cylinder(10.0, 20.0, 128),
            spacing: 0.5,
            axis: Axis::Z,
            placement: LIE_DOWN,
        },
        Case {
            id: "sphere-bundle-along-x",
            note: "a non-default bundle axis, so Axis::cyclic is under contract too",
            mesh: || shapes::icosphere(12.0, 4),
            spacing: 0.4,
            axis: Axis::X,
            placement: Mat4::IDENTITY,
        },
        Case {
            id: "sphere-placed-off-origin",
            note: "an off-origin placement, so no ray origin is a round number",
            mesh: || shapes::icosphere(12.0, 4),
            spacing: 0.4,
            axis: Axis::Z,
            placement: Mat4::from_translation(Vec3::new(1.0 / 3.0, -0.078_125, 7.5)),
        },
    ]
}

fn main() {
    println!("{{");
    println!("  \"_note\": \"Generated by examples/generate_dexel_corpus.rs. Do not hand-edit.\",");
    println!(
        "  \"_format\": \"Goldens are digests, not files: a .dexel blob cannot be diffed. \
         Volume and distribution are recorded alongside so a digest change can be \
         attributed -- digest alone moving is serialization, digest and volume moving \
         together is geometry.\","
    );
    println!("  \"cases\": [");

    let all = cases();
    for (index, case) in all.iter().enumerate() {
        let mesh = (case.mesh)();
        let options = BuildOptions {
            spacing_xyz: None,
            spacing: case.spacing,
            axis: case.axis,
            placement: case.placement,
            margin: 0.0,
        };
        let (field, stats) =
            DexelField::build(&mesh, &options).unwrap_or_else(|e| panic!("{}: {e}", case.id));

        let mut h = CanonicalHash::new();
        h.add(&field);
        let digest = h.finish().to_hex();

        let bytes = dexel_io::to_bytes(&field).expect("writes");
        let mut bh = CanonicalHash::new();
        bh.bytes(&bytes);
        let file_digest = bh.finish().to_hex();

        let distribution: Vec<String> = field
            .arena()
            .distribution()
            .into_iter()
            .map(|(spans, rays)| format!("[{spans}, {rays}]"))
            .collect();

        println!("    {{");
        println!("      \"id\": \"{}\",", case.id);
        println!("      \"note\": \"{}\",", case.note);
        println!("      \"axis\": \"{}\",", case.axis.as_str());
        println!("      \"spacing_mm\": {},", case.spacing);
        println!("      \"rays\": {},", stats.rays);
        println!("      \"empty_rays\": {},", stats.empty_rays);
        println!("      \"total_spans\": {},", field.total_spans());
        println!("      \"spilled_rays\": {},", field.arena().spilled_rays());
        // Seventeen significant digits, always. A volume written with fewer would
        // reload as a different number, which is the whole point of ADR 0004.
        println!("      \"volume_mm3\": {:.17e},", field.volume());
        println!(
            "      \"span_distribution\": [{}],",
            distribution.join(", ")
        );
        println!("      \"file_bytes\": {},", bytes.len());
        println!("      \"file_digest\": \"{file_digest}\",");
        println!("      \"digest\": \"{digest}\"");
        print!("    }}");
        println!("{}", if index + 1 == all.len() { "" } else { "," });
    }

    println!("  ]");
    println!("}}");
}
