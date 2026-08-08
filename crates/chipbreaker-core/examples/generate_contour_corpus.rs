// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Regenerates `tests/corpus/contour/expectations.json`.
//!
//! Goldens are **digests plus attributable side-channels**, as everywhere else
//! in this project: an extracted mesh is thousands of triangles and cannot be
//! diffed usefully, so what makes a failure diagnosable is what is recorded
//! beside the digest.
//!
//! - **Vertex and triangle counts**, which move on any connectivity change.
//! - **The soundness flags and the signed volume**, because the exit criterion
//!   is zero non-manifold outputs and a corpus that did not pin it would be
//!   pinning the wrong thing.
//! - **The Euler characteristic per component**, so a lost hole is a topology
//!   change rather than a small numeric one.
//! - **The rank histogram**, which is the sharp-feature measurement. A change
//!   here with the counts unmoved means the QEF started classifying flats as
//!   edges or the reverse -- a threshold change, not a geometry one.
//! - **Corner disagreement and multi-crossing counts**, which are properties of
//!   the *field* rather than of the extractor, so a change in them points
//!   upstream of this module.
//! - **Both extraction modes.** With and without normals are different code
//!   paths -- a QEF solve against a centroid fallback -- and the control has to
//!   keep working or the sharp-feature comparison stops meaning anything.

#![allow(missing_docs, reason = "an example binary, not API")]

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

struct Case {
    id: &'static str,
    note: &'static str,
    spacing: f64,
}

fn cases() -> Vec<Case> {
    vec![
        Case {
            id: "box-uncut",
            note: "all flats and sharp edges: the case the QEF reconstructs exactly, so any \
                   deviation at all is a regression rather than a tolerance",
            spacing: 0.5,
        },
        Case {
            id: "sphere-uncut",
            note: "no sharp feature anywhere, so every vertex should be rank 1 and the \
                   histogram pins that",
            spacing: 0.6,
        },
        Case {
            id: "torus-uncut",
            note: "genus 1, so a lost hole shows up in the Euler characteristic instead of \
                   hiding in a digest",
            spacing: 0.6,
        },
        Case {
            id: "slot-cut",
            note: "a through slot, so the surface includes cut faces whose normals come from \
                   the tool and are negated. An inverted cut face flips the signed volume",
            spacing: 0.5,
        },
        Case {
            id: "bore-through",
            note: "a hole right through, turning a genus 0 solid into a genus 1 one",
            spacing: 0.5,
        },
        Case {
            id: "tangential-skim",
            note: "a ball nose grazing the top face a few hundredths deep, which leaves \
                   slivers and near-degenerate sign configurations",
            spacing: 0.5,
        },
        Case {
            id: "diagonal-contact",
            note: "two solids meeting corner to corner: the configuration that forces a cell \
                   to emit more than one vertex, where plain DC would be non-manifold",
            spacing: 0.5,
        },
    ]
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

/// Rebuilds a case's field from its id. Mirrored exactly by the test.
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
            "the contour corpus has a case {other} this builder cannot rebuild. Add it here, \
             not only in the case list, or the golden pins nothing."
        ),
    }
}

fn main() {
    println!("{{");
    println!(
        "  \"_note\": \"Generated by examples/generate_contour_corpus.rs. Do not hand-edit.\","
    );
    println!(
        "  \"_exit_criterion\": \"Every case is manifold, watertight, orientation consistent \
         and positively oriented, in BOTH extraction modes. An entry recording otherwise is a \
         bug in the corpus, not a tolerated result.\","
    );
    println!("  \"cases\": [");

    let all = cases();
    for (index, case) in all.iter().enumerate() {
        let field = field_for(case.id, case.spacing);
        println!("    {{");
        println!("      \"id\": \"{}\",", case.id);
        println!("      \"note\": \"{}\",", case.note);
        println!("      \"spacing_mm\": {},", case.spacing);
        println!("      \"modes\": [");
        for (m, use_normals) in [true, false].into_iter().enumerate() {
            let (mesh, stats) = extract(
                &field,
                &ContourOptions {
                    use_normals,
                    ..ContourOptions::default()
                },
            )
            .expect("extracts");
            let report = validate(&mesh);
            let mut h = CanonicalHash::new();
            h.add(&mesh);
            println!("        {{");
            println!("          \"normals\": {use_normals},");
            println!("          \"vertices\": {},", mesh.vertex_count());
            println!("          \"triangles\": {},", mesh.triangle_count());
            println!("          \"is_manifold\": {},", report.is_manifold);
            println!("          \"is_watertight\": {},", report.is_watertight);
            println!(
                "          \"is_orientation_consistent\": {},",
                report.is_orientation_consistent
            );
            println!(
                "          \"signed_volume\": {:.17e},",
                report.signed_volume
            );
            println!(
                "          \"euler_characteristic\": [{}],",
                report
                    .components
                    .iter()
                    .map(|c| c.euler_characteristic.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            println!(
                "          \"rank_histogram\": [{}],",
                stats
                    .rank_histogram
                    .iter()
                    .map(std::string::ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            println!("          \"corners\": {},", stats.corners);
            println!(
                "          \"corner_disagreements\": {},",
                stats.corner_disagreements
            );
            println!("          \"crossing_edges\": {},", stats.crossing_edges);
            println!(
                "          \"multi_crossing_edges\": {},",
                stats.multi_crossing_edges
            );
            println!(
                "          \"cells_with_multiple_vertices\": {},",
                stats.cells_with_multiple_vertices
            );
            println!(
                "          \"clamped_vertices\": {},",
                stats.clamped_vertices
            );
            println!("          \"digest\": \"{}\"", h.finish().to_hex());
            print!("        }}");
            println!("{}", if m == 1 { "" } else { "," });
        }
        println!("      ]");
        print!("    }}");
        println!("{}", if index + 1 == all.len() { "" } else { "," });
    }
    println!("  ]");
    println!("}}");
}
