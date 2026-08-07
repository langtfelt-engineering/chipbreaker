// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The deviation table: per-bundle, best-of-three, against mesh and analytic.
//!
//! ADR 0005: volume is a construction-time diagnostic, deviation is the
//! assertion metric. This is where deviation gets measured.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-core --example tri_deviation`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::deviation::{Analytic, coverage, measure, sample_mesh_budget};
use chipbreaker_core::dexel::tri::{AXES, AxisSet, TriBuildOptions, TriDexelField};
use chipbreaker_core::dexel::tri::{
    AXIS_ALIGNED_SAMPLE_CONSTANT, PERPENDICULAR_CONSTANT, SAMPLE_DISTANCE_CONSTANT,
    WORST_CASE_COSINE,
};
use chipbreaker_core::math::{Axis, Vec3};
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::transcendental::acos;

/// Cell sizes, coarse to fine.
const SPACINGS: [f64; 5] = [1.6, 0.8, 0.4, 0.2, 0.1];

/// Surface points per case. Enough to find the worst region, few enough that
/// the harness finishes.
const SAMPLE_BUDGET: usize = 20_000;

/// Parametric steps for the analytic samplers.
const ANALYTIC_STEPS: u32 = 90;

struct Case {
    name: &'static str,
    mesh: fn() -> TriMesh,
    analytic: Option<fn() -> Analytic>,
}

fn cases() -> Vec<Case> {
    vec![
        Case {
            name: "sphere r=10",
            // Deliberately fine: the analytic column is only meaningful while
            // the mesh's own error stays under the sampling error, and U5
            // showed a coarse icosphere floors out at h/R = 1/40.
            mesh: || shapes::icosphere(10.0, 5),
            analytic: Some(|| Analytic::Sphere { radius: 10.0 }),
        },
        Case {
            name: "cylinder r=8 h=24, axis along Z",
            mesh: || shapes::cylinder(8.0, 24.0, 256),
            analytic: Some(|| Analytic::Cylinder {
                radius: 8.0,
                height: 24.0,
                axis: Axis::Z,
            }),
        },
        Case {
            name: "torus R=12 r=4",
            mesh: || shapes::torus(12.0, 4.0, 256, 128),
            analytic: Some(|| Analytic::Torus {
                major: 12.0,
                minor: 4.0,
            }),
        },
        Case {
            name: "box 30x20x10 (axis-aligned, the exact case)",
            mesh: || shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
            analytic: None,
        },
        Case {
            name: "oriented faces incl. a (1,1,1) normal",
            mesh: oriented_faces,
            analytic: None,
        },
    ]
}

/// A closed solid whose faces sit at many orientations, including the worst
/// case: a face whose normal is the body diagonal `(1,1,1)/sqrt(3)`.
///
/// An octahedron gives eight faces with normals `(+-1,+-1,+-1)/sqrt(3)` —
/// every one of them exactly at the bound. Rotating a second copy by small and
/// large angles adds the 1, 45 and 89 degree orientations the plan asked for.
fn oriented_faces() -> TriMesh {
    use chipbreaker_core::math::{Mat3, Mat4};
    use chipbreaker_core::transcendental::sin_cos;

    // The regular octahedron: normals are exactly the four body diagonals.
    let r = 10.0;
    let vertices = vec![
        Vec3::new(r, 0.0, 0.0),
        Vec3::new(-r, 0.0, 0.0),
        Vec3::new(0.0, r, 0.0),
        Vec3::new(0.0, -r, 0.0),
        Vec3::new(0.0, 0.0, r),
        Vec3::new(0.0, 0.0, -r),
    ];
    let triangles = vec![
        [0, 2, 4],
        [2, 1, 4],
        [1, 3, 4],
        [3, 0, 4],
        [2, 0, 5],
        [1, 2, 5],
        [3, 1, 5],
        [0, 3, 5],
    ];
    let octahedron = TriMesh::new(
        vertices,
        triangles,
        chipbreaker_core::mesh::MeshMeta::synthetic(),
    )
    .expect("valid");

    // And a box tilted by 1 degree about two axes, giving faces at 1 and 89
    // degrees to the axes; plus one at 45.
    let mut all_vertices = octahedron.vertices().to_vec();
    let mut all_triangles = octahedron.triangles().to_vec();
    for (index, degrees) in [1.0f64, 45.0].into_iter().enumerate() {
        let radians = degrees * core::f64::consts::PI / 180.0;
        let (s, c) = sin_cos(radians);
        let rotate = Mat4::from_mat3_translation(
            Mat3::from_rows_array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]),
            Vec3::new(60.0 * (index as f64 + 1.0), 0.0, 0.0),
        );
        let cube = shapes::cube(12.0);
        let offset = u32::try_from(all_vertices.len()).expect("small");
        all_vertices.extend(cube.vertices().iter().map(|v| rotate.transform_point(*v)));
        all_triangles.extend(
            cube.triangles()
                .iter()
                .map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]),
        );
    }
    TriMesh::new(
        all_vertices,
        all_triangles,
        chipbreaker_core::mesh::MeshMeta::synthetic(),
    )
    .expect("valid")
}

fn main() {
    println!("Deviation: one-sided Hausdorff distance from densely sampled surface");
    println!("points to the field's span endpoints, which are EXACT ray-surface");
    println!("intersections. Sampling adequacy, not reconstruction error (that is U9).");
    println!();
    println!("ADR 0005: volume is a diagnostic, deviation is the metric.");
    println!();

    let mut all_monotone = true;
    let mut worst_constant = 0.0f64;
    let mut worst_cosine_seen = f64::INFINITY;

    for case in cases() {
        let mesh = (case.mesh)();
        println!(
            "=== {} ({} triangles) ===",
            case.name,
            mesh.triangle_count()
        );

        let (cover, normal) = coverage(&mesh, AxisSet::XYZ);
        println!(
            "  worst sampling cosine over the surface: {cover:.9}  ({:.4} deg)  at n = \
             [{:.4}, {:.4}, {:.4}]",
            acos(cover.min(1.0)) * 180.0 / core::f64::consts::PI,
            normal[0],
            normal[1],
            normal[2]
        );
        println!("  bound is 1/sqrt(3) = {WORST_CASE_COSINE:.9}");
        worst_cosine_seen = worst_cosine_seen.min(cover);
        println!();

        println!(
            "  {:>7}  {:>11}  {:>11}  {:>11}  {:>11}  {:>7}  {:>11}",
            "h (mm)", "dev X", "dev Y", "dev Z", "BEST-OF-3", "C=dev/h", "vs analytic"
        );

        let mut previous = f64::INFINITY;
        for spacing in SPACINGS {
            let (field, _) = TriDexelField::build(
                &mesh,
                &TriBuildOptions {
                    spacing_xyz: None,
                    spacing,
                    ..TriBuildOptions::default()
                },
            )
            .expect("builds");

            // Against the MESH: isolates sampling error, because the samples sit
            // on exactly the surface the rays met.
            // A fixed budget, not a spacing-derived one: deviation is a
            // supremum, so coverage of every region matters and fineness within
            // one does not. Tying it to `h` would make the point count grow as
            // 1/h^2 alongside the field and the run would never finish.
            let (samples, _) = sample_mesh_budget(&mesh, SAMPLE_BUDGET);
            let report = measure(&field, &samples);

            // Against the ANALYTIC solid: adds U3's tessellation error.
            let analytic = case.analytic.map(|make| {
                let points = make().sample(ANALYTIC_STEPS);
                measure(&field, &points).best_max
            });

            let cell = |v: Option<f64>| {
                v.map_or_else(|| "         --".to_owned(), |x| format!("{x:>11.6}"))
            };
            println!(
                "  {spacing:>7}  {}  {}  {}  {:>11.6}  {:>7.3}  {}",
                cell(report.per_axis_max[0]),
                cell(report.per_axis_max[1]),
                cell(report.per_axis_max[2]),
                report.best_max,
                report.constant(),
                cell(analytic),
            );

            if report.best_max > previous {
                all_monotone = false;
                println!(
                    "     ^^ NOT MONOTONE: {:.6} at h={spacing} exceeds {previous:.6} at the \
                     coarser step",
                    report.best_max
                );
            }
            previous = report.best_max;
            worst_constant = worst_constant.max(report.constant());
        }
        println!();
    }

    println!("--- summary ---");
    println!(
        "  best-of-three deviation monotone in h: {}",
        if all_monotone { "yes" } else { "NO" }
    );
    println!("  largest observed C in deviation <= C*h: {worst_constant:.4}");
    println!("  bound for this metric: h*sqrt(3/2) = {SAMPLE_DISTANCE_CONSTANT:.6} * h");
    println!("  cos=1 floor:           h/sqrt(2)   = {AXIS_ALIGNED_SAMPLE_CONSTANT:.6} * h");
    println!("  worst sampling cosine anywhere: {worst_cosine_seen:.9}");
    println!("  bound: {WORST_CASE_COSINE:.9}");
    println!();
    println!("The per-bundle columns exist so the anisotropy stays visible. A surface");
    println!("parallel to one bundle is sampled sparsely by it and densely by another;");
    println!("reporting only the worst bundle would make a tri-dexel field look as bad");
    println!("as its weakest axis, which is exactly what the third bundle removes.");
    println!();
    println!("The metric here is SAMPLE DISTANCE -- how far a surface point is from the");
    println!("nearest place the field sampled the surface. It is bounded by");
    println!(
        "h*sqrt(3/2) = {SAMPLE_DISTANCE_CONSTANT:.6}*h and that bound is TIGHT: an octahedron,"
    );
    println!("whose every face normal is a body diagonal, attains it exactly at a vertex.");
    println!(
        "An axis-aligned box attains exactly h/sqrt(2) = {AXIS_ALIGNED_SAMPLE_CONSTANT:.6}*h."
    );
    println!();
    println!("It is NOT perpendicular deviation. Span endpoints are exact ray-surface");
    println!("intersections, so for a plane the component of this distance along the");
    println!("surface normal is exactly zero -- the whole of it is lateral. Perpendicular");
    println!("error belongs to a RECONSTRUCTION, not to the field: for a flat top per cell");
    println!(
        "it is bounded by h/sqrt(3) = {PERPENDICULAR_CONSTANT:.6}*h. Unit 12's gouge depth is a"
    );
    println!("perpendicular quantity, so quoting this column there would overstate it.");
    let _ = AXES;
}
