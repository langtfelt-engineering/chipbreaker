// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Three bundles: the theorem, the offset invariant, and the format.

use chipbreaker_core::dexel::io as dexel_io;
use chipbreaker_core::dexel::tri::{
    AXES, AxisSet, DEVIATION_CONSTANT, TriBuildOptions, TriDexelField, WORST_CASE_COSINE,
    best_cosine,
};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};
use chipbreaker_core::transcendental::acos;

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

fn options(spacing: f64) -> TriBuildOptions {
    TriBuildOptions {
        spacing,
        ..TriBuildOptions::default()
    }
}

// --- the theorem -----------------------------------------------------------

#[test]
fn no_unit_normal_is_poorly_sampled_by_all_three_axes() {
    // THE load-bearing claim of this unit. If all three |n.d| were below
    // 1/sqrt(3), the squares would sum to less than 1, contradicting |n| = 1.
    //
    // Swept deterministically over a fine spherical grid rather than randomly:
    // a seeded RNG would be reproducible but this is cheaper and covers the
    // sphere evenly, which is what the bound is about.
    let mut worst = f64::INFINITY;
    let mut worst_normal = Vec3::new(0.0, 0.0, 1.0);
    let steps = 360;
    for i in 0..steps {
        for j in 0..=steps {
            let theta = core::f64::consts::PI * f64::from(i) / f64::from(steps);
            let phi = 2.0 * core::f64::consts::PI * f64::from(j) / f64::from(steps);
            let (st, ct) = chipbreaker_core::transcendental::sin_cos(theta);
            let (sp, cp) = chipbreaker_core::transcendental::sin_cos(phi);
            let n = Vec3::new(st * cp, st * sp, ct);
            let best = best_cosine(n, AxisSet::XYZ);
            if best < worst {
                worst = best;
                worst_normal = n;
            }
        }
    }
    assert!(
        worst >= WORST_CASE_COSINE - 1e-12,
        "a normal {worst_normal:?} was sampled at only {worst}, below the 1/sqrt(3) \
         bound of {WORST_CASE_COSINE}. That bound is the whole guarantee of this unit."
    );
}

#[test]
fn the_bound_is_tight_at_the_body_diagonal() {
    // Not merely satisfied: attained. A bound nobody reaches would mean the
    // guarantee was weaker than it needed to be.
    let d = 1.0 / (3.0f64).sqrt();
    for signs in [
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, -1.0],
    ] {
        let n = Vec3::new(signs[0] * d, signs[1] * d, signs[2] * d);
        let best = best_cosine(n, AxisSet::XYZ);
        assert!(
            (best - WORST_CASE_COSINE).abs() < 1e-15,
            "the body diagonal {n:?} should attain the bound exactly, got {best}"
        );
    }
    // 54.7356 degrees, which is the number the report quotes.
    let degrees = acos(WORST_CASE_COSINE) * 180.0 / core::f64::consts::PI;
    assert!((degrees - 54.735_610_317_245_35).abs() < 1e-9, "{degrees}");
}

#[test]
fn the_deviation_constant_follows_from_the_bound() {
    // (h/2) * sqrt(1 - 1/3). Pinned so the two constants cannot drift apart:
    // they are one derivation, not two numbers.
    let expected = (1.0 - WORST_CASE_COSINE * WORST_CASE_COSINE).sqrt() / 2.0;
    assert!(
        (DEVIATION_CONSTANT - expected).abs() < 1e-15,
        "DEVIATION_CONSTANT {DEVIATION_CONSTANT} does not follow from \
         WORST_CASE_COSINE {WORST_CASE_COSINE}: expected {expected}"
    );
}

#[test]
fn two_bundles_carry_no_such_guarantee() {
    // Why the guarantee needs all three, stated as a test so that a future
    // "optimisation" dropping a bundle fails loudly rather than silently
    // weakening the claim.
    let xz = AxisSet::parse("xz").expect("valid");
    // A normal along Y is invisible to both X and Z bundles.
    assert_eq!(best_cosine(Vec3::new(0.0, 1.0, 0.0), xz), 0.0);
    assert!(best_cosine(Vec3::new(0.0, 1.0, 0.0), AxisSet::XYZ) >= WORST_CASE_COSINE);
}

// --- the offset invariant, per bundle --------------------------------------

#[test]
fn every_bundle_offsets_in_its_own_transverse_plane() {
    // Unit 5 proved the offset is load-bearing on one axis; this extends it to
    // all three. The invariant is PER BUNDLE, not global: each applies the half
    // cell in the two axes its own lattice spans.
    let mesh = shapes::lattice_block(9);
    let (field, stats) = TriDexelField::build(&mesh, &options(1.0)).expect("builds");
    assert_eq!(stats.coplanar_rejected, 0);

    for (axis, bundle) in field.bundles() {
        let lattice = bundle.lattice();
        let [u, v, _] = axis.cyclic();
        let [nx, ny] = lattice.counts();
        for i in 0..nx {
            for j in 0..ny {
                let origin = lattice.origin_of(i, j).to_array();
                for k in [u, v] {
                    let value = origin[k];
                    assert!(
                        (value - value.round()).abs() > 1e-12,
                        "{axis:?} bundle ray ({i}, {j}) has transverse coordinate {value} \
                         on the integer lattice. The half-cell offset is per bundle and \
                         this one has lost it."
                    );
                }
            }
        }
    }
}

#[test]
fn the_safety_gate_holds_on_all_three_axes() {
    // Zero coplanar rejections and zero odd crossing counts. Construction
    // aborts on either, so building at all is the assertion -- but named here
    // so a regression says what it broke.
    let meshes: [(&str, TriMesh); 6] = [
        ("lattice-block", shapes::lattice_block(9)),
        (
            "box",
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
        ),
        ("sphere", shapes::icosphere(12.0, 3)),
        ("cylinder", shapes::cylinder(10.0, 20.0, 64)),
        ("cone", shapes::cone(10.0, 20.0, 64)),
        ("torus", shapes::torus(15.0, 4.0, 48, 24)),
    ];
    for (name, mesh) in &meshes {
        let (_, stats) =
            TriDexelField::build(mesh, &options(0.6)).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(stats.coplanar_rejected, 0, "{name}");
        for axis in AXES {
            let per = stats.per_axis[axis.index()].expect("all three built");
            assert_eq!(per.predicates.coplanar_rejected, 0, "{name}/{axis:?}");
            assert!(per.rays > 0, "{name}/{axis:?}");
        }
    }
}

// --- the field -------------------------------------------------------------

#[test]
fn all_three_bundles_are_built_and_each_keeps_its_own_lattice() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 20.0, 10.0));
    let (field, _) = TriDexelField::build(&mesh, &options(0.5)).expect("builds");
    assert!(field.is_complete());

    // Deliberately NOT co-registered, and each records its own geometry. A
    // 40x20x10 box gives three different ray counts, which is the cheapest
    // possible demonstration that they are independent lattices.
    let counts: Vec<[u32; 2]> = field.bundles().map(|(_, b)| b.lattice().counts()).collect();
    assert_eq!(counts, vec![[40, 20], [20, 80], [80, 40]]);
    for (axis, bundle) in field.bundles() {
        assert_eq!(bundle.lattice().axis(), axis);
    }
}

#[test]
fn the_three_bundles_measure_a_box_identically_because_a_box_is_exact() {
    // The one solid every bundle captures exactly. Not a volume-agreement
    // assertion in general -- ADR 0005 forbids that -- but a box is the case
    // where agreement is structural rather than lucky, so disagreement here
    // means a bundle is broken.
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 20.0, 10.0));
    let (field, _) = TriDexelField::build(&mesh, &options(0.5)).expect("builds");
    let expected = 40.0 * 20.0 * 10.0;
    for (axis, volume) in AXES.iter().zip(field.volumes()) {
        let v = volume.expect("built");
        assert!((v - expected).abs() / expected < 1e-12, "{axis:?}: {v}");
    }
}

#[test]
fn a_partial_field_is_allowed_but_says_it_is_incomplete() {
    let mesh = shapes::icosphere(8.0, 3);
    let (field, stats) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: 0.5,
            axes: AxisSet::parse("xz").expect("valid"),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds");
    assert!(!field.is_complete());
    assert_eq!(field.axes().len(), 2);
    assert!(field.bundle(Axis::Y).is_none());
    assert!(stats.per_axis[Axis::Y.index()].is_none());
}

#[test]
fn provenance_records_the_source_and_its_tessellation() {
    let mesh = shapes::icosphere(10.0, 2);
    let (field, _) = TriDexelField::build(&mesh, &options(0.5)).expect("builds");
    let p = field.provenance();
    assert_eq!(p.source_triangles, mesh.triangle_count());
    assert!((p.requested_spacing_mm - 0.5).abs() < 1e-15);
    // A coarse icosphere has real curvature, so it must report a real sagitta.
    assert!(
        p.tessellation.percentile_sagitta_mm > 0.0,
        "{:?}",
        p.tessellation
    );
    // And the digest identifies the mesh it came from.
    let mut h = CanonicalHash::new();
    h.add(&mesh);
    assert_eq!(p.source_digest, h.finish());
}

// --- the format ------------------------------------------------------------

#[test]
fn a_tri_field_survives_a_round_trip_bit_identically() {
    let mesh = shapes::icosphere(9.0, 3);
    let (field, _) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: 0.7,
            placement: Mat4::from_translation(Vec3::new(1.0 / 3.0, -7.125e-3, 12.0)),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds");

    let bytes = dexel_io::tri_to_bytes(&field).expect("writes");
    let reloaded = dexel_io::tri_from_bytes(&bytes).expect("reads");
    assert_eq!(digest(&field), digest(&reloaded));

    for axis in AXES {
        let a = field.bundle(axis).expect("built");
        let b = reloaded.bundle(axis).expect("built");
        assert_eq!(a.volume().to_bits(), b.volume().to_bits(), "{axis:?}");
        let rays = u32::try_from(a.arena().rays()).expect("small");
        for ray in 0..rays {
            for (x, y) in a.arena().get(ray).iter().zip(b.arena().get(ray)) {
                assert_eq!(x.t0.to_bits(), y.t0.to_bits());
                assert_eq!(x.t1.to_bits(), y.t1.to_bits());
            }
        }
    }
    // Provenance survives too, including the float estimate.
    assert_eq!(field.provenance(), reloaded.provenance());
}

#[test]
fn writing_the_same_tri_field_twice_gives_the_same_bytes() {
    let mesh = shapes::cylinder(8.0, 20.0, 48);
    let (field, _) = TriDexelField::build(&mesh, &options(0.6)).expect("builds");
    assert_eq!(
        dexel_io::tri_to_bytes(&field).expect("writes"),
        dexel_io::tri_to_bytes(&field).expect("writes")
    );
}

#[test]
fn a_partial_field_round_trips_and_does_not_hash_like_a_complete_one() {
    let mesh = shapes::icosphere(8.0, 3);
    let (partial, _) = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: 0.6,
            axes: AxisSet::parse("xz").expect("valid"),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds");
    let (complete, _) = TriDexelField::build(&mesh, &options(0.6)).expect("builds");

    let bytes = dexel_io::tri_to_bytes(&partial).expect("writes");
    let reloaded = dexel_io::tri_from_bytes(&bytes).expect("reads");
    assert_eq!(digest(&partial), digest(&reloaded));
    assert!(!reloaded.is_complete());
    assert_ne!(
        digest(&partial),
        digest(&complete),
        "a two-bundle field must not hash like a three-bundle one"
    );
}

#[test]
fn both_formats_stay_readable_and_are_told_apart() {
    use chipbreaker_core::dexel::{BuildOptions, DexelField, FieldFormat};
    let mesh = shapes::cube(10.0);
    let (single, _) = DexelField::build(
        &mesh,
        &BuildOptions {
            spacing: 1.0,
            ..BuildOptions::default()
        },
    )
    .expect("builds");
    let (tri, _) = TriDexelField::build(&mesh, &options(1.0)).expect("builds");

    let single_bytes = dexel_io::to_bytes(&single).expect("writes");
    let tri_bytes = dexel_io::tri_to_bytes(&tri).expect("writes");

    assert_eq!(dexel_io::detect(&single_bytes), Some(FieldFormat::Single));
    assert_eq!(dexel_io::detect(&tri_bytes), Some(FieldFormat::Tri));
    assert_eq!(dexel_io::detect(b"neither of them"), None);

    // And each reader refuses the other's file rather than misreading it.
    assert!(dexel_io::tri_from_bytes(&single_bytes).is_err());
    assert!(dexel_io::from_bytes(&tri_bytes).is_err());
    // The old format still reads, which is the compatibility requirement.
    assert!(dexel_io::from_bytes(&single_bytes).is_ok());
}

#[test]
fn a_truncated_tri_file_is_refused_at_every_length() {
    let mesh = shapes::icosphere(7.0, 2);
    let (field, _) = TriDexelField::build(&mesh, &options(1.0)).expect("builds");
    let bytes = dexel_io::tri_to_bytes(&field).expect("writes");
    for cut in (0..bytes.len()).step_by(211) {
        assert!(
            dexel_io::tri_from_bytes(&bytes[..cut]).is_err(),
            "a {cut}-byte prefix of a {}-byte file was accepted",
            bytes.len()
        );
    }
}

// --- tessellation adequacy -------------------------------------------------

#[test]
fn a_coarse_mesh_asked_for_fine_cells_produces_advice() {
    use chipbreaker_core::dexel::tessellation;
    // A coarse icosphere: big facets, real curvature, so a large sagitta.
    let coarse = tessellation::estimate(&shapes::icosphere(20.0, 1));
    assert!(coarse.percentile_sagitta_mm > 0.0, "{coarse:?}");
    assert!(
        coarse.is_finer_than_the_mesh_supports(0.01),
        "0.01 mm cells on a 2-subdivision sphere must be flagged: {coarse:?}"
    );
    let advice = coarse.advice(0.01).expect("advice");
    assert!(advice.contains("finer than this mesh supports"), "{advice}");

    // And a fine one asked for reasonable cells must NOT be flagged, or the
    // warning is noise and will be ignored when it matters.
    let fine = tessellation::estimate(&shapes::icosphere(20.0, 5));
    assert!(
        !fine.is_finer_than_the_mesh_supports(0.5),
        "0.5 mm cells on a 5-subdivision sphere must not be flagged: {fine:?}"
    );
    assert!(fine.advice(0.5).is_none());
}

#[test]
fn a_faceted_part_is_not_mistaken_for_a_coarse_smooth_one() {
    use chipbreaker_core::dexel::tessellation;
    // A cube's 90-degree edges are the design, not an approximation of a
    // curve. Treating them as tessellation error would condemn every prismatic
    // part in the corpus.
    let cube = tessellation::estimate(&shapes::cube(20.0));
    assert!(
        cube.sharp_edges > 0,
        "the cube's edges must be seen as sharp"
    );
    assert_eq!(
        cube.edges, 0,
        "a cube has no curvature to estimate from: {cube:?}"
    );
    assert!(!cube.is_finer_than_the_mesh_supports(0.01));
}

#[test]
fn the_estimate_tracks_refinement() {
    use chipbreaker_core::dexel::tessellation;
    // Each subdivision roughly quarters the sagitta. Asserting the direction
    // rather than the factor: the proxy is a proxy.
    let mut previous = f64::INFINITY;
    for subdivisions in 1..5 {
        let e = tessellation::estimate(&shapes::icosphere(20.0, subdivisions));
        assert!(
            e.percentile_sagitta_mm < previous,
            "subdividing must reduce the estimated deviation: {subdivisions} gave {e:?}"
        );
        previous = e.percentile_sagitta_mm;
    }
}

#[test]
fn a_mesh_with_no_interior_edges_estimates_nothing_rather_than_guessing() {
    use chipbreaker_core::dexel::tessellation;
    let soup = TriMesh::new(
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
        MeshMeta::synthetic(),
    )
    .expect("valid");
    let e = tessellation::estimate(&soup);
    assert_eq!(e.edges, 0);
    assert_eq!(e.percentile_sagitta_mm, 0.0);
    assert!(!e.is_finer_than_the_mesh_supports(1e-6));
}

// --- deviation: the assertion metric ---------------------------------------

#[test]
fn best_of_three_deviation_falls_monotonically_and_is_bounded_by_c_times_h() {
    // The definition of done, and the thing volume could not give us. ADR 0005
    // has the argument; this is the assertion.
    use chipbreaker_core::dexel::deviation::{measure, sample_mesh_budget};

    let cases: [(&str, TriMesh); 3] = [
        ("sphere", shapes::icosphere(10.0, 4)),
        ("cylinder", shapes::cylinder(8.0, 24.0, 128)),
        (
            "box",
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
        ),
    ];
    for (name, mesh) in &cases {
        let (samples, _) = sample_mesh_budget(mesh, 4_000);
        let mut previous = f64::INFINITY;
        for spacing in [1.6, 0.8, 0.4, 0.2] {
            let (field, _) = TriDexelField::build(
                mesh,
                &TriBuildOptions {
                    spacing,
                    ..TriBuildOptions::default()
                },
            )
            .expect("builds");
            let report = measure(&field, &samples);

            assert!(
                report.best_max < previous,
                "{name}: deviation must FALL as cells shrink -- {} at h={spacing} is not \
                 below {previous} at the coarser step. Monotonicity is the property \
                 that lets a customer be told a finer simulation is a safer one, and \
                 it is exactly what volume could not provide.",
                report.best_max
            );
            // The measured constant runs about 0.67 to 0.81 across the corpus.
            // Bounded generously at 1.5 so this catches a regression rather than
            // ordinary variation between shapes.
            assert!(
                report.constant() < 1.5,
                "{name} at h={spacing}: deviation {} is {:.3} times the cell size, \
                 outside the C*h envelope",
                report.best_max,
                report.constant()
            );
            previous = report.best_max;
        }
    }
}

#[test]
fn the_worst_sampling_angle_never_falls_below_the_bound() {
    // The direct check on §2, over real surfaces rather than synthetic normals.
    use chipbreaker_core::dexel::deviation::coverage;
    for (name, mesh) in [
        ("sphere", shapes::icosphere(10.0, 4)),
        ("torus", shapes::torus(12.0, 4.0, 64, 32)),
        ("cylinder", shapes::cylinder(8.0, 24.0, 128)),
        ("octahedron (1,1,1) faces", octahedron()),
    ] {
        let (worst, normal) = coverage(&mesh, AxisSet::XYZ);
        assert!(
            worst >= WORST_CASE_COSINE - 1e-12,
            "{name}: a face with normal {normal:?} is sampled at only {worst}, below \
             the 1/sqrt(3) bound"
        );
    }
    // And the octahedron ATTAINS it: its faces are exactly the body diagonals,
    // so it is the worst case a closed solid can present.
    let (worst, _) = coverage(&octahedron(), AxisSet::XYZ);
    assert!(
        (worst - WORST_CASE_COSINE).abs() < 1e-12,
        "the octahedron should sit exactly on the bound, got {worst}"
    );
}

#[test]
fn a_single_bundle_is_visibly_worse_than_three_and_that_is_the_point() {
    // The measurement that justifies the unit. A box's side faces are parallel
    // to a Z bundle, so its Z endpoints lie only on the top and bottom -- the
    // worst point on a side face is half the depth away, and REFINING DOES NOT
    // HELP. Best-of-three fixes it because every face is normal to some axis.
    use chipbreaker_core::dexel::deviation::{measure, sample_mesh_budget};
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0));
    let (samples, _) = sample_mesh_budget(&mesh, 4_000);

    let mut per_axis_at = Vec::new();
    let mut best_at = Vec::new();
    for spacing in [0.8, 0.2] {
        let (field, _) = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing,
                ..TriBuildOptions::default()
            },
        )
        .expect("builds");
        let report = measure(&field, &samples);
        per_axis_at.push(report.per_axis_max[2].expect("Z built"));
        best_at.push(report.best_max);
    }

    // Four times the resolution barely moves the single bundle...
    assert!(
        per_axis_at[1] > per_axis_at[0] * 0.9,
        "a single bundle should stay stuck near half the depth: {per_axis_at:?}"
    );
    // ...and cuts best-of-three by about four.
    assert!(
        best_at[1] < best_at[0] * 0.35,
        "best-of-three should fall with h: {best_at:?}"
    );
    assert!(
        per_axis_at[1] > best_at[1] * 20.0,
        "at fine cells the single bundle should be an order of magnitude worse: \
         {per_axis_at:?} vs {best_at:?}"
    );
}

/// The worst case a closed solid can present: every face normal is a body
/// diagonal, so all eight sit exactly on the `1/sqrt(3)` bound.
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
