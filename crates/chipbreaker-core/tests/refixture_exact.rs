// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does turning the part over cost anything?
//!
//! # The claim under test
//!
//! A second operation is usually the part rotated a quarter turn or flipped,
//! and the claim is that this costs **nothing at all** — not "little", not
//! "within tolerance". A 90° rotation is a signed permutation of the
//! coordinates, so rays map onto rays and the whole operation is a relabelling.
//!
//! The way to test that claim is not to compare the moved field against itself
//! but against a field **built directly in the target orientation**, from a mesh
//! that was rotated first. If those agree span for span, the move introduced
//! nothing, and the zero bound the report prints is a fact rather than a hope.
//!
//! # Why the mutation checks are not optional here
//!
//! "Two fields agree" passes trivially when both are empty, when both are the
//! whole block, or when the comparison walks nothing. Each test below therefore
//! also asserts that the fields contain material, that a *wrong* rotation is
//! caught, and that the identity is not the only case that works.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::refixture::{Regime, classify, refixture_exact};

const SPACING: f64 = 0.5;

/// A deliberately lopsided block, so a wrong axis mapping cannot pass by
/// symmetry. A cube would agree with its own rotation whatever the code did.
fn block() -> TriMesh {
    shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(24.0, 16.0, 8.0))
}

fn transformed_mesh(m: &TriMesh, t: &Mat4) -> TriMesh {
    let v: Vec<Vec3> = m.vertices().iter().map(|p| t.transform_point(*p)).collect();
    TriMesh::new(v, m.triangles().to_vec(), m.meta().clone())
        .expect("a rigid motion preserves validity")
}

fn build(m: &TriMesh) -> TriDexelField {
    TriDexelField::build(
        m,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// Every span of every ray of every bundle, as comparable text.
fn dump(f: &TriDexelField) -> Vec<String> {
    let mut out = Vec::new();
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let Some(b) = f.bundle(axis) else {
            out.push(format!("{axis:?}: absent"));
            continue;
        };
        let l = b.lattice();
        out.push(format!(
            "{axis:?} counts {:?} spacing {:?} origin {:?} extent {:?} len {}",
            l.counts(),
            l.spacing_uv(),
            l.origin().to_array(),
            l.extent(),
            l.length()
        ));
        for ray in 0..u32::try_from(l.ray_count()).unwrap_or(0) {
            for s in b.arena().get(ray) {
                out.push(format!(
                    "{axis:?} {ray} {:.12} {:.12} {} {} {} {}",
                    s.t0, s.t1, s.n0.u, s.n0.v, s.n1.u, s.n1.v
                ));
            }
        }
    }
    out
}

/// A rotation written out exactly, rather than through a cosine.
fn quarter_turn_z() -> Mat4 {
    Mat4::from_rows_array([
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

fn flip_x() -> Mat4 {
    // A half turn about X: the part turned over, which is the other common
    // second operation.
    Mat4::from_rows_array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

fn assert_moves_exactly(t: &Mat4, name: &str) {
    let mesh = block();
    let source = build(&mesh);
    let direct = build(&transformed_mesh(&mesh, t));
    let moved = refixture_exact(&source, t)
        .unwrap_or_else(|| panic!("{name}: the transform was refused, but it is axis-aligned"));

    let (a, b) = (dump(&moved), dump(&direct));
    // Before comparing: both must actually contain something.
    assert!(
        a.len() > 100,
        "{name}: the moved field has only {} entries, so the comparison below \
         would pass on almost nothing",
        a.len()
    );
    assert_eq!(
        a.len(),
        b.len(),
        "{name}: the moved field has {} entries and the direct build {}",
        a.len(),
        b.len()
    );
    let mismatch = a
        .iter()
        .zip(&b)
        .filter(|(x, y)| x != y)
        .take(4)
        .map(|(x, y)| format!("\n  moved:  {x}\n  direct: {y}"))
        .collect::<Vec<_>>();
    assert!(
        mismatch.is_empty(),
        "{name}: moving the field differs from building it in the target \
         orientation, so the zero bound is not earned:{}",
        mismatch.join("")
    );
}

#[test]
fn a_quarter_turn_moves_the_field_exactly() {
    assert_moves_exactly(&quarter_turn_z(), "quarter turn about Z");
}

#[test]
fn turning_the_part_over_moves_the_field_exactly() {
    assert_moves_exactly(&flip_x(), "half turn about X");
}

#[test]
fn the_identity_moves_the_field_exactly() {
    assert_moves_exactly(&Mat4::IDENTITY, "identity");
}

#[test]
fn the_comparison_would_notice_the_wrong_rotation() {
    // **The mutation check the rest of the file rests on.** If the dump
    // compared too little -- only counts, say, or only the first ray -- then
    // moving by one rotation and building by another would still agree. The
    // block is lopsided precisely so this cannot pass by symmetry.
    let mesh = block();
    let source = build(&mesh);
    let moved = refixture_exact(&source, &quarter_turn_z()).expect("axis-aligned");
    let wrong = build(&transformed_mesh(&mesh, &flip_x()));
    assert_ne!(
        dump(&moved),
        dump(&wrong),
        "a quarter turn about Z matched a build flipped about X, so the \
         comparison is not sensitive to orientation at all"
    );
}

#[test]
fn an_arbitrary_rotation_is_refused_rather_than_moved() {
    // The exact path must decline what it cannot do exactly. Quietly resampling
    // here would let a caller print a zero bound for a transform that lost
    // something.
    let (c, s) = (
        chipbreaker_core::transcendental::cos(0.4),
        chipbreaker_core::transcendental::sin(0.4),
    );
    let m = Mat4::from_rows_array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let source = build(&block());
    assert!(
        refixture_exact(&source, &m).is_none(),
        "an arbitrary rotation was moved by the exact path, which would claim a \
         zero bound for a resample"
    );
    // And it is classified as carrying one.
    match classify(&m, SPACING).expect("rigid") {
        Regime::Resampled { bound_mm } => assert!(bound_mm > 0.0),
        Regime::Exact { .. } => panic!("an arbitrary rotation classified as exact"),
    }
}

#[test]
fn a_moved_field_has_the_same_volume() {
    // A coarser check than span equality, and it catches a whole class of
    // mistake that span equality would too -- but it is worth having separately
    // because it fails with a number a reader can interpret rather than a diff.
    let mesh = block();
    let source = build(&mesh);
    let moved = refixture_exact(&source, &quarter_turn_z()).expect("axis-aligned");
    let (v0, v1) = (source.volume(), moved.volume());
    assert!(v0 > 1000.0, "the fixture must contain material, got {v0}");
    assert!(
        (v0 - v1).abs() < 1.0e-9,
        "turning the part over changed its volume from {v0} to {v1}"
    );
}

#[test]
fn a_single_setup_report_carries_no_boundary_section() {
    // **The common case must not pay for the general one.** A reader who never
    // re-fixtures should see the report they have always seen, so an empty
    // boundary list is rendered as absent rather than as `[]`.
    use chipbreaker_core::findings::report::{Manifest, Report};
    use chipbreaker_core::findings::verdict::{self, GateOutcome, Verdict};

    let manifest = Manifest {
        inputs: Vec::new(),
        spacing_mm: [SPACING; 3],
        tolerance_mm: 0.1,
        cluster_radius_mm: 1.0,
        engine_version: "test".to_owned(),
        engine_selftest: "test".to_owned(),
        boundaries: Vec::new(),
    };
    let report = Report {
        manifest,
        semantics: chipbreaker_core::findings::report::semantics_from(
            &chipbreaker_core::deviation::DeviationField::default(),
            [SPACING; 3],
            0.1,
            None,
        ),
        findings: Vec::new(),
        collisions: Vec::new(),
        verdict: Verdict::new().with(verdict::GATE_GOUGE, GateOutcome::pass()),
        rapid_path: None,
    };
    let json = report.to_json();
    assert!(
        json["manifest"]["boundaries"].is_null(),
        "a single-setup job rendered a boundary section: {}",
        json["manifest"]["boundaries"]
    );
    assert_eq!(
        json["manifest"]["accumulated_transform_bound_mm"], 0.0,
        "a job that crossed no boundary must accumulate no bound"
    );
}

#[test]
fn boundaries_reach_the_manifest_digest() {
    // Two jobs that arrived at the same geometry by different re-fixturing did
    // not do the same thing, and a digest that could not tell them apart would
    // be claiming they had.
    use chipbreaker_core::findings::report::{Boundary, Manifest};

    let base = Manifest {
        inputs: Vec::new(),
        spacing_mm: [SPACING; 3],
        tolerance_mm: 0.1,
        cluster_radius_mm: 1.0,
        engine_version: "test".to_owned(),
        engine_selftest: "test".to_owned(),
        boundaries: Vec::new(),
    };
    let mut crossed = base.clone();
    crossed.boundaries.push(Boundary {
        into_setup: 1,
        regime: "resampled".to_owned(),
        bound_mm: 0.306,
    });
    assert_ne!(
        base.digest(),
        crossed.digest(),
        "crossing a setup boundary left the manifest digest unchanged, so two \
         jobs that did different things would share an identity"
    );

    // And an exact crossing is still a crossing: it must not hash the same as
    // never having moved at all.
    let mut exact = base.clone();
    exact.boundaries.push(Boundary {
        into_setup: 1,
        regime: "exact".to_owned(),
        bound_mm: 0.0,
    });
    assert_ne!(base.digest(), exact.digest());
    assert_ne!(exact.digest(), crossed.digest());
}
