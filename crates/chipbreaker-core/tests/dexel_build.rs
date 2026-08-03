// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Field construction: what it builds, and what it refuses to build.

use chipbreaker_core::dexel::{BuildError, BuildOptions, DexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::{MeshMeta, TriMesh, shapes};

fn digest(field: &DexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

fn options(spacing: f64) -> BuildOptions {
    BuildOptions {
        spacing,
        ..BuildOptions::default()
    }
}

// --- what it builds --------------------------------------------------------

#[test]
fn a_box_is_captured_exactly_because_it_is_aligned_with_the_bundle() {
    // The one case a single-axis field gets right to machine precision: every
    // ray enters the top face and leaves the bottom, both analytic, and the
    // transverse silhouette is a rectangle the lattice tiles exactly. If this
    // is not near-exact, something is wrong with pairing rather than with
    // sampling.
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 10.0, 5.0));
    let (field, stats) = DexelField::build(&mesh, &options(0.5)).expect("builds");

    assert_eq!(stats.rays, 40 * 20);
    assert_eq!(stats.empty_rays, 0);
    assert_eq!(stats.spans, 40 * 20);
    assert_eq!(stats.spilled_rays, 0);
    assert_eq!(stats.predicates.coplanar_rejected, 0);

    let expected = 20.0 * 10.0 * 5.0;
    let error = (field.volume() - expected).abs() / expected;
    assert!(error < 1e-12, "volume {} vs {expected}", field.volume());
}

#[test]
fn a_sphere_is_captured_to_within_the_sampling_error() {
    let mesh = shapes::icosphere(10.0, 4);
    let (field, stats) = DexelField::build(&mesh, &options(0.1)).expect("builds");
    // Against the mesh, not the analytic sphere: an icosphere is a polyhedron
    // inscribed in one, and mixing the two error sources here would make this
    // test fail for a reason it is not testing. The convergence table separates
    // them properly.
    let expected = mesh.signed_volume();
    let error = (field.volume() - expected).abs() / expected;
    assert!(
        error < 1e-3,
        "volume {} vs {expected} ({error:e})",
        field.volume()
    );
    assert!(
        stats.empty_rays > 0,
        "the corners of the lattice are outside"
    );
}

#[test]
fn a_cavity_produces_two_spans_and_the_volume_excludes_it() {
    let mesh = nested_shells(10.0, 5.0);
    let (field, _) = DexelField::build(&mesh, &options(0.1)).expect("builds");
    let expected = mesh.signed_volume();
    let error = (field.volume() - expected).abs() / expected;
    assert!(error < 2e-3, "volume {} vs {expected}", field.volume());

    let distribution = field.arena().distribution();
    assert!(
        distribution.get(&2).copied().unwrap_or(0) > 0,
        "rays through the cavity must carry two spans: {distribution:?}"
    );
}

#[test]
fn a_hole_along_the_bundle_gives_empty_rays_not_two_span_rays() {
    // Recorded as a test because it is the assumption that was wrong when the
    // arena was sized. A torus whose axis runs ALONG the rays shows its hole as
    // rays that find nothing, not as rays that find two intervals. Two spans
    // need a TRANSVERSE hole, which the case below covers.
    let mesh = shapes::torus(20.0, 6.0, 64, 32);
    let (field, stats) = DexelField::build(&mesh, &options(0.5)).expect("builds");
    let distribution = field.arena().distribution();
    assert!(stats.empty_rays > 0, "the hole must show as empty rays");
    assert_eq!(
        distribution.get(&2).copied().unwrap_or(0),
        0,
        "an axis-along hole must not produce two-span rays: {distribution:?}"
    );
}

#[test]
fn a_transverse_hole_gives_two_span_rays() {
    // The same torus lying down. Its hole now runs across the bundle, so rays
    // through the middle enter, leave, enter and leave again.
    let mesh = shapes::torus(20.0, 6.0, 64, 32);
    let lying_down = BuildOptions {
        spacing: 0.5,
        // Rotate 90 degrees about X: the torus axis moves from Z to Y.
        placement: Mat4::from_rows_array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
        ..BuildOptions::default()
    };
    let (field, _) = DexelField::build(&mesh, &lying_down).expect("builds");
    let distribution = field.arena().distribution();
    assert!(
        distribution.get(&2).copied().unwrap_or(0) > 0,
        "a transverse hole must produce two-span rays: {distribution:?}"
    );
}

// --- what it refuses -------------------------------------------------------

#[test]
fn an_open_mesh_is_a_hard_error_rather_than_a_guess() {
    // One triangle removed from a face the bundle actually crosses. A ray
    // through the gap enters and never leaves, and there is no honest span to
    // record for it.
    //
    // Which triangle matters: dropping one from a side face parallel to the rays
    // changes nothing a Z-bundle can see, so the hole has to be in a face the
    // rays pass through. Picked by normal rather than by index, so a change to
    // the box tessellation order cannot quietly turn this test into a no-op.
    let closed = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0));
    let vertices = closed.vertices();
    let facing_the_rays = closed
        .triangles()
        .iter()
        .position(|t| {
            let a = vertices[t[0] as usize];
            let b = vertices[t[1] as usize];
            let c = vertices[t[2] as usize];
            let n = (b - a).cross(c - a);
            n.z.abs() > 0.9 * n.length()
        })
        .expect("a box has faces normal to Z");
    let mut triangles = closed.triangles().to_vec();
    triangles.remove(facing_the_rays);
    let open = TriMesh::new(closed.vertices().to_vec(), triangles, MeshMeta::synthetic())
        .expect("indices unchanged");

    match DexelField::build(&open, &options(1.0)) {
        Err(BuildError::OddCrossings {
            crossings, origin, ..
        }) => {
            assert_eq!(crossings % 2, 1);
            // The message has to say where, because that is the first question.
            assert!(origin.iter().all(|c| c.is_finite()));
        }
        other => panic!("an open mesh must abort the build, got {other:?}"),
    }
}

#[test]
fn a_face_lying_in_a_rays_own_plane_is_a_hard_error() {
    // ADR 0001 Part 2 requires this to abort rather than be counted. Reaching it
    // deliberately takes work, which is the point: the cell-centre offset makes
    // it unreachable for ordinary axis-aligned stock.
    //
    // Two boxes meeting at x = 2.5, with the whole spanning 0..4 at 1 mm
    // spacing, so ray origins land on 0.5, 1.5, 2.5, 3.5. The rays at x = 2.5
    // lie exactly in the plane of both boxes' shared faces.
    let left = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.5, 4.0, 4.0));
    let right = shapes::box_solid(Vec3::new(2.5, 0.0, 0.0), Vec3::new(4.0, 4.0, 4.0));
    let mut vertices = left.vertices().to_vec();
    let mut triangles = left.triangles().to_vec();
    let offset = left.vertex_count();
    vertices.extend_from_slice(right.vertices());
    triangles.extend(
        right
            .triangles()
            .iter()
            .map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]),
    );
    let touching = TriMesh::new(vertices, triangles, MeshMeta::synthetic()).expect("valid");

    match DexelField::build(&touching, &options(1.0)) {
        Err(BuildError::Coplanar { rejected, .. }) => assert!(rejected > 0),
        other => panic!("a coplanar face must abort the build, got {other:?}"),
    }
}

#[test]
fn a_degenerate_placement_is_refused_before_any_ray_is_cast() {
    let mesh = shapes::cube(10.0);
    let flattened = BuildOptions {
        placement: Mat4::from_scale(Vec3::new(1.0, 1.0, 0.0)),
        ..options(1.0)
    };
    assert!(matches!(
        DexelField::build(&mesh, &flattened),
        Err(BuildError::BadPlacement { .. })
    ));
}

#[test]
fn an_empty_mesh_is_refused() {
    let empty = TriMesh::new(Vec::new(), Vec::new(), MeshMeta::synthetic()).expect("valid");
    assert!(matches!(
        DexelField::build(&empty, &options(1.0)),
        Err(BuildError::EmptyMesh)
    ));
}

// --- placement -------------------------------------------------------------

#[test]
fn translating_the_stock_moves_the_material_and_not_its_volume() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 6.0));
    let (at_origin, _) = DexelField::build(&mesh, &options(0.5)).expect("builds");
    let moved_options = BuildOptions {
        placement: Mat4::from_translation(Vec3::new(100.0, -50.0, 25.0)),
        ..options(0.5)
    };
    let (moved, _) = DexelField::build(&mesh, &moved_options).expect("builds");

    assert!((at_origin.volume() - moved.volume()).abs() < 1e-9);
    let a = at_origin.material_bounds();
    let b = moved.material_bounds();
    assert!((b.min.x - a.min.x - 100.0).abs() < 1e-9, "{a:?} {b:?}");
    assert!((b.min.y - a.min.y + 50.0).abs() < 1e-9, "{a:?} {b:?}");
    assert!((b.min.z - a.min.z - 25.0).abs() < 1e-9, "{a:?} {b:?}");
}

#[test]
fn a_mirrored_placement_still_yields_positive_material() {
    // A negative-determinant transform inverts every face. Without reversing the
    // winding, every ray would report leaving before entering and the field
    // would be the complement of the stock -- which shows up as spans running
    // from the entry of one solid to the entry of the next, not as an error.
    let mesh = shapes::box_solid(Vec3::new(1.0, 1.0, 1.0), Vec3::new(11.0, 9.0, 7.0));
    let mirrored = BuildOptions {
        placement: Mat4::from_scale(Vec3::new(-1.0, 1.0, 1.0)),
        ..options(0.5)
    };
    let (plain, _) = DexelField::build(&mesh, &options(0.5)).expect("builds");
    let (flipped, _) = DexelField::build(&mesh, &mirrored).expect("builds");
    assert!(flipped.volume() > 0.0);
    assert!(
        (plain.volume() - flipped.volume()).abs() / plain.volume() < 1e-12,
        "{} vs {}",
        plain.volume(),
        flipped.volume()
    );
}

#[test]
fn a_margin_adds_empty_rays_and_changes_nothing_else() {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0));
    let (tight, tight_stats) = DexelField::build(&mesh, &options(0.5)).expect("builds");
    let with_margin = BuildOptions {
        margin: 2.0,
        ..options(0.5)
    };
    let (roomy, roomy_stats) = DexelField::build(&mesh, &with_margin).expect("builds");

    assert!(roomy_stats.rays > tight_stats.rays);
    assert!(roomy_stats.empty_rays > 0);
    assert_eq!(tight_stats.empty_rays, 0);
    assert!((tight.volume() - roomy.volume()).abs() / tight.volume() < 1e-12);
}

// --- determinism -----------------------------------------------------------

#[test]
fn building_the_same_stock_twice_gives_the_same_field() {
    let mesh = shapes::icosphere(12.0, 3);
    let (a, _) = DexelField::build(&mesh, &options(0.4)).expect("builds");
    let (b, _) = DexelField::build(&mesh, &options(0.4)).expect("builds");
    assert_eq!(digest(&a), digest(&b));
    assert_eq!(a.volume().to_bits(), b.volume().to_bits());
}

#[test]
fn the_field_hash_separates_lattice_placement_and_contents() {
    let mesh = shapes::cube(10.0);
    let (base, _) = DexelField::build(&mesh, &options(0.5)).expect("builds");
    let (finer, _) = DexelField::build(&mesh, &options(0.25)).expect("builds");
    let (moved, _) = DexelField::build(
        &mesh,
        &BuildOptions {
            placement: Mat4::from_translation(Vec3::new(5.0, 0.0, 0.0)),
            ..options(0.5)
        },
    )
    .expect("builds");

    assert_ne!(digest(&base), digest(&finer));
    assert_ne!(digest(&base), digest(&moved));
}

#[test]
fn every_axis_measures_the_same_box() {
    // An axis-aligned box is the one solid a bundle along any axis captures
    // exactly, so all three must agree. When they stop agreeing, the culprit is
    // `Axis::cyclic` and not the geometry.
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(12.0, 8.0, 4.0));
    let expected = 12.0 * 8.0 * 4.0;
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let (field, stats) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing: 0.5,
                axis,
                ..BuildOptions::default()
            },
        )
        .expect("builds");
        assert_eq!(stats.empty_rays, 0, "{axis:?}");
        let error = (field.volume() - expected).abs() / expected;
        assert!(error < 1e-12, "{axis:?}: {} vs {expected}", field.volume());
    }
}

#[test]
fn an_empty_field_measures_zero() {
    let mesh = shapes::cube(10.0);
    let (field, _) = DexelField::build(&mesh, &options(1.0)).expect("builds");
    let blank = DexelField::empty(field.lattice().clone());
    assert_eq!(blank.volume(), 0.0);
    assert_eq!(blank.total_spans(), 0);
    assert!(blank.material_bounds().is_empty());
}

// --- helpers ---------------------------------------------------------------

/// A sphere with a smaller reversed sphere inside it: a solid with a cavity.
fn nested_shells(outer: f64, inner: f64) -> TriMesh {
    let shell = shapes::icosphere(outer, 3);
    let hole = shapes::icosphere(inner, 3);
    let offset = shell.vertex_count();
    let mut vertices = shell.vertices().to_vec();
    let mut triangles = shell.triangles().to_vec();
    vertices.extend_from_slice(hole.vertices());
    triangles.extend(
        hole.triangles()
            .iter()
            .map(|t| [t[0] + offset, t[2] + offset, t[1] + offset]),
    );
    TriMesh::new(vertices, triangles, shell.meta().clone()).expect("valid")
}
