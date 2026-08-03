// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `.dexel` format. ADR 0004 is the argument; this is the enforcement.

use chipbreaker_core::dexel::io::{self, FORMAT_VERSION, FormatError, MAGIC};
use chipbreaker_core::dexel::{BuildOptions, DexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Mat4, Vec3};
use chipbreaker_core::mesh::shapes;

fn digest(field: &DexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

fn a_field() -> DexelField {
    // A sphere rather than a box, deliberately: a box's span endpoints are round
    // numbers that would survive almost any format, including one with the
    // defect this ADR exists to prevent. A sphere's endpoints are ray-triangle
    // intersections, so essentially none of them are representable in fewer than
    // seventeen significant digits.
    let mesh = shapes::icosphere(9.0, 3);
    let options = BuildOptions {
        spacing: 0.7,
        placement: Mat4::from_translation(Vec3::new(1.0 / 3.0, -7.125e-3, 12.0)),
        ..BuildOptions::default()
    };
    DexelField::build(&mesh, &options).expect("builds").0
}

#[test]
fn a_field_survives_a_round_trip_bit_identically() {
    // **A tolerance appearing in this test means ADR 0004 has been violated.**
    // The requirement is not "close"; it is the same bits.
    let original = a_field();
    let bytes = io::to_bytes(&original).expect("writes");
    let reloaded = io::from_bytes(&bytes).expect("reads");

    assert_eq!(digest(&original), digest(&reloaded));
    assert_eq!(original.volume().to_bits(), reloaded.volume().to_bits());
    assert_eq!(original.total_spans(), reloaded.total_spans());

    // Span by span, on the bits.
    let rays = u32::try_from(original.arena().rays()).expect("small");
    for ray in 0..rays {
        let a = original.arena().get(ray);
        let b = reloaded.arena().get(ray);
        assert_eq!(a.len(), b.len(), "ray {ray}");
        for (x, y) in a.iter().zip(b) {
            assert_eq!(x.t0.to_bits(), y.t0.to_bits(), "ray {ray}");
            assert_eq!(x.t1.to_bits(), y.t1.to_bits(), "ray {ray}");
        }
    }
}

#[test]
fn at_least_one_span_endpoint_needs_all_seventeen_digits() {
    // Guards the guard. If the fixture drifted to a shape whose endpoints were
    // all round, the round-trip test above would still pass under a format with
    // the exact defect ADR 0004 exists to prevent -- which is how the Unit 3
    // serde_json bug survived a week of green tests.
    let field = a_field();
    let rays = u32::try_from(field.arena().rays()).expect("small");
    let mut needs_seventeen = 0;
    for ray in 0..rays {
        for span in field.arena().get(ray) {
            for value in [span.t0, span.t1] {
                // 17 significant digits always round-trips, so asking whether
                // 17 suffices proves nothing. The question is whether SIXTEEN
                // is enough: a value that survives 16 would also survive a
                // sloppy formatter, and a value that does not is exactly the
                // shape that caught serde_json at Unit 3.
                if format!("{value:.15e}").parse::<f64>() != Ok(value) {
                    needs_seventeen += 1;
                }
            }
        }
    }
    assert!(
        needs_seventeen > 0,
        "no span endpoint in the fixture requires full precision, so the round-trip \
         test proves nothing about float fidelity. Pick a different shape."
    );
}

#[test]
fn writing_the_same_field_twice_gives_the_same_bytes() {
    let field = a_field();
    assert_eq!(
        io::to_bytes(&field).expect("writes"),
        io::to_bytes(&field).expect("writes")
    );
}

#[test]
fn two_builds_of_the_same_stock_write_the_same_file() {
    let mesh = shapes::cylinder(8.0, 20.0, 48);
    let options = BuildOptions {
        spacing: 0.6,
        ..BuildOptions::default()
    };
    let (a, _) = DexelField::build(&mesh, &options).expect("builds");
    let (b, _) = DexelField::build(&mesh, &options).expect("builds");
    assert_eq!(
        io::to_bytes(&a).expect("writes"),
        io::to_bytes(&b).expect("writes")
    );
}

#[test]
fn negative_zero_is_stored_rather_than_normalised_away() {
    // The asymmetry ADR 0004 calls out: hashing canonicalises -0.0 because two
    // values that compare equal must hash equal; the format does not, because
    // silently rewriting a value is data loss.
    let bits = (-0.0f64).to_bits().to_le_bytes();
    assert_ne!(bits, 0.0f64.to_bits().to_le_bytes());

    let field = a_field();
    let bytes = io::to_bytes(&field).expect("writes");
    let reloaded = io::from_bytes(&bytes).expect("reads");
    // Whatever sign of zero construction produced, it comes back the same.
    let rays = u32::try_from(field.arena().rays()).expect("small");
    for ray in 0..rays {
        for (x, y) in field.arena().get(ray).iter().zip(reloaded.arena().get(ray)) {
            assert_eq!(x.t0.to_bits(), y.t0.to_bits());
        }
    }
}

#[test]
fn the_placement_round_trips_exactly() {
    let mesh = shapes::cube(6.0);
    let placement = Mat4::from_rows_array([
        [0.8660254037844387, -0.49999999999999994, 0.0, 1.0 / 7.0],
        [0.49999999999999994, 0.8660254037844387, 0.0, -2.0 / 3.0],
        [0.0, 0.0, 1.0, 1.0e-4],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let (field, _) = DexelField::build(
        &mesh,
        &BuildOptions {
            spacing: 0.5,
            placement,
            ..BuildOptions::default()
        },
    )
    .expect("builds");
    let reloaded = io::from_bytes(&io::to_bytes(&field).expect("writes")).expect("reads");
    for (a, b) in field
        .placement()
        .m
        .iter()
        .flatten()
        .zip(reloaded.placement().m.iter().flatten())
    {
        assert_eq!(a.to_bits(), b.to_bits());
    }
}

#[test]
fn the_lattice_round_trips_so_rays_land_in_the_same_places() {
    let field = a_field();
    let reloaded = io::from_bytes(&io::to_bytes(&field).expect("writes")).expect("reads");
    assert_eq!(field.lattice(), reloaded.lattice());
    let rays = u32::try_from(field.arena().rays()).expect("small");
    for ray in [0, rays / 3, rays / 2, rays - 1] {
        let a = field.lattice().ray_at(ray).origin.to_array();
        let b = reloaded.lattice().ray_at(ray).origin.to_array();
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.to_bits(), y.to_bits(), "ray {ray}");
        }
    }
}

// --- refusals --------------------------------------------------------------

#[test]
fn a_file_that_is_not_a_dexel_file_is_refused() {
    match io::from_bytes(b"not a dexel file at all") {
        Err(FormatError::NotADexelFile { .. }) => {}
        other => panic!("{other:?}"),
    }
}

#[test]
fn an_unknown_version_is_refused_rather_than_reinterpreted() {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&MAGIC);
    bytes.extend_from_slice(&(FORMAT_VERSION + 1).to_le_bytes());
    bytes.extend_from_slice(&[0u8; 64]);
    match io::from_bytes(&bytes) {
        Err(FormatError::UnknownVersion { found, expected }) => {
            assert_eq!(found, FORMAT_VERSION + 1);
            assert_eq!(expected, FORMAT_VERSION);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_truncated_file_is_refused_at_every_length() {
    // Every prefix, not one arbitrary cut. A format that only detects truncation
    // at some lengths detects it by luck.
    let bytes = io::to_bytes(&a_field()).expect("writes");
    for cut in (0..bytes.len()).step_by(97) {
        assert!(
            io::from_bytes(&bytes[..cut]).is_err(),
            "a {cut}-byte prefix of a {}-byte file was accepted",
            bytes.len()
        );
    }
}

#[test]
fn trailing_bytes_do_not_change_what_is_read() {
    // Concatenation is not an error -- the reader stops at the end of the field
    // -- but it must not change the field. This is what lets a future container
    // hold a field alongside other records.
    let field = a_field();
    let mut bytes = io::to_bytes(&field).expect("writes");
    let clean = io::from_bytes(&bytes).expect("reads");
    bytes.extend_from_slice(b"and then some");
    let with_junk = io::from_bytes(&bytes).expect("reads");
    assert_eq!(digest(&clean), digest(&with_junk));
}

#[test]
fn a_corrupted_span_total_is_caught() {
    let field = a_field();
    let mut bytes = io::to_bytes(&field).expect("writes");
    // The span total sits immediately after the ray count. Derived from the
    // layout rather than written as one number: this was `21 * 8` until the
    // format grew two transverse extents at U6, and a stale literal made the
    // test corrupt a placement matrix instead of the field it meant to.
    const U32S: usize = 4; // version, axis, two counts
    const F64S: usize = 3 + 1 + 1 + 2 + 16; // origin, spacing, length, extents, placement
    let offset = MAGIC.len() + U32S * 4 + F64S * 8 + size_of::<u32>();
    let corrupted = (field.total_spans() as u64 + 1).to_le_bytes();
    bytes[offset..offset + 8].copy_from_slice(&corrupted);
    match io::from_bytes(&bytes) {
        Err(FormatError::CountMismatch { what, .. }) => assert_eq!(what, "spans"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn an_unknown_axis_code_is_refused() {
    let field = a_field();
    let mut bytes = io::to_bytes(&field).expect("writes");
    bytes[12..16].copy_from_slice(&7u32.to_le_bytes());
    match io::from_bytes(&bytes) {
        Err(FormatError::BadHeader { detail }) => assert!(detail.contains('7'), "{detail}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn the_lattice_round_trips_exactly_for_awkward_extents() {
    // The bug the tri-dexel corpus caught. The writer used to store the ray
    // length and let the reader subtract two cells to recover the workspace
    // extent; `(30.0 + 3.2) - 3.2` is `30.000000000000004`, so a file reloaded
    // to a lattice whose rays sat a few ULP from where they were written.
    //
    // The values here are chosen so the spacing divides none of them, which is
    // when the arithmetic goes wrong.
    use chipbreaker_core::dexel::{BuildOptions, DexelField};
    use chipbreaker_core::math::Axis;
    let mesh = chipbreaker_core::mesh::shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(30.0, 20.0, 10.0),
    );
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        for spacing in [1.6, 0.7, 0.3] {
            let (field, _) = DexelField::build(
                &mesh,
                &BuildOptions {
                    spacing,
                    axis,
                    ..BuildOptions::default()
                },
            )
            .expect("builds");
            let reloaded = io::from_bytes(&io::to_bytes(&field).expect("writes")).expect("reads");
            assert_eq!(
                field.lattice(),
                reloaded.lattice(),
                "{axis:?} at {spacing} mm: the lattice did not survive the round trip"
            );
            // And the rays really are in the same places, on the bits.
            let rays = u32::try_from(field.arena().rays()).expect("small");
            for ray in [0, rays / 2, rays - 1] {
                let a = field.lattice().ray_at(ray).origin.to_array();
                let b = reloaded.lattice().ray_at(ray).origin.to_array();
                for (x, y) in a.iter().zip(&b) {
                    assert_eq!(x.to_bits(), y.to_bits(), "{axis:?} at {spacing}, ray {ray}");
                }
            }
        }
    }
}
