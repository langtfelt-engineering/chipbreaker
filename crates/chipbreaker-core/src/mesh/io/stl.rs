// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! STL, binary and ASCII.
//!
//! # Precision
//!
//! **Binary STL stores `f32`.** Coordinates are widened to `f64` on load, which
//! is exact — every `f32` is representable as an `f64` — but the source only ever
//! had a 24-bit mantissa. At a 100 mm coordinate that is a resolution of about
//! 6e-6 mm, so vertices that were coincident in the CAD system arrive differing
//! by a few units in the last `f32` place.
//!
//! That is precisely why [`crate::eps::EPS_WELD`] is 1e-6 mm rather than
//! something finer: a lattice below the `f32` noise floor would fail to weld
//! vertices that really are the same point, and leave the mesh full of spurious
//! boundary edges.
//!
//! # The stored normal is ignored
//!
//! Every facet carries a normal, and it is wrong often enough — zero,
//! unnormalised, or pointing inward — that trusting it is a liability. Normals
//! are always recomputed from the winding; see
//! [`crate::mesh::TriMesh::face_normal`].

use crate::math::Vec3;
use crate::mesh::io::ParseError;
use crate::mesh::units::Unit;
use crate::mesh::{MeshError, MeshMeta, TriMesh};

const FORMAT_BINARY: &str = "stl-binary";
const FORMAT_ASCII: &str = "stl-ascii";

/// Bytes before the triangle array: 80-byte header plus a `u32` count.
const HEADER_LEN: usize = 84;
/// Bytes per triangle: 12 `f32` plus a `u16` attribute count.
const TRIANGLE_LEN: usize = 50;

/// True if `bytes` has exactly the length a binary STL of its declared triangle
/// count would have.
///
/// Checked before any `solid` prefix, because the 80-byte header is arbitrary
/// and frequently begins with that word.
#[must_use]
pub fn looks_binary(bytes: &[u8]) -> bool {
    if bytes.len() < HEADER_LEN {
        return false;
    }
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    count
        .checked_mul(TRIANGLE_LEN)
        .and_then(|n| n.checked_add(HEADER_LEN))
        .is_some_and(|expected| expected == bytes.len())
}

fn mesh_error(format: &'static str, e: MeshError) -> ParseError {
    ParseError::general(format, e.to_string())
}

/// Reads a binary STL, scaling from `unit` to millimetres.
///
/// The declared triangle count is validated against the file length before any
/// triangle is read, so a truncated or padded file produces a specific error
/// rather than garbage geometry or a panic.
///
/// # Errors
/// [`ParseError`] for a short header, a count that disagrees with the file
/// length, or geometry the mesh constructor rejects.
pub fn read_binary(bytes: &[u8], unit: Unit) -> Result<TriMesh, ParseError> {
    if bytes.len() < HEADER_LEN {
        return Err(ParseError::at_offset(
            FORMAT_BINARY,
            bytes.len(),
            format!(
                "file is {} bytes; a binary STL needs at least {HEADER_LEN}",
                bytes.len()
            ),
        ));
    }
    let declared = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected = declared
        .checked_mul(TRIANGLE_LEN)
        .and_then(|n| n.checked_add(HEADER_LEN))
        .ok_or_else(|| {
            ParseError::at_offset(
                FORMAT_BINARY,
                80,
                format!("declared triangle count {declared} overflows any possible file"),
            )
        })?;
    if expected != bytes.len() {
        return Err(ParseError::at_offset(
            FORMAT_BINARY,
            80,
            format!(
                "header declares {declared} triangles, which needs {expected} bytes, \
                 but the file is {} bytes ({}); refusing to read past the end",
                bytes.len(),
                if bytes.len() < expected {
                    "truncated"
                } else {
                    "trailing data"
                }
            ),
        ));
    }

    let scale = unit.millimetres_per();
    let mut vertices = Vec::with_capacity(declared * 3);
    let mut triangles = Vec::with_capacity(declared);
    for i in 0..declared {
        // Skip the 12-byte normal: recomputed, never trusted.
        let base = HEADER_LEN + i * TRIANGLE_LEN + 12;
        let mut corner = [Vec3::ZERO; 3];
        for (c, slot) in corner.iter_mut().enumerate() {
            let mut xyz = [0.0f64; 3];
            for (a, out) in xyz.iter_mut().enumerate() {
                let at = base + c * 12 + a * 4;
                let raw =
                    f32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]]);
                // Widening f32 to f64 is exact; the scale multiply is the only
                // rounding, and it happens once.
                *out = f64::from(raw) * scale;
            }
            *slot = Vec3::new(xyz[0], xyz[1], xyz[2]);
        }
        let first = vertices.len() as u32;
        vertices.extend_from_slice(&corner);
        triangles.push([first, first + 1, first + 2]);
    }

    let meta = MeshMeta {
        source_format: FORMAT_BINARY.to_owned(),
        source_unit: unit,
        polygons_triangulated: 0,
        ignored_records: declared as u32,
    };
    TriMesh::new(vertices, triangles, meta).map_err(|e| mesh_error(FORMAT_BINARY, e))
}

/// Reads an ASCII STL, scaling from `unit` to millimetres.
///
/// Tolerates the malformations real files have: a missing `endsolid`, mixed line
/// endings, and a `solid` name containing spaces. Rust's `str::parse::<f64>` is
/// correctly rounded and platform-independent, so text parsing introduces no
/// determinism risk.
///
/// # Errors
/// [`ParseError`] naming the line, for a malformed vertex, a facet with the
/// wrong number of vertices, or geometry the mesh constructor rejects.
pub fn read_ascii(text: &str, unit: Unit) -> Result<TriMesh, ParseError> {
    let scale = unit.millimetres_per();
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut pending: Vec<Vec3> = Vec::new();
    let mut in_facet = false;
    let mut facets = 0usize;
    let mut saw_solid = false;
    let mut saw_endsolid = false;

    for (index, raw) in text.lines().enumerate() {
        let line = index + 1;
        // `lines()` strips \n and \r\n, and `split_whitespace` skips any
        // remaining leading whitespace as well as a lone \r left by a file with
        // mixed endings, so no explicit trim is needed.
        let mut tokens = raw.split_whitespace();
        let Some(keyword) = tokens.next() else {
            continue;
        };
        match keyword {
            // The name may contain spaces, so the rest of the line is ignored.
            "solid" => saw_solid = true,
            "endsolid" => saw_endsolid = true,
            "facet" | "outer" => {
                // "facet normal nx ny nz" and "outer loop". The normal is
                // deliberately not read.
                if keyword == "facet" {
                    in_facet = true;
                    pending.clear();
                }
            }
            "vertex" => {
                let mut xyz = [0.0f64; 3];
                for (a, out) in xyz.iter_mut().enumerate() {
                    let token = tokens.next().ok_or_else(|| {
                        ParseError::at_line(
                            FORMAT_ASCII,
                            line,
                            format!("vertex needs three coordinates, found {a}"),
                        )
                    })?;
                    let value: f64 = token.parse().map_err(|_| {
                        ParseError::at_line(
                            FORMAT_ASCII,
                            line,
                            format!("coordinate {a} is not a number: `{token}`"),
                        )
                    })?;
                    *out = value * scale;
                }
                pending.push(Vec3::new(xyz[0], xyz[1], xyz[2]));
            }
            "endloop" => {}
            "endfacet" => {
                if pending.len() != 3 {
                    return Err(ParseError::at_line(
                        FORMAT_ASCII,
                        line,
                        format!(
                            "facet has {} vertices; STL facets are triangles",
                            pending.len()
                        ),
                    ));
                }
                let first = vertices.len() as u32;
                vertices.extend_from_slice(&pending);
                triangles.push([first, first + 1, first + 2]);
                pending.clear();
                in_facet = false;
                facets += 1;
            }
            _ => {}
        }
    }

    if in_facet {
        return Err(ParseError::general(
            FORMAT_ASCII,
            "file ends inside a facet; it is truncated",
        ));
    }
    if !saw_solid && facets == 0 {
        return Err(ParseError::general(
            FORMAT_ASCII,
            "no `solid` header and no facets; this does not look like an ASCII STL",
        ));
    }
    // A missing `endsolid` is common and harmless once every facet has closed,
    // so it is counted rather than rejected.
    let meta = MeshMeta {
        source_format: FORMAT_ASCII.to_owned(),
        source_unit: unit,
        polygons_triangulated: 0,
        ignored_records: u32::from(!saw_endsolid),
    };
    TriMesh::new(vertices, triangles, meta).map_err(|e| mesh_error(FORMAT_ASCII, e))
}

/// Writes a binary STL in millimetres.
///
/// Coordinates narrow to `f32`, which is lossy — that is the format, not a
/// choice. A round trip through binary STL is bit-exact only for values that
/// were already representable in `f32`.
#[must_use]
pub fn write_binary(mesh: &TriMesh) -> Vec<u8> {
    let count = mesh.triangle_count();
    let mut out = Vec::with_capacity(HEADER_LEN + count as usize * TRIANGLE_LEN);
    // A fixed header rather than a timestamp or a hostname: the output must be
    // byte-identical for identical input, or round-trip tests become flaky and
    // golden hashes of written files become impossible.
    let mut header = [0u8; 80];
    let banner = b"chipbreaker";
    header[..banner.len()].copy_from_slice(banner);
    out.extend_from_slice(&header);
    out.extend_from_slice(&count.to_le_bytes());
    for i in 0..count {
        let tri = mesh.triangle(i);
        let n = mesh.face_normal(i).unwrap_or(Vec3::ZERO);
        for v in [n, tri[0], tri[1], tri[2]] {
            for c in v.to_array() {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// Writes an ASCII STL in millimetres.
///
/// Floats are formatted by [`ryu`]: the shortest decimal that parses back to the
/// same `f64`, so an ASCII round trip is **value-exact**.
#[must_use]
pub fn write_ascii(mesh: &TriMesh, name: &str) -> String {
    let mut buffer = ryu::Buffer::new();
    let mut out = String::new();
    out.push_str("solid ");
    out.push_str(name);
    out.push('\n');
    for i in 0..mesh.triangle_count() {
        let tri = mesh.triangle(i);
        let n = mesh.face_normal(i).unwrap_or(Vec3::ZERO);
        out.push_str("  facet normal");
        for c in n.to_array() {
            out.push(' ');
            out.push_str(buffer.format(c));
        }
        out.push_str("\n    outer loop\n");
        for v in tri {
            out.push_str("      vertex");
            for c in v.to_array() {
                out.push(' ');
                out.push_str(buffer.format(c));
            }
            out.push('\n');
        }
        out.push_str("    endloop\n  endfacet\n");
    }
    out.push_str("endsolid ");
    out.push_str(name);
    out.push('\n');
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::shapes;
    use crate::mesh::weld::weld;

    #[test]
    fn binary_round_trip_is_bit_exact_for_f32_representable_geometry() {
        // The cube's coordinates are small integers, so they survive the f32
        // narrowing exactly and the round trip must be bit-for-bit.
        let original = shapes::cube(10.0);
        let bytes = write_binary(&original);
        assert!(looks_binary(&bytes));
        let read = read_binary(&bytes, Unit::Millimetre).expect("reads");
        assert_eq!(read.triangle_count(), original.triangle_count());
        for i in 0..original.triangle_count() {
            assert_eq!(read.triangle(i), original.triangle(i), "triangle {i}");
        }
        assert_eq!(read.signed_volume(), original.signed_volume());
        // Writing the same mesh twice must give the same bytes.
        assert_eq!(write_binary(&original), bytes);
    }

    #[test]
    fn ascii_round_trip_is_value_exact_even_for_awkward_values() {
        // ryu guarantees the shortest decimal that parses back identically, so
        // this holds for values that have no short decimal form.
        let original = shapes::icosphere(7.0, 1);
        let text = write_ascii(&original, "part");
        let read = read_ascii(&text, Unit::Millimetre).expect("reads");
        assert_eq!(read.triangle_count(), original.triangle_count());
        for i in 0..original.triangle_count() {
            assert_eq!(read.triangle(i), original.triangle(i), "triangle {i}");
        }
        assert_eq!(read.signed_volume(), original.signed_volume());
    }

    #[test]
    fn stl_is_a_soup_that_welds_back_to_the_original_topology() {
        // STL has no shared vertices; welding is what restores topology.
        let original = shapes::cube(10.0);
        let read = read_binary(&write_binary(&original), Unit::Millimetre).expect("reads");
        assert_eq!(read.vertex_count(), 36, "a soup of 12 loose triangles");
        let (welded, report) = weld(&read, crate::eps::EPS_WELD).expect("welds");
        assert_eq!(report.vertices_after, 8);
        assert!(crate::mesh::validate::validate(&welded).is_solid());
    }

    #[test]
    fn units_are_applied_once_at_load() {
        let mesh = shapes::cube(1.0);
        let bytes = write_binary(&mesh);
        let as_inches = read_binary(&bytes, Unit::Inch).expect("reads");
        assert_eq!(as_inches.meta().source_unit, Unit::Inch);
        // A 1-unit cube read as inches is 25.4 mm on a side.
        assert!((as_inches.bounds().extent().x - 25.4).abs() < 1e-12);
        assert!((as_inches.signed_volume() - 25.4f64.powi(3)).abs() < 1e-6);
    }

    #[test]
    fn a_truncated_binary_file_is_a_specific_error() {
        let mut bytes = write_binary(&shapes::cube(1.0));
        bytes.truncate(bytes.len() - 10);
        let e = read_binary(&bytes, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("truncated"), "{e}");
        assert_eq!(e.offset, Some(80));
    }

    #[test]
    fn a_wrong_declared_count_is_a_specific_error() {
        let mut bytes = write_binary(&shapes::cube(1.0));
        bytes[80..84].copy_from_slice(&999u32.to_le_bytes());
        let e = read_binary(&bytes, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("999"), "{e}");
        assert!(!looks_binary(&bytes), "the length check must notice");

        // And a count so large it overflows the arithmetic.
        bytes[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(read_binary(&bytes, Unit::Millimetre).is_err());
    }

    #[test]
    fn a_header_only_file_reads_as_an_empty_mesh() {
        let mut bytes = vec![0u8; 84];
        bytes[80..84].copy_from_slice(&0u32.to_le_bytes());
        let m = read_binary(&bytes, Unit::Millimetre).expect("valid, if empty");
        assert!(m.is_empty());
        // Shorter than a header is not.
        assert!(read_binary(&[0u8; 20], Unit::Millimetre).is_err());
    }

    #[test]
    fn ascii_tolerates_the_malformations_real_files_have() {
        // Missing endsolid, CRLF endings, and a name with spaces.
        let text = "solid my great part\r\n\
             facet normal 0 0 0\r\n\
             outer loop\r\n\
             vertex 0 0 0\r\n\
             vertex 1 0 0\r\n\
             vertex 0 1 0\r\n\
             endloop\r\n\
             endfacet\r\n";
        let m = read_ascii(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(
            m.meta().ignored_records,
            1,
            "the missing endsolid is counted"
        );

        // Scientific notation, which exporters use freely.
        let text = "solid s\nfacet normal 0 0 1\nouter loop\n\
             vertex 1e-3 0 0\nvertex 1 0 0\nvertex 0 1.5E2 0\n\
             endloop\nendfacet\nendsolid s\n";
        let m = read_ascii(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangle(0)[0].x, 1e-3);
        assert_eq!(m.triangle(0)[2].y, 150.0);
    }

    #[test]
    fn ascii_rejects_malformed_input_with_a_line_number() {
        let bad = "solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0\nendloop\nendfacet\n";
        let e = read_ascii(bad, Unit::Millimetre).expect_err("must reject");
        assert_eq!(e.line, Some(4));
        assert!(e.to_string().contains("three coordinates"), "{e}");

        let bad = "solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 zero 0\nendloop\nendfacet\n";
        let e = read_ascii(bad, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("zero"), "{e}");

        // A facet with four vertices.
        let bad = "solid s\nfacet normal 0 0 1\nouter loop\n\
             vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nvertex 1 1 0\n\
             endloop\nendfacet\n";
        let e = read_ascii(bad, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("4 vertices"), "{e}");

        // Truncated inside a facet.
        let bad = "solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\n";
        let e = read_ascii(bad, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("truncated"), "{e}");

        // Not an STL at all.
        assert!(read_ascii("hello\nworld\n", Unit::Millimetre).is_err());
    }

    #[test]
    fn non_finite_geometry_is_rejected_at_the_boundary() {
        let text = "solid s\nfacet normal 0 0 1\nouter loop\n\
             vertex NaN 0 0\nvertex 1 0 0\nvertex 0 1 0\n\
             endloop\nendfacet\nendsolid s\n";
        let e = read_ascii(text, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("non-finite"), "{e}");

        // And through the binary path, where the f32 carries the infinity.
        let mut bytes = write_binary(&shapes::cube(1.0));
        bytes[96..100].copy_from_slice(&f32::INFINITY.to_le_bytes());
        let e = read_binary(&bytes, Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("non-finite"), "{e}");
    }

    #[test]
    fn the_stored_normal_is_ignored_and_recomputed() {
        // A file whose normals are all zero, or point the wrong way, must still
        // produce correct geometry.
        let text = "solid s\nfacet normal 0 0 0\nouter loop\n\
             vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n\
             endloop\nendfacet\nendsolid s\n";
        let m = read_ascii(text, Unit::Millimetre).expect("reads");
        assert_eq!(
            m.face_normal(0),
            Some(Vec3::Z),
            "recomputed from the winding"
        );
    }

    #[test]
    fn writing_is_deterministic() {
        // No timestamp, no hostname: identical input gives identical bytes, or
        // round-trip tests and golden hashes of written files are impossible.
        let m = shapes::icosphere(3.0, 1);
        assert_eq!(write_binary(&m), write_binary(&m));
        assert_eq!(write_ascii(&m, "p"), write_ascii(&m, "p"));
        assert!(write_ascii(&m, "p").starts_with("solid p\n"));
        assert!(write_ascii(&m, "p").ends_with("endsolid p\n"));
    }
}
