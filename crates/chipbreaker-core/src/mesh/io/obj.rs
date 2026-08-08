// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Wavefront OBJ.
//!
//! Only `v` and `f` are read. `vt`, `vn`, `usemtl`, `mtllib`, `g`, `o` and `s`
//! are ignored but **counted**, and the count is reported by `mesh inspect`, so
//! a user whose material assignments silently vanished can see that they did.
//!
//! # Polygon faces
//!
//! OBJ permits faces with any number of vertices. They are triangulated by a
//! **fan from the first vertex in declaration order**, which is deterministic
//! and cheap.
//!
//! It is also poor for non-convex faces: a fan from a reflex vertex produces
//! triangles that lie outside the polygon, so the resulting solid is wrong. This
//! is a real limitation, not a theoretical one — architectural and sheet-metal
//! exports contain non-convex faces routinely. The triangulated polygon count is
//! reported so the user can tell whether it applies to their file, and a proper
//! ear-clipping triangulation is the fix if it ever matters.

use crate::math::{Vec2, Vec3};
use crate::mesh::io::ParseError;
use crate::mesh::units::Unit;
use crate::mesh::{MeshMeta, TriMesh};
use crate::predicates::{Orientation, orient2d};

const FORMAT: &str = "obj";

/// True if a polygon is convex, decided exactly.
///
/// Fan triangulation from the first vertex is correct exactly when the polygon
/// is **star-shaped from that vertex**. Convexity is a cheap sufficient
/// condition — a convex polygon is star-shaped from every vertex — and erring
/// toward flagging is the right direction: a false positive costs a validation
/// finding on a face that happened to fan correctly, a false negative costs
/// silently wrong geometry.
///
/// The polygon is projected onto the coordinate plane its Newell normal is most
/// aligned with, which is the projection that cannot collapse it, and the turn
/// direction at each corner is then decided by exact [`orient2d`]. Collinear
/// corners are permitted: three points in a row on a straight edge do not make a
/// polygon non-convex.
fn is_convex(points: &[Vec3]) -> bool {
    if points.len() < 4 {
        // A triangle is convex, and is not fan-triangulated anyway.
        return true;
    }
    // Newell's method: robust for polygons that are only approximately planar,
    // which OBJ faces frequently are.
    let mut normal = Vec3::ZERO;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        normal += Vec3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    let n = normal.abs();
    let drop_axis = if n.x >= n.y && n.x >= n.z {
        0
    } else if n.y >= n.z {
        1
    } else {
        2
    };
    let project = |v: Vec3| match drop_axis {
        0 => Vec2::new(v.y, v.z),
        1 => Vec2::new(v.z, v.x),
        _ => Vec2::new(v.x, v.y),
    };

    let mut seen = Orientation::Zero;
    for i in 0..points.len() {
        let a = project(points[i]);
        let b = project(points[(i + 1) % points.len()]);
        let c = project(points[(i + 2) % points.len()]);
        let turn = orient2d(a, b, c);
        if turn == Orientation::Zero {
            continue;
        }
        if seen == Orientation::Zero {
            seen = turn;
        } else if seen != turn {
            return false;
        }
    }
    true
}

/// Resolves an OBJ face index to a zero-based vertex index.
///
/// OBJ indices are one-based, and **negative indices are relative to the end**
/// of the vertex list so far — `-1` is the most recently declared vertex. That
/// second form is rare enough to be forgotten and common enough to matter.
fn resolve(token: &str, declared: usize, line: usize) -> Result<u32, ParseError> {
    // "v/vt/vn": only the vertex part is read.
    let head = token.split('/').next().unwrap_or(token);
    let raw: i64 = head.parse().map_err(|_| {
        ParseError::at_line(
            FORMAT,
            line,
            format!("face index is not an integer: `{token}`"),
        )
    })?;
    let zero_based = if raw > 0 {
        raw - 1
    } else if raw < 0 {
        declared as i64 + raw
    } else {
        return Err(ParseError::at_line(
            FORMAT,
            line,
            "face index 0 is invalid; OBJ indices are one-based",
        ));
    };
    if zero_based < 0 || zero_based >= declared as i64 {
        return Err(ParseError::at_line(
            FORMAT,
            line,
            format!(
                "face references vertex {raw}, which resolves to {zero_based}, but \
                 only {declared} vertices have been declared"
            ),
        ));
    }
    u32::try_from(zero_based).map_err(|_| {
        ParseError::at_line(
            FORMAT,
            line,
            format!("vertex index {zero_based} exceeds u32"),
        )
    })
}

/// Reads an OBJ, scaling from `unit` to millimetres.
///
/// # Errors
/// [`ParseError`] naming the line for a malformed vertex or face, an
/// out-of-range index, or geometry the mesh constructor rejects.
pub fn read(text: &str, unit: Unit) -> Result<TriMesh, ParseError> {
    let scale = unit.millimetres_per();
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut ignored = 0u32;
    let mut polygons = 0u32;
    let mut non_convex = 0u32;

    for (index, raw) in text.lines().enumerate() {
        let line = index + 1;
        let trimmed = raw.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut tokens = trimmed.split_whitespace();
        let Some(keyword) = tokens.next() else {
            continue;
        };
        match keyword {
            "v" => {
                let mut xyz = [0.0f64; 3];
                for (a, out) in xyz.iter_mut().enumerate() {
                    let token = tokens.next().ok_or_else(|| {
                        ParseError::at_line(
                            FORMAT,
                            line,
                            format!("vertex needs three coordinates, found {a}"),
                        )
                    })?;
                    let value: f64 = token.parse().map_err(|_| {
                        ParseError::at_line(
                            FORMAT,
                            line,
                            format!("coordinate {a} is not a number: `{token}`"),
                        )
                    })?;
                    *out = value * scale;
                }
                vertices.push(Vec3::new(xyz[0], xyz[1], xyz[2]));
            }
            "f" => {
                let corners: Vec<&str> = tokens.collect();
                if corners.len() < 3 {
                    return Err(ParseError::at_line(
                        FORMAT,
                        line,
                        format!(
                            "face has {} vertices; at least three are needed",
                            corners.len()
                        ),
                    ));
                }
                if corners.len() > 3 {
                    polygons += 1;
                }
                let resolved: Result<Vec<u32>, ParseError> = corners
                    .iter()
                    .map(|c| resolve(c, vertices.len(), line))
                    .collect();
                let resolved = resolved?;
                if resolved.len() > 3 {
                    let loop_points: Vec<Vec3> =
                        resolved.iter().map(|i| vertices[*i as usize]).collect();
                    if !is_convex(&loop_points) {
                        // Recorded rather than rejected: the fan may still be
                        // correct if the face happens to be star-shaped from its
                        // first vertex. `validate` turns this into a finding, so
                        // the user is told rather than left to wonder.
                        non_convex += 1;
                    }
                }
                // Fan from the first vertex, in declaration order.
                for k in 1..resolved.len() - 1 {
                    triangles.push([resolved[0], resolved[k], resolved[k + 1]]);
                }
            }
            _ => ignored += 1,
        }
    }

    let meta = MeshMeta {
        source_format: FORMAT.to_owned(),
        source_unit: unit,
        polygons_triangulated: polygons,
        non_convex_polygons: non_convex,
        ignored_records: ignored,
    };
    TriMesh::new(vertices, triangles, meta).map_err(|e| ParseError::general(FORMAT, e.to_string()))
}

/// Writes an OBJ in millimetres.
///
/// Floats go through [`ryu`], so the round trip is value-exact. Indices are
/// written one-based and positive, which every consumer understands.
#[must_use]
pub fn write(mesh: &TriMesh) -> String {
    let mut buffer = ryu::Buffer::new();
    let mut out = String::new();
    out.push_str("# written by chipbreaker; coordinates are millimetres\n");
    for v in mesh.vertices() {
        out.push('v');
        for c in v.to_array() {
            out.push(' ');
            out.push_str(buffer.format(c));
        }
        out.push('\n');
    }
    for t in mesh.triangles() {
        out.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::shapes;

    #[test]
    fn round_trip_is_value_exact_and_preserves_topology() {
        // Unlike STL, OBJ is indexed, so the topology survives without welding.
        let original = shapes::icosphere(7.0, 1);
        let read_back = read(&write(&original), Unit::Millimetre).expect("reads");
        assert_eq!(read_back.vertices(), original.vertices());
        assert_eq!(read_back.triangles(), original.triangles());
        assert_eq!(read_back.signed_volume(), original.signed_volume());
        assert!(crate::mesh::validate::validate(&read_back).is_solid());
    }

    #[test]
    fn one_based_indices_are_resolved() {
        let m = read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", Unit::Millimetre).expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2]]);
    }

    #[test]
    fn negative_indices_are_relative_to_the_end() {
        // -1 is the most recently declared vertex. Easy to forget, and files in
        // the wild use it.
        let m = read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n", Unit::Millimetre).expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2]]);

        // And they are relative to the count *at that point*, not to the final
        // count, so a later vertex must not change an earlier face.
        let m = read(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\nv 9 9 9\n",
            Unit::Millimetre,
        )
        .expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2]]);
        assert_eq!(m.vertex_count(), 4);
    }

    #[test]
    fn slash_forms_are_accepted_and_the_extras_ignored() {
        let text = "v 0 0 0\nv 1 0 0\nv 0 1 0\n\
             vt 0 0\nvn 0 0 1\n\
             f 1/1/1 2/2/1 3/3/1\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2]]);
        assert_eq!(m.meta().ignored_records, 2, "vt and vn are counted");

        // The "v//vn" form, with no texture index.
        let m = read(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//1 2//1 3//1\n",
            Unit::Millimetre,
        )
        .expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2]]);
    }

    #[test]
    fn polygons_are_fan_triangulated_and_counted() {
        // A convex quad fans correctly into two triangles.
        let text = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangles(), [[0, 1, 2], [0, 2, 3]]);
        assert_eq!(m.meta().polygons_triangulated, 1);

        // A pentagon gives three triangles, all fanned from the first vertex.
        let text = "v 0 0 0\nv 1 0 0\nv 2 1 0\nv 1 2 0\nv 0 2 0\nf 1 2 3 4 5\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangle_count(), 3);
        assert!(
            m.triangles().iter().all(|t| t[0] == 0),
            "fanned from vertex 1"
        );
    }

    #[test]
    fn a_non_convex_face_triangulates_badly_and_the_count_says_so() {
        // Documents the known limitation, using a face that genuinely defeats
        // fan triangulation.
        //
        // Non-convexity alone is not enough: a fan from vertex 1 is correct
        // whenever the polygon is *star-shaped* from that vertex, which an
        // L-shape happens to be. The face has to be one where vertex 1 cannot
        // see every other vertex. This U — a 3x3 square with a notch cut down
        // from the top — is such a shape: the segment from (0,0) to (2,3)
        // leaves the polygon through the notch.
        //
        // True area is 9 - 2 = 7. The fan folds triangles back over themselves,
        // so the triangulated area exceeds it.
        let text = "v 0 0 0\nv 3 0 0\nv 3 3 0\nv 2 3 0\n\
                    v 2 1 0\nv 1 1 0\nv 1 3 0\nv 0 3 0\n\
                    f 1 2 3 4 5 6 7 8\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.meta().polygons_triangulated, 1);
        assert_eq!(m.triangle_count(), 6, "an octagon fans into six triangles");
        assert!(
            m.surface_area() > 7.0,
            "the fan should over-cover the U shape, got {}",
            m.surface_area()
        );

        // A convex polygon of the same vertex count fans correctly, which is
        // what makes the comparison meaningful rather than a property of size.
        let convex = "v 0 0 0\nv 2 0 0\nv 3 1 0\nv 3 2 0\n\
                      v 2 3 0\nv 0 3 0\nv -1 2 0\nv -1 1 0\n\
                      f 1 2 3 4 5 6 7 8\n";
        let c = read(convex, Unit::Millimetre).expect("reads");
        // And the loader says which is which, so `validate` can warn.
        assert_eq!(m.meta().non_convex_polygons, 1, "the U must be flagged");
        assert_eq!(
            c.meta().non_convex_polygons,
            0,
            "the convex one must not be"
        );
        // Shoelace area of that octagon is exactly 10.
        assert!(
            (c.surface_area() - 10.0).abs() < 1e-12,
            "{}",
            c.surface_area()
        );
    }

    #[test]
    fn convexity_is_decided_exactly_and_in_every_orientation() {
        let p = |x: f64, y: f64| Vec3::new(x, y, 0.0);
        // A square: convex, either winding.
        assert!(is_convex(&[
            p(0.0, 0.0),
            p(1.0, 0.0),
            p(1.0, 1.0),
            p(0.0, 1.0)
        ]));
        assert!(is_convex(&[
            p(0.0, 1.0),
            p(1.0, 1.0),
            p(1.0, 0.0),
            p(0.0, 0.0)
        ]));
        // A collinear point on one edge is not a corner, so still convex.
        assert!(is_convex(&[
            p(0.0, 0.0),
            p(0.5, 0.0),
            p(1.0, 0.0),
            p(1.0, 1.0),
            p(0.0, 1.0)
        ]));
        // An L: genuinely non-convex, even though it happens to fan correctly.
        // Flagging it is the conservative direction.
        assert!(!is_convex(&[
            p(0.0, 0.0),
            p(2.0, 0.0),
            p(2.0, 1.0),
            p(1.0, 1.0),
            p(1.0, 2.0),
            p(0.0, 2.0)
        ]));
        // Triangles are never fanned and are trivially convex.
        assert!(is_convex(&[p(0.0, 0.0), p(1.0, 0.0), p(0.0, 1.0)]));

        // The answer must not depend on which plane the face lies in; Newell's
        // normal chooses the projection.
        for axis in 0..3 {
            let q = |x: f64, y: f64| match axis {
                0 => Vec3::new(7.0, x, y),
                1 => Vec3::new(y, 7.0, x),
                _ => Vec3::new(x, y, 7.0),
            };
            assert!(
                is_convex(&[q(0.0, 0.0), q(1.0, 0.0), q(1.0, 1.0), q(0.0, 1.0)]),
                "axis {axis}"
            );
            assert!(
                !is_convex(&[
                    q(0.0, 0.0),
                    q(2.0, 0.0),
                    q(2.0, 1.0),
                    q(1.0, 1.0),
                    q(1.0, 2.0),
                    q(0.0, 2.0)
                ]),
                "axis {axis}"
            );
        }
    }

    #[test]
    fn a_non_convex_fan_becomes_a_validation_finding() {
        // A count in the metadata that nobody reads is not a warning; it has to
        // reach the report people actually look at.
        use crate::mesh::validate::{FindingKind, validate};
        let text = "v 0 0 0\nv 3 0 0\nv 3 3 0\nv 2 3 0\n\
                    v 2 1 0\nv 1 1 0\nv 1 3 0\nv 0 3 0\n\
                    f 1 2 3 4 5 6 7 8\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        let report = validate(&m);
        assert_eq!(report.count_of(FindingKind::NonConvexPolygonFan), 1);
        let finding = report
            .findings
            .iter()
            .find(|f| f.kind == FindingKind::NonConvexPolygonFan)
            .expect("present");
        assert!(finding.detail.contains("star-shaped"), "{}", finding.detail);

        // A convex face of the same size produces no such finding.
        let convex = "v 0 0 0\nv 2 0 0\nv 3 1 0\nv 3 2 0\n\
                      v 2 3 0\nv 0 3 0\nv -1 2 0\nv -1 1 0\n\
                      f 1 2 3 4 5 6 7 8\n";
        let c = read(convex, Unit::Millimetre).expect("reads");
        assert_eq!(validate(&c).count_of(FindingKind::NonConvexPolygonFan), 0);
    }

    #[test]
    fn comments_blank_lines_and_unknown_records_are_skipped() {
        let text = "# a comment\n\n   \n\
             mtllib part.mtl\ng group1\no object1\ns off\nusemtl steel\n\
             v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        let m = read(text, Unit::Millimetre).expect("reads");
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(m.meta().ignored_records, 5, "mtllib, g, o, s, usemtl");
    }

    #[test]
    fn malformed_input_names_the_line() {
        let e = read("v 0 0 0\nv 1 0\n", Unit::Millimetre).expect_err("must reject");
        assert_eq!(e.line, Some(2));
        assert!(e.to_string().contains("three coordinates"), "{e}");

        let e =
            read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2\n", Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("at least three"), "{e}");

        let e = read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n", Unit::Millimetre)
            .expect_err("must reject");
        assert!(e.to_string().contains("only 3 vertices"), "{e}");

        let e = read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n", Unit::Millimetre)
            .expect_err("must reject");
        assert!(e.to_string().contains("one-based"), "{e}");

        let e = read("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 x\n", Unit::Millimetre)
            .expect_err("must reject");
        assert!(e.to_string().contains("not an integer"), "{e}");

        // A negative index reaching past the start.
        let e = read("v 0 0 0\nf -5 -1 -1\n", Unit::Millimetre).expect_err("must reject");
        assert!(e.to_string().contains("resolves to"), "{e}");
    }

    #[test]
    fn units_are_applied_once_at_load() {
        let m = read("v 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n", Unit::Inch).expect("reads");
        assert_eq!(m.meta().source_unit, Unit::Inch);
        assert!((m.vertices()[0].x - 25.4).abs() < 1e-12);
    }

    #[test]
    fn writing_is_deterministic_and_one_based() {
        let m = shapes::cube(1.0);
        assert_eq!(write(&m), write(&m));
        let text = write(&m);
        assert!(
            text.contains("\nf 1 3 2\n"),
            "indices are one-based: {text}"
        );
        assert!(text.starts_with("# written by chipbreaker"));
    }

    #[test]
    fn an_empty_obj_is_an_empty_mesh() {
        let m = read("# nothing here\n", Unit::Millimetre).expect("valid, if empty");
        assert!(m.is_empty());
        assert_eq!(m.vertex_count(), 0);
    }
}
