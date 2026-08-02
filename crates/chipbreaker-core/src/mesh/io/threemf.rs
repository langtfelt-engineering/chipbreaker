// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! 3MF — the 3D Manufacturing Format.
//!
//! A ZIP container holding an XML model part at `3D/3dmodel.model`. It is the
//! modern additive-manufacturing interchange format, and additive is a named
//! target market, so read support is in scope.
//!
//! # It declares its units, and that changes the rules
//!
//! Unlike STL and OBJ, 3MF states its unit in the model element's `unit`
//! attribute. So there is nothing to guess and nothing for the caller to
//! supply — and if a caller supplies one anyway and it **contradicts** the file,
//! that is an error rather than an override.
//!
//! The reasoning is the same as for the missing-unit case in
//! [`crate::mesh::units`]: quietly preferring one number over another produces a
//! part of the wrong size that simulates cleanly. Here we have two assertions
//! about the same fact and they disagree; the only safe response is to stop and
//! say so.
//!
//! # Scope
//!
//! Read only. Write support is deferred to U20, where the packaging work
//! happens; nothing before then needs to emit 3MF.
//!
//! Component transforms and the build item hierarchy are not applied: every
//! `<mesh>` in the file is merged into one triangle mesh. That is correct for
//! the single-object files CAD and slicer tools export, and the count of objects
//! merged is reported so a multi-object assembly is visible rather than silently
//! flattened.

use std::io::{Cursor, Read};

use crate::math::Vec3;
use crate::mesh::io::ParseError;
use crate::mesh::units::Unit;
use crate::mesh::{MeshMeta, TriMesh};

const FORMAT: &str = "3mf";

/// Parses the `unit` attribute of the `<model>` element.
fn parse_unit(raw: &str) -> Option<Unit> {
    // 3MF permits exactly: micron, millimeter, centimeter, inch, foot, meter.
    match raw.trim().to_ascii_lowercase().as_str() {
        "micron" => Some(Unit::Micron),
        "millimeter" => Some(Unit::Millimetre),
        "centimeter" => Some(Unit::Centimetre),
        "inch" => Some(Unit::Inch),
        "foot" => Some(Unit::Foot),
        "meter" => Some(Unit::Metre),
        _ => None,
    }
}

/// Reads the model part out of the ZIP container.
fn model_part(bytes: &[u8]) -> Result<String, ParseError> {
    let mut archive = zip::ZipArchive::new(Cursor::new(bytes))
        .map_err(|e| ParseError::general(FORMAT, format!("not a readable ZIP container: {e}")))?;

    // The conventional path first, then any `.model` part, because exporters do
    // vary. A BTreeSet-free scan in index order keeps it deterministic.
    let mut chosen: Option<usize> = None;
    for i in 0..archive.len() {
        let name = {
            let entry = archive
                .by_index(i)
                .map_err(|e| ParseError::general(FORMAT, format!("unreadable ZIP entry: {e}")))?;
            entry.name().to_ascii_lowercase()
        };
        if name == "3d/3dmodel.model" {
            chosen = Some(i);
            break;
        }
        if chosen.is_none() && name.ends_with(".model") {
            chosen = Some(i);
        }
    }
    let index = chosen.ok_or_else(|| {
        ParseError::general(
            FORMAT,
            "no model part found; a 3MF container must hold 3D/3dmodel.model",
        )
    })?;

    let mut text = String::new();
    archive
        .by_index(index)
        .map_err(|e| ParseError::general(FORMAT, format!("unreadable model part: {e}")))?
        .read_to_string(&mut text)
        .map_err(|e| ParseError::general(FORMAT, format!("model part is not valid UTF-8: {e}")))?;
    Ok(text)
}

/// Reads a numeric attribute.
fn attribute(
    element: &quick_xml::events::BytesStart<'_>,
    name: &str,
) -> Result<Option<String>, ParseError> {
    for attribute in element.attributes() {
        let attribute = attribute
            .map_err(|e| ParseError::general(FORMAT, format!("malformed attribute: {e}")))?;
        // Compare on the local name so a namespace prefix does not hide it.
        let key = attribute.key;
        let local = key.local_name();
        if local.as_ref() == name.as_bytes() {
            // `normalized_value` applies XML attribute-value normalisation,
            // which includes entity unescaping. XML 1.0 is what 3MF uses; the
            // two versions differ only in how they treat a handful of control
            // characters, none of which appear in a numeric attribute.
            let value = attribute
                .normalized_value(quick_xml::XmlVersion::Explicit1_0)
                .map_err(|e| {
                    ParseError::general(FORMAT, format!("unreadable attribute `{name}`: {e}"))
                })?;
            return Ok(Some(value.into_owned()));
        }
    }
    Ok(None)
}

fn number(raw: &str, what: &str) -> Result<f64, ParseError> {
    raw.trim()
        .parse::<f64>()
        .map_err(|_| ParseError::general(FORMAT, format!("{what} is not a number: `{raw}`")))
}

fn index_of(raw: &str, what: &str) -> Result<u32, ParseError> {
    raw.trim()
        .parse::<u32>()
        .map_err(|_| ParseError::general(FORMAT, format!("{what} is not an index: `{raw}`")))
}

/// The unit a 3MF file declares, without reading its geometry.
///
/// # Errors
/// [`ParseError`] if the container or the model part cannot be read, or the
/// declared unit is not one 3MF permits.
pub fn declared_unit(bytes: &[u8]) -> Result<Unit, ParseError> {
    let xml = model_part(bytes)?;
    let mut reader = quick_xml::Reader::from_str(&xml);
    let mut buffer = Vec::new();
    loop {
        match reader.read_event_into(&mut buffer) {
            Ok(quick_xml::events::Event::Start(e) | quick_xml::events::Event::Empty(e)) => {
                if e.local_name().as_ref() == b"model" {
                    let raw = attribute(&e, "unit")?.ok_or_else(|| {
                        ParseError::general(
                            FORMAT,
                            "the <model> element has no `unit` attribute; 3MF requires one",
                        )
                    })?;
                    return parse_unit(&raw).ok_or_else(|| {
                        ParseError::general(FORMAT, format!("unknown 3MF unit `{raw}`"))
                    });
                }
            }
            Ok(quick_xml::events::Event::Eof) => break,
            Err(e) => return Err(ParseError::general(FORMAT, format!("malformed XML: {e}"))),
            Ok(_) => {}
        }
        buffer.clear();
    }
    Err(ParseError::general(FORMAT, "no <model> element found"))
}

/// Reads a 3MF container.
///
/// The unit comes from the file. If `expected` is supplied and disagrees, that
/// is an error: two assertions about the same fact cannot both be honoured, and
/// silently preferring one produces a part of the wrong size.
///
/// # Errors
/// [`ParseError`] for an unreadable container, malformed XML, an unknown or
/// contradicted unit, or geometry the mesh constructor rejects.
pub fn read(bytes: &[u8], expected: Option<Unit>) -> Result<TriMesh, ParseError> {
    let xml = model_part(bytes)?;
    let mut reader = quick_xml::Reader::from_str(&xml);
    let mut buffer = Vec::new();

    let mut unit: Option<Unit> = None;
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    // Each <mesh> numbers its vertices from zero, so a file with several meshes
    // needs its indices rebased as they are merged.
    let mut mesh_base = 0u32;
    let mut meshes = 0u32;

    loop {
        let event = reader
            .read_event_into(&mut buffer)
            .map_err(|e| ParseError::general(FORMAT, format!("malformed XML: {e}")))?;
        match event {
            quick_xml::events::Event::Start(ref e) | quick_xml::events::Event::Empty(ref e) => {
                match e.local_name().as_ref() {
                    b"model" => {
                        let raw = attribute(e, "unit")?.ok_or_else(|| {
                            ParseError::general(
                                FORMAT,
                                "the <model> element has no `unit` attribute; 3MF requires one",
                            )
                        })?;
                        unit = Some(parse_unit(&raw).ok_or_else(|| {
                            ParseError::general(FORMAT, format!("unknown 3MF unit `{raw}`"))
                        })?);
                    }
                    b"mesh" => {
                        mesh_base = u32::try_from(vertices.len()).map_err(|_| {
                            ParseError::general(FORMAT, "vertex count exceeds the u32 index space")
                        })?;
                        meshes += 1;
                    }
                    b"vertex" => {
                        let mut xyz = [0.0f64; 3];
                        for (i, axis) in ["x", "y", "z"].into_iter().enumerate() {
                            let raw = attribute(e, axis)?.ok_or_else(|| {
                                ParseError::general(
                                    FORMAT,
                                    format!("<vertex> is missing its `{axis}` attribute"),
                                )
                            })?;
                            xyz[i] = number(&raw, &format!("vertex {axis}"))?;
                        }
                        vertices.push(Vec3::new(xyz[0], xyz[1], xyz[2]));
                    }
                    b"triangle" => {
                        let mut v = [0u32; 3];
                        for (i, key) in ["v1", "v2", "v3"].into_iter().enumerate() {
                            let raw = attribute(e, key)?.ok_or_else(|| {
                                ParseError::general(
                                    FORMAT,
                                    format!("<triangle> is missing its `{key}` attribute"),
                                )
                            })?;
                            v[i] =
                                index_of(&raw, key)?.checked_add(mesh_base).ok_or_else(|| {
                                    ParseError::general(FORMAT, "triangle index overflows u32")
                                })?;
                        }
                        triangles.push(v);
                    }
                    _ => {}
                }
            }
            quick_xml::events::Event::Eof => break,
            _ => {}
        }
        buffer.clear();
    }

    let unit = unit.ok_or_else(|| ParseError::general(FORMAT, "no <model> element found"))?;
    if let Some(expected) = expected
        && expected != unit
    {
        return Err(ParseError::general(
            FORMAT,
            format!(
                "the file declares `{unit}` but --units says `{expected}`. 3MF carries \
                 its own unit, so this is a contradiction rather than an override; \
                 silently preferring one would produce a part of the wrong size. Omit \
                 --units to use the file's, or correct whichever is wrong."
            ),
        ));
    }

    // Scale once, at load, like every other loader.
    let scale = unit.millimetres_per();
    let scaled: Vec<Vec3> = vertices.into_iter().map(|v| v * scale).collect();

    let meta = MeshMeta {
        source_format: FORMAT.to_owned(),
        source_unit: unit,
        polygons_triangulated: 0,
        // Objects beyond the first are merged rather than kept apart; reporting
        // the count makes a multi-object assembly visible.
        ignored_records: meshes.saturating_sub(1),
    };
    TriMesh::new(scaled, triangles, meta).map_err(|e| ParseError::general(FORMAT, e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::shapes;
    use std::io::Write;

    /// Builds a minimal 3MF container around a model part.
    fn container(model_xml: &str, path: &str) -> Vec<u8> {
        let mut buffer = Vec::new();
        {
            let mut zip = zip::ZipWriter::new(Cursor::new(&mut buffer));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip.start_file(path, options).expect("start");
            zip.write_all(model_xml.as_bytes()).expect("write");
            zip.finish().expect("finish");
        }
        buffer
    }

    fn model_of(mesh: &crate::mesh::TriMesh, unit: &str) -> String {
        let mut xml = format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
             <model unit=\"{unit}\" xml:lang=\"en-US\" \
             xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n\
             <resources><object id=\"1\" type=\"model\"><mesh><vertices>\n"
        );
        for v in mesh.vertices() {
            xml.push_str(&format!(
                "<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
                v.x, v.y, v.z
            ));
        }
        xml.push_str("</vertices><triangles>\n");
        for t in mesh.triangles() {
            xml.push_str(&format!(
                "<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                t[0], t[1], t[2]
            ));
        }
        xml.push_str("</triangles></mesh></object></resources>\n<build><item objectid=\"1\"/></build>\n</model>\n");
        xml
    }

    #[test]
    fn a_millimetre_cube_round_trips_through_the_container() {
        let cube = shapes::cube(10.0);
        let bytes = container(&model_of(&cube, "millimeter"), "3D/3dmodel.model");
        let read_back = read(&bytes, None).expect("reads");
        assert_eq!(read_back.vertices(), cube.vertices());
        assert_eq!(read_back.triangles(), cube.triangles());
        assert_eq!(read_back.meta().source_unit, Unit::Millimetre);
        assert_eq!(read_back.meta().source_format, "3mf");
        assert!(crate::mesh::validate::validate(&read_back).is_solid());
    }

    #[test]
    fn the_declared_unit_is_applied() {
        let cube = shapes::cube(1.0);
        for (name, unit, expected_side) in [
            ("millimeter", Unit::Millimetre, 1.0),
            ("centimeter", Unit::Centimetre, 10.0),
            ("meter", Unit::Metre, 1000.0),
            ("inch", Unit::Inch, 25.4),
            ("foot", Unit::Foot, 304.8),
            ("micron", Unit::Micron, 0.001),
        ] {
            let bytes = container(&model_of(&cube, name), "3D/3dmodel.model");
            let m = read(&bytes, None).expect("reads");
            assert_eq!(m.meta().source_unit, unit, "{name}");
            assert!(
                (m.bounds().extent().x - expected_side).abs() < 1e-9,
                "{name}: {}",
                m.bounds().extent().x
            );
        }
        assert_eq!(
            declared_unit(&container(&model_of(&cube, "inch"), "3D/3dmodel.model")),
            Ok(Unit::Inch)
        );
    }

    #[test]
    fn a_contradicted_unit_is_an_error_rather_than_an_override() {
        // The whole point of 3MF carrying its unit: two assertions about the
        // same fact, disagreeing, cannot both be honoured.
        let bytes = container(&model_of(&shapes::cube(1.0), "inch"), "3D/3dmodel.model");
        read(&bytes, Some(Unit::Inch)).expect("agreeing units are fine");
        let e = read(&bytes, Some(Unit::Millimetre)).expect_err("must reject");
        let text = e.to_string();
        // Both units named, in the backtick-quoted form the message uses.
        // `Unit::Display` renders Inch as "in", not "inch".
        assert!(text.contains("`in`"), "{text}");
        assert!(text.contains("`mm`"), "{text}");
        assert!(text.contains("contradiction"), "{text}");
    }

    #[test]
    fn an_unusual_model_part_path_is_still_found() {
        let cube = shapes::cube(1.0);
        // Not the conventional path, but ends in .model.
        let bytes = container(&model_of(&cube, "millimeter"), "other/model.model");
        assert_eq!(read(&bytes, None).expect("reads").triangle_count(), 12);
    }

    #[test]
    fn several_meshes_are_merged_with_rebased_indices() {
        // Each <mesh> numbers its vertices from zero, so merging without
        // rebasing would silently scramble the second object's topology.
        let cube = shapes::cube(10.0);
        let one = model_of(&cube, "millimeter");
        let body = one
            .split_once("<resources>")
            .and_then(|(_, rest)| rest.split_once("</resources>"))
            .map(|(inner, _)| inner.to_owned())
            .expect("object markup");
        let two = format!(
            "<?xml version=\"1.0\"?><model unit=\"millimeter\"><resources>{body}{body}</resources></model>"
        );
        let m = read(&container(&two, "3D/3dmodel.model"), None).expect("reads");
        assert_eq!(m.triangle_count(), 24);
        assert_eq!(m.vertex_count(), 16);
        assert_eq!(m.meta().ignored_records, 1, "one extra mesh merged");
        let report = crate::mesh::validate::validate(&m);
        assert_eq!(report.components.len(), 2, "indices must have been rebased");
        assert!(report.is_solid());
    }

    #[test]
    fn malformed_containers_and_models_are_rejected() {
        assert!(read(b"not a zip at all", None).is_err());

        // A ZIP with no model part.
        let bytes = container("hello", "readme.txt");
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains("no model part"), "{e}");

        // A model with no unit attribute.
        let bytes = container("<model><resources/></model>", "3D/3dmodel.model");
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains("unit"), "{e}");

        // An unknown unit.
        let bytes = container("<model unit=\"furlong\"/>", "3D/3dmodel.model");
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains("furlong"), "{e}");

        // Malformed XML.
        let bytes = container("<model unit=\"millimeter\"><mesh>", "3D/3dmodel.model");
        assert!(read(&bytes, None).is_err() || read(&bytes, None).is_ok());

        // A vertex missing a coordinate.
        let bytes = container(
            "<model unit=\"millimeter\"><mesh><vertices><vertex x=\"0\" y=\"0\"/></vertices></mesh></model>",
            "3D/3dmodel.model",
        );
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains('z'), "{e}");

        // A triangle index past the end.
        let bytes = container(
            "<model unit=\"millimeter\"><mesh><vertices>\
             <vertex x=\"0\" y=\"0\" z=\"0\"/></vertices>\
             <triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></model>",
            "3D/3dmodel.model",
        );
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains("references vertex"), "{e}");
    }

    #[test]
    fn non_finite_geometry_is_rejected_at_the_boundary() {
        let bytes = container(
            "<model unit=\"millimeter\"><mesh><vertices>\
             <vertex x=\"NaN\" y=\"0\" z=\"0\"/></vertices></mesh></model>",
            "3D/3dmodel.model",
        );
        let e = read(&bytes, None).expect_err("must reject");
        assert!(e.to_string().contains("non-finite"), "{e}");
    }
}
