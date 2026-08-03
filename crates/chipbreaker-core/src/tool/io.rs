// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The tool library file: a versioned JSON document.
//!
//! # Why versioned, and why it refuses the future
//!
//! A tool library outlives the program that wrote it. Someone will open a file
//! written by a later version, and there are two things the reader can do: guess,
//! or refuse. Guessing means silently ignoring fields it does not understand —
//! and a field it does not understand might be the one that says the holder is
//! 40 mm wider than it used to be. So an unknown [`TOOL_FILE_VERSION`] is a hard
//! error naming both versions, and unknown keys within a known version are
//! errors too.
//!
//! # Why the geometry is stored, not the dimensions
//!
//! The file holds segments and arcs, not "ball nose, 6 mm, 20 mm flute". A
//! catalogue entry is an input to [`super::catalog`]; what gets simulated is the
//! profile it produced. Storing the dimensions would mean a library written
//! today deserialises to a different solid if a constructor is ever corrected,
//! and a saved simulation would stop matching its own recorded hash. Storing the
//! geometry means the file says exactly what was cut with.
//!
//! # Round-tripping
//!
//! Floats are written by `serde_json`, which uses the shortest representation
//! that reads back to the same `f64`. Keys are emitted in sorted order because
//! the underlying map is a `BTreeMap`. Both together mean writing a library,
//! reading it, and writing it again produces byte-identical output — which is
//! what lets a library be hashed and put under golden-file control.

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec2;

use super::profile::{
    ArcDirection, ElementRole, Profile, ProfileElement, ProfileError, RoledElement,
};
use super::{Tool, ToolError, ToolId};

use serde_json::{Map, Value, json};

use core::fmt;

/// Schema marker written into every library file.
pub const TOOL_FILE_SCHEMA: &str = "chipbreaker.tool-library";

/// Version of the tool library format.
///
/// Bump this only when the meaning of an existing field changes or a required
/// field is added. Readers refuse anything they do not recognise; see the module
/// header for why.
pub const TOOL_FILE_VERSION: u32 = 1;

/// Why a tool library could not be read.
#[derive(Debug, Clone, PartialEq)]
pub enum ToolFileError {
    /// The bytes are not JSON at all.
    NotJson {
        /// What the parser said.
        detail: String,
    },
    /// The `schema` field is absent or names something else.
    WrongSchema {
        /// What the file claims to be.
        found: String,
    },
    /// The file was written by a version this reader does not understand.
    UnsupportedVersion {
        /// The version in the file.
        found: u64,
        /// The version this build writes and reads.
        supported: u32,
    },
    /// A required field is missing.
    MissingField {
        /// Where in the document.
        path: String,
        /// Which field.
        field: &'static str,
    },
    /// A field has the wrong JSON type.
    BadType {
        /// Where in the document.
        path: String,
        /// What was expected.
        expected: &'static str,
    },
    /// A field has an unusable value.
    BadValue {
        /// Where in the document.
        path: String,
        /// Why it cannot be used.
        reason: String,
    },
    /// A key this version does not define.
    ///
    /// Not ignored: an unrecognised key may be the one that matters.
    UnknownField {
        /// Where in the document.
        path: String,
        /// The offending key.
        field: String,
    },
    /// The geometry read back did not validate.
    Profile(ProfileError),
    /// The tool read back did not validate.
    Tool(ToolError),
}

impl fmt::Display for ToolFileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotJson { detail } => write!(f, "not valid JSON: {detail}"),
            Self::WrongSchema { found } => write!(
                f,
                "this is not a Chipbreaker tool library: schema is {found:?}, \
                 expected {TOOL_FILE_SCHEMA:?}"
            ),
            Self::UnsupportedVersion { found, supported } => write!(
                f,
                "tool library version {found} was written by a different build; \
                 this one reads and writes version {supported}"
            ),
            Self::MissingField { path, field } => {
                write!(f, "{path}: missing required field {field:?}")
            }
            Self::BadType { path, expected } => write!(f, "{path}: expected {expected}"),
            Self::BadValue { path, reason } => write!(f, "{path}: {reason}"),
            Self::UnknownField { path, field } => write!(
                f,
                "{path}: unknown field {field:?}; this build will not guess what it means"
            ),
            Self::Profile(e) => write!(f, "{e}"),
            Self::Tool(e) => write!(f, "{e}"),
        }
    }
}

impl core::error::Error for ToolFileError {}

impl From<ProfileError> for ToolFileError {
    fn from(e: ProfileError) -> Self {
        Self::Profile(e)
    }
}

impl From<ToolError> for ToolFileError {
    fn from(e: ToolError) -> Self {
        Self::Tool(e)
    }
}

/// A named collection of tools.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ToolLibrary {
    tools: Vec<Tool>,
}

impl ToolLibrary {
    /// An empty library.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds a library from tools, rejecting duplicate identifiers.
    ///
    /// # Errors
    ///
    /// [`ToolFileError::BadValue`] naming the identifier that repeats. A library
    /// with two tools of the same name resolves by position, and which one a
    /// lookup finds would depend on the order they were added.
    pub fn from_tools(tools: Vec<Tool>) -> Result<Self, ToolFileError> {
        for (i, tool) in tools.iter().enumerate() {
            if tools[..i].iter().any(|t| t.id() == tool.id()) {
                return Err(ToolFileError::BadValue {
                    path: format!("tools[{i}]"),
                    reason: format!("duplicate tool identifier {:?}", tool.id().as_str()),
                });
            }
        }
        Ok(Self { tools })
    }

    /// The tools, in file order.
    #[inline]
    #[must_use]
    pub fn tools(&self) -> &[Tool] {
        &self.tools
    }

    /// How many tools.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.tools.len()
    }

    /// True if the library holds no tools.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tools.is_empty()
    }

    /// Looks a tool up by identifier.
    #[must_use]
    pub fn get(&self, id: &str) -> Option<&Tool> {
        self.tools.iter().find(|t| t.id().as_str() == id)
    }

    /// Renders the library as JSON.
    #[must_use]
    pub fn to_json(&self) -> String {
        let tools: Vec<Value> = self.tools.iter().map(tool_to_json).collect();
        let document = json!({
            "schema": TOOL_FILE_SCHEMA,
            "version": TOOL_FILE_VERSION,
            "tools": tools,
        });
        let mut text = serde_json::to_string_pretty(&document).unwrap_or_default();
        text.push('\n');
        text
    }

    /// Parses a library from JSON.
    ///
    /// # Errors
    ///
    /// See [`ToolFileError`]. Every variant names where in the document the
    /// problem is, because a tool library is hand-edited more often than not.
    pub fn from_json(text: &str) -> Result<Self, ToolFileError> {
        let document: Value = serde_json::from_str(text).map_err(|e| ToolFileError::NotJson {
            detail: e.to_string(),
        })?;
        let root = object(&document, "$")?;
        known_keys(root, "$", &["schema", "version", "tools"])?;

        let schema = string(root, "$", "schema")?;
        if schema != TOOL_FILE_SCHEMA {
            return Err(ToolFileError::WrongSchema {
                found: schema.to_owned(),
            });
        }
        let version =
            root.get("version")
                .and_then(Value::as_u64)
                .ok_or(ToolFileError::MissingField {
                    path: "$".to_owned(),
                    field: "version",
                })?;
        if version != u64::from(TOOL_FILE_VERSION) {
            return Err(ToolFileError::UnsupportedVersion {
                found: version,
                supported: TOOL_FILE_VERSION,
            });
        }

        let entries =
            root.get("tools")
                .and_then(Value::as_array)
                .ok_or(ToolFileError::BadType {
                    path: "$.tools".to_owned(),
                    expected: "an array of tools",
                })?;

        let mut tools = Vec::with_capacity(entries.len());
        for (i, entry) in entries.iter().enumerate() {
            tools.push(tool_from_json(entry, &format!("$.tools[{i}]"))?);
        }
        Self::from_tools(tools)
    }
}

impl Hashable for ToolLibrary {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("ToolLibrary");
        h.u64(u64::from(TOOL_FILE_VERSION));
        h.usize(self.tools.len());
        for tool in &self.tools {
            h.add(tool);
        }
        h.end();
    }
}

fn tool_to_json(tool: &Tool) -> Value {
    let profile: Vec<Value> = tool
        .profile()
        .elements()
        .iter()
        .map(element_to_json)
        .collect();
    json!({
        "id": tool.id().as_str(),
        "description": tool.description(),
        "gauge_length": tool.gauge_length(),
        "profile": profile,
    })
}

fn element_to_json(roled: &RoledElement) -> Value {
    let point = |p: Vec2| json!([p.x, p.y]);
    match roled.element {
        ProfileElement::Segment { start, end } => json!({
            "kind": "segment",
            "role": roled.role.as_str(),
            "start": point(start),
            "end": point(end),
        }),
        ProfileElement::Arc {
            start,
            end,
            center,
            direction,
        } => json!({
            "kind": "arc",
            "role": roled.role.as_str(),
            "start": point(start),
            "end": point(end),
            "center": point(center),
            "direction": direction.as_str(),
        }),
    }
}

fn tool_from_json(value: &Value, path: &str) -> Result<Tool, ToolFileError> {
    let map = object(value, path)?;
    known_keys(map, path, &["id", "description", "gauge_length", "profile"])?;

    let id = ToolId::new(string(map, path, "id")?)?;
    let description = map
        .get("description")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_owned();
    let gauge_length =
        map.get("gauge_length")
            .and_then(Value::as_f64)
            .ok_or(ToolFileError::MissingField {
                path: path.to_owned(),
                field: "gauge_length",
            })?;

    let elements = map
        .get("profile")
        .and_then(Value::as_array)
        .ok_or(ToolFileError::BadType {
            path: format!("{path}.profile"),
            expected: "an array of profile elements",
        })?;

    let mut parsed = Vec::with_capacity(elements.len());
    for (i, element) in elements.iter().enumerate() {
        parsed.push(element_from_json(element, &format!("{path}.profile[{i}]"))?);
    }
    let profile = Profile::new(parsed)?;
    Ok(Tool::new(id, description, profile, gauge_length)?)
}

fn element_from_json(value: &Value, path: &str) -> Result<RoledElement, ToolFileError> {
    let map = object(value, path)?;
    let kind = string(map, path, "kind")?;
    let role = match string(map, path, "role")? {
        "cutting" => ElementRole::Cutting,
        "non-cutting" => ElementRole::NonCutting,
        "holder" => ElementRole::Holder,
        other => {
            return Err(ToolFileError::BadValue {
                path: path.to_owned(),
                reason: format!(
                    "role {other:?} is not one of \"cutting\", \"non-cutting\", \"holder\""
                ),
            });
        }
    };

    let element = match kind {
        "segment" => {
            known_keys(map, path, &["kind", "role", "start", "end"])?;
            ProfileElement::Segment {
                start: point(map, path, "start")?,
                end: point(map, path, "end")?,
            }
        }
        "arc" => {
            known_keys(
                map,
                path,
                &["kind", "role", "start", "end", "center", "direction"],
            )?;
            let direction = match string(map, path, "direction")? {
                "ccw" => ArcDirection::CounterClockwise,
                "cw" => ArcDirection::Clockwise,
                other => {
                    return Err(ToolFileError::BadValue {
                        path: path.to_owned(),
                        reason: format!("direction {other:?} is not \"cw\" or \"ccw\""),
                    });
                }
            };
            ProfileElement::Arc {
                start: point(map, path, "start")?,
                end: point(map, path, "end")?,
                center: point(map, path, "center")?,
                direction,
            }
        }
        other => {
            return Err(ToolFileError::BadValue {
                path: path.to_owned(),
                reason: format!("kind {other:?} is not \"segment\" or \"arc\""),
            });
        }
    };
    Ok(RoledElement { element, role })
}

fn object<'v>(value: &'v Value, path: &str) -> Result<&'v Map<String, Value>, ToolFileError> {
    value.as_object().ok_or_else(|| ToolFileError::BadType {
        path: path.to_owned(),
        expected: "an object",
    })
}

fn string<'m>(
    map: &'m Map<String, Value>,
    path: &str,
    field: &'static str,
) -> Result<&'m str, ToolFileError> {
    map.get(field)
        .and_then(Value::as_str)
        .ok_or_else(|| ToolFileError::MissingField {
            path: path.to_owned(),
            field,
        })
}

fn point(map: &Map<String, Value>, path: &str, field: &'static str) -> Result<Vec2, ToolFileError> {
    let array = map
        .get(field)
        .and_then(Value::as_array)
        .ok_or(ToolFileError::MissingField {
            path: path.to_owned(),
            field,
        })?;
    if array.len() != 2 {
        return Err(ToolFileError::BadValue {
            path: format!("{path}.{field}"),
            reason: format!("a point is [r, z]; found {} values", array.len()),
        });
    }
    let mut coordinates = [0.0f64; 2];
    for (slot, entry) in coordinates.iter_mut().zip(array) {
        *slot = entry.as_f64().ok_or_else(|| ToolFileError::BadType {
            path: format!("{path}.{field}"),
            expected: "two numbers",
        })?;
        if !slot.is_finite() {
            return Err(ToolFileError::BadValue {
                path: format!("{path}.{field}"),
                reason: "coordinates must be finite".to_owned(),
            });
        }
    }
    Ok(Vec2::new(coordinates[0], coordinates[1]))
}

/// Rejects any key the format does not define at this position.
fn known_keys(map: &Map<String, Value>, path: &str, allowed: &[&str]) -> Result<(), ToolFileError> {
    for key in map.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(ToolFileError::UnknownField {
                path: path.to_owned(),
                field: key.clone(),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool::catalog::{HolderStage, Shank, ball_end_mill, bull_end_mill, flat_end_mill};

    fn library() -> ToolLibrary {
        let tools = vec![
            Tool::new(
                ToolId::new("em6").expect("valid"),
                "6 mm flat, 3 flute",
                flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
                75.0,
            )
            .expect("valid"),
            Tool::new(
                ToolId::new("bn8").expect("valid"),
                "8 mm ball nose",
                ball_end_mill(8.0, 25.0, &Shank::plain(8.0, 60.0)).expect("valid"),
                90.0,
            )
            .expect("valid"),
            Tool::new(
                ToolId::new("bull10-r2").expect("valid"),
                "10 mm bull, 2 mm corner, in a shrink holder",
                bull_end_mill(
                    10.0,
                    2.0,
                    30.0,
                    &Shank::with_holder(
                        10.0,
                        55.0,
                        [
                            HolderStage::cylinder(30.0, 25.0),
                            HolderStage::taper(30.0, 48.0, 20.0),
                        ],
                    ),
                )
                .expect("valid"),
                140.0,
            )
            .expect("valid"),
        ];
        ToolLibrary::from_tools(tools).expect("distinct identifiers")
    }

    #[test]
    fn a_library_round_trips_through_json_unchanged() {
        let original = library();
        let text = original.to_json();
        let parsed = ToolLibrary::from_json(&text).expect("what we just wrote");
        assert_eq!(original, parsed);
    }

    #[test]
    fn writing_a_parsed_library_reproduces_the_bytes_exactly() {
        let text = library().to_json();
        let again = ToolLibrary::from_json(&text).expect("valid").to_json();
        assert_eq!(
            text, again,
            "a library must survive a write-read-write cycle byte for byte, \
             or it cannot be put under golden-file control"
        );
    }

    #[test]
    fn the_geometry_survives_exactly_not_approximately() {
        let original = library();
        let parsed = ToolLibrary::from_json(&original.to_json()).expect("valid");
        for (a, b) in original.tools().iter().zip(parsed.tools()) {
            for (ea, eb) in a.profile().elements().iter().zip(b.profile().elements()) {
                assert_eq!(
                    ea, eb,
                    "every coordinate must come back bit-identical, not merely close"
                );
            }
            assert_eq!(a.gauge_length(), b.gauge_length());
        }
    }

    #[test]
    fn a_library_hashes_the_same_after_a_round_trip() {
        let hash = |l: &ToolLibrary| {
            let mut h = CanonicalHash::new();
            h.add(l);
            h.finish().to_hex()
        };
        let original = library();
        let parsed = ToolLibrary::from_json(&original.to_json()).expect("valid");
        assert_eq!(hash(&original), hash(&parsed));
    }

    #[test]
    fn lookup_is_by_name_and_duplicates_are_refused() {
        let l = library();
        assert!(l.get("bn8").is_some());
        assert!(l.get("nonesuch").is_none());

        let duplicated = vec![l.tools()[0].clone(), l.tools()[0].clone()];
        let err = ToolLibrary::from_tools(duplicated).expect_err("two tools called em6");
        assert!(matches!(err, ToolFileError::BadValue { .. }), "{err:?}");
    }

    #[test]
    fn a_file_from_a_later_version_is_refused_rather_than_guessed_at() {
        let text = library()
            .to_json()
            .replace("\"version\": 1", "\"version\": 2");
        let err = ToolLibrary::from_json(&text).expect_err("version 2 is not this build");
        match err {
            ToolFileError::UnsupportedVersion { found, supported } => {
                assert_eq!(found, 2);
                assert_eq!(supported, TOOL_FILE_VERSION);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn an_unknown_field_is_an_error_not_something_to_ignore() {
        // The field a later version might add is exactly the field that matters.
        let text = library().to_json().replace(
            "\"kind\": \"segment\"",
            "\"kind\": \"segment\", \"taper\": 3.0",
        );
        let err = ToolLibrary::from_json(&text).expect_err("taper is not a field of this version");
        match err {
            ToolFileError::UnknownField { field, .. } => assert_eq!(field, "taper"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_file_that_is_not_a_tool_library_is_refused_by_name() {
        let err = ToolLibrary::from_json(r#"{"schema":"something.else","version":1,"tools":[]}"#)
            .expect_err("wrong schema");
        assert!(matches!(err, ToolFileError::WrongSchema { .. }), "{err:?}");
    }

    #[test]
    fn malformed_input_is_reported_with_a_path_to_the_problem() {
        let cases: Vec<(&str, &str)> = vec![
            ("not json at all", "NotJson"),
            (
                r#"{"schema":"chipbreaker.tool-library","version":1}"#,
                "BadType",
            ),
            (
                r#"{"schema":"chipbreaker.tool-library","version":1,"tools":[{}]}"#,
                "MissingField",
            ),
            (
                r#"{"schema":"chipbreaker.tool-library","version":1,"tools":[
                     {"id":"t","description":"","gauge_length":10.0,"profile":[
                       {"kind":"helix","role":"cutting","start":[0,0],"end":[1,0]}]}]}"#,
                "BadValue",
            ),
            (
                r#"{"schema":"chipbreaker.tool-library","version":1,"tools":[
                     {"id":"t","description":"","gauge_length":10.0,"profile":[
                       {"kind":"segment","role":"grinding","start":[0,0],"end":[1,0]}]}]}"#,
                "BadValue",
            ),
            (
                r#"{"schema":"chipbreaker.tool-library","version":1,"tools":[
                     {"id":"t","description":"","gauge_length":10.0,"profile":[
                       {"kind":"segment","role":"cutting","start":[0,0,0],"end":[1,0]}]}]}"#,
                "BadValue",
            ),
        ];
        for (text, expected) in cases {
            let err = ToolLibrary::from_json(text).expect_err(text);
            let rendered = format!("{err:?}");
            assert!(
                rendered.starts_with(expected),
                "expected a {expected} for {text}, got {rendered}"
            );
            // Every error must be readable, not only matchable.
            assert!(!err.to_string().is_empty());
        }
    }

    #[test]
    fn geometry_that_does_not_validate_is_refused_on_the_way_in() {
        // A profile that does not start at the tip. Hand-edited libraries are
        // the normal case, so the validation has to run on read, not only on
        // construction.
        let text = r#"{"schema":"chipbreaker.tool-library","version":1,"tools":[
             {"id":"t","description":"","gauge_length":10.0,"profile":[
               {"kind":"segment","role":"cutting","start":[1,0],"end":[3,0]}]}]}"#;
        let err = ToolLibrary::from_json(text).expect_err("does not begin at the tip");
        assert!(
            matches!(
                err,
                ToolFileError::Profile(ProfileError::TipNotAtOrigin { .. })
            ),
            "{err:?}"
        );
    }

    #[test]
    fn an_empty_library_is_valid() {
        let text = ToolLibrary::new().to_json();
        let parsed = ToolLibrary::from_json(&text).expect("valid");
        assert!(parsed.is_empty());
        assert_eq!(parsed.len(), 0);
    }

    #[test]
    fn the_file_names_itself_and_its_version() {
        let text = library().to_json();
        assert!(text.contains(TOOL_FILE_SCHEMA));
        assert!(text.contains("\"version\": 1"));
        assert!(text.ends_with('\n'), "text files end with a newline");
    }
}
