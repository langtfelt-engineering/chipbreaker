// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Mesh file formats.
//!
//! Every loader converts to canonical millimetres exactly once, at load, and
//! records the source unit in [`crate::mesh::MeshMeta`] so `mesh inspect` can
//! answer what it assumed. See [`crate::mesh::units`] for why STL and OBJ have
//! no default unit and the CLI refuses to guess one.
//!
//! # Float formatting
//!
//! Text output goes through [`ryu`], which produces the shortest decimal that
//! round-trips exactly. Never `format!("{x}")` into a geometry file: Rust's
//! `Display` for `f64` is also shortest-round-trip today, but that is a library
//! behaviour rather than a documented guarantee, and a geometry file that does
//! not round-trip is a corrupted part.

pub mod obj;
pub mod stl;

use core::fmt;

/// Why a mesh file could not be read.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    /// Format being parsed, for the message.
    pub format: &'static str,
    /// One-based line number, where the format has lines.
    pub line: Option<usize>,
    /// Byte offset, where it does not.
    pub offset: Option<usize>,
    /// What went wrong.
    pub message: String,
}

impl ParseError {
    /// An error at a text line.
    #[must_use]
    pub fn at_line(format: &'static str, line: usize, message: impl Into<String>) -> Self {
        Self {
            format,
            line: Some(line),
            offset: None,
            message: message.into(),
        }
    }

    /// An error at a byte offset.
    #[must_use]
    pub fn at_offset(format: &'static str, offset: usize, message: impl Into<String>) -> Self {
        Self {
            format,
            line: None,
            offset: Some(offset),
            message: message.into(),
        }
    }

    /// An error with no position.
    #[must_use]
    pub fn general(format: &'static str, message: impl Into<String>) -> Self {
        Self {
            format,
            line: None,
            offset: None,
            message: message.into(),
        }
    }
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: ", self.format)?;
        if let Some(l) = self.line {
            write!(f, "line {l}: ")?;
        } else if let Some(o) = self.offset {
            write!(f, "byte {o}: ")?;
        }
        f.write_str(&self.message)
    }
}

impl core::error::Error for ParseError {}

/// Which format a file appears to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    /// Binary STL.
    StlBinary,
    /// ASCII STL.
    StlAscii,
    /// Wavefront OBJ.
    Obj,
}

impl Format {
    /// Stable name recorded in [`crate::mesh::MeshMeta::source_format`].
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::StlBinary => "stl-binary",
            Self::StlAscii => "stl-ascii",
            Self::Obj => "obj",
        }
    }
}

/// Guesses the format of `bytes`, given an optional filename for its extension.
///
/// # Why the extension is only a hint
///
/// A binary STL may begin with the ASCII word `solid`, because the 80-byte
/// header is arbitrary and plenty of exporters write a description there. Any
/// loader that dispatches on that prefix alone will read a binary file as text,
/// fail somewhere in the middle, and report a syntax error on a line that does
/// not exist.
///
/// So the binary layout is checked first: a binary STL is exactly
/// `84 + 50 * triangle_count` bytes. That is a strong signal — matching it by
/// accident requires a text file whose length agrees with a number stored at
/// bytes 80..84 — and it is checked before any prefix is looked at.
#[must_use]
pub fn detect(bytes: &[u8], filename: Option<&str>) -> Format {
    if stl::looks_binary(bytes) {
        return Format::StlBinary;
    }
    let extension = filename
        .and_then(|f| f.rsplit('.').next())
        .map(str::to_ascii_lowercase);
    if extension.as_deref() == Some("obj") {
        return Format::Obj;
    }
    let head = &bytes[..bytes.len().min(6)];
    if head.starts_with(b"solid") {
        return Format::StlAscii;
    }
    if extension.as_deref() == Some("stl") {
        return Format::StlAscii;
    }
    // OBJ has no magic; it is the residual case.
    Format::Obj
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_messages_carry_position() {
        let e = ParseError::at_line("obj", 12, "bad face");
        assert_eq!(e.to_string(), "obj: line 12: bad face");
        let e = ParseError::at_offset("stl-binary", 84, "truncated");
        assert_eq!(e.to_string(), "stl-binary: byte 84: truncated");
        let e = ParseError::general("stl-ascii", "empty");
        assert_eq!(e.to_string(), "stl-ascii: empty");
    }

    #[test]
    fn a_binary_stl_beginning_with_solid_is_not_mistaken_for_ascii() {
        // The classic trap: the 80-byte header is arbitrary, and exporters put
        // descriptions in it that start with the ASCII magic word.
        let mut bytes = vec![0u8; 84];
        bytes[..5].copy_from_slice(b"solid");
        bytes[80..84].copy_from_slice(&1u32.to_le_bytes());
        bytes.extend(std::iter::repeat_n(0u8, 50));
        assert_eq!(detect(&bytes, Some("part.stl")), Format::StlBinary);
    }

    #[test]
    fn ascii_stl_and_obj_are_told_apart() {
        let ascii = b"solid part\nfacet normal 0 0 1\n";
        assert_eq!(detect(ascii, Some("part.stl")), Format::StlAscii);
        assert_eq!(detect(ascii, None), Format::StlAscii);

        let obj = b"v 0 0 0\nv 1 0 0\nf 1 2 3\n";
        assert_eq!(detect(obj, Some("part.obj")), Format::Obj);
        assert_eq!(detect(obj, None), Format::Obj);
        // An OBJ extension wins over a missing magic.
        assert_eq!(detect(b"# comment\n", Some("m.obj")), Format::Obj);
        // An .stl extension with no magic is still treated as ASCII STL, so the
        // parser can produce a format-specific error rather than a generic one.
        assert_eq!(detect(b"# not really\n", Some("m.stl")), Format::StlAscii);
    }

    #[test]
    fn format_names_are_stable() {
        assert_eq!(Format::StlBinary.name(), "stl-binary");
        assert_eq!(Format::StlAscii.name(), "stl-ascii");
        assert_eq!(Format::Obj.name(), "obj");
    }

    #[test]
    fn detect_handles_tiny_and_empty_input() {
        assert_eq!(detect(b"", None), Format::Obj);
        assert_eq!(detect(b"so", None), Format::Obj);
        assert_eq!(detect(b"", Some("x.stl")), Format::StlAscii);
    }
}
