// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `.dexel` file format: raw IEEE-754 bit patterns, little-endian.
//!
//! ADR 0004 has the argument in full. The short version is that Unit 3 caught
//! `serde_json` reading a float one ULP low — `2.0481555856608242` came back as
//! `2.048155585660824` — and a dexel field is millions of computed span
//! endpoints, essentially none of them round. A text format would make the
//! determinism contract depend on the correctness of every writer and reader
//! that ever touches a file. One ULP is not a rounding detail when the product
//! claim is bit-identical output; it is a spurious hash mismatch that looks
//! exactly like a real regression.
//!
//! So every float is `to_bits().to_le_bytes()`, every index and count is `u32`
//! little-endian, and the round-trip requirement is **bit-identical** rather
//! than approximate. If a tolerance ever appears in
//! `a_field_survives_a_round_trip_bit_identically`, ADR 0004 has been violated.
//!
//! # Layout
//!
//! ```text
//! magic       8 bytes   b"CBDEXEL\0"
//! version     u32       FORMAT_VERSION; a reader refuses anything else
//! axis        u32       0 = X, 1 = Y, 2 = Z
//! counts      2 x u32   rays along each lattice axis
//! origin      3 x f64   lower corner of the workspace
//! spacing     f64
//! length      f64       workspace extent along the ray axis
//! placement   16 x f64  the stock placement, row-major
//! rays        u32       counts[0] * counts[1], restated so truncation is caught
//! total_spans u64       restated for the same reason
//! per ray:    u32 count, then that many pairs of f64
//! ```
//!
//! Little-endian is fixed by the format rather than inherited from the host.
//! Every platform we target is little-endian, so today this costs nothing;
//! writing it down means a big-endian port byte-swaps instead of silently
//! producing files that disagree with everyone else's.
//!
//! # NaN and `-0.0` are stored as they are
//!
//! The canonical hashing layer canonicalises both, because two values that
//! compare equal must hash equal. **This does not.** Hashing answers "are these
//! the same field?"; serialization answers "can I reconstruct exactly what I
//! had?", and silently rewriting a value is data loss. Construction should not
//! produce a NaN at all — and if it ever does, the file preserving it is what
//! lets us find out where it came from.

use std::io::{Read, Write};

use crate::golden::Digest;
use crate::math::{Axis, Mat4, Vec3};
use crate::spans::Span;

use super::arena::Arena;
use super::field::DexelField;
use super::lattice::Lattice;
use super::tessellation::TessellationEstimate;
use super::tri::{AXES, Provenance, TriDexelField};

/// File magic. Eight bytes so the header stays aligned.
pub const MAGIC: [u8; 8] = *b"CBDEXEL\0";

/// Format version. Bumped whenever the layout changes; a reader refuses any
/// version it does not know rather than misinterpreting the bytes.
///
/// **2** added the two transverse workspace extents. Unit 6 found that anchoring
/// cells at the workspace minimum could put a cell centre exactly on the stock's
/// own face, so the lattice is now centred; recovering the centring offset on
/// read needs the true extent, because `counts * spacing` has already rounded
/// up. Version 1 files are refused rather than read with the wrong ray
/// positions.
pub const FORMAT_VERSION: u32 = 2;

/// Why a `.dexel` file could not be read or written.
#[derive(Debug)]
pub enum FormatError {
    /// The underlying reader or writer failed.
    Io(std::io::Error),
    /// The file does not start with [`MAGIC`].
    NotADexelFile {
        /// The first eight bytes found.
        found: [u8; 8],
    },
    /// The file declares a version this build does not know.
    UnknownVersion {
        /// What the file said.
        found: u32,
        /// What this build writes.
        expected: u32,
    },
    /// The file ended before the data it declared.
    Truncated {
        /// What was being read.
        what: &'static str,
    },
    /// A field in the header is not usable.
    BadHeader {
        /// What is wrong.
        detail: String,
    },
    /// The declared totals disagree with the data that followed.
    ///
    /// Cheap to check and it catches a truncated or concatenated file that
    /// happens to end on a record boundary.
    CountMismatch {
        /// What disagreed.
        what: &'static str,
        /// What the header declared.
        declared: u64,
        /// What was actually read.
        found: u64,
    },
}

impl core::fmt::Display for FormatError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::NotADexelFile { found } => write!(
                f,
                "this is not a .dexel file: it starts with {found:?}, not {MAGIC:?}"
            ),
            Self::UnknownVersion { found, expected } => write!(
                f,
                "this file is .dexel format version {found}; this build writes and reads \
                 version {expected}"
            ),
            Self::Truncated { what } => {
                write!(f, "the file ended in the middle of {what}")
            }
            Self::BadHeader { detail } => write!(f, "unusable header: {detail}"),
            Self::CountMismatch {
                what,
                declared,
                found,
            } => write!(
                f,
                "the header declares {declared} {what} but the file contains {found}"
            ),
        }
    }
}

impl core::error::Error for FormatError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for FormatError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Writes a field.
///
/// # Errors
/// [`FormatError::Io`] if the writer fails.
pub fn write<W: Write>(field: &DexelField, w: &mut W) -> Result<(), FormatError> {
    let lattice = field.lattice();
    let arena = field.arena();

    w.write_all(&MAGIC)?;
    put_u32(w, FORMAT_VERSION)?;
    put_u32(w, axis_code(lattice.axis()))?;
    let counts = lattice.counts();
    put_u32(w, counts[0])?;
    put_u32(w, counts[1])?;
    for value in lattice.origin().to_array() {
        put_f64(w, value)?;
    }
    put_f64(w, lattice.spacing())?;
    // The workspace extent along the ray axis, recovered from the ray length so
    // the lattice reconstructs exactly rather than approximately.
    put_f64(w, lattice.ray_length() - 2.0 * lattice.spacing())?;
    // The true transverse extents, which `counts * spacing` cannot recover.
    for value in lattice.extent() {
        put_f64(w, value)?;
    }
    for row in &field.placement().m {
        for value in row {
            put_f64(w, *value)?;
        }
    }

    let rays = u32::try_from(arena.rays()).unwrap_or(u32::MAX);
    put_u32(w, rays)?;
    put_u64(w, arena.total_spans() as u64)?;

    // Ascending ray index, which is the same order everything else in this
    // module uses. A different order here would produce a different file for
    // the same field.
    for ray in 0..rays {
        let spans = arena.get(ray);
        put_u32(w, u32::try_from(spans.len()).unwrap_or(u32::MAX))?;
        for span in spans {
            put_f64(w, span.t0)?;
            put_f64(w, span.t1)?;
        }
    }
    Ok(())
}

/// Reads a field.
///
/// # Errors
/// See [`FormatError`].
pub fn read<R: Read>(r: &mut R) -> Result<DexelField, FormatError> {
    let mut magic = [0u8; 8];
    fill(r, &mut magic, "the magic")?;
    if magic != MAGIC {
        return Err(FormatError::NotADexelFile { found: magic });
    }
    let version = get_u32(r, "the version")?;
    if version != FORMAT_VERSION {
        return Err(FormatError::UnknownVersion {
            found: version,
            expected: FORMAT_VERSION,
        });
    }

    let axis = axis_from_code(get_u32(r, "the axis")?)?;
    let counts = [get_u32(r, "the ray counts")?, get_u32(r, "the ray counts")?];
    let origin = Vec3::new(
        get_f64(r, "the origin")?,
        get_f64(r, "the origin")?,
        get_f64(r, "the origin")?,
    );
    let spacing = get_f64(r, "the spacing")?;
    let length = get_f64(r, "the length")?;
    let extent = [
        get_f64(r, "the transverse extents")?,
        get_f64(r, "the transverse extents")?,
    ];
    let mut placement = Mat4::ZERO;
    for row in 0..4 {
        for column in 0..4 {
            placement.m[row][column] = get_f64(r, "the placement")?;
        }
    }

    // Rebuilt from the stored parts rather than from a reconstructed box: the
    // centring offset depends on the TRUE extent, and `counts * spacing` has
    // already rounded up, so a box inferred from the counts would place every
    // ray slightly wrong.
    let lattice = Lattice::from_parts(axis, origin, spacing, counts, extent, length);
    if lattice.counts() != counts {
        return Err(FormatError::BadHeader {
            detail: format!(
                "the header says {counts:?} rays but the lattice gives {:?}",
                lattice.counts()
            ),
        });
    }

    let rays = get_u32(r, "the ray count")?;
    if rays as usize != lattice.ray_count() {
        return Err(FormatError::CountMismatch {
            what: "rays",
            declared: u64::from(rays),
            found: lattice.ray_count() as u64,
        });
    }
    let declared_spans = get_u64(r, "the span total")?;

    let mut arena = Arena::new(rays as usize);
    let mut spans: Vec<Span> = Vec::new();
    let mut total = 0u64;
    for ray in 0..rays {
        let count = get_u32(r, "a ray's span count")?;
        spans.clear();
        spans.reserve(count as usize);
        for _ in 0..count {
            let t0 = get_f64(r, "a span")?;
            let t1 = get_f64(r, "a span")?;
            spans.push(Span::new(t0, t1));
        }
        total += u64::from(count);
        if count > 0 {
            arena.set(ray, &spans);
        }
    }
    if total != declared_spans {
        return Err(FormatError::CountMismatch {
            what: "spans",
            declared: declared_spans,
            found: total,
        });
    }

    Ok(DexelField::from_parts(lattice, arena, placement))
}

/// Writes a field to a byte vector.
///
/// # Errors
/// See [`FormatError`].
pub fn to_bytes(field: &DexelField) -> Result<Vec<u8>, FormatError> {
    let mut out = Vec::new();
    write(field, &mut out)?;
    Ok(out)
}

/// Reads a field from bytes.
///
/// # Errors
/// See [`FormatError`].
pub fn from_bytes(bytes: &[u8]) -> Result<DexelField, FormatError> {
    read(&mut &bytes[..])
}

// --- primitives ------------------------------------------------------------
//
// Deliberately hand-written and deliberately boring. The whole point of ADR
// 0004 is that no library's defaults sit between a computed f64 and the bytes
// on disk.

fn put_u32<W: Write>(w: &mut W, value: u32) -> Result<(), FormatError> {
    w.write_all(&value.to_le_bytes())?;
    Ok(())
}

fn put_u64<W: Write>(w: &mut W, value: u64) -> Result<(), FormatError> {
    w.write_all(&value.to_le_bytes())?;
    Ok(())
}

/// The whole reason this module exists: bits out, not digits.
fn put_f64<W: Write>(w: &mut W, value: f64) -> Result<(), FormatError> {
    w.write_all(&value.to_bits().to_le_bytes())?;
    Ok(())
}

fn fill<R: Read>(r: &mut R, buffer: &mut [u8], what: &'static str) -> Result<(), FormatError> {
    match r.read_exact(buffer) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
            Err(FormatError::Truncated { what })
        }
        Err(e) => Err(FormatError::Io(e)),
    }
}

fn get_u32<R: Read>(r: &mut R, what: &'static str) -> Result<u32, FormatError> {
    let mut bytes = [0u8; 4];
    fill(r, &mut bytes, what)?;
    Ok(u32::from_le_bytes(bytes))
}

fn get_u64<R: Read>(r: &mut R, what: &'static str) -> Result<u64, FormatError> {
    let mut bytes = [0u8; 8];
    fill(r, &mut bytes, what)?;
    Ok(u64::from_le_bytes(bytes))
}

/// Bits in, not digits. See [`put_f64`].
fn get_f64<R: Read>(r: &mut R, what: &'static str) -> Result<f64, FormatError> {
    let mut bytes = [0u8; 8];
    fill(r, &mut bytes, what)?;
    Ok(f64::from_bits(u64::from_le_bytes(bytes)))
}

const fn axis_code(axis: Axis) -> u32 {
    match axis {
        Axis::X => 0,
        Axis::Y => 1,
        Axis::Z => 2,
    }
}

fn axis_from_code(code: u32) -> Result<Axis, FormatError> {
    match code {
        0 => Ok(Axis::X),
        1 => Ok(Axis::Y),
        2 => Ok(Axis::Z),
        other => Err(FormatError::BadHeader {
            detail: format!("axis code {other} is not one of 0 (x), 1 (y), 2 (z)"),
        }),
    }
}

// --- .tdx: three bundles ---------------------------------------------------

/// Magic for a tri-dexel file. Distinct from [`MAGIC`], so a reader given the
/// wrong one says so instead of misinterpreting the header.
pub const TDX_MAGIC: [u8; 8] = *b"CBTDX\0\0\0";

/// `.tdx` format version, **independent of [`FORMAT_VERSION`]**.
///
/// Versioned separately because the two formats will not change together: a
/// single-bundle field is still a useful thing to write at U10 and beyond, and
/// tying its version to this one would force pointless bumps.
pub const TDX_FORMAT_VERSION: u32 = 1;

/// Writes a tri-dexel field.
///
/// Layout: magic, version, then provenance, then a present-flag and (if
/// present) a full bundle record per axis in `AXES` order. Each bundle carries
/// **its own** axis, origin, spacing and counts, because the lattices are
/// deliberately not co-registered and U9 must be able to reason about their
/// relationship rather than assume one.
///
/// # Errors
/// [`FormatError::Io`] if the writer fails.
pub fn write_tri<W: Write>(field: &TriDexelField, w: &mut W) -> Result<(), FormatError> {
    w.write_all(&TDX_MAGIC)?;
    put_u32(w, TDX_FORMAT_VERSION)?;

    let p = field.provenance();
    w.write_all(p.source_digest.as_bytes())?;
    put_u32(w, p.source_triangles)?;
    put_f64(w, p.requested_spacing_mm)?;
    let t = &p.tessellation;
    put_u64(w, t.edges)?;
    put_u64(w, t.sharp_edges)?;
    put_f64(w, t.max_sagitta_mm)?;
    put_f64(w, t.percentile_sagitta_mm)?;
    put_f64(w, t.mean_edge_mm)?;
    put_f64(w, t.max_dihedral_deg)?;

    for axis in AXES {
        match field.bundle(axis) {
            Some(bundle) => {
                put_u32(w, 1)?;
                write(bundle, w)?;
            }
            None => put_u32(w, 0)?,
        }
    }
    Ok(())
}

/// Reads a tri-dexel field.
///
/// # Errors
/// See [`FormatError`].
pub fn read_tri<R: Read>(r: &mut R) -> Result<TriDexelField, FormatError> {
    let mut magic = [0u8; 8];
    fill(r, &mut magic, "the magic")?;
    if magic != TDX_MAGIC {
        return Err(FormatError::NotADexelFile { found: magic });
    }
    let version = get_u32(r, "the version")?;
    if version != TDX_FORMAT_VERSION {
        return Err(FormatError::UnknownVersion {
            found: version,
            expected: TDX_FORMAT_VERSION,
        });
    }

    let mut digest_bytes = [0u8; 32];
    fill(r, &mut digest_bytes, "the source digest")?;
    let provenance = Provenance {
        source_digest: Digest::from_bytes(digest_bytes),
        source_triangles: get_u32(r, "the triangle count")?,
        requested_spacing_mm: get_f64(r, "the spacing")?,
        tessellation: TessellationEstimate {
            edges: get_u64(r, "the edge count")?,
            sharp_edges: get_u64(r, "the sharp edge count")?,
            max_sagitta_mm: get_f64(r, "the max sagitta")?,
            percentile_sagitta_mm: get_f64(r, "the percentile sagitta")?,
            mean_edge_mm: get_f64(r, "the mean edge length")?,
            max_dihedral_deg: get_f64(r, "the max dihedral")?,
        },
    };

    let mut bundles: [Option<DexelField>; 3] = [None, None, None];
    for axis in AXES {
        match get_u32(r, "a bundle's present flag")? {
            0 => {}
            1 => bundles[axis.index()] = Some(read(r)?),
            other => {
                return Err(FormatError::BadHeader {
                    detail: format!("bundle present flag is {other}, not 0 or 1"),
                });
            }
        }
    }
    Ok(TriDexelField::from_parts(bundles, provenance))
}

/// Writes a tri-dexel field to bytes.
///
/// # Errors
/// See [`FormatError`].
pub fn tri_to_bytes(field: &TriDexelField) -> Result<Vec<u8>, FormatError> {
    let mut out = Vec::new();
    write_tri(field, &mut out)?;
    Ok(out)
}

/// Reads a tri-dexel field from bytes.
///
/// # Errors
/// See [`FormatError`].
pub fn tri_from_bytes(bytes: &[u8]) -> Result<TriDexelField, FormatError> {
    read_tri(&mut &bytes[..])
}

/// What kind of field a file holds, without committing to reading it.
///
/// Both formats stay readable, so a caller handed a path can dispatch rather
/// than guess from the extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldFormat {
    /// A single bundle, `.dexel`.
    Single,
    /// Three bundles, `.tdx`.
    Tri,
}

/// Identifies a field file from its first eight bytes.
#[must_use]
pub fn detect(bytes: &[u8]) -> Option<FieldFormat> {
    if bytes.starts_with(&MAGIC) {
        Some(FieldFormat::Single)
    } else if bytes.starts_with(&TDX_MAGIC) {
        Some(FieldFormat::Tri)
    } else {
        None
    }
}
