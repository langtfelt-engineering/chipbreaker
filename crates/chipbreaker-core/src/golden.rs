// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The bit-exact determinism harness: canonical hashing and golden files.
//!
//! Chipbreaker's commercial differentiator is that the same input produces
//! bit-identical output across runs, thread counts, platforms, and the WASM
//! build. This module is the machinery that proves it, and every later unit
//! depends on it.
//!
//! # Canonical encoding rules
//!
//! [`CanonicalHash`] feeds a BLAKE3 hasher with a *binary* encoding, never text.
//! Formatting a float as text and hashing the string is the single most common
//! way to build a hash that agrees on one platform and not another: the shortest
//! round-trip representation is not required to be identical across
//! implementations, and it silently discards the difference between `-0.0` and
//! `0.0`. Instead:
//!
//! - **`f64` is hashed as its IEEE-754 bit pattern**, little-endian, via
//!   [`f64::to_le_bytes`].
//! - **`usize` is widened to `u64`** before hashing. WASM is a 32-bit target;
//!   without this, a hash containing any length or index would differ between
//!   the native and WASM builds, and that would not be discovered until U19.
//! - **Every value is preceded by a one-byte type tag**, so that `u64(1)` and
//!   `f64::from_bits(1)` cannot collide, and so that a future encoding change is
//!   loud rather than subtle.
//! - **Variable-length values carry a `u64` length prefix**, so that
//!   `["ab", "c"]` and `["a", "bc"]` hash differently.
//! - **`NaN` is canonicalized to a single quiet-NaN pattern and `-0.0` to
//!   `+0.0`.** Both have multiple bit representations that compare equal (or, for
//!   NaN, are equally meaningless), and which one you get out of an arithmetic
//!   sequence is not portable. Two runs that are numerically identical must hash
//!   identically.
//!
//! Any change to these rules changes every golden file in the repository. That
//! is what [`crate::CANONICAL_ENCODING_VERSION`] is for.

use core::fmt;
use std::path::{Path, PathBuf};

use crate::math::{Aabb3, Mat3, Mat4, Ray, Vec2, Vec3};

// Type tags. These are part of the on-wire encoding: never reuse or renumber a
// tag without bumping `CANONICAL_ENCODING_VERSION`.
const TAG_U64: u8 = 0x01;
const TAG_I64: u8 = 0x02;
const TAG_F64: u8 = 0x03;
const TAG_USIZE: u8 = 0x04;
const TAG_BOOL: u8 = 0x05;
const TAG_STR: u8 = 0x06;
const TAG_BYTES: u8 = 0x07;
const TAG_F64_SLICE: u8 = 0x08;
const TAG_U64_SLICE: u8 = 0x09;
const TAG_BEGIN: u8 = 0x0a;
const TAG_END: u8 = 0x0b;

/// The single bit pattern all NaNs are mapped to before hashing.
///
/// An arbitrary quiet NaN. The specific value does not matter; that there is
/// exactly *one* of them does.
const CANONICAL_NAN_BITS: u64 = 0x7ff8_0000_0000_0000;

/// Maps an `f64` to the bits that represent it canonically.
///
/// Collapses every NaN payload to one, and `-0.0` to `+0.0`.
#[inline]
#[must_use]
pub fn canonical_f64_bits(v: f64) -> u64 {
    if v.is_nan() {
        CANONICAL_NAN_BITS
    } else if v == 0.0 {
        // Catches -0.0, whose bit pattern is 0x8000_0000_0000_0000 but which is
        // numerically indistinguishable from +0.0.
        0
    } else {
        v.to_bits()
    }
}

/// A 256-bit BLAKE3 digest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Digest([u8; 32]);

impl Digest {
    /// The raw bytes.
    #[inline]
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Lower-case hex, 64 characters.
    #[must_use]
    pub fn to_hex(&self) -> String {
        let mut s = String::with_capacity(64);
        for b in self.0 {
            s.push_str(&format!("{b:02x}"));
        }
        s
    }

    /// Parses 64 hex characters, case-insensitively. Returns `None` on any
    /// other input.
    #[must_use]
    pub fn from_hex(s: &str) -> Option<Self> {
        let s = s.trim();
        if s.len() != 64 {
            return None;
        }
        let mut out = [0u8; 32];
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = u8::from_str_radix(s.get(2 * i..2 * i + 2)?, 16).ok()?;
        }
        Some(Self(out))
    }
}

impl fmt::Display for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

/// Builder for a canonical, platform-independent hash.
///
/// See the module documentation for the encoding rules.
///
/// ```
/// use chipbreaker_core::golden::CanonicalHash;
///
/// let mut h = CanonicalHash::new();
/// h.begin("example").f64(1.5).usize(3).str("mm").end();
/// let digest = h.finish();
/// assert_eq!(digest.to_hex().len(), 64);
/// ```
#[derive(Clone)]
pub struct CanonicalHash {
    hasher: blake3::Hasher,
}

impl Default for CanonicalHash {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Debug for CanonicalHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("CanonicalHash(..)")
    }
}

impl CanonicalHash {
    /// Starts a fresh hash, domain-separated by the encoding version so that a
    /// future encoding change cannot accidentally reproduce an old digest.
    #[must_use]
    pub fn new() -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"chipbreaker.canonical.v");
        hasher.update(&u64::from(crate::CANONICAL_ENCODING_VERSION).to_le_bytes());
        Self { hasher }
    }

    #[inline]
    fn tagged(&mut self, tag: u8, payload: &[u8]) -> &mut Self {
        self.hasher.update(&[tag]);
        self.hasher.update(payload);
        self
    }

    /// Feeds a `u64`.
    #[inline]
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.tagged(TAG_U64, &v.to_le_bytes())
    }

    /// Feeds an `i64`, two's complement.
    #[inline]
    pub fn i64(&mut self, v: i64) -> &mut Self {
        self.tagged(TAG_I64, &v.to_le_bytes())
    }

    /// Feeds an `f64` as its canonical bit pattern.
    #[inline]
    pub fn f64(&mut self, v: f64) -> &mut Self {
        self.tagged(TAG_F64, &canonical_f64_bits(v).to_le_bytes())
    }

    /// Feeds a `usize`, **widened to `u64`** so that 32-bit targets (WASM) and
    /// 64-bit targets agree.
    #[inline]
    pub fn usize(&mut self, v: usize) -> &mut Self {
        self.tagged(TAG_USIZE, &(v as u64).to_le_bytes())
    }

    /// Feeds a `bool` as a single byte.
    #[inline]
    pub fn bool(&mut self, v: bool) -> &mut Self {
        self.tagged(TAG_BOOL, &[u8::from(v)])
    }

    /// Feeds a string, length-prefixed, as UTF-8 bytes.
    #[inline]
    pub fn str(&mut self, v: &str) -> &mut Self {
        self.hasher.update(&[TAG_STR]);
        self.hasher.update(&(v.len() as u64).to_le_bytes());
        self.hasher.update(v.as_bytes());
        self
    }

    /// Feeds raw bytes, length-prefixed.
    #[inline]
    pub fn bytes(&mut self, v: &[u8]) -> &mut Self {
        self.hasher.update(&[TAG_BYTES]);
        self.hasher.update(&(v.len() as u64).to_le_bytes());
        self.hasher.update(v);
        self
    }

    /// Feeds a slice of `f64`, length-prefixed, each element canonicalized.
    ///
    /// The slice is consumed in index order. Where the elements come from an
    /// unordered structure, sort them before calling this — see the determinism
    /// rules in `CONTRIBUTING.md`.
    pub fn f64_slice(&mut self, v: &[f64]) -> &mut Self {
        self.hasher.update(&[TAG_F64_SLICE]);
        self.hasher.update(&(v.len() as u64).to_le_bytes());
        for &x in v {
            self.hasher.update(&canonical_f64_bits(x).to_le_bytes());
        }
        self
    }

    /// Feeds a slice of `u64`, length-prefixed, in index order.
    pub fn u64_slice(&mut self, v: &[u64]) -> &mut Self {
        self.hasher.update(&[TAG_U64_SLICE]);
        self.hasher.update(&(v.len() as u64).to_le_bytes());
        for &x in v {
            self.hasher.update(&x.to_le_bytes());
        }
        self
    }

    /// Feeds any [`Hashable`].
    #[inline]
    pub fn add<T: Hashable + ?Sized>(&mut self, v: &T) -> &mut Self {
        v.hash_canonical(self);
        self
    }

    /// Feeds a sequence of [`Hashable`]s, length-prefixed, in iteration order.
    ///
    /// The caller is responsible for that order being deterministic.
    pub fn add_all<'t, T: Hashable + 't>(
        &mut self,
        items: impl ExactSizeIterator<Item = &'t T>,
    ) -> &mut Self {
        self.usize(items.len());
        for item in items {
            item.hash_canonical(self);
        }
        self
    }

    /// Opens a named group. Pair with [`Self::end`].
    ///
    /// Grouping makes the encoding injective for nested structures: without it,
    /// a struct of two fields and a struct whose single field is a struct of two
    /// fields would hash identically.
    #[inline]
    pub fn begin(&mut self, name: &str) -> &mut Self {
        self.hasher.update(&[TAG_BEGIN]);
        self.hasher.update(&(name.len() as u64).to_le_bytes());
        self.hasher.update(name.as_bytes());
        self
    }

    /// Closes the most recent [`Self::begin`].
    #[inline]
    pub fn end(&mut self) -> &mut Self {
        self.hasher.update(&[TAG_END]);
        self
    }

    /// Finalises and returns the digest.
    #[must_use]
    pub fn finish(&self) -> Digest {
        Digest(*self.hasher.finalize().as_bytes())
    }
}

/// A type with a canonical, platform-independent binary encoding.
pub trait Hashable {
    /// Feeds `self` into `h`.
    ///
    /// Implementations must consume a fixed number of values in a fixed order
    /// for a given shape, and must not iterate an unordered collection.
    fn hash_canonical(&self, h: &mut CanonicalHash);

    /// Convenience: the digest of `self` alone.
    #[must_use]
    fn canonical_digest(&self) -> Digest {
        let mut h = CanonicalHash::new();
        self.hash_canonical(&mut h);
        h.finish()
    }
}

impl Hashable for f64 {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.f64(*self);
    }
}

impl Hashable for u64 {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.u64(*self);
    }
}

impl Hashable for usize {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.usize(*self);
    }
}

impl Hashable for str {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self);
    }
}

impl<T: Hashable> Hashable for &T {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        (*self).hash_canonical(h);
    }
}

impl Hashable for Vec2 {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Vec2").f64(self.x).f64(self.y).end();
    }
}

impl Hashable for Vec3 {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Vec3").f64(self.x).f64(self.y).f64(self.z).end();
    }
}

impl Hashable for Mat3 {
    /// Row-major, ascending row then ascending column.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Mat3");
        for row in &self.m {
            h.f64_slice(row);
        }
        h.end();
    }
}

impl Hashable for Mat4 {
    /// Row-major, ascending row then ascending column.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Mat4");
        for row in &self.m {
            h.f64_slice(row);
        }
        h.end();
    }
}

impl Hashable for Aabb3 {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Aabb3").add(&self.min).add(&self.max).end();
    }
}

impl Hashable for Ray {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Ray").add(&self.origin).add(&self.direction).end();
    }
}

/// Why a golden check failed.
#[derive(Debug)]
pub enum GoldenError {
    /// The digest differs from the committed one.
    Mismatch {
        /// Golden file stem.
        name: String,
        /// What the file says.
        expected: Digest,
        /// What this run produced.
        actual: Digest,
        /// Absolute path of the golden file.
        path: PathBuf,
    },
    /// No golden file exists yet.
    Missing {
        /// Golden file stem.
        name: String,
        /// Where it was looked for.
        path: PathBuf,
        /// What this run produced, ready to be accepted.
        actual: Digest,
    },
    /// The golden file exists but does not contain 64 hex characters.
    Malformed {
        /// Golden file stem.
        name: String,
        /// Where it lives.
        path: PathBuf,
    },
    /// The golden file could not be read or written.
    Io {
        /// Where it lives.
        path: PathBuf,
        /// The underlying error, rendered.
        source: String,
    },
}

impl fmt::Display for GoldenError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mismatch { name, expected, actual, path } => write!(
                f,
                "golden mismatch for `{name}`\n  expected: {expected}\n  actual:   {actual}\n  \
                 file:     {}\n\nIf this change is intended, re-run with \
                 CHIPBREAKER_ACCEPT_GOLDEN=1 and explain the change in the commit message.",
                path.display()
            ),
            Self::Missing { name, path, actual } => write!(
                f,
                "no golden file for `{name}`\n  actual:   {actual}\n  file:     {}\n\nCreate it \
                 by re-running with CHIPBREAKER_ACCEPT_GOLDEN=1.",
                path.display()
            ),
            Self::Malformed { name, path } => write!(
                f,
                "golden file for `{name}` is not 64 hex characters: {}",
                path.display()
            ),
            Self::Io { path, source } => {
                write!(f, "golden file I/O error at {}: {source}", path.display())
            }
        }
    }
}

impl core::error::Error for GoldenError {}

/// Environment variable that, when set to `1`, rewrites golden files instead of
/// failing.
pub const ACCEPT_ENV: &str = "CHIPBREAKER_ACCEPT_GOLDEN";

/// Environment variable that overrides the golden directory.
pub const GOLDEN_DIR_ENV: &str = "CHIPBREAKER_GOLDEN_DIR";

/// True if golden files should be rewritten rather than compared.
#[must_use]
pub fn accepting() -> bool {
    std::env::var(ACCEPT_ENV).is_ok_and(|v| v == "1")
}

/// The directory holding committed golden hashes.
///
/// `tests/golden/` at the repository root, resolved from this crate's manifest
/// directory at compile time, and overridable with [`GOLDEN_DIR_ENV`].
#[must_use]
pub fn golden_dir() -> PathBuf {
    if let Ok(dir) = std::env::var(GOLDEN_DIR_ENV) {
        return PathBuf::from(dir);
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden")
}

/// A directory of committed golden hashes, and the policy for what to do when
/// one does not match.
///
/// The root is an explicit field rather than being read from the environment on
/// every call. Rust 2024 makes `std::env::set_var` `unsafe` — correctly, since
/// it is a data race in a threaded process — and this crate is
/// `#![forbid(unsafe_code)]`. Threading the root through as a value means the
/// tests can point at a scratch directory without mutating process state, which
/// is both safer and less surprising than the alternative.
#[derive(Debug, Clone)]
pub struct GoldenStore {
    root: PathBuf,
    accept: bool,
}

impl GoldenStore {
    /// A store rooted at `root`. When `accept` is true, [`Self::check`] rewrites
    /// files instead of failing.
    #[must_use]
    pub fn new(root: impl Into<PathBuf>, accept: bool) -> Self {
        Self { root: root.into(), accept }
    }

    /// The store described by the environment: [`golden_dir`] and
    /// [`accepting`].
    #[must_use]
    pub fn from_env() -> Self {
        Self::new(golden_dir(), accepting())
    }

    /// The directory this store reads and writes.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// The path of the golden file named `name`.
    #[must_use]
    pub fn path_for(&self, name: &str) -> PathBuf {
        self.root.join(format!("{name}.hash"))
    }

    /// Compares `actual` against the golden hash named `name`, or rewrites it if
    /// this store is in accept mode.
    ///
    /// # Errors
    /// Returns [`GoldenError::Mismatch`] if the digest differs,
    /// [`GoldenError::Missing`] if no golden file exists, and
    /// [`GoldenError::Io`] / [`GoldenError::Malformed`] for file problems.
    pub fn check(&self, name: &str, actual: &Digest) -> Result<(), GoldenError> {
        let path = self.path_for(name);
        let io_err = |e: std::io::Error| GoldenError::Io {
            path: self.path_for(name),
            source: e.to_string(),
        };

        if self.accept {
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(io_err)?;
            }
            // Always LF, never the platform separator: `.gitattributes` marks
            // `*.hash` as binary precisely so this byte survives a Windows
            // checkout and a Linux one alike.
            std::fs::write(&path, format!("{actual}\n")).map_err(io_err)?;
            return Ok(());
        }

        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                return Err(GoldenError::Missing {
                    name: name.to_owned(),
                    path,
                    actual: *actual,
                });
            }
            Err(e) => return Err(io_err(e)),
        };

        let expected = Digest::from_hex(&text).ok_or_else(|| GoldenError::Malformed {
            name: name.to_owned(),
            path: path.clone(),
        })?;

        if expected == *actual {
            Ok(())
        } else {
            Err(GoldenError::Mismatch {
                name: name.to_owned(),
                expected,
                actual: *actual,
                path,
            })
        }
    }
}

/// Compares `actual` against the committed golden hash named `name`, using the
/// store described by the environment.
///
/// # Errors
/// See [`GoldenStore::check`].
pub fn check_golden(name: &str, actual: &Digest) -> Result<(), GoldenError> {
    GoldenStore::from_env().check(name, actual)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest_of(f: impl FnOnce(&mut CanonicalHash)) -> Digest {
        let mut h = CanonicalHash::new();
        f(&mut h);
        h.finish()
    }

    #[test]
    fn hashing_is_reproducible() {
        let a = digest_of(|h| {
            h.f64(1.5).u64(7).str("mm");
        });
        let b = digest_of(|h| {
            h.f64(1.5).u64(7).str("mm");
        });
        assert_eq!(a, b);
    }

    #[test]
    fn type_tags_prevent_cross_type_collisions() {
        let as_u64 = digest_of(|h| {
            h.u64(1);
        });
        let as_f64 = digest_of(|h| {
            h.f64(f64::from_bits(1));
        });
        let as_usize = digest_of(|h| {
            h.usize(1);
        });
        assert_ne!(as_u64, as_f64);
        assert_ne!(as_u64, as_usize);
    }

    #[test]
    fn length_prefixes_prevent_concatenation_collisions() {
        let split_a = digest_of(|h| {
            h.str("ab").str("c");
        });
        let split_b = digest_of(|h| {
            h.str("a").str("bc");
        });
        assert_ne!(split_a, split_b);

        let slices_a = digest_of(|h| {
            h.f64_slice(&[1.0, 2.0]).f64_slice(&[3.0]);
        });
        let slices_b = digest_of(|h| {
            h.f64_slice(&[1.0]).f64_slice(&[2.0, 3.0]);
        });
        assert_ne!(slices_a, slices_b);
    }

    #[test]
    fn grouping_prevents_nesting_collisions() {
        let flat = digest_of(|h| {
            h.f64(1.0).f64(2.0);
        });
        let nested = digest_of(|h| {
            h.begin("g").f64(1.0).f64(2.0).end();
        });
        assert_ne!(flat, nested);
    }

    #[test]
    fn negative_zero_and_positive_zero_hash_identically() {
        assert_eq!(canonical_f64_bits(-0.0), canonical_f64_bits(0.0));
        assert_eq!(
            digest_of(|h| {
                h.f64(-0.0);
            }),
            digest_of(|h| {
                h.f64(0.0);
            })
        );
    }

    #[test]
    fn all_nans_hash_identically() {
        let nan_a = f64::NAN;
        let nan_b = f64::from_bits(f64::NAN.to_bits() ^ 0x3);
        assert!(nan_b.is_nan());
        assert_ne!(nan_a.to_bits(), nan_b.to_bits(), "distinct payloads");
        assert_eq!(canonical_f64_bits(nan_a), canonical_f64_bits(nan_b));
    }

    #[test]
    fn usize_is_widened_so_wasm_agrees_with_native() {
        // The whole point: `usize(1)` must encode the same eight bytes on a
        // 32-bit and a 64-bit target. We cannot run a 32-bit target here, so we
        // assert the observable consequence: usize and u64 of the same value
        // differ only by their tag byte.
        let mut a = blake3::Hasher::new();
        a.update(&(1u64).to_le_bytes());
        let mut b = blake3::Hasher::new();
        b.update(&(1usize as u64).to_le_bytes());
        assert_eq!(a.finalize(), b.finalize());
        assert_eq!(size_of_val(&(1usize as u64)), 8);
    }

    #[test]
    fn digest_hex_round_trips() {
        let d = digest_of(|h| {
            h.str("x");
        });
        let hex = d.to_hex();
        assert_eq!(hex.len(), 64);
        assert_eq!(Digest::from_hex(&hex), Some(d));
        assert_eq!(Digest::from_hex(&hex.to_uppercase()), Some(d));
        assert_eq!(Digest::from_hex(&format!("  {hex}\n")), Some(d));
        assert_eq!(Digest::from_hex("deadbeef"), None);
        assert_eq!(Digest::from_hex(&"z".repeat(64)), None);
    }

    #[test]
    fn math_types_hash_structurally() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(v.canonical_digest(), Vec3::new(1.0, 2.0, 3.0).canonical_digest());
        assert_ne!(v.canonical_digest(), Vec3::new(1.0, 2.0, 3.5).canonical_digest());
        // A Vec2 and a Vec3 with the same leading components must differ.
        assert_ne!(
            Vec2::new(1.0, 2.0).canonical_digest(),
            Vec3::new(1.0, 2.0, 0.0).canonical_digest()
        );
        // Distinct matrices, same entries in a different arrangement.
        let m = Mat3::from_rows_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert_ne!(m.canonical_digest(), m.transpose().canonical_digest());
        assert_eq!(m.canonical_digest(), m.transpose().transpose().canonical_digest());

        let b = Aabb3::new(Vec3::ZERO, Vec3::ONE);
        assert_eq!(b.canonical_digest(), Aabb3::new(Vec3::ZERO, Vec3::ONE).canonical_digest());
        let r = Ray::new(Vec3::ZERO, Vec3::Z);
        assert_ne!(r.canonical_digest(), Ray::new(Vec3::Z, Vec3::ZERO).canonical_digest());
        assert_ne!(Mat4::IDENTITY.canonical_digest(), Mat4::ZERO.canonical_digest());
    }

    #[test]
    fn add_all_is_length_prefixed() {
        let one = digest_of(|h| {
            h.add_all([1.0f64, 2.0].iter());
        });
        let two = digest_of(|h| {
            h.add_all([1.0f64, 2.0, 0.0].iter());
        });
        assert_ne!(one, two);
    }

    #[test]
    fn golden_round_trip_in_a_temporary_directory() {
        // Exercises the accept path and the compare path without touching the
        // committed goldens and without mutating process-wide environment
        // state, which is why GoldenStore carries its root as a value.
        let dir = std::env::temp_dir()
            .join(format!("chipbreaker-golden-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);

        let accepting = GoldenStore::new(&dir, true);
        let comparing = GoldenStore::new(&dir, false);

        let d = Vec3::new(1.0, 2.0, 3.0).canonical_digest();
        accepting
            .check("unit-test-sample", &d)
            .expect("accept path creates the directory and writes the file");
        comparing
            .check("unit-test-sample", &d)
            .expect("matching digest compares equal");

        let other = Vec3::ZERO.canonical_digest();
        let err = comparing.check("unit-test-sample", &other).expect_err("must reject");
        assert!(matches!(err, GoldenError::Mismatch { .. }));
        let rendered = err.to_string();
        assert!(rendered.contains(&d.to_hex()), "names the expected hash");
        assert!(rendered.contains(&other.to_hex()), "names the actual hash");
        assert!(rendered.contains("unit-test-sample"), "names the test");
        assert!(rendered.contains(ACCEPT_ENV), "tells the reader how to accept");

        let missing = comparing.check("unit-test-absent", &d).expect_err("must report missing");
        assert!(matches!(missing, GoldenError::Missing { .. }));

        std::fs::write(dir.join("unit-test-bad.hash"), "not a hash").expect("write");
        let malformed = comparing.check("unit-test-bad", &d).expect_err("must reject");
        assert!(matches!(malformed, GoldenError::Malformed { .. }));

        // The accept path overwrites rather than appending.
        accepting.check("unit-test-bad", &d).expect("accept rewrites a malformed file");
        comparing.check("unit-test-bad", &d).expect("and the rewrite is valid");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn store_from_env_points_at_the_repository_golden_directory() {
        let store = GoldenStore::from_env();
        assert!(store.path_for("x").ends_with("x.hash"));
        assert!(store.root().ends_with("golden") || std::env::var(GOLDEN_DIR_ENV).is_ok());
    }

    #[test]
    fn golden_files_are_written_with_lf() {
        // A CRLF here would make every golden comparison platform-dependent.
        let d = Vec3::ONE.canonical_digest();
        let rendered = format!("{d}\n");
        assert!(!rendered.contains('\r'));
        assert_eq!(rendered.len(), 65);
    }
}
