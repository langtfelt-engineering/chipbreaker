// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The versioned corpus of near-degenerate predicate configurations.
//!
//! # Why a data file rather than a table in Rust
//!
//! The corpus grows every unit and will eventually be shared with the WASM build
//! and (at U20) with the Python bindings. A text file with a stable, documented
//! format can be regenerated, diffed, and extended by someone who is debugging a
//! customer's model rather than writing Rust.
//!
//! # Why it is `include_str!`d rather than read from disk
//!
//! The CLI's `selftest` runs the corpus, and `selftest` must produce a
//! bit-identical `results` hash under `wasmtime`, where there is no filesystem
//! unless one is explicitly preopened. Embedding the corpus at compile time
//! removes an entire class of "works natively, mysteriously differs on WASM"
//! failure before it can happen.
//!
//! # Format
//!
//! One case per line. `#` starts a comment, blank lines are ignored.
//!
//! ```text
//! <case-id> <predicate> <expected> <coord> <coord> ...
//! ```
//!
//! - `case-id` — unique, no whitespace. Used in failure messages.
//! - `predicate` — `orient2d`, `orient3d`, `incircle`, or `insphere`.
//! - `expected` — `+`, `-`, or `0`.
//! - `coord` — either a decimal literal (`1.5`, `-1e300`) or an exact IEEE-754
//!   bit pattern written as `0x` followed by 16 hex digits.
//!
//! Decimal literals are safe here: Rust's decimal-to-`f64` conversion is
//! correctly rounded and platform-independent, so `0.1` denotes the same bits
//! everywhere. The `0x` form exists for coordinates that are a specific number
//! of ULPs from a target value, where a decimal literal would be unreadable and
//! its rounding would have to be trusted rather than stated.
//!
//! The expected result is stored rather than derived so that the CLI can run the
//! corpus without linking a bignum library. It is not taken on faith: the test
//! suite recomputes every expectation with exact rational arithmetic and fails
//! if the file disagrees.

use core::fmt;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Vec2, Vec3};
use crate::predicates::{Orientation, Predicates};

/// The corpus shipped with this crate, as raw text.
///
/// Lives at `tests/corpus/predicates/degenerate.txt` in the repository root, the
/// versioned home for test inputs across all units.
pub const DEGENERATE_CORPUS_SOURCE: &str =
    include_str!("../../../../tests/corpus/predicates/degenerate.txt");

/// Which predicate a corpus case exercises.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PredicateKind {
    /// [`crate::predicates::orient2d`]: 3 points, 6 coordinates.
    Orient2d,
    /// [`crate::predicates::orient3d`]: 4 points, 12 coordinates.
    Orient3d,
    /// [`crate::predicates::incircle`]: 4 points, 8 coordinates.
    InCircle,
    /// [`crate::predicates::insphere`]: 5 points, 15 coordinates.
    InSphere,
}

/// The largest coordinate count of any predicate ([`PredicateKind::InSphere`]).
pub const MAX_COORDS: usize = 15;

impl PredicateKind {
    /// All kinds, in the order used for grouping in reports.
    pub const ALL: [PredicateKind; 4] = [
        PredicateKind::Orient2d,
        PredicateKind::Orient3d,
        PredicateKind::InCircle,
        PredicateKind::InSphere,
    ];

    /// The number of coordinates a case of this kind carries.
    #[inline]
    #[must_use]
    pub const fn arity(self) -> usize {
        match self {
            Self::Orient2d => 6,
            Self::Orient3d => 12,
            Self::InCircle => 8,
            Self::InSphere => 15,
        }
    }

    /// The coordinate magnitudes for which this predicate is exact.
    ///
    /// Narrows as the determinant's degree rises; see
    /// [`crate::predicates::ORIENT2D_COORDS`].
    #[inline]
    #[must_use]
    pub const fn coord_range(self) -> crate::predicates::CoordRange {
        match self {
            Self::Orient2d => crate::predicates::ORIENT2D_COORDS,
            Self::Orient3d => crate::predicates::ORIENT3D_COORDS,
            Self::InCircle => crate::predicates::INCIRCLE_COORDS,
            Self::InSphere => crate::predicates::INSPHERE_COORDS,
        }
    }

    /// The lower-case name used in the corpus file.
    #[inline]
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Orient2d => "orient2d",
            Self::Orient3d => "orient3d",
            Self::InCircle => "incircle",
            Self::InSphere => "insphere",
        }
    }

    /// Parses the corpus-file name.
    #[must_use]
    pub fn from_name(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.name() == s)
    }
}

impl fmt::Display for PredicateKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl Hashable for PredicateKind {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.name());
    }
}

/// A single corpus case, borrowing its identifier from the source text.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorpusCase<'a> {
    /// Unique identifier, used in failure messages.
    pub id: &'a str,
    /// Which predicate to run.
    pub kind: PredicateKind,
    /// The mathematically correct result, verified against exact rational
    /// arithmetic by the test suite.
    pub expected: Orientation,
    coords: [f64; MAX_COORDS],
}

impl<'a> CorpusCase<'a> {
    /// The coordinates, exactly `kind.arity()` of them.
    #[inline]
    #[must_use]
    pub fn coords(&self) -> &[f64] {
        &self.coords[..self.kind.arity()]
    }

    fn v2(&self, i: usize) -> Vec2 {
        Vec2::new(self.coords[2 * i], self.coords[2 * i + 1])
    }

    fn v3(&self, i: usize) -> Vec3 {
        Vec3::new(
            self.coords[3 * i],
            self.coords[3 * i + 1],
            self.coords[3 * i + 2],
        )
    }

    /// Runs the case against a predicate implementation.
    #[must_use]
    pub fn evaluate<P: Predicates + ?Sized>(&self, p: &P) -> Orientation {
        match self.kind {
            PredicateKind::Orient2d => p.orient2d(self.v2(0), self.v2(1), self.v2(2)),
            PredicateKind::Orient3d => p.orient3d(self.v3(0), self.v3(1), self.v3(2), self.v3(3)),
            PredicateKind::InCircle => p.incircle(self.v2(0), self.v2(1), self.v2(2), self.v2(3)),
            PredicateKind::InSphere => {
                p.insphere(self.v3(0), self.v3(1), self.v3(2), self.v3(3), self.v3(4))
            }
        }
    }

    /// Renders the case as a canonical corpus line, with every coordinate as an
    /// exact bit pattern.
    ///
    /// Used by the corpus regenerator; hand-maintained lines keep their
    /// human-readable decimal form.
    #[must_use]
    pub fn to_canonical_line(&self) -> String {
        let mut s = format!(
            "{} {} {}",
            self.id,
            self.kind.name(),
            self.expected.as_char()
        );
        for &c in self.coords() {
            s.push_str(&format!(" 0x{:016x}", c.to_bits()));
        }
        s
    }
}

impl Hashable for CorpusCase<'_> {
    /// Hashes identity, inputs, and expectation — everything that defines the
    /// case. A silent edit to a coordinate therefore changes the golden hash.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.id);
        self.kind.hash_canonical(h);
        self.expected.hash_canonical(h);
        h.f64_slice(self.coords());
    }
}

/// A malformed corpus line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CorpusError {
    /// One-based line number in the source text.
    pub line: usize,
    /// What was wrong.
    pub message: String,
}

impl fmt::Display for CorpusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "corpus line {}: {}", self.line, self.message)
    }
}

impl core::error::Error for CorpusError {}

/// Parses one coordinate token: either a decimal literal or `0x` + 16 hex
/// digits.
fn parse_coord(tok: &str) -> Option<f64> {
    if let Some(hex) = tok.strip_prefix("0x").or_else(|| tok.strip_prefix("0X")) {
        if hex.len() != 16 {
            return None;
        }
        return u64::from_str_radix(hex, 16).ok().map(f64::from_bits);
    }
    tok.parse::<f64>().ok()
}

/// Parses corpus text into cases.
///
/// # Errors
/// Returns the first malformed line, with its one-based line number.
pub fn parse(src: &str) -> Result<Vec<CorpusCase<'_>>, CorpusError> {
    let mut out = Vec::new();
    for (idx, raw) in src.lines().enumerate() {
        let line = idx + 1;
        let text = raw.split('#').next().unwrap_or("").trim();
        if text.is_empty() {
            continue;
        }
        let err = |message: String| CorpusError { line, message };

        let mut tokens = text.split_whitespace();
        let id = tokens.next().ok_or_else(|| err("missing case id".into()))?;
        let kind_tok = tokens
            .next()
            .ok_or_else(|| err("missing predicate name".into()))?;
        let kind = PredicateKind::from_name(kind_tok)
            .ok_or_else(|| err(format!("unknown predicate `{kind_tok}`")))?;
        let expected_tok = tokens
            .next()
            .ok_or_else(|| err("missing expected result".into()))?;
        let expected = expected_tok
            .chars()
            .next()
            .filter(|_| expected_tok.chars().count() == 1)
            .and_then(Orientation::from_char)
            .ok_or_else(|| err(format!("expected `+`, `-` or `0`, found `{expected_tok}`")))?;

        let mut coords = [0.0f64; MAX_COORDS];
        let arity = kind.arity();
        for (i, slot) in coords.iter_mut().enumerate().take(arity) {
            let tok = tokens
                .next()
                .ok_or_else(|| err(format!("{kind} needs {arity} coordinates, found {i}")))?;
            *slot = parse_coord(tok)
                .ok_or_else(|| err(format!("coordinate {i} is not a number: `{tok}`")))?;
        }
        if let Some(extra) = tokens.next() {
            return Err(err(format!(
                "{kind} takes {arity} coordinates; trailing token `{extra}`"
            )));
        }
        out.push(CorpusCase {
            id,
            kind,
            expected,
            coords,
        });
    }
    Ok(out)
}

/// The embedded degenerate corpus, parsed.
///
/// # Panics
/// Panics if the embedded corpus is malformed. That is a compile-time-constant
/// input, so a failure here is a broken build, not a runtime condition.
#[must_use]
pub fn degenerate_corpus() -> Vec<CorpusCase<'static>> {
    match parse(DEGENERATE_CORPUS_SOURCE) {
        Ok(cases) => cases,
        Err(e) => panic!("embedded predicate corpus is malformed: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicates::ADAPTIVE;

    #[test]
    fn embedded_corpus_parses_and_is_substantial() {
        let cases = degenerate_corpus();
        assert!(
            cases.len() >= 50,
            "the specification calls for ~50 hand-written cases, found {}",
            cases.len()
        );
        // Every predicate must be represented.
        for kind in PredicateKind::ALL {
            assert!(
                cases.iter().any(|c| c.kind == kind),
                "corpus has no {kind} cases"
            );
        }
    }

    #[test]
    fn corpus_ids_are_unique() {
        let cases = degenerate_corpus();
        let mut ids: Vec<&str> = cases.iter().map(|c| c.id).collect();
        ids.sort_unstable();
        let before = ids.len();
        ids.dedup();
        assert_eq!(before, ids.len(), "duplicate case ids in corpus");
    }

    #[test]
    fn corpus_covers_the_degenerate_case() {
        let cases = degenerate_corpus();
        let zeros = cases.iter().filter(|c| c.expected.is_zero()).count();
        assert!(
            zeros >= 10,
            "only {zeros} exactly-degenerate cases; that is the whole point"
        );
    }

    #[test]
    fn every_corpus_case_is_inside_its_predicate_exact_range() {
        // A case outside the range would be testing the predicate where it is
        // documented not to work, and would fail for a reason that has nothing
        // to do with adaptivity.
        for case in degenerate_corpus() {
            let range = case.kind.coord_range();
            assert!(
                range.contains_all(case.coords()),
                "case `{}` has a coordinate outside the exact range for {} \
                 [{:e}, {:e}]: {:?}",
                case.id,
                case.kind,
                range.min,
                range.max,
                case.coords()
            );
        }
    }

    #[test]
    fn corpus_reaches_the_extremes_of_the_exact_range() {
        // The corpus is meant to press against the limits, not sit safely in the
        // middle of them.
        let cases = degenerate_corpus();
        for kind in PredicateKind::ALL {
            let range = kind.coord_range();
            let magnitudes: Vec<f64> = cases
                .iter()
                .filter(|c| c.kind == kind)
                .flat_map(|c| c.coords().to_vec())
                .filter(|v| *v != 0.0)
                .map(f64::abs)
                .collect();
            let largest = magnitudes.iter().copied().fold(0.0f64, f64::max);
            let smallest = magnitudes.iter().copied().fold(f64::INFINITY, f64::min);
            assert!(
                largest >= range.max / 1e10,
                "{kind} corpus tops out at {largest:e}, far short of {:e}",
                range.max
            );
            assert!(
                smallest <= range.min * 1e10,
                "{kind} corpus bottoms out at {smallest:e}, far short of {:e}",
                range.min
            );
        }
    }

    #[test]
    fn adaptive_predicates_match_every_stored_expectation() {
        for case in degenerate_corpus() {
            assert_eq!(
                case.evaluate(&ADAPTIVE),
                case.expected,
                "case `{}` ({})",
                case.id,
                case.kind
            );
        }
    }

    #[test]
    fn hex_and_decimal_coordinates_round_trip() {
        assert_eq!(parse_coord("1.5"), Some(1.5));
        assert_eq!(parse_coord("-1e300"), Some(-1e300));
        assert_eq!(parse_coord("0x3ff0000000000000"), Some(1.0));
        assert_eq!(
            parse_coord(&format!("0x{:016x}", (0.1f64).to_bits())),
            Some(0.1)
        );
        // Rejected: wrong digit count, not a number.
        assert_eq!(parse_coord("0x1"), None);
        assert_eq!(parse_coord("banana"), None);
    }

    #[test]
    fn parse_rejects_malformed_lines() {
        assert_eq!(parse("a orient2d + 0 0 1 0 0 1").map(|v| v.len()), Ok(1));
        // Comments and blank lines.
        assert_eq!(
            parse("# nothing\n\n   \na orient2d + 0 0 1 0 0 1 # trailing").map(|v| v.len()),
            Ok(1)
        );
        assert_eq!(
            parse("a nosuchpred + 0")
                .expect_err("unknown predicate")
                .line,
            1
        );
        assert!(parse("a orient2d ? 0 0 1 0 0 1").is_err());
        assert!(
            parse("a orient2d + 0 0 1 0 0").is_err(),
            "too few coordinates"
        );
        assert!(
            parse("a orient2d + 0 0 1 0 0 1 9").is_err(),
            "too many coordinates"
        );
        assert_eq!(
            parse("ok orient2d + 0 0 1 0 0 1\nbad orient2d +")
                .expect_err("second line is short")
                .line,
            2
        );
    }

    #[test]
    fn canonical_line_round_trips() {
        for case in degenerate_corpus() {
            let line = case.to_canonical_line();
            let reparsed = parse(&line).expect("canonical line must reparse");
            assert_eq!(reparsed.len(), 1);
            assert_eq!(reparsed[0].kind, case.kind);
            assert_eq!(reparsed[0].expected, case.expected);
            assert_eq!(reparsed[0].coords(), case.coords(), "case `{}`", case.id);
        }
    }
}
