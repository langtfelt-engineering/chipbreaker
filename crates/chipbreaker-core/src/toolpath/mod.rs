// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The canonical toolpath IR: a flat, ordered, fully resolved motion stream.
//!
//! **Nothing in Chipbreaker reads G-code text after this point.** A dexel field
//! is built from this type, the sweep moves the tool along these segments, and
//! everything downstream
//! consume and extend it. Getting its shape right matters more than getting any
//! parser exhaustive, because the parser can be improved later and this cannot:
//! every unit downstream is written against it.
//!
//! # Coordinates are machine coordinates
//!
//! Not workpiece coordinates. See [ADR 0003][adr] for the argument in full; the
//! short version is that "workpiece coordinates" does not name a frame when a
//! program uses `G54` and `G55`, and that a program which switches offsets
//! without moving would otherwise have a segment whose `start` differs from the
//! previous `end` while the tool has not moved at all.
//!
//! In the machine frame, **`start == previous.end` holds exactly, always, with
//! no tolerance and no exceptions.** Downstream code may assert it. Every work
//! offset is in [`ToolpathHeader::offsets`] and every activation is a
//! [`PathEvent`], so a consumer that wants a workpiece frame applies one
//! transform.
//!
//! [adr]: https://github.com/spanwerk/chipbreaker/blob/main/docs/adr/0003-toolpath-ir-coordinate-frame.md
//!
//! # What is deliberately reserved
//!
//! [`MotionSegment::orientation`] exists and is always `None`. 5-axis work
//! adds 5-axis, and an unused `Option` today is very much cheaper than a schema
//! migration across ten units later. Populating it will move golden hashes, and
//! that will be deliberate.

pub mod arc;
pub mod event;
pub mod feed;
pub mod segment;

pub use arc::{ArcData, ArcForm, ArcPlane};
pub use event::{PathEvent, PathEventKind};
pub use feed::{FeedMode, FeedSpec};
pub use segment::{MotionKind, MotionSegment, NOT_A_CYCLE_STEP, Orient, Provenance};

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Vec3};

use std::collections::BTreeMap;

/// Version of the toolpath IR schema.
///
/// Frozen. A change to the meaning of any field, or the
/// addition of a required one, bumps this — and moves every golden hash that
/// covers a toolpath, which is the point.
pub const TOOLPATH_SCHEMA_VERSION: u32 = 1;

/// How `G0` is represented geometrically.
///
/// Real controls move each axis of a rapid at its own maximum rate, so the path
/// is a dogleg polyline rather than a straight line. Which one is correct
/// depends on what the answer is for: a straight line is what the programmer
/// drew and what every CAM preview shows, and a dogleg is what the machine
/// actually does.
///
/// The choice is recorded in the header rather than assumed, because a collision
/// report is only as trustworthy as the path it was computed against.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum RapidPath {
    /// One straight segment from start to end. The default.
    #[default]
    Linear,
    /// Each axis at its own rate, giving a polyline. The conservative choice for
    /// collision checking, and the one that needs per-axis rates the NC file
    /// does not contain.
    Dogleg,
}

impl RapidPath {
    /// Name used in reports and in the IR header.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Linear => "linear",
            Self::Dogleg => "dogleg",
        }
    }
}

impl Hashable for RapidPath {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// One of the standard work offsets.
///
/// A newtype over the index rather than a bare integer, because `G54` is offset
/// 1 and not offset 54, and that off-by-53 is a mistake worth making impossible.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WorkOffsetId(u32);

impl WorkOffsetId {
    /// `G54` through `G59` map to 1..=6; `G59.1` through `G59.3` to 7..=9.
    ///
    /// # Errors
    /// Returns `None` for a code outside the standard set.
    #[must_use]
    pub const fn from_gcode(major: u32, minor: u32) -> Option<Self> {
        match (major, minor) {
            (54..=59, 0) => Some(Self(major - 53)),
            (59, 1..=3) => Some(Self(6 + minor)),
            _ => None,
        }
    }

    /// The raw index, 1-based.
    #[inline]
    #[must_use]
    pub const fn index(self) -> u32 {
        self.0
    }

    /// The G-code that selects this offset, as written.
    #[must_use]
    pub fn as_gcode(self) -> String {
        if self.0 <= 6 {
            format!("G{}", self.0 + 53)
        } else {
            format!("G59.{}", self.0 - 6)
        }
    }
}

impl Hashable for WorkOffsetId {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.u64(u64::from(self.0));
    }
}

/// One value of a work offset, and the segment from which it applied.
///
/// Programs that never touch `G10` have exactly one epoch per offset, starting
/// at segment zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffsetEpoch {
    /// Translation from machine origin to workpiece origin, in millimetres.
    pub value: Vec3,
    /// Index of the first segment for which this value was in force.
    pub from_segment: u32,
}

impl Hashable for OffsetEpoch {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("OffsetEpoch");
        h.f64_slice(&self.value.to_array());
        h.u64(u64::from(self.from_segment));
        h.end();
    }
}

/// Everything needed to interpret the segments that follow.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolpathHeader {
    /// Schema version; see [`TOOLPATH_SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Name of the program this came from, for reports.
    pub program: String,
    /// Work offsets seen anywhere in the program, **versioned**.
    ///
    /// Versioned because `G10 L2` lets a program rewrite an offset partway
    /// through. The segments themselves are unaffected — they are in machine
    /// coordinates, so earlier geometry stays correct — but a report rendering
    /// into a workpiece frame must use the value that was in force *then*, not
    /// the value the program finished with. A flat map would place every early
    /// move wrongly and look entirely reasonable doing it.
    ///
    /// A `BTreeMap` rather than a `HashMap`: this is iterated when hashing, and
    /// an unordered iteration reaching a float is exactly what the determinism
    /// rules forbid.
    pub offsets: BTreeMap<WorkOffsetId, Vec<OffsetEpoch>>,
    /// How rapids were represented.
    pub rapid_path: RapidPath,
    /// Arc radius mismatch tolerance actually used, in millimetres.
    pub arc_tolerance: f64,
    /// Path-control tolerance from `G64 P…`, if the program set one.
    ///
    /// Recorded rather than applied: we simulate the commanded path, and this
    /// says how far the machine was permitted to depart from it.
    pub path_tolerance: Option<f64>,
    /// Whether blocks marked with a leading `/` were executed.
    pub block_skip_executed: bool,
    /// Canned cycles expanded with motion the machine makes but this IR omits.
    ///
    /// Counts `G73` firings expanded without a chip-break clearance. A real
    /// control retracts a short distance between pecks, by a machine parameter
    /// that is not in the NC file; without it the IR contains a straight plunge
    /// where the machine oscillates.
    ///
    /// **This is structural, not a warning, and that is deliberate.** A
    /// diagnostic in a list is too easy for a downstream unit to ignore, and a
    /// collision check that reports "no collisions found" against a path missing
    /// motion the machine makes is the failure mode this project keeps declining
    /// to accept. Nothing may certify a program as collision-clean while
    /// this is non-zero.
    ///
    /// Supplying `--chip-break-clearance` emits the real motion and leaves this
    /// at zero.
    pub unmodelled_retracts: u32,
}

impl Hashable for ToolpathHeader {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("ToolpathHeader");
        h.u64(u64::from(self.schema_version));
        h.str(&self.program);
        h.usize(self.offsets.len());
        for (id, epochs) in &self.offsets {
            h.add(id);
            h.usize(epochs.len());
            for epoch in epochs {
                h.add(epoch);
            }
        }
        h.add(&self.rapid_path);
        h.f64(self.arc_tolerance);
        match self.path_tolerance {
            Some(t) => {
                h.bool(true);
                h.f64(t);
            }
            None => {
                h.bool(false);
            }
        }
        h.bool(self.block_skip_executed);
        h.u64(u64::from(self.unmodelled_retracts));
        h.end();
    }
}

/// A parsed program: header, motion, and the events between.
#[derive(Debug, Clone, PartialEq)]
pub struct Toolpath {
    /// Units, policies, offsets, schema version.
    pub header: ToolpathHeader,
    /// Flat and ordered. No nesting: subprograms and canned cycles are already
    /// expanded, because a consumer that has to interpret structure is a
    /// consumer that can interpret it differently from the next one.
    pub segments: Vec<MotionSegment>,
    /// Tool changes, stops, offset changes. Each names the segment index it
    /// precedes.
    pub events: Vec<PathEvent>,
}

/// Why a toolpath was rejected at construction.
#[derive(Debug, Clone, PartialEq)]
pub enum ToolpathError {
    /// Two consecutive segments do not join.
    ///
    /// In machine coordinates this cannot happen for any legitimate reason; see
    /// the module header.
    Discontinuous {
        /// Index of the segment that begins in the wrong place.
        index: u32,
        /// Where the previous segment ended.
        expected: Vec3,
        /// Where this one begins.
        found: Vec3,
    },
    /// A coordinate that is not finite reached the IR.
    ///
    /// Rejected at the boundary because `Orientation::from_determinant` panics
    /// on NaN in release builds by design, so a NaN that reaches the predicates
    /// aborts the process.
    NotFinite {
        /// Index of the offending segment.
        index: u32,
    },
    /// An event names a segment that does not exist.
    EventOutOfRange {
        /// Index of the offending event.
        index: u32,
        /// The segment it named.
        segment: u32,
    },
    /// Events are not in segment order.
    EventsOutOfOrder {
        /// Index of the offending event.
        index: u32,
    },
}

impl core::fmt::Display for ToolpathError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Discontinuous {
                index,
                expected,
                found,
            } => write!(
                f,
                "segment {index} begins at {:?} but segment {} ended at {:?}; \
                 in machine coordinates the toolpath is continuous by construction",
                found.to_array(),
                index.saturating_sub(1),
                expected.to_array()
            ),
            Self::NotFinite { index } => {
                write!(f, "segment {index} has a non-finite coordinate")
            }
            Self::EventOutOfRange { index, segment } => write!(
                f,
                "event {index} names segment {segment}, which does not exist"
            ),
            Self::EventsOutOfOrder { index } => {
                write!(f, "event {index} precedes the event before it")
            }
        }
    }
}

impl core::error::Error for ToolpathError {}

impl Toolpath {
    /// Builds a toolpath, checking the invariants downstream units rely on.
    ///
    /// # Errors
    ///
    /// See [`ToolpathError`]. Contiguity is checked with `==` and not with a
    /// tolerance: in machine coordinates there is no legitimate way for it to
    /// fail, so an approximate check would only hide a bug.
    pub fn new(
        header: ToolpathHeader,
        segments: Vec<MotionSegment>,
        events: Vec<PathEvent>,
    ) -> Result<Self, ToolpathError> {
        for (i, segment) in segments.iter().enumerate() {
            let index = u32::try_from(i).unwrap_or(u32::MAX);
            if !segment.start.is_finite() || !segment.end.is_finite() {
                return Err(ToolpathError::NotFinite { index });
            }
            if let Some(arc) = &segment.arc
                && !arc.is_finite()
            {
                return Err(ToolpathError::NotFinite { index });
            }
            if i > 0 {
                let previous = segments[i - 1].end;
                if segment.start != previous {
                    return Err(ToolpathError::Discontinuous {
                        index,
                        expected: previous,
                        found: segment.start,
                    });
                }
            }
        }

        let mut last_segment = 0u32;
        for (i, event) in events.iter().enumerate() {
            let index = u32::try_from(i).unwrap_or(u32::MAX);
            if event.at_segment as usize > segments.len() {
                return Err(ToolpathError::EventOutOfRange {
                    index,
                    segment: event.at_segment,
                });
            }
            if event.at_segment < last_segment {
                return Err(ToolpathError::EventsOutOfOrder { index });
            }
            last_segment = event.at_segment;
        }

        Ok(Self {
            header,
            segments,
            events,
        })
    }

    /// Total commanded distance, summed in segment order.
    ///
    /// The order is part of the contract: floating-point addition is not
    /// associative, so "the obvious order" has to be the written-down one.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.segments.iter().map(MotionSegment::length).sum()
    }

    /// Bounding box of every point the tool tip reaches.
    ///
    /// This is what a dexel field is sized from, which is why `path bounds`
    /// exists as a command in its own right. Note that it bounds the *tip*: the
    /// tool's body extends beyond it, so a field expands by the tool radius.
    #[must_use]
    pub fn tip_bounds(&self) -> Aabb3 {
        let mut bounds = Aabb3::EMPTY;
        for segment in &self.segments {
            bounds = bounds.union(&segment.bounds());
        }
        bounds
    }

    /// Segment count, as a `u32` because it is hashed and serialized.
    #[must_use]
    pub fn segment_count(&self) -> u32 {
        u32::try_from(self.segments.len()).unwrap_or(u32::MAX)
    }

    /// The tools used, in ascending order.
    #[must_use]
    pub fn tools_used(&self) -> Vec<u32> {
        let mut seen: Vec<u32> = self.segments.iter().map(|s| s.tool).collect();
        seen.sort_unstable();
        seen.dedup();
        seen
    }
}

impl Hashable for Toolpath {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Toolpath");
        h.add(&self.header);
        h.usize(self.segments.len());
        for segment in &self.segments {
            h.add(segment);
        }
        h.usize(self.events.len());
        for event in &self.events {
            h.add(event);
        }
        h.end();
    }
}
