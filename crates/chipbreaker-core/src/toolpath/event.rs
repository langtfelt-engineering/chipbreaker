// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Things that happen between moves.
//!
//! An event is anything that changes what a subsequent segment *means* without
//! itself being motion: a tool change, a program stop, a work offset selection,
//! a spindle command.
//!
//! # Why these are not fields on the segment
//!
//! A tool number is on the segment, because every segment has one and a consumer
//! always needs it. A tool *change* is an event, because it happens once and
//! between two segments, and putting "did a tool change happen just before me"
//! on every segment would be a field that is false a million times and true
//! eight.
//!
//! # A work offset change is a label, not a discontinuity
//!
//! Worth stating because it is the thing most likely to be misread. Segments are
//! in machine coordinates, so a `G54` to `G55` change moves nothing and breaks
//! nothing: the geometry is continuous across it. The event records *which*
//! workpiece frame was in force, for reports and for stock placement. See
//! ADR 0003.

use crate::golden::{CanonicalHash, Hashable};

use super::WorkOffsetId;

/// What happened.
#[derive(Debug, Clone, PartialEq)]
pub enum PathEventKind {
    /// `M6`: the tool was changed to this number.
    ToolChange {
        /// The tool now in the spindle.
        tool: u32,
    },
    /// `M3`/`M4`/`M5`: spindle started, reversed, or stopped.
    Spindle {
        /// Signed speed in rev/min; negative is `M4`, zero is `M5`.
        rpm: f64,
    },
    /// `G54`–`G59.3`: a different work offset became active.
    ///
    /// A label. The path does not jump; see the module header.
    WorkOffsetChanged {
        /// The offset now in force.
        to: WorkOffsetId,
    },
    /// `G10 L2`/`L20`: the program rewrote a work offset in the machine's table.
    ///
    /// Recorded separately from [`Self::WorkOffsetChanged`] because it mutates
    /// the meaning of an offset that earlier segments may already have used, and
    /// a reader reconstructing frames needs to know it happened rather than
    /// seeing only the final value.
    WorkOffsetRedefined {
        /// Which offset was rewritten.
        offset: WorkOffsetId,
    },
    /// `G92`: a coordinate shift was set, or cleared by `G92.1`/`G92.2`.
    CoordinateShift {
        /// True if a shift became active, false if one was cleared.
        active: bool,
    },
    /// `G43`/`G44`/`G49`: tool length compensation changed.
    ToolLengthOffset {
        /// The `H` number applied, or `None` for `G49`.
        h: Option<u32>,
    },
    /// `M0`: unconditional program stop, awaiting the operator.
    Stop,
    /// `M1`: optional stop, taken only if the machine's switch is set.
    OptionalStop,
    /// `M2`/`M30`: end of program.
    ProgramEnd,
    /// `G4 P…`: a dwell.
    Dwell {
        /// Duration in seconds.
        seconds: f64,
    },
    /// An `M` code we recognise as valid but model no behaviour for.
    ///
    /// Coolant, for instance. Recorded rather than dropped so that a report can
    /// say the program used it, and so a future unit that cares can find them
    /// without re-parsing.
    UnmodelledMCode {
        /// The code number.
        code: u32,
    },
}

impl PathEventKind {
    /// Short name used in reports and in the IR.
    #[must_use]
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::ToolChange { .. } => "tool-change",
            Self::Spindle { .. } => "spindle",
            Self::WorkOffsetChanged { .. } => "work-offset-changed",
            Self::WorkOffsetRedefined { .. } => "work-offset-redefined",
            Self::CoordinateShift { .. } => "coordinate-shift",
            Self::ToolLengthOffset { .. } => "tool-length-offset",
            Self::Stop => "stop",
            Self::OptionalStop => "optional-stop",
            Self::ProgramEnd => "program-end",
            Self::Dwell { .. } => "dwell",
            Self::UnmodelledMCode { .. } => "unmodelled-m-code",
        }
    }
}

impl Hashable for PathEventKind {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("PathEventKind");
        h.str(self.as_str());
        match self {
            Self::ToolChange { tool } => h.u64(u64::from(*tool)),
            Self::Spindle { rpm } => h.f64(*rpm),
            Self::WorkOffsetChanged { to } => h.add(to),
            Self::WorkOffsetRedefined { offset } => h.add(offset),
            Self::CoordinateShift { active } => h.bool(*active),
            Self::ToolLengthOffset { h: number } => match number {
                Some(n) => h.bool(true).u64(u64::from(*n)),
                None => h.bool(false),
            },
            Self::Dwell { seconds } => h.f64(*seconds),
            Self::UnmodelledMCode { code } => h.u64(u64::from(*code)),
            Self::Stop | Self::OptionalStop | Self::ProgramEnd => h,
        };
        h.end();
    }
}

/// An event, and where in the motion stream it sits.
#[derive(Debug, Clone, PartialEq)]
pub struct PathEvent {
    /// Index of the segment this event *precedes*.
    ///
    /// Equal to `segments.len()` for an event after the last move, which is
    /// where `M30` lives. Events are stored in non-decreasing order of this
    /// field.
    pub at_segment: u32,
    /// What happened.
    pub kind: PathEventKind,
    /// Which line of which file said so.
    pub source: super::Provenance,
}

impl Hashable for PathEvent {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("PathEvent");
        h.u64(u64::from(self.at_segment));
        h.add(&self.kind);
        h.add(&self.source);
        h.end();
    }
}
