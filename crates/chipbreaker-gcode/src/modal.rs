// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Stage three: the state every block is interpreted against.
//!
//! # One struct, snapshotted, not a scattering of variables
//!
//! Modality is the architecture of RS-274, not a feature of it. A bare `X10.`
//! line inherits its motion mode, plane, units, distance mode, work offset, feed
//! mode, feed rate, tool, and canned cycle from whatever came before. Modelling
//! that as a dozen mutable variables threaded through a parser is how the
//! interactions between them get lost; modelling it as one struct that is
//! updated and then *read* makes every interpretation a pure function of an
//! explicit state.
//!
//! The state is also what makes an error message useful. "G2 with no plane" is
//! not a thing that can happen — a plane is always active — but "the arc on line
//! 412 was interpreted in G18 because line 3 selected it" is a sentence this
//! design can produce.

use chipbreaker_core::toolpath::{ArcPlane, FeedMode, WorkOffsetId};

/// `G90` or `G91`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DistanceMode {
    /// `G90`: coordinates are positions.
    #[default]
    Absolute,
    /// `G91`: coordinates are displacements from where the tool is.
    Incremental,
}

/// `G90.1` or `G91.1`: how `I`, `J` and `K` are read.
///
/// Fanuc's default is incremental — the offsets are measured from the arc's
/// start point. Some configurations make them absolute positions instead.
/// Getting this wrong produces arcs that are wildly wrong and parse perfectly,
/// which is why the resolved arc records which reading was used.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ArcCentreMode {
    /// `G91.1`: offsets from the start point. The default.
    #[default]
    Incremental,
    /// `G90.1`: absolute positions in the active work offset.
    Absolute,
}

/// `G20` or `G21`.
///
/// Can change mid-program, and the change affects feed rates and offsets as well
/// as coordinates — which is the part that is easy to miss.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Units {
    /// `G21`: millimetres.
    #[default]
    Millimetres,
    /// `G20`: inches.
    Inches,
}

impl Units {
    /// Millimetres per unit.
    #[must_use]
    pub const fn to_mm(self) -> f64 {
        match self {
            Self::Millimetres => 1.0,
            // Exact: 25.4 is representable, and the inch is *defined* as this.
            Self::Inches => 25.4,
        }
    }

    /// Name for reports.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Millimetres => "mm",
            Self::Inches => "inch",
        }
    }
}

/// `G98` or `G99`: where a canned cycle retracts to between holes.
///
/// The one that changes every intermediate retract in a multi-hole pattern, and
/// therefore whether the tool clears a clamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CycleReturn {
    /// `G98`: back to the Z the cycle started from.
    #[default]
    InitialZ,
    /// `G99`: back to the R plane only.
    RPlane,
}

/// `G61`, `G61.1` or `G64`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum PathControl {
    /// `G61`: exact stop at every corner.
    #[default]
    ExactStop,
    /// `G61.1`: exact path.
    ExactPath,
    /// `G64`: blended, optionally with a tolerance.
    ///
    /// The control may round corners by up to the tolerance, so the actual path
    /// differs from the commanded one. We simulate the commanded path and record
    /// this so a report can say by how much reality was allowed to differ.
    Blended {
        /// `P` from `G64 P…`, in millimetres, if given.
        tolerance: Option<f64>,
    },
}

/// The motion in force, including a canned cycle.
///
/// Cycles are motion modes rather than a separate concept because that is what
/// they are: once active, every subsequent block carrying axis words fires the
/// cycle again, until `G80`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MotionMode {
    /// `G0`.
    #[default]
    Rapid,
    /// `G1`.
    Linear,
    /// `G2`: clockwise looking down the plane's normal.
    ArcClockwise,
    /// `G3`: counter-clockwise.
    ArcCounterClockwise,
    /// A canned cycle, by code key: 730 for `G73`, 810 for `G81`, and so on.
    Cycle(u32),
    /// `G80`: no motion mode at all. A block with axis words and no motion mode
    /// is an error rather than a rapid.
    None,
}

impl MotionMode {
    /// True if this is a canned cycle.
    #[must_use]
    pub const fn is_cycle(self) -> bool {
        matches!(self, Self::Cycle(_))
    }
}

/// Parameters a canned cycle remembers between blocks.
///
/// They persist: once a cycle is active, a block carrying only `X` fires it
/// again at the new position with the same `R`, `Z` and `Q`. Storing them on the
/// modal state rather than re-reading them per block is what makes that work.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CycleParams {
    /// Bottom of the hole, in the active work offset, before unit conversion.
    pub z: f64,
    /// Retract plane.
    pub r: f64,
    /// Peck depth for `G73`/`G83`, or shift for `G76`/`G87`.
    pub q: Option<f64>,
    /// Dwell for `G82`, in seconds.
    pub p: Option<f64>,
    /// Where Z was when the cycle was first commanded, for `G98`.
    pub initial_z: f64,
}

/// Tool length compensation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ToolLength {
    /// `G49`: none active.
    #[default]
    None,
    /// `G43 H…`: the commanded position is the tool tip.
    Positive {
        /// The `H` number.
        h: u32,
    },
    /// `G44 H…`: negative compensation. Rare, and modelled the same way with
    /// the sign flipped where it matters.
    Negative {
        /// The `H` number.
        h: u32,
    },
}

impl ToolLength {
    /// The `H` number, if one is active.
    #[must_use]
    pub const fn h(self) -> Option<u32> {
        match self {
            Self::None => None,
            Self::Positive { h } | Self::Negative { h } => Some(h),
        }
    }
}

/// Everything modal, as of the start of a block.
#[derive(Debug, Clone, PartialEq)]
pub struct ModalState {
    /// Motion mode, including any active canned cycle.
    pub motion: MotionMode,
    /// Plane for arcs and cycles.
    pub plane: ArcPlane,
    /// Absolute or incremental.
    pub distance: DistanceMode,
    /// How `I`/`J`/`K` are read.
    pub arc_centre: ArcCentreMode,
    /// Feed rate mode.
    pub feed_mode: FeedMode,
    /// Units.
    pub units: Units,
    /// Canned cycle return level.
    pub cycle_return: CycleReturn,
    /// Path control mode.
    pub path_control: PathControl,
    /// Active work offset.
    pub work_offset: WorkOffsetId,
    /// Tool length compensation.
    pub tool_length: ToolLength,
    /// Feed rate as programmed, in the active units.
    ///
    /// `None` until an `F` word appears, which is why a feed move before any `F`
    /// is an error rather than a move at zero.
    pub feed: Option<f64>,
    /// Spindle speed, signed: negative for `M4`, zero for `M5`.
    pub spindle: Option<f64>,
    /// Tool number from the last `T`.
    pub tool: u32,
    /// Canned cycle parameters, when one is active.
    pub cycle: Option<CycleParams>,
}

impl Default for ModalState {
    /// The state a control powers up in.
    ///
    /// `G54` rather than "no offset": every real control has one active, and
    /// modelling the absence would mean a distinction that cannot arise.
    /// Millimetres rather than inches because the engine's internal unit is the
    /// millimetre and a file that never says is far more likely to be metric.
    fn default() -> Self {
        Self {
            motion: MotionMode::Rapid,
            plane: ArcPlane::Xy,
            distance: DistanceMode::Absolute,
            arc_centre: ArcCentreMode::Incremental,
            feed_mode: FeedMode::UnitsPerMinute,
            units: Units::Millimetres,
            cycle_return: CycleReturn::InitialZ,
            path_control: PathControl::ExactStop,
            work_offset: WorkOffsetId::from_gcode(54, 0).unwrap_or_else(|| unreachable!()),
            tool_length: ToolLength::None,
            feed: None,
            spindle: None,
            tool: 0,
            cycle: None,
        }
    }
}

impl ModalState {
    /// The feed rate in millimetres per minute, or the raw value for `G93`.
    ///
    /// Inverse time is not a distance rate, so converting it by the unit factor
    /// would be nonsense: `F4` under `G93` means "this block takes a quarter of
    /// a minute" in inches exactly as in millimetres.
    #[must_use]
    pub fn feed_mm(&self) -> Option<f64> {
        let value = self.feed?;
        Some(match self.feed_mode {
            FeedMode::InverseTime => value,
            FeedMode::UnitsPerMinute | FeedMode::UnitsPerRevolution => value * self.units.to_mm(),
        })
    }
}
