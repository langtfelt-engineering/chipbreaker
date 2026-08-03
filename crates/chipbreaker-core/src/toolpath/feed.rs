// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Feed rate, and the mode that says what the number means.

use crate::golden::{CanonicalHash, Hashable};

/// What an `F` word means.
///
/// The three modes are not three units for one quantity — they are three
/// different quantities, and treating the mode as decoration is how a
/// simulation ends up with segment timings that are wrong by orders of
/// magnitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum FeedMode {
    /// `G94`: distance per minute. The ordinary case.
    #[default]
    UnitsPerMinute,
    /// `G95`: distance per spindle revolution. Turning, and drilling with a
    /// rigid-tapping-style feed. Converting to a time needs the spindle speed,
    /// which is why [`FeedSpec`] keeps them together.
    UnitsPerRevolution,
    /// `G93`: **inverse time**. `F` is the reciprocal of the number of minutes
    /// the block should take, so the same `F` means different speeds on
    /// different-length moves and the value is meaningless without its segment.
    ///
    /// Parsed in Unit 4 rather than at U16 because it is the norm in 5-axis
    /// output, and discovering at U16 that every segment's timing needs
    /// rethinking would be worse than carrying the mode now.
    InverseTime,
}

impl FeedMode {
    /// Name used in reports and in the IR.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::UnitsPerMinute => "units-per-minute",
            Self::UnitsPerRevolution => "units-per-revolution",
            Self::InverseTime => "inverse-time",
        }
    }

    /// The G-code that selects this mode.
    #[must_use]
    pub const fn as_gcode(self) -> &'static str {
        match self {
            Self::UnitsPerMinute => "G94",
            Self::UnitsPerRevolution => "G95",
            Self::InverseTime => "G93",
        }
    }
}

impl Hashable for FeedMode {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// A feed rate together with everything needed to interpret it.
///
/// The spindle speed rides along because `G95` cannot be turned into a duration
/// without it, and because a downstream unit that had to go looking for the
/// spindle speed separately would be a downstream unit that sometimes forgot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeedSpec {
    /// The `F` value as programmed, converted to millimetres where the mode is a
    /// distance rate.
    pub value: f64,
    /// What the value means.
    pub mode: FeedMode,
    /// Spindle speed in rev/min, if one was commanded. Signed: negative is `M4`.
    pub spindle_rpm: Option<f64>,
}

impl FeedSpec {
    /// A rapid, which has no feed rate at all.
    ///
    /// `G0` runs at the machine's rapid rate, which is a machine parameter and
    /// not in the NC file. Representing it as "feed zero" would invite a
    /// division; representing it as the last commanded feed would be a lie.
    #[must_use]
    pub const fn rapid() -> Self {
        Self {
            value: 0.0,
            mode: FeedMode::UnitsPerMinute,
            spindle_rpm: None,
        }
    }

    /// True if this is the placeholder a rapid carries.
    #[must_use]
    pub fn is_rapid(&self) -> bool {
        self.value == 0.0
    }

    /// How long a move of `distance` millimetres takes, in minutes.
    ///
    /// `None` when the answer is not determined by the IR alone: a rapid, whose
    /// rate is a machine parameter, or a `G95` feed with no spindle speed
    /// commanded.
    #[must_use]
    pub fn duration_minutes(&self, distance: f64) -> Option<f64> {
        if self.value <= 0.0 || !self.value.is_finite() {
            return None;
        }
        match self.mode {
            FeedMode::UnitsPerMinute => Some(distance / self.value),
            FeedMode::UnitsPerRevolution => {
                let rpm = self.spindle_rpm?.abs();
                if rpm <= 0.0 {
                    return None;
                }
                Some(distance / (self.value * rpm))
            }
            // The whole block takes 1/F minutes, whatever its length.
            FeedMode::InverseTime => Some(1.0 / self.value),
        }
    }
}

impl Hashable for FeedSpec {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("FeedSpec");
        h.f64(self.value);
        h.add(&self.mode);
        match self.spindle_rpm {
            Some(rpm) => {
                h.bool(true);
                h.f64(rpm);
            }
            None => {
                h.bool(false);
            }
        }
        h.end();
    }
}
