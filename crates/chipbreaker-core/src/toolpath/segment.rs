// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! One move: the unit a field subtracts and a sweep moves along.

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Vec3};
use crate::transcendental as t;

use super::arc::ArcData;
use super::feed::FeedSpec;

/// What kind of move this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MotionKind {
    /// `G0`. No commanded feed; the rate is a machine parameter.
    Rapid,
    /// `G1`.
    Linear,
    /// `G2`/`G3` with no motion normal to the plane.
    Arc,
    /// `G2`/`G3` with motion normal to the plane as well.
    ///
    /// A separate kind rather than an arc with a `z` difference, because the sweep
    /// sweep the two differently and a consumer that has to check for a rise to
    /// find out which it has is a consumer that will forget.
    Helix,
}

impl MotionKind {
    /// Name used in reports and in the IR.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Rapid => "rapid",
            Self::Linear => "linear",
            Self::Arc => "arc",
            Self::Helix => "helix",
        }
    }

    /// True if the tool is expected to be cutting.
    ///
    /// A rapid through material is a crash, not a cut, and it is reported
    /// differently.
    #[must_use]
    pub const fn is_cutting(self) -> bool {
        !matches!(self, Self::Rapid)
    }
}

impl Hashable for MotionKind {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// Tool orientation. **Reserved for 5-axis work and always `None` today.**
///
/// Present now so that adding 5-axis is a change to one crate rather than a
/// schema migration through every unit that consumes a toolpath. The cost is an
/// `Option` that is always empty; the alternative cost is rewriting every
/// downstream consumer to thread a new field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Orient {
    /// Unit vector along the tool axis, from tip toward spindle.
    pub axis: Vec3,
}

impl Hashable for Orient {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Orient");
        h.f64_slice(&self.axis.to_array());
        h.end();
    }
}

/// Where a segment came from in the source program.
///
/// **Not optional.** When a gouge is reported, it must be possible to name the line
/// of NC that caused it. A finding the user cannot trace back to their own
/// program is very nearly worthless — they cannot fix it, and they cannot judge
/// whether it is real.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Provenance {
    /// Which file, indexed into the parse's file table. Subprograms and included
    /// files each get an entry.
    pub file: u32,
    /// One-based line number within that file.
    pub line: u32,
    /// Zero-based index of the block within the file, counting only blocks that
    /// produced motion or state changes.
    pub block: u32,
    /// Which motion of a expanded canned cycle this is, or `u32::MAX` when the
    /// segment came from a block written out longhand.
    ///
    /// One line of `G81` becomes rapid-plunge-retract, and a gouge report that
    /// says "line 42" three times without saying which of the three is a report
    /// that makes the user find out for themselves.
    pub cycle_step: u32,
}

/// The sentinel [`Provenance::cycle_step`] carries when a segment was written
/// out longhand rather than expanded from a cycle.
pub const NOT_A_CYCLE_STEP: u32 = u32::MAX;

impl Provenance {
    /// A segment from an ordinary block.
    #[must_use]
    pub const fn new(file: u32, line: u32, block: u32) -> Self {
        Self {
            file,
            line,
            block,
            cycle_step: NOT_A_CYCLE_STEP,
        }
    }

    /// The same, tagged as the `step`th motion of an expanded cycle.
    #[must_use]
    pub const fn in_cycle(self, step: u32) -> Self {
        Self {
            cycle_step: step,
            ..self
        }
    }

    /// True if this segment came from expanding a canned cycle.
    #[must_use]
    pub const fn is_from_cycle(&self) -> bool {
        self.cycle_step != NOT_A_CYCLE_STEP
    }
}

impl Hashable for Provenance {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Provenance");
        h.u64(u64::from(self.file));
        h.u64(u64::from(self.line));
        h.u64(u64::from(self.block));
        h.u64(u64::from(self.cycle_step));
        h.end();
    }
}

/// One commanded move, fully resolved.
///
/// Coordinates are machine coordinates in millimetres; see the module header of
/// [`super`] and ADR 0003.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionSegment {
    /// What kind of move.
    pub kind: MotionKind,
    /// Where it begins. Equals the previous segment's `end` exactly.
    pub start: Vec3,
    /// Where it ends.
    pub end: Vec3,
    /// Centre, plane and sweep, for [`MotionKind::Arc`] and
    /// [`MotionKind::Helix`]. `None` otherwise.
    pub arc: Option<ArcData>,
    /// **Always `None` today.** See [`Orient`].
    pub orientation: Option<Orient>,
    /// Tool number in force, as programmed with `T`.
    pub tool: u32,
    /// Feed rate and its mode.
    pub feed: FeedSpec,
    /// Which line of which file produced this.
    pub source: Provenance,
}

impl MotionSegment {
    /// Straight-line distance from `start` to `end`.
    ///
    /// For an arc this is the chord, not the path; use [`Self::length`].
    #[must_use]
    pub fn chord(&self) -> f64 {
        (self.end - self.start).length()
    }

    /// Distance travelled along the path.
    ///
    /// For a helix this is the true three-dimensional length: the hypotenuse of
    /// the planar arc length and the rise. Getting that wrong understates a
    /// helical ramp's cutting time by up to the rise, which matters to a report.
    #[must_use]
    pub fn length(&self) -> f64 {
        match (&self.arc, self.kind) {
            (Some(arc), MotionKind::Helix) => {
                let normal = arc.plane.normal();
                let rise = (self.end - self.start).dot(normal);
                t::hypot(arc.planar_length(), rise)
            }
            (Some(arc), _) => arc.planar_length(),
            (None, _) => self.chord(),
        }
    }

    /// A box containing every point on the path.
    ///
    /// For an arc this bounds the whole circle rather than solving for the
    /// extreme points of the swept portion. That is deliberately conservative:
    /// it is used to size the dexel field and to reject rays early, where being
    /// slightly too large costs a little work and being too small loses
    /// material.
    #[must_use]
    pub fn bounds(&self) -> Aabb3 {
        let mut bounds = Aabb3::from_min_max(self.start, self.end);
        if let Some(arc) = &self.arc {
            let [u, v, _] = arc.plane.axes();
            let mut lo = arc.center.to_array();
            let mut hi = arc.center.to_array();
            lo[u] -= arc.radius;
            hi[u] += arc.radius;
            lo[v] -= arc.radius;
            hi[v] += arc.radius;
            // The out-of-plane axis is bounded by the endpoints, which
            // `from_min_max` already covers.
            let circle = Aabb3::from_min_max(Vec3::from_array(lo), Vec3::from_array(hi));
            let normal_axis = arc.plane.axes()[2];
            let mut min = circle.min.to_array();
            let mut max = circle.max.to_array();
            min[normal_axis] = bounds.min.to_array()[normal_axis];
            max[normal_axis] = bounds.max.to_array()[normal_axis];
            bounds = Aabb3::from_min_max(Vec3::from_array(min), Vec3::from_array(max));
        }
        bounds
    }

    /// True if the move commands no motion at all.
    ///
    /// Zero-length segments are dropped at construction: they carry no geometry,
    /// they would divide by zero in any direction calculation, and a full circle
    /// is *not* one of them — its start and end coincide but its sweep does not
    /// vanish.
    #[must_use]
    pub fn is_degenerate(&self) -> bool {
        match &self.arc {
            Some(arc) => arc.sweep == 0.0 || arc.radius == 0.0,
            None => self.start == self.end,
        }
    }
}

impl Hashable for MotionSegment {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("MotionSegment");
        h.add(&self.kind);
        h.f64_slice(&self.start.to_array());
        h.f64_slice(&self.end.to_array());
        match &self.arc {
            Some(arc) => {
                h.bool(true);
                h.add(arc);
            }
            None => {
                h.bool(false);
            }
        }
        match &self.orientation {
            Some(o) => {
                h.bool(true);
                h.add(o);
            }
            None => {
                h.bool(false);
            }
        }
        h.u64(u64::from(self.tool));
        h.add(&self.feed);
        h.add(&self.source);
        h.end();
    }
}
