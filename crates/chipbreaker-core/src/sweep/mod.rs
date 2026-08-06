// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Material removal: subtracting a swept tool from a dexel field.
//!
//! # What a cut is
//!
//! For each motion segment, for each bundle, for each ray: compute the intervals
//! of the ray that lie inside the **swept tool volume**, and subtract them from
//! the ray's material. `Spans::subtract` has been property-tested since Unit 1,
//! so the entire difficulty of this unit is computing the swept intervals.
//!
//! # Two contracts
//!
//! **Subtract per bundle, never compare.** A swept volume meets each bundle's
//! rays independently. The three fields will disagree about where a cut surface
//! lies by `O(h)`, with different signs, and reconciling them is Unit 9's job.
//! Nothing here reads one bundle to inform another.
//!
//! **Cutting does not accumulate error.** A cut is exact along each ray:
//! interval arithmetic on exact intersection parameters, not a resampling. After
//! a thousand cuts, bundle X still holds exactly the true remaining solid
//! sampled on X's lattice, and the only error is the fixed transverse sampling
//! set by `h`. This is what makes Unit 15's chained-equals-monolithic test
//! achievable, and [`reference`] is where it is demonstrated rather than
//! asserted.
//!
//! # The reference comes first
//!
//! [`reference`] subdivides a motion into `N` steps and unions the static tool
//! at each. It is slow, it is obviously correct in the limit, and every faster
//! path in this module is differential-tested against it. Two independent
//! computations of the same geometry — one trivially correct, one subtle — is
//! the strongest tool available here, and it is what will catch a sign error in
//! a swept prism that no amount of staring would.

pub mod arc;
pub mod cut;
pub mod horizontal;
pub mod plunge;
pub mod reference;

use crate::math::{Aabb3, Ray, Vec3};
use crate::spans::Spans;
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};

/// A linear motion of a tool: where it starts, where it ends.
///
/// Arcs are Unit 8. This carries no feed, no provenance and no tool number,
/// because none of them affect the geometry of what is removed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearMove {
    /// Tool tip position at the start.
    pub start: Vec3,
    /// Tool tip position at the end.
    pub end: Vec3,
}

impl LinearMove {
    /// The displacement.
    #[inline]
    #[must_use]
    pub fn delta(&self) -> Vec3 {
        self.end - self.start
    }

    /// Horizontal displacement magnitude.
    #[inline]
    #[must_use]
    pub fn horizontal(&self) -> f64 {
        let d = self.delta();
        crate::transcendental::hypot(d.x, d.y)
    }

    /// Vertical displacement magnitude.
    #[inline]
    #[must_use]
    pub fn vertical(&self) -> f64 {
        self.delta().z.abs()
    }

    /// Tool tip position at parameter `s` in `[0, 1]`.
    #[inline]
    #[must_use]
    pub fn at(&self, s: f64) -> Vec3 {
        self.start + self.delta() * s
    }

    /// Which sweep case this motion falls into.
    #[must_use]
    pub fn case(&self) -> SweepCase {
        match (
            self.horizontal() > ZERO_MOTION,
            self.vertical() > ZERO_MOTION,
        ) {
            (false, false) => SweepCase::Stationary,
            (true, false) => SweepCase::Horizontal,
            (false, true) => SweepCase::Plunge,
            (true, true) => SweepCase::Ramp,
        }
    }

    /// The axis-aligned box the swept tool occupies.
    ///
    /// The union of the tool's box at both ends, which is exact for a linear
    /// move because the tool translates rigidly. **This is the cheap rejection
    /// that decides whether a job takes minutes or days**: on a finishing pass a
    /// segment touches a vanishing fraction of a four-million-ray field, so the
    /// rays never examined dominate the cost of the ones that are.
    #[must_use]
    pub fn swept_bounds(&self, profile: &Profile) -> Aabb3 {
        tool_bounds(profile, self.start).union(&tool_bounds(profile, self.end))
    }
}

/// Displacements at or below this count as zero when classifying a move.
///
/// Far below any real machining displacement, so a shallow finishing ramp is
/// never mistaken for a horizontal pass. A move of a nanometre in `z` is a
/// rounding artefact from upstream, not a ramp.
pub const ZERO_MOTION: f64 = 1.0e-9;

/// Which reduction applies to a linear move.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SweepCase {
    /// No motion. The swept volume is the static tool.
    Stationary,
    /// `dz = 0`. Contouring, pocketing, facing, every constant-depth pass.
    Horizontal,
    /// `dxy = 0`. Drilling and every canned cycle Unit 4 expanded.
    Plunge,
    /// Both non-zero. Helical entries, ramped leads, sloped contouring.
    Ramp,
}

impl SweepCase {
    /// Stable name for reports and hashing.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Stationary => "stationary",
            Self::Horizontal => "horizontal",
            Self::Plunge => "plunge",
            Self::Ramp => "ramp",
        }
    }
}

/// The box a tool occupies with its tip at `position`.
#[must_use]
pub fn tool_bounds(profile: &Profile, position: Vec3) -> Aabb3 {
    let r = profile.max_radius();
    let top = profile.total_length();
    Aabb3::from_min_max(
        Vec3::new(position.x - r, position.y - r, position.z),
        Vec3::new(position.x + r, position.y + r, position.z + top),
    )
}

/// Intervals of `ray` inside a tool whose tip sits at `position`.
///
/// The profile lives in its own frame with the tip at the origin, so the ray is
/// translated rather than the tool. `t` is unchanged by a translation, so the
/// spans come back in the caller's parameterisation with no rescaling — which is
/// what lets them be subtracted from a dexel ray directly.
pub fn spans_in_tool_at(
    profile: &Profile,
    position: Vec3,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    let local = Ray {
        origin: ray.origin - position,
        direction: ray.direction,
    };
    profile.intersect_ray_into(&local, scratch, out, stats);
}
