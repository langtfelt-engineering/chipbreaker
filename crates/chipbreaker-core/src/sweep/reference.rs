// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The dense sub-stepping reference: slow, obvious, and the ground truth.
//!
//! Subdivide a motion into `N` steps, cast the static tool at each, union the
//! results. Every faster path in this module is differential-tested against it.
//!
//! # Why it is correct in the limit, and only in the limit
//!
//! The union over sampled positions is a **subset** of the true swept volume:
//! every sampled tool really is inside the sweep, and nothing outside it can
//! appear. So the reference under-reports, monotonically, and converges upward
//! as `N` grows. That one-sidedness is what makes it usable as ground truth — a
//! disagreement in one direction is a convergence question, and a disagreement
//! in the other is a bug.
//!
//! # The deviation bound
//!
//! Between consecutive samples the tool translates by `|delta| / N`. A point of
//! the true sweep sits between two samples, so it is within half that of a
//! sampled position — and because the tool translates rigidly, a point missed by
//! the union is at most `|delta| / (2N)` from material the union does contain.
//!
//! ```text
//! deviation  <=  |delta| / (2 N)
//! ```
//!
//! [`substeps_for_error`] inverts it. That is a **computed bound, not a step
//! count**: "we used 64 sub-steps" guarantees nothing, and this module never
//! reports a step count without the bound it achieves.
//!
//! The bound is on the *geometry*, not on the removed volume. Volume error is
//! bounded by the deviation times the swept surface area, which is a larger and
//! less useful statement; the deviation is what a customer's tolerance is
//! expressed in.

use crate::math::Ray;
use crate::spans::Spans;
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};

use super::{LinearMove, spans_in_tool_at};

/// The largest number of sub-steps [`substeps_for_error`] will ask for.
///
/// A guard, not a tuning parameter: without it a tolerance of zero, or one
/// accidentally given in metres, asks for an unbounded loop. Callers that hit
/// the cap get a worse bound and are told so rather than being silently served.
pub const MAX_SUBSTEPS: u32 = 1 << 20;

/// Sub-steps needed to bring the reference's deviation under `tolerance`.
///
/// From `deviation <= |delta| / (2N)`, so `N >= |delta| / (2 * tolerance)`.
///
/// Returns the step count and the bound it actually achieves — never the count
/// alone. A caller that wants to state an accuracy must state the second number,
/// and making it awkward to drop is deliberate.
///
/// # Panics
/// Panics if `tolerance` is not positive and finite.
#[must_use]
pub fn substeps_for_error(motion: &LinearMove, tolerance: f64) -> (u32, f64) {
    assert!(
        tolerance.is_finite() && tolerance > 0.0,
        "the sweep tolerance must be a positive length, got {tolerance}"
    );
    let distance = motion.delta().length();
    if distance <= 0.0 {
        return (1, 0.0);
    }
    let wanted = distance / (2.0 * tolerance);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "clamped into u32 immediately below"
    )]
    let n = if wanted.is_finite() && wanted < f64::from(MAX_SUBSTEPS) {
        (wanted.ceil() as u32).max(1)
    } else {
        MAX_SUBSTEPS
    };
    (n, distance / (2.0 * f64::from(n)))
}

/// Intervals of `ray` inside the swept volume, by dense sub-stepping.
///
/// `steps` is the number of intervals, so `steps + 1` tool positions are cast
/// including both endpoints. `out` is cleared first.
///
/// # Panics
/// Panics if `steps` is zero.
pub fn swept_spans_into(
    profile: &Profile,
    motion: &LinearMove,
    steps: u32,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    assert!(steps > 0, "a reference sweep needs at least one step");
    out.clear();
    let mut piece = Spans::new();
    let mut merged = Spans::new();
    for k in 0..=steps {
        let s = f64::from(k) / f64::from(steps);
        let position = motion.at(s);
        spans_in_tool_at(profile, position, ray, scratch, &mut piece, stats);
        if piece.is_empty() {
            continue;
        }
        // Accumulate through `union_into` rather than pushing: the pieces
        // overlap heavily by construction, and `Spans::union` is the operation
        // that has been property-tested from the start for exactly this.
        out.union_into(&piece, &mut merged);
        core::mem::swap(out, &mut merged);
    }
}

/// Intervals of `ray` inside a swept arc or helix, by dense sub-stepping.
///
/// The same machinery as the linear reference; only the path differs. `out` is
/// cleared first.
///
/// # Panics
/// Panics if `steps` is zero.
pub fn arc_spans_into(
    profile: &Profile,
    arc: &super::arc::ArcMove,
    steps: u32,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    assert!(steps > 0, "a reference sweep needs at least one step");
    out.clear();
    let mut piece = Spans::new();
    let mut merged = Spans::new();
    for k in 0..=steps {
        let s = f64::from(k) / f64::from(steps);
        spans_in_tool_at(profile, arc.at(s), ray, scratch, &mut piece, stats);
        if piece.is_empty() {
            continue;
        }
        out.union_into(&piece, &mut merged);
        core::mem::swap(out, &mut merged);
    }
}

/// Allocating form of [`swept_spans_into`].
///
/// # Panics
/// Panics if `steps` is zero.
#[must_use]
pub fn swept_spans(profile: &Profile, motion: &LinearMove, steps: u32, ray: &Ray) -> Spans {
    let mut scratch = RaycastScratch::with_capacity(profile.len());
    let mut out = Spans::new();
    let mut stats = RaycastStats::default();
    swept_spans_into(
        profile,
        motion,
        steps,
        ray,
        &mut scratch,
        &mut out,
        &mut stats,
    );
    out
}
