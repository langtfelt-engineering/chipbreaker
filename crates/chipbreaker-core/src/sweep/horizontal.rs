// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Case A: horizontal motion, `dz = 0`.
//!
//! Contouring, pocketing, facing, and every finishing pass at constant depth.
//!
//! # The decomposition, and why it is exactly three pieces
//!
//! With `dz = 0` the profile's radius is constant along the motion, so at every
//! height `z` the swept cross-section is the disc of radius `rho(z)` dragged
//! along the segment: a **stadium**. A stadium is a half-disc at each end plus a
//! rectangle, and each of those is contained in the corresponding full disc or
//! in the rectangle. So
//!
//! ```text
//! swept  =  tool(P0)  union  tool(P1)  union  prism
//! ```
//!
//! and the argument is an equality in both directions. Every sampled tool
//! position is inside the union, because a point within `rho(z)` of *some* point
//! of the segment is within `rho(z)` of the *nearest* point of the segment, and
//! that nearest point is either an endpoint or interior. Conversely each of the
//! three pieces is inside the sweep: the ends at `s = 0` and `s = 1`, and the
//! prism because a point of it projects onto the segment at some `s`.
//!
//! `Spans::union` handles the overlap between the pieces, so there is no
//! double-counting to reason about — which is the property that makes a
//! three-piece decomposition worth having over a single clever formula.
//!
//! # The middle piece is a problem the stationary raycaster already solves
//!
//! Set up a frame on the motion: `d` along it, `n` perpendicular, both
//! horizontal. A point is in the prism iff its projection `a` lies in `[0, L]`
//! and `|b| <= rho(w)`, where `b` is the perpendicular offset and `w` the height
//! above the tool tip.
//!
//! The region `{ (b, w) : |b| <= rho(w) }` is **exactly the revolved tool solid
//! cut by the plane `y = 0`**, because `hypot(b, 0) = |b|`. So the middle piece
//! needs no new intersection code at all: map the ray into `(b, 0, w)`, hand it
//! to `Profile::intersect_ray`, and clip by the slab. The root solver, the
//! tangency policy and `EPS_TANGENT` all carry over unchanged, which is the
//! whole reason to write it this way rather than deriving a swept surface.
//!
//! One wrinkle: the mapped direction is not a unit vector, so the returned
//! parameters are in units of its length and have to be rescaled. The
//! raycaster's
//! tangency tolerance is a length and depends on that normalisation, so the ray
//! is normalised before casting and the spans divided afterwards, rather than
//! casting an unnormalised ray and hoping.

use crate::math::{Ray, Vec2, Vec3};
use crate::spans::{Span, Spans};
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};

use super::{LinearMove, spans_in_tool_at};

/// Below this, the mapped cross-section direction counts as degenerate.
///
/// Reached when the ray runs parallel to the motion and level with it, so the
/// ray's `(b, w)` image is a single point rather than a line. Not an edge case
/// to be tolerated away: an X bundle meeting a move along X hits it on every
/// single ray.
const DEGENERATE_SPEED: f64 = 1.0e-12;

/// Intervals of `ray` inside a horizontally swept tool.
///
/// `out` is cleared first. The caller must have established that the motion is
/// horizontal; [`super::SweepCase::Horizontal`] is how.
///
/// # Panics
/// Panics if the motion has no horizontal extent, which would make the frame
/// undefined. Callers dispatch on [`super::LinearMove::case`] first.
pub fn swept_spans_into(
    profile: &Profile,
    motion: &LinearMove,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    let length = motion.horizontal();
    assert!(
        length > 0.0,
        "a horizontal sweep needs horizontal motion; dispatch on LinearMove::case first"
    );

    out.clear();
    let mut piece = Spans::new();
    let mut merged = Spans::new();

    // The two ends, verbatim from the stationary raycaster.
    for position in [motion.start, motion.end] {
        spans_in_tool_at(profile, position, ray, scratch, &mut piece, stats);
        if !piece.is_empty() {
            out.union_into(&piece, &mut merged);
            core::mem::swap(out, &mut merged);
        }
    }

    // And the middle.
    prism_spans_into(profile, motion, length, ray, scratch, &mut piece, stats);
    if !piece.is_empty() {
        out.union_into(&piece, &mut merged);
        core::mem::swap(out, &mut merged);
    }
}

/// The middle piece: the profile region extruded along the motion.
fn prism_spans_into(
    profile: &Profile,
    motion: &LinearMove,
    length: f64,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    out.clear();
    let delta = motion.delta();
    let along = Vec2::new(delta.x / length, delta.y / length);
    // Perpendicular, in the same handedness as `Axis::cyclic`: rotating
    // `(x, y)` by ninety degrees gives `(-y, x)`.
    let across = Vec2::new(-along.y, along.x);

    let relative = Vec2::new(ray.origin.x - motion.start.x, ray.origin.y - motion.start.y);
    let direction = Vec2::new(ray.direction.x, ray.direction.y);

    // Projection onto the motion, which the slab constrains.
    let a0 = relative.x * along.x + relative.y * along.y;
    let da = direction.x * along.x + direction.y * along.y;
    // Perpendicular offset and height, which the profile region constrains.
    let b0 = relative.x * across.x + relative.y * across.y;
    let db = direction.x * across.x + direction.y * across.y;
    let w0 = ray.origin.z - motion.start.z;
    let dw = ray.direction.z;

    let Some(slab) = slab_interval(a0, da, length) else {
        return;
    };

    let speed = crate::transcendental::hypot(db, dw);
    if speed <= DEGENERATE_SPEED {
        // The ray runs along the motion at a fixed offset and height, so its
        // whole length is inside the cross-section or none of it is. An X
        // bundle meeting a move along X takes this path on every ray.
        if profile.contains_rz(b0.abs(), w0) {
            out.push_merge(slab);
        }
        return;
    }

    // The cross-section, cast as an ordinary profile ray in the plane `y = 0`.
    let local = Ray {
        origin: Vec3::new(b0, 0.0, w0),
        direction: Vec3::new(db / speed, 0.0, dw / speed),
    };
    let mut cross = Spans::new();
    profile.intersect_ray_into(&local, scratch, &mut cross, stats);
    if cross.is_empty() {
        return;
    }

    // Back into the caller's parameter, then clipped to the slab.
    //
    // The normals come back in the cross-section's own frame, where `x` is the
    // perpendicular offset `b` and `z` is the height `w`. The prism's surface
    // has no component along the motion -- that is what makes it a prism -- so
    // rotating `(n_b, n_w)` back into the world is one linear combination of
    // `across` and the vertical, with nothing along `along`.
    let restore = |n: crate::math::OctNormal| {
        let local = n.decode();
        crate::math::OctNormal::encode(Vec3::new(local.x * across.x, local.x * across.y, local.z))
    };
    let mut rescaled = Spans::with_capacity(cross.len());
    for span in cross.iter() {
        rescaled.push_merge(Span::with_normals(
            span.t0 / speed,
            span.t1 / speed,
            restore(span.n0),
            restore(span.n1),
        ));
    }
    let clipped = rescaled.clipped_to(slab);
    for span in clipped.iter() {
        out.push_merge(*span);
    }
}

/// The interval of `t` for which `a0 + t * da` lies in `[0, length]`.
///
/// `None` when the ray never enters the slab. A ray perpendicular to the motion
/// has `da = 0` and is either inside for all `t` or outside for all of it, which
/// is why the zero case is decided on `a0` rather than rejected.
fn slab_interval(a0: f64, da: f64, length: f64) -> Option<Span> {
    if da.abs() <= DEGENERATE_SPEED {
        return if (0.0..=length).contains(&a0) {
            Some(Span::new(f64::NEG_INFINITY, f64::INFINITY))
        } else {
            None
        };
    }
    let t0 = -a0 / da;
    let t1 = (length - a0) / da;
    Some(Span::ordered(t0, t1))
}

/// Allocating form of [`swept_spans_into`].
///
/// # Panics
/// See [`swept_spans_into`].
#[must_use]
pub fn swept_spans(profile: &Profile, motion: &LinearMove, ray: &Ray) -> Spans {
    let mut scratch = RaycastScratch::with_capacity(profile.len());
    let mut out = Spans::new();
    let mut stats = RaycastStats::default();
    swept_spans_into(profile, motion, ray, &mut scratch, &mut out, &mut stats);
    out
}
