// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Case B: pure plunges, `dxy = 0`.
//!
//! Drilling, and every canned cycle Unit 4 expanded.
//!
//! # The swept volume is itself a solid of revolution
//!
//! With `dxy = 0` the condition becomes `exists zeta in window : |u| <= rho(zeta)`,
//! so the swept radius is
//!
//! ```text
//! rho_swept(w)  =  max over zeta in [w - dz, w] of rho(zeta)
//! ```
//!
//! which depends only on height. A **moving maximum**, and the place a subtle
//! error would hide: sweeping the profile takes the *upper envelope* of `rho`
//! over the window, which is not the same as translating the profile chain.
//!
//! That distinction is not academic. `bull-10-r2` in the standard library has
//! radii `0, 3, 5, 5, 4, 4` — the shank necks in below the cutting diameter — so
//! translating its chain would under-report the swept solid over the whole neck.
//! Any tool with an undercut, a dovetail, or a relieved shank does the same.
//!
//! # Computed pointwise, not constructed
//!
//! Building the envelope as a new `Profile` chain means computing the upper
//! envelope of a set of arcs and segments, which is the genuinely hard part.
//! This module never does it. Because the boundary of a radially convex solid of
//! revolution *is* its profile chain,
//!
//! ```text
//! rho_swept(lo, hi)  =  max { r : (r, z) on the chain, z in [lo, hi] }
//! ```
//!
//! and that is a finite maximum over per-element candidates: clipped endpoints,
//! and each arc's rightmost point where `dr/dz` vanishes. Exact, and it sidesteps
//! the envelope entirely.
//!
//! # Then the rays split by orientation, and both halves are exact
//!
//! - **Along the plunge** (a Z bundle): the tool translates rigidly along the
//!   ray, so the swept spans are the static spans dilated by the motion. No cast
//!   at all.
//! - **Across it** (an X or Y bundle): the ray sits at constant height, where
//!   the swept solid is a disc of radius `rho_swept`. The span is that disc's
//!   chord.
//!
//! # The precondition, checked rather than assumed
//!
//! Both halves assume the solid is **radially convex**: that every `r` below the
//! boundary is inside. Every tool in the catalogue is, because a tool is filled
//! from its axis outward — but `Profile` does not require it, and a chain that
//! doubled back to enclose an annular void would break the disc argument
//! silently. [`is_radially_convex`] checks, and the dispatcher falls back to
//! bounded sub-stepping when it fails, so an exotic profile degrades in accuracy
//! rather than in correctness.

use crate::math::Ray;
use crate::spans::{Span, Spans};
use crate::tool::raycast::{RaycastScratch, RaycastStats};
use crate::tool::{Profile, ProfileElement};

use super::{LinearMove, spans_in_tool_at};

/// Direction components at or below this count as zero.
const DEGENERATE: f64 = 1.0e-12;

/// Radial samples used to check radial convexity.
///
/// A check, not a computation: it looks for an annular void by testing whether
/// the solid is continuous from the axis outward at a spread of heights. Coarse
/// on purpose, because a profile that fails it falls back to sub-stepping and a
/// profile that passes is one of the ordinary ones.
const CONVEXITY_SAMPLES: u32 = 64;

/// The largest radius the profile chain reaches with `z` in `[lo, hi]`.
///
/// The moving maximum, exactly. Returns zero when no part of the chain lies in
/// the window, which is the honest answer: the tool is not there.
#[must_use]
pub fn max_radius_over_z(profile: &Profile, lo: f64, hi: f64) -> f64 {
    let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
    let mut best = 0.0f64;
    let mut consider = |r: f64, z: f64| {
        if z >= lo && z <= hi {
            best = best.max(r);
        }
    };

    for roled in profile.elements() {
        let element = &roled.element;
        // Endpoints, whenever they fall in the window.
        consider(element.start().x, element.start().y);
        consider(element.end().x, element.end().y);

        match element {
            ProfileElement::Segment { start, end } => {
                // `r` is affine in `z`, so its maximum over a clipped range is at
                // an end of that range. A segment at constant `z` -- an annular
                // face -- is covered by the endpoints already.
                if (end.y - start.y).abs() > DEGENERATE {
                    for edge in [lo, hi] {
                        let s = (edge - start.y) / (end.y - start.y);
                        if (0.0..=1.0).contains(&s) {
                            consider(start.x + (end.x - start.x) * s, edge);
                        }
                    }
                }
            }
            ProfileElement::Arc { center, .. } => {
                let radius = element.radius().unwrap_or(0.0);
                // Where the window's edges cross the circle.
                for edge in [lo, hi] {
                    let dz = edge - center.y;
                    let inside = radius * radius - dz * dz;
                    if inside < 0.0 {
                        continue;
                    }
                    let dr = inside.sqrt();
                    for r in [center.x + dr, center.x - dr] {
                        if element
                            .contains_angle(crate::transcendental::atan2(dz, r - center.x), 1.0e-12)
                        {
                            consider(r, edge);
                        }
                    }
                }
                // And the arc's rightmost point, where `dr/dz` vanishes. This is
                // the interior critical point, and leaving it out is exactly how
                // a bulged tool would be under-reported.
                if element.contains_angle(0.0, 1.0e-12) {
                    consider(center.x + radius, center.y);
                }
            }
        }
    }
    best
}

/// True if the solid fills continuously from the axis outward.
///
/// The precondition both exact paths rest on. See the module header.
#[must_use]
pub fn is_radially_convex(profile: &Profile) -> bool {
    let top = profile.total_length();
    let max_r = profile.max_radius();
    if top <= 0.0 || max_r <= 0.0 {
        return false;
    }
    for i in 1..CONVEXITY_SAMPLES {
        let z = top * f64::from(i) / f64::from(CONVEXITY_SAMPLES);
        // Walk outward and look for material returning after a gap, which is
        // what an annular void looks like from the axis.
        let mut seen_gap = false;
        for j in 0..CONVEXITY_SAMPLES {
            let r = max_r * f64::from(j) / f64::from(CONVEXITY_SAMPLES);
            let inside = profile.contains_rz(r, z);
            if inside && seen_gap {
                return false;
            }
            if !inside {
                seen_gap = true;
            }
        }
    }
    true
}

/// Intervals of `ray` inside a plunge-swept tool.
///
/// Returns `false` without touching `out` when the ray is neither along the
/// plunge nor across it, or when the profile is not radially convex. The caller
/// then falls back to bounded sub-stepping.
///
/// # Panics
/// Panics if the motion has no vertical extent.
pub fn swept_spans_into(
    profile: &Profile,
    motion: &LinearMove,
    ray: &Ray,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) -> bool {
    let dz = motion.delta().z;
    assert!(
        dz.abs() > 0.0,
        "a plunge sweep needs vertical motion; dispatch on LinearMove::case first"
    );
    if !is_radially_convex(profile) {
        return false;
    }

    let horizontal_ray =
        crate::transcendental::hypot(ray.direction.x, ray.direction.y) > DEGENERATE;
    let vertical_ray = ray.direction.z.abs() > DEGENERATE;

    if vertical_ray && !horizontal_ray {
        // Along the plunge: the tool translates rigidly along the ray, so the
        // spans translate with it. Dilating them is the whole answer, and it
        // needs no cast beyond the one static one.
        out.clear();
        let mut base = Spans::new();
        spans_in_tool_at(profile, motion.start, ray, scratch, &mut base, stats);
        if base.is_empty() {
            return true;
        }
        // The ray direction is `+axis` and unit, so a tool translation of `dz`
        // moves a span by `dz / direction.z`.
        let shift = dz / ray.direction.z;
        let (lo, hi) = if shift <= 0.0 {
            (shift, 0.0)
        } else {
            (0.0, shift)
        };
        for span in base.iter() {
            out.push_merge(Span::ordered(span.t0 + lo, span.t1 + hi));
        }
        return true;
    }

    if horizontal_ray && !vertical_ray {
        // Across the plunge: the ray is at constant height, and the swept solid
        // there is a disc.
        out.clear();
        let w = ray.origin.z - motion.start.z;
        let radius = max_radius_over_z(profile, w - dz, w);
        if radius <= 0.0 {
            return true;
        }
        // Closest approach of the ray to the tool axis, in plan.
        let ax = motion.start.x;
        let ay = motion.start.y;
        let ox = ray.origin.x - ax;
        let oy = ray.origin.y - ay;
        let dx = ray.direction.x;
        let dy = ray.direction.y;
        let speed2 = dx * dx + dy * dy;
        let closest = -(ox * dx + oy * dy) / speed2;
        let miss2 = (ox + dx * closest).powi(2) + (oy + dy * closest).powi(2);
        let inside = radius * radius - miss2;
        if inside <= 0.0 {
            // Tangential or clear: no interval, and deliberately not a sliver.
            return true;
        }
        let half = inside.sqrt() / speed2.sqrt();
        out.push_merge(Span::ordered(closest - half, closest + half));
        return true;
    }

    false
}
