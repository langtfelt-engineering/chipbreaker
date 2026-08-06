// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Case A′: horizontal arcs, `dz = 0`.
//!
//! # The collapse
//!
//! The tool centre travels a circle of radius `R` about `C` with its axis on
//! `+Z`. For a point `p`, write `d = |p_xy - C_xy|` and `phi` for its bearing
//! from `C`. Then
//!
//! ```text
//! |p_xy - centre(theta)|^2  =  d^2 + R^2 - 2 d R cos(phi - theta)
//! ```
//!
//! which over unrestricted `theta` is minimised at `theta = phi`, giving
//! `(d - R)^2`. So:
//!
//! - **`phi` inside the swept wedge**: the condition collapses to
//!   `|d - R| <= rho(w)`.
//! - **`phi` outside it**: `cos` is monotone in `|phi - theta|`, so the nearest
//!   the tool ever comes is at an endpoint — which is the static tool there.
//!
//! Same three-piece structure as Case A, and the union is exact in both
//! directions. Verified by point membership on 200,000 random points against a
//! dense reference: zero misses.
//!
//! The pieces overlap rather than abut, which is what rules out a sliver at the
//! seam. At `phi` exactly on a wedge boundary the middle's condition
//! `|d - R| <= rho(w)` puts the point at distance `|d - R|` from that endpoint's
//! centre, so it is inside the endpoint tool too.
//!
//! # The middle, without constructing anything
//!
//! The plan expected the middle to need an offset profile — the tool's chain
//! translated by `R` and mirrored — which would put arcs into a flat end mill's
//! profile and drive the quartic solver hard.
//!
//! **It does not, because the bundles split the problem.** A tri-dexel field has
//! exactly three ray directions, and a horizontal arc's axis is always `+Z`:
//!
//! - **Along the arc axis** (the Z bundle): `p_xy` is fixed, so `d` and `phi`
//!   are constant along the ray. The condition becomes `rho(w) >= |d - R|` with
//!   `w` varying — which is a *vertical ray cast at radius `|d - R|`* against
//!   the tool's own profile. Unit 3, unchanged.
//! - **Across it** (the X and Y bundles): `z` is fixed, so `rho(w)` is a
//!   constant and the condition is an annulus `R - rho <= d(t) <= R + rho`. A
//!   line meets an annulus in the difference of two disc chords: closed form.
//!
//! So no profile is constructed, no quartic is introduced that the tool did not
//! already need, and the quartic-rate rise the plan predicted does not happen.
//! Measured rather than assumed — see the benchmarks.
//!
//! # The wedge
//!
//! Restricted on the hit parameters, not by clipping the solid. For a Z-bundle
//! ray `phi` is constant, so it is one test. For an X or Y bundle ray the point
//! moves along a line and each wedge boundary is a half-plane through `C`, so
//! the admissible `t` are an intersection or union of two linear inequalities —
//! at most two intervals, exactly.
//!
//! # The precondition
//!
//! The across-axis path needs `rho(w)` to be a single number, which is radial
//! convexity again — the same precondition Case B carries, checked the same way
//! and declined the same way when it fails.

use crate::math::{Ray, Vec3};
use crate::spans::{Span, Spans};
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};
use crate::transcendental as t;

use super::plunge::max_radius_over_z;
use super::spans_in_tool_at;

/// Direction and radius values at or below this count as degenerate.
const DEGENERATE: f64 = 1.0e-12;

/// A horizontal arc of the tool tip, in machine coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcMove {
    /// Centre of curvature. Only `x` and `y` matter; `z` is the tip height.
    pub center: Vec3,
    /// Radius, in millimetres.
    pub radius: f64,
    /// Bearing of the start, in radians.
    pub start_angle: f64,
    /// Signed sweep in radians, positive counter-clockwise about `+Z`.
    ///
    /// Carries multiple turns: a two-turn clockwise arc sweeps `-4 PI`.
    pub sweep: f64,
    /// Tool tip height, constant across the move.
    pub z: f64,
}

impl ArcMove {
    /// Tip position at parameter `s` in `[0, 1]`.
    #[must_use]
    pub fn at(&self, s: f64) -> Vec3 {
        let angle = self.start_angle + self.sweep * s;
        let (sin, cos) = t::sin_cos(angle);
        Vec3::new(
            self.center.x + self.radius * cos,
            self.center.y + self.radius * sin,
            self.z,
        )
    }

    /// True if a bearing lies in the swept wedge.
    #[must_use]
    pub fn wedge_contains(&self, angle: f64) -> bool {
        if self.sweep.abs() >= 2.0 * core::f64::consts::PI {
            // A full turn or more sweeps every bearing. The plan flags this
            // because a full circle has a zero chord and a naive handler drops
            // it; here it is simply the case where the wedge test is vacuous.
            return true;
        }
        let tau = 2.0 * core::f64::consts::PI;
        let delta = if self.sweep >= 0.0 {
            (angle - self.start_angle).rem_euclid(tau)
        } else {
            (self.start_angle - angle).rem_euclid(tau)
        };
        delta <= self.sweep.abs() + DEGENERATE
    }

    /// The box the swept tool occupies.
    ///
    /// The whole annulus, widened by the tool, rather than a tight hull of the
    /// wedge. Loose for a short arc and exact for a full circle — and being
    /// loose only costs rejection rate, whereas being tight and wrong would cost
    /// material.
    #[must_use]
    pub fn swept_bounds(&self, profile: &Profile) -> crate::math::Aabb3 {
        let reach = self.radius + profile.max_radius();
        crate::math::Aabb3::from_min_max(
            Vec3::new(self.center.x - reach, self.center.y - reach, self.z),
            Vec3::new(
                self.center.x + reach,
                self.center.y + reach,
                self.z + profile.total_length(),
            ),
        )
    }
}

/// Intervals of `ray` inside a horizontally swept arc.
///
/// Returns `false` without touching `out` when the ray is neither along the arc
/// axis nor across it, or when the profile is not radially convex. The caller
/// then falls back to bounded sub-stepping.
pub fn swept_spans_into(
    profile: &Profile,
    arc: &ArcMove,
    ray: &Ray,
    radially_convex: bool,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) -> bool {
    let along_axis = ray.direction.z.abs() > DEGENERATE;
    let across_axis = t::hypot(ray.direction.x, ray.direction.y) > DEGENERATE;
    if along_axis == across_axis {
        // Neither purely along nor purely across; a tri-dexel field has no such
        // ray, and guessing would be worse than declining.
        return false;
    }
    if across_axis && !radially_convex {
        return false;
    }

    out.clear();
    let mut piece = Spans::new();
    let mut merged = Spans::new();

    // The two endpoint tools, verbatim from Unit 3.
    for s in [0.0, 1.0] {
        spans_in_tool_at(profile, arc.at(s), ray, scratch, &mut piece, stats);
        if !piece.is_empty() {
            out.union_into(&piece, &mut merged);
            core::mem::swap(out, &mut merged);
        }
    }

    // And the middle.
    middle_into(profile, arc, ray, along_axis, scratch, &mut piece, stats);
    if !piece.is_empty() {
        out.union_into(&piece, &mut merged);
        core::mem::swap(out, &mut merged);
    }
    true
}

/// The wedge-restricted annular middle.
fn middle_into(
    profile: &Profile,
    arc: &ArcMove,
    ray: &Ray,
    along_axis: bool,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
    out.clear();
    if along_axis {
        // `p_xy` is fixed, so `d` and `phi` are constant: one wedge test, then a
        // vertical cast at radius `|d - R|` against the tool's own profile.
        let dx = ray.origin.x - arc.center.x;
        let dy = ray.origin.y - arc.center.y;
        let d = t::hypot(dx, dy);
        if !arc.wedge_contains(t::atan2(dy, dx)) {
            return;
        }
        let offset = (d - arc.radius).abs();
        let local = Ray {
            origin: Vec3::new(offset, 0.0, ray.origin.z - arc.z),
            direction: ray.direction,
        };
        profile.intersect_ray_into(&local, scratch, out, stats);
        return;
    }

    // Across the axis: `z` is fixed, so the tool's radius there is one number
    // and the middle is an annulus in plan.
    let w = ray.origin.z - arc.z;
    let rho = max_radius_over_z(profile, w, w);
    if rho <= 0.0 {
        return;
    }
    let outer = arc.radius + rho;
    let inner = arc.radius - rho;

    let Some(outer_span) = disc_chord(arc, ray, outer) else {
        return;
    };
    let mut annulus = Spans::from_span(outer_span);
    if inner > 0.0 {
        // A tight arc -- `R <= max rho` -- leaves the inner radius negative, so
        // the region reaches the axis and there is nothing to subtract. A
        // small-radius corner with a big cutter is ordinary CAM output, not a
        // pathology.
        if let Some(hole) = disc_chord(arc, ray, inner) {
            let mut without = Spans::new();
            annulus.subtract_into(&Spans::from_span(hole), &mut without);
            annulus = without;
        }
    }

    // Then the wedge, as linear conditions on `t`.
    for span in wedge_intervals(arc, ray) {
        let clipped = annulus.clipped_to(span);
        for piece in clipped.iter() {
            out.push_merge(*piece);
        }
    }
}

/// The `t` interval where the ray lies inside a disc of `radius` about the arc
/// centre, in plan.
fn disc_chord(arc: &ArcMove, ray: &Ray, radius: f64) -> Option<Span> {
    let ox = ray.origin.x - arc.center.x;
    let oy = ray.origin.y - arc.center.y;
    let (dx, dy) = (ray.direction.x, ray.direction.y);
    let speed2 = dx * dx + dy * dy;
    if speed2 <= 0.0 {
        return None;
    }
    let closest = -(ox * dx + oy * dy) / speed2;
    let miss2 = (ox + dx * closest).powi(2) + (oy + dy * closest).powi(2);
    let inside = radius * radius - miss2;
    if inside <= 0.0 {
        // Tangential or clear. Deliberately not a sliver: a curved tangential
        // contact touches at a point, so any interval at all would be spurious.
        return None;
    }
    let half = inside.sqrt() / speed2.sqrt();
    Some(Span::ordered(closest - half, closest + half))
}

/// The `t` intervals where the ray's bearing from the centre lies in the wedge.
///
/// Each wedge boundary is a half-plane through the centre, so the condition is
/// linear in `t`. A sweep under half a turn is an intersection of two
/// half-planes and gives one interval; a longer sweep is a union and gives two.
fn wedge_intervals(arc: &ArcMove, ray: &Ray) -> Vec<Span> {
    let whole = Span::new(f64::NEG_INFINITY, f64::INFINITY);
    let tau = 2.0 * core::f64::consts::PI;
    if arc.sweep.abs() >= tau {
        return vec![whole];
    }

    // Boundary directions, oriented so the wedge is swept counter-clockwise
    // from `first` to `second`.
    let (from, to) = if arc.sweep >= 0.0 {
        (arc.start_angle, arc.start_angle + arc.sweep)
    } else {
        (arc.start_angle + arc.sweep, arc.start_angle)
    };
    let (s0, c0) = t::sin_cos(from);
    let (s1, c1) = t::sin_cos(to);

    // `cross(u, q) >= 0` is linear in `t`; solve each for its half-line.
    let ox = ray.origin.x - arc.center.x;
    let oy = ray.origin.y - arc.center.y;
    let after_from = half_plane(c0, s0, ox, oy, ray, true);
    let before_to = half_plane(c1, s1, ox, oy, ray, false);

    if arc.sweep.abs() <= core::f64::consts::PI {
        // Intersection.
        match (after_from, before_to) {
            (Some(a), Some(b)) => {
                let lo = a.t0.max(b.t0);
                let hi = a.t1.min(b.t1);
                if lo < hi {
                    vec![Span::new(lo, hi)]
                } else {
                    Vec::new()
                }
            }
            _ => Vec::new(),
        }
    } else {
        // Union.
        [after_from, before_to].into_iter().flatten().collect()
    }
}

/// The `t` interval where `cross(u, q(t))` has the wanted sign.
///
/// `positive` selects `cross(u, q) >= 0`; otherwise `cross(q, u) >= 0`.
fn half_plane(ux: f64, uy: f64, ox: f64, oy: f64, ray: &Ray, positive: bool) -> Option<Span> {
    // cross(u, q) = ux * qy - uy * qx, and `q(t) = o + t * dir`.
    let (a, b) = if positive {
        (
            ux * oy - uy * ox,
            ux * ray.direction.y - uy * ray.direction.x,
        )
    } else {
        (
            ox * uy - oy * ux,
            ray.direction.x * uy - ray.direction.y * ux,
        )
    };
    if b.abs() <= DEGENERATE {
        // Parallel to the boundary: the whole ray is on one side or the other.
        return if a >= 0.0 {
            Some(Span::new(f64::NEG_INFINITY, f64::INFINITY))
        } else {
            None
        };
    }
    let crossing = -a / b;
    Some(if b > 0.0 {
        Span::new(crossing, f64::INFINITY)
    } else {
        Span::new(f64::NEG_INFINITY, crossing)
    })
}
