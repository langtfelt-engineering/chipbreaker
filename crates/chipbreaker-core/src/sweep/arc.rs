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

use crate::math::{OctNormal, Ray, Vec3};
use crate::spans::{Span, Spans};
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};
use crate::toolpath::ArcPlane;
use crate::transcendental as t;

use super::LinearMove;
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
    /// Tool tip height at the start.
    pub z: f64,
    /// Which plane the arc turns in.
    ///
    /// The closed form needs the arc's axis parallel to the tool's, so only
    /// [`ArcPlane::Xy`] collapses. A `G18` or `G19` arc turns about a horizontal
    /// axis and is sub-stepped, which is the honest answer rather than a
    /// worked-around one.
    pub plane: ArcPlane,
    /// Axial rise over the whole sweep, in millimetres.
    ///
    /// Zero for a plain arc, which is Case A′ and closed form. Non-zero makes it
    /// a helix, which is Case B′ and sub-stepped: the angular and axial terms
    /// couple and there is no collapse.
    pub rise: f64,
}

impl ArcMove {
    /// Tip position at parameter `s` in `[0, 1]`.
    #[must_use]
    pub fn at(&self, s: f64) -> Vec3 {
        let angle = self.start_angle + self.sweep * s;
        let (sin, cos) = t::sin_cos(angle);
        let [u, v, w] = self.plane.axes();
        let centre = self.center.to_array();
        let mut point = [0.0; 3];
        point[u] = centre[u] + self.radius * cos;
        point[v] = centre[v] + self.radius * sin;
        point[w] = self.z + self.rise * s;
        Vec3::from_array(point)
    }

    /// True if the axial term couples in, making this a helix.
    #[must_use]
    pub fn is_helix(&self) -> bool {
        self.rise.abs() > DEGENERATE
    }

    /// True if the closed form of Case A′ applies.
    ///
    /// Needs the arc's axis parallel to the tool's -- so `G17` only -- and no
    /// axial rise.
    #[must_use]
    pub fn is_level_xy(&self) -> bool {
        matches!(self.plane, ArcPlane::Xy) && !self.is_helix()
    }

    /// Length of the tip's path, in millimetres.
    ///
    /// The **helical** length, not the chord. A chord under-states it badly --
    /// 20.8% on a 2.4 radian sweep of a 10 mm radius with a 6 mm rise -- so a
    /// bound derived from the chord would claim an accuracy it does not have.
    #[must_use]
    pub fn path_length(&self) -> f64 {
        t::hypot(self.radius * self.sweep, self.rise)
    }

    /// Worst distance from any point of the true path to the nearest of `steps`
    /// evenly spaced samples of it.
    ///
    /// Between consecutive samples the tip travels a helical arc of angular
    /// extent `delta = |sweep| / N` and axial rise `h = |rise| / N`. By symmetry
    /// the farthest point is the midpoint, at angular `delta/2` and axial `h/2`
    /// from each end, so
    ///
    /// ```text
    /// deviation(N) = sqrt( (2 R sin(delta/4))^2 + (h/2)^2 )
    /// ```
    ///
    /// Exact, not an estimate. The tool translates rigidly along the path, so
    /// the tip's deviation is the swept volume's deviation.
    ///
    /// # Panics
    /// Panics if `steps` is zero.
    #[must_use]
    pub fn deviation_bound(&self, steps: u32) -> f64 {
        assert!(steps > 0, "a sub-stepped sweep needs at least one step");
        let n = f64::from(steps);
        let angular = self.sweep.abs() / n;
        let axial = self.rise.abs() / n;
        t::hypot(2.0 * self.radius * t::sin(angular / 4.0), axial / 2.0)
    }

    /// Worst distance from the true path to the chord polyline through `steps`
    /// evenly spaced points of it.
    ///
    /// **Not the same quantity as [`Self::deviation_bound`]**, and confusing the
    /// two would be easy. That one measures a *stepped* approximation, where the
    /// tool sits still at each sample and the path between samples is not
    /// represented at all; the worst point is the midpoint of the gap and the
    /// error is half the step. This one measures a *chord*, which does traverse
    /// the gap, and the worst point is the arc's bulge away from its own chord:
    ///
    /// ```text
    /// sagitta(N) = R * (1 - cos(delta/2)),   delta = |sweep| / N
    /// ```
    ///
    /// The axial term does not appear, and that is not an omission. A chord
    /// between two points of a helix interpolates the axial coordinate linearly
    /// in the same parameter the true helix does, and both ends agree, so the
    /// axial component of a chord is **exact**. Only the turn is approximated.
    ///
    /// Consequently a chord is far more accurate than a sub-step at the same
    /// count -- `R(1 - cos(d/2))` is `O(d^2)` where `2R sin(d/4)` is `O(d)` --
    /// which is why linearising and sub-stepping need different step counts to
    /// reach the same tolerance.
    ///
    /// # Panics
    /// Panics if `steps` is zero.
    #[must_use]
    pub fn chord_deviation(&self, steps: u32) -> f64 {
        assert!(steps > 0, "a linearised arc needs at least one chord");
        let delta = self.sweep.abs() / f64::from(steps);
        self.radius * (1.0 - t::cos(delta / 2.0))
    }

    /// Chords needed to bring [`Self::chord_deviation`] under `tolerance`.
    ///
    /// Inverts the sagitta directly: `delta <= 2 * acos(1 - tol / R)`. When the
    /// tolerance is at least the radius, one chord per half turn already
    /// suffices and the formula saturates, which the clamp handles.
    ///
    /// # Panics
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn chords_for_error(&self, tolerance: f64) -> u32 {
        assert!(
            tolerance.is_finite() && tolerance > 0.0,
            "the linearisation tolerance must be a positive length, got {tolerance}"
        );
        let sweep = self.sweep.abs();
        if sweep <= DEGENERATE || self.radius <= DEGENERATE {
            return 1;
        }
        // `1 - tol/R` below -1 means any chord is inside tolerance; the `max`
        // keeps `acos` in its domain rather than letting a NaN through.
        let cosine = (1.0 - tolerance / self.radius).max(-1.0);
        let delta = 2.0 * t::acos(cosine);
        if delta <= 0.0 {
            return super::reference::MAX_SUBSTEPS;
        }
        let wanted = sweep / delta;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped into u32 immediately below"
        )]
        if wanted.is_finite() && wanted < f64::from(super::reference::MAX_SUBSTEPS) {
            (wanted.ceil() as u32).max(1)
        } else {
            super::reference::MAX_SUBSTEPS
        }
    }

    /// Replaces the arc with the chord polyline a CAM post would emit.
    ///
    /// This is what `--no-arc-native` produces, and what the native path is
    /// differential-tested against. It is not a fallback: every controller on
    /// the floor accepts `G2`/`G3`, but a great many posts linearise anyway, so
    /// the linearised result is the one a customer is most likely to be
    /// comparing against.
    ///
    /// Endpoints come from [`Self::at`], so the chain starts and ends exactly
    /// where the arc does and the joins are shared points -- no gaps for a
    /// rounding to open up in.
    ///
    /// # Panics
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn linearise(&self, tolerance: f64) -> Vec<LinearMove> {
        let steps = self.chords_for_error(tolerance);
        let mut out = Vec::with_capacity(steps as usize);
        let mut previous = self.at(0.0);
        for k in 1..=steps {
            let point = self.at(f64::from(k) / f64::from(steps));
            out.push(LinearMove {
                start: previous,
                end: point,
            });
            previous = point;
        }
        out
    }

    /// Steps needed to bring [`Self::deviation_bound`] under `tolerance`, and
    /// the bound actually achieved.
    ///
    /// Chosen from `L / (2N)` with `L` the helical path length, which bounds the
    /// exact form because `sin(x) <= x`. So the choice is conservative and the
    /// number reported alongside is tight -- never a step count on its own.
    ///
    /// # Panics
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn substeps_for_error(&self, tolerance: f64) -> (u32, f64) {
        assert!(
            tolerance.is_finite() && tolerance > 0.0,
            "the sweep tolerance must be a positive length, got {tolerance}"
        );
        let length = self.path_length();
        if length <= 0.0 {
            return (1, 0.0);
        }
        let wanted = length / (2.0 * tolerance);
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped into u32 immediately below"
        )]
        let steps = if wanted.is_finite() && wanted < f64::from(super::reference::MAX_SUBSTEPS) {
            (wanted.ceil() as u32).max(1)
        } else {
            super::reference::MAX_SUBSTEPS
        };
        (steps, self.deviation_bound(steps))
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
        // Built from the sampled extremes of the path rather than from a formula
        // per plane: for a non-`G17` arc the tool's own axis is not the arc's,
        // so the reach is not symmetric and a formula would have to special-case
        // each plane. Sampling the path is loose only in that it may be slightly
        // large, which costs rejection rate and never material.
        let mut lo = self.at(0.0).to_array();
        let mut hi = lo;
        let samples = 64;
        for k in 0..=samples {
            let p = self.at(f64::from(k) / f64::from(samples)).to_array();
            for axis in 0..3 {
                lo[axis] = lo[axis].min(p[axis]);
                hi[axis] = hi[axis].max(p[axis]);
            }
        }
        let r = profile.max_radius();
        crate::math::Aabb3::from_min_max(
            Vec3::new(lo[0] - r, lo[1] - r, lo[2]),
            Vec3::new(hi[0] + r, hi[1] + r, hi[2] + profile.total_length()),
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
    if !arc.is_level_xy() {
        // Either the axial term couples in, or the arc turns about a horizontal
        // axis so the collapse -- which needs the arc axis parallel to the
        // tool's -- does not apply. Either way the caller sub-steps.
        return false;
    }
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
    middle_spans_into(profile, arc, ray, along_axis, scratch, out, stats);

    // The normals, stamped once at the end rather than carried through.
    //
    // Both branches assemble their answer out of mapped casts and boolean
    // operations — an annulus is a disc minus a disc, clipped by a wedge — and
    // threading a normal through each of those steps means getting a sign right
    // in four places instead of one. Every bound of the result is a point on the
    // swept surface, and the swept surface of a **level** arc is the tool's own
    // surface at the nearest position on the arc circle. So the world position of
    // the bound determines the normal on its own, and stating it once here is
    // both shorter and harder to get wrong.
    //
    // This holds because the arc is level: the tool's height is constant, so the
    // nearest arc position is found in plan alone. A helical arc has no such
    // collapse, and is sub-stepped instead — where each sub-step is a linear
    // motion whose own case supplies the normal.
    out.set_normals_with(|t| middle_normal(profile, arc, ray.at(t)));
}

/// The wedge-restricted annular middle, without normals.
fn middle_spans_into(
    profile: &Profile,
    arc: &ArcMove,
    ray: &Ray,
    along_axis: bool,
    scratch: &mut RaycastScratch,
    out: &mut Spans,
    stats: &mut RaycastStats,
) {
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

/// The outward normal of the swept surface at a world point on it.
///
/// The tool that produced this point sits at the nearest position on the arc
/// circle, found in plan because the arc is level. Subtracting that position puts
/// the point in the tool's own frame, where the profile answers directly.
fn middle_normal(profile: &Profile, arc: &ArcMove, p: Vec3) -> OctNormal {
    let dx = p.x - arc.center.x;
    let dy = p.y - arc.center.y;
    let d = t::hypot(dx, dy);
    // On the arc's own centre every bearing is equidistant. The point is then
    // inside the tool wherever it is placed, so no bound of the result can land
    // here; answering with a fixed bearing keeps the function total.
    let (ux, uy) = if d > 0.0 {
        (dx / d, dy / d)
    } else {
        (1.0, 0.0)
    };
    let local = Vec3::new(
        p.x - (arc.center.x + arc.radius * ux),
        p.y - (arc.center.y + arc.radius * uy),
        p.z - arc.z,
    );
    crate::tool::surface_normal(profile, local).map_or(OctNormal::PLACEHOLDER, OctNormal::encode)
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
