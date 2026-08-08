// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Ray against tool solid, returning the intervals of the ray that lie inside.
//!
//! # Why the stationary case lives here, apart from the sweep
//!
//! A dexel field is built by casting three orthogonal bundles of rays at the
//! stock and subtracting what the tool occupies. Cutting does the same against a
//! *moving* tool, which is the same problem with the polynomial degree raised.
//! Getting the stationary case right — including the tangencies, the joints
//! between elements, and the torus — is the part that can be done now, and doing
//! it separately means the sweep inherits a tested intersection routine rather
//! than growing one of its own.
//!
//! # The surfaces, and the degree each one costs
//!
//! | element | revolved | equation in `t` |
//! |---|---|---|
//! | segment, `dz != 0` | cone or cylinder | quadratic |
//! | segment, `dz == 0` | annular disc | linear |
//! | arc centred on the axis | sphere | quadratic |
//! | arc centred off the axis | **torus** | **quartic** |
//! | top cap | disc | linear |
//!
//! # How the intervals are decided
//!
//! Not by parity. Collecting crossings and pairing them off assumes they
//! alternate, which is true of exact arithmetic and not of `f64`: a ray that
//! grazes a corner produces two crossings that may round to one, and a single
//! lost crossing turns every interval after it inside out. That failure is
//! silent, and in a field it becomes a semi-infinite column of stock removed from the
//! middle of a part.
//!
//! So the crossings are used only as *candidate boundaries*. Each interval
//! between consecutive candidates is classified by testing its midpoint against
//! [`Profile::contains_rz`], which is an independent computation. A missed
//! crossing then merges two spans — visibly wrong, and bounded — instead of
//! inverting everything downstream of it. It also means the answer cannot
//! disagree with the containment predicate, which is the property a field leans
//! on when it reconciles three ray bundles that all describe the same solid.

use crate::eps::{EPS_LENGTH, EPS_SPAN_MIN, eps_tangent};
use crate::math::{OctNormal, Ray, Vec3};
use crate::roots::{solve_quadratic, solve_quartic};
use crate::spans::{Span, Spans};
use crate::transcendental as t;

use super::Tool;
use super::normal::surface_normal;
use super::profile::{Profile, ProfileElement};

/// Angular slack, in radians, when deciding whether a hit lies on an arc.
///
/// Without it a ray passing exactly through the joint between two elements
/// belongs to neither and the solid leaks there. The value is generous because
/// counting a crossing twice is harmless — duplicates collapse — while missing
/// one is not.
const ARC_ANGLE_SLACK: f64 = 1.0e-9;

/// What a raycast did, for the parity and convergence tests.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RaycastStats {
    /// Rays cast.
    pub rays: u64,
    /// Candidate crossings found, before collapsing.
    pub crossings: u64,
    /// Crossings discarded because another lay within the tangency tolerance.
    pub collapsed: u64,
    /// Spans returned.
    pub spans: u64,
    /// Spans dropped for being shorter than [`EPS_SPAN_MIN`]: grazes, which are
    /// contact without material removal.
    pub grazes: u64,
    /// Roots the solver returned with multiplicity above one: the ray met that
    /// surface tangentially rather than crossing it.
    pub tangencies: u64,
    /// Times the torus branch was taken, needing a quartic solve.
    ///
    /// Counted because arcs were predicted to drive this rate up:
    /// the middle piece of a swept arc was expected to need an *offset* profile,
    /// the tool's chain translated by the arc radius, which would put arcs into
    /// even a flat end mill's silhouette. It does not, because the three bundles
    /// split the problem and no profile is ever constructed -- so this counter
    /// should depend on the **tool** alone and be flat in the motion kind.
    ///
    /// A prediction that cheap to check is worth a counter.
    pub quartics: u64,
}

impl RaycastStats {
    /// Adds another set of counts.
    pub fn merge(&mut self, other: &Self) {
        self.rays += other.rays;
        self.crossings += other.crossings;
        self.collapsed += other.collapsed;
        self.spans += other.spans;
        self.grazes += other.grazes;
        self.tangencies += other.tangencies;
        self.quartics += other.quartics;
    }
}

/// Reusable working storage, so that a raycast in an inner loop allocates
/// nothing.
#[derive(Debug, Clone, Default)]
pub struct RaycastScratch {
    candidates: Vec<f64>,
}

impl RaycastScratch {
    /// Empty scratch.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Scratch sized for a profile of `elements` elements.
    #[must_use]
    pub fn with_capacity(elements: usize) -> Self {
        Self {
            // Four roots per element at worst, plus the cap, plus the ray origin.
            candidates: Vec::with_capacity(4 * elements + 2),
        }
    }
}

/// Pushes `value` if it is a finite parameter at or beyond the ray origin.
fn push_candidate(out: &mut Vec<f64>, value: f64) {
    if value.is_finite() && value >= 0.0 {
        out.push(value);
    }
}

/// Crossings of the ray with the surface swept by one profile element.
fn element_crossings(
    element: &ProfileElement,
    ray: &Ray,
    out: &mut Vec<f64>,
    stats: &mut RaycastStats,
) {
    let o = ray.origin;
    let d = ray.direction;
    // The radial quadratic, r(t)^2 = x(t)^2 + y(t)^2.
    let e2 = d.x * d.x + d.y * d.y;
    let e1 = 2.0 * (o.x * d.x + o.y * d.y);
    let e0 = o.x * o.x + o.y * o.y;

    match element {
        ProfileElement::Segment { start, end } => {
            let dz = end.y - start.y;
            let (z_lo, z_hi) = element.z_range();
            if dz.abs() <= EPS_LENGTH {
                // Horizontal: an annular disc in the plane z = start.y.
                if d.z == 0.0 {
                    return;
                }
                let hit = (start.y - o.z) / d.z;
                let (r_lo, r_hi) = element.radius_range();
                let radius = radius_at(ray, hit);
                if radius >= r_lo - EPS_LENGTH && radius <= r_hi + EPS_LENGTH {
                    push_candidate(out, hit);
                }
                return;
            }

            // A cone, or a cylinder when the slope is zero:
            //   x^2 + y^2 = (line + slope * z)^2
            let slope = (end.x - start.x) / dz;
            let line = start.x - start.y * slope;
            let u0 = line + slope * o.z;
            let u1 = slope * d.z;

            let roots = solve_quadratic(e2 - u1 * u1, e1 - 2.0 * u0 * u1, e0 - u0 * u0);
            for (hit, multiplicity) in roots.iter() {
                if multiplicity > 1 {
                    stats.tangencies += 1;
                }
                let z = o.z + hit * d.z;
                if z < z_lo - EPS_LENGTH || z > z_hi + EPS_LENGTH {
                    continue;
                }
                // Squaring admits the mirrored cone, where the radius the line
                // predicts is negative. Those points are not on the solid.
                if line + slope * z < -EPS_LENGTH {
                    continue;
                }
                push_candidate(out, hit);
            }
        }
        ProfileElement::Arc { center, .. } => {
            let rho = element.radius().unwrap_or(0.0);
            let major = center.x;
            let g0 = o.z - center.y;
            let g1 = d.z;

            // `major.abs()`, not `major`. A barrel cutter's arc is centred at
            // *negative* radius — a 12 mm barrel on a 60 mm arc has its centre
            // at r = -54 — and testing the signed value sent every barrel down
            // the sphere branch below, which is a different surface entirely.
            if major.abs() <= EPS_LENGTH {
                // Centre on the axis: a sphere, not a torus. Feeding a zero
                // major radius to the quartic below would turn every crossing
                // into a double root of a perfect square.
                let roots =
                    solve_quadratic(e2 + g1 * g1, e1 + 2.0 * g0 * g1, e0 + g0 * g0 - rho * rho);
                for (hit, multiplicity) in roots.iter() {
                    if multiplicity > 1 {
                        stats.tangencies += 1;
                    }
                    if on_arc(element, ray, hit) {
                        push_candidate(out, hit);
                    }
                }
                return;
            }

            // A torus of major radius `major` and minor radius `rho`:
            //   (x^2 + y^2 + (z - cz)^2 + R^2 - rho^2)^2 = 4 R^2 (x^2 + y^2)
            let s2 = e2 + g1 * g1;
            let s1 = e1 + 2.0 * g0 * g1;
            let s0 = e0 + g0 * g0 + major * major - rho * rho;
            let four_r2 = 4.0 * major * major;

            stats.quartics += 1;
            let roots = solve_quartic(
                s2 * s2,
                2.0 * s2 * s1,
                s1 * s1 + 2.0 * s2 * s0 - four_r2 * e2,
                2.0 * s1 * s0 - four_r2 * e1,
                s0 * s0 - four_r2 * e0,
            );
            for (hit, multiplicity) in roots.iter() {
                if multiplicity > 1 {
                    stats.tangencies += 1;
                }
                if on_arc(element, ray, hit) {
                    push_candidate(out, hit);
                }
            }
        }
    }
}

/// Distance from the axis at ray parameter `hit`.
fn radius_at(ray: &Ray, hit: f64) -> f64 {
    let p = ray.at(hit);
    t::hypot(p.x, p.y)
}

/// True if the ray point at `hit` lies on the surface swept by the arc, within
/// the arc's own sweep.
///
/// # Two things have to be rejected here
///
/// **The mirror surface.** The torus equation is derived by squaring
/// `(r - cr)^2 + (z - cz)^2 = rho^2` to clear the `sqrt(x^2 + y^2)`, and squaring
/// admits the solutions of the equation with `r` negated as well. Those points
/// are on a torus reflected through the axis, and they are not on the tool. The
/// residual test below rejects them: it evaluates the *unsquared* equation, which
/// they do not satisfy.
///
/// **Hits outside the sweep.** The surface is generated by an arc, not a whole
/// circle, so a point can satisfy the equation and still lie on the part of the
/// circle the profile never reaches.
fn on_arc(element: &ProfileElement, ray: &Ray, hit: f64) -> bool {
    let ProfileElement::Arc { center, .. } = element else {
        return false;
    };
    let rho = element.radius().unwrap_or(0.0);
    let p = ray.at(hit);
    let radius = t::hypot(p.x, p.y);
    let dr = radius - center.x;
    let dz = p.z - center.y;

    // Scale the residual tolerance by the geometry: the residual is a squared
    // length, and a hit accurate to `d` in position leaves a residual near
    // `2 rho d`. The tangency floor is the accuracy actually available.
    let scale = rho.max(center.x.abs()).max(radius);
    let tolerance = 8.0 * rho.max(1.0) * eps_tangent(scale);
    if (dr * dr + dz * dz - rho * rho).abs() > tolerance {
        return false;
    }

    element.contains_angle(t::atan2(dz, dr), ARC_ANGLE_SLACK)
}

/// The outward tool normal at `p`, quantised for storage in a span.
///
/// Falls back to the placeholder only when the profile has no surface at all,
/// which [`Profile::new`] rejects — so in practice this always answers.
fn encode_normal(profile: &Profile, p: Vec3) -> OctNormal {
    surface_normal(profile, p).map_or(OctNormal::PLACEHOLDER, OctNormal::encode)
}

impl Profile {
    /// The intervals of the ray, in `t`, that lie inside the solid.
    ///
    /// The ray's direction must be normalised — [`Ray::new_normalized`]
    /// guarantees it — so that `t` is a distance and the tangency tolerance is a
    /// length rather than a parameter.
    ///
    /// Allocates. Use [`Profile::intersect_ray_into`] in an inner loop.
    #[must_use]
    pub fn intersect_ray(&self, ray: &Ray) -> Spans {
        let mut scratch = RaycastScratch::with_capacity(self.len());
        let mut out = Spans::new();
        let mut stats = RaycastStats::default();
        self.intersect_ray_into(ray, &mut scratch, &mut out, &mut stats);
        out
    }

    /// As [`Profile::intersect_ray`], reusing the caller's storage and recording
    /// what happened.
    ///
    /// `out` is cleared first. `stats` is accumulated into, not reset, so a
    /// whole bundle of rays can share one.
    pub fn intersect_ray_into(
        &self,
        ray: &Ray,
        scratch: &mut RaycastScratch,
        out: &mut Spans,
        stats: &mut RaycastStats,
    ) {
        out.clear();
        stats.rays += 1;

        // Decided before any candidate is pushed, because the answer changes the
        // meaning of a span that begins at `t = 0`.
        let started_outside = !self.contains_xyz(ray.origin);

        let candidates = &mut scratch.candidates;
        candidates.clear();

        // The ray origin bounds the first interval; without it a ray that starts
        // inside the tool would have its first span begin at the far wall.
        candidates.push(0.0);

        for e in self.elements() {
            element_crossings(&e.element, ray, candidates, stats);
        }

        // The top cap: the disc that closes the solid.
        let top = self.top();
        if ray.direction.z != 0.0 {
            let hit = (top.y - ray.origin.z) / ray.direction.z;
            if radius_at(ray, hit) <= top.x + EPS_LENGTH {
                push_candidate(candidates, hit);
            }
        }

        stats.crossings += candidates.len() as u64 - 1;
        candidates.sort_by(f64::total_cmp);

        // Collapse crossings that agree to within the tangency tolerance. Two
        // roots that close are one grazing contact, and keeping both would
        // produce a span narrower than the arithmetic that found it.
        let scale = self.total_length().max(self.max_radius());
        let mut write = 0usize;
        for read in 0..candidates.len() {
            let value = candidates[read];
            if write > 0 {
                let previous = candidates[write - 1];
                if value - previous <= eps_tangent(scale.max(value.abs())) {
                    stats.collapsed += 1;
                    continue;
                }
            }
            candidates[write] = value;
            write += 1;
        }
        candidates.truncate(write);

        // Classify each interval by its midpoint. See the module header for why
        // this is not done by parity.
        for pair in candidates.windows(2) {
            let (lo, hi) = (pair[0], pair[1]);
            let middle = 0.5 * (lo + hi);
            let p = ray.at(middle);
            if !self.contains_rz(t::hypot(p.x, p.y), p.z) {
                continue;
            }
            if hi - lo < EPS_SPAN_MIN {
                stats.grazes += 1;
                continue;
            }
            // The outward tool normal at each end, analytically. Both endpoints
            // lie on the surface by construction -- they are roots of an
            // element's equation -- so this is a lookup of which element, not a
            // reconstruction from neighbours.
            //
            // The exception is `lo == 0.0` when the ray *starts* inside the
            // tool: that bound is the ray's origin rather than a surface, and
            // there is no normal to record. It is left as the placeholder, which
            // is honest about knowing nothing, and no dexel ray meets it since
            // every bundle originates outside the stock.
            let n0 = if lo == 0.0 && !started_outside {
                OctNormal::PLACEHOLDER
            } else {
                encode_normal(self, ray.at(lo))
            };
            let n1 = encode_normal(self, ray.at(hi));
            out.push_merge(Span::with_normals(lo, hi, n0, n1));
        }
        stats.spans += out.len() as u64;
        out.debug_check_invariant();
    }

    /// True if the point, in profile coordinates, is inside the solid.
    ///
    /// Convenience wrapper over [`Profile::contains_rz`] for callers holding a
    /// three-dimensional point.
    #[must_use]
    pub fn contains_xyz(&self, p: Vec3) -> bool {
        self.contains_rz(t::hypot(p.x, p.y), p.z)
    }
}

impl Tool {
    /// The intervals of the ray, in `t`, that lie inside the tool.
    #[must_use]
    pub fn intersect_ray(&self, ray: &Ray) -> Spans {
        self.profile().intersect_ray(ray)
    }

    /// As [`Tool::intersect_ray`], reusing the caller's storage.
    pub fn intersect_ray_into(
        &self,
        ray: &Ray,
        scratch: &mut RaycastScratch,
        out: &mut Spans,
        stats: &mut RaycastStats,
    ) {
        self.profile().intersect_ray_into(ray, scratch, out, stats);
    }
}
