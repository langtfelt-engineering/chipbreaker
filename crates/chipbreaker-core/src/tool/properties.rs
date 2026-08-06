// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Closed-form volume, area, bounds, silhouette, and contact queries.
//!
//! # Why these are analytic and not measured from a mesh
//!
//! Every one of these could be estimated by tessellating the tool and summing
//! over triangles. None of them is, and the reason is that they are the
//! *reference* against which the tessellation is checked. A convergence test
//! that compares a mesh against a finer mesh proves only that the mesher is
//! self-consistent; comparing it against a closed form proves it is right.
//! Section 9's tessellation error bound is stated in terms of the numbers
//! computed here, so these must be independent of it.
//!
//! # Where the volume formula comes from
//!
//! Pappus's theorem gives the volume of a solid of revolution as `2 * PI` times
//! the centroid radius times the area — but the centroid of the generating
//! region is itself an integral, so that is a restatement rather than a formula.
//! Green's theorem turns it into one. Revolving a plane region `R` about the
//! axis gives
//!
//! ```text
//! V = 2 PI * integral over R of r dA = PI * contour integral of r^2 dz
//! ```
//!
//! taken counter-clockwise around `R`'s boundary. The boundary here is the
//! profile, then the top cap, then the axis — and the last two contribute
//! nothing at all: the cap is horizontal so `dz` is zero along it, and the axis
//! has `r = 0`. So the entire volume of the tool is a sum of one integral per
//! profile element, each of which has an elementary antiderivative.
//!
//! That is why the profile does not store its caps. They cannot affect the
//! volume, and giving them no representation means they cannot be forgotten,
//! duplicated, or wrongly oriented either.

use crate::eps::EPS_LENGTH;
use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Vec2, Vec3};
use crate::transcendental as t;

use super::Tool;
use super::profile::{ElementRole, Profile, ProfileElement};

use core::f64::consts::PI;

/// The smallest cylinder about the axis that contains the whole tool.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingCylinder {
    /// Radius of the cylinder: the tool's largest radius.
    pub radius: f64,
    /// Lowest `z`. Zero for a valid tool, since the tip is the origin.
    pub z_min: f64,
    /// Highest `z`: the top of the tool.
    pub z_max: f64,
}

impl BoundingCylinder {
    /// Volume of the cylinder itself, which bounds the tool's volume above.
    #[must_use]
    pub fn volume(&self) -> f64 {
        PI * self.radius * self.radius * (self.z_max - self.z_min)
    }
}

impl Hashable for BoundingCylinder {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("BoundingCylinder");
        h.f64_slice(&[self.radius, self.z_min, self.z_max]);
        h.end();
    }
}

/// A point on the tool's surface, and what part of the tool it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    /// Index of the profile element the point lies on, or `u32::MAX` for the
    /// top cap, which is not an element.
    pub element: u32,
    /// What that part of the tool is for. Decides whether contact is a cut, a
    /// rub, or a crash.
    pub role: ElementRole,
    /// Distance from the query point to the surface. Never negative — use
    /// [`Tool::contains_point`] to tell inside from outside.
    pub distance: f64,
    /// The closest point, in profile coordinates `(r, z)`.
    pub closest: Vec2,
}

/// Index reserved for the top cap, which closes the solid but is not a profile
/// element.
pub const TOP_CAP: u32 = u32::MAX;

impl Hashable for Contact {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Contact");
        h.u64(u64::from(self.element));
        h.add(&self.role);
        h.f64_slice(&[self.distance, self.closest.x, self.closest.y]);
        h.end();
    }
}

/// `integral of r^2 dz` along one profile element.
///
/// Multiplied by `PI` this is the element's contribution to the volume; see the
/// module header for why the caps contribute nothing and so are absent.
fn volume_integral(element: &ProfileElement) -> f64 {
    match element {
        ProfileElement::Segment { start, end } => {
            let dr = end.x - start.x;
            let dz = end.y - start.y;
            // integral over u in [0,1] of (r0 + u dr)^2 dz
            dz * (start.x * start.x + start.x * dr + dr * dr / 3.0)
        }
        ProfileElement::Arc { center, .. } => {
            let radius = element.radius().unwrap_or(0.0);
            let Some((a0, _, sweep)) = element.angles() else {
                return 0.0;
            };
            arc_volume_term(center.x, radius, a0 + sweep) - arc_volume_term(center.x, radius, a0)
        }
    }
}

/// Antiderivative of `r^2 dz` along a circle of radius `rho` centred at radius
/// `cr`, evaluated at angle `theta`.
///
/// With `r = cr + rho cos t` and `dz = rho cos t dt`, the integrand expands to
/// `rho (cr^2 cos t + 2 cr rho cos^2 t + rho^2 cos^3 t)`, whose three terms
/// integrate to `sin t`, `t/2 + sin 2t / 4`, and `sin t - sin^3 t / 3`.
fn arc_volume_term(cr: f64, rho: f64, theta: f64) -> f64 {
    let (sin, _) = t::sin_cos(theta);
    let sin_two = t::sin(2.0 * theta);
    rho * (cr * cr * sin
        + 2.0 * cr * rho * (0.5 * theta + 0.25 * sin_two)
        + rho * rho * (sin - sin * sin * sin / 3.0))
}

/// Lateral surface area swept by one profile element, by Pappus.
fn area_integral(element: &ProfileElement) -> f64 {
    match element {
        ProfileElement::Segment { start, end } => {
            // A frustum: mean radius times slant length, times 2 PI.
            PI * (start.x + end.x) * (*end - *start).length()
        }
        ProfileElement::Arc { center, .. } => {
            let radius = element.radius().unwrap_or(0.0);
            let Some((a0, a1, sweep)) = element.angles() else {
                return 0.0;
            };
            // 2 PI rho * integral of (cr + rho cos t) |dt| over the sweep.
            let (sin0, _) = t::sin_cos(a0);
            let (sin1, _) = t::sin_cos(a1);
            2.0 * PI * radius * (center.x * sweep.abs() + radius * sweep.signum() * (sin1 - sin0))
        }
    }
}

/// The radii at which `element` crosses the horizontal line `z`.
///
/// Writes up to two values into `out` and returns how many. Used both by the
/// silhouette and by the containment test, so that the two cannot disagree about
/// where the boundary is.
fn crossings_at(element: &ProfileElement, z: f64, out: &mut [f64; 2]) -> usize {
    match element {
        ProfileElement::Segment { start, end } => {
            let dz = end.y - start.y;
            if dz.abs() <= EPS_LENGTH {
                // Horizontal: parallel to the line, so not a transversal
                // crossing however close it lies.
                return 0;
            }
            let u = (z - start.y) / dz;
            // Half-open, so a vertex shared by two segments is counted once.
            if (0.0..1.0).contains(&u) {
                out[0] = start.x + u * (end.x - start.x);
                1
            } else {
                0
            }
        }
        ProfileElement::Arc { center, .. } => {
            let radius = element.radius().unwrap_or(0.0);
            let dz = z - center.y;
            let discriminant = radius * radius - dz * dz;
            if discriminant <= 0.0 {
                // Below or above the circle, or exactly tangent to it. A tangent
                // touch is not a crossing: the boundary does not pass through.
                return 0;
            }
            let dr = discriminant.sqrt();
            let mut n = 0;
            for candidate in [center.x - dr, center.x + dr] {
                let angle = t::atan2(dz, candidate - center.x);
                if element.contains_angle(angle, 0.0) {
                    out[n] = candidate;
                    n += 1;
                }
            }
            n
        }
    }
}

/// Distance from `p` to an element, in the `(r, z)` half-plane, and the closest
/// point on it.
fn distance_to(element: &ProfileElement, p: Vec2) -> (f64, Vec2) {
    match element {
        ProfileElement::Segment { start, end } => {
            let along = *end - *start;
            let length_squared = along.length_squared();
            let u = if length_squared <= 0.0 {
                0.0
            } else {
                ((p - *start).dot(along) / length_squared).clamp(0.0, 1.0)
            };
            let closest = *start + along * u;
            ((p - closest).length(), closest)
        }
        ProfileElement::Arc {
            start, end, center, ..
        } => {
            let radius = element.radius().unwrap_or(0.0);
            let offset = p - *center;
            let angle = t::atan2(offset.y, offset.x);
            if element.contains_angle(angle, 0.0) && offset.length() > 0.0 {
                let closest = *center + offset * (radius / offset.length());
                ((p - closest).length(), closest)
            } else {
                // Outside the sweep: the nearest point is an endpoint.
                let da = (p - *start).length();
                let db = (p - *end).length();
                if da <= db { (da, *start) } else { (db, *end) }
            }
        }
    }
}

impl Profile {
    /// Volume of the solid this profile generates.
    ///
    /// Exact for segments and arcs, to the accuracy of the arithmetic — not a
    /// tessellated approximation. See the module header for the derivation.
    #[must_use]
    pub fn volume(&self) -> f64 {
        PI * self
            .elements()
            .iter()
            .map(|e| volume_integral(&e.element))
            .sum::<f64>()
    }

    /// Total surface area: the revolved profile plus the disc that closes the
    /// top.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let lateral: f64 = self
            .elements()
            .iter()
            .map(|e| area_integral(&e.element))
            .sum();
        let top = self.top().x;
        lateral + PI * top * top
    }

    /// Volume contributed by elements of one role.
    ///
    /// Each element's contribution is well defined on its own — the volume is a
    /// sum over elements with no cross terms — so this partitions the tool
    /// exactly. Note that a role's share can be negative where the profile moves
    /// downward, as it does across the underside of an undercut.
    #[must_use]
    pub fn volume_of_role(&self, role: ElementRole) -> f64 {
        PI * self
            .elements()
            .iter()
            .filter(|e| e.role == role)
            .map(|e| volume_integral(&e.element))
            .sum::<f64>()
    }

    /// The smallest cylinder about the axis containing the solid.
    #[must_use]
    pub fn bounding_cylinder(&self) -> BoundingCylinder {
        let mut z_min = 0.0f64;
        let mut z_max = 0.0f64;
        for e in self.elements() {
            let (lo, hi) = e.element.z_range();
            z_min = z_min.min(lo);
            z_max = z_max.max(hi);
        }
        BoundingCylinder {
            radius: self.max_radius(),
            z_min,
            z_max,
        }
    }

    /// The tool's radius at height `z`: its silhouette seen from the side.
    ///
    /// Returns zero above the top of the tool and below the tip. At a height
    /// where the boundary is horizontal — the flat bottom of an end mill, the
    /// underside of an undercut — the value is the largest radius the boundary
    /// reaches there, which is what a silhouette shows.
    #[must_use]
    pub fn silhouette_radius(&self, z: f64) -> f64 {
        let mut best = 0.0f64;
        let mut out = [0.0f64; 2];
        for e in self.elements() {
            let n = crossings_at(&e.element, z, &mut out);
            for &r in &out[..n] {
                best = best.max(r);
            }
            // A horizontal boundary crosses no line but is still silhouette.
            let (lo, hi) = e.element.z_range();
            if (hi - lo) <= EPS_LENGTH && (z - lo).abs() <= EPS_LENGTH {
                let (_, r_hi) = e.element.radius_range();
                best = best.max(r_hi);
            }
        }
        if z > self.total_length() + EPS_LENGTH || z < -EPS_LENGTH {
            return 0.0;
        }
        best
    }

    /// True if `(r, z)` lies inside the generated solid.
    ///
    /// Crossing count along a ray leaving the point in `+r`. The axis and the
    /// top cap cannot be crossed by such a ray — the axis is at `r = 0`, never
    /// to the right of a point with `r >= 0`, and the cap is parallel to it — so
    /// only the profile elements are counted, and an odd count means inside.
    ///
    /// Points exactly on the boundary are not classified either way; the caller
    /// that needs an answer there wants [`Profile::nearest_surface`] and a
    /// tolerance of its own choosing.
    #[must_use]
    pub fn contains_rz(&self, r: f64, z: f64) -> bool {
        if r < 0.0 || z < 0.0 || z > self.total_length() {
            return false;
        }
        let mut count = 0usize;
        let mut out = [0.0f64; 2];
        for e in self.elements() {
            let n = crossings_at(&e.element, z, &mut out);
            for &crossing in &out[..n] {
                if crossing > r {
                    count += 1;
                }
            }
        }
        count % 2 == 1
    }

    /// The nearest point of the tool's surface to `(r, z)`, and what part of the
    /// tool it belongs to.
    ///
    /// The distance is measured in the `(r, z)` half-plane, which is the true
    /// three-dimensional distance: the surface is a surface of revolution, so
    /// the query point's angle about the axis cannot matter.
    ///
    /// # Panics
    /// Never — a validated profile always has at least one element.
    #[must_use]
    pub fn nearest_surface(&self, r: f64, z: f64) -> Contact {
        let p = Vec2::new(r, z);
        let mut best = Contact {
            element: TOP_CAP,
            role: self
                .elements()
                .last()
                .map_or(ElementRole::NonCutting, |e| e.role),
            distance: f64::INFINITY,
            closest: self.top(),
        };

        for (index, e) in self.elements().iter().enumerate() {
            let (distance, closest) = distance_to(&e.element, p);
            if distance < best.distance {
                best = Contact {
                    element: u32::try_from(index).unwrap_or(u32::MAX - 1),
                    role: e.role,
                    distance,
                    closest,
                };
            }
        }

        // The top cap: the horizontal disc from the axis to the last point.
        let top = self.top();
        let cap = Vec2::new(r.clamp(0.0, top.x), top.y);
        let cap_distance = (p - cap).length();
        if cap_distance < best.distance {
            best = Contact {
                element: TOP_CAP,
                role: self
                    .elements()
                    .last()
                    .map_or(ElementRole::NonCutting, |e| e.role),
                distance: cap_distance,
                closest: cap,
            };
        }
        best
    }
}

impl Tool {
    /// Volume of the tool solid.
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.profile().volume()
    }

    /// Total surface area, including the disc that closes the top.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        self.profile().surface_area()
    }

    /// The smallest cylinder about the axis containing the tool.
    #[must_use]
    pub fn bounding_cylinder(&self) -> BoundingCylinder {
        self.profile().bounding_cylinder()
    }

    /// The tool's radius at height `z`.
    #[must_use]
    pub fn silhouette_radius(&self, z: f64) -> f64 {
        self.profile().silhouette_radius(z)
    }

    /// True if the point, in tool coordinates, is inside the solid.
    #[must_use]
    pub fn contains_point(&self, p: Vec3) -> bool {
        self.profile().contains_rz(t::hypot(p.x, p.y), p.z)
    }

    /// The nearest surface point to a point in tool coordinates, and what part
    /// of the tool it is.
    ///
    /// This is the query U8 asks of every contact it detects: the answer decides
    /// whether what happened was a cut, a rub, or a crash.
    #[must_use]
    pub fn nearest_surface(&self, p: Vec3) -> Contact {
        self.profile().nearest_surface(t::hypot(p.x, p.y), p.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool::catalog::{
        HolderStage, Shank, ball_end_mill, barrel_end_mill, bull_end_mill, drill, flat_end_mill,
    };

    /// Relative agreement, which is what matters for a volume spanning orders of
    /// magnitude between the tip and the shank.
    fn close(a: f64, b: f64, tolerance: f64) -> bool {
        (a - b).abs() <= tolerance * a.abs().max(b.abs()).max(1.0)
    }

    #[test]
    fn a_cylinder_has_the_volume_and_area_a_cylinder_has() {
        // A flat end mill whose shank matches its cutter is exactly a cylinder.
        let p = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
        let expected = PI * 9.0 * 50.0;
        assert!(close(p.volume(), expected, 1e-14), "{}", p.volume());

        // Side, plus both discs.
        let expected_area = 2.0 * PI * 3.0 * 50.0 + 2.0 * PI * 9.0;
        assert!(
            close(p.surface_area(), expected_area, 1e-14),
            "{}",
            p.surface_area()
        );
    }

    #[test]
    fn a_ball_nose_is_a_hemisphere_on_a_cylinder() {
        let radius = 4.0f64;
        let length = 50.0;
        let p =
            ball_end_mill(2.0 * radius, 25.0, &Shank::plain(2.0 * radius, length)).expect("valid");

        // Hemisphere below z = radius, cylinder above it, plus the top disc.
        let expected = 2.0 / 3.0 * PI * radius.powi(3) + PI * radius * radius * (length - radius);
        assert!(close(p.volume(), expected, 1e-14), "{}", p.volume());

        let expected_area = 2.0 * PI * radius * radius
            + 2.0 * PI * radius * (length - radius)
            + PI * radius * radius;
        assert!(
            close(p.surface_area(), expected_area, 1e-14),
            "{}",
            p.surface_area()
        );
    }

    #[test]
    fn a_drill_point_is_a_cone_subtracted_from_the_cylinder() {
        let radius = 5.0f64;
        let angle = 118.0f64;
        let height = radius / t::tan(0.5 * angle * PI / 180.0);
        let overall = 80.0;
        let p = drill(
            2.0 * radius,
            angle,
            40.0,
            &Shank::plain(2.0 * radius, overall),
        )
        .expect("valid");

        // The cone itself, plus the full-diameter cylinder above it. Note this
        // is *not* the cylinder less the cone: the cone replaces a cylindrical
        // slice of height `height`, and is a third of it, so the tool loses
        // two thirds of that slice rather than one cone's worth.
        let cone = PI * radius * radius * height / 3.0;
        let expected = cone + PI * radius * radius * (overall - height);
        assert!(close(p.volume(), expected, 1e-14), "{}", p.volume());
    }

    #[test]
    fn a_bull_nose_sits_between_the_flat_and_the_ball_it_interpolates() {
        let flat = flat_end_mill(8.0, 25.0, &Shank::plain(8.0, 50.0)).expect("valid");
        let bull = bull_end_mill(8.0, 1.5, 25.0, &Shank::plain(8.0, 50.0)).expect("valid");
        let ball = ball_end_mill(8.0, 25.0, &Shank::plain(8.0, 50.0)).expect("valid");
        assert!(
            ball.volume() < bull.volume() && bull.volume() < flat.volume(),
            "ball {} < bull {} < flat {}",
            ball.volume(),
            bull.volume(),
            flat.volume()
        );
    }

    #[test]
    fn a_barrel_is_more_slender_than_the_ball_it_generalises() {
        let ball = ball_end_mill(12.0, 60.0, &Shank::plain(12.0, 100.0)).expect("valid");
        let barrel = barrel_end_mill(12.0, 200.0, 60.0, &Shank::plain(12.0, 100.0)).expect("valid");
        assert!(
            barrel.volume() < ball.volume(),
            "a ball reaches full width 6 mm above the tip and a 200 mm barrel \
             takes 48 mm to do it, so the barrel is the thinner solid: \
             barrel {} vs ball {}",
            barrel.volume(),
            ball.volume()
        );
        // Both are bounded by the same cylinder.
        let cylinder = ball.bounding_cylinder();
        assert!((cylinder.radius - 6.0).abs() < 1e-12);
        assert!((cylinder.z_max - 100.0).abs() < 1e-12);
        assert!(barrel.volume() < cylinder.volume());
    }

    #[test]
    fn role_volumes_partition_the_tool_exactly() {
        let p = flat_end_mill(
            6.0,
            20.0,
            &Shank::with_holder(6.0, 40.0, [HolderStage::cylinder(25.0, 30.0)]),
        )
        .expect("valid");
        let sum = p.volume_of_role(ElementRole::Cutting)
            + p.volume_of_role(ElementRole::NonCutting)
            + p.volume_of_role(ElementRole::Holder);
        assert!(close(sum, p.volume(), 1e-14), "{sum} vs {}", p.volume());
    }

    #[test]
    fn the_bounding_cylinder_is_the_smallest_one_that_fits() {
        let p = bull_end_mill(10.0, 2.0, 30.0, &Shank::plain(8.0, 70.0)).expect("valid");
        let c = p.bounding_cylinder();
        assert!((c.radius - 5.0).abs() < 1e-12, "the cutter, not the shank");
        assert!(c.z_min.abs() < 1e-12);
        assert!((c.z_max - 70.0).abs() < 1e-12);
        assert!(
            p.volume() < c.volume(),
            "a tool with a necked shank cannot fill its own bounding cylinder"
        );
    }

    #[test]
    fn the_silhouette_traces_the_ball_then_the_shank() {
        let radius = 4.0f64;
        let p =
            ball_end_mill(2.0 * radius, 25.0, &Shank::plain(2.0 * radius, 50.0)).expect("valid");
        // On the ball: r = sqrt(R^2 - (R - z)^2).
        for z in [0.5, 1.0, 2.0, 3.0] {
            let expected = (radius * radius - (radius - z) * (radius - z)).sqrt();
            let found = p.silhouette_radius(z);
            assert!(
                (found - expected).abs() < 1e-9,
                "z {z}: {found} vs {expected}"
            );
        }
        // On the shank.
        assert!((p.silhouette_radius(30.0) - radius).abs() < 1e-12);
        // Off the ends.
        assert_eq!(p.silhouette_radius(60.0), 0.0);
        assert_eq!(p.silhouette_radius(-1.0), 0.0);
    }

    #[test]
    fn the_silhouette_shows_the_flat_bottom_of_an_end_mill() {
        let p = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
        assert!(
            (p.silhouette_radius(0.0) - 3.0).abs() < 1e-12,
            "the bottom face is horizontal, and crosses no horizontal line, \
             but it is still the outline of the tool at z = 0"
        );
    }

    #[test]
    fn containment_agrees_with_the_silhouette_everywhere_it_is_asked() {
        let p = bull_end_mill(10.0, 2.0, 30.0, &Shank::plain(6.0, 70.0)).expect("valid");
        for i in 1..200 {
            let z = 70.0 * f64::from(i) / 200.0;
            let r = p.silhouette_radius(z);
            if r <= 0.0 {
                continue;
            }
            assert!(
                p.contains_rz(0.5 * r, z),
                "half the silhouette radius must be inside at z {z}"
            );
            assert!(
                !p.contains_rz(r * 1.5 + 1.0, z),
                "well outside the silhouette must be outside at z {z}"
            );
        }
    }

    #[test]
    fn containment_in_three_dimensions_ignores_the_angle_about_the_axis() {
        let tool = crate::tool::Tool::new(
            1,
            crate::tool::ToolId::new("t1").expect("valid"),
            "6 mm flat",
            flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
            60.0,
        )
        .expect("valid");
        for i in 0..16 {
            let angle = 2.0 * PI * f64::from(i) / 16.0;
            let (sin, cos) = t::sin_cos(angle);
            assert!(tool.contains_point(Vec3::new(2.0 * cos, 2.0 * sin, 10.0)));
            assert!(!tool.contains_point(Vec3::new(4.0 * cos, 4.0 * sin, 10.0)));
        }
        assert!(
            !tool.contains_point(Vec3::new(0.0, 0.0, -1.0)),
            "below the tip"
        );
        assert!(
            !tool.contains_point(Vec3::new(0.0, 0.0, 51.0)),
            "above the top"
        );
    }

    #[test]
    fn the_nearest_surface_names_the_role_that_would_be_hit() {
        let p = flat_end_mill(
            6.0,
            20.0,
            &Shank::with_holder(6.0, 40.0, [HolderStage::cylinder(30.0, 30.0)]),
        )
        .expect("valid");

        // Beside the flutes.
        let near_flute = p.nearest_surface(4.0, 10.0);
        assert_eq!(near_flute.role, ElementRole::Cutting);
        assert!((near_flute.distance - 1.0).abs() < 1e-12);

        // Beside the shank, above the flutes.
        assert_eq!(p.nearest_surface(4.0, 30.0).role, ElementRole::NonCutting);

        // Beside the holder.
        let near_holder = p.nearest_surface(20.0, 55.0);
        assert_eq!(
            near_holder.role,
            ElementRole::Holder,
            "contact here is a crash, and the query is what tells U8 so"
        );
    }

    #[test]
    fn the_nearest_surface_to_a_point_above_the_tool_is_the_top_cap() {
        let p = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
        let contact = p.nearest_surface(1.0, 60.0);
        assert_eq!(contact.element, TOP_CAP);
        assert!((contact.distance - 10.0).abs() < 1e-12);
        assert!((contact.closest.y - 50.0).abs() < 1e-12);
    }

    #[test]
    fn the_tip_is_on_the_surface_and_the_axis_just_above_it_is_inside() {
        let p = ball_end_mill(8.0, 25.0, &Shank::plain(8.0, 50.0)).expect("valid");
        assert!(p.nearest_surface(0.0, 0.0).distance < 1e-12, "the tip");
        assert!(p.contains_rz(0.0, 2.0), "on the axis, inside the ball");
        assert!(!p.contains_rz(0.0, -0.5), "below the tip");
    }
}
