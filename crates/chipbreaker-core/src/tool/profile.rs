// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The generating profile of a tool: a chain of segments and arcs in `(r, z)`.

use crate::eps::{EPS_LENGTH, EPS_RELATIVE};
use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec2;
use crate::transcendental as t;

use core::f64::consts::{PI, TAU};
use core::fmt;

/// Which way an arc turns, going from its start point to its end point.
///
/// Two arcs pass through any pair of points on a circle. This picks one, and it
/// is the only thing that can: a sweep angle would work for arcs below a
/// semicircle and become ambiguous above it, and undercut cutters need arcs
/// above it.
///
/// The sense is that of the `(r, z)` plane drawn with `r` to the right and `z`
/// upward — the same handedness as the `xy` plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ArcDirection {
    /// Increasing angle about the centre.
    CounterClockwise,
    /// Decreasing angle about the centre.
    Clockwise,
}

impl ArcDirection {
    /// `+1` for counter-clockwise, `-1` for clockwise.
    #[inline]
    #[must_use]
    pub const fn sign(self) -> f64 {
        match self {
            Self::CounterClockwise => 1.0,
            Self::Clockwise => -1.0,
        }
    }

    /// The name used in the tool file format.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::CounterClockwise => "ccw",
            Self::Clockwise => "cw",
        }
    }
}

impl Hashable for ArcDirection {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// What a piece of the tool is for.
///
/// This is a *material* distinction, not a geometric one. Every role is solid
/// and every role removes stock if it touches it — a holder that ploughs through
/// the part removes material exactly as a flute would. The difference is what it
/// means when it happens: cutting is the point, non-cutting is a rub, and holder
/// contact is a crash. U8 reports each differently and U12 must never optimise a
/// path by letting the shank cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ElementRole {
    /// Flutes: the part of the tool intended to remove material.
    Cutting,
    /// Shank, neck, or taper above the flutes. Solid, but contact is a defect.
    NonCutting,
    /// Collet, nut, spindle nose. Contact here is a crash.
    Holder,
}

impl ElementRole {
    /// The name used in the tool file format and in reports.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Cutting => "cutting",
            Self::NonCutting => "non-cutting",
            Self::Holder => "holder",
        }
    }

    /// Ordering by severity of contact: cutting is intended, holder is a crash.
    #[must_use]
    pub const fn severity(self) -> u8 {
        match self {
            Self::Cutting => 0,
            Self::NonCutting => 1,
            Self::Holder => 2,
        }
    }
}

impl Hashable for ElementRole {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

impl fmt::Display for ElementRole {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One piece of a profile: a straight segment or a circular arc, in `(r, z)`.
///
/// Both carry their own start and end rather than relying on the chain, so an
/// element can be intersected, measured, and revolved on its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProfileElement {
    /// A straight line from `start` to `end`. Revolves to a cylinder, a cone, or
    /// an annular disc.
    Segment {
        /// `(r, z)` of the start.
        start: Vec2,
        /// `(r, z)` of the end.
        end: Vec2,
    },
    /// A circular arc. Revolves to a sphere when the centre is on the axis, and
    /// to a torus when it is not.
    Arc {
        /// `(r, z)` of the start.
        start: Vec2,
        /// `(r, z)` of the end.
        end: Vec2,
        /// `(r, z)` of the centre of curvature.
        center: Vec2,
        /// Which of the two arcs through `start` and `end` is meant.
        direction: ArcDirection,
    },
}

impl ProfileElement {
    /// `(r, z)` where the element begins.
    #[inline]
    #[must_use]
    pub const fn start(&self) -> Vec2 {
        match self {
            Self::Segment { start, .. } | Self::Arc { start, .. } => *start,
        }
    }

    /// `(r, z)` where the element ends.
    #[inline]
    #[must_use]
    pub const fn end(&self) -> Vec2 {
        match self {
            Self::Segment { end, .. } | Self::Arc { end, .. } => *end,
        }
    }

    /// Radius of curvature, or `None` for a segment.
    #[must_use]
    pub fn radius(&self) -> Option<f64> {
        match self {
            Self::Segment { .. } => None,
            Self::Arc { start, center, .. } => Some((*start - *center).length()),
        }
    }

    /// The angles of `start` and `end` about the centre, and the signed sweep
    /// between them, or `None` for a segment.
    ///
    /// The sweep is always in `(0, TAU)` and carries the sign of the direction,
    /// so `start_angle + sweep` lands on `end_angle` modulo `TAU` however far
    /// around the arc goes.
    #[must_use]
    pub fn angles(&self) -> Option<(f64, f64, f64)> {
        let Self::Arc {
            start,
            end,
            center,
            direction,
        } = self
        else {
            return None;
        };
        let from = *start - *center;
        let to = *end - *center;
        let a0 = t::atan2(from.y, from.x);
        let a1 = t::atan2(to.y, to.x);
        let mut sweep = (a1 - a0) * direction.sign();
        // atan2 returns (-PI, PI], so the raw difference is in (-TAU, TAU).
        // Exactly one representative lies in (0, TAU].
        while sweep <= 0.0 {
            sweep += TAU;
        }
        Some((a0, a1, sweep * direction.sign()))
    }

    /// Arc length in the `(r, z)` plane.
    #[must_use]
    pub fn length(&self) -> f64 {
        match self {
            Self::Segment { start, end } => (*end - *start).length(),
            Self::Arc { .. } => {
                let radius = self.radius().unwrap_or(0.0);
                let (_, _, sweep) = self.angles().unwrap_or((0.0, 0.0, 0.0));
                radius * sweep.abs()
            }
        }
    }

    /// The point at parameter `u`, which runs from `0.0` at `start` to `1.0` at
    /// `end`.
    ///
    /// Parameterised by *angle* on an arc rather than by arc length, because
    /// angle is what the ray intersection produces and the two must agree
    /// exactly about which points belong to the element.
    #[must_use]
    pub fn point_at(&self, u: f64) -> Vec2 {
        match self {
            Self::Segment { start, end } => *start + (*end - *start) * u,
            Self::Arc { center, .. } => {
                let radius = self.radius().unwrap_or(0.0);
                let (a0, _, sweep) = self.angles().unwrap_or((0.0, 0.0, 0.0));
                let (sin, cos) = t::sin_cos(a0 + sweep * u);
                *center + Vec2::new(cos, sin) * radius
            }
        }
    }

    /// True if `angle` (about the arc centre) lies within the arc's sweep.
    ///
    /// Always false for a segment. `slack` widens the range at both ends, in
    /// radians, so a ray that meets the surface a hair outside the element still
    /// counts — without it, adjacent elements of a chain would leak between them.
    #[must_use]
    pub fn contains_angle(&self, angle: f64, slack: f64) -> bool {
        let Some((a0, _, sweep)) = self.angles() else {
            return false;
        };
        // Offset from the start, in the direction of travel, wrapped to [0, TAU).
        let mut offset = (angle - a0) * sweep.signum();
        while offset < 0.0 {
            offset += TAU;
        }
        while offset >= TAU {
            offset -= TAU;
        }
        // The wrap puts a point just *before* the start near TAU, not near zero.
        offset <= sweep.abs() + slack || offset >= TAU - slack
    }

    /// The element with `start` and `end` exchanged, tracing the same geometry
    /// backwards.
    #[must_use]
    pub fn reversed(&self) -> Self {
        match *self {
            Self::Segment { start, end } => Self::Segment {
                start: end,
                end: start,
            },
            Self::Arc {
                start,
                end,
                center,
                direction,
            } => Self::Arc {
                start: end,
                end: start,
                center,
                direction: match direction {
                    ArcDirection::CounterClockwise => ArcDirection::Clockwise,
                    ArcDirection::Clockwise => ArcDirection::CounterClockwise,
                },
            },
        }
    }

    /// The smallest and largest radius reached by the element.
    ///
    /// Not simply the endpoints: an arc can bulge past both of them, and the
    /// bulge is what sets the tool's swept diameter.
    #[must_use]
    pub fn radius_range(&self) -> (f64, f64) {
        let (mut lo, mut hi) = min_max(self.start().x, self.end().x);
        if let Self::Arc { center, .. } = self {
            let radius = self.radius().unwrap_or(0.0);
            // The extremes of r on a circle are at angle 0 and PI.
            for (angle, candidate) in [(0.0, center.x + radius), (PI, center.x - radius)] {
                if self.contains_angle(angle, 0.0) {
                    lo = lo.min(candidate);
                    hi = hi.max(candidate);
                }
            }
        }
        (lo, hi)
    }

    /// The smallest and largest `z` reached by the element.
    #[must_use]
    pub fn z_range(&self) -> (f64, f64) {
        let (mut lo, mut hi) = min_max(self.start().y, self.end().y);
        if let Self::Arc { center, .. } = self {
            let radius = self.radius().unwrap_or(0.0);
            for (angle, candidate) in [
                (0.5 * PI, center.y + radius),
                (-0.5 * PI, center.y - radius),
            ] {
                if self.contains_angle(angle, 0.0) {
                    lo = lo.min(candidate);
                    hi = hi.max(candidate);
                }
            }
        }
        (lo, hi)
    }
}

fn min_max(a: f64, b: f64) -> (f64, f64) {
    if a <= b { (a, b) } else { (b, a) }
}

impl Hashable for ProfileElement {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        match self {
            Self::Segment { start, end } => {
                h.begin("Segment");
                h.f64_slice(&[start.x, start.y, end.x, end.y]);
                h.end();
            }
            Self::Arc {
                start,
                end,
                center,
                direction,
            } => {
                h.begin("Arc");
                h.f64_slice(&[start.x, start.y, end.x, end.y, center.x, center.y]);
                h.add(direction);
                h.end();
            }
        }
    }
}

/// A profile element together with what it is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoledElement {
    /// The geometry.
    pub element: ProfileElement,
    /// What this piece of the tool is.
    pub role: ElementRole,
}

impl RoledElement {
    /// A cutting element.
    #[must_use]
    pub const fn cutting(element: ProfileElement) -> Self {
        Self {
            element,
            role: ElementRole::Cutting,
        }
    }

    /// A non-cutting element: shank, neck, or taper.
    #[must_use]
    pub const fn non_cutting(element: ProfileElement) -> Self {
        Self {
            element,
            role: ElementRole::NonCutting,
        }
    }

    /// A holder element.
    #[must_use]
    pub const fn holder(element: ProfileElement) -> Self {
        Self {
            element,
            role: ElementRole::Holder,
        }
    }
}

impl Hashable for RoledElement {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("RoledElement");
        h.add(&self.element);
        h.add(&self.role);
        h.end();
    }
}

/// Why a profile was rejected.
///
/// Every variant names the element it applies to, because a profile that fails
/// validation is usually a data-entry error in a tool library and the index is
/// the only thing that makes it findable.
#[derive(Debug, Clone, PartialEq)]
pub enum ProfileError {
    /// A profile needs at least one element.
    Empty,
    /// The first point must be the tool tip, exactly `(0, 0)`.
    TipNotAtOrigin {
        /// Where the profile actually begins.
        found: Vec2,
    },
    /// Element `index` does not begin where element `index - 1` ended.
    Discontinuous {
        /// Index of the element that begins in the wrong place.
        index: usize,
        /// Where the previous element ended.
        expected: Vec2,
        /// Where this element begins.
        found: Vec2,
    },
    /// A negative radius. The profile lives in the half-plane `r >= 0`.
    NegativeRadius {
        /// Index of the offending element.
        index: usize,
        /// The negative radius.
        radius: f64,
    },
    /// An element of zero length, which revolves to nothing and has no
    /// well-defined normal.
    ZeroLength {
        /// Index of the offending element.
        index: usize,
    },
    /// An arc whose endpoints are not the same distance from its centre.
    InconsistentArcRadius {
        /// Index of the offending element.
        index: usize,
        /// Distance from the centre to the start.
        start_radius: f64,
        /// Distance from the centre to the end.
        end_radius: f64,
    },
    /// An arc that closes on itself, whose two endpoints coincide.
    ///
    /// A full circle cannot be expressed as one element because its start and
    /// end are the same point and its direction is then unrecoverable. Split it.
    ClosedArc {
        /// Index of the offending element.
        index: usize,
    },
    /// A coordinate that is not finite.
    NotFinite {
        /// Index of the offending element.
        index: usize,
    },
    /// The chain moves downward past the tip.
    ///
    /// The tip is the origin and the tool occupies `z >= 0`; a profile that dips
    /// below it would put material under the point that defines gauge length.
    BelowTip {
        /// Index of the offending element.
        index: usize,
        /// The lowest `z` it reaches.
        z: f64,
    },
    /// The chain crosses itself.
    ///
    /// A profile bounds a region together with the axis and one cap, and that
    /// region is only well defined if the chain is simple. A crossing makes
    /// containment and volume meaningless rather than merely inaccurate; see
    /// [`super::selfintersect`].
    SelfIntersecting {
        /// Index of the earlier element.
        first: u32,
        /// Index of the later element.
        second: u32,
        /// `(r, z)` where they meet.
        at: Vec2,
    },
    /// Roles are out of order.
    ///
    /// Reading from the tip upward, cutting geometry must come before
    /// non-cutting geometry, which must come before the holder. A holder below a
    /// flute is not a tool, and silently accepting it would make U8's contact
    /// classification meaningless.
    RolesOutOfOrder {
        /// Index of the element whose role is lower than its predecessor's.
        index: usize,
        /// The predecessor's role.
        previous: ElementRole,
        /// This element's role.
        found: ElementRole,
    },
}

impl fmt::Display for ProfileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "a profile needs at least one element"),
            Self::TipNotAtOrigin { found } => write!(
                f,
                "the profile must begin at the tip (r 0, z 0), not (r {}, z {})",
                found.x, found.y
            ),
            Self::Discontinuous {
                index,
                expected,
                found,
            } => write!(
                f,
                "element {index} begins at (r {}, z {}) but element {} ended at (r {}, z {})",
                found.x,
                found.y,
                index - 1,
                expected.x,
                expected.y
            ),
            Self::NegativeRadius { index, radius } => {
                write!(f, "element {index} has negative radius {radius}")
            }
            Self::ZeroLength { index } => write!(f, "element {index} has zero length"),
            Self::InconsistentArcRadius {
                index,
                start_radius,
                end_radius,
            } => write!(
                f,
                "arc {index} has radius {start_radius} at its start and {end_radius} at its end"
            ),
            Self::ClosedArc { index } => write!(
                f,
                "arc {index} begins and ends at the same point; split it into two"
            ),
            Self::NotFinite { index } => write!(f, "element {index} has a non-finite coordinate"),
            Self::BelowTip { index, z } => {
                write!(f, "element {index} reaches z = {z}, below the tip at z = 0")
            }
            Self::SelfIntersecting { first, second, at } => write!(
                f,
                "elements {first} and {second} cross at (r {}, z {}); a profile                  that crosses itself does not bound a well-defined solid",
                at.x, at.y
            ),
            Self::RolesOutOfOrder {
                index,
                previous,
                found,
            } => write!(
                f,
                "element {index} is {found} but follows {previous}; \
                 roles must run cutting, then non-cutting, then holder"
            ),
        }
    }
}

impl core::error::Error for ProfileError {}

/// A validated generating profile, running from the tool tip upward.
///
/// # The coordinate convention
///
/// * The tool axis is `+Z`. The tip is at the origin.
/// * The profile lives in the half-plane `r >= 0`, `z >= 0`, revolved about the
///   axis.
/// * It begins at `(0, 0)` — on the axis, at the tip — and runs upward. The
///   solid is closed at the top by a disc from the axis to the last point.
///
/// That last rule is what makes the solid well defined without storing the caps:
/// the profile, the axis, and one horizontal disc bound a closed region, and
/// revolving it gives the tool. A profile is therefore always an *open* chain,
/// and a closed one is a validation error rather than a second representation to
/// support.
///
/// # What is deliberately not here
///
/// Flutes, helix angle, rake, relief, number of teeth. Material removal models
/// the swept envelope of the tool, which is a surface of revolution; a four-flute
/// and a two-flute cutter of the same diameter and corner radius remove exactly
/// the same material. Adding flute geometry would add no accuracy and would make
/// every ray intersection a great deal harder. It is out of scope for the
/// project, not merely for this unit.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    elements: Vec<RoledElement>,
}

impl Profile {
    /// Validates a chain of elements and builds a profile.
    ///
    /// # Errors
    ///
    /// Returns the first [`ProfileError`] found, scanning from the tip upward.
    pub fn new(elements: Vec<RoledElement>) -> Result<Self, ProfileError> {
        if elements.is_empty() {
            return Err(ProfileError::Empty);
        }

        let first = elements[0].element.start();
        if first.x != 0.0 || first.y != 0.0 {
            return Err(ProfileError::TipNotAtOrigin { found: first });
        }

        let mut previous_role = ElementRole::Cutting;
        for (index, roled) in elements.iter().enumerate() {
            let element = &roled.element;

            if !element.start().is_finite() || !element.end().is_finite() {
                return Err(ProfileError::NotFinite { index });
            }
            if let ProfileElement::Arc { center, .. } = element
                && !center.is_finite()
            {
                return Err(ProfileError::NotFinite { index });
            }

            if index > 0 {
                let expected = elements[index - 1].element.end();
                let found = element.start();
                if (found - expected).length() > EPS_LENGTH {
                    return Err(ProfileError::Discontinuous {
                        index,
                        expected,
                        found,
                    });
                }
            }

            if let ProfileElement::Arc {
                start, end, center, ..
            } = element
            {
                if (*end - *start).length() <= EPS_LENGTH {
                    return Err(ProfileError::ClosedArc { index });
                }
                let start_radius = (*start - *center).length();
                let end_radius = (*end - *center).length();
                let scale = start_radius.max(end_radius).max(1.0);
                if (start_radius - end_radius).abs() > EPS_RELATIVE * scale {
                    return Err(ProfileError::InconsistentArcRadius {
                        index,
                        start_radius,
                        end_radius,
                    });
                }
            }

            if element.length() <= EPS_LENGTH {
                return Err(ProfileError::ZeroLength { index });
            }

            let (r_lo, _) = element.radius_range();
            if r_lo < -EPS_LENGTH {
                return Err(ProfileError::NegativeRadius {
                    index,
                    radius: r_lo,
                });
            }
            let (z_lo, _) = element.z_range();
            if z_lo < -EPS_LENGTH {
                return Err(ProfileError::BelowTip { index, z: z_lo });
            }

            if index > 0 && roled.role.severity() < previous_role.severity() {
                return Err(ProfileError::RolesOutOfOrder {
                    index,
                    previous: previous_role,
                    found: roled.role,
                });
            }
            previous_role = roled.role;
        }

        // Last, because it is the only O(n^2) check and because its message is
        // the least useful of the set: if the chain is also discontinuous or
        // has a zero-length element, say that instead.
        if let Some(crossing) = super::selfintersect::first_crossing(
            &elements.iter().map(|e| e.element).collect::<Vec<_>>(),
        ) {
            return Err(ProfileError::SelfIntersecting {
                first: crossing.first,
                second: crossing.second,
                at: crossing.at,
            });
        }

        Ok(Self { elements })
    }

    /// The elements, from the tip upward.
    #[inline]
    #[must_use]
    pub fn elements(&self) -> &[RoledElement] {
        &self.elements
    }

    /// Number of elements.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Always false — a validated profile has at least one element.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// The largest radius anywhere on the profile: the tool's swept radius.
    #[must_use]
    pub fn max_radius(&self) -> f64 {
        self.elements
            .iter()
            .map(|e| e.element.radius_range().1)
            .fold(0.0, f64::max)
    }

    /// The highest `z` on the profile. The tool occupies `[0, this]`.
    #[must_use]
    pub fn total_length(&self) -> f64 {
        self.elements
            .iter()
            .map(|e| e.element.z_range().1)
            .fold(0.0, f64::max)
    }

    /// The point where the top cap sits: the end of the last element.
    #[must_use]
    pub fn top(&self) -> Vec2 {
        self.elements
            .last()
            .map_or(Vec2::new(0.0, 0.0), |e| e.element.end())
    }

    /// The highest `z` reached by any element of the given role, or `None` if
    /// the profile has none.
    #[must_use]
    pub fn top_of_role(&self, role: ElementRole) -> Option<f64> {
        self.elements
            .iter()
            .filter(|e| e.role == role)
            .map(|e| e.element.z_range().1)
            .fold(None, |acc: Option<f64>, z| {
                Some(acc.map_or(z, |a| a.max(z)))
            })
    }
}

impl Hashable for Profile {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Profile");
        h.usize(self.elements.len());
        for e in &self.elements {
            h.add(e);
        }
        h.end();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden::CanonicalHash;

    fn v(r: f64, z: f64) -> Vec2 {
        Vec2::new(r, z)
    }

    fn seg(a: Vec2, b: Vec2) -> ProfileElement {
        ProfileElement::Segment { start: a, end: b }
    }

    /// The quarter circle a ball nose is made of: tip to equator.
    fn ball_arc(radius: f64) -> ProfileElement {
        ProfileElement::Arc {
            start: v(0.0, 0.0),
            end: v(radius, radius),
            center: v(0.0, radius),
            direction: ArcDirection::CounterClockwise,
        }
    }

    #[test]
    fn a_ball_nose_arc_sweeps_exactly_a_quarter_turn() {
        let arc = ball_arc(5.0);
        let (start, end, sweep) = arc.angles().expect("an arc has angles");
        assert!((start + 0.5 * PI).abs() < 1e-12, "starts pointing down");
        assert!(end.abs() < 1e-12, "ends pointing out along +r");
        assert!((sweep - 0.5 * PI).abs() < 1e-12, "sweep {sweep}");
        assert!((arc.length() - 0.5 * PI * 5.0).abs() < 1e-12);
        assert_eq!(arc.radius(), Some(5.0));
    }

    #[test]
    fn point_at_runs_from_start_to_end() {
        for element in [seg(v(1.0, 2.0), v(4.0, 6.0)), ball_arc(3.0)] {
            let a = element.point_at(0.0);
            let b = element.point_at(1.0);
            assert!((a - element.start()).length() < 1e-12, "{element:?}");
            assert!((b - element.end()).length() < 1e-12, "{element:?}");
            assert!(element.point_at(0.5).is_finite());
        }
    }

    #[test]
    fn an_arc_reports_the_radius_it_bulges_to_not_only_its_endpoints() {
        // A semicircle from (1, 0) to (1, 2) about (1, 1), bulging out to r = 2.
        let arc = ProfileElement::Arc {
            start: v(1.0, 0.0),
            end: v(1.0, 2.0),
            center: v(1.0, 1.0),
            direction: ArcDirection::CounterClockwise,
        };
        let (lo, hi) = arc.radius_range();
        assert!((lo - 1.0).abs() < 1e-12, "lo {lo}");
        assert!(
            (hi - 2.0).abs() < 1e-12,
            "the bulge, not the endpoints: {hi}"
        );
        let (z_lo, z_hi) = arc.z_range();
        assert!((z_lo - 0.0).abs() < 1e-12 && (z_hi - 2.0).abs() < 1e-12);
    }

    #[test]
    fn reversing_an_arc_traces_the_same_points_backwards() {
        let arc = ball_arc(2.0);
        let back = arc.reversed();
        for i in 0..=8 {
            let u = f64::from(i) / 8.0;
            let a = arc.point_at(u);
            let b = back.point_at(1.0 - u);
            assert!((a - b).length() < 1e-12, "u {u}: {a:?} vs {b:?}");
        }
    }

    #[test]
    fn contains_angle_covers_the_sweep_and_nothing_else() {
        let arc = ball_arc(1.0);
        // The quarter turn runs from -PI/2 to 0.
        assert!(arc.contains_angle(-0.25 * PI, 0.0));
        assert!(arc.contains_angle(-0.5 * PI, 0.0));
        assert!(arc.contains_angle(0.0, 0.0));
        assert!(!arc.contains_angle(0.25 * PI, 0.0));
        assert!(!arc.contains_angle(-0.75 * PI, 0.0));
        // Slack reaches a hair outside at both ends, which is what keeps a ray
        // from leaking between two elements that meet there.
        assert!(arc.contains_angle(1e-9, 1e-6));
        assert!(arc.contains_angle(-0.5 * PI - 1e-9, 1e-6));
    }

    #[test]
    fn a_flat_end_mill_profile_validates() {
        let profile = Profile::new(vec![
            RoledElement::cutting(seg(v(0.0, 0.0), v(3.0, 0.0))),
            RoledElement::cutting(seg(v(3.0, 0.0), v(3.0, 20.0))),
            RoledElement::non_cutting(seg(v(3.0, 20.0), v(3.0, 60.0))),
        ])
        .expect("a flat end mill is a valid profile");
        assert_eq!(profile.len(), 3);
        assert!((profile.max_radius() - 3.0).abs() < 1e-12);
        assert!((profile.total_length() - 60.0).abs() < 1e-12);
        assert_eq!(profile.top(), v(3.0, 60.0));
        assert_eq!(profile.top_of_role(ElementRole::Cutting), Some(20.0));
        assert_eq!(profile.top_of_role(ElementRole::Holder), None);
    }

    #[test]
    fn an_empty_profile_is_rejected() {
        assert_eq!(Profile::new(vec![]), Err(ProfileError::Empty));
    }

    #[test]
    fn a_profile_that_does_not_start_at_the_tip_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(seg(v(1.0, 0.0), v(3.0, 0.0)))])
            .expect_err("must start at the origin");
        assert!(
            matches!(err, ProfileError::TipNotAtOrigin { .. }),
            "{err:?}"
        );
    }

    #[test]
    fn a_gap_in_the_chain_is_rejected_and_names_the_element() {
        let err = Profile::new(vec![
            RoledElement::cutting(seg(v(0.0, 0.0), v(3.0, 0.0))),
            RoledElement::cutting(seg(v(3.5, 0.0), v(3.5, 20.0))),
        ])
        .expect_err("a gap is not a profile");
        match err {
            ProfileError::Discontinuous { index, .. } => assert_eq!(index, 1),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_negative_radius_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(seg(v(0.0, 0.0), v(-3.0, 0.0)))])
            .expect_err("the profile lives in r >= 0");
        assert!(
            matches!(err, ProfileError::NegativeRadius { .. }),
            "{err:?}"
        );
    }

    #[test]
    fn dipping_below_the_tip_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(seg(v(0.0, 0.0), v(2.0, -1.0)))])
            .expect_err("the tip is the lowest point of the tool");
        assert!(matches!(err, ProfileError::BelowTip { .. }), "{err:?}");
    }

    #[test]
    fn a_zero_length_element_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(seg(v(0.0, 0.0), v(0.0, 0.0)))])
            .expect_err("nothing revolves to nothing");
        assert!(matches!(err, ProfileError::ZeroLength { .. }), "{err:?}");
    }

    #[test]
    fn an_arc_whose_endpoints_disagree_about_its_radius_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(ProfileElement::Arc {
            start: v(0.0, 0.0),
            end: v(3.0, 5.0),
            center: v(0.0, 5.0),
            direction: ArcDirection::CounterClockwise,
        })])
        .expect_err("5 from the start, 3 from the end");
        match err {
            ProfileError::InconsistentArcRadius {
                start_radius,
                end_radius,
                ..
            } => {
                assert!((start_radius - 5.0).abs() < 1e-12);
                assert!((end_radius - 3.0).abs() < 1e-12);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn an_arc_that_closes_on_itself_is_rejected() {
        let err = Profile::new(vec![RoledElement::cutting(ProfileElement::Arc {
            start: v(0.0, 0.0),
            end: v(0.0, 0.0),
            center: v(0.0, 5.0),
            direction: ArcDirection::CounterClockwise,
        })])
        .expect_err("a full circle has no recoverable direction");
        assert!(matches!(err, ProfileError::ClosedArc { .. }), "{err:?}");
    }

    #[test]
    fn a_holder_below_a_flute_is_rejected() {
        let err = Profile::new(vec![
            RoledElement::holder(seg(v(0.0, 0.0), v(3.0, 0.0))),
            RoledElement::cutting(seg(v(3.0, 0.0), v(3.0, 20.0))),
        ])
        .expect_err("roles must run cutting, non-cutting, holder");
        match err {
            ProfileError::RolesOutOfOrder { index, .. } => assert_eq!(index, 1),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_non_finite_coordinate_is_rejected() {
        for bad in [f64::NAN, f64::INFINITY] {
            let err = Profile::new(vec![RoledElement::cutting(seg(v(0.0, 0.0), v(bad, 1.0)))])
                .expect_err("not a coordinate");
            assert!(matches!(err, ProfileError::NotFinite { .. }), "{err:?}");
        }
    }

    #[test]
    fn hashing_a_profile_is_stable_and_distinguishes_arc_direction() {
        let hash = |h: &dyn Fn(&mut CanonicalHash)| {
            let mut c = CanonicalHash::new();
            h(&mut c);
            c.finish().to_hex()
        };

        let profile = Profile::new(vec![RoledElement::cutting(ball_arc(4.0))]).expect("valid");
        assert_eq!(
            hash(&|c| {
                c.add(&profile);
            }),
            hash(&|c| {
                c.add(&profile);
            }),
            "hashing is a function"
        );

        // Direction is the only thing separating these two, and they are
        // different solids: the clockwise one goes the long way round, through
        // negative radius.
        let ccw = ball_arc(4.0);
        let cw = ProfileElement::Arc {
            start: v(0.0, 0.0),
            end: v(4.0, 4.0),
            center: v(0.0, 4.0),
            direction: ArcDirection::Clockwise,
        };
        assert_ne!(
            hash(&|c| {
                c.add(&ccw);
            }),
            hash(&|c| {
                c.add(&cw);
            }),
        );
    }

    #[test]
    fn an_arc_that_swings_through_negative_radius_is_rejected() {
        // Same endpoints and centre as a ball nose, but taken the long way:
        // 270 degrees clockwise, which passes through r = -4.
        let err = Profile::new(vec![RoledElement::cutting(ProfileElement::Arc {
            start: v(0.0, 0.0),
            end: v(4.0, 4.0),
            center: v(0.0, 4.0),
            direction: ArcDirection::Clockwise,
        })])
        .expect_err("the long way round leaves the half-plane");
        match err {
            ProfileError::NegativeRadius { radius, .. } => {
                assert!((radius + 4.0).abs() < 1e-12, "reaches r = {radius}");
            }
            other => panic!("{other:?}"),
        }
    }
}
