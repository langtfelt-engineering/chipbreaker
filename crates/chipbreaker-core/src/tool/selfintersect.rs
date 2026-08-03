// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Detecting a profile that crosses itself.
//!
//! # Why this is a rejection and not a warning
//!
//! A profile bounds a region: itself, the axis, and one cap. That region is only
//! well defined if the chain is *simple*. Let it cross itself and the region
//! becomes ambiguous — a ray leaving a point in `+r` crosses an even number of
//! boundaries in one place and an odd number a millimetre away, so
//! [`Profile::contains_rz`] returns an answer that is not wrong so much as
//! meaningless. Volume follows suit: Green's theorem integrates the boundary
//! quite happily and returns a number with no interpretation.
//!
//! Unit 3's parity test would eventually notice, as a leaking ray somewhere
//! downstream in U5, at which point the diagnosis is a day's work. Rejecting at
//! construction costs one `O(n^2)` scan over a handful of elements and names the
//! two elements involved.
//!
//! # What is exact and what is not
//!
//! Worth stating plainly rather than implying more rigour than is here.
//!
//! **Segment against segment is exact.** It is four [`orient2d`] calls and no
//! arithmetic of our own, so the answer is the true one for any coordinates
//! inside [`ORIENT2D_COORDS`].
//!
//! **Anything involving an arc is not.** The intersection of a line and a circle
//! is irrational in the inputs, so there is no exact predicate to appeal to; the
//! points are computed through the root solver and classified with a tolerance.
//! This is a deliberate limit rather than an oversight — an exact treatment would
//! need the algebraic-number machinery that the whole dexel approach exists to
//! avoid, and it would be answering a question about tool profiles that have at
//! most a handful of elements and never approach the degenerate cases.
//!
//! The tolerance is chosen to fail *towards rejection*: a crossing within
//! [`crate::eps::EPS_LENGTH`] of an endpoint is reported. A tool profile whose
//! elements touch that closely without sharing an endpoint is malformed anyway.

use crate::eps::EPS_LENGTH;
use crate::math::Vec2;
use crate::predicates::{Orientation, orient2d};
use crate::roots::solve_quadratic;
use crate::transcendental as t;

use super::profile::ProfileElement;

/// Where two profile elements meet when they should not.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing {
    /// Index of the earlier element.
    pub first: u32,
    /// Index of the later element.
    pub second: u32,
    /// A point at which they meet, in `(r, z)`.
    pub at: Vec2,
}

/// Does the point lie on the element, within `slack`?
fn lies_on(element: &ProfileElement, p: Vec2) -> bool {
    match element {
        ProfileElement::Segment { start, end } => {
            let along = *end - *start;
            let length_squared = along.length_squared();
            if length_squared <= 0.0 {
                return false;
            }
            let u = (p - *start).dot(along) / length_squared;
            if !(0.0..=1.0).contains(&u) {
                return false;
            }
            (p - (*start + along * u)).length() <= EPS_LENGTH
        }
        ProfileElement::Arc { center, .. } => {
            let radius = element.radius().unwrap_or(0.0);
            let offset = p - *center;
            if (offset.length() - radius).abs() > EPS_LENGTH {
                return false;
            }
            element.contains_angle(t::atan2(offset.y, offset.x), 1.0e-9)
        }
    }
}

/// Exact segment-versus-segment crossing, by sidedness alone.
///
/// Two segments cross when each straddles the other's line. `orient2d` answers
/// "which side" exactly, so this is exact — no tolerance, no intersection point
/// computed for the decision. The point is only computed afterwards, for the
/// error message.
fn segments_cross(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> Option<Vec2> {
    let d0 = orient2d(a0, a1, b0);
    let d1 = orient2d(a0, a1, b1);
    let d2 = orient2d(b0, b1, a0);
    let d3 = orient2d(b0, b1, a1);

    let straddles = |p: Orientation, q: Orientation| {
        (p == Orientation::Positive && q == Orientation::Negative)
            || (p == Orientation::Negative && q == Orientation::Positive)
    };

    if straddles(d0, d1) && straddles(d2, d3) {
        // A proper crossing. Solve for it only now, to report where.
        let da = a1 - a0;
        let db = b1 - b0;
        let denominator = da.cross(db);
        if denominator == 0.0 {
            return Some(a0);
        }
        let u = (b0 - a0).cross(db) / denominator;
        return Some(a0 + da * u);
    }

    // Collinear overlap: every endpoint on the other's line, and at least one
    // endpoint strictly inside the other segment. Not a crossing in the
    // straddling sense, but just as malformed.
    if d0 == Orientation::Zero && d1 == Orientation::Zero {
        for (p, other) in [
            (b0, (a0, a1)),
            (b1, (a0, a1)),
            (a0, (b0, b1)),
            (a1, (b0, b1)),
        ] {
            let along = other.1 - other.0;
            let length_squared = along.length_squared();
            if length_squared <= 0.0 {
                continue;
            }
            let u = (p - other.0).dot(along) / length_squared;
            if u > EPS_LENGTH && u < 1.0 - EPS_LENGTH {
                return Some(p);
            }
        }
    }
    None
}

/// Candidate intersection points of the circles or lines underlying two
/// elements, written into `out`; returns how many.
///
/// Not exact: see the module header. Everything here is a line-circle or
/// circle-circle solve, both of which are irrational in the inputs.
fn candidate_points(a: &ProfileElement, b: &ProfileElement, out: &mut [Vec2; 4]) -> usize {
    let mut n = 0;
    let mut push = |p: Vec2, n: &mut usize| {
        if *n < 4 && p.is_finite() {
            out[*n] = p;
            *n += 1;
        }
    };

    match (a, b) {
        (ProfileElement::Segment { .. }, ProfileElement::Segment { .. }) => 0,

        // Line against circle: substitute the parameterised line into the
        // circle and solve the quadratic.
        (ProfileElement::Segment { start, end }, ProfileElement::Arc { center, .. })
        | (ProfileElement::Arc { center, .. }, ProfileElement::Segment { start, end }) => {
            let arc = if matches!(a, ProfileElement::Arc { .. }) {
                a
            } else {
                b
            };
            let radius = arc.radius().unwrap_or(0.0);
            let d = *end - *start;
            let f = *start - *center;
            for (u, _) in
                solve_quadratic(d.dot(d), 2.0 * f.dot(d), f.dot(f) - radius * radius).iter()
            {
                if (0.0..=1.0).contains(&u) {
                    push(*start + d * u, &mut n);
                }
            }
            n
        }

        // Circle against circle: the radical line, then the two points on it.
        (ProfileElement::Arc { center: ca, .. }, ProfileElement::Arc { center: cb, .. }) => {
            let ra = a.radius().unwrap_or(0.0);
            let rb = b.radius().unwrap_or(0.0);
            let between = *cb - *ca;
            let distance = between.length();
            if distance <= 0.0 || distance > ra + rb || distance < (ra - rb).abs() {
                return 0;
            }
            let along = (distance * distance + ra * ra - rb * rb) / (2.0 * distance);
            let height_squared = ra * ra - along * along;
            if height_squared < 0.0 {
                return 0;
            }
            let height = height_squared.sqrt();
            let unit = between * (1.0 / distance);
            let foot = *ca + unit * along;
            let normal = Vec2::new(-unit.y, unit.x);
            push(foot + normal * height, &mut n);
            if height > 0.0 {
                push(foot - normal * height, &mut n);
            }
            n
        }
    }
}

/// Finds the first place a profile chain crosses itself, or `None`.
///
/// Elements adjacent in the chain share an endpoint by construction; that shared
/// point is not a crossing. Anything else is.
#[must_use]
pub fn first_crossing(elements: &[ProfileElement]) -> Option<Crossing> {
    for i in 0..elements.len() {
        for j in (i + 1)..elements.len() {
            let a = &elements[i];
            let b = &elements[j];

            // For adjacent elements exactly one point of contact is legitimate:
            // the endpoint they share. Anything else — including contact that
            // merely *involves* an endpoint — is a fault.
            //
            // An earlier version asked instead whether the contact point was
            // interior to both elements, and that let a whole class through. Two
            // collinear segments where the second retraces the first backwards
            // and then overruns it touch along an interval, and the witness
            // point the detector returns is the first element's own start. That
            // is not the shared endpoint, but it is not interior either, so the
            // weaker test discarded it and the chain validated.
            let permitted = if j == i + 1 { Some(a.end()) } else { None };
            let is_expected =
                |at: Vec2| permitted.is_some_and(|shared| (at - shared).length() <= EPS_LENGTH);

            // Segment against segment is decided exactly, by sidedness.
            if let (
                ProfileElement::Segment { start: a0, end: a1 },
                ProfileElement::Segment { start: b0, end: b1 },
            ) = (a, b)
            {
                if let Some(at) = segments_cross(*a0, *a1, *b0, *b1)
                    && !is_expected(at)
                {
                    return Some(Crossing {
                        first: u32::try_from(i).unwrap_or(u32::MAX),
                        second: u32::try_from(j).unwrap_or(u32::MAX),
                        at,
                    });
                }
                continue;
            }

            let mut points = [Vec2::new(0.0, 0.0); 4];
            let count = candidate_points(a, b, &mut points);
            for &at in &points[..count] {
                if !lies_on(a, at) || !lies_on(b, at) || is_expected(at) {
                    continue;
                }
                return Some(Crossing {
                    first: u32::try_from(i).unwrap_or(u32::MAX),
                    second: u32::try_from(j).unwrap_or(u32::MAX),
                    at,
                });
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool::catalog::{
        Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill, flat_end_mill,
        tapered_end_mill,
    };
    use crate::tool::profile::{ArcDirection, Profile, RoledElement};

    fn v(r: f64, z: f64) -> Vec2 {
        Vec2::new(r, z)
    }

    fn seg(a: Vec2, b: Vec2) -> ProfileElement {
        ProfileElement::Segment { start: a, end: b }
    }

    #[test]
    fn no_catalogue_tool_is_falsely_reported_as_crossing() {
        // The check runs inside `Profile::new`, so a false positive here would
        // make an ordinary tool unconstructable. Every standard form, plus the
        // awkward ones: a necked shank steps inward, a barrel's arc centre is
        // at negative radius, a holder steps outward twice.
        let cases: Vec<(&str, Profile)> = vec![
            (
                "flat",
                flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
            ),
            (
                "necked",
                flat_end_mill(10.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
            ),
            (
                "overhung",
                flat_end_mill(6.0, 20.0, &Shank::plain(10.0, 50.0)).expect("valid"),
            ),
            (
                "ball",
                ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
            ),
            (
                "bull",
                bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
            ),
            (
                "chamfer",
                chamfer_mill(8.0, 1.0, 90.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
            ),
            (
                "taper",
                tapered_end_mill(2.0, 10.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
            ),
            (
                "drill",
                drill(6.0, 118.0, 30.0, &Shank::plain(6.0, 50.0)).expect("valid"),
            ),
            (
                "barrel",
                barrel_end_mill(12.0, 200.0, 60.0, &Shank::plain(12.0, 90.0)).expect("valid"),
            ),
        ];
        for (name, profile) in cases {
            let elements: Vec<ProfileElement> =
                profile.elements().iter().map(|e| e.element).collect();
            assert!(
                first_crossing(&elements).is_none(),
                "{name} was reported as self-intersecting: {:?}",
                first_crossing(&elements)
            );
        }
    }

    #[test]
    fn adjacent_elements_sharing_an_endpoint_are_not_a_crossing() {
        // Every valid chain has this at every joint, so a check that reported it
        // would reject every profile ever written.
        let elements = [
            seg(v(0.0, 0.0), v(3.0, 0.0)),
            seg(v(3.0, 0.0), v(3.0, 20.0)),
            seg(v(3.0, 20.0), v(5.0, 20.0)),
        ];
        assert_eq!(first_crossing(&elements), None);
    }

    #[test]
    fn a_chain_that_doubles_back_through_itself_is_caught() {
        // Out to r=5, up, back in past the axis side, then up again crossing the
        // first riser. This is what a mis-entered form tool looks like.
        let elements = [
            seg(v(0.0, 0.0), v(5.0, 0.0)),
            seg(v(5.0, 0.0), v(5.0, 10.0)),
            seg(v(5.0, 10.0), v(2.0, 5.0)),
            seg(v(2.0, 5.0), v(8.0, 5.0)),
        ];
        let found = first_crossing(&elements).expect("elements 1 and 3 cross");
        assert_eq!((found.first, found.second), (1, 3));
        assert!(
            (found.at.x - 5.0).abs() < 1e-9 && (found.at.y - 5.0).abs() < 1e-9,
            "crossing reported at {:?}",
            found.at
        );
    }

    #[test]
    fn profile_new_rejects_a_crossing_chain() {
        let err = Profile::new(vec![
            RoledElement::cutting(seg(v(0.0, 0.0), v(5.0, 0.0))),
            RoledElement::cutting(seg(v(5.0, 0.0), v(5.0, 10.0))),
            RoledElement::cutting(seg(v(5.0, 10.0), v(2.0, 5.0))),
            RoledElement::cutting(seg(v(2.0, 5.0), v(8.0, 5.0))),
        ])
        .expect_err("a crossing chain does not bound a solid");
        match err {
            crate::tool::ProfileError::SelfIntersecting { first, second, .. } => {
                assert_eq!((first, second), (1, 3));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_segment_passing_through_an_arc_is_caught() {
        // A quarter arc bulging to r = 4, then a chain that cuts back across it.
        let elements = [
            ProfileElement::Arc {
                start: v(0.0, 0.0),
                end: v(4.0, 4.0),
                center: v(0.0, 4.0),
                direction: ArcDirection::CounterClockwise,
            },
            seg(v(4.0, 4.0), v(6.0, 4.0)),
            seg(v(6.0, 4.0), v(6.0, 1.0)),
            seg(v(6.0, 1.0), v(0.5, 1.0)),
        ];
        let found = first_crossing(&elements).expect("the last segment crosses the arc");
        assert_eq!(found.first, 0);
        assert_eq!(found.second, 3);
    }

    #[test]
    fn two_arcs_that_cross_are_caught() {
        // Two quarter circles of radius 4, centred at (0,4) and (6,0). Their
        // circles meet at (3.961, 3.441) and (2.039, 0.559); the first of those
        // lies inside *both* sweeps, so the arcs genuinely cross.
        //
        // Worth noting how this case was arrived at: the first attempt used two
        // arcs whose circles intersect but whose sweeps do not contain either
        // intersection, so nothing was crossing and the detector was right to
        // say so. Circles meeting is not arcs meeting.
        let elements = [
            ProfileElement::Arc {
                start: v(0.0, 0.0),
                end: v(4.0, 4.0),
                center: v(0.0, 4.0),
                direction: ArcDirection::CounterClockwise,
            },
            seg(v(4.0, 4.0), v(6.0, 4.0)),
            ProfileElement::Arc {
                start: v(6.0, 4.0),
                end: v(2.0, 0.0),
                center: v(6.0, 0.0),
                direction: ArcDirection::CounterClockwise,
            },
        ];
        let found = first_crossing(&elements).expect("arcs 0 and 2 cross");
        assert_eq!((found.first, found.second), (0, 2));
        assert!(
            (found.at.x - 3.961).abs() < 1e-3 && (found.at.y - 3.441).abs() < 1e-3,
            "crossing reported at {:?}, expected about (3.961, 3.441)",
            found.at
        );
    }

    #[test]
    fn circles_that_meet_outside_both_sweeps_are_not_a_crossing() {
        // The case the previous test was accidentally written as, kept because
        // it is the obvious way to get an arc check wrong: test the circles and
        // forget that an arc is only part of one.
        let elements = [
            ProfileElement::Arc {
                start: v(0.0, 0.0),
                end: v(4.0, 4.0),
                center: v(0.0, 4.0),
                direction: ArcDirection::CounterClockwise,
            },
            seg(v(4.0, 4.0), v(4.0, 9.0)),
            ProfileElement::Arc {
                start: v(4.0, 9.0),
                end: v(0.0, 13.0),
                center: v(0.0, 9.0),
                direction: ArcDirection::CounterClockwise,
            },
        ];
        assert_eq!(
            first_crossing(&elements),
            None,
            "these two circles intersect, but neither intersection is on either arc"
        );
    }

    #[test]
    fn collinear_overlap_is_caught_even_without_a_proper_crossing() {
        // Two segments on the same line, overlapping. Neither straddles the
        // other, so a naive sidedness test sees nothing.
        let elements = [
            seg(v(0.0, 0.0), v(6.0, 0.0)),
            seg(v(6.0, 0.0), v(6.0, 4.0)),
            seg(v(6.0, 4.0), v(3.0, 4.0)),
            seg(v(3.0, 4.0), v(9.0, 4.0)),
        ];
        assert!(
            first_crossing(&elements).is_some(),
            "elements 2 and 3 lie on z = 4 and overlap between r = 3 and r = 6"
        );
    }

    #[test]
    fn a_single_element_can_never_cross_itself() {
        assert_eq!(first_crossing(&[seg(v(0.0, 0.0), v(3.0, 0.0))]), None);
        assert_eq!(first_crossing(&[]), None);
    }
}
