// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Adaptive-precision exact geometric predicates.
//!
//! # Why these exist
//!
//! The naive way to ask "is point `c` left of the line `ab`?" is to compute a
//! determinant in `f64` and test its sign. That test is wrong whenever the
//! determinant is smaller than its own rounding error, which is exactly the case
//! for the near-degenerate configurations that dominate real machining data:
//! coplanar triangle fans, tool paths tangent to a surface, arcs discretised
//! onto a grid. A single inconsistent answer — `c` reported left of `ab` but
//! `a` reported left of `bc` — corrupts topology, and the resulting failure
//! surfaces thousands of operations later as a hole in a mesh.
//!
//! Shewchuk's adaptive predicates fix this: they compute a fast `f64` estimate
//! with a rigorous error bound, and only when the estimate cannot be trusted do
//! they escalate to exact expansion arithmetic. The answer is always the
//! mathematically correct sign; the cost is only paid on the cases that need it.
//!
//! # Rule
//!
//! **Never test the sign of a raw `f64` determinant for an orientation
//! question.** Use these. `Vec3::cross` and friends exist for computing
//! directions and magnitudes, not for deciding sidedness.
//!
//! # Backing implementation
//!
//! Currently the [`robust`] crate, a direct port of Shewchuk's C. It is reached
//! only through the [`Predicates`] trait and [`Orientation`] enum defined here,
//! so swapping it — for a vendored copy, or for an interval-filtered
//! implementation with different performance characteristics — touches this file
//! and nothing else.
//!
//! # Range limits
//!
//! Adaptive predicates are exact only while no *intermediate product* overflows.
//! See [`MAX_SAFE_ORIENT_COORD`] and [`MAX_SAFE_LIFTED_COORD`]; these are real
//! limits, not theoretical ones, and the corpus tests right up against them.

pub mod corpus;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Vec2, Vec3};

/// The coordinate magnitudes for which a given predicate is exact.
///
/// See [`ORIENT2D_COORDS`] and friends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoordRange {
    /// Smallest non-zero magnitude. Below this, intermediate products underflow
    /// into the subnormal range and lose the low-order bits that Shewchuk's
    /// expansion arithmetic depends on.
    pub min: f64,
    /// Largest magnitude. Above this, intermediate products overflow to
    /// infinity and the determinant becomes NaN.
    pub max: f64,
    /// Degree of the determinant as a polynomial in the input coordinates.
    pub degree: u32,
}

impl CoordRange {
    /// True if `v` is exactly zero or lies within the range.
    #[inline]
    #[must_use]
    pub fn contains(&self, v: f64) -> bool {
        v == 0.0 || (v.abs() >= self.min && v.abs() <= self.max)
    }

    /// True if every coordinate of `coords` is in range.
    #[must_use]
    pub fn contains_all(&self, coords: &[f64]) -> bool {
        coords.iter().all(|&v| self.contains(v))
    }
}

/// Coordinate range within which [`orient2d`] is exact.
///
/// # Why there is a range at all
///
/// Adaptive predicates are exact *provided no intermediate over- or underflows*.
/// Shewchuk's expansion arithmetic represents an exact product as an unevaluated
/// sum of `f64` values; that representation only works while every partial
/// product is a normal `f64`. Overflow turns a determinant into NaN; underflow
/// into the subnormal range silently discards the low-order term, which is the
/// term that carries the answer in exactly the near-degenerate cases these
/// predicates exist for.
///
/// # Where these numbers come from
///
/// They are **measured, not derived**. A first-principles estimate gets the
/// overflow end roughly right (`c^degree` must stay below `1.798e308`) but is
/// badly wrong at the underflow end, because what underflows first is not the
/// product but the low-order *error term* of the expansion — some `2^-106`
/// smaller. The published bounds below are taken from
/// `published_coord_ranges_are_inside_the_measured_exact_band` in
/// `tests/exact_predicates.rs`, which scans decade by decade against the exact
/// oracle to find the contiguous band where the predicate is exact, and asserts
/// that these constants sit at least a decade inside it on both sides. Change a
/// constant and that test tells you whether you were right.
///
/// For reference, the measured bands are `orient2d` `1e-153..1e153`, `orient3d`
/// `1e-92..1e102`, `incircle` `1e-65..1e76`, and `insphere` `1e-61..1e61`.
///
/// # This is a real limit, not a theoretical one
///
/// The specification for this unit asked for corpus coverage "near
/// `f64::MAX_EXP`, where naive determinants overflow". That turns out not to be
/// possible: at those magnitudes the *adaptive* predicate overflows too, and
/// returns NaN rather than a sign. There is no coordinate range in which the
/// naive determinant overflows but the exact one survives. What the corpus tests
/// instead is the band just inside these bounds, where the naive determinant is
/// finite but has lost every significant bit to cancellation, and the adaptive
/// predicate is still exact.
///
/// For scale, `1e150` mm is some 1e147 metres. No machining workspace comes near
/// it; the bound is documented because a corrupted transform or an uninitialised
/// coordinate can produce such values, and the resulting failure should be
/// comprehensible rather than mysterious.
pub const ORIENT2D_COORDS: CoordRange = CoordRange {
    min: 1e-150,
    max: 1e150,
    degree: 2,
};

/// Coordinate range within which [`orient3d`] is exact.
///
/// A degree-3 determinant, so intermediates are `O(c^3)` and the range is
/// correspondingly narrower than [`ORIENT2D_COORDS`]. Note the asymmetry: the
/// underflow end bites nearly a decade sooner than the naive `c^3` argument
/// predicts. See [`ORIENT2D_COORDS`] for the full explanation.
pub const ORIENT3D_COORDS: CoordRange = CoordRange {
    min: 1e-90,
    max: 1e100,
    degree: 3,
};

/// Coordinate range within which [`incircle`] is exact.
///
/// Degree 4: the lifted coordinate is a squared distance (`O(c^2)`) multiplied
/// by a 2x2 minor (`O(c^2)`). See [`ORIENT2D_COORDS`].
pub const INCIRCLE_COORDS: CoordRange = CoordRange {
    min: 1e-63,
    max: 1e74,
    degree: 4,
};

/// Coordinate range within which [`insphere`] is exact.
///
/// Degree 5, and therefore the narrowest range of the four: a squared distance
/// (`O(c^2)`) multiplied by a 3x3 minor (`O(c^3)`). See [`ORIENT2D_COORDS`].
pub const INSPHERE_COORDS: CoordRange = CoordRange {
    min: 1e-59,
    max: 1e59,
    degree: 5,
};

/// The result of an exact geometric predicate.
///
/// Three-valued on purpose. A `bool` would force the degenerate case to be
/// folded into one of the two branches at the point of computation, where the
/// caller has the least context to decide which; an `f64` sign would invite
/// exactly the `> 0.0` comparison these predicates exist to eliminate.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Orientation {
    /// The determinant is exactly negative.
    Negative = -1,
    /// The determinant is exactly zero: the configuration is degenerate
    /// (collinear, coplanar, cocircular, or cospherical).
    Zero = 0,
    /// The determinant is exactly positive.
    Positive = 1,
}

impl Orientation {
    /// Classifies the sign of an exactly-computed determinant.
    ///
    /// # Panics
    /// Panics if `det` is NaN. An exact predicate cannot produce NaN from finite
    /// inputs, so this means non-finite coordinates reached the predicate, which
    /// is a bug upstream and must not be silently mapped onto [`Self::Zero`].
    #[inline]
    #[must_use]
    pub fn from_determinant(det: f64) -> Self {
        assert!(
            !det.is_nan(),
            "predicate produced NaN: non-finite input coordinates"
        );
        if det > 0.0 {
            Self::Positive
        } else if det < 0.0 {
            Self::Negative
        } else {
            // Covers both +0.0 and -0.0.
            Self::Zero
        }
    }

    /// The sign as `-1`, `0`, or `1`.
    #[inline]
    #[must_use]
    pub const fn as_i8(self) -> i8 {
        self as i8
    }

    /// Swaps [`Self::Positive`] and [`Self::Negative`], leaving [`Self::Zero`]
    /// alone.
    ///
    /// Exchanging any two arguments of any of these predicates negates the
    /// determinant; that identity is a property test in this module.
    #[inline]
    #[must_use]
    pub const fn reverse(self) -> Self {
        match self {
            Self::Positive => Self::Negative,
            Self::Negative => Self::Positive,
            Self::Zero => Self::Zero,
        }
    }

    /// True for the degenerate case.
    #[inline]
    #[must_use]
    pub const fn is_zero(self) -> bool {
        matches!(self, Self::Zero)
    }

    /// The single-character form used in the corpus file: `+`, `-`, or `0`.
    #[inline]
    #[must_use]
    pub const fn as_char(self) -> char {
        match self {
            Self::Positive => '+',
            Self::Negative => '-',
            Self::Zero => '0',
        }
    }

    /// Parses the single-character corpus form.
    #[inline]
    #[must_use]
    pub const fn from_char(c: char) -> Option<Self> {
        match c {
            '+' => Some(Self::Positive),
            '-' => Some(Self::Negative),
            '0' => Some(Self::Zero),
            _ => None,
        }
    }
}

impl Hashable for Orientation {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.i64(i64::from(self.as_i8()));
    }
}

/// The four exact predicates, behind a swappable implementation.
///
/// Takes `&self` rather than being a set of associated functions so that an
/// implementation can be selected at run time through `&dyn Predicates` — the
/// extractor's
/// dual contouring will want to A/B a filtered implementation against this one
/// on real data without a recompile.
pub trait Predicates {
    /// Sidedness of `c` relative to the directed line `a -> b`.
    ///
    /// [`Orientation::Positive`] when `a`, `b`, `c` are counterclockwise (`c` is
    /// to the **left** of `a -> b`), [`Orientation::Negative`] when clockwise,
    /// [`Orientation::Zero`] when collinear.
    fn orient2d(&self, a: Vec2, b: Vec2, c: Vec2) -> Orientation;

    /// Sidedness of `d` relative to the oriented plane through `a`, `b`, `c`.
    ///
    /// [`Orientation::Positive`] when `d` lies **below** the plane, meaning
    /// `a`, `b`, `c` appear counterclockwise when viewed from above it;
    /// equivalently, the sign of `det[a - d; b - d; c - d]`.
    /// [`Orientation::Zero`] when all four points are coplanar.
    fn orient3d(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Orientation;

    /// Whether `d` lies inside the circle through `a`, `b`, `c`.
    ///
    /// [`Orientation::Positive`] means **inside**, [`Orientation::Zero`] means
    /// cocircular. Requires `a`, `b`, `c` to be in counterclockwise order; if
    /// they are clockwise the sense of the result is inverted.
    fn incircle(&self, a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Orientation;

    /// Whether `e` lies inside the sphere through `a`, `b`, `c`, `d`.
    ///
    /// [`Orientation::Positive`] means **inside**, [`Orientation::Zero`] means
    /// cospherical. Requires `a`, `b`, `c`, `d` to be positively oriented (that
    /// is, `orient3d(a, b, c, d)` is [`Orientation::Positive`]); if they are
    /// negatively oriented the sense of the result is inverted.
    fn insphere(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, e: Vec3) -> Orientation;
}

/// The default backend: Shewchuk's adaptive-precision predicates, via the
/// [`robust`] crate.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Adaptive;

/// The instance used by the free functions in this module.
pub const ADAPTIVE: Adaptive = Adaptive;

#[inline]
fn c2(v: Vec2) -> robust::Coord<f64> {
    robust::Coord { x: v.x, y: v.y }
}

#[inline]
fn c3(v: Vec3) -> robust::Coord3D<f64> {
    robust::Coord3D {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

/// Debug-only guard that the inputs are inside the predicate's exact range.
///
/// A smoke alarm, not a proof: staying in range is necessary for exactness but
/// not sufficient, since two in-range coordinates can still cancel down to a
/// difference small enough for their product to go subnormal. It catches the
/// failure that actually happens in practice — garbage coordinates from a
/// corrupted transform — and it costs nothing in release.
#[inline]
fn debug_check_range(name: &str, range: &CoordRange, coords: &[f64]) {
    debug_assert!(
        range.contains_all(coords),
        "{name} called with coordinates outside its exact range \
         [{:e}, {:e}]: {coords:?}. Outside this band the adaptive predicate \
         over- or underflows and its sign is meaningless.",
        range.min,
        range.max
    );
}

impl Predicates for Adaptive {
    #[inline]
    fn orient2d(&self, a: Vec2, b: Vec2, c: Vec2) -> Orientation {
        debug_check_range(
            "orient2d",
            &ORIENT2D_COORDS,
            &[a.x, a.y, b.x, b.y, c.x, c.y],
        );
        Orientation::from_determinant(robust::orient2d(c2(a), c2(b), c2(c)))
    }

    #[inline]
    fn orient3d(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Orientation {
        debug_check_range(
            "orient3d",
            &ORIENT3D_COORDS,
            &[a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z],
        );
        Orientation::from_determinant(robust::orient3d(c3(a), c3(b), c3(c), c3(d)))
    }

    #[inline]
    fn incircle(&self, a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Orientation {
        debug_check_range(
            "incircle",
            &INCIRCLE_COORDS,
            &[a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y],
        );
        Orientation::from_determinant(robust::incircle(c2(a), c2(b), c2(c), c2(d)))
    }

    #[inline]
    fn insphere(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, e: Vec3) -> Orientation {
        debug_check_range(
            "insphere",
            &INSPHERE_COORDS,
            &[
                a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x, d.y, d.z, e.x, e.y, e.z,
            ],
        );
        Orientation::from_determinant(robust::insphere(c3(a), c3(b), c3(c), c3(d), c3(e)))
    }
}

/// Sidedness of `c` relative to the directed line `a -> b`. See
/// [`Predicates::orient2d`].
#[inline]
#[must_use]
pub fn orient2d(a: Vec2, b: Vec2, c: Vec2) -> Orientation {
    ADAPTIVE.orient2d(a, b, c)
}

/// Sidedness of `d` relative to the plane through `a`, `b`, `c`. See
/// [`Predicates::orient3d`].
#[inline]
#[must_use]
pub fn orient3d(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Orientation {
    ADAPTIVE.orient3d(a, b, c, d)
}

/// Whether `d` lies inside the circle through `a`, `b`, `c`. See
/// [`Predicates::incircle`].
#[inline]
#[must_use]
pub fn incircle(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Orientation {
    ADAPTIVE.incircle(a, b, c, d)
}

/// Whether `e` lies inside the sphere through `a`, `b`, `c`, `d`. See
/// [`Predicates::insphere`].
#[inline]
#[must_use]
pub fn insphere(a: Vec3, b: Vec3, c: Vec3, d: Vec3, e: Vec3) -> Orientation {
    ADAPTIVE.insphere(a, b, c, d, e)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orient2d_sign_convention() {
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(1.0, 0.0);
        assert_eq!(orient2d(a, b, Vec2::new(0.0, 1.0)), Orientation::Positive);
        assert_eq!(orient2d(a, b, Vec2::new(0.0, -1.0)), Orientation::Negative);
        assert_eq!(orient2d(a, b, Vec2::new(2.0, 0.0)), Orientation::Zero);
    }

    #[test]
    fn orient3d_sign_convention() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        let c = Vec3::new(0.0, 0.0, 1.0);
        // det[a-d; b-d; c-d] with d at the origin is the identity: +1.
        assert_eq!(orient3d(a, b, c, Vec3::ZERO), Orientation::Positive);
        // a, b, c span the plane x + y + z = 1.
        assert_eq!(
            orient3d(a, b, c, Vec3::new(1.0, 1.0, -1.0)),
            Orientation::Zero
        );
        assert_eq!(
            orient3d(a, b, c, Vec3::new(-3.0, 2.0, 2.0)),
            Orientation::Zero
        );
        // The origin has x + y + z = 0 < 1; anything with a larger sum is on the
        // far side.
        assert_eq!(orient3d(a, b, c, Vec3::splat(2.0)), Orientation::Negative);
    }

    #[test]
    fn incircle_sign_convention() {
        // Counterclockwise triangle on the unit circle.
        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        let c = Vec2::new(-1.0, 0.0);
        assert_eq!(orient2d(a, b, c), Orientation::Positive);
        assert_eq!(incircle(a, b, c, Vec2::ZERO), Orientation::Positive);
        assert_eq!(incircle(a, b, c, Vec2::new(0.0, -1.0)), Orientation::Zero);
        assert_eq!(
            incircle(a, b, c, Vec2::new(0.0, -2.0)),
            Orientation::Negative
        );
    }

    #[test]
    fn insphere_sign_convention() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        let c = Vec3::new(0.0, 0.0, 1.0);
        let d = Vec3::ZERO;
        assert_eq!(
            orient3d(a, b, c, d),
            Orientation::Positive,
            "must be positively oriented"
        );
        // Circumcentre of these four points.
        assert_eq!(
            insphere(a, b, c, d, Vec3::splat(0.5)),
            Orientation::Positive
        );
        assert_eq!(
            insphere(a, b, c, d, Vec3::new(1.0, 1.0, 0.0)),
            Orientation::Zero
        );
        assert_eq!(
            insphere(a, b, c, d, Vec3::splat(10.0)),
            Orientation::Negative
        );
    }

    #[test]
    fn swapping_two_arguments_negates_the_result() {
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(3.0, 1.0);
        let c = Vec2::new(1.0, 7.0);
        assert_eq!(orient2d(a, b, c), orient2d(b, a, c).reverse());
        assert_eq!(orient2d(a, b, c), orient2d(a, c, b).reverse());

        let p = Vec3::new(0.0, 0.0, 0.0);
        let q = Vec3::new(1.0, 2.0, 3.0);
        let r = Vec3::new(-4.0, 1.0, 0.5);
        let s = Vec3::new(2.0, -3.0, 6.0);
        assert_eq!(orient3d(p, q, r, s), orient3d(q, p, r, s).reverse());
        assert_eq!(orient3d(p, q, r, s), orient3d(p, q, s, r).reverse());
    }

    #[test]
    fn exactness_where_naive_f64_fails() {
        // Three points that are exactly collinear but whose naive determinant
        // evaluates non-zero: the classic Shewchuk demonstration.
        let a = Vec2::new(0.5, 0.5);
        let b = Vec2::new(12.0, 12.0);
        let c = Vec2::new(24.0, 24.0);
        assert_eq!(orient2d(a, b, c), Orientation::Zero);

        // One ULP off the line must be detected, not rounded away.
        let c_up = Vec2::new(24.0, 24.0_f64.next_up());
        assert_eq!(orient2d(a, b, c_up), Orientation::Positive);
        let c_down = Vec2::new(24.0, 24.0_f64.next_down());
        assert_eq!(orient2d(a, b, c_down), Orientation::Negative);
    }

    #[test]
    fn orientation_helpers() {
        assert_eq!(Orientation::from_determinant(0.0), Orientation::Zero);
        assert_eq!(Orientation::from_determinant(-0.0), Orientation::Zero);
        assert_eq!(Orientation::from_determinant(1e-320), Orientation::Positive);
        assert_eq!(Orientation::Positive.reverse(), Orientation::Negative);
        assert_eq!(Orientation::Zero.reverse(), Orientation::Zero);
        assert_eq!(Orientation::Negative.as_i8(), -1);
        assert!(Orientation::Zero.is_zero());
        for o in [
            Orientation::Positive,
            Orientation::Negative,
            Orientation::Zero,
        ] {
            assert_eq!(Orientation::from_char(o.as_char()), Some(o));
        }
        assert_eq!(Orientation::from_char('x'), None);
    }

    #[test]
    #[should_panic(expected = "predicate produced NaN")]
    fn nan_determinant_is_not_silently_degenerate() {
        let _ = Orientation::from_determinant(f64::NAN);
    }

    #[test]
    fn trait_object_is_usable() {
        let p: &dyn Predicates = &ADAPTIVE;
        assert_eq!(
            p.orient2d(Vec2::ZERO, Vec2::X, Vec2::Y),
            Orientation::Positive
        );
    }
}
