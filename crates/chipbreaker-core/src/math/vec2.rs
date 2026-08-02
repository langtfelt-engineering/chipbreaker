// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Two-dimensional `f64` vector.

use core::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

use crate::eps::EPS_NORMALIZE;

/// A 2D vector or point with `f64` components.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// First component.
    pub x: f64,
    /// Second component.
    pub y: f64,
}

impl Vec2 {
    /// The zero vector.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };
    /// The vector with both components one.
    pub const ONE: Self = Self { x: 1.0, y: 1.0 };
    /// The `+X` unit vector.
    pub const X: Self = Self { x: 1.0, y: 0.0 };
    /// The `+Y` unit vector.
    pub const Y: Self = Self { x: 0.0, y: 1.0 };

    /// Constructs a vector from its components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Constructs a vector with both components set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self { x: v, y: v }
    }

    /// Returns the components as an array, in `x, y` order.
    #[inline]
    #[must_use]
    pub const fn to_array(self) -> [f64; 2] {
        [self.x, self.y]
    }

    /// Constructs a vector from an array in `x, y` order.
    #[inline]
    #[must_use]
    pub const fn from_array(a: [f64; 2]) -> Self {
        Self { x: a[0], y: a[1] }
    }

    /// Dot product, accumulated in ascending component order.
    #[inline]
    #[must_use]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// The scalar cross product (perp-dot): `self.x * rhs.y - self.y * rhs.x`.
    ///
    /// Sign indicates the turn direction from `self` to `rhs`. **Do not use the
    /// sign of this for orientation decisions** — use
    /// [`crate::predicates::orient2d`], which is exact.
    #[inline]
    #[must_use]
    pub fn cross(self, rhs: Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    /// Squared Euclidean length. Preferred over [`Self::length`] when only
    /// comparing magnitudes, since it avoids a `sqrt`.
    #[inline]
    #[must_use]
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    /// Euclidean length.
    ///
    /// `sqrt` is correctly rounded per IEEE-754, so this is bit-identical on
    /// every target.
    #[inline]
    #[must_use]
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Returns the unit vector in the same direction, or `None` if `self` is too
    /// short for a direction to be meaningful (see
    /// [`crate::eps::EPS_NORMALIZE`]).
    ///
    /// Never returns a NaN-bearing vector: that is the entire point of the
    /// `Option`.
    #[inline]
    #[must_use]
    pub fn normalize(self) -> Option<Self> {
        let len = self.length();
        if len < EPS_NORMALIZE || !len.is_finite() {
            return None;
        }
        Some(Self::new(self.x / len, self.y / len))
    }

    /// Component-wise minimum.
    #[inline]
    #[must_use]
    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y))
    }

    /// Component-wise maximum.
    #[inline]
    #[must_use]
    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y))
    }

    /// Component-wise absolute value.
    #[inline]
    #[must_use]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs())
    }

    /// Returns true if every component is finite (neither infinite nor NaN).
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Index<usize> for Vec2 {
    type Output = f64;

    /// Panics if `i > 1`.
    #[inline]
    fn index(&self, i: usize) -> &f64 {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Vec2 index out of range: {i}"),
        }
    }
}

impl IndexMut<usize> for Vec2 {
    /// Panics if `i > 1`.
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Vec2 index out of range: {i}"),
        }
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s)
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;
    #[inline]
    fn mul(self, v: Vec2) -> Vec2 {
        v * self
    }
}

impl Div<f64> for Vec2 {
    type Output = Self;
    #[inline]
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl MulAssign<f64> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

impl DivAssign<f64> for Vec2 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        *self = *self / s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 8.0);
        assert_eq!(a + b, Vec2::new(5.0, 10.0));
        assert_eq!(b - a, Vec2::new(3.0, 6.0));
        assert_eq!(-a, Vec2::new(-1.0, -2.0));
        assert_eq!(a * 2.0, Vec2::new(2.0, 4.0));
        assert_eq!(2.0 * a, Vec2::new(2.0, 4.0));
        assert_eq!(b / 2.0, Vec2::new(2.0, 4.0));
    }

    #[test]
    fn products() {
        let a = Vec2::new(3.0, 4.0);
        assert_eq!(a.dot(Vec2::new(1.0, 2.0)), 11.0);
        assert_eq!(a.length_squared(), 25.0);
        assert_eq!(a.length(), 5.0);
        assert_eq!(Vec2::X.cross(Vec2::Y), 1.0);
        assert_eq!(Vec2::Y.cross(Vec2::X), -1.0);
    }

    #[test]
    fn normalize_rejects_degenerate_input() {
        assert_eq!(Vec2::new(3.0, 4.0).normalize(), Some(Vec2::new(0.6, 0.8)));
        assert_eq!(Vec2::ZERO.normalize(), None);
        // The failure mode this Option exists to prevent.
        assert_eq!(Vec2::new(f64::INFINITY, 0.0).normalize(), None);
        assert_eq!(Vec2::new(f64::NAN, 0.0).normalize(), None);
    }

    #[test]
    fn indexing_matches_fields() {
        let mut v = Vec2::new(7.0, 9.0);
        assert_eq!(v[0], 7.0);
        assert_eq!(v[1], 9.0);
        v[1] = 3.0;
        assert_eq!(v.y, 3.0);
        assert_eq!(v.to_array(), [7.0, 3.0]);
        assert_eq!(Vec2::from_array([7.0, 3.0]), v);
    }
}
