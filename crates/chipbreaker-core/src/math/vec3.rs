// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Three-dimensional `f64` vector.

use core::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::eps::EPS_NORMALIZE;

/// A 3D vector or point with `f64` components.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    /// First component.
    pub x: f64,
    /// Second component.
    pub y: f64,
    /// Third component.
    pub z: f64,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    /// The vector with all components one.
    pub const ONE: Self = Self { x: 1.0, y: 1.0, z: 1.0 };
    /// The `+X` unit vector.
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    /// The `+Y` unit vector.
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    /// The `+Z` unit vector.
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    /// Constructs a vector from its components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Constructs a vector with all components set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self { x: v, y: v, z: v }
    }

    /// Returns the components as an array, in `x, y, z` order.
    #[inline]
    #[must_use]
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Constructs a vector from an array in `x, y, z` order.
    #[inline]
    #[must_use]
    pub const fn from_array(a: [f64; 3]) -> Self {
        Self { x: a[0], y: a[1], z: a[2] }
    }

    /// Dot product, accumulated in ascending component order (`x`, then `y`,
    /// then `z`).
    ///
    /// The order is part of the API contract: floating-point addition is not
    /// associative, so changing it changes results.
    #[inline]
    #[must_use]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Cross product `self × rhs`, right-handed.
    ///
    /// **Do not use the sign of a component of this for orientation decisions**
    /// — use [`crate::predicates::orient3d`], which is exact.
    #[inline]
    #[must_use]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
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

    /// Distance between two points.
    #[inline]
    #[must_use]
    pub fn distance(self, rhs: Self) -> f64 {
        (self - rhs).length()
    }

    /// Squared distance between two points.
    #[inline]
    #[must_use]
    pub fn distance_squared(self, rhs: Self) -> f64 {
        (self - rhs).length_squared()
    }

    /// Returns the unit vector in the same direction, or `None` if `self` is too
    /// short for a direction to be meaningful (see
    /// [`crate::eps::EPS_NORMALIZE`]).
    ///
    /// Never returns a NaN-bearing vector: that is the entire point of the
    /// `Option`. A zero-length tool axis or a degenerate triangle normal must be
    /// handled by the caller, not silently propagated as NaN into a dexel field
    /// where it will poison every comparison downstream.
    #[inline]
    #[must_use]
    pub fn normalize(self) -> Option<Self> {
        let len = self.length();
        if len < EPS_NORMALIZE || !len.is_finite() {
            return None;
        }
        Some(Self::new(self.x / len, self.y / len, self.z / len))
    }

    /// Component-wise minimum.
    #[inline]
    #[must_use]
    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z))
    }

    /// Component-wise maximum.
    #[inline]
    #[must_use]
    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z))
    }

    /// Component-wise absolute value.
    #[inline]
    #[must_use]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    /// The largest component.
    #[inline]
    #[must_use]
    pub fn max_element(self) -> f64 {
        self.x.max(self.y).max(self.z)
    }

    /// The smallest component.
    #[inline]
    #[must_use]
    pub fn min_element(self) -> f64 {
        self.x.min(self.y).min(self.z)
    }

    /// Returns true if every component is finite (neither infinite nor NaN).
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Drops the `z` component.
    #[inline]
    #[must_use]
    pub const fn xy(self) -> super::Vec2 {
        super::Vec2::new(self.x, self.y)
    }
}

impl Index<usize> for Vec3 {
    type Output = f64;

    /// Panics if `i > 2`.
    #[inline]
    fn index(&self, i: usize) -> &f64 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Vec3 index out of range: {i}"),
        }
    }
}

impl IndexMut<usize> for Vec3 {
    /// Panics if `i > 2`.
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vec3 index out of range: {i}"),
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl MulAssign<f64> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

impl DivAssign<f64> for Vec3 {
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
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 8.0, 12.0);
        assert_eq!(a + b, Vec3::new(5.0, 10.0, 15.0));
        assert_eq!(b - a, Vec3::new(3.0, 6.0, 9.0));
        assert_eq!(-a, Vec3::new(-1.0, -2.0, -3.0));
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(2.0 * a, Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(b / 4.0, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn cross_is_right_handed() {
        assert_eq!(Vec3::X.cross(Vec3::Y), Vec3::Z);
        assert_eq!(Vec3::Y.cross(Vec3::Z), Vec3::X);
        assert_eq!(Vec3::Z.cross(Vec3::X), Vec3::Y);
        // Anti-commutative.
        assert_eq!(Vec3::Y.cross(Vec3::X), -Vec3::Z);
        // Orthogonal to both inputs.
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(-4.0, 5.0, 6.0);
        let c = a.cross(b);
        assert_eq!(c.dot(a), 0.0);
        assert_eq!(c.dot(b), 0.0);
    }

    #[test]
    fn lengths_and_distances() {
        let a = Vec3::new(2.0, 3.0, 6.0);
        assert_eq!(a.length_squared(), 49.0);
        assert_eq!(a.length(), 7.0);
        assert_eq!(Vec3::ZERO.distance(a), 7.0);
        assert_eq!(Vec3::ZERO.distance_squared(a), 49.0);
    }

    #[test]
    fn normalize_rejects_degenerate_input() {
        let n = Vec3::new(2.0, 3.0, 6.0).normalize().expect("finite non-zero");
        assert!((n.length() - 1.0).abs() < 1e-15);
        assert_eq!(Vec3::ZERO.normalize(), None);
        assert_eq!(Vec3::new(0.0, f64::INFINITY, 0.0).normalize(), None);
        assert_eq!(Vec3::new(0.0, 0.0, f64::NAN).normalize(), None);
    }

    #[test]
    fn component_helpers() {
        let a = Vec3::new(-1.0, 5.0, 2.0);
        let b = Vec3::new(3.0, -2.0, 2.0);
        assert_eq!(a.min(b), Vec3::new(-1.0, -2.0, 2.0));
        assert_eq!(a.max(b), Vec3::new(3.0, 5.0, 2.0));
        assert_eq!(a.abs(), Vec3::new(1.0, 5.0, 2.0));
        assert_eq!(a.max_element(), 5.0);
        assert_eq!(a.min_element(), -1.0);
        assert_eq!(a.xy(), crate::math::Vec2::new(-1.0, 5.0));
    }

    #[test]
    fn indexing_matches_fields() {
        let mut v = Vec3::new(7.0, 9.0, 11.0);
        assert_eq!([v[0], v[1], v[2]], [7.0, 9.0, 11.0]);
        v[2] = 1.0;
        assert_eq!(v.z, 1.0);
        assert_eq!(Vec3::from_array(v.to_array()), v);
    }
}
