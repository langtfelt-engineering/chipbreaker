// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! A parametrised line in three dimensions.

use crate::math::Vec3;

/// A ray: an origin and a direction, parametrised by `t`.
///
/// The direction is **not** required to be unit length. A dexel ray carries an
/// unnormalized direction on purpose, so that `t` can be expressed in units of
/// the field spacing rather than in millimetres. Where unit length is required,
/// build the ray with [`Ray::new_normalized`].
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    /// The point at `t = 0`.
    pub origin: Vec3,
    /// The direction of travel; `t` advances by one direction-length per unit.
    pub direction: Vec3,
}

impl Ray {
    /// Constructs a ray, taking the direction as given.
    #[inline]
    #[must_use]
    pub const fn new(origin: Vec3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// Constructs a ray with a unit-length direction, or `None` if `direction`
    /// is too short to normalize (see [`crate::eps::EPS_NORMALIZE`]).
    #[inline]
    #[must_use]
    pub fn new_normalized(origin: Vec3, direction: Vec3) -> Option<Self> {
        Some(Self {
            origin,
            direction: direction.normalize()?,
        })
    }

    /// The point at parameter `t`: `origin + direction * t`.
    #[inline]
    #[must_use]
    pub fn at(&self, t: f64) -> Vec3 {
        self.origin + self.direction * t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn at_interpolates() {
        let r = Ray::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(2.0, 0.0, 0.0));
        assert_eq!(r.at(0.0), r.origin);
        assert_eq!(r.at(1.0), Vec3::new(3.0, 2.0, 3.0));
        assert_eq!(r.at(-0.5), Vec3::new(0.0, 2.0, 3.0));
    }

    #[test]
    fn new_normalized_rejects_degenerate_direction() {
        let r = Ray::new_normalized(Vec3::ZERO, Vec3::new(0.0, 0.0, 5.0)).expect("non-zero");
        assert_eq!(r.direction, Vec3::Z);
        assert_eq!(r.at(3.0), Vec3::new(0.0, 0.0, 3.0));
        assert!(Ray::new_normalized(Vec3::ZERO, Vec3::ZERO).is_none());
    }

    #[test]
    fn unnormalized_direction_is_preserved() {
        // Dexel rays rely on this: t is in units of the direction vector.
        let r = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 0.25));
        assert_eq!(r.direction, Vec3::new(0.0, 0.0, 0.25));
        assert_eq!(r.at(4.0), Vec3::Z);
    }
}
