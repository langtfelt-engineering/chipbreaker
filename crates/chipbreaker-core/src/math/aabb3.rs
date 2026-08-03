// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Axis-aligned bounding box in three dimensions.

use crate::math::Vec3;

/// A coordinate axis.
///
/// Ordering is `X < Y < Z`, which is also the tie-breaking order used by
/// [`Aabb3::longest_axis`].
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Axis {
    /// The X axis, index 0.
    X = 0,
    /// The Y axis, index 1.
    Y = 1,
    /// The Z axis, index 2.
    Z = 2,
}

impl Axis {
    /// All three axes in ascending order.
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    /// The component index this axis selects (`0`, `1`, or `2`).
    #[inline]
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// The unit vector along this axis.
    #[must_use]
    pub const fn direction(self) -> Vec3 {
        match self {
            Self::X => Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Self::Y => Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Self::Z => Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        }
    }

    /// The other two axes, in right-handed cyclic order, then this one.
    ///
    /// `Z` gives `[X, Y, Z]`, `X` gives `[Y, Z, X]`, `Y` gives `[Z, X, Y]`. The
    /// convention matches [`crate::toolpath::ArcPlane`], deliberately: two
    /// places that both name "the XZ plane" and disagree about its hand are a
    /// sign error waiting to happen.
    ///
    /// Used by the dexel lattice, where the first two index the grid and the
    /// third is the direction the rays run.
    #[must_use]
    pub const fn cyclic(self) -> [usize; 3] {
        match self {
            Self::X => [1, 2, 0],
            Self::Y => [2, 0, 1],
            Self::Z => [0, 1, 2],
        }
    }

    /// Lowercase name, as used in reports and file headers.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::X => "x",
            Self::Y => "y",
            Self::Z => "z",
        }
    }

    /// Parses a name, case-insensitively.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        match name.to_ascii_lowercase().as_str() {
            "x" => Some(Self::X),
            "y" => Some(Self::Y),
            "z" => Some(Self::Z),
            _ => None,
        }
    }
}

impl core::fmt::Display for Axis {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl crate::golden::Hashable for Axis {
    fn hash_canonical(&self, h: &mut crate::golden::CanonicalHash) {
        h.str(self.as_str());
    }
}

/// An axis-aligned bounding box.
///
/// The **empty** box is represented by an inverted interval (`min = +inf`,
/// `max = -inf`), which makes [`Aabb3::EMPTY`] the identity element for
/// [`Aabb3::union`] and lets a box be accumulated by folding without a special
/// first-element case. This gets heavy use in U2's BVH build.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb3 {
    /// Lower corner. Greater than `max` on some axis iff the box is empty.
    pub min: Vec3,
    /// Upper corner.
    pub max: Vec3,
}

impl Default for Aabb3 {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Aabb3 {
    /// The empty box: the identity for [`Aabb3::union`].
    pub const EMPTY: Self = Self {
        min: Vec3::splat(f64::INFINITY),
        max: Vec3::splat(f64::NEG_INFINITY),
    };

    /// The box covering all of space.
    pub const UNBOUNDED: Self = Self {
        min: Vec3::splat(f64::NEG_INFINITY),
        max: Vec3::splat(f64::INFINITY),
    };

    /// Constructs a box from explicit corners, **without** reordering them.
    ///
    /// Passing `min > max` on any axis yields an empty box, which is usually a
    /// bug at the call site; use [`Aabb3::from_min_max`] if the corners might
    /// arrive in either order.
    #[inline]
    #[must_use]
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Constructs a box from two corners in any order.
    #[inline]
    #[must_use]
    pub fn from_min_max(a: Vec3, b: Vec3) -> Self {
        Self {
            min: a.min(b),
            max: a.max(b),
        }
    }

    /// The degenerate box containing exactly one point.
    #[inline]
    #[must_use]
    pub const fn from_point(p: Vec3) -> Self {
        Self { min: p, max: p }
    }

    /// The box bounding all of `points`. Returns [`Aabb3::EMPTY`] for an empty
    /// slice.
    ///
    /// Folds in slice order. Because `min`/`max` are exact operations on `f64`,
    /// the result does not actually depend on that order — but the order is
    /// fixed anyway so that a future change to this function is visibly a
    /// change.
    #[must_use]
    pub fn from_points(points: &[Vec3]) -> Self {
        let mut acc = Self::EMPTY;
        for &p in points {
            acc = acc.union_point(p);
        }
        acc
    }

    /// True if the box contains no points at all.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// The smallest box containing both inputs.
    #[inline]
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// The smallest box containing this box and `p`.
    #[inline]
    #[must_use]
    pub fn union_point(&self, p: Vec3) -> Self {
        Self {
            min: self.min.min(p),
            max: self.max.max(p),
        }
    }

    /// The overlap of the two boxes, which is empty if they do not overlap.
    #[inline]
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
        let candidate = Self {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        };
        if candidate.is_empty() {
            Self::EMPTY
        } else {
            candidate
        }
    }

    /// True if the two boxes share at least one point.
    #[inline]
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        !self.intersection(other).is_empty()
    }

    /// True if `p` lies inside the box, boundary included.
    #[inline]
    #[must_use]
    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// True if `other` lies entirely inside this box. An empty `other` is
    /// contained by everything.
    #[inline]
    #[must_use]
    pub fn contains_aabb(&self, other: &Self) -> bool {
        other.is_empty() || (self.contains(other.min) && self.contains(other.max))
    }

    /// Grows the box by `margin` on every side. A negative margin shrinks it,
    /// possibly to empty.
    ///
    /// Returns [`Aabb3::EMPTY`] unchanged: growing nothing still yields nothing.
    #[inline]
    #[must_use]
    pub fn expand(&self, margin: f64) -> Self {
        if self.is_empty() {
            return Self::EMPTY;
        }
        let m = Vec3::splat(margin);
        let candidate = Self {
            min: self.min - m,
            max: self.max + m,
        };
        if candidate.is_empty() {
            Self::EMPTY
        } else {
            candidate
        }
    }

    /// The extent along each axis. Zero on every axis for an empty box.
    #[inline]
    #[must_use]
    pub fn extent(&self) -> Vec3 {
        if self.is_empty() {
            return Vec3::ZERO;
        }
        self.max - self.min
    }

    /// The midpoint of the box.
    ///
    /// Computed as `min/2 + max/2`, not `(min + max) / 2` and not
    /// `min + (max - min) / 2`. Both of those overflow to infinity for a box
    /// whose corners approach `f64::MAX`, and the predicate corpus deliberately
    /// exercises coordinates at that scale. Halving is exact for normal `f64`,
    /// so this form is also one rounding cheaper than the `max - min` version.
    #[inline]
    #[must_use]
    pub fn center(&self) -> Vec3 {
        if self.is_empty() {
            return Vec3::ZERO;
        }
        self.min / 2.0 + self.max / 2.0
    }

    /// Total surface area, `2 * (xy + yz + zx)`, summed in that order. Zero for
    /// an empty box.
    ///
    /// This is the cost metric for the surface area heuristic in U2's BVH build.
    #[inline]
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let e = self.extent();
        2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
    }

    /// Volume of the box. Zero for an empty box.
    #[inline]
    #[must_use]
    pub fn volume(&self) -> f64 {
        let e = self.extent();
        e.x * e.y * e.z
    }

    /// The axis along which the box is longest.
    ///
    /// Ties break toward the lower axis (`X` before `Y` before `Z`). A
    /// deterministic tie-break matters: BVH split decisions feed directly into
    /// traversal order, and an arbitrary one would make tree shape depend on
    /// comparison implementation details. Returns [`Axis::X`] for an empty box.
    #[inline]
    #[must_use]
    pub fn longest_axis(&self) -> Axis {
        let e = self.extent();
        if e.x >= e.y && e.x >= e.z {
            Axis::X
        } else if e.y >= e.z {
            Axis::Y
        } else {
            Axis::Z
        }
    }

    /// Returns true if every corner component is finite. An empty box is not
    /// finite, since its corners are infinite by construction.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.min.is_finite() && self.max.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit() -> Aabb3 {
        Aabb3::new(Vec3::ZERO, Vec3::ONE)
    }

    #[test]
    fn empty_is_the_union_identity() {
        assert!(Aabb3::EMPTY.is_empty());
        assert_eq!(Aabb3::EMPTY.union(&unit()), unit());
        assert_eq!(unit().union(&Aabb3::EMPTY), unit());
        assert_eq!(Aabb3::EMPTY.union(&Aabb3::EMPTY), Aabb3::EMPTY);
        assert_eq!(Aabb3::from_points(&[]), Aabb3::EMPTY);
    }

    #[test]
    fn from_points_bounds_everything() {
        let pts = [
            Vec3::new(1.0, -2.0, 3.0),
            Vec3::new(-4.0, 5.0, -6.0),
            Vec3::new(0.0, 0.0, 0.0),
        ];
        let b = Aabb3::from_points(&pts);
        assert_eq!(b.min, Vec3::new(-4.0, -2.0, -6.0));
        assert_eq!(b.max, Vec3::new(1.0, 5.0, 3.0));
        for p in pts {
            assert!(b.contains(p));
        }
    }

    #[test]
    fn from_min_max_reorders_but_new_does_not() {
        let a = Vec3::new(5.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 5.0, 0.0);
        assert_eq!(Aabb3::from_min_max(a, b).min, Vec3::ZERO);
        assert!(Aabb3::new(a, b).is_empty());
    }

    #[test]
    fn intersection_and_intersects_agree() {
        let a = unit();
        let overlapping = Aabb3::new(Vec3::splat(0.5), Vec3::splat(2.0));
        assert_eq!(
            a.intersection(&overlapping),
            Aabb3::new(Vec3::splat(0.5), Vec3::ONE)
        );
        assert!(a.intersects(&overlapping));

        let disjoint = Aabb3::new(Vec3::splat(2.0), Vec3::splat(3.0));
        assert_eq!(a.intersection(&disjoint), Aabb3::EMPTY);
        assert!(!a.intersects(&disjoint));
        // Touching boxes share a face, so they do intersect.
        let touching = Aabb3::new(Vec3::ONE, Vec3::splat(2.0));
        assert!(a.intersects(&touching));
    }

    #[test]
    fn containment() {
        let a = unit();
        assert!(a.contains(Vec3::ZERO));
        assert!(a.contains(Vec3::ONE));
        assert!(a.contains(Vec3::splat(0.5)));
        assert!(!a.contains(Vec3::splat(1.5)));
        assert!(a.contains_aabb(&Aabb3::new(Vec3::splat(0.25), Vec3::splat(0.75))));
        assert!(a.contains_aabb(&Aabb3::EMPTY));
        assert!(!a.contains_aabb(&Aabb3::new(Vec3::ZERO, Vec3::splat(2.0))));
    }

    #[test]
    fn expand_grows_and_shrinks() {
        let a = unit();
        assert_eq!(
            a.expand(1.0),
            Aabb3::new(Vec3::splat(-1.0), Vec3::splat(2.0))
        );
        // Shrinking past the middle empties the box rather than inverting it.
        assert_eq!(a.expand(-1.0), Aabb3::EMPTY);
        assert_eq!(Aabb3::EMPTY.expand(5.0), Aabb3::EMPTY);
    }

    #[test]
    fn measurements() {
        let a = Aabb3::new(Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(a.extent(), Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(a.center(), Vec3::new(0.5, 1.0, 1.5));
        assert_eq!(a.volume(), 6.0);
        // 2 * (1*2 + 2*3 + 3*1) = 22
        assert_eq!(a.surface_area(), 22.0);
        assert_eq!(Aabb3::EMPTY.surface_area(), 0.0);
        assert_eq!(Aabb3::EMPTY.volume(), 0.0);
        assert_eq!(Aabb3::EMPTY.extent(), Vec3::ZERO);
    }

    #[test]
    fn center_does_not_overflow_at_extreme_coordinates() {
        // (min + max) / 2 would produce +inf here.
        let huge = Aabb3::new(Vec3::splat(-f64::MAX), Vec3::splat(f64::MAX));
        assert!(huge.center().is_finite());
    }

    #[test]
    fn longest_axis_breaks_ties_toward_lower_axis() {
        assert_eq!(
            Aabb3::new(Vec3::ZERO, Vec3::new(1.0, 5.0, 2.0)).longest_axis(),
            Axis::Y
        );
        assert_eq!(
            Aabb3::new(Vec3::ZERO, Vec3::new(1.0, 2.0, 5.0)).longest_axis(),
            Axis::Z
        );
        // A cube is a three-way tie: X wins.
        assert_eq!(unit().longest_axis(), Axis::X);
        // A two-way tie between Y and Z: Y wins.
        assert_eq!(
            Aabb3::new(Vec3::ZERO, Vec3::new(1.0, 5.0, 5.0)).longest_axis(),
            Axis::Y
        );
        assert_eq!(Aabb3::EMPTY.longest_axis(), Axis::X);
    }

    #[test]
    fn axis_indices() {
        assert_eq!(Axis::ALL.map(Axis::index), [0, 1, 2]);
        assert!(Axis::X < Axis::Y && Axis::Y < Axis::Z);
    }
}
