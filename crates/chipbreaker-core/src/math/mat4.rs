// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! 4x4 `f64` matrix.

use core::ops::Mul;

use crate::eps::EPS_DETERMINANT;
use crate::math::{Mat3, Vec3};

/// A 4x4 matrix, stored **row-major**: `m[row][col]`.
///
/// Vectors are treated as columns, so `M * v` applies `M` to `v` and the
/// composition `A * B` applies `B` first. Translation lives in column 3.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat4 {
    /// Rows, outer index is the row.
    pub m: [[f64; 4]; 4],
}

impl Default for Mat4 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat4 {
    /// The multiplicative identity.
    pub const IDENTITY: Self = Self {
        m: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    };

    /// The all-zero matrix.
    pub const ZERO: Self = Self { m: [[0.0; 4]; 4] };

    /// Constructs from a row-major array.
    #[inline]
    #[must_use]
    pub const fn from_rows_array(m: [[f64; 4]; 4]) -> Self {
        Self { m }
    }

    /// A pure translation.
    #[inline]
    #[must_use]
    pub const fn from_translation(t: Vec3) -> Self {
        Self {
            m: [
                [1.0, 0.0, 0.0, t.x],
                [0.0, 1.0, 0.0, t.y],
                [0.0, 0.0, 1.0, t.z],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// A pure (possibly non-uniform) scale about the origin.
    #[inline]
    #[must_use]
    pub const fn from_scale(s: Vec3) -> Self {
        Self {
            m: [
                [s.x, 0.0, 0.0, 0.0],
                [0.0, s.y, 0.0, 0.0],
                [0.0, 0.0, s.z, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Embeds a 3x3 linear map, with zero translation.
    #[inline]
    #[must_use]
    pub const fn from_mat3(a: Mat3) -> Self {
        Self::from_mat3_translation(a, Vec3::ZERO)
    }

    /// Combines a 3x3 linear map with a translation: applies `a` first, then
    /// translates by `t`.
    #[inline]
    #[must_use]
    pub const fn from_mat3_translation(a: Mat3, t: Vec3) -> Self {
        let n = &a.m;
        Self {
            m: [
                [n[0][0], n[0][1], n[0][2], t.x],
                [n[1][0], n[1][1], n[1][2], t.y],
                [n[2][0], n[2][1], n[2][2], t.z],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// The upper-left 3x3 block: the linear part, with translation discarded.
    #[inline]
    #[must_use]
    pub const fn upper_left3(&self) -> Mat3 {
        let m = &self.m;
        Mat3::from_rows_array([
            [m[0][0], m[0][1], m[0][2]],
            [m[1][0], m[1][1], m[1][2]],
            [m[2][0], m[2][1], m[2][2]],
        ])
    }

    /// The translation column.
    #[inline]
    #[must_use]
    pub const fn translation(&self) -> Vec3 {
        Vec3::new(self.m[0][3], self.m[1][3], self.m[2][3])
    }

    /// Returns the transpose.
    #[must_use]
    pub fn transpose(&self) -> Self {
        let mut out = [[0.0f64; 4]; 4];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.m[j][i];
            }
        }
        Self { m: out }
    }

    /// The 3x3 minor obtained by deleting `skip_row` and `skip_col`.
    fn minor(&self, skip_row: usize, skip_col: usize) -> f64 {
        let mut sub = [[0.0f64; 3]; 3];
        let mut si = 0usize;
        for i in 0..4 {
            if i == skip_row {
                continue;
            }
            let mut sj = 0usize;
            for j in 0..4 {
                if j == skip_col {
                    continue;
                }
                sub[si][sj] = self.m[i][j];
                sj += 1;
            }
            si += 1;
        }
        Mat3::from_rows_array(sub).determinant()
    }

    /// The cofactor of entry `(i, j)`.
    fn cofactor(&self, i: usize, j: usize) -> f64 {
        let minor = self.minor(i, j);
        if (i + j).is_multiple_of(2) {
            minor
        } else {
            -minor
        }
    }

    /// Determinant, by cofactor expansion along **row 0**, summed in ascending
    /// column order.
    ///
    /// This is the readable formulation rather than the fastest one (Laplace
    /// expansion by complementary 2x2 minors saves roughly a third of the
    /// multiplies). 4x4 inversion happens once per toolpath move, not once per
    /// dexel ray, so clarity is the better trade here. Revisit if a profile ever
    /// says otherwise.
    #[must_use]
    pub fn determinant(&self) -> f64 {
        let mut acc = 0.0;
        for j in 0..4 {
            acc += self.m[0][j] * self.cofactor(0, j);
        }
        acc
    }

    /// Returns the inverse, or `None` if the matrix is singular (see
    /// [`crate::eps::EPS_DETERMINANT`]) or contains a non-finite entry.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if !det.is_finite() || det.abs() < EPS_DETERMINANT {
            return None;
        }
        // inverse = adjugate / det, and adjugate = cofactor^T.
        let mut out = [[0.0f64; 4]; 4];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.cofactor(j, i) / det;
            }
        }
        let inv = Self { m: out };
        if inv.is_finite() { Some(inv) } else { None }
    }

    /// Transforms a **point**: applies the linear part *and* the translation,
    /// then divides through by the homogeneous `w`.
    ///
    /// Every transform in Chipbreaker is affine, so `w` is exactly `1.0` and the
    /// division is exact and free. The division is nevertheless written out
    /// rather than skipped, so that a non-affine matrix arriving from a future
    /// caller produces a mathematically correct answer instead of a silently
    /// wrong one.
    ///
    /// # Panics
    /// In debug builds, panics if `w` is zero or non-finite — that means the
    /// caller built a degenerate matrix, which is a bug upstream, not a case to
    /// be handled here.
    #[inline]
    #[must_use]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        let m = &self.m;
        let x = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
        let y = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
        let z = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
        let w = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];
        debug_assert!(
            w.is_finite() && w != 0.0,
            "Mat4::transform_point on a degenerate matrix (w = {w})"
        );
        Vec3::new(x / w, y / w, z / w)
    }

    /// Transforms a **direction**: applies the linear part only, ignoring
    /// translation.
    ///
    /// Use this for tool axes, surface normals of a rigid transform, and ray
    /// directions. Note that for a non-uniform scale this does *not* preserve
    /// normals — the correct normal transform is the inverse transpose, which
    /// the caller must build explicitly.
    #[inline]
    #[must_use]
    pub fn transform_direction(&self, d: Vec3) -> Vec3 {
        let m = &self.m;
        Vec3::new(
            m[0][0] * d.x + m[0][1] * d.y + m[0][2] * d.z,
            m[1][0] * d.x + m[1][1] * d.y + m[1][2] * d.z,
            m[2][0] * d.x + m[2][1] * d.y + m[2][2] * d.z,
        )
    }

    /// Returns true if every entry is finite.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.m.iter().flatten().all(|v| v.is_finite())
    }
}

impl Mul for Mat4 {
    type Output = Self;

    /// Matrix product. `(a * b)` applies `b` first, then `a`.
    ///
    /// Accumulation order is ascending `k`, per output entry, in ascending
    /// `(row, col)` order.
    fn mul(self, rhs: Self) -> Self {
        let mut out = [[0.0f64; 4]; 4];
        for (i, out_row) in out.iter_mut().enumerate() {
            for (j, cell) in out_row.iter_mut().enumerate() {
                *cell = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j]
                    + self.m[i][3] * rhs.m[3][j];
            }
        }
        Self { m: out }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn affine() -> Mat4 {
        Mat4::from_mat3_translation(
            Mat3::from_rows_array([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]]),
            Vec3::new(10.0, -20.0, 30.0),
        )
    }

    #[test]
    fn identity_is_neutral() {
        let a = affine();
        assert_eq!(a * Mat4::IDENTITY, a);
        assert_eq!(Mat4::IDENTITY * a, a);
        assert_eq!(Mat4::IDENTITY.determinant(), 1.0);
    }

    #[test]
    fn determinant_of_affine_is_determinant_of_linear_part() {
        let a = affine();
        assert_eq!(a.determinant(), a.upper_left3().determinant());
        assert_eq!(
            Mat4::from_scale(Vec3::new(2.0, 3.0, 4.0)).determinant(),
            24.0
        );
        assert_eq!(Mat4::ZERO.determinant(), 0.0);
    }

    #[test]
    fn point_transform_includes_translation_direction_does_not() {
        let t = Mat4::from_translation(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(t.transform_point(Vec3::ZERO), Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(t.transform_direction(Vec3::X), Vec3::X);
        assert_eq!(t.translation(), Vec3::new(1.0, 2.0, 3.0));

        let s = Mat4::from_scale(Vec3::new(2.0, 4.0, 8.0));
        assert_eq!(s.transform_point(Vec3::ONE), Vec3::new(2.0, 4.0, 8.0));
        assert_eq!(s.transform_direction(Vec3::ONE), Vec3::new(2.0, 4.0, 8.0));
    }

    #[test]
    fn inverse_round_trips() {
        let a = affine();
        let inv = a.inverse().expect("det == 1, invertible");
        assert_eq!(a * inv, Mat4::IDENTITY);
        assert_eq!(inv * a, Mat4::IDENTITY);
        // Round-tripping a point is exact for this integral matrix.
        let p = Vec3::new(3.0, -7.0, 11.0);
        assert_eq!(inv.transform_point(a.transform_point(p)), p);
    }

    #[test]
    fn inverse_rejects_singular_and_nonfinite() {
        assert_eq!(Mat4::ZERO.inverse(), None);
        assert_eq!(Mat4::from_scale(Vec3::new(1.0, 0.0, 1.0)).inverse(), None);
        let mut nan = Mat4::IDENTITY;
        nan.m[2][2] = f64::NAN;
        assert_eq!(nan.inverse(), None);
    }

    #[test]
    fn transpose_is_involutive_and_swaps_indices() {
        let a = affine();
        assert_eq!(a.transpose().transpose(), a);
        assert_eq!(a.transpose().m[0][3], a.m[3][0]);
    }

    #[test]
    fn composition_order_matches_documentation() {
        let translate = Mat4::from_translation(Vec3::new(1.0, 0.0, 0.0));
        let scale = Mat4::from_scale(Vec3::splat(10.0));
        // (scale * translate) translates first: (0,0,0) -> (1,0,0) -> (10,0,0).
        assert_eq!(
            (scale * translate).transform_point(Vec3::ZERO),
            Vec3::new(10.0, 0.0, 0.0)
        );
        // (translate * scale) scales first: (0,0,0) -> (0,0,0) -> (1,0,0).
        assert_eq!(
            (translate * scale).transform_point(Vec3::ZERO),
            Vec3::new(1.0, 0.0, 0.0)
        );
    }
}
