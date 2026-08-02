// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! 3x3 `f64` matrix.

use core::ops::Mul;

use crate::eps::EPS_DETERMINANT;
use crate::math::Vec3;

/// A 3x3 matrix, stored **row-major**: `m[row][col]`.
///
/// Vectors are treated as columns, so `M * v` applies `M` to `v` and the
/// composition `A * B` applies `B` first.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    /// Rows, outer index is the row.
    pub m: [[f64; 3]; 3],
}

impl Default for Mat3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat3 {
    /// The multiplicative identity.
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// The all-zero matrix.
    pub const ZERO: Self = Self { m: [[0.0; 3]; 3] };

    /// Constructs from a row-major array.
    #[inline]
    #[must_use]
    pub const fn from_rows_array(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    /// Constructs from three row vectors.
    #[inline]
    #[must_use]
    pub const fn from_rows(r0: Vec3, r1: Vec3, r2: Vec3) -> Self {
        Self {
            m: [r0.to_array(), r1.to_array(), r2.to_array()],
        }
    }

    /// Constructs from three column vectors.
    #[inline]
    #[must_use]
    pub const fn from_cols(c0: Vec3, c1: Vec3, c2: Vec3) -> Self {
        Self {
            m: [[c0.x, c1.x, c2.x], [c0.y, c1.y, c2.y], [c0.z, c1.z, c2.z]],
        }
    }

    /// A diagonal (non-uniform scale) matrix.
    #[inline]
    #[must_use]
    pub const fn from_scale(s: Vec3) -> Self {
        Self {
            m: [[s.x, 0.0, 0.0], [0.0, s.y, 0.0], [0.0, 0.0, s.z]],
        }
    }

    /// Returns row `i`.
    ///
    /// # Panics
    /// Panics if `i > 2`.
    #[inline]
    #[must_use]
    pub const fn row(&self, i: usize) -> Vec3 {
        Vec3::from_array(self.m[i])
    }

    /// Returns column `j`.
    ///
    /// # Panics
    /// Panics if `j > 2`.
    #[inline]
    #[must_use]
    pub const fn col(&self, j: usize) -> Vec3 {
        Vec3::new(self.m[0][j], self.m[1][j], self.m[2][j])
    }

    /// Returns the transpose.
    #[inline]
    #[must_use]
    pub const fn transpose(&self) -> Self {
        let m = &self.m;
        Self {
            m: [
                [m[0][0], m[1][0], m[2][0]],
                [m[0][1], m[1][1], m[2][1]],
                [m[0][2], m[1][2], m[2][2]],
            ],
        }
    }

    /// Determinant, by cofactor expansion along **row 0**, summed left to right.
    ///
    /// The expansion row and the summation order are part of the contract:
    /// expanding along a different row is algebraically identical and
    /// numerically different.
    #[inline]
    #[must_use]
    pub fn determinant(&self) -> f64 {
        let m = &self.m;
        let c0 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let c1 = m[1][0] * m[2][2] - m[1][2] * m[2][0];
        let c2 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        m[0][0] * c0 - m[0][1] * c1 + m[0][2] * c2
    }

    /// Returns the inverse, or `None` if the matrix is singular (see
    /// [`crate::eps::EPS_DETERMINANT`]) or contains a non-finite entry.
    ///
    /// Computed as the transposed cofactor matrix divided entry-wise by the
    /// determinant. Entry-wise division rather than multiplication by a
    /// precomputed reciprocal: the reciprocal costs an extra rounding for no
    /// benefit outside a hot loop, and this is not one.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if !det.is_finite() || det.abs() < EPS_DETERMINANT {
            return None;
        }
        let m = &self.m;
        // cof[i][j] is the cofactor of entry (i, j).
        let cof = [
            [
                m[1][1] * m[2][2] - m[1][2] * m[2][1],
                -(m[1][0] * m[2][2] - m[1][2] * m[2][0]),
                m[1][0] * m[2][1] - m[1][1] * m[2][0],
            ],
            [
                -(m[0][1] * m[2][2] - m[0][2] * m[2][1]),
                m[0][0] * m[2][2] - m[0][2] * m[2][0],
                -(m[0][0] * m[2][1] - m[0][1] * m[2][0]),
            ],
            [
                m[0][1] * m[1][2] - m[0][2] * m[1][1],
                -(m[0][0] * m[1][2] - m[0][2] * m[1][0]),
                m[0][0] * m[1][1] - m[0][1] * m[1][0],
            ],
        ];
        // inverse = adjugate / det, and adjugate = cofactor^T.
        let mut out = [[0.0f64; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = cof[j][i] / det;
            }
        }
        let inv = Self { m: out };
        if inv.m.iter().flatten().all(|v| v.is_finite()) {
            Some(inv)
        } else {
            None
        }
    }

    /// Applies the matrix to a column vector: `self * v`.
    #[inline]
    #[must_use]
    pub fn mul_vec3(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.row(0).dot(v), self.row(1).dot(v), self.row(2).dot(v))
    }

    /// Returns true if every entry is finite.
    #[inline]
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.m.iter().flatten().all(|v| v.is_finite())
    }
}

impl Mul for Mat3 {
    type Output = Self;

    /// Matrix product. `(a * b)` applies `b` first, then `a`.
    ///
    /// Accumulation order is ascending `k`, per output entry, in ascending
    /// `(row, col)` order.
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        let mut out = [[0.0f64; 3]; 3];
        for (i, out_row) in out.iter_mut().enumerate() {
            for (j, cell) in out_row.iter_mut().enumerate() {
                *cell = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j];
            }
        }
        Self { m: out }
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        self.mul_vec3(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Mat3 {
        Mat3::from_rows_array([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]])
    }

    #[test]
    fn identity_is_neutral() {
        let a = sample();
        assert_eq!(a * Mat3::IDENTITY, a);
        assert_eq!(Mat3::IDENTITY * a, a);
        assert_eq!(Mat3::IDENTITY.determinant(), 1.0);
        let v = Vec3::new(3.0, -1.0, 7.0);
        assert_eq!(Mat3::IDENTITY * v, v);
    }

    #[test]
    fn rows_cols_and_transpose() {
        let a = sample();
        assert_eq!(a.row(1), Vec3::new(0.0, 1.0, 4.0));
        assert_eq!(a.col(1), Vec3::new(2.0, 1.0, 6.0));
        assert_eq!(a.transpose().transpose(), a);
        assert_eq!(a.transpose().row(1), a.col(1));
        assert_eq!(
            Mat3::from_cols(a.col(0), a.col(1), a.col(2)),
            a,
            "from_cols must be the inverse of col()"
        );
    }

    #[test]
    fn determinant_matches_hand_computation() {
        // This matrix has a known integer inverse, which is why it was chosen.
        assert_eq!(sample().determinant(), 1.0);
        assert_eq!(
            Mat3::from_scale(Vec3::new(2.0, 3.0, 4.0)).determinant(),
            24.0
        );
        assert_eq!(Mat3::ZERO.determinant(), 0.0);
    }

    #[test]
    fn inverse_round_trips() {
        let a = sample();
        let inv = a.inverse().expect("det == 1, invertible");
        // Exact: the inverse of this matrix is integral.
        let expected =
            Mat3::from_rows_array([[-24.0, 18.0, 5.0], [20.0, -15.0, -4.0], [-5.0, 4.0, 1.0]]);
        assert_eq!(inv, expected);
        assert_eq!(a * inv, Mat3::IDENTITY);
        assert_eq!(inv * a, Mat3::IDENTITY);
    }

    #[test]
    fn inverse_rejects_singular_and_nonfinite() {
        assert_eq!(Mat3::ZERO.inverse(), None);
        // Rank-deficient: row 2 = row 0 + row 1.
        let singular = Mat3::from_rows_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [5.0, 7.0, 9.0]]);
        assert_eq!(singular.inverse(), None);
        let nan = Mat3::from_rows_array([[f64::NAN, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(nan.inverse(), None);
    }

    #[test]
    fn multiplication_applies_right_hand_side_first() {
        let scale = Mat3::from_scale(Vec3::new(2.0, 2.0, 2.0));
        let swap_xy = Mat3::from_rows_array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        let v = Vec3::new(1.0, 0.0, 0.0);
        // (scale * swap) applies swap first: (1,0,0) -> (0,1,0) -> (0,2,0).
        assert_eq!((scale * swap_xy) * v, Vec3::new(0.0, 2.0, 0.0));
    }
}
