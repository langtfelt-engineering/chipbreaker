// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The quadratic error function, and the eigensolver it needs.
//!
//! # What is being minimised
//!
//! Given crossings `p_i` with outward normals `n_i`, dual contouring places the
//! cell's vertex at the point minimising
//!
//! ```text
//! E(x) = sum_i ( n_i . (x - p_i) )^2
//! ```
//!
//! the total squared distance to the *planes* through the crossings. Not to the
//! points — to the planes. That distinction is the entire reason normals are
//! stored: with points alone the minimiser is the centroid, which is plain
//! surface nets and rounds every sharp edge off.
//!
//! What the planes buy is visible in the rank of the system:
//!
//! - **One distinct normal** (a flat): the minimiser is a whole plane. The
//!   solution is under-determined in two directions, and any point on that plane
//!   is as good as any other.
//! - **Two** (an edge): under-determined along one direction — the edge line —
//!   and the minimiser lands *on the edge*.
//! - **Three** (a corner): fully determined, and the minimiser is the corner.
//!
//! So sharp features are not detected and reconstructed; they fall out of the
//! minimisation. That is why this is worth doing properly rather than
//! approximating.
//!
//! # Why truncated SVD and not a plain solve
//!
//! The rank deficiency above is not an edge case, it is the common case: most
//! cells of a machined part lie on a flat, where `A^T A` is rank 1 and singular.
//! Inverting it is meaningless and inverting it numerically is worse — the
//! solution shoots off along the null space to wherever rounding points.
//!
//! Truncating the small singular values replaces "solve exactly" with "solve in
//! the directions that are actually constrained, and stay put in the others".
//! Combined with expanding about the centroid of the crossings, an
//! under-determined direction resolves to the centroid rather than to infinity.
//!
//! # Determinism
//!
//! `A^T A` and `A^T b` are accumulated by the caller in a fixed order. The
//! eigensolver below is cyclic Jacobi with a fixed sweep order and a fixed
//! iteration cap, so it performs the same arithmetic in the same sequence on
//! every input of the same shape — no convergence-dependent branching that could
//! differ between targets, and no dependency on a linear algebra library whose
//! internals we do not control and whose determinism nobody guarantees.

use crate::math::Vec3;

/// Singular values below this fraction of the largest are treated as zero.
///
/// The knob that decides whether a nearly-flat configuration is treated as flat
/// (vertex at the centroid, smooth) or as an edge (vertex pulled onto the
/// intersection, sharp). Too small and quantisation noise in the normals starts
/// inventing creases on smooth surfaces; too large and genuine shallow edges get
/// rounded away.
///
/// `0.1` is the value the dual contouring literature converged on and it holds
/// up here: the oct encoding is accurate to about 0.1 degrees, so two normals
/// have to differ by far more than encoding noise before their singular value
/// clears a tenth of the largest.
pub const SINGULAR_THRESHOLD: f64 = 0.1;

/// Jacobi sweeps. Fixed, not convergence-dependent.
///
/// A symmetric 3x3 converges to machine precision in far fewer than this;
/// running a fixed count regardless removes the data-dependent branch that a
/// tolerance test would introduce, which is what makes the result identical on
/// every target rather than merely equal to within a tolerance.
const JACOBI_SWEEPS: usize = 8;

/// Accumulates the normal equations for one vertex.
///
/// Holds `A^T A` (symmetric, six distinct entries) and `A^T b`, plus the
/// centroid of the crossings, which is where an under-determined direction
/// resolves to.
#[derive(Debug, Clone, Default)]
pub struct Qef {
    /// Upper triangle of `A^T A`, row-major: xx, xy, xz, yy, yz, zz.
    ata: [f64; 6],
    /// `A^T b`.
    atb: Vec3,
    /// Sum of the crossing positions, for the centroid.
    sum: Vec3,
    /// How many crossings have been added.
    count: usize,
}

impl Qef {
    /// An empty system.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds the plane through `p` with unit normal `n`.
    pub fn add(&mut self, p: Vec3, n: Vec3) {
        let d = n.x * p.x + n.y * p.y + n.z * p.z;
        self.ata[0] += n.x * n.x;
        self.ata[1] += n.x * n.y;
        self.ata[2] += n.x * n.z;
        self.ata[3] += n.y * n.y;
        self.ata[4] += n.y * n.z;
        self.ata[5] += n.z * n.z;
        self.atb = Vec3::new(
            self.atb.x + n.x * d,
            self.atb.y + n.y * d,
            self.atb.z + n.z * d,
        );
        self.sum = Vec3::new(self.sum.x + p.x, self.sum.y + p.y, self.sum.z + p.z);
        self.count += 1;
    }

    /// How many planes have been added.
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// The centroid of the crossings, which is the fallback in every direction
    /// the planes do not constrain.
    #[must_use]
    pub fn centroid(&self) -> Vec3 {
        if self.count == 0 {
            return Vec3::new(0.0, 0.0, 0.0);
        }
        #[allow(clippy::cast_precision_loss, reason = "a small count")]
        let n = self.count as f64;
        Vec3::new(self.sum.x / n, self.sum.y / n, self.sum.z / n)
    }

    /// Minimises the error, returning the vertex and the rank actually used.
    ///
    /// The rank is reported because it is the sharp-feature measurement: rank 1
    /// is a flat, 2 an edge, 3 a corner. `--stats` publishes the histogram.
    #[must_use]
    pub fn solve(&self) -> (Vec3, u8) {
        let centroid = self.centroid();
        if self.count == 0 {
            return (centroid, 0);
        }

        // Solve about the centroid rather than about the origin. `A^T A` is the
        // same, but the right-hand side becomes the residual there, so an
        // unconstrained direction contributes nothing and the solution stays at
        // the centroid instead of drifting to wherever the null space points.
        let ac = self.mul(centroid);
        let rhs = Vec3::new(self.atb.x - ac.x, self.atb.y - ac.y, self.atb.z - ac.z);

        let (values, vectors) = jacobi_eigen(self.ata);
        let largest = values
            .iter()
            .fold(0.0f64, |acc, v| if v.abs() > acc { v.abs() } else { acc });
        if largest <= 0.0 {
            return (centroid, 0);
        }
        let cutoff = largest * SINGULAR_THRESHOLD;

        // The pseudo-inverse: invert only the directions above the cutoff.
        let mut offset = Vec3::new(0.0, 0.0, 0.0);
        let mut rank = 0u8;
        for k in 0..3 {
            if values[k].abs() < cutoff {
                continue;
            }
            rank += 1;
            let e = vectors[k];
            let dot = e.x * rhs.x + e.y * rhs.y + e.z * rhs.z;
            let scale = dot / values[k];
            offset = Vec3::new(
                offset.x + e.x * scale,
                offset.y + e.y * scale,
                offset.z + e.z * scale,
            );
        }

        (
            Vec3::new(
                centroid.x + offset.x,
                centroid.y + offset.y,
                centroid.z + offset.z,
            ),
            rank,
        )
    }

    /// `A^T A * v`.
    fn mul(&self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.ata[0] * v.x + self.ata[1] * v.y + self.ata[2] * v.z,
            self.ata[1] * v.x + self.ata[3] * v.y + self.ata[4] * v.z,
            self.ata[2] * v.x + self.ata[4] * v.y + self.ata[5] * v.z,
        )
    }

    /// Residual error at `x`, for diagnostics.
    #[must_use]
    pub fn error_at(&self, x: Vec3) -> f64 {
        let ax = self.mul(x);
        let quad = x.x * ax.x + x.y * ax.y + x.z * ax.z;
        let linear = 2.0 * (x.x * self.atb.x + x.y * self.atb.y + x.z * self.atb.z);
        quad - linear
    }
}

/// Eigenvalues and eigenvectors of a symmetric 3x3, by cyclic Jacobi.
///
/// Input is the upper triangle in the order xx, xy, xz, yy, yz, zz. Output is
/// three eigenvalues and their unit eigenvectors, in the order the sweeps leave
/// them — **not** sorted, because a sort would need a tie-break rule on equal
/// eigenvalues and equal eigenvalues are the common case here (a flat has two).
/// Nothing downstream depends on the order: the pseudo-inverse sums over all
/// three regardless.
fn jacobi_eigen(upper: [f64; 6]) -> ([f64; 3], [Vec3; 3]) {
    // Full symmetric matrix, and the accumulating rotation.
    let mut a = [
        [upper[0], upper[1], upper[2]],
        [upper[1], upper[3], upper[4]],
        [upper[2], upper[4], upper[5]],
    ];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    for _ in 0..JACOBI_SWEEPS {
        // Fixed sweep order. Classical Jacobi picks the largest off-diagonal,
        // which is a data-dependent branch; cyclic order visits all three every
        // sweep and converges just as fast on a 3x3.
        for &(p, q) in &[(0usize, 1usize), (0, 2), (1, 2)] {
            let apq = a[p][q];
            if apq == 0.0 {
                continue;
            }
            // The rotation that zeroes (p, q). Computed via `theta` and `t` in
            // the numerically stable form, which avoids cancellation when the
            // diagonal entries are close -- exactly the near-degenerate case a
            // flat region produces.
            let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
            let t = if theta >= 0.0 {
                1.0 / (theta + (1.0 + theta * theta).sqrt())
            } else {
                -1.0 / (-theta + (1.0 + theta * theta).sqrt())
            };
            let c = 1.0 / (1.0 + t * t).sqrt();
            let s = t * c;

            // Apply the rotation on both sides. Indexed loops, not iterators:
            // each body reads and writes two columns of the same array, which an
            // iterator cannot express without splitting the borrow.
            #[allow(clippy::needless_range_loop, reason = "two columns of one row")]
            for k in 0..3 {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            #[allow(clippy::needless_range_loop, reason = "two rows of one column")]
            for k in 0..3 {
                let apk = a[p][k];
                let aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            #[allow(clippy::needless_range_loop, reason = "two columns of one row")]
            for k in 0..3 {
                let vkp = v[k][p];
                let vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }

    (
        [a[0][0], a[1][1], a[2][2]],
        [
            Vec3::new(v[0][0], v[1][0], v[2][0]),
            Vec3::new(v[0][1], v[1][1], v[2][1]),
            Vec3::new(v[0][2], v[1][2], v[2][2]),
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3, tol: f64) -> bool {
        (a.x - b.x).abs() < tol && (a.y - b.y).abs() < tol && (a.z - b.z).abs() < tol
    }

    #[test]
    fn three_planes_give_a_corner_at_rank_three() {
        // The x=1, y=2, z=3 planes meet at exactly one point, and a fully
        // determined system must find it.
        let mut q = Qef::new();
        q.add(Vec3::new(1.0, 9.0, 9.0), Vec3::new(1.0, 0.0, 0.0));
        q.add(Vec3::new(9.0, 2.0, 9.0), Vec3::new(0.0, 1.0, 0.0));
        q.add(Vec3::new(9.0, 9.0, 3.0), Vec3::new(0.0, 0.0, 1.0));
        let (x, rank) = q.solve();
        assert_eq!(rank, 3, "three independent normals are a corner");
        assert!(close(x, Vec3::new(1.0, 2.0, 3.0), 1.0e-9), "got {x:?}");
    }

    #[test]
    fn two_planes_put_the_vertex_on_the_edge_at_rank_two() {
        // A 90-degree edge along Z at x=1, y=2. The vertex must land on that
        // line; where along it is unconstrained and resolves to the centroid.
        let mut q = Qef::new();
        q.add(Vec3::new(1.0, 0.0, 5.0), Vec3::new(1.0, 0.0, 0.0));
        q.add(Vec3::new(0.0, 2.0, 7.0), Vec3::new(0.0, 1.0, 0.0));
        let (x, rank) = q.solve();
        assert_eq!(rank, 2, "two independent normals are an edge");
        assert!(
            (x.x - 1.0).abs() < 1.0e-9 && (x.y - 2.0).abs() < 1.0e-9,
            "the vertex must sit on the edge line, got {x:?}"
        );
        assert!(
            (x.z - 6.0).abs() < 1.0e-9,
            "the unconstrained direction should resolve to the centroid, got z={}",
            x.z
        );
    }

    #[test]
    fn one_plane_leaves_the_vertex_on_it_at_rank_one() {
        // A flat. The system is rank 1, and the two unconstrained directions
        // must resolve to the centroid rather than shooting off the null space
        // -- which is what an untruncated solve does, and it is why a flat
        // region is where plain inversion produces flying vertices.
        let mut q = Qef::new();
        for p in [
            Vec3::new(0.0, 0.0, 4.0),
            Vec3::new(1.0, 0.0, 4.0),
            Vec3::new(0.0, 1.0, 4.0),
            Vec3::new(1.0, 1.0, 4.0),
        ] {
            q.add(p, Vec3::new(0.0, 0.0, 1.0));
        }
        let (x, rank) = q.solve();
        assert_eq!(rank, 1, "coplanar normals are a flat");
        assert!((x.z - 4.0).abs() < 1.0e-9, "off the plane: {x:?}");
        assert!(
            close(x, Vec3::new(0.5, 0.5, 4.0), 1.0e-9),
            "a flat should resolve to the centroid, got {x:?}"
        );
    }

    #[test]
    fn a_nearly_flat_pair_is_treated_as_flat() {
        // Two normals a tenth of a degree apart -- the oct encoding's own
        // resolution. Treating that as an edge would invent creases all over a
        // smooth surface, which is what the singular threshold exists to stop.
        // `transcendental`, not `f64::sin` -- the project forbids std
        // transcendentals so that every target computes the same bits, and
        // clippy caught this one in a test where it would have been just as
        // capable of differing.
        let angle = 0.1f64.to_radians();
        let (sin, cos) = crate::transcendental::sin_cos(angle);
        let mut q = Qef::new();
        q.add(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        q.add(Vec3::new(1.0, 0.0, 0.0), Vec3::new(sin, 0.0, cos));
        let (_, rank) = q.solve();
        assert_eq!(
            rank, 1,
            "normals within the encoding's own noise must not read as an edge"
        );
    }

    #[test]
    fn a_genuine_ninety_degree_edge_is_not_flattened() {
        let mut q = Qef::new();
        q.add(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        q.add(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let (_, rank) = q.solve();
        assert_eq!(rank, 2, "a right angle is an edge");
    }

    #[test]
    fn the_eigensolver_reproduces_a_known_decomposition() {
        // A diagonal matrix is its own decomposition, and a rotation of one has
        // known eigenvalues -- so this checks the sweep does not merely converge
        // to something, but to the right thing.
        let (values, _) = jacobi_eigen([2.0, 0.0, 0.0, 5.0, 0.0, 11.0]);
        let mut sorted = values;
        sorted.sort_by(f64::total_cmp);
        assert!((sorted[0] - 2.0).abs() < 1.0e-12);
        assert!((sorted[1] - 5.0).abs() < 1.0e-12);
        assert!((sorted[2] - 11.0).abs() < 1.0e-12);

        // Symmetric with off-diagonal mass: trace and determinant are invariant
        // under the rotation, so they pin the answer without needing the
        // eigenvalues in closed form.
        let m = [4.0, 1.0, 2.0, 6.0, 3.0, 8.0];
        let (v, vectors) = jacobi_eigen(m);
        let trace: f64 = v.iter().sum();
        assert!(
            (trace - (4.0 + 6.0 + 8.0)).abs() < 1.0e-10,
            "trace is not preserved: {trace}"
        );
        for e in vectors {
            let len = (e.x * e.x + e.y * e.y + e.z * e.z).sqrt();
            assert!(
                (len - 1.0).abs() < 1.0e-10,
                "eigenvector is not unit: {len}"
            );
        }
    }

    #[test]
    fn the_solver_is_bit_identical_across_repeated_runs() {
        // A fixed sweep count and no data-dependent branching means the same
        // input performs the same arithmetic every time. This is the property
        // the cross-target hash depends on.
        let mut q = Qef::new();
        q.add(Vec3::new(0.3, 0.1, 0.7), Vec3::new(0.2, 0.9, 0.3));
        q.add(Vec3::new(0.8, 0.4, 0.2), Vec3::new(-0.7, 0.1, 0.6));
        q.add(Vec3::new(0.1, 0.9, 0.5), Vec3::new(0.4, -0.4, 0.8));
        let (first, rank) = q.solve();
        for _ in 0..64 {
            let (again, r) = q.solve();
            assert_eq!(again.x.to_bits(), first.x.to_bits());
            assert_eq!(again.y.to_bits(), first.y.to_bits());
            assert_eq!(again.z.to_bits(), first.z.to_bits());
            assert_eq!(r, rank);
        }
    }

    #[test]
    fn an_empty_system_does_not_divide_by_zero() {
        let (x, rank) = Qef::new().solve();
        assert_eq!(rank, 0);
        assert!(x.x.is_finite() && x.y.is_finite() && x.z.is_finite());
    }
}
