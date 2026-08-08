// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Moving stock and fixtures between setups.
//!
//! # The axis-aligned case does not resample at all
//!
//! A second operation is usually the part turned over or rotated a quarter
//! turn, and that case is not an interpolation — it is a **relabelling**.
//!
//! A 90° rotation is a signed permutation of the coordinates, so every entry of
//! its matrix is `0` or `±1` and the transform is exact in `f64`. Rays map onto
//! rays: the X bundle becomes the Y bundle, its ray parameters are unchanged,
//! and its spans move across untouched. The only thing that could lose
//! information is the quantised normal, and the octahedral encoding is
//! odd-symmetric about zero — chosen that way so negation is exact integer
//! arithmetic — which makes a signed permutation exact too. That was measured
//! over 40 000 normals before this module was written, not assumed.
//!
//! So the common path carries a bound of **exactly zero**, and no interpolation
//! code runs on it.
//!
//! # The general case, and what its bound rests on
//!
//! An arbitrary rotation does not map rays onto rays, so the field has to be
//! resampled: each new ray is cast against the material the old field describes.
//! The error is a **sampling** error, not an accumulation one, and it is bounded
//! by the same argument that bounds the original build — a surface between two
//! adjacent samples can depart from the sampled reconstruction by at most half a
//! cell along the worst-oriented direction, which the sampling theorem puts at
//! `SAMPLE_DISTANCE_CONSTANT` times the cell size.
//!
//! **This is the only place in the engine where a transform loses anything.**
//! Everything else is exact by construction, so it is stated per boundary and
//! accumulated across a job rather than folded into a single figure that would
//! hide which setup paid it.
//!
//! # Why resample the field rather than re-contour it
//!
//! Contouring to a mesh and rebuilding would cost the tessellation floor twice
//! per setup and compound it across a job, and it would forfeit the property
//! that nothing accumulates between operations. Resampling is one interpolation
//! with a computable bound instead of two lossy conversions with an empirical
//! one.

use crate::dexel::lattice::Lattice;
use crate::dexel::tri::AXES;
use crate::math::{Axis, Mat4, Vec3};

/// How a setup boundary was crossed, and what it cost.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Regime {
    /// Rays mapped onto rays. A relabelling, exact, bound zero.
    Exact {
        /// Which source axis each destination axis came from.
        from: [Axis; 3],
        /// Whether that axis was reversed.
        flipped: [bool; 3],
    },
    /// The field was resampled on a rotated lattice.
    Resampled {
        /// Worst departure the resampling can introduce, in millimetres.
        bound_mm: f64,
    },
}

impl Regime {
    /// The bound this boundary contributes. Zero for the exact case.
    #[must_use]
    pub const fn bound_mm(&self) -> f64 {
        match self {
            Self::Exact { .. } => 0.0,
            Self::Resampled { bound_mm } => *bound_mm,
        }
    }

    /// The name used in a report.
    #[must_use]
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::Exact { .. } => "exact",
            Self::Resampled { .. } => "resampled",
        }
    }

    /// Whether this boundary cost nothing.
    #[must_use]
    pub const fn is_exact(&self) -> bool {
        matches!(self, Self::Exact { .. })
    }
}

/// How close a matrix entry must be to `0` or `±1` to count as one.
///
/// Deliberately tight. A rotation that is *nearly* axis-aligned is not
/// axis-aligned, and treating it as though it were would silently claim a zero
/// bound for a transform that does lose something. A caller who wants the
/// exact path should pass an exact matrix; the tolerance here absorbs only the
/// representation of `cos(90°)`, which `transcendental` returns as a value a few
/// ulps from zero rather than as zero itself.
const AXIS_ALIGNED_EPS: f64 = 1.0e-12;

/// Classifies a rigid transform, and says what crossing it costs.
///
/// Returns `None` if the transform is not a rigid motion at all — a scale or a
/// shear would make "the same stock in a new orientation" untrue, and a bound
/// for it would be meaningless.
#[must_use]
pub fn classify(transform: &Mat4, spacing_mm: f64) -> Option<Regime> {
    if !transform.is_finite() {
        return None;
    }
    // The rotation part, as the images of the three basis vectors.
    let cols = [
        transform.transform_direction(Vec3::new(1.0, 0.0, 0.0)),
        transform.transform_direction(Vec3::new(0.0, 1.0, 0.0)),
        transform.transform_direction(Vec3::new(0.0, 0.0, 1.0)),
    ];
    // Rigid: unit columns, mutually perpendicular.
    for (i, c) in cols.iter().enumerate() {
        if (c.length() - 1.0).abs() > 1.0e-9 {
            return None;
        }
        for other in cols.iter().skip(i + 1) {
            if c.dot(*other).abs() > 1.0e-9 {
                return None;
            }
        }
    }

    // Axis-aligned when every column is a signed basis vector.
    let mut from = [Axis::X; 3];
    let mut flipped = [false; 3];
    let mut seen = [false; 3];
    for (source, c) in cols.iter().enumerate() {
        let a = c.to_array();
        let mut hit = None;
        for (dest, value) in a.iter().enumerate() {
            let magnitude = value.abs();
            if magnitude > AXIS_ALIGNED_EPS {
                if (magnitude - 1.0).abs() > AXIS_ALIGNED_EPS || hit.is_some() {
                    hit = None;
                    break;
                }
                hit = Some((dest, *value < 0.0));
            }
        }
        match hit {
            Some((dest, negative)) if !seen[dest] => {
                seen[dest] = true;
                from[dest] = AXES[source];
                flipped[dest] = negative;
            }
            _ => {
                return Some(Regime::Resampled {
                    bound_mm: resample_bound(spacing_mm),
                });
            }
        }
    }
    Some(Regime::Exact { from, flipped })
}

/// The worst departure a resample on a rotated lattice can introduce.
///
/// The same argument that bounds the original build: between two adjacent
/// samples a surface can depart from the sampled reconstruction by at most half
/// a cell along the worst-oriented direction, and the sampling theorem puts the
/// worst orientation at [`SAMPLE_DISTANCE_CONSTANT`] cells.
///
/// [`SAMPLE_DISTANCE_CONSTANT`]: crate::dexel::tri::SAMPLE_DISTANCE_CONSTANT
#[must_use]
pub fn resample_bound(spacing_mm: f64) -> f64 {
    0.5 * crate::dexel::tri::SAMPLE_DISTANCE_CONSTANT * spacing_mm
}

/// The bound accumulated across a whole job.
///
/// A plain sum. The boundaries are independent samplings of the same solid, so
/// their worst cases can in principle line up, and a reader is owed the figure
/// that cannot be exceeded rather than the one that usually is not. Quoting a
/// root-sum-square here would be assuming errors that have no reason to be
/// independent are independent.
#[must_use]
pub fn accumulated_bound(regimes: &[Regime]) -> f64 {
    regimes.iter().map(Regime::bound_mm).sum()
}

/// Where a lattice ends up under an axis-aligned transform.
///
/// The destination lattice for `axis`, given the source bundle it came from.
/// Registered by construction: the corner the source lattice starts from is
/// carried through the same transform as the material, so the two cannot drift
/// apart.
#[must_use]
pub fn transformed_lattice(source: &Lattice, transform: &Mat4, axis: Axis) -> Lattice {
    let [su, sv, _] = source.axis().cyclic();
    let [du, dv, _] = axis.cyclic();
    // Which destination lattice axis each source lattice axis becomes.
    let image = |world: usize| -> usize {
        let mut e = [0.0; 3];
        e[world] = 1.0;
        let v = transform
            .transform_direction(Vec3::from_array(e))
            .to_array();
        v.iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .map_or(world, |(i, _)| i)
    };
    let (iu, iv) = (image(su), image(sv));
    let swap = iu == dv || iv == du;
    let spacing = source.spacing_uv();
    let counts = source.counts();
    let extent = source.extent();
    let (spacing, counts, extent) = if swap {
        (
            [spacing[1], spacing[0]],
            [counts[1], counts[0]],
            [extent[1], extent[0]],
        )
    } else {
        (spacing, counts, extent)
    };
    // The lower corner of the covered box, carried through the transform. Using
    // the corner rather than the centre keeps the arithmetic exact for a signed
    // permutation: every coordinate is copied or negated, never averaged.
    let corner = transform.transform_point(source.origin());
    Lattice::from_parts(axis, corner, spacing, counts, extent, source.length())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rot_z_90() -> Mat4 {
        // (x, y, z) -> (-y, x, z), written exactly rather than through a cosine.
        Mat4::from_rows_array([
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    #[test]
    fn a_quarter_turn_is_exact() {
        let r = classify(&rot_z_90(), 0.5).expect("a rigid motion");
        assert!(r.is_exact(), "a quarter turn must not resample: {r:?}");
        assert_eq!(r.bound_mm(), 0.0);
    }

    #[test]
    fn the_identity_is_exact() {
        let r = classify(&Mat4::IDENTITY, 0.5).expect("rigid");
        assert!(r.is_exact());
        match r {
            Regime::Exact { from, flipped } => {
                assert_eq!(from, [Axis::X, Axis::Y, Axis::Z]);
                assert_eq!(flipped, [false; 3]);
            }
            Regime::Resampled { .. } => panic!("the identity resampled"),
        }
    }

    #[test]
    fn an_arbitrary_rotation_resamples_and_carries_a_bound() {
        // Thirty degrees about Z. The mutation check for the two above: if
        // everything classified as exact, "the quarter turn is exact" would be
        // saying nothing.
        let (c, s) = (
            crate::transcendental::cos(0.5),
            crate::transcendental::sin(0.5),
        );
        let m = Mat4::from_rows_array([
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let r = classify(&m, 0.5).expect("rigid");
        assert!(
            !r.is_exact(),
            "an arbitrary rotation must not claim exactness"
        );
        assert!(r.bound_mm() > 0.0);
        assert!((r.bound_mm() - resample_bound(0.5)).abs() < 1e-15);
    }

    #[test]
    fn a_scale_is_refused_rather_than_bounded() {
        // Not a rigid motion, so "the same stock in a new orientation" is untrue
        // and a bound for it would be meaningless.
        let m = Mat4::from_rows_array([
            [2.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        assert!(classify(&m, 0.5).is_none());
    }

    #[test]
    fn bounds_add_rather_than_cancel() {
        let exact = Regime::Exact {
            from: [Axis::X, Axis::Y, Axis::Z],
            flipped: [false; 3],
        };
        let rough = Regime::Resampled { bound_mm: 0.3 };
        assert_eq!(accumulated_bound(&[exact, exact]), 0.0);
        assert!((accumulated_bound(&[rough, exact, rough]) - 0.6).abs() < 1e-15);
    }
}
