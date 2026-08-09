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

use crate::dexel::arena::Arena;
use crate::dexel::field::DexelField;
use crate::dexel::lattice::Lattice;
use crate::dexel::tri::{AXES, TriDexelField};
use crate::math::{Aabb3, Axis, Mat4, OctNormal, Vec3};
use crate::spans::Span;

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
    // **Folded from `0.0`, not summed.** The standard library's `Sum` for `f64`
    // uses `-0.0` as its identity, deliberately, so that adding it preserves the
    // sign of every term. The consequence is that an empty sum is **negative
    // zero**, and `-0.0` reached the published JSON as
    // `"accumulated_transform_bound_mm": -0.0` for every single-setup job.
    //
    // It compares equal to zero in most languages and is a different value in
    // this engine's own canonical hashing, which covers signed zero explicitly.
    // A published field must not depend on a reader knowing that.
    regimes.iter().map(Regime::bound_mm).fold(0.0, |a, b| a + b)
}

/// The workspace a lattice covers, recovered from its stored parts.
///
/// `origin` is the lower corner and the extents are stored rather than derived,
/// so this is the box the lattice was built from and not the box its cells
/// happen to cover. The difference is the padding, and rebuilding from the
/// covered box instead would grow the workspace by a cell on every setup.
#[must_use]
fn workspace_of(lattice: &Lattice) -> Aabb3 {
    let [u, v, w] = lattice.axis().cyclic();
    let mut span = [0.0; 3];
    span[u] = lattice.extent()[0];
    span[v] = lattice.extent()[1];
    span[w] = lattice.length();
    let min = lattice.origin();
    Aabb3::from_min_max(min, min + Vec3::from_array(span))
}

/// Moves a field into a new setup, exactly.
///
/// Only for the axis-aligned case, where rays map onto rays and the whole
/// operation is a relabelling. Returns `None` if the transform is not
/// axis-aligned — the caller then owes the reader a bound, and silently
/// resampling here would hide that.
///
/// # What makes the result bit-identical to a direct build
///
/// The destination lattices are constructed from the **rotated workspace** by
/// the same constructor a fresh build uses, rather than by carrying counts and
/// extents across. Under a signed permutation the rotated box is exact, so the
/// two lattices agree by construction rather than by luck — and every ray then
/// lands on a ray, with its parameters unchanged.
#[must_use]
pub fn refixture_exact(field: &TriDexelField, transform: &Mat4) -> Option<TriDexelField> {
    let spacing = field
        .bundles()
        .next()
        .map(|(_, b)| b.lattice().spacing_uv()[0])?;
    let Regime::Exact { from, flipped } = classify(transform, spacing)? else {
        return None;
    };

    // The rotated workspace, from whichever bundle is present. Every bundle of
    // a field covers the same box, so any of them will do.
    let source_box = field
        .bundles()
        .next()
        .map(|(_, b)| workspace_of(b.lattice()))?;
    let rotated = rotate_box(&source_box, transform);

    let mut bundles: [Option<DexelField>; 3] = [None, None, None];
    for (index, dest) in AXES.into_iter().enumerate() {
        let source_axis = from[index];
        let Some(source) = field.bundle(source_axis) else {
            continue;
        };
        let src_lattice = source.lattice();
        let dst_lattice = Lattice::anisotropic(
            rotated,
            [
                src_lattice.spacing_uv()[0],
                src_lattice.spacing_uv()[0],
                src_lattice.spacing_uv()[0],
            ],
            dest,
        )
        .ok()?;
        let mut arena = Arena::new(dst_lattice.ray_count());

        let sign = if flipped[index] { -1.0 } else { 1.0 };
        let d = dest.index();
        let mut moved = Vec::new();
        for ray in 0..u32::try_from(src_lattice.ray_count()).unwrap_or(u32::MAX) {
            let spans = source.arena().get(ray);
            if spans.is_empty() {
                continue;
            }
            let (i, j) = src_lattice.coords(ray);
            let start = src_lattice.origin_of(i, j);
            let image = transform.transform_point(start);
            let Some(target) = ray_at_point(&dst_lattice, image) else {
                // A ray whose image falls outside the destination lattice would
                // mean the rotated workspace does not cover the rotated stock,
                // which is a bug here rather than a case to skip quietly.
                return None;
            };
            let dst_start = dst_lattice.origin_of(target.0, target.1);
            let delta = image.to_array()[d] - dst_start.to_array()[d];

            moved.clear();
            if sign > 0.0 {
                for s in spans {
                    moved.push(Span::with_normals(
                        s.t0 + delta,
                        s.t1 + delta,
                        rotate_normal(s.n0, transform),
                        rotate_normal(s.n1, transform),
                    ));
                }
            } else {
                // The ray runs the other way, so the intervals reverse and each
                // one's ends swap -- including which normal belongs to which.
                for s in spans.iter().rev() {
                    moved.push(Span::with_normals(
                        delta - s.t1,
                        delta - s.t0,
                        rotate_normal(s.n1, transform),
                        rotate_normal(s.n0, transform),
                    ));
                }
            }
            arena.set(dst_lattice.index(target.0, target.1), &moved);
        }
        bundles[index] = Some(DexelField::from_parts(dst_lattice, arena, Mat4::IDENTITY));
    }
    Some(TriDexelField::from_parts(
        bundles,
        field.provenance().clone(),
    ))
}

/// The image of an axis-aligned box under a signed permutation.
///
/// Exact: every coordinate is copied or negated, never combined.
fn rotate_box(b: &Aabb3, transform: &Mat4) -> Aabb3 {
    let (lo, hi) = (b.min.to_array(), b.max.to_array());
    let mut out = Aabb3::EMPTY;
    for corner in 0..8u8 {
        let p = Vec3::new(
            if corner & 1 == 0 { lo[0] } else { hi[0] },
            if corner & 2 == 0 { lo[1] } else { hi[1] },
            if corner & 4 == 0 { lo[2] } else { hi[2] },
        );
        out = out.union_point(transform.transform_point(p));
    }
    out
}

/// Rotates an encoded normal.
///
/// Decode, rotate, re-encode. Exact for a signed permutation, because the
/// octahedral encoding is odd-symmetric about zero — measured over 40 000
/// normals rather than assumed. See the module header.
fn rotate_normal(n: OctNormal, transform: &Mat4) -> OctNormal {
    OctNormal::encode(transform.transform_direction(n.decode()))
}

/// Which ray of `lattice` passes through `point`.
fn ray_at_point(lattice: &Lattice, point: Vec3) -> Option<(u32, u32)> {
    let [u, v, _] = lattice.axis().cyclic();
    let pad = lattice.pad();
    let origin = lattice.origin().to_array();
    let p = point.to_array();
    let spacing = lattice.spacing_uv();
    let counts = lattice.counts();
    let index = |world: usize, k: usize| -> Option<u32> {
        let raw = (p[world] - origin[world] + pad[k]) / spacing[k] - 0.5;
        let rounded = raw.round();
        // A ray image that does not land on a ray means the two lattices are
        // not registered, which is the failure this whole approach exists to
        // avoid. Half a thousandth of a cell is far looser than the exact case
        // needs and far tighter than a genuine misregistration.
        if (raw - rounded).abs() > 1.0e-3 || rounded < 0.0 {
            return None;
        }
        #[allow(clippy::cast_possible_truncation, reason = "range checked below")]
        let n = rounded as i64;
        u32::try_from(n).ok().filter(|k2| *k2 < counts[k])
    };
    Some((index(u, 0)?, index(v, 1)?))
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
    fn an_accumulated_bound_of_zero_is_positive_zero() {
        // **`Sum` for f64 uses -0.0 as its identity**, so an empty sum is
        // negative zero -- and it reached the published JSON as
        // `"accumulated_transform_bound_mm": -0.0` on every single-setup job.
        //
        // It compares equal to zero in most languages and is a *different
        // value* to this engine's canonical hashing, which covers signed zero
        // deliberately. A published field must not depend on a reader knowing
        // that, so the fold starts from positive zero.
        let exact = Regime::Exact {
            from: [Axis::X, Axis::Y, Axis::Z],
            flipped: [false; 3],
        };
        for regimes in [&[][..], &[exact][..], &[exact, exact][..]] {
            let b = accumulated_bound(regimes);
            assert_eq!(b, 0.0);
            assert!(
                b.is_sign_positive(),
                "an accumulated bound of zero came out negative for {} boundary(ies),                  which serialises as -0.0",
                regimes.len()
            );
        }
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
