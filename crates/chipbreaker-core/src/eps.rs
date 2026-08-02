// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Named tolerance constants.
//!
//! **Every tolerance in the engine lives here.** A bare `1e-9` at a call site is
//! a latent bug: it carries no units, no rationale, and no way to audit what
//! happens when the workspace scale changes. If you need a new tolerance, add it
//! here with a written justification.
//!
//! # Units
//!
//! Chipbreaker's model unit is the **millimetre**. All length tolerances below
//! are absolute, in millimetres, and are chosen against the largest workspace we
//! intend to support: a 10 m (10<sup>4</sup> mm) cube, which is comfortably
//! larger than any machine tool we target.
//!
//! The relevant scale for "how small can a meaningful absolute tolerance be" is
//! the spacing of `f64` values at the far corner of that workspace:
//!
//! | coordinate | ULP        |
//! |-----------:|-----------:|
//! | 1 mm       | 2.2e-16 mm |
//! | 10^3 mm    | 1.1e-13 mm |
//! | 10^4 mm    | 1.8e-12 mm |
//!
//! A tolerance of 1e-9 mm is therefore roughly 550 ULP at the worst-case
//! coordinate — enough headroom to absorb accumulated rounding from a chain of
//! transforms, while being 1000x smaller than the finest machining tolerance
//! anyone cares about (1 µm = 1e-3 mm).
//!
//! # Determinism
//!
//! These are compile-time `f64` constants written as exactly-representable-ish
//! decimal literals. Rust's decimal-to-`f64` conversion is correctly rounded and
//! platform-independent, so every target sees identical bit patterns.

/// Minimum length of a vector that can be normalized.
///
/// Below this, the direction is numerically meaningless and
/// [`crate::math::Vec3::normalize`] returns `None` rather than producing NaN or
/// an arbitrary unit vector. Chosen as the square root of the smallest normal
/// `f64` scaled up generously: any vector this short would lose most of its
/// mantissa to the division.
pub const EPS_NORMALIZE: f64 = 1e-300;

/// Absolute tolerance for "are these two lengths the same" comparisons in
/// millimetres.
///
/// This is the general-purpose geometric tolerance. It is **not** used for any
/// orientation or sidedness question — those go through
/// [`crate::predicates`], which are exact.
pub const EPS_LENGTH: f64 = 1e-9;

/// Relative tolerance for comparing quantities whose magnitude is not known in
/// advance (e.g. a determinant, an accumulated measure).
///
/// Used as `|a - b| <= EPS_LENGTH + EPS_RELATIVE * max(|a|, |b|)`. The absolute
/// term keeps the comparison meaningful near zero, where a purely relative test
/// degenerates.
pub const EPS_RELATIVE: f64 = 1e-12;

/// Determinant magnitude below which a [`crate::math::Mat3`] or
/// [`crate::math::Mat4`] is treated as singular and inversion returns `None`.
///
/// This is deliberately tiny: it is a guard against dividing by zero, not a
/// conditioning test. A matrix with a determinant just above this threshold is
/// invertible in the arithmetic sense but numerically useless, and it is the
/// caller's job to know that. Machining transforms are rigid-body plus uniform
/// scale, so a near-singular matrix means the caller made a mistake upstream.
pub const EPS_DETERMINANT: f64 = 1e-300;

/// Gap below which two otherwise-disjoint spans are merged into one, in
/// millimetres.
///
/// # Rationale
///
/// A dexel ray crossing a solid can pick up a hairline gap purely from rounding:
/// two triangles that share an edge exactly in the model may produce ray
/// intersections that differ in the last few ULP, leaving a sub-nanometre void
/// where the solid is in fact continuous. Left alone, those voids accumulate
/// across thousands of cuts and eventually surface as spurious "excess stock"
/// slivers in the U12 deviation field.
///
/// 1e-9 mm sits far above that rounding noise (~550 ULP at a 10 m coordinate,
/// see the module docs) and far below any feature a machinist can produce or
/// measure — the finest achievable surface finish is on the order of 1e-4 mm.
///
/// # Interaction with dexel resolution (U5)
///
/// This constant is a *rounding-noise* threshold and is intentionally decoupled
/// from dexel spacing, which is a *sampling* parameter typically in the 1e-2 to
/// 1e-1 mm range — seven orders of magnitude larger. Tying the two together
/// would make the span algebra's behaviour depend on the field resolution, and
/// then a resolution change could alter topology rather than just sample
/// density. Keep them separate.
pub const EPS_SPAN_MERGE: f64 = 1e-9;

/// Minimum length of a span that survives normalization, in millimetres.
///
/// Spans shorter than this are dropped by [`crate::spans::Spans::normalize`]
/// and by every set operation.
///
/// # Why this equals [`EPS_SPAN_MERGE`]
///
/// A gap of length *g* and a span of length *g* describe the same physical
/// scale — one is absence of material, the other presence of it — and there is
/// no principled reason to treat them asymmetrically. Making them equal also
/// makes normalization idempotent: after one pass, every gap is strictly
/// greater than the threshold and every span is at least the threshold, so a
/// second pass is a no-op. If the two differed, a single pass could leave the
/// set in a state that a second pass would change again, which is exactly the
/// kind of quiet non-convergence that makes a geometry kernel untrustworthy.
///
/// See [`crate::spans`] for the normalization order (merge first, then drop) and
/// why that order is the one that terminates.
pub const EPS_SPAN_MIN: f64 = 1e-9;

/// Compares two lengths with the combined absolute/relative tolerance described
/// on [`EPS_RELATIVE`].
///
/// Returns `false` if either input is NaN.
#[inline]
#[must_use]
pub fn approx_eq(a: f64, b: f64) -> bool {
    if a.is_nan() || b.is_nan() {
        return false;
    }
    if a == b {
        // Catches the infinities, which the subtraction below would turn to NaN.
        return true;
    }
    let diff = (a - b).abs();
    if !diff.is_finite() {
        // Opposite infinities, or one infinite operand. Without this the
        // comparison below reads `inf <= inf`, which is true, and reports two
        // opposite infinities as approximately equal.
        return false;
    }
    let scale = a.abs().max(b.abs());
    diff <= EPS_LENGTH + EPS_RELATIVE * scale
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn approx_eq_handles_exact_equality_and_nan() {
        assert!(approx_eq(1.0, 1.0));
        assert!(approx_eq(f64::INFINITY, f64::INFINITY));
        assert!(!approx_eq(f64::NAN, f64::NAN));
        assert!(!approx_eq(1.0, f64::NAN));
        assert!(!approx_eq(f64::INFINITY, f64::NEG_INFINITY));
    }

    #[test]
    fn approx_eq_scales_with_magnitude() {
        // Near zero the absolute term dominates.
        assert!(approx_eq(0.0, 0.5 * EPS_LENGTH));
        assert!(!approx_eq(0.0, 10.0 * EPS_LENGTH));
        // At large magnitude the relative term takes over.
        assert!(approx_eq(1.0e6, 1.0e6 + 1.0e-7));
        assert!(!approx_eq(1.0e6, 1.0e6 + 1.0e-3));
    }

    #[test]
    fn span_thresholds_are_consistent() {
        // The idempotence argument on EPS_SPAN_MIN depends on this.
        assert!((EPS_SPAN_MIN - EPS_SPAN_MERGE).abs() < f64::EPSILON);
        // And both must be far above the ULP at the far corner of the largest
        // workspace we claim to support, or normalization would be chasing
        // rounding noise rather than filtering it.
        let ulp_at_worst_case = (1.0e4_f64).next_up() - 1.0e4_f64;
        assert!(EPS_SPAN_MERGE > 100.0 * ulp_at_worst_case);
    }
}
