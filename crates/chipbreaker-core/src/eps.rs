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

/// Lattice spacing used to weld coincident vertices, in millimetres.
///
/// # Why a lattice rather than a tolerance
///
/// Tolerance-based welding — "merge `a` and `b` if `|a - b| < eps`" — is **not
/// transitive**. `a` can be within tolerance of `b`, and `b` of `c`, while `a`
/// and `c` are not. Whether `a` and `c` end up welded then depends on the order
/// the pairs were considered in, which is precisely the class of order-dependent
/// result the whole project forbids.
///
/// Snapping to a lattice is transitive by construction: two coordinates weld iff
/// they round to the same lattice point, and that is a property of each
/// coordinate alone. See [`crate::mesh::weld`] for the full argument, including
/// the cost this trade accepts.
///
/// # Why 1e-6 mm
///
/// One nanometre. Two orders of magnitude below the finest surface finish any
/// machining process produces (~1e-4 mm), so it never merges two vertices a
/// machinist would consider distinct.
///
/// It also has to sit above the noise in the input. Binary STL stores `f32`,
/// whose 24-bit mantissa gives a resolution of about 6e-6 mm at a 100 mm
/// coordinate — so vertices that were coincident in the CAD system arrive
/// differing by a few units in the last `f32` place. A lattice finer than that
/// would fail to weld them and leave the mesh non-manifold for no reason. 1e-6 mm
/// is comfortably coarser than `f32` noise at part scale and comfortably finer
/// than any real feature.
///
/// Overridable per-invocation with `--weld-tol`.
pub const EPS_WELD: f64 = 1e-6;

/// Relative magnitude below which a floating-point edge function in the
/// ray-triangle test is considered untrustworthy, forcing the exact fallback.
///
/// The fast path computes the three edge functions in `f64` and accepts the
/// classification when every one of them is comfortably away from zero.
/// "Comfortably" means: larger than this multiple of the magnitude of the terms
/// that produced it, so that the accumulated rounding error cannot have changed
/// the sign.
///
/// A 3x3 determinant of differences accumulates a relative error of a few units
/// in the last place — call it `8 * f64::EPSILON` to be generous, roughly
/// `1.8e-15`. `1e-12` leaves three orders of magnitude of margin, which costs a
/// slightly higher exact-fallback rate and buys certainty that the fast path is
/// never wrong. Being wrong here means a leaked ray, and a leaked ray is a
/// tunnel through the simulated stock.
pub const EPS_EDGE_FN: f64 = 1e-12;

/// Leading-coefficient threshold below which a polynomial degrades one degree.
///
/// A quartic whose `a` satisfies `|a| <= ROOT_DEGENERACY_TAU * max|other|` is
/// solved as a cubic, and so on down.
///
/// # Why discarding a root is safe
///
/// As `a` tends to zero a quartic root does not vanish, it **escapes**: the
/// magnitude of the departing root grows like `|b/a|`. At this threshold that
/// puts it at `1e14` times the coefficient scale — a ray parameter around
/// `1e15` mm, when the tool's bounding cylinder is a few millimetres across. The
/// discarded root provably cannot be a hit, so degrading loses nothing real.
///
/// `1e-14` is roughly 45 machine epsilons: just above the noise already present
/// in the coefficients themselves, so a leading term that survives the test is
/// one the input actually determined.
///
/// The alternative — always solving at the stated degree — is not viable.
/// Ferrari's method divides through by `a`, so a leading coefficient near the
/// noise floor destroys the *other* three roots, which are ordinary and
/// physically real.
pub const ROOT_DEGENERACY_TAU: f64 = 1e-14;

/// The square root of [`f64::EPSILON`], exactly.
///
/// `f64::EPSILON` is `2^-52`, so its square root is `2^-26` — a power of two,
/// therefore exactly representable, therefore identical on every target. Written
/// as a literal rather than computed because `sqrt` is not a `const fn`.
///
/// # Why this number governs tangency
///
/// It is the accuracy floor for a **double root**. At a simple root, a
/// coefficient perturbation of `eps` moves the root by about `eps / |p'|` — a
/// relative error near `1e-16`. At a double root `p'` vanishes, the expansion
/// starts at the quadratic term, and the same perturbation moves the root by
/// about `sqrt(eps)` instead: a relative error near `1.5e-8`, eight digits
/// worse.
///
/// So two roots closer together than this are not "nearly equal"; they are
/// **indistinguishable by any `f64` solver**, and the gap between them is noise
/// rather than geometry. That is what makes collapsing them a fact about the
/// arithmetic rather than a tuning choice.
pub const SQRT_F64_EPSILON: f64 = 1.490_116_119_384_765_6e-8;

/// Root separation below which a ray is treated as **tangent** to a solid of
/// revolution, contributing no interval.
///
/// `scale` is the characteristic size of the solid — its bounding-cylinder
/// diagonal — because the threshold is a length and a length without a scale is
/// not a quantity.
///
/// # The policy, and why it cannot contradict [`EPS_SPAN_MIN`]
///
/// A ray grazing a tool tangentially removes no material, so it must produce
/// **no interval** — not a zero-length one, and emphatically not a sliver of
/// numerical noise. Over millions of rays those slivers accumulate into visible
/// artefacts on the simulated surface.
///
/// The threshold is [`SQRT_F64_EPSILON`] times the scale, which is exactly the
/// point below which the solver cannot tell a double root from two near ones.
/// It is then **floored at [`EPS_SPAN_MIN`]**, and that floor is what keeps the
/// two thresholds from disagreeing:
///
/// - anything this rule *keeps* is at least `EPS_SPAN_MIN` long, so
///   [`crate::spans::Spans`] keeps it too;
/// - anything `Spans` would drop, this rule has already dropped.
///
/// There is therefore no regime in which one admits what the other rejects. The
/// two are a single monotone policy, not two independently tunable numbers, and
/// this function is the only place the relationship is expressed.
///
/// For a 10 mm tool the threshold lands near `2e-7` mm — some 5000x below any
/// achievable surface finish, so nothing a machinist could produce is discarded.
#[inline]
#[must_use]
pub fn eps_tangent(scale: f64) -> f64 {
    let derived = SQRT_F64_EPSILON * scale.abs();
    if derived > EPS_SPAN_MIN {
        derived
    } else {
        EPS_SPAN_MIN
    }
}

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
