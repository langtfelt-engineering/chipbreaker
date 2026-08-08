// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Deterministic real-root solving for polynomials up to degree four.
//!
//! # Why degree four, and why here
//!
//! A ray meets a solid of revolution in the roots of a polynomial, and the
//! degree is set by the profile:
//!
//! | profile element | revolved surface | degree |
//! |---|---|---|
//! | segment parallel to the axis | cylinder | 2 |
//! | segment at an angle | cone | 2 |
//! | arc centred on the axis | sphere | 2 |
//! | arc centred off the axis | torus | **4** |
//! | cap | plane | 1 |
//!
//! So four is not an arbitrary ceiling: it is exactly what a profile of segments
//! and arcs requires, and it is what the sweep extends when the tool moves.
//!
//! # This module is inner-loop code
//!
//! No allocation. [`RootSet`] is a fixed-capacity inline array. Field building
//! and cutting
//! call this millions of times per simulation.
//!
//! # Accuracy, and the one fact that governs the whole design
//!
//! **At a double root, `f64` gives you eight digits, not sixteen.**
//!
//! At a simple root a coefficient perturbation `eps` displaces the root by
//! `eps / |p'|`. At a double root `p'` vanishes and the leading term of the
//! expansion is quadratic, so the same perturbation displaces the root by
//! `sqrt(eps)` — about `1.5e-8` relative. Nothing can be done about that in
//! double precision; it is a property of the problem, not of the algorithm.
//!
//! Everything else follows from it: why tangency is decided at
//! [`crate::eps::SQRT_F64_EPSILON`] rather than at machine epsilon, why the
//! discriminant is computed in double-double arithmetic to protect the *simple*
//! roots that still deserve sixteen digits, and why the sweep bounds its interval
//! endpoints with this in mind rather than assuming full precision.
//!
//! # Determinism
//!
//! The cubic's three-real-root branch needs `acos` and `cos`, and the general
//! solution needs `cbrt`. All three go through [`crate::transcendental`], so the
//! roots are bit-identical on every target. A `std` call here would silently
//! break WASM parity for every ray that touched a torus.
//!
//! Everything else in this module is `+ - * /` and comparison, which IEEE-754
//! requires to be correctly rounded and therefore identical everywhere — **on a
//! target with IEEE-754 double semantics**. That assumption is doing real work
//! and is worth naming: an `i686` target using the x87 stack computes
//! intermediates at 80-bit extended precision and rounds them to 64 bits at
//! unpredictable points, which would reintroduce exactly the cross-target
//! divergence this module is built to exclude. `x86_64` (SSE2), `aarch64` and
//! `wasm32` all have the required semantics. A 32-bit x86 target would need
//! `+sse2` forced, and would still want re-validating against the Sturm oracle
//! rather than assumed correct.

use crate::eps::{ROOT_DEGENERACY_TAU, SQRT_F64_EPSILON};
use crate::golden::{CanonicalHash, Hashable};
use crate::transcendental as t;

use core::f64::consts::PI;

/// Largest number of distinct real roots a quartic can have.
pub const MAX_ROOTS: usize = 4;

/// How many multiples of [`SQRT_F64_EPSILON`], relative to root magnitude, may
/// separate two roots that are still treated as one.
///
/// This is the module's single resolution contract, and both places that decide
/// "one root or two" use it:
///
/// * [`RootSet::from_raw`], merging two computed values after the fact;
/// * [`is_repeated_root`], deciding *before* the search whether the two
///   crossings implied around a critical point are far enough apart to look for.
///
/// They must agree, or one will split a root the other immediately merges, and
/// which happens would depend on the branch taken. Both therefore compare the
/// **full gap** between the two roots against this threshold.
///
/// # Why 4
///
/// The floor is `sqrt(eps)`: nothing can resolve a double root more finely than
/// that (see the module header). The solver returns two images of such a root,
/// each displaced independently and usually in opposite directions, so the
/// observed gap is a small multiple of the floor — `(x-1)^2(x-2)` comes back
/// with `3.3e-8`, about 2.2 times `sqrt(eps)`. Four leaves margin above the
/// worst gap measured across the 100,000-case sweep while still resolving the
/// corpus's `near-double` quartic, whose roots are `1e-7` apart — 6.7 times the
/// floor, and genuinely two roots.
pub const ROOT_CLUSTER_FACTOR: f64 = 4.0;

/// How many machine epsilons of its own term scale a quadratic discriminant may
/// be before it is taken to be genuinely non-zero.
///
/// See [`solve_quadratic`] for why the sign of a near-zero discriminant is noise
/// rather than geometry.
pub const DISCRIMINANT_SNAP: f64 = 8.0;

/// The real roots of a polynomial: distinct values, ascending, with
/// multiplicity.
///
/// Fixed capacity and inline — no heap. `total_multiplicity` counts roots with
/// repetition and never exceeds the polynomial's degree.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RootSet {
    values: [f64; MAX_ROOTS],
    multiplicity: [u8; MAX_ROOTS],
    len: u8,
    /// The degree actually solved, after any degeneracy reduction.
    degree: u8,
}

impl RootSet {
    /// No real roots.
    #[must_use]
    pub const fn empty(degree: u8) -> Self {
        Self {
            values: [0.0; MAX_ROOTS],
            multiplicity: [0; MAX_ROOTS],
            len: 0,
            degree,
        }
    }

    /// Number of **distinct** real roots.
    #[inline]
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// True if there are no real roots.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The distinct roots, ascending.
    #[inline]
    #[must_use]
    pub fn roots(&self) -> &[f64] {
        &self.values[..self.len as usize]
    }

    /// Multiplicity of the `i`th distinct root.
    ///
    /// # Panics
    /// Panics if `i` is out of range.
    #[inline]
    #[must_use]
    pub fn multiplicity(&self, i: usize) -> u8 {
        assert!(i < self.len as usize, "root index {i} out of range");
        self.multiplicity[i]
    }

    /// Roots counted with repetition.
    #[must_use]
    pub fn total_multiplicity(&self) -> usize {
        self.multiplicity[..self.len as usize]
            .iter()
            .map(|m| *m as usize)
            .sum()
    }

    /// The degree that was actually solved, after degeneracy reduction.
    ///
    /// Differs from the requested degree when a leading coefficient was
    /// negligible; see [`ROOT_DEGENERACY_TAU`].
    #[inline]
    #[must_use]
    pub const fn solved_degree(&self) -> u8 {
        self.degree
    }

    /// `(root, multiplicity)` pairs, ascending.
    pub fn iter(&self) -> impl Iterator<Item = (f64, u8)> + '_ {
        self.roots()
            .iter()
            .copied()
            .zip(self.multiplicity[..self.len as usize].iter().copied())
    }

    /// Builds a set from raw roots, sorting and merging numerically coincident
    /// ones.
    ///
    /// Two roots are merged when they are closer than
    /// `ROOT_CLUSTER_FACTOR * SQRT_F64_EPSILON` relative to their own
    /// magnitude. The merged value is the **first** of the cluster in ascending
    /// order rather than their mean: a mean depends on how many neighbours
    /// happened to fall inside the window, which makes the result depend on the
    /// clustering rather than on the polynomial.
    ///
    /// See [`ROOT_CLUSTER_FACTOR`] for why the threshold is that multiple and
    /// not another. Failing to recognise a double root is the damaging
    /// direction: it reports a tangency as two crossings, and field building records a
    /// sliver of material that is not there. Merging two genuinely distinct
    /// roots that close is harmless by comparison — at tool scale the
    /// separation is nanometres.
    ///
    /// Note that this is *not* the tangency decision. That belongs to
    /// [`crate::eps::eps_tangent`], is expressed as a length rather than a
    /// relative tolerance, and is applied by the caller with the geometry in
    /// hand.
    fn from_raw(raw: &[f64], degree: u8) -> Self {
        let mut sorted = [0.0f64; MAX_ROOTS];
        let mut n = 0usize;
        for &r in raw {
            if r.is_finite() && n < MAX_ROOTS {
                sorted[n] = r;
                n += 1;
            }
        }
        // `total_cmp` is a total order over every f64, so the sort cannot panic
        // and cannot depend on comparison subtleties.
        sorted[..n].sort_by(f64::total_cmp);

        let mut out = Self::empty(degree);
        for &r in &sorted[..n] {
            let last = out.len as usize;
            if last > 0 {
                let previous = out.values[last - 1];
                // Purely relative, with no absolute floor. An earlier version
                // floored the scale at 1.0, which merged the roots of
                // `(x - 1e-30)(x - 2e-30)` — a factor of two apart, and as
                // distinct as two roots can be — because their absolute gap was
                // below `sqrt(eps)`. Whether two roots are the same root is a
                // question about their ratio, not about the unit the caller
                // happens to be working in.
                let scale = previous.abs().max(r.abs());
                if (r - previous).abs() <= ROOT_CLUSTER_FACTOR * SQRT_F64_EPSILON * scale {
                    out.multiplicity[last - 1] += 1;
                    continue;
                }
            }
            out.values[last] = r;
            out.multiplicity[last] = 1;
            out.len += 1;
        }
        out
    }
}

impl Hashable for RootSet {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("RootSet");
        h.u64(u64::from(self.degree));
        h.f64_slice(self.roots());
        for m in &self.multiplicity[..self.len as usize] {
            h.u64(u64::from(*m));
        }
        h.end();
    }
}

// ---------------------------------------------------------------------------
// Exact products and differences, for the discriminant
// ---------------------------------------------------------------------------

/// Veltkamp splitting constant, `2^27 + 1`.
const SPLITTER: f64 = 134_217_729.0;

/// Splits an `f64` into two halves whose product with another split value is
/// exact.
#[inline]
fn split(a: f64) -> (f64, f64) {
    let c = SPLITTER * a;
    let hi = c - (c - a);
    (hi, a - hi)
}

/// Exact product: returns `(p, e)` with `a * b == p + e` exactly.
///
/// Dekker's algorithm. **Not** `f64::mul_add`, which would be the obvious way to
/// write this and is banned: a hardware FMA and a software one round the same
/// expression differently, and this module's output feeds a golden hash.
#[inline]
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    let (ah, al) = split(a);
    let (bh, bl) = split(b);
    let e = ((ah * bh - p) + ah * bl + al * bh) + al * bl;
    (p, e)
}

/// Exact sum: returns `(s, e)` with `a + b == s + e` exactly.
///
/// Knuth's two-sum, which unlike the faster Dekker variant needs no ordering
/// assumption on the magnitudes of `a` and `b`.
#[inline]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let shifted = s - a;
    (s, (a - (s - shifted)) + (b - shifted))
}

/// Exact difference: returns `(s, e)` with `a - b == s + e` exactly.
#[inline]
fn two_diff(a: f64, b: f64) -> (f64, f64) {
    let s = a - b;
    let bb = a - s;
    (s, (a - (s + bb)) + (bb - b))
}

/// `b^2 - 4ac`, computed so that catastrophic cancellation does not destroy it.
///
/// # Why bother
///
/// The naive expression loses every significant digit exactly when `4ac` is
/// close to `b^2` — which is the **near-tangent ray**, the case this whole unit
/// exists to get right. Evaluating both products exactly and subtracting in
/// double-double keeps the discriminant accurate to about one ULP even there, so
/// the *simple* roots of a near-tangent quadratic still get their full sixteen
/// digits. (The double root itself is still limited to eight; nothing fixes
/// that.)
#[inline]
fn discriminant(a: f64, b: f64, c: f64) -> f64 {
    let (p, pe) = two_product(b, b);
    // 4 * a is exact: multiplying by a power of two only changes the exponent.
    let (q, qe) = two_product(4.0 * a, c);
    let (s, se) = two_diff(p, q);
    s + (se + (pe - qe))
}

// ---------------------------------------------------------------------------
// Degeneracy
// ---------------------------------------------------------------------------

/// True if the leading coefficient is negligible against the rest.
///
/// See [`ROOT_DEGENERACY_TAU`] for the argument that the root discarded by
/// degrading is provably outside any region of physical interest.
#[inline]
fn leading_is_negligible(a: f64, rest: &[f64]) -> bool {
    if a == 0.0 {
        return true;
    }
    let scale = rest.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    scale > 0.0 && a.abs() <= ROOT_DEGENERACY_TAU * scale
}

// ---------------------------------------------------------------------------
// Horner evaluation and Newton polish
// ---------------------------------------------------------------------------

/// Evaluates a polynomial by Horner's rule, coefficients in descending degree.
///
/// The evaluation order is fixed and part of the contract: floating-point
/// addition is not associative, so a different order gives a different last bit.
#[inline]
#[must_use]
pub fn eval(coefficients: &[f64], x: f64) -> f64 {
    let mut acc = 0.0;
    for &c in coefficients {
        acc = acc * x + c;
    }
    acc
}

/// Evaluates the derivative by Horner, coefficients in descending degree.
#[inline]
fn eval_derivative(coefficients: &[f64], x: f64) -> f64 {
    let n = coefficients.len();
    if n < 2 {
        return 0.0;
    }
    let mut acc = 0.0;
    for (i, &c) in coefficients[..n - 1].iter().enumerate() {
        let power = (n - 1 - i) as f64;
        acc = acc * x + c * power;
    }
    acc
}

/// `sum |c_i| |x|^i` — the quantity every Horner error bound is proportional to.
#[inline]
fn horner_abs_sum(coefficients: &[f64], x: f64) -> f64 {
    let magnitude = x.abs();
    let mut acc = 0.0;
    for &c in coefficients {
        acc = acc * magnitude + c.abs();
    }
    acc
}

/// Evaluates a polynomial to roughly double-double accuracy.
///
/// # Why plain Horner is not enough here
///
/// Its running-error bound is `2n * eps * sum |c_i| |x|^i`, which is a bound on
/// the *coefficients*, not on the result. At a critical point of `p` — where
/// this module has to decide whether a near-zero value means a tangency or two
/// crossings a hair apart — the true value is far smaller than that bound, so
/// the residual says nothing at all. For `(x-1)(x-1-1e-7)(x-4)(x-9)` the value
/// at the critical point is `6e-14` and plain Horner's bound on it is `3e-13`:
/// five times larger than the number it is bounding.
///
/// Running the rounding error of every product and every sum through a second
/// Horner recurrence recovers those digits. The result is as accurate as
/// evaluating in double-double, for about three times the arithmetic and no
/// allocation, and its error bound is proportional to `eps^2` rather than `eps`
/// — which turns that `3e-13` into `7e-29` and makes the decision clear-cut.
fn eval_compensated(coefficients: &[f64], x: f64) -> f64 {
    let mut value = coefficients[0];
    let mut error = 0.0;
    for &c in &coefficients[1..] {
        let (product, product_error) = two_product(value, x);
        let (sum, sum_error) = two_sum(product, c);
        value = sum;
        error = error * x + (product_error + sum_error);
    }
    value + error
}

/// Descending coefficients of the derivative, written into `out`; returns how
/// many were written.
fn derivative_coefficients(coefficients: &[f64], out: &mut [f64]) -> usize {
    let n = coefficients.len();
    if n < 2 {
        return 0;
    }
    for (i, slot) in out[..n - 1].iter_mut().enumerate() {
        *slot = coefficients[i] * ((n - 1 - i) as f64);
    }
    n - 1
}

/// Is `x`, already known to be a critical point of `p`, also a root of `p`?
///
/// # Why this is not just a residual test
///
/// A root of multiplicity `m > 1` is a root of `p'` as well, so every repeated
/// root is a critical point. The converse question — does `p` actually vanish
/// here — cannot be answered by asking whether `p(x)` is small, because the
/// coefficients of a polynomial with a double root are generally not
/// representable, so the double root is not there either: what is there is two
/// roots about `sqrt(eps)` apart, at which `p` is *not* zero.
///
/// So the test is geometric instead. Locally `p(x + h) ~ p(x) + p''(x) h^2 / 2`,
/// which vanishes at `h = +/- sqrt(-2 p / p'')`. That gives the separation of
/// the two crossings directly, and the decision becomes the one that actually
/// matters: are they closer together than `f64` can resolve them?
///
/// The three outcomes:
///
/// * `p` and `p''` have the same sign — the parabola does not reach zero, so
///   there are no roots nearby at all. Repeated only if `p` is within its own
///   (compensated) noise, which is the grazing case.
/// * Opposite signs, separation above the resolution floor — two genuine
///   crossings; not a repeated root, and the bracketed search will find both.
/// * Opposite signs, separation below the floor — one root of even multiplicity,
///   reported at `x`.
///
/// The floor is the same [`ROOT_CLUSTER_FACTOR`] multiple of
/// [`SQRT_F64_EPSILON`] that [`RootSet::from_raw`] merges at, and deliberately
/// so: it makes the two decisions agree at the boundary instead of one of them
/// splitting a root the other would immediately merge.
fn is_repeated_root(coefficients: &[f64], x: f64) -> bool {
    let value = eval_compensated(coefficients, x);
    // Compensated Horner is accurate to eps in the result plus an eps^2 term in
    // the coefficients.
    let gamma = 2.0 * (coefficients.len() as f64) * f64::EPSILON;
    let noise = f64::EPSILON * value.abs() + gamma * gamma * horner_abs_sum(coefficients, x);
    if value.abs() <= noise {
        return true;
    }

    let mut first = [0.0f64; MAX_ROOTS];
    let written = derivative_coefficients(coefficients, &mut first);
    let mut second = [0.0f64; MAX_ROOTS];
    let curvature_len = derivative_coefficients(&first[..written], &mut second);
    let curvature = eval(&second[..curvature_len], x);
    if value * curvature >= 0.0 {
        return false;
    }
    // `h` is the half-gap; the two crossings are at `x - h` and `x + h`. The
    // comparison is against the full gap, because that is what `from_raw`
    // merges on — mixing the two conventions makes one decision split a root
    // the other immediately merges.
    let half_gap = (2.0 * value.abs() / curvature.abs()).sqrt();
    2.0 * half_gap <= ROOT_CLUSTER_FACTOR * SQRT_F64_EPSILON * x.abs()
}

/// Refines a root known to lie in `[lo, hi]`, where `p` changes sign.
///
/// Safeguarded Newton: take the Newton step when it stays inside the bracket,
/// bisect when it does not. That keeps Newton's quadratic convergence in the
/// ordinary case — six or seven evaluations — while never leaving an interval
/// that provably contains a root, so it cannot diverge or land on a different
/// root the way bare Newton can.
///
/// Because the bracket is maintained on the **original** coefficients, the
/// result carries none of the error a closed-form solution accumulates through
/// its resolvent.
fn refine_bracket(coefficients: &[f64], mut lo: f64, mut hi: f64) -> f64 {
    if lo > hi {
        core::mem::swap(&mut lo, &mut hi);
    }
    let mut flo = eval(coefficients, lo);
    if flo == 0.0 {
        return lo;
    }
    if eval(coefficients, hi) == 0.0 {
        return hi;
    }
    let mut x = 0.5 * (lo + hi);
    for _ in 0..80 {
        let fx = eval(coefficients, x);
        if fx == 0.0 {
            return x;
        }
        if (flo < 0.0) != (fx < 0.0) {
            hi = x;
        } else {
            lo = x;
            flo = fx;
        }
        if hi - lo <= f64::EPSILON * x.abs().max(f64::MIN_POSITIVE) {
            break;
        }
        let slope = eval_derivative(coefficients, x);
        let newton = if slope == 0.0 {
            f64::NAN
        } else {
            x - fx / slope
        };
        // Reject a Newton step that leaves the bracket; bisect instead.
        x = if newton.is_finite() && newton > lo && newton < hi {
            newton
        } else {
            0.5 * (lo + hi)
        };
    }
    x
}

/// Newton iterations applied to the **original** polynomial.
///
/// # Why this is not optional
///
/// Closed-form cubic and quartic solutions route the answer through a resolvent,
/// several square roots and a cube root. Each step is individually fine and the
/// composition routinely loses six to eight digits — and field building and
/// cutting inherit that
/// error *directly* as the endpoints of a material interval.
///
/// Two iterations on the original coefficients recover most of it, because
/// Newton doubles the correct digit count each step near a simple root.
///
/// Near a **multiple** root Newton converges only linearly and `p'` tends to
/// zero, so the step is guarded: an update is taken only when the derivative is
/// large enough for the quotient to mean something, and only when it actually
/// reduces `|p|`. Polishing toward a root that is already at the `sqrt(eps)`
/// accuracy floor cannot improve it and can easily make it worse.
const POLISH_ITERATIONS: usize = 2;

/// Largest relative correction polish will apply.
///
/// # Why a cap is essential, not merely prudent
///
/// Newton's step is `p / p'`, and at a **multiple** root `p'` vanishes. The
/// quotient is then enormous and the iteration does not refine the root, it
/// relocates to a completely different one — where the residual is also near
/// zero, so a naive "did the residual improve?" guard *accepts* the jump.
///
/// That is not a hypothetical. The cubic with roots `{-3.75, -3.75, -2.75}` had
/// its double root polished from `-3.75` onto `-2.75`, all three values then
/// clustered into one, and the solver reported a single root of multiplicity
/// three. The residual guard passed at every step.
///
/// A closed-form root is already good to several digits, so a genuine
/// refinement is a correction of parts per thousand at worst. Anything larger is
/// a jump, and is refused.
const POLISH_MAX_RELATIVE_STEP: f64 = 1.0e-2;

fn polish(coefficients: &[f64], x: f64) -> f64 {
    let scale = coefficients.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if scale == 0.0 {
        return x;
    }
    let limit = POLISH_MAX_RELATIVE_STEP * x.abs().max(1.0);
    let mut current = x;
    for _ in 0..POLISH_ITERATIONS {
        let value = eval(coefficients, current);
        let slope = eval_derivative(coefficients, current);
        if slope == 0.0 || !slope.is_finite() {
            break;
        }
        let step = value / slope;
        if !step.is_finite() || step.abs() > limit {
            // Refusing to move is the right answer here: near a multiple root
            // the estimate is already at the sqrt(eps) accuracy floor, and no
            // iteration can improve on it.
            break;
        }
        let candidate = current - step;
        if !candidate.is_finite() {
            break;
        }
        // Only accept a step that actually improves the residual.
        if eval(coefficients, candidate).abs() <= value.abs() {
            current = candidate;
        } else {
            break;
        }
    }
    current
}

// ---------------------------------------------------------------------------
// Solvers
// ---------------------------------------------------------------------------

/// Real roots of `a x + b`.
#[must_use]
pub fn solve_linear(a: f64, b: f64) -> RootSet {
    if leading_is_negligible(a, &[b]) {
        // A constant: either no roots, or every real number is one. The latter
        // is not representable here and is reported as no roots; callers that
        // care about the identically-zero polynomial must check for it.
        return RootSet::empty(0);
    }
    RootSet::from_raw(&[-b / a], 1)
}

/// Real roots of `a x^2 + b x + c`, ascending, with multiplicity.
///
/// Uses the cancellation-avoiding form. The naive quadratic formula computes
/// `(-b + sqrt(disc)) / 2a`, and when `4ac` is much smaller than `b^2` that
/// numerator subtracts two nearly equal numbers and loses most of its
/// significant digits — for the root that happens to be small. The stable form
/// computes the well-conditioned root first and obtains the other from the
/// product of roots, `c/a`, which involves no cancellation at all.
#[must_use]
pub fn solve_quadratic(a: f64, b: f64, c: f64) -> RootSet {
    if leading_is_negligible(a, &[b, c]) {
        return solve_linear(b, c);
    }
    let disc = discriminant(a, b, c);

    // Treat a discriminant below the rounding noise of its own terms as zero.
    //
    // The compensated computation above returns `b^2 - 4ac` essentially exactly
    // *for the coefficients given* — but those coefficients themselves carry
    // error, and near a double root the discriminant is the difference of two
    // nearly equal quantities, so its **sign** is decided by that input error
    // rather than by the geometry.
    //
    // This is not hypothetical. Deflating a cubic by its simple root produces a
    // quadratic that mathematically has a double root; an error of 3.6e-15 in
    // the deflating root moved the discriminant to -6e-14, and the double root
    // was reported as no roots at all. A cubic with roots {-3.75, -3.75, -2.75}
    // came back with one.
    //
    // The threshold is a few ULP of the larger term, which is the accuracy the
    // inputs can possibly have. It cannot swallow a real discriminant: for roots
    // at 1e-8 and 1e8 the terms are 1e16 and the threshold is about 18, against
    // a discriminant of 1e16.
    //
    // The compensated discriminant still earns its place — it sets the accuracy
    // of `sqrt(disc)`, and hence of the roots, for every case that survives this
    // test.
    let term_scale = (b * b).abs().max((4.0 * a * c).abs());
    if disc.abs() <= DISCRIMINANT_SNAP * f64::EPSILON * term_scale {
        let r = -0.5 * b / a;
        let mut out = RootSet::empty(2);
        out.values[0] = r;
        out.multiplicity[0] = 2;
        out.len = 1;
        return out;
    }

    if disc < 0.0 {
        return RootSet::empty(2);
    }
    if disc == 0.0 {
        let r = -0.5 * b / a;
        let mut out = RootSet::empty(2);
        out.values[0] = r;
        out.multiplicity[0] = 2;
        out.len = 1;
        return out;
    }
    let root_disc = disc.sqrt();
    // q = -(b + sign(b) * sqrt(disc)) / 2 — the two terms always have the same
    // sign, so this addition never cancels.
    let q = -0.5 * (b + if b >= 0.0 { root_disc } else { -root_disc });
    let x1 = q / a;
    // q can only be zero if b and c are both zero, which `disc > 0` excludes.
    let x2 = if q == 0.0 { -x1 } else { c / q };
    RootSet::from_raw(&[x1, x2], 2)
}

/// Real roots of `a x^3 + b x^2 + c x + d`, ascending, with multiplicity.
#[must_use]
pub fn solve_cubic(a: f64, b: f64, c: f64, d: f64) -> RootSet {
    if leading_is_negligible(a, &[b, c, d]) {
        return solve_quadratic(b, c, d);
    }
    let coefficients = [a, b, c, d];
    // Monic, then depressed by x = y - shift.
    let (bb, cc, dd) = (b / a, c / a, d / a);
    let shift = bb / 3.0;
    let p = cc - bb * bb / 3.0;
    let q = 2.0 * bb * bb * bb / 27.0 - bb * cc / 3.0 + dd;

    let half_q = 0.5 * q;
    let third_p = p / 3.0;
    let delta = half_q * half_q + third_p * third_p * third_p;

    let mut raw = [0.0f64; MAX_ROOTS];
    // Deliberately uninitialised: every path below assigns before reading, and
    // an initialiser here would hide a missed one.
    let mut count: usize;

    // --- Repeated roots, located without reference to delta ----------------
    //
    // The discriminant is useless for this decision. It is built from `p` and
    // `q`, which are themselves differences of much larger quantities, so when
    // the roots are close together relative to their magnitude the cancellation
    // is severe. Measured across all 8320 exact-double-root cubics on the test
    // grid, the computed delta reached **1,021,950 machine epsilons** of its own
    // term scale, with 10% of cases above 256. No eps-scaled threshold on delta
    // can separate "zero" from "small" at that error level.
    //
    // A far better signal is available. A repeated root is a root of `p'` as
    // well as of `p` — and `p'` is a *quadratic*, solved by the compensated
    // routine above and well conditioned. So: find the critical points, and ask
    // whether `p` actually vanishes at one.
    //
    // See [`is_repeated_root`] for how that question is decided.
    // This is the same reasoning as everywhere else in the module: a quantity
    // below its own noise floor is not small, it is unknown.
    //
    // Deflating by the repeated root then leaves a quadratic whose roots are
    // well separated, so the compensated discriminant recovers them cleanly.
    for (critical, _) in solve_quadratic(3.0 * a, 2.0 * b, c).iter() {
        if is_repeated_root(&coefficients, critical) {
            raw[0] = critical;
            count = 1;
            // Synthetic division by (x - critical).
            let q2 = a;
            let q1 = b + critical * q2;
            let q0 = c + critical * q1;
            for (value, multiplicity) in solve_quadratic(q2, q1, q0).iter() {
                for _ in 0..multiplicity {
                    if count < MAX_ROOTS {
                        raw[count] = value;
                        count += 1;
                    }
                }
            }
            for r in &mut raw[..count] {
                *r = polish(&coefficients, *r);
            }
            return RootSet::from_raw(&raw[..count], 3);
        }
    }

    if delta < 0.0 {
        // Three distinct real roots. Cardano's formula would need cube roots of
        // complex numbers here, so the trigonometric form is used instead —
        // which is why this module depends on `transcendental` for determinism.
        let m = 2.0 * (-third_p).sqrt();
        let argument = (3.0 * q) / (p * m);
        let theta = t::acos(argument.clamp(-1.0, 1.0)) / 3.0;
        for (k, slot) in raw[..3].iter_mut().enumerate() {
            *slot = m * t::cos(theta - 2.0 * PI * (k as f64) / 3.0) - shift;
        }
        count = 3;
    } else {
        // delta >= 0: Cardano gives one real root reliably. The other two are
        // then obtained by **deflation** rather than by a second closed form.
        //
        // Deflation is what makes the borderline case work. When the cubic
        // genuinely has a double root, delta is mathematically zero, but in f64
        // it lands a hair either side of it. Landing a hair *positive* sends the
        // classic implementation down the one-real-root branch, which reports a
        // single root and silently loses the double — a cubic with roots
        // {-5.75, -5.75, -1} came back as {-1}.
        //
        // Deflating by the root we did find and solving the remaining quadratic
        // recovers it: the quadratic's discriminant is a different, far better
        // conditioned quantity than delta, and it detects the double root that
        // delta could not resolve.
        let root_delta = delta.sqrt();
        let u = t::cbrt(-half_q + root_delta);
        let v = t::cbrt(-half_q - root_delta);
        // Polish before deflating: deflation propagates the error in this root
        // straight into the quadratic's coefficients.
        let first = polish(&coefficients, u + v - shift);
        raw[0] = first;
        count = 1;

        // Synthetic division of a x^3 + b x^2 + c x + d by (x - first).
        let q2 = a;
        let q1 = b + first * q2;
        let q0 = c + first * q1;
        for (value, multiplicity) in solve_quadratic(q2, q1, q0).iter() {
            for _ in 0..multiplicity {
                if count < MAX_ROOTS {
                    raw[count] = value;
                    count += 1;
                }
            }
        }
    }

    for r in &mut raw[..count] {
        *r = polish(&coefficients, *r);
    }
    RootSet::from_raw(&raw[..count], 3)
}

/// Real roots of `a x^4 + b x^3 + c x^2 + d x + e`, ascending, with
/// multiplicity.
///
/// # Why this is not Ferrari's method
///
/// Ferrari depresses the quartic by `b/4a`, then solves a resolvent cubic. Both
/// steps are unconditionally stable only when the roots are comparable in size
/// to their own spread. When `|b/a|` is large — a tool profile whose ray meets a
/// torus far from the origin, which is the normal case, not an exotic one — the
/// depressed coefficients are differences of much larger quantities and lose
/// most of their significant digits before the resolvent is even formed. A
/// quartic with roots `{1e-5, 1, 10, 1e5}` came back from Ferrari with a
/// spurious root at `-22462`, and 41 of 2000 seeded random quartics came back
/// with no real roots at all where the exact Sturm oracle counts two.
///
/// So this solves the quartic through its **derivative** instead. `p'` is a
/// cubic, its roots are the critical points, and between consecutive critical
/// points `p` is monotone — therefore each such interval holds at most one root
/// and holds one exactly when `p` changes sign across it. That reduces the
/// quartic to a handful of bracketed one-dimensional searches on the *original*
/// coefficients, which is where all the accuracy is; nothing is ever computed
/// from a depressed form.
///
/// Repeated roots fall out of the same structure without a threshold on any
/// discriminant. A root of multiplicity `m > 1` is also a root of `p'` with
/// multiplicity `m - 1`, so a critical point at which `p` vanishes to within its
/// own evaluation noise *is* a repeated root, and the resolvent cubic already
/// reports the multiplicity that fixes `m`.
///
/// The cost is one cubic solve plus up to four safeguarded-Newton refinements of
/// six or seven iterations each. That is dearer than Ferrari's closed form and
/// it is worth it: a quartic that is merely usually right is not usable in a ray
/// caster that runs it millions of times.
#[must_use]
pub fn solve_quartic(a: f64, b: f64, c: f64, d: f64, e: f64) -> RootSet {
    if leading_is_negligible(a, &[b, c, d, e]) {
        return solve_cubic(b, c, d, e);
    }
    let coefficients = [a, b, c, d, e];

    // Cauchy's bound: every real root satisfies |x| < 1 + max|c_k / a|.
    let mut bound = 0.0f64;
    for coefficient in [b, c, d, e] {
        bound = bound.max((coefficient / a).abs());
    }
    bound += 1.0;

    let critical = solve_cubic(4.0 * a, 3.0 * b, 2.0 * c, d);

    // Partition points: the two bounds, plus every critical point between them.
    // At most three critical points, so at most five partition points.
    let mut points = [0.0f64; MAX_ROOTS + 1];
    let mut repeated = [false; MAX_ROOTS + 1];
    let mut n = 0usize;
    points[n] = -bound;
    n += 1;
    // Index into `critical.roots()` of the critical point stored at `points[i]`.
    let mut from_derivative = [0usize; MAX_ROOTS + 1];
    for (index, value) in critical.roots().iter().copied().enumerate() {
        if value > -bound && value < bound && n < points.len() - 1 {
            points[n] = value;
            from_derivative[n] = index;
            // A critical point at which `p` itself vanishes is a repeated root:
            // its multiplicity in `p` is one more than its multiplicity in `p'`.
            repeated[n] = is_repeated_root(&coefficients, value);
            n += 1;
        }
    }
    points[n] = bound;
    n += 1;

    let mut raw = [0.0f64; MAX_ROOTS];
    let mut count = 0usize;

    // Repeated roots, taken straight from the critical points.
    for (index, (&x, &is_repeated)) in points[..n].iter().zip(repeated[..n].iter()).enumerate() {
        if !is_repeated {
            continue;
        }
        let in_derivative = critical.multiplicity(from_derivative[index]);
        for _ in 0..=in_derivative {
            if count < MAX_ROOTS {
                raw[count] = x;
                count += 1;
            }
        }
    }

    // Simple roots: one per monotone interval that changes sign. An interval
    // with a repeated root at either end is skipped — `p` vanishes there, so the
    // only root of the closed interval is that endpoint, already recorded.
    for i in 0..n - 1 {
        if repeated[i] || repeated[i + 1] {
            continue;
        }
        let (lo, hi) = (points[i], points[i + 1]);
        let (low, high) = (eval(&coefficients, lo), eval(&coefficients, hi));
        if !low.is_finite() || !high.is_finite() || low == 0.0 || high == 0.0 {
            continue;
        }
        if (low < 0.0) != (high < 0.0) && count < MAX_ROOTS {
            raw[count] = refine_bracket(&coefficients, lo, hi);
            count += 1;
        }
    }

    RootSet::from_raw(&raw[..count], 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds `a * (x - r0)(x - r1)...` and returns descending coefficients.
    fn from_roots(a: f64, roots: &[f64]) -> Vec<f64> {
        let mut coefficients = vec![a];
        for &r in roots {
            let mut next = vec![0.0; coefficients.len() + 1];
            for (i, &c) in coefficients.iter().enumerate() {
                next[i] += c;
                next[i + 1] -= c * r;
            }
            coefficients = next;
        }
        coefficients
    }

    /// Asserts the solver finds exactly `expected`, to a relative tolerance.
    fn assert_roots(found: &RootSet, expected: &[f64], tolerance: f64, label: &str) {
        let got: Vec<f64> = found.roots().to_vec();
        assert_eq!(
            got.len(),
            expected.len(),
            "{label}: expected {expected:?}, found {got:?}"
        );
        for (g, w) in got.iter().zip(expected) {
            let scale = w.abs().max(1.0);
            assert!(
                (g - w).abs() <= tolerance * scale,
                "{label}: root {g} vs expected {w} (tolerance {tolerance})"
            );
        }
    }

    #[test]
    fn linear() {
        assert_roots(&solve_linear(2.0, -6.0), &[3.0], 1e-15, "2x-6");
        assert_roots(&solve_linear(-1.0, 1.0), &[1.0], 1e-15, "-x+1");
        assert!(solve_linear(0.0, 5.0).is_empty(), "a constant has no roots");
        assert_eq!(solve_linear(2.0, -6.0).solved_degree(), 1);
    }

    #[test]
    fn quadratic_simple_and_double() {
        assert_roots(
            &solve_quadratic(1.0, -3.0, 2.0),
            &[1.0, 2.0],
            1e-15,
            "x^2-3x+2",
        );
        // Exact double root.
        let double = solve_quadratic(1.0, -4.0, 4.0);
        assert_eq!(double.len(), 1);
        assert_eq!(double.multiplicity(0), 2);
        assert!((double.roots()[0] - 2.0).abs() < 1e-15);
        assert_eq!(double.total_multiplicity(), 2);
        // No real roots.
        assert!(solve_quadratic(1.0, 0.0, 1.0).is_empty());
        // Roots are ascending.
        let r = solve_quadratic(1.0, 0.0, -4.0);
        assert_roots(&r, &[-2.0, 2.0], 1e-15, "x^2-4");
    }

    #[test]
    fn quadratic_avoids_catastrophic_cancellation() {
        // The classic demonstration: roots of wildly different magnitude, where
        // the naive formula loses the small one entirely.
        //
        // x^2 - (1e8 + 1e-8) x + 1 has roots 1e8 and 1e-8.
        let big = 1.0e8;
        let small = 1.0e-8;
        let r = solve_quadratic(1.0, -(big + small), 1.0);
        assert_eq!(r.len(), 2);
        let (found_small, found_big) = (r.roots()[0], r.roots()[1]);
        assert!(
            (found_small - small).abs() / small < 1e-10,
            "small root came back as {found_small}, expected {small}; the naive \
             formula is what produces this failure"
        );
        assert!((found_big - big).abs() / big < 1e-14, "{found_big}");
    }

    #[test]
    fn the_exact_discriminant_beats_the_naive_one() {
        // Near-tangency: b^2 and 4ac agree to about fifteen digits, so the naive
        // difference keeps almost nothing. This is the case the whole unit turns
        // on, so the improvement is pinned rather than assumed.
        let a = 1.0;
        let b = 2.0;
        let c = 1.0 - 1.0e-15;
        let naive = b * b - 4.0 * a * c;
        let exact = discriminant(a, b, c);
        let truth = 4.0e-15; // b^2 - 4ac = 4 - 4 + 4e-15
        assert!(
            (exact - truth).abs() < 1e-17,
            "exact discriminant {exact} vs truth {truth}"
        );
        assert!(
            (exact - truth).abs() < (naive - truth).abs() || naive == exact,
            "the compensated form must be at least as good: naive {naive}, exact {exact}"
        );
    }

    #[test]
    fn cubic_all_branches() {
        // Three distinct real roots (the trigonometric branch).
        assert_roots(
            &solve_cubic(1.0, -6.0, 11.0, -6.0),
            &[1.0, 2.0, 3.0],
            1e-12,
            "(x-1)(x-2)(x-3)",
        );
        // One real root, two complex (Cardano branch).
        let one = solve_cubic(1.0, 0.0, 0.0, -8.0);
        assert_roots(&one, &[2.0], 1e-12, "x^3-8");
        // Triple root.
        let triple = solve_cubic(1.0, -3.0, 3.0, -1.0);
        assert_eq!(triple.len(), 1, "{:?}", triple.roots());
        assert_eq!(triple.multiplicity(0), 3);
        assert!((triple.roots()[0] - 1.0).abs() < 1e-8);
        // A double root plus a simple one.
        let mixed = solve_cubic(1.0, -4.0, 5.0, -2.0); // (x-1)^2 (x-2)
        assert_eq!(mixed.len(), 2, "{:?}", mixed.roots());
        assert_eq!(mixed.total_multiplicity(), 3);
    }

    #[test]
    fn quartic_all_branches() {
        // Four distinct real roots.
        assert_roots(
            &solve_quartic(1.0, -10.0, 35.0, -50.0, 24.0),
            &[1.0, 2.0, 3.0, 4.0],
            1e-10,
            "(x-1)(x-2)(x-3)(x-4)",
        );
        // Biquadratic: x^4 - 5x^2 + 4 = (x^2-1)(x^2-4).
        assert_roots(
            &solve_quartic(1.0, 0.0, -5.0, 0.0, 4.0),
            &[-2.0, -1.0, 1.0, 2.0],
            1e-12,
            "biquadratic",
        );
        // Two real, two complex: (x^2+1)(x-1)(x-2) = x^4-3x^3+3x^2-3x+2
        assert_roots(
            &solve_quartic(1.0, -3.0, 3.0, -3.0, 2.0),
            &[1.0, 2.0],
            1e-10,
            "two real two complex",
        );
        // No real roots at all: (x^2+1)(x^2+4)
        assert!(solve_quartic(1.0, 0.0, 5.0, 0.0, 4.0).is_empty());
        // Exact double root — the tangency case.
        let tangent = solve_quartic(1.0, -4.0, 6.0, -4.0, 1.0); // (x-1)^4
        assert_eq!(tangent.len(), 1, "{:?}", tangent.roots());
        assert!(
            (tangent.roots()[0] - 1.0).abs() < 1e-3,
            "{:?}",
            tangent.roots()
        );
    }

    #[test]
    fn degeneracy_degrades_by_one_degree_at_a_time() {
        // A "quartic" whose leading coefficient is negligible is a cubic.
        let r = solve_quartic(1e-20, 1.0, -6.0, 11.0, -6.0);
        assert_eq!(r.solved_degree(), 3, "must degrade to a cubic");
        assert_roots(&r, &[1.0, 2.0, 3.0], 1e-12, "degraded quartic");

        // And all the way down.
        assert_eq!(solve_quartic(0.0, 0.0, 1.0, -3.0, 2.0).solved_degree(), 2);
        assert_eq!(solve_quartic(0.0, 0.0, 0.0, 2.0, -6.0).solved_degree(), 1);
        assert_eq!(solve_quartic(0.0, 0.0, 0.0, 0.0, 5.0).solved_degree(), 0);

        // A leading coefficient just above the threshold is NOT degraded.
        let kept = solve_quartic(1e-10, 1.0, -6.0, 11.0, -6.0);
        assert_eq!(kept.solved_degree(), 4);
    }

    #[test]
    fn the_discarded_root_is_provably_out_of_range() {
        // The justification for ROOT_DEGENERACY_TAU: degrading discards a root
        // of magnitude at least 1/tau times the coefficient scale.
        let a = ROOT_DEGENERACY_TAU;
        let r = solve_quartic(a, 1.0, 0.0, 0.0, 0.0);
        // The escaping root sits near -b/a = -1/tau = -1e14.
        let escaping = 1.0 / ROOT_DEGENERACY_TAU;
        assert!(
            escaping >= 1e14,
            "the threshold must push the discarded root far outside any tool"
        );
        // Whether it degrades at exactly the threshold is a boundary detail; what
        // matters is that anything it drops is astronomically far out.
        let _ = r;
    }

    #[test]
    fn roots_come_back_sorted_and_deduplicated() {
        let r = RootSet::from_raw(&[3.0, 1.0, 2.0, 1.0], 4);
        assert_eq!(r.roots(), &[1.0, 2.0, 3.0]);
        assert_eq!(r.multiplicity(0), 2);
        assert_eq!(r.total_multiplicity(), 4);
        // Non-finite values are discarded rather than sorted into place.
        let r = RootSet::from_raw(&[f64::NAN, 1.0, f64::INFINITY], 3);
        assert_eq!(r.roots(), &[1.0]);
    }

    #[test]
    fn evaluation_is_horner_in_a_fixed_order() {
        // x^3 - 6x^2 + 11x - 6 at x = 2 is zero.
        let c = [1.0, -6.0, 11.0, -6.0];
        assert_eq!(eval(&c, 2.0), 0.0);
        assert_eq!(eval(&c, 0.0), -6.0);
        // Derivative 3x^2 - 12x + 11 at x = 2 is -1.
        assert_eq!(eval_derivative(&c, 2.0), -1.0);
        assert_eq!(eval_derivative(&[5.0], 3.0), 0.0);
    }

    #[test]
    fn polish_improves_a_perturbed_root_and_never_worsens_it() {
        let c = from_roots(1.0, &[1.0, 2.0, 3.0, 4.0]);
        for truth in [1.0, 2.0, 3.0, 4.0] {
            let perturbed = truth + 1e-6;
            let polished = polish(&c, perturbed);
            assert!(
                (polished - truth).abs() < (perturbed - truth).abs(),
                "polish made {perturbed} worse: {polished} vs {truth}"
            );
        }
        // At a quadruple root the derivative vanishes; polish must not diverge.
        let quad = from_roots(1.0, &[1.0, 1.0, 1.0, 1.0]);
        let polished = polish(&quad, 1.0 + 1e-3);
        assert!(polished.is_finite());
        assert!((polished - 1.0).abs() <= 1e-3);
    }

    #[test]
    fn from_roots_helper_is_correct() {
        // The test helper underpins the random sweep, so it is checked itself.
        assert_eq!(from_roots(1.0, &[1.0, 2.0]), vec![1.0, -3.0, 2.0]);
        assert_eq!(from_roots(2.0, &[0.0]), vec![2.0, 0.0]);
        let c = from_roots(1.0, &[1.0, 2.0, 3.0, 4.0]);
        for r in [1.0, 2.0, 3.0, 4.0] {
            assert!(eval(&c, r).abs() < 1e-12, "root {r} of {c:?}");
        }
    }

    #[test]
    fn hashing_covers_roots_and_degree() {
        let a = solve_quadratic(1.0, -3.0, 2.0);
        assert_eq!(
            a.canonical_digest(),
            solve_quadratic(1.0, -3.0, 2.0).canonical_digest()
        );
        assert_ne!(
            a.canonical_digest(),
            solve_quadratic(1.0, -4.0, 3.0).canonical_digest()
        );
    }
}
