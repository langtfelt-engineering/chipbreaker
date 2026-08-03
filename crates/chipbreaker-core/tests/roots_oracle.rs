// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Validation of the fast root solver against an exact Sturm-sequence oracle.
//!
//! Same pattern as Unit 1's predicate validation: a slow, unimpeachable
//! reference, and a fast implementation that must agree with it.
//!
//! # Why this one needs rationals, when the predicate oracle did not
//!
//! Unit 1's oracle avoided rational arithmetic entirely, because every predicate
//! there was a *homogeneous polynomial* in its inputs and could be rescaled into
//! exact integers. That note also warned this would not generalise, and a Sturm
//! chain is exactly the case it warned about: building it requires polynomial
//! **division**, `p[k+1] = -rem(p[k-1], p[k])`, which leaves the integers
//! immediately. So this oracle uses `BigRational`, and it is slow — which is
//! fine, because its job is certainty rather than speed.
//!
//! # What the oracle establishes
//!
//! Sturm's theorem gives the exact number of **distinct** real roots in an
//! interval, as the drop in sign variations across the chain. Applied to the
//! square-free part `p / gcd(p, p')` it is immune to multiplicities, so it
//! answers "how many distinct real roots are there" with certainty and with no
//! floating point anywhere in the decision.
//!
//! # The two sweeps
//!
//! - **Sturm agreement** on the hand-written corpus and a few thousand random
//!   cases. Slow, exact, independent of how the polynomial was built.
//! - **100,000 seeded random cases built from known roots**, where ground truth
//!   is exact *by construction*. The roots are dyadic rationals small enough
//!   that expanding the product is exact in `f64`, so the coefficients the
//!   solver sees represent precisely the polynomial whose roots we know.

use chipbreaker_core::roots::{RootSet, eval, solve_cubic, solve_quadratic, solve_quartic};
use num_bigint::BigInt;
use num_rational::BigRational;
use num_traits::{One, Signed, Zero};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

// ---------------------------------------------------------------------------
// Exact polynomials over the rationals
// ---------------------------------------------------------------------------

/// Coefficients in **ascending** degree: `poly[i]` multiplies `x^i`.
type Poly = Vec<BigRational>;

/// Converts a finite `f64` to an exact rational.
fn exact(v: f64) -> BigRational {
    assert!(v.is_finite(), "the oracle takes finite coefficients only");
    if v == 0.0 {
        return BigRational::zero();
    }
    let bits = v.to_bits();
    let negative = (bits >> 63) == 1;
    let exponent_field = ((bits >> 52) & 0x7ff) as i32;
    let mantissa_field = bits & ((1u64 << 52) - 1);
    let (mantissa, exponent) = if exponent_field == 0 {
        (mantissa_field, -1074)
    } else {
        (mantissa_field | (1u64 << 52), exponent_field - 1075)
    };
    let mut numerator = BigInt::from(mantissa);
    if negative {
        numerator = -numerator;
    }
    if exponent >= 0 {
        BigRational::from(numerator << usize::try_from(exponent).expect("fits"))
    } else {
        let denominator = BigInt::one() << usize::try_from(-exponent).expect("fits");
        BigRational::new(numerator, denominator)
    }
}

/// Descending `f64` coefficients to an ascending exact polynomial.
fn to_poly(descending: &[f64]) -> Poly {
    descending.iter().rev().map(|c| exact(*c)).collect()
}

fn degree(p: &Poly) -> Option<usize> {
    p.iter().rposition(|c| !c.is_zero())
}

fn trim(mut p: Poly) -> Poly {
    while p.len() > 1 && p.last().is_some_and(num_traits::Zero::is_zero) {
        p.pop();
    }
    p
}

fn derivative(p: &Poly) -> Poly {
    if p.len() <= 1 {
        return vec![BigRational::zero()];
    }
    trim(
        p.iter()
            .enumerate()
            .skip(1)
            .map(|(i, c)| c * BigRational::from(BigInt::from(i)))
            .collect(),
    )
}

/// Polynomial remainder `a mod b`, exact over the rationals.
fn remainder(a: &Poly, b: &Poly) -> Poly {
    let Some(db) = degree(b) else {
        panic!("division by the zero polynomial");
    };
    let mut r = a.clone();
    while let Some(dr) = degree(&r) {
        if dr < db {
            break;
        }
        let factor = &r[dr] / &b[db];
        let shift = dr - db;
        for i in 0..=db {
            let term = &factor * &b[i];
            r[i + shift] -= term;
        }
        // The leading term cancels exactly; assign zero so no residue survives
        // a rounding that cannot happen but would be invisible if it did.
        r[dr] = BigRational::zero();
        r = trim(r);
        if degree(&r).is_none() {
            break;
        }
    }
    trim(r)
}

/// Monic greatest common divisor.
fn gcd(a: &Poly, b: &Poly) -> Poly {
    let mut x = trim(a.clone());
    let mut y = trim(b.clone());
    while degree(&y).is_some() && !(degree(&y) == Some(0) && y[0].is_zero()) {
        let r = remainder(&x, &y);
        x = y;
        y = r;
        if degree(&y) == Some(0) && !y[0].is_zero() {
            // A non-zero constant: the polynomials are coprime.
            return vec![BigRational::one()];
        }
    }
    // Normalise to monic so the result is canonical.
    let d = degree(&x).unwrap_or(0);
    let lead = x[d].clone();
    if lead.is_zero() {
        return vec![BigRational::one()];
    }
    x.iter().map(|c| c / &lead).collect()
}

/// `p` with every repeated factor reduced to a single one.
fn square_free(p: &Poly) -> Poly {
    let d = derivative(p);
    if degree(&d).is_none() || (degree(&d) == Some(0) && d[0].is_zero()) {
        return trim(p.clone());
    }
    let g = gcd(p, &d);
    if degree(&g) == Some(0) {
        return trim(p.clone());
    }
    // Exact division p / g, which is exact because g divides p.
    let mut quotient = vec![BigRational::zero(); degree(p).unwrap_or(0) + 1];
    let mut r = trim(p.clone());
    let dg = degree(&g).expect("non-constant");
    while let Some(dr) = degree(&r) {
        if dr < dg {
            break;
        }
        let factor = &r[dr] / &g[dg];
        let shift = dr - dg;
        quotient[shift] = factor.clone();
        for i in 0..=dg {
            let term = &factor * &g[i];
            r[i + shift] -= term;
        }
        r[dr] = BigRational::zero();
        r = trim(r);
    }
    trim(quotient)
}

fn evaluate(p: &Poly, x: &BigRational) -> BigRational {
    let mut acc = BigRational::zero();
    for c in p.iter().rev() {
        acc = acc * x + c;
    }
    acc
}

/// The Sturm chain of a square-free polynomial.
fn sturm_chain(p: &Poly) -> Vec<Poly> {
    let mut chain = vec![trim(p.clone()), derivative(p)];
    loop {
        let n = chain.len();
        if degree(&chain[n - 1]).is_none_or(|d| d == 0) {
            break;
        }
        let r = remainder(&chain[n - 2], &chain[n - 1]);
        if degree(&r).is_none() {
            break;
        }
        let negated: Poly = r.iter().map(|c| -c).collect();
        if degree(&negated) == Some(0) && negated[0].is_zero() {
            break;
        }
        chain.push(trim(negated));
    }
    chain
}

fn sign_variations(chain: &[Poly], x: &BigRational) -> usize {
    let mut previous = 0i32;
    let mut count = 0usize;
    for p in chain {
        let v = evaluate(p, x);
        let s = if v.is_positive() {
            1
        } else if v.is_negative() {
            -1
        } else {
            0
        };
        if s != 0 {
            if previous != 0 && s != previous {
                count += 1;
            }
            previous = s;
        }
    }
    count
}

/// A Cauchy bound: every real root lies strictly inside `(-b, b)`.
fn cauchy_bound(p: &Poly) -> BigRational {
    let d = degree(p).unwrap_or(0);
    if d == 0 {
        return BigRational::one();
    }
    let lead = p[d].clone().abs();
    let mut max = BigRational::zero();
    for c in &p[..d] {
        let ratio = c.clone().abs() / &lead;
        if ratio > max {
            max = ratio;
        }
    }
    max + BigRational::one()
}

/// The exact number of **distinct** real roots of `descending`.
fn distinct_real_roots(descending: &[f64]) -> usize {
    let p = trim(to_poly(descending));
    match degree(&p) {
        None | Some(0) => 0,
        Some(_) => {
            let free = square_free(&p);
            if degree(&free).is_none_or(|d| d == 0) {
                return 0;
            }
            let chain = sturm_chain(&free);
            let bound = cauchy_bound(&free);
            let low = -bound.clone();
            sign_variations(&chain, &low).saturating_sub(sign_variations(&chain, &bound))
        }
    }
}

// ---------------------------------------------------------------------------
// The oracle validates itself first
// ---------------------------------------------------------------------------

#[test]
fn oracle_counts_known_polynomials_correctly() {
    // (x-1)(x-2)(x-3)
    assert_eq!(distinct_real_roots(&[1.0, -6.0, 11.0, -6.0]), 3);
    // x^2 + 1: none
    assert_eq!(distinct_real_roots(&[1.0, 0.0, 1.0]), 0);
    // (x-1)^2: one distinct
    assert_eq!(distinct_real_roots(&[1.0, -2.0, 1.0]), 1);
    // (x-1)^3: still one distinct
    assert_eq!(distinct_real_roots(&[1.0, -3.0, 3.0, -1.0]), 1);
    // (x-1)^2 (x-2): two distinct
    assert_eq!(distinct_real_roots(&[1.0, -4.0, 5.0, -2.0]), 2);
    // (x^2+1)(x-1)(x-2): two distinct
    assert_eq!(distinct_real_roots(&[1.0, -3.0, 3.0, -3.0, 2.0]), 2);
    // (x-1)(x-2)(x-3)(x-4)
    assert_eq!(distinct_real_roots(&[1.0, -10.0, 35.0, -50.0, 24.0]), 4);
    // (x^2+1)(x^2+4): none
    assert_eq!(distinct_real_roots(&[1.0, 0.0, 5.0, 0.0, 4.0]), 0);
    // Linear and constant.
    assert_eq!(distinct_real_roots(&[2.0, -6.0]), 1);
    assert_eq!(distinct_real_roots(&[5.0]), 0);
}

#[test]
fn oracle_helpers_behave() {
    let p = to_poly(&[1.0, -3.0, 2.0]); // x^2 - 3x + 2, ascending [2, -3, 1]
    assert_eq!(degree(&p), Some(2));
    let d = derivative(&p); // 2x - 3
    assert_eq!(degree(&d), Some(1));
    assert_eq!(
        evaluate(&p, &BigRational::from(BigInt::from(1))),
        BigRational::zero()
    );
    assert_eq!(
        evaluate(&p, &BigRational::from(BigInt::from(0))),
        exact(2.0)
    );
    // (x-1)^2 has square-free part (x-1).
    let sq = square_free(&to_poly(&[1.0, -2.0, 1.0]));
    assert_eq!(degree(&sq), Some(1));
    // Exact conversion round-trips awkward values.
    for v in [0.1, -1e-17, 1e300, 5e-324, core::f64::consts::PI] {
        let r = exact(v);
        assert_eq!(r.is_negative(), v < 0.0, "{v}");
    }
}

// ---------------------------------------------------------------------------
// Corpus
// ---------------------------------------------------------------------------

/// Builds descending coefficients of `a * prod(x - r)`.
fn from_roots(a: f64, roots: &[f64]) -> Vec<f64> {
    let mut c = vec![a];
    for &r in roots {
        let mut next = vec![0.0; c.len() + 1];
        for (i, &v) in c.iter().enumerate() {
            next[i] += v;
            next[i + 1] -= v * r;
        }
        c = next;
    }
    c
}

fn solve(descending: &[f64]) -> RootSet {
    match descending.len() {
        3 => solve_quadratic(descending[0], descending[1], descending[2]),
        4 => solve_cubic(descending[0], descending[1], descending[2], descending[3]),
        5 => solve_quartic(
            descending[0],
            descending[1],
            descending[2],
            descending[3],
            descending[4],
        ),
        n => panic!("unsupported degree with {n} coefficients"),
    }
}

/// The hand-written hard cases. Each names the difficulty it exists to exercise.
fn corpus() -> Vec<(&'static str, Vec<f64>)> {
    vec![
        ("quadratic/simple", from_roots(1.0, &[1.0, 2.0])),
        ("quadratic/double", from_roots(1.0, &[3.0, 3.0])),
        ("quadratic/no-real", vec![1.0, 0.0, 1.0]),
        // Roots spanning sixteen orders of magnitude: the case the naive
        // quadratic formula destroys.
        (
            "quadratic/wide-magnitude",
            from_roots(1.0, &[1.0e-8, 1.0e8]),
        ),
        ("quadratic/tiny-roots", from_roots(1.0, &[1.0e-30, 2.0e-30])),
        ("cubic/three-real", from_roots(1.0, &[-5.0, 1.0, 7.0])),
        ("cubic/one-real", vec![1.0, 0.0, 0.0, -8.0]),
        ("cubic/triple", from_roots(1.0, &[2.0, 2.0, 2.0])),
        (
            "cubic/double-plus-simple",
            from_roots(1.0, &[1.0, 1.0, 4.0]),
        ),
        (
            "cubic/wide-magnitude",
            from_roots(1.0, &[1.0e-6, 1.0, 1.0e6]),
        ),
        (
            "quartic/four-real",
            from_roots(1.0, &[-3.0, -1.0, 2.0, 5.0]),
        ),
        (
            "quartic/two-real-two-complex",
            vec![1.0, -3.0, 3.0, -3.0, 2.0],
        ),
        ("quartic/no-real", vec![1.0, 0.0, 5.0, 0.0, 4.0]),
        ("quartic/biquadratic", vec![1.0, 0.0, -5.0, 0.0, 4.0]),
        (
            "quartic/double-double",
            from_roots(1.0, &[1.0, 1.0, 3.0, 3.0]),
        ),
        ("quartic/quadruple", from_roots(1.0, &[2.0, 2.0, 2.0, 2.0])),
        (
            "quartic/triple-plus-simple",
            from_roots(1.0, &[1.0, 1.0, 1.0, 5.0]),
        ),
        // Tangency: the physically important case. Two roots a hair apart.
        (
            "quartic/near-double",
            from_roots(1.0, &[1.0, 1.0 + 1.0e-7, 4.0, 9.0]),
        ),
        (
            "quartic/wide-magnitude",
            from_roots(1.0, &[1.0e-5, 1.0, 10.0, 1.0e5]),
        ),
        // A large leading coefficient with small roots, and the reverse.
        (
            "quartic/large-lead",
            from_roots(1.0e6, &[0.5, 1.5, 2.5, 3.5]),
        ),
        (
            "quartic/small-lead",
            from_roots(1.0e-6, &[0.5, 1.5, 2.5, 3.5]),
        ),
        (
            "quartic/negative-lead",
            from_roots(-2.0, &[-1.0, 0.0, 1.0, 2.0]),
        ),
        (
            "quartic/root-at-zero",
            from_roots(1.0, &[0.0, 1.0, 2.0, 3.0]),
        ),
    ]
}

#[test]
fn the_solver_agrees_with_the_sturm_oracle_on_the_corpus() {
    let mut disagreements = Vec::new();
    for (name, coefficients) in corpus() {
        let expected = distinct_real_roots(&coefficients);
        let found = solve(&coefficients);
        if found.len() != expected {
            disagreements.push(format!(
                "{name}: Sturm says {expected} distinct real roots, solver found {} ({:?})",
                found.len(),
                found.roots()
            ));
        }
        // Every reported root must actually be one: a small residual relative to
        // the polynomial's scale at that point.
        let scale = coefficients.iter().fold(0.0f64, |m, c| m.max(c.abs()));
        for r in found.roots() {
            let residual = eval(&coefficients, *r).abs();
            let magnitude = r.abs().max(1.0);
            let tolerance = 1e-6 * scale * magnitude.powi(coefficients.len() as i32 - 1);
            assert!(
                residual <= tolerance,
                "{name}: root {r} has residual {residual}, tolerance {tolerance}"
            );
        }
        // Total multiplicity can never exceed the degree.
        assert!(
            found.total_multiplicity() < coefficients.len(),
            "{name}: multiplicity {} exceeds degree",
            found.total_multiplicity()
        );
    }
    assert!(
        disagreements.is_empty(),
        "solver disagreed with the exact oracle:\n  {}",
        disagreements.join("\n  ")
    );
}

// ---------------------------------------------------------------------------
// Seeded sweeps
// ---------------------------------------------------------------------------

/// Documented and fixed, so a CI failure reproduces on the first attempt.
const SWEEP_SEED: u64 = 0x0000_C41B_0000_0200;

/// Total cases in the by-construction sweep.
const SWEEP_CASES: usize = 100_000;

/// Cases additionally checked against Sturm. Far fewer, because `BigRational`
/// polynomial division is orders of magnitude slower than the solver.
const STURM_CASES: usize = 2_000;

/// A root drawn from a dyadic grid small enough that expanding the product of
/// four of them is **exact** in `f64`.
///
/// Roots are `k/4` for `|k| <= 32`, so the expanded coefficients have
/// denominator at most `256` and numerator at most `32^4`, both far inside the
/// 53-bit mantissa. That is what makes ground truth exact by construction rather
/// than merely close.
fn dyadic_root(rng: &mut StdRng) -> f64 {
    f64::from(rng.random_range(-32i32..=32)) / 4.0
}

#[test]
fn a_hundred_thousand_polynomials_built_from_known_roots() {
    let mut rng = StdRng::seed_from_u64(SWEEP_SEED);
    let mut worst_error = 0.0f64;
    let mut worst_case = String::new();
    let mut checked = 0usize;

    for i in 0..SWEEP_CASES {
        let degree = 2 + (i % 3); // 2, 3, 4 in rotation
        let mut roots: Vec<f64> = (0..degree).map(|_| dyadic_root(&mut rng)).collect();
        roots.sort_by(f64::total_cmp);
        let lead = if rng.random_bool(0.5) { 1.0 } else { -2.0 };
        let coefficients = from_roots(lead, &roots);

        // Distinct roots, in ascending order, as ground truth.
        let mut distinct = roots.clone();
        distinct.dedup();

        let found = solve(&coefficients);
        assert_eq!(
            found.len(),
            distinct.len(),
            "case {i}: roots {roots:?} gave {:?}",
            found.roots()
        );
        assert_eq!(
            found.total_multiplicity(),
            roots.len(),
            "case {i}: multiplicities lost for {roots:?}"
        );
        for (got, want) in found.roots().iter().zip(&distinct) {
            let scale = want.abs().max(1.0);
            let error = (got - want).abs() / scale;
            if error > worst_error {
                worst_error = error;
                worst_case = format!("roots {roots:?} -> {:?}", found.roots());
            }
        }
        checked += 1;
    }

    assert_eq!(checked, SWEEP_CASES);
    // A multiple root is limited to sqrt(eps) accuracy; that is the floor, and
    // the bound is set just above it rather than at full precision.
    assert!(
        worst_error < 1.0e-6,
        "worst relative root error {worst_error:e} in {worst_case}"
    );
    eprintln!("{SWEEP_CASES} by-construction cases, worst relative error {worst_error:e}");
}

#[test]
fn a_seeded_sweep_agrees_with_the_sturm_oracle() {
    // Independent of how the polynomial was built: arbitrary coefficients, and
    // the oracle decides the truth.
    let mut rng = StdRng::seed_from_u64(SWEEP_SEED ^ 0xFFFF);
    let mut disagreements = 0usize;
    let mut with_real_roots = 0usize;

    for i in 0..STURM_CASES {
        let n = 3 + (i % 3); // 3, 4, 5 coefficients
        let coefficients: Vec<f64> = (0..n)
            .map(|_| {
                // Ragged mantissas across a few orders of magnitude, so the
                // cases are not all comfortably conditioned.
                let mantissa: f64 = rng.random_range(-1.0..1.0);
                let scale: i32 = rng.random_range(-6i32..=6);
                mantissa * 2.0f64.powi(scale)
            })
            .collect();
        if coefficients[0].abs() < 1e-3 {
            continue; // leave degeneracy to its own test
        }
        let expected = distinct_real_roots(&coefficients);
        let found = solve(&coefficients);
        if found.len() != expected {
            disagreements += 1;
            if disagreements <= 3 {
                eprintln!(
                    "case {i}: coefficients {coefficients:?}\n  Sturm {expected}, solver {} ({:?})",
                    found.len(),
                    found.roots()
                );
            }
        }
        if expected > 0 {
            with_real_roots += 1;
        }
    }

    assert!(
        with_real_roots > STURM_CASES / 10,
        "only {with_real_roots} cases had real roots; the generator is not exercising much"
    );
    assert_eq!(
        disagreements, 0,
        "{disagreements} disagreements with the exact oracle"
    );
    eprintln!(
        "{STURM_CASES} Sturm-verified cases, {with_real_roots} with real roots, 0 disagreements"
    );
}
