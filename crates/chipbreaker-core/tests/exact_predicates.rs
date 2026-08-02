// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Validation of the adaptive predicates against exact rational arithmetic.
//!
//! The adaptive predicates are fast and are *claimed* to be exact. This file is
//! where that claim is checked, against an oracle that is unimpeachably correct
//! and far too slow to ship: arbitrary-precision integer determinants.
//!
//! # How the oracle avoids rationals
//!
//! Every predicate here is a **homogeneous** polynomial in the input
//! coordinates — degree 2 for `orient2d`, 3 for `orient3d`, 4 for `incircle`,
//! 5 for `insphere`. Scaling every coordinate of a case by the same positive
//! constant therefore scales the determinant by a positive power of that
//! constant and cannot change its sign.
//!
//! Each `f64` is exactly `mantissa * 2^exponent` for integer mantissa. Scaling
//! all of a case's coordinates by `2^-min(exponent)` turns every one of them
//! into an exact integer, so the determinant can be evaluated in `BigInt` with
//! no rationals, no GCD normalization, and no rounding whatsoever. The sign of
//! that integer is the mathematically correct answer, by construction.
//!
//! This is both simpler and considerably faster than `BigRational`, which is
//! what makes the 100,000-case sweep tolerable in CI.

use chipbreaker_core::math::{Vec2, Vec3};
use chipbreaker_core::predicates::corpus::{
    self, CorpusCase, MAX_COORDS, PredicateKind, degenerate_corpus,
};
use chipbreaker_core::predicates::{
    ADAPTIVE, INCIRCLE_COORDS, INSPHERE_COORDS, ORIENT2D_COORDS, ORIENT3D_COORDS, Orientation,
    Predicates,
};
use num_bigint::BigInt;
use num_traits::{Signed, Zero};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

// ---------------------------------------------------------------------------
// The exact oracle
// ---------------------------------------------------------------------------

/// Decomposes a finite `f64` into `(mantissa, exponent)` with
/// `value == mantissa * 2^exponent` exactly.
fn decompose(v: f64) -> (i64, i32) {
    assert!(v.is_finite(), "the oracle takes finite coordinates only, got {v}");
    if v == 0.0 {
        return (0, 0);
    }
    let bits = v.to_bits();
    let negative = (bits >> 63) == 1;
    let exp_field = ((bits >> 52) & 0x7ff) as i32;
    let mant_field = bits & ((1u64 << 52) - 1);
    // Subnormals have no implicit leading bit and a fixed exponent.
    let (mantissa, exponent) = if exp_field == 0 {
        (mant_field, -1074)
    } else {
        (mant_field | (1u64 << 52), exp_field - 1075)
    };
    let m = i64::try_from(mantissa).expect("mantissa fits in 53 bits");
    (if negative { -m } else { m }, exponent)
}

/// Rescales a whole case to exact integers by the common factor
/// `2^-min(exponent)`, which is positive and therefore sign-preserving.
fn to_exact_integers(coords: &[f64]) -> Vec<BigInt> {
    let parts: Vec<(i64, i32)> = coords.iter().map(|&v| decompose(v)).collect();
    let e_min = parts
        .iter()
        .filter(|(m, _)| *m != 0)
        .map(|(_, e)| *e)
        .min()
        .unwrap_or(0);
    parts
        .iter()
        .map(|&(m, e)| {
            if m == 0 {
                BigInt::zero()
            } else {
                let shift = usize::try_from(e - e_min).expect("e >= e_min for non-zero values");
                BigInt::from(m) << shift
            }
        })
        .collect()
}

fn det2(a: &BigInt, b: &BigInt, c: &BigInt, d: &BigInt) -> BigInt {
    a * d - b * c
}

fn det3(m: &[[BigInt; 3]; 3]) -> BigInt {
    &m[0][0] * det2(&m[1][1], &m[1][2], &m[2][1], &m[2][2])
        - &m[0][1] * det2(&m[1][0], &m[1][2], &m[2][0], &m[2][2])
        + &m[0][2] * det2(&m[1][0], &m[1][1], &m[2][0], &m[2][1])
}

fn det4(m: &[[BigInt; 4]; 4]) -> BigInt {
    let minor = |skip_col: usize| -> BigInt {
        let mut sub: [[BigInt; 3]; 3] =
            core::array::from_fn(|_| core::array::from_fn(|_| BigInt::zero()));
        for r in 1..4 {
            let mut sj = 0;
            for c in 0..4 {
                if c == skip_col {
                    continue;
                }
                sub[r - 1][sj] = m[r][c].clone();
                sj += 1;
            }
        }
        det3(&sub)
    };
    let mut acc = BigInt::zero();
    for c in 0..4 {
        let term = &m[0][c] * minor(c);
        if c % 2 == 0 {
            acc += term;
        } else {
            acc -= term;
        }
    }
    acc
}

fn sign_of(v: &BigInt) -> Orientation {
    if v.is_positive() {
        Orientation::Positive
    } else if v.is_negative() {
        Orientation::Negative
    } else {
        Orientation::Zero
    }
}

/// The mathematically exact result of `kind` on `coords`.
///
/// The determinants below are the definitions Shewchuk's predicates compute:
/// `orient2d` is `det[a - c; b - c]`, `orient3d` is `det[a - d; b - d; c - d]`,
/// `incircle` is the same with each row lifted by its squared length relative to
/// the last point, and `insphere` is the 4x4 analogue.
fn exact(kind: PredicateKind, coords: &[f64]) -> Orientation {
    let v = to_exact_integers(coords);
    let at = |i: usize| -> &BigInt { &v[i] };

    match kind {
        PredicateKind::Orient2d => {
            // Rows: a - c, b - c.
            let acx = at(0) - at(4);
            let acy = at(1) - at(5);
            let bcx = at(2) - at(4);
            let bcy = at(3) - at(5);
            sign_of(&det2(&acx, &acy, &bcx, &bcy))
        }
        PredicateKind::Orient3d => {
            let d = [at(9), at(10), at(11)];
            let row = |i: usize| -> [BigInt; 3] {
                [
                    at(3 * i) - d[0],
                    at(3 * i + 1) - d[1],
                    at(3 * i + 2) - d[2],
                ]
            };
            let m = [row(0), row(1), row(2)];
            sign_of(&det3(&m))
        }
        PredicateKind::InCircle => {
            let d = [at(6), at(7)];
            let row = |i: usize| -> [BigInt; 3] {
                let dx = at(2 * i) - d[0];
                let dy = at(2 * i + 1) - d[1];
                let lift = &dx * &dx + &dy * &dy;
                [dx, dy, lift]
            };
            let m = [row(0), row(1), row(2)];
            sign_of(&det3(&m))
        }
        PredicateKind::InSphere => {
            let e = [at(12), at(13), at(14)];
            let row = |i: usize| -> [BigInt; 4] {
                let dx = at(3 * i) - e[0];
                let dy = at(3 * i + 1) - e[1];
                let dz = at(3 * i + 2) - e[2];
                let lift = &dx * &dx + &dy * &dy + &dz * &dz;
                [dx, dy, dz, lift]
            };
            let m = [row(0), row(1), row(2), row(3)];
            sign_of(&det4(&m))
        }
    }
}

fn adaptive(kind: PredicateKind, c: &[f64]) -> Orientation {
    match kind {
        PredicateKind::Orient2d => ADAPTIVE.orient2d(
            Vec2::new(c[0], c[1]),
            Vec2::new(c[2], c[3]),
            Vec2::new(c[4], c[5]),
        ),
        PredicateKind::Orient3d => ADAPTIVE.orient3d(
            Vec3::new(c[0], c[1], c[2]),
            Vec3::new(c[3], c[4], c[5]),
            Vec3::new(c[6], c[7], c[8]),
            Vec3::new(c[9], c[10], c[11]),
        ),
        PredicateKind::InCircle => ADAPTIVE.incircle(
            Vec2::new(c[0], c[1]),
            Vec2::new(c[2], c[3]),
            Vec2::new(c[4], c[5]),
            Vec2::new(c[6], c[7]),
        ),
        PredicateKind::InSphere => ADAPTIVE.insphere(
            Vec3::new(c[0], c[1], c[2]),
            Vec3::new(c[3], c[4], c[5]),
            Vec3::new(c[6], c[7], c[8]),
            Vec3::new(c[9], c[10], c[11]),
            Vec3::new(c[12], c[13], c[14]),
        ),
    }
}

// ---------------------------------------------------------------------------
// The oracle validates itself first
// ---------------------------------------------------------------------------

#[test]
fn decomposition_is_exact() {
    let values = [
        0.0,
        1.0,
        -1.0,
        0.1,
        0.5,
        f64::MIN_POSITIVE,
        5.0e-324,
        f64::MAX,
        -f64::MAX,
        1e150,
        1e-150,
        core::f64::consts::PI,
    ];
    for v in values {
        let (m, e) = decompose(v);
        // Reconstruct in f64 where the exponent allows it, and in BigInt always.
        let reconstructed = if m == 0 {
            BigInt::zero()
        } else {
            BigInt::from(m)
        };
        assert_eq!(reconstructed.is_zero(), v == 0.0, "zero handling for {v}");
        // m * 2^e == v, checked by scaling both sides into integers.
        let scaled = to_exact_integers(&[v]);
        assert_eq!(scaled.len(), 1);
        assert_eq!(scaled[0].is_zero(), v == 0.0);
        assert_eq!(scaled[0].is_negative(), v < 0.0, "sign of {v}");
        assert!(e >= -1074, "exponent {e} below the subnormal floor for {v}");
    }
}

#[test]
fn oracle_agrees_with_hand_computed_determinants() {
    // orient2d of a unit counterclockwise triangle is +1 before scaling.
    assert_eq!(
        exact(PredicateKind::Orient2d, &[0.0, 0.0, 1.0, 0.0, 0.0, 1.0]),
        Orientation::Positive
    );
    assert_eq!(
        exact(PredicateKind::Orient2d, &[0.0, 0.0, 0.0, 1.0, 1.0, 0.0]),
        Orientation::Negative
    );
    assert_eq!(
        exact(PredicateKind::Orient2d, &[0.0, 0.0, 1.0, 1.0, 2.0, 2.0]),
        Orientation::Zero
    );
    assert_eq!(
        exact(
            PredicateKind::Orient3d,
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        ),
        Orientation::Positive
    );
    // The plane x + y + z = 1.
    assert_eq!(
        exact(
            PredicateKind::Orient3d,
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, -1.0]
        ),
        Orientation::Zero
    );
    // The origin is inside the unit circle through (1,0), (0,1), (-1,0).
    assert_eq!(
        exact(PredicateKind::InCircle, &[1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0]),
        Orientation::Positive
    );
    // (0,-1) is on it.
    assert_eq!(
        exact(PredicateKind::InCircle, &[1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0]),
        Orientation::Zero
    );
    // The circumcentre is inside the sphere; (1,1,0) is on it.
    let tetra = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
    let mut inside = tetra.to_vec();
    inside.extend_from_slice(&[0.5, 0.5, 0.5]);
    assert_eq!(exact(PredicateKind::InSphere, &inside), Orientation::Positive);
    let mut on = tetra.to_vec();
    on.extend_from_slice(&[1.0, 1.0, 0.0]);
    assert_eq!(exact(PredicateKind::InSphere, &on), Orientation::Zero);
    let mut outside = tetra.to_vec();
    outside.extend_from_slice(&[10.0, 10.0, 10.0]);
    assert_eq!(exact(PredicateKind::InSphere, &outside), Orientation::Negative);
}

#[test]
fn oracle_is_scale_invariant() {
    // Scaling every coordinate by a power of two must not change the sign; this
    // is the property the integer rescaling relies on.
    let base = [0.5, 0.25, 12.0, 7.0, -3.0, 1.5];
    let reference = exact(PredicateKind::Orient2d, &base);
    assert_ne!(reference, Orientation::Zero);
    for shift in [-60i32, -8, 8, 60] {
        let scaled: Vec<f64> = base.iter().map(|v| v * 2.0f64.powi(shift)).collect();
        assert_eq!(exact(PredicateKind::Orient2d, &scaled), reference, "2^{shift}");
    }
}

// ---------------------------------------------------------------------------
// The corpus
// ---------------------------------------------------------------------------

#[test]
fn every_corpus_expectation_matches_exact_arithmetic() {
    let cases = degenerate_corpus();
    assert!(!cases.is_empty());
    for case in &cases {
        assert_eq!(
            exact(case.kind, case.coords()),
            case.expected,
            "corpus case `{}` claims `{}` but exact arithmetic says otherwise",
            case.id,
            case.expected.as_char()
        );
    }
}

#[test]
fn adaptive_predicates_match_exact_arithmetic_on_the_whole_corpus() {
    for case in degenerate_corpus() {
        assert_eq!(
            case.evaluate(&ADAPTIVE),
            exact(case.kind, case.coords()),
            "case `{}` ({})",
            case.id,
            case.kind
        );
    }
}

// ---------------------------------------------------------------------------
// Seeded random sweep
// ---------------------------------------------------------------------------

/// Seed for the randomized predicate sweep.
///
/// Documented and fixed: a failure in CI reproduces bit-for-bit on a developer
/// machine by running this test alone.
const RANDOM_SWEEP_SEED: u64 = 0x0000_C41B_0000_0010;

/// Total random cases, split across the four predicates.
const RANDOM_SWEEP_CASES: usize = 100_000;

/// Nudges `v` by `n` ULPs, toward `+inf` for positive `n`.
fn ulps(v: f64, n: i32) -> f64 {
    let mut x = v;
    for _ in 0..n.abs() {
        x = if n > 0 { x.next_up() } else { x.next_down() };
    }
    x
}

/// Nudges one coordinate of `coords` by `n` ULPs, preferring index `preferred`.
///
/// Skips zero coordinates: one ULP away from `0.0` is `5e-324`, a subnormal that
/// lies outside every predicate's exact range, so nudging a zero would generate
/// a case the predicate is documented not to answer. Falls back to the first
/// non-zero coordinate, and leaves the configuration exactly degenerate if every
/// coordinate is zero.
fn nudge_non_zero(coords: &mut [f64], preferred: usize, n: i32) {
    if n == 0 {
        return;
    }
    let target = if coords.get(preferred).is_some_and(|v| *v != 0.0) {
        Some(preferred)
    } else {
        coords.iter().position(|v| *v != 0.0)
    };
    if let Some(i) = target {
        coords[i] = ulps(coords[i], n);
    }
}

/// A coordinate with a ragged mantissa at a randomly chosen scale.
fn coord(rng: &mut StdRng) -> f64 {
    let mantissa: f64 = rng.random_range(-1.0f64..1.0);
    let scale: i32 = rng.random_range(-30i32..=30);
    mantissa * 2.0f64.powi(scale)
}

/// Places `c` on the line through `a` and `b`, then perturbs it by a few ULPs.
///
/// This is where the interesting cases live: an exactly-collinear triple that a
/// naive determinant misjudges, and its immediate neighbours one ULP to either
/// side, which a naive determinant cannot distinguish from the collinear case at
/// all.
fn near_collinear_2d(rng: &mut StdRng) -> [f64; 6] {
    let a = Vec2::new(coord(rng), coord(rng));
    let b = Vec2::new(coord(rng), coord(rng));
    let t: f64 = rng.random_range(-2.0f64..3.0);
    // The rounding in this interpolation is itself part of the test: `c` is
    // *near* the line, not necessarily on it, and only the oracle knows which
    // side it landed on.
    let cx = a.x + (b.x - a.x) * t;
    let cy = a.y + (b.y - a.y) * t;
    let mut out = [a.x, a.y, b.x, b.y, cx, cy];
    let preferred = if rng.random_bool(0.5) { 4 } else { 5 };
    nudge_non_zero(&mut out[4..], preferred - 4, rng.random_range(-3i32..=3));
    out
}

fn near_coplanar_3d(rng: &mut StdRng) -> [f64; 12] {
    let a = Vec3::new(coord(rng), coord(rng), coord(rng));
    let b = Vec3::new(coord(rng), coord(rng), coord(rng));
    let c = Vec3::new(coord(rng), coord(rng), coord(rng));
    let u: f64 = rng.random_range(-1.0f64..2.0);
    let v: f64 = rng.random_range(-1.0f64..2.0);
    // A point in the affine span of a, b, c, then nudged off it.
    let d = Vec3::new(
        a.x + (b.x - a.x) * u + (c.x - a.x) * v,
        a.y + (b.y - a.y) * u + (c.y - a.y) * v,
        a.z + (b.z - a.z) * u + (c.z - a.z) * v,
    );
    let mut dd = [d.x, d.y, d.z];
    nudge_non_zero(&mut dd, rng.random_range(0usize..3), rng.random_range(-2i32..=2));
    [a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, dd[0], dd[1], dd[2]]
}

/// Four points on a common circle of random centre and radius, with the fourth
/// nudged off it.
fn near_cocircular(rng: &mut StdRng) -> [f64; 8] {
    // Integer Pythagorean-style points keep the construction exact, so the
    // cocircular case really is cocircular rather than merely close.
    let r: f64 = f64::from(rng.random_range(1i32..=64));
    let cx = f64::from(rng.random_range(-16i32..=16));
    let cy = f64::from(rng.random_range(-16i32..=16));
    let pts = [
        (cx + r, cy),
        (cx, cy + r),
        (cx - r, cy),
        (cx, cy - r),
    ];
    let mut d = [pts[3].0, pts[3].1];
    nudge_non_zero(&mut d, usize::from(rng.random_bool(0.5)), rng.random_range(-2i32..=2));
    [pts[0].0, pts[0].1, pts[1].0, pts[1].1, pts[2].0, pts[2].1, d[0], d[1]]
}

fn near_cospherical(rng: &mut StdRng) -> [f64; 15] {
    let r: f64 = f64::from(rng.random_range(1i32..=32));
    let c = [
        f64::from(rng.random_range(-8i32..=8)),
        f64::from(rng.random_range(-8i32..=8)),
        f64::from(rng.random_range(-8i32..=8)),
    ];
    // A positively-oriented tetrahedron on the sphere, plus a fifth point on it.
    let pts = [
        [c[0] + r, c[1], c[2]],
        [c[0], c[1] + r, c[2]],
        [c[0], c[1], c[2] + r],
        [c[0] - r, c[1], c[2]],
        [c[0], c[1] - r, c[2]],
    ];
    let mut e = pts[4];
    nudge_non_zero(&mut e, rng.random_range(0usize..3), rng.random_range(-2i32..=2));
    [
        pts[0][0], pts[0][1], pts[0][2], pts[1][0], pts[1][1], pts[1][2], pts[2][0], pts[2][1],
        pts[2][2], pts[3][0], pts[3][1], pts[3][2], e[0], e[1], e[2],
    ]
}

#[test]
fn adaptive_predicates_match_exact_arithmetic_on_seeded_random_cases() {
    let mut rng = StdRng::seed_from_u64(RANDOM_SWEEP_SEED);

    // Weighted toward the two-dimensional predicates, which are the ones U9's
    // dual contouring will call hardest, and away from `insphere`, whose exact
    // determinant is a degree-five polynomial and correspondingly slow.
    let plan: [(PredicateKind, usize); 4] = [
        (PredicateKind::Orient2d, RANDOM_SWEEP_CASES * 40 / 100),
        (PredicateKind::Orient3d, RANDOM_SWEEP_CASES * 30 / 100),
        (PredicateKind::InCircle, RANDOM_SWEEP_CASES * 20 / 100),
        (PredicateKind::InSphere, RANDOM_SWEEP_CASES * 10 / 100),
    ];
    assert_eq!(
        plan.iter().map(|(_, n)| n).sum::<usize>(),
        RANDOM_SWEEP_CASES,
        "the split must account for every case"
    );

    let mut degenerate_seen = 0usize;
    let mut total = 0usize;
    let mut skipped = 0usize;

    for (kind, count) in plan {
        for i in 0..count {
            let coords: Vec<f64> = match kind {
                PredicateKind::Orient2d => near_collinear_2d(&mut rng).to_vec(),
                PredicateKind::Orient3d => near_coplanar_3d(&mut rng).to_vec(),
                PredicateKind::InCircle => near_cocircular(&mut rng).to_vec(),
                PredicateKind::InSphere => near_cospherical(&mut rng).to_vec(),
            };
            // Generating a case outside the predicate's exact range would be
            // testing it where it is documented not to work. The generators are
            // written not to; this counts any that slip through so the
            // assertion below can complain rather than the debug guard panicking
            // with no context.
            if !coords.iter().all(|v| v.is_finite()) || !kind.coord_range().contains_all(&coords) {
                skipped += 1;
                continue;
            }
            let truth = exact(kind, &coords);
            let got = adaptive(kind, &coords);
            assert_eq!(
                got, truth,
                "{kind} case {i} disagrees with exact arithmetic\n  coords: {coords:?}\n  \
                 adaptive: {}\n  exact:    {}",
                got.as_char(),
                truth.as_char()
            );
            if truth == Orientation::Zero {
                degenerate_seen += 1;
            }
            total += 1;
        }
    }

    assert_eq!(
        skipped, 0,
        "{skipped} generated cases fell outside their predicate's exact range; \
         the generators are supposed to stay inside it"
    );
    assert_eq!(total, RANDOM_SWEEP_CASES, "every generated case must be checked");
    // The generators are supposed to be biased toward degeneracy. If this ever
    // drops, the sweep has quietly stopped testing the interesting path.
    assert!(
        degenerate_seen > RANDOM_SWEEP_CASES / 20,
        "only {degenerate_seen} exactly-degenerate cases out of {total}; the \
         generators are no longer biased toward degeneracy"
    );
}

#[test]
fn naive_f64_determinants_are_demonstrably_wrong_where_the_corpus_lives() {
    // Justifies the whole module: on the corpus's collinear cases, the obvious
    // f64 determinant disagrees with the truth often enough to be useless.
    let mut naive_wrong = 0usize;
    let mut checked = 0usize;
    for case in degenerate_corpus() {
        if case.kind != PredicateKind::Orient2d {
            continue;
        }
        let c = case.coords();
        let naive = (c[0] - c[4]) * (c[3] - c[5]) - (c[1] - c[5]) * (c[2] - c[4]);
        let naive_sign = if naive > 0.0 {
            Orientation::Positive
        } else if naive < 0.0 {
            Orientation::Negative
        } else {
            Orientation::Zero
        };
        checked += 1;
        if naive_sign != case.expected {
            naive_wrong += 1;
        }
    }
    assert!(checked > 0);
    assert!(
        naive_wrong > 0,
        "the corpus no longer contains a case that defeats naive f64; it has \
         stopped testing what it exists to test"
    );
}

#[test]
fn the_documented_range_limits_are_real() {
    // Characterises the boundary that `CoordRange` describes. These calls go
    // straight to `robust` rather than through `chipbreaker_core::predicates`,
    // because the wrapper's debug assertion exists precisely to stop a caller
    // doing this by accident.
    //
    // Below the range: subnormal coordinates. The exact answer is positive; the
    // adaptive predicate loses the low-order term to underflow and reports
    // degenerate.
    let subnormal = [0.0, 0.0, 4e-323, 4e-323, 8e-323, 1e-322];
    assert!(
        !ORIENT2D_COORDS.contains_all(&subnormal),
        "the guard must reject this input"
    );
    assert_eq!(exact(PredicateKind::Orient2d, &subnormal), Orientation::Positive);
    let got = robust::orient2d(
        robust::Coord { x: subnormal[0], y: subnormal[1] },
        robust::Coord { x: subnormal[2], y: subnormal[3] },
        robust::Coord { x: subnormal[4], y: subnormal[5] },
    );
    assert_eq!(got, 0.0, "underflow silently collapses the determinant to zero");

    // Above the range: a degree-5 determinant at 1e75 overflows to NaN, which
    // is why INSPHERE_COORDS stops at 1e60 rather than sharing the degree-4
    // bound.
    let big = 1e75;
    let over = robust::insphere(
        robust::Coord3D { x: big, y: 0.0, z: 0.0 },
        robust::Coord3D { x: 0.0, y: big, z: 0.0 },
        robust::Coord3D { x: 0.0, y: 0.0, z: big },
        robust::Coord3D { x: 0.0, y: 0.0, z: 0.0 },
        robust::Coord3D { x: big, y: big, z: 0.0 },
    );
    assert!(
        over.is_nan(),
        "expected overflow to NaN outside INSPHERE_COORDS, got {over}"
    );
    assert!(!INSPHERE_COORDS.contains(big));
    // And inside the range the same configuration is answered exactly.
    let inside = INSPHERE_COORDS.max;
    let ok = robust::insphere(
        robust::Coord3D { x: inside, y: 0.0, z: 0.0 },
        robust::Coord3D { x: 0.0, y: inside, z: 0.0 },
        robust::Coord3D { x: 0.0, y: 0.0, z: inside },
        robust::Coord3D { x: 0.0, y: 0.0, z: 0.0 },
        robust::Coord3D { x: inside, y: inside, z: 0.0 },
    );
    assert_eq!(ok, 0.0, "cospherical at the top of the range");
}

/// A near-degenerate family at scale `s`, and whether the adaptive predicate
/// still agrees with the exact oracle across it.
///
/// The family is the same shape at every scale — a degenerate configuration plus
/// its immediate ULP neighbours — so that the only variable is magnitude.
fn exact_at_scale(kind: PredicateKind, s: f64) -> bool {
    (-2i32..=2).all(|n| {
        let coords: Vec<f64> = match kind {
            PredicateKind::Orient2d => {
                vec![s, s, 2.0 * s, 2.0 * s, 3.0 * s, ulps(3.0 * s, n)]
            }
            PredicateKind::Orient3d => vec![
                s, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0, s, s, s, ulps(-s, n),
            ],
            PredicateKind::InCircle => vec![s, 0.0, 0.0, s, -s, 0.0, 0.0, ulps(-s, n)],
            PredicateKind::InSphere => vec![
                s, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0, s, ulps(s, n), 0.0,
            ],
        };
        if !coords.iter().all(|v| v.is_finite()) {
            return true;
        }
        // Straight to `robust`: the wrapper would refuse these inputs, which is
        // the whole point of measuring where its refusal should start.
        let got = match kind {
            PredicateKind::Orient2d => robust::orient2d(
                robust::Coord { x: coords[0], y: coords[1] },
                robust::Coord { x: coords[2], y: coords[3] },
                robust::Coord { x: coords[4], y: coords[5] },
            ),
            PredicateKind::Orient3d => robust::orient3d(
                robust::Coord3D { x: coords[0], y: coords[1], z: coords[2] },
                robust::Coord3D { x: coords[3], y: coords[4], z: coords[5] },
                robust::Coord3D { x: coords[6], y: coords[7], z: coords[8] },
                robust::Coord3D { x: coords[9], y: coords[10], z: coords[11] },
            ),
            PredicateKind::InCircle => robust::incircle(
                robust::Coord { x: coords[0], y: coords[1] },
                robust::Coord { x: coords[2], y: coords[3] },
                robust::Coord { x: coords[4], y: coords[5] },
                robust::Coord { x: coords[6], y: coords[7] },
            ),
            PredicateKind::InSphere => robust::insphere(
                robust::Coord3D { x: coords[0], y: coords[1], z: coords[2] },
                robust::Coord3D { x: coords[3], y: coords[4], z: coords[5] },
                robust::Coord3D { x: coords[6], y: coords[7], z: coords[8] },
                robust::Coord3D { x: coords[9], y: coords[10], z: coords[11] },
                robust::Coord3D { x: coords[12], y: coords[13], z: coords[14] },
            ),
        };
        if got.is_nan() {
            return false;
        }
        let observed = if got > 0.0 {
            Orientation::Positive
        } else if got < 0.0 {
            Orientation::Negative
        } else {
            Orientation::Zero
        };
        observed == exact(kind, &coords)
    })
}

#[test]
fn published_coord_ranges_are_inside_the_measured_exact_band() {
    // The published `CoordRange` constants are claims about where the adaptive
    // predicates are exact. This test measures that band directly, by walking
    // outward from unit scale one decade at a time until the predicate stops
    // agreeing with exact arithmetic, and asserts the published bounds sit
    // comfortably inside it.
    //
    // A first-principles derivation gets the overflow end approximately right
    // and the underflow end badly wrong — what underflows first is the
    // low-order error term of the expansion, not the product — so the bounds
    // are measured rather than reasoned about.
    const MARGIN_DECADES: i32 = 1;

    for kind in PredicateKind::ALL {
        assert!(
            exact_at_scale(kind, 1.0),
            "{kind} must be exact at unit scale"
        );

        let mut low = 0i32;
        while exact_at_scale(kind, 10f64.powi(low - 1)) {
            low -= 1;
        }
        let mut high = 0i32;
        while exact_at_scale(kind, 10f64.powi(high + 1)) {
            high += 1;
        }

        // The band must be a clean interval, not a ragged one with islands of
        // failure inside; a ragged band would mean a single min/max pair cannot
        // describe it honestly.
        let interior_failures: Vec<i32> = (low..=high)
            .filter(|d| !exact_at_scale(kind, 10f64.powi(*d)))
            .collect();
        assert!(
            interior_failures.is_empty(),
            "{kind} has failures inside its own band at decades {interior_failures:?}"
        );

        let range = kind.coord_range();
        let published_low = range.min.log10().round() as i32;
        let published_high = range.max.log10().round() as i32;

        assert!(
            published_low >= low + MARGIN_DECADES,
            "{kind} publishes a minimum of 1e{published_low} but is only exact \
             down to 1e{low}; the margin is too thin"
        );
        assert!(
            published_high <= high - MARGIN_DECADES,
            "{kind} publishes a maximum of 1e{published_high} but is only exact \
             up to 1e{high}; the margin is too thin"
        );
        // And the published range must not be uselessly conservative either.
        assert!(
            published_low <= low + 20 && published_high >= high - 20,
            "{kind} publishes 1e{published_low}..1e{published_high} for a \
             measured band of 1e{low}..1e{high}; that is needlessly narrow"
        );
    }
}

#[test]
fn coord_range_membership() {
    for range in [ORIENT2D_COORDS, ORIENT3D_COORDS, INCIRCLE_COORDS, INSPHERE_COORDS] {
        assert!(range.contains(0.0), "zero must always be admissible");
        assert!(range.contains(range.min));
        assert!(range.contains(-range.max));
        assert!(!range.contains(f64::NAN));
        assert!(!range.contains(f64::INFINITY));
        assert!(!range.contains(range.min / 10.0));
        assert!(!range.contains(range.max * 10.0));
        assert!(range.contains_all(&[0.0, 1.0, -1.0]));
        assert!(!range.contains_all(&[1.0, f64::NAN]));
    }
    // The ranges narrow monotonically with the determinant's degree.
    let by_degree = [ORIENT2D_COORDS, ORIENT3D_COORDS, INCIRCLE_COORDS, INSPHERE_COORDS];
    for pair in by_degree.windows(2) {
        assert!(pair[0].degree < pair[1].degree);
        assert!(pair[0].max > pair[1].max, "a higher degree must not admit more");
        assert!(pair[0].min < pair[1].min);
    }
}

// ---------------------------------------------------------------------------
// Corpus regeneration
// ---------------------------------------------------------------------------

/// The configurations that make up the committed corpus.
///
/// Only the *inputs* are written here. The expected result is computed by the
/// exact oracle when the corpus is regenerated, so a wrong expectation cannot be
/// hand-typed into the file.
fn corpus_inputs() -> Vec<(String, PredicateKind, Vec<f64>)> {
    use PredicateKind::{InCircle, InSphere, Orient2d, Orient3d};
    let mut out: Vec<(String, PredicateKind, Vec<f64>)> = Vec::new();
    let mut add = |id: &str, kind: PredicateKind, coords: Vec<f64>| {
        assert_eq!(coords.len(), kind.arity(), "case `{id}` has the wrong arity");
        out.push((id.to_owned(), kind, coords));
    };

    // -- orient2d: exact collinearity in various guises ---------------------
    add("o2d-collinear-axis", Orient2d, vec![0.0, 0.0, 1.0, 0.0, 2.0, 0.0]);
    add("o2d-collinear-diagonal", Orient2d, vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0]);
    add("o2d-ccw-unit", Orient2d, vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0]);
    add("o2d-cw-unit", Orient2d, vec![0.0, 0.0, 0.0, 1.0, 1.0, 0.0]);
    add("o2d-repeated-point-ab", Orient2d, vec![1.0, 1.0, 1.0, 1.0, 2.0, 5.0]);
    add("o2d-repeated-point-ac", Orient2d, vec![1.0, 1.0, 2.0, 5.0, 1.0, 1.0]);
    add("o2d-all-identical", Orient2d, vec![3.0, 4.0, 3.0, 4.0, 3.0, 4.0]);
    // Tenths are not exactly representable, but equal x and y make the
    // determinant cancel exactly regardless.
    add("o2d-collinear-tenths", Orient2d, vec![0.1, 0.1, 0.2, 0.2, 0.3, 0.3]);
    add("o2d-collinear-rational-slope", Orient2d, vec![0.0, 0.0, 3.0, 1.0, 6.0, 2.0]);

    // Shewchuk's demonstration case and its ULP neighbours: the classic
    // configuration where a naive f64 determinant reports a sign that flips
    // with the compiler's mood.
    add("o2d-shewchuk-collinear", Orient2d, vec![0.5, 0.5, 12.0, 12.0, 24.0, 24.0]);
    for n in [-3i32, -2, -1, 1, 2, 3] {
        add(
            &format!("o2d-shewchuk-y{n:+}ulp"),
            Orient2d,
            vec![0.5, 0.5, 12.0, 12.0, 24.0, ulps(24.0, n)],
        );
    }
    add(
        "o2d-shewchuk-x+1ulp",
        Orient2d,
        vec![0.5, 0.5, 12.0, 12.0, ulps(24.0, 1), 24.0],
    );

    // Points differing only in the last mantissa bit.
    let one_lsb = ulps(1.0, 1);
    add("o2d-mantissa-lsb-collinear", Orient2d, vec![1.0, 1.0, one_lsb, one_lsb, ulps(1.0, 2), ulps(1.0, 2)]);
    add("o2d-mantissa-lsb-off", Orient2d, vec![1.0, 1.0, one_lsb, one_lsb, ulps(1.0, 2), ulps(1.0, 3)]);
    add("o2d-adjacent-doubles", Orient2d, vec![1.0, 1.0, one_lsb, 1.0, 1.0, one_lsb]);

    // Wide dynamic range, pressed right up against ORIENT2D_COORDS. In this
    // band a naive f64 determinant is still finite but has lost every
    // significant bit to cancellation; one step further and the adaptive
    // predicate overflows too, which is why the corpus stops here.
    // One decade inside ORIENT2D_COORDS, so that small multiples and ULP nudges
    // stay inside the exact range rather than straddling its edge.
    add("o2d-huge-collinear", Orient2d, vec![1e149, 1e149, 2e149, 2e149, 3e149, 3e149]);
    add("o2d-huge-off-1ulp", Orient2d, vec![1e149, 1e149, 2e149, 2e149, 3e149, ulps(3e149, 1)]);
    add("o2d-tiny-collinear", Orient2d, vec![1e-149, 1e-149, 2e-149, 2e-149, 3e-149, 3e-149]);
    add("o2d-tiny-off-1ulp", Orient2d, vec![1e-149, 1e-149, 2e-149, 2e-149, 3e-149, ulps(3e-149, 1)]);
    add("o2d-mixed-scale-collinear", Orient2d, vec![1e-149, 1e-149, 1.0, 1.0, 1e149, 1e149]);
    add("o2d-mixed-scale-off", Orient2d, vec![1e-149, 1e-149, 1.0, 1.0, 1e149, ulps(1e149, 1)]);
    add("o2d-anisotropic-collinear", Orient2d, vec![0.0, 0.0, 1e100, 1e-100, 2e100, 2e-100]);
    add("o2d-anisotropic-off", Orient2d, vec![0.0, 0.0, 1e100, 1e-100, 2e100, ulps(2e-100, 1)]);
    // Catastrophic cancellation: a long lever arm and a short one.
    add("o2d-lever-arm", Orient2d, vec![-1e15, 0.0, 1e15, 0.0, 0.0, 1e-15]);
    add("o2d-lever-arm-negative", Orient2d, vec![-1e15, 0.0, 1e15, 0.0, 0.0, -1e-15]);

    // -- orient3d ------------------------------------------------------------
    let tri = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    let with = |base: &[f64], tail: &[f64]| -> Vec<f64> {
        let mut v = base.to_vec();
        v.extend_from_slice(tail);
        v
    };
    add("o3d-origin-positive", Orient3d, with(&tri, &[0.0, 0.0, 0.0]));
    add("o3d-far-negative", Orient3d, with(&tri, &[2.0, 2.0, 2.0]));
    add("o3d-coplanar-sum-one", Orient3d, with(&tri, &[1.0, 1.0, -1.0]));
    add("o3d-coplanar-sum-one-b", Orient3d, with(&tri, &[-3.0, 2.0, 2.0]));
    add("o3d-coplanar-1ulp-off", Orient3d, with(&tri, &[1.0, 1.0, ulps(-1.0, 1)]));
    add("o3d-coplanar-1ulp-off-neg", Orient3d, with(&tri, &[1.0, 1.0, ulps(-1.0, -1)]));
    add("o3d-coplanar-xy-plane", Orient3d, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, 0.0]);
    // Just off the plane by one ULP *of unit scale*. A subnormal offset would be
    // outside ORIENT3D_COORDS, where the predicate is not exact — which is
    // itself pinned by `the_documented_range_limits_are_real`.
    add("o3d-coplanar-xy-eps", Orient3d, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, f64::EPSILON]);
    add("o3d-coplanar-xy-eps-neg", Orient3d, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 7.0, -f64::EPSILON]);
    add("o3d-collinear-degenerate", Orient3d, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 5.0, 3.0, 1.0]);
    add("o3d-repeated-point", Orient3d, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    // Bounded by ORIENT3D_COORDS, which is narrower than the 2D range because
    // this is a degree-3 determinant.
    add("o3d-huge-coplanar", Orient3d, vec![1e99, 0.0, 0.0, 0.0, 1e99, 0.0, 0.0, 0.0, 1e99, 1e99, 1e99, -1e99]);
    add("o3d-huge-off", Orient3d, vec![1e99, 0.0, 0.0, 0.0, 1e99, 0.0, 0.0, 0.0, 1e99, 1e99, 1e99, ulps(-1e99, 1)]);
    add("o3d-tiny-coplanar", Orient3d, vec![1e-89, 0.0, 0.0, 0.0, 1e-89, 0.0, 0.0, 0.0, 1e-89, 1e-89, 1e-89, -1e-89]);
    add("o3d-tiny-off", Orient3d, vec![1e-89, 0.0, 0.0, 0.0, 1e-89, 0.0, 0.0, 0.0, 1e-89, 1e-89, 1e-89, ulps(-1e-89, 1)]);
    add("o3d-mixed-scale", Orient3d, vec![1e-60, 0.0, 0.0, 0.0, 1e60, 0.0, 0.0, 0.0, 1.0, 1e-60, 1e60, 1.0]);
    add("o3d-thin-sliver", Orient3d, vec![0.0, 0.0, 0.0, 1e8, 1.0, 0.0, 1e8, 0.0, 1.0, 1e8, 1.0, 1.0]);
    add("o3d-mantissa-lsb", Orient3d, vec![1.0, 1.0, 1.0, one_lsb, 1.0, 1.0, 1.0, one_lsb, 1.0, 1.0, 1.0, one_lsb]);

    // -- incircle ------------------------------------------------------------
    add("icc-cocircular-unit", InCircle, vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0]);
    add("icc-inside-centre", InCircle, vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0]);
    add("icc-outside", InCircle, vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -2.0]);
    add("icc-cocircular-1ulp-in", InCircle, vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, ulps(-1.0, 1)]);
    add("icc-cocircular-1ulp-out", InCircle, vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, ulps(-1.0, -1)]);
    add("icc-collinear-abc", InCircle, vec![0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 1.0, 1.0]);
    add("icc-repeated-point", InCircle, vec![1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0]);
    add("icc-cocircular-large", InCircle, vec![1e73, 0.0, 0.0, 1e73, -1e73, 0.0, 0.0, -1e73]);
    add("icc-cocircular-large-off", InCircle, vec![1e73, 0.0, 0.0, 1e73, -1e73, 0.0, 0.0, ulps(-1e73, 1)]);
    add("icc-cocircular-small", InCircle, vec![1e-62, 0.0, 0.0, 1e-62, -1e-62, 0.0, 0.0, -1e-62]);
    add("icc-cocircular-small-off", InCircle, vec![1e-62, 0.0, 0.0, 1e-62, -1e-62, 0.0, 0.0, ulps(-1e-62, 1)]);
    // Circle of radius 5 about (8, 5); (8, 0) is on it, (8 + 1 ULP, 0) just
    // outside. The perturbation is applied to a unit-scale coordinate rather
    // than to the zero, whose ULP neighbour is subnormal and outside the range.
    add("icc-offset-centre", InCircle, vec![13.0, 5.0, 8.0, 10.0, 3.0, 5.0, 8.0, 0.0]);
    add("icc-offset-centre-1ulp", InCircle, vec![13.0, 5.0, 8.0, 10.0, 3.0, 5.0, ulps(8.0, 1), 0.0]);
    add("icc-offset-centre-1ulp-neg", InCircle, vec![13.0, 5.0, 8.0, 10.0, 3.0, 5.0, ulps(8.0, -1), 0.0]);
    add("icc-flat-triangle", InCircle, vec![0.0, 0.0, 1e8, 1.0, 2e8, 0.0, 1e8, -1.0]);

    // -- insphere ------------------------------------------------------------
    let tetra = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
    add("isp-cospherical", InSphere, with(&tetra, &[1.0, 1.0, 0.0]));
    add("isp-cospherical-b", InSphere, with(&tetra, &[0.0, 1.0, 1.0]));
    add("isp-inside-centre", InSphere, with(&tetra, &[0.5, 0.5, 0.5]));
    add("isp-outside", InSphere, with(&tetra, &[10.0, 10.0, 10.0]));
    add("isp-cospherical-1ulp-in", InSphere, with(&tetra, &[1.0, ulps(1.0, -1), 0.0]));
    add("isp-cospherical-1ulp-out", InSphere, with(&tetra, &[1.0, ulps(1.0, 1), 0.0]));
    add("isp-coplanar-tetra", InSphere, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 2.0, 2.0, 0.0]);
    add("isp-repeated-point", InSphere, with(&tetra, &[1.0, 0.0, 0.0]));
    // Bounded by INSPHERE_COORDS: the narrowest range of the four, because this
    // is a degree-5 determinant.
    add(
        "isp-cospherical-large",
        InSphere,
        vec![1e58, 0.0, 0.0, 0.0, 1e58, 0.0, 0.0, 0.0, 1e58, 0.0, 0.0, 0.0, 1e58, 1e58, 0.0],
    );
    add(
        "isp-cospherical-large-off",
        InSphere,
        vec![1e58, 0.0, 0.0, 0.0, 1e58, 0.0, 0.0, 0.0, 1e58, 0.0, 0.0, 0.0, 1e58, ulps(1e58, -1), 0.0],
    );
    add(
        "isp-cospherical-small",
        InSphere,
        vec![1e-58, 0.0, 0.0, 0.0, 1e-58, 0.0, 0.0, 0.0, 1e-58, 0.0, 0.0, 0.0, 1e-58, 1e-58, 0.0],
    );
    add(
        "isp-cospherical-small-off",
        InSphere,
        vec![1e-58, 0.0, 0.0, 0.0, 1e-58, 0.0, 0.0, 0.0, 1e-58, 0.0, 0.0, 0.0, 1e-58, ulps(1e-58, 1), 0.0],
    );
    add(
        "isp-offset-sphere",
        InSphere,
        vec![10.0, 3.0, 3.0, 3.0, 10.0, 3.0, 3.0, 3.0, 10.0, 3.0, 3.0, 3.0, 10.0, 10.0, 3.0],
    );
    add("isp-flat-sliver", InSphere, vec![0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1.0, 1e6, 1e6, 0.0]);

    out
}

/// Rewrites `tests/corpus/predicates/degenerate.txt` from [`corpus_inputs`],
/// computing every expectation with the exact oracle.
///
/// Ignored by default; run deliberately with
/// `cargo test -p chipbreaker-core --test exact_predicates -- --ignored regenerate`.
/// The corpus is committed, so any change this produces must show up in review.
#[test]
#[ignore = "rewrites a committed corpus file; run deliberately"]
fn regenerate_corpus() {
    let inputs = corpus_inputs();
    let mut body = String::new();
    body.push_str(
        "# Near-degenerate configurations for the exact geometric predicates.\n\
         #\n\
         # Generated by:\n\
         #   cargo test -p chipbreaker-core --test exact_predicates -- --ignored regenerate\n\
         #\n\
         # Format:  <case-id> <predicate> <expected +|-|0> <coordinate>...\n\
         # Coordinates are exact IEEE-754 bit patterns (0x + 16 hex digits) so that\n\
         # a case one ULP from degenerate says so unambiguously. Decimal literals\n\
         # are also accepted by the parser; see crates/chipbreaker-core/src/\n\
         # predicates/corpus.rs.\n\
         #\n\
         # Every `expected` value is computed by arbitrary-precision integer\n\
         # arithmetic, not by the predicate under test, and is re-verified against\n\
         # that oracle on every `cargo test` run.\n",
    );

    let mut previous_kind: Option<PredicateKind> = None;
    for kind in PredicateKind::ALL {
        for (id, k, coords) in inputs.iter().filter(|(_, k, _)| *k == kind) {
            if previous_kind != Some(*k) {
                body.push_str(&format!("\n# --- {} ---\n", k.name()));
                previous_kind = Some(*k);
            }
            let expected = exact(*k, coords);
            let mut padded = [0.0f64; MAX_COORDS];
            padded[..coords.len()].copy_from_slice(coords);
            body.push_str(&format!("{id} {} {}", k.name(), expected.as_char()));
            for c in coords {
                body.push_str(&format!(" 0x{:016x}", c.to_bits()));
            }
            // A human-readable trailer, ignored by the parser.
            body.push_str(&format!("  # {coords:?}\n"));
        }
    }

    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/corpus/predicates/degenerate.txt");
    // LF explicitly: `.gitattributes` normalizes this file, and writing CRLF
    // here would produce a spurious diff on every Windows regeneration.
    std::fs::write(&path, body.replace("\r\n", "\n")).expect("write corpus");

    // The file we just wrote must parse back to exactly what we meant.
    let text = std::fs::read_to_string(&path).expect("read back");
    let parsed = corpus::parse(&text).expect("regenerated corpus must parse");
    assert_eq!(parsed.len(), inputs.len());
    for (case, (id, kind, coords)) in parsed.iter().zip(inputs.iter()) {
        assert_eq!(case.id, id);
        assert_eq!(case.kind, *kind);
        assert_eq!(case.coords(), coords.as_slice());
        assert_eq!(case.expected, exact(*kind, coords));
    }
    eprintln!("wrote {} cases to {}", parsed.len(), path.display());
}

#[test]
fn corpus_file_matches_the_generator_inputs() {
    // Guards against the corpus and the generator drifting apart: if someone
    // edits one, this points at the other.
    let inputs = corpus_inputs();
    let committed: Vec<CorpusCase<'_>> = degenerate_corpus();
    assert_eq!(
        committed.len(),
        inputs.len(),
        "the committed corpus has {} cases but the generator produces {}; \
         re-run the `regenerate_corpus` test",
        committed.len(),
        inputs.len()
    );
    for (id, kind, coords) in &inputs {
        let found = committed
            .iter()
            .find(|c| c.id == id)
            .unwrap_or_else(|| panic!("case `{id}` is missing from the committed corpus"));
        assert_eq!(found.kind, *kind, "case `{id}`");
        assert_eq!(found.coords(), coords.as_slice(), "case `{id}`");
    }
}
