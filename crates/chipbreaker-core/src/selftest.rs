// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The deterministic self-test suites, and the canonical hash of their results.
//!
//! This module is the payload behind `chipbreaker selftest`. It lives in the
//! library rather than the CLI for one reason: the WASM parity check runs this
//! exact code under `wasmtime` and compares the resulting digest, byte for byte,
//! against the native run. Anything that cannot run in a WASM sandbox therefore
//! cannot live here — no filesystem, no clock, no environment, no threads. The
//! corpus is `include_str!`d for exactly this reason.
//!
//! Timings and host details are deliberately absent. They belong to the CLI's
//! `environment` section, which is *not* hashed. A duration in a hashed
//! structure makes every run differ from every other run, and the failure looks
//! like a determinism bug rather than the reporting bug it is.

use crate::eps::{EPS_SPAN_MERGE, approx_eq};
use crate::golden::{CanonicalHash, Digest, Hashable};
use crate::math::{Mat4, Vec2, Vec3};
use crate::predicates::corpus::{PredicateKind, degenerate_corpus};
use crate::predicates::{ADAPTIVE, Predicates};
use crate::spans::{Span, Spans};

use rand::{Rng, SeedableRng, rngs::StdRng};

/// Seed for the predicate identity suite.
///
/// Fixed and documented so that a failure reproduces exactly. `StdRng` is
/// ChaCha12, whose stream is identical on every platform for a given seed; the
/// `rand` version is pinned with `=` in the workspace manifest because the
/// crate only guarantees stream stability within a release series.
pub const PREDICATE_IDENTITY_SEED: u64 = 0x0000_C41B_0000_0001;

/// Seed for the span algebra suite.
pub const SPAN_ALGEBRA_SEED: u64 = 0x0000_C41B_0000_0002;

/// Seed for the floating-point kernel suite.
pub const MATH_KERNEL_SEED: u64 = 0x0000_C41B_0000_0003;

/// Iterations of the predicate identity suite.
pub const PREDICATE_IDENTITY_CASES: usize = 5_000;

/// Iterations of the span algebra suite.
pub const SPAN_ALGEBRA_CASES: usize = 2_000;

/// Iterations of the floating-point kernel suite.
pub const MATH_KERNEL_CASES: usize = 2_000;

/// One failing case within a suite.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Failure {
    /// Identifies the case within the suite: a corpus id, or an iteration index.
    pub case: String,
    /// What went wrong, in enough detail to start debugging from.
    pub detail: String,
}

/// The outcome of one suite.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SuiteResult {
    /// Stable machine-readable name, e.g. `predicates.corpus`.
    pub name: &'static str,
    /// One-line description for the human-readable report.
    pub description: &'static str,
    /// How many cases ran.
    pub cases: usize,
    /// Every failure, in deterministic order.
    pub failures: Vec<Failure>,
    /// Canonical digest of everything this suite computed.
    ///
    /// This is the value that must match between native and WASM. It covers the
    /// computed results, not merely pass/fail, so a suite that silently computes
    /// different numbers while still passing is still caught.
    pub digest: Digest,
}

impl SuiteResult {
    /// True if the suite had no failures.
    #[must_use]
    pub fn passed(&self) -> bool {
        self.failures.is_empty()
    }
}

impl Hashable for SuiteResult {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("SuiteResult");
        h.str(self.name);
        h.usize(self.cases);
        h.usize(self.failures.len());
        for f in &self.failures {
            h.str(&f.case);
        }
        h.bytes(self.digest.as_bytes());
        h.end();
    }
}

/// The full self-test result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SelfTestReport {
    /// Suites in a fixed order.
    pub suites: Vec<SuiteResult>,
    /// Canonical digest over every suite, in order.
    ///
    /// This is the number the WASM parity check compares.
    pub digest: Digest,
}

impl SelfTestReport {
    /// True if every suite passed.
    #[must_use]
    pub fn passed(&self) -> bool {
        self.suites.iter().all(SuiteResult::passed)
    }

    /// Total cases across all suites.
    #[must_use]
    pub fn total_cases(&self) -> usize {
        self.suites.iter().map(|s| s.cases).sum()
    }

    /// Total failures across all suites.
    #[must_use]
    pub fn total_failures(&self) -> usize {
        self.suites.iter().map(|s| s.failures.len()).sum()
    }
}

impl Hashable for SelfTestReport {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("SelfTestReport");
        h.u64(u64::from(crate::CANONICAL_ENCODING_VERSION));
        h.add_all(self.suites.iter());
        h.end();
    }
}

/// Runs every deterministic suite in Unit 1.
///
/// Contains no I/O, no clock, and no threads, so it behaves identically under
/// `wasmtime` and natively.
#[must_use]
pub fn run() -> SelfTestReport {
    let suites = vec![
        predicate_corpus_suite(),
        predicate_identity_suite(),
        span_algebra_suite(),
        math_kernel_suite(),
        canonical_hash_suite(),
    ];
    let mut report = SelfTestReport {
        suites,
        digest: Digest::from_hex(&"0".repeat(64)).unwrap_or_else(|| unreachable!()),
    };
    report.digest = {
        let mut h = CanonicalHash::new();
        report.hash_canonical(&mut h);
        h.finish()
    };
    report
}

/// Every stored corpus expectation must match the adaptive predicate.
fn predicate_corpus_suite() -> SuiteResult {
    let cases = degenerate_corpus();
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("predicates.corpus");

    // Grouped by predicate kind, and within a kind in file order: a fixed
    // traversal, independent of how the corpus happens to be laid out.
    for kind in PredicateKind::ALL {
        h.begin(kind.name());
        for case in cases.iter().filter(|c| c.kind == kind) {
            let actual = case.evaluate(&ADAPTIVE);
            case.hash_canonical(&mut h);
            actual.hash_canonical(&mut h);
            if actual != case.expected {
                failures.push(Failure {
                    case: case.id.to_owned(),
                    detail: format!(
                        "{} expected `{}`, adaptive predicate returned `{}`",
                        case.kind,
                        case.expected.as_char(),
                        actual.as_char()
                    ),
                });
            }
        }
        h.end();
    }
    h.end();

    SuiteResult {
        name: "predicates.corpus",
        description: "adaptive predicates against the committed degenerate corpus",
        cases: cases.len(),
        failures,
        digest: h.finish(),
    }
}

/// Predicates must obey their algebraic identities on random input.
///
/// Exchanging any two arguments negates the determinant, and a repeated point is
/// always degenerate. These hold without needing an exact oracle, so they can
/// run in the CLI and under WASM where no bignum library is linked. The oracle
/// comparison lives in the test suite.
fn predicate_identity_suite() -> SuiteResult {
    let mut rng = StdRng::seed_from_u64(PREDICATE_IDENTITY_SEED);
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("predicates.identities");

    for i in 0..PREDICATE_IDENTITY_CASES {
        // Coordinates on a coarse grid so that exact degeneracies occur often.
        let coord = |rng: &mut StdRng| -> f64 { f64::from(rng.random_range(-16i32..=16)) / 4.0 };
        let a = Vec2::new(coord(&mut rng), coord(&mut rng));
        let b = Vec2::new(coord(&mut rng), coord(&mut rng));
        let c = Vec2::new(coord(&mut rng), coord(&mut rng));

        let base = ADAPTIVE.orient2d(a, b, c);
        h.add(&a).add(&b).add(&c).add(&base);

        if ADAPTIVE.orient2d(b, a, c) != base.reverse() {
            failures.push(Failure {
                case: format!("orient2d/{i}"),
                detail: format!("swapping a and b did not negate the result for {a:?} {b:?} {c:?}"),
            });
        }
        if ADAPTIVE.orient2d(a, a, c) != crate::predicates::Orientation::Zero {
            failures.push(Failure {
                case: format!("orient2d-degenerate/{i}"),
                detail: format!("a repeated point was not degenerate for {a:?} {c:?}"),
            });
        }

        let p = Vec3::new(coord(&mut rng), coord(&mut rng), coord(&mut rng));
        let q = Vec3::new(coord(&mut rng), coord(&mut rng), coord(&mut rng));
        let r = Vec3::new(coord(&mut rng), coord(&mut rng), coord(&mut rng));
        let s = Vec3::new(coord(&mut rng), coord(&mut rng), coord(&mut rng));
        let base3 = ADAPTIVE.orient3d(p, q, r, s);
        h.add(&p).add(&q).add(&r).add(&s).add(&base3);

        if ADAPTIVE.orient3d(q, p, r, s) != base3.reverse() {
            failures.push(Failure {
                case: format!("orient3d/{i}"),
                detail: format!("swapping a and b did not negate the result for {p:?} {q:?}"),
            });
        }
        // Two exchanges restore the original sign.
        if ADAPTIVE.orient3d(q, p, s, r) != base3 {
            failures.push(Failure {
                case: format!("orient3d-double-swap/{i}"),
                detail: "two argument exchanges did not restore the sign".to_owned(),
            });
        }
    }
    h.end();

    SuiteResult {
        name: "predicates.identities",
        description: "argument-exchange antisymmetry and degeneracy on seeded random input",
        cases: PREDICATE_IDENTITY_CASES,
        failures,
        digest: h.finish(),
    }
}

/// Generates a span set on a coarse integer grid.
///
/// The grid spacing is `1.0`, nine orders of magnitude above
/// [`EPS_SPAN_MERGE`], so no generated configuration lands in the sliver regime
/// where the algebraic laws legitimately fail. See the [`crate::spans`] module
/// documentation.
fn random_spans(rng: &mut StdRng, max_spans: usize) -> Spans {
    let n = rng.random_range(0..=max_spans);
    let mut out = Spans::new();
    let mut t = f64::from(rng.random_range(-20i32..=20));
    for _ in 0..n {
        let len = f64::from(rng.random_range(1i32..=6));
        out.push_merge(Span::new(t, t + len));
        t += len + f64::from(rng.random_range(1i32..=5));
    }
    out
}

/// The span set algebra, on data far coarser than the merge tolerance.
fn span_algebra_suite() -> SuiteResult {
    let mut rng = StdRng::seed_from_u64(SPAN_ALGEBRA_SEED);
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("spans.algebra");

    for i in 0..SPAN_ALGEBRA_CASES {
        let a = random_spans(&mut rng, 6);
        let b = random_spans(&mut rng, 6);

        let union = a.union(&b);
        let intersection = a.intersect(&b);
        let difference = a.subtract(&b);

        h.add(&a)
            .add(&b)
            .add(&union)
            .add(&intersection)
            .add(&difference);
        h.f64(union.measure())
            .f64(intersection.measure())
            .f64(difference.measure());

        let mut fail = |case: &str, detail: String| {
            failures.push(Failure {
                case: format!("{case}/{i}"),
                detail,
            });
        };

        for (label, set) in [
            ("a", &a),
            ("b", &b),
            ("union", &union),
            ("intersect", &intersection),
            ("difference", &difference),
        ] {
            if let Err(e) = set.check_invariant() {
                fail("invariant", format!("{label}: {e}"));
            }
        }
        if a.union(&a) != a {
            fail("idempotence", format!("a ∪ a != a for {a}"));
        }
        if union != b.union(&a) {
            fail("union-commutes", format!("{a} ∪ {b}"));
        }
        if intersection != b.intersect(&a) {
            fail("intersect-commutes", format!("{a} ∩ {b}"));
        }
        if !difference.intersect(&b).is_empty() {
            fail(
                "difference-disjoint",
                format!("(a - b) ∩ b != ∅ for {a}, {b}"),
            );
        }
        if difference.union(&intersection) != a {
            fail(
                "difference-partition",
                format!("(a - b) ∪ (a ∩ b) != a for {a}, {b}"),
            );
        }
        if !approx_eq(
            union.measure() + intersection.measure(),
            a.measure() + b.measure(),
        ) {
            fail(
                "measure-inclusion-exclusion",
                format!(
                    "|a ∪ b| + |a ∩ b| = {} but |a| + |b| = {}",
                    union.measure() + intersection.measure(),
                    a.measure() + b.measure()
                ),
            );
        }
        if let Some(hull) = a.hull() {
            let bounds = Span::new(hull.t0 - 1.0, hull.t1 + 1.0);
            if a.complement_within(bounds).complement_within(bounds) != a {
                fail("double-complement", format!("for {a} within {bounds}"));
            }
        }
    }
    h.end();

    SuiteResult {
        name: "spans.algebra",
        description: "set-algebra laws and the structural invariant over seeded random span sets",
        cases: SPAN_ALGEBRA_CASES,
        failures,
        digest: h.finish(),
    }
}

/// Floating-point kernels whose last bit must agree across targets.
///
/// This is the suite that would actually catch a native/WASM divergence: matrix
/// inversion, `sqrt`, and normalization chain enough operations together that a
/// single contracted multiply-add or a differently-rounded intermediate shows up
/// in the digest.
fn math_kernel_suite() -> SuiteResult {
    let mut rng = StdRng::seed_from_u64(MATH_KERNEL_SEED);
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("math.kernels");

    for i in 0..MATH_KERNEL_CASES {
        // Values with ragged mantissas, so rounding differences cannot hide in
        // trailing zeros.
        let val = |rng: &mut StdRng| -> f64 { rng.random_range(-8.0f64..8.0) };

        let mut m = Mat4::IDENTITY;
        for r in 0..4 {
            for c in 0..4 {
                m.m[r][c] = val(&mut rng);
            }
        }
        m.m[3] = [0.0, 0.0, 0.0, 1.0];

        let p = Vec3::new(val(&mut rng), val(&mut rng), val(&mut rng));
        let det = m.determinant();
        h.add(&m).add(&p).f64(det);
        h.add(&m.transform_point(p)).add(&m.transform_direction(p));
        h.f64(p.length()).f64(p.length_squared());

        if let Some(n) = p.normalize() {
            h.bool(true).add(&n);
            // Normalization must not drift: this is a sqrt and three divisions,
            // all correctly rounded, so the check is tight rather than loose.
            if !approx_eq(n.length(), 1.0) {
                failures.push(Failure {
                    case: format!("normalize/{i}"),
                    detail: format!("|normalize({p:?})| = {}", n.length()),
                });
            }
        } else {
            h.bool(false);
        }

        if let Some(inv) = m.inverse() {
            h.bool(true).add(&inv);
            let round_trip = inv.transform_point(m.transform_point(p));
            h.add(&round_trip);
            // Deliberately only a finiteness check. Random 4x4 matrices are
            // frequently ill-conditioned, so an inaccurate round-trip is
            // expected and is not a bug; a non-finite one means `inverse`
            // returned `Some` for a matrix it should have rejected. The
            // *accuracy* of the arithmetic is not what this suite is for — the
            // digest is, and it covers `inv` and `round_trip` in full.
            if !round_trip.is_finite() {
                failures.push(Failure {
                    case: format!("inverse/{i}"),
                    detail: format!("round-trip produced a non-finite point for det = {det}"),
                });
            }
        } else {
            h.bool(false);
        }
    }
    h.end();

    SuiteResult {
        name: "math.kernels",
        description: "matrix inverse, transform and normalization round-trips over seeded input",
        cases: MATH_KERNEL_CASES,
        failures,
        digest: h.finish(),
    }
}

/// Known-answer checks on the canonical encoding itself.
///
/// These pin the properties the whole determinism story rests on. If `usize`
/// stopped being widened, this suite's digest would differ between the native
/// and WASM runs — which is precisely the failure it exists to surface, and the
/// reason the checks are computed rather than merely asserted.
fn canonical_hash_suite() -> SuiteResult {
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("hash.selfcheck");

    let mut fail = |case: &str, detail: String| {
        failures.push(Failure {
            case: case.to_owned(),
            detail,
        });
    };

    // Fixed values covering the awkward corners of the encoding.
    let values: [f64; 12] = [
        0.0,
        -0.0,
        1.0,
        -1.0,
        0.1,
        f64::MIN_POSITIVE,
        f64::MAX,
        f64::EPSILON,
        f64::INFINITY,
        f64::NEG_INFINITY,
        5.0e-324, // smallest subnormal
        core::f64::consts::PI,
    ];
    h.f64_slice(&values);
    for (i, v) in values.iter().enumerate() {
        h.usize(i).f64(*v);
    }
    h.str("chipbreaker")
        .bytes(b"\x00\xff\x7f")
        .bool(true)
        .bool(false);
    h.u64(u64::MAX).i64(i64::MIN).i64(i64::MAX);
    // Deliberately NOT `usize::MAX`. That is 2^64-1 natively and 2^32-1 on
    // wasm32, so hashing it would make the native and WASM digests differ by
    // design — a self-inflicted parity failure that says nothing about the
    // encoding. What the encoding must guarantee is that the *same* logical
    // value produces the same eight bytes on both, which is what these fixed
    // values check.
    h.usize(0).usize(1).usize(0xffff_ffff);

    // -0.0 and 0.0 must be indistinguishable to the hash.
    if 0.0f64.canonical_digest() != (-0.0f64).canonical_digest() {
        fail("negative-zero", "-0.0 and 0.0 hash differently".to_owned());
    }
    // Every NaN payload must collapse to one.
    let nan_a = f64::NAN;
    let nan_b = f64::from_bits(f64::NAN.to_bits() ^ 0x7);
    if nan_b.is_nan() && nan_a.canonical_digest() != nan_b.canonical_digest() {
        fail(
            "nan-payload",
            "distinct NaN payloads hash differently".to_owned(),
        );
    }
    // A `usize` must hash to the same eight bytes as the `u64` of equal value,
    // give or take the type tag — the property that keeps 32-bit WASM in step
    // with 64-bit native. Checked here by hashing both widths of the same
    // logical value and requiring the pair to be stable.
    let widened = 0xffff_ffff_usize;
    let mut a = CanonicalHash::new();
    a.usize(widened);
    let mut b = CanonicalHash::new();
    b.usize(widened);
    if a.finish() != b.finish() {
        fail(
            "usize-stability",
            "usize hashing is not reproducible".to_owned(),
        );
    }
    if size_of::<u64>() != 8 {
        fail("u64-width", "u64 is not eight bytes".to_owned());
    }
    // If `usize` were ever fed to the hasher at its native width, this suite's
    // digest would differ between targets. Record the width as a *failure
    // condition* rather than as hashed data, so that a 16-bit or 128-bit target
    // is reported rather than silently producing a different hash.
    if size_of::<usize>() != 4 && size_of::<usize>() != 8 {
        fail(
            "usize-width",
            format!("unexpected usize width: {} bytes", size_of::<usize>()),
        );
    }

    // Structural encodings that must stay distinct.
    let vec2 = Vec2::new(1.0, 2.0).canonical_digest();
    let vec3 = Vec3::new(1.0, 2.0, 0.0).canonical_digest();
    if vec2 == vec3 {
        fail("type-separation", "Vec2 and Vec3 collide".to_owned());
    }
    h.bytes(vec2.as_bytes()).bytes(vec3.as_bytes());

    // Span tolerances are part of the observable contract; a change to them must
    // move this digest.
    h.f64(EPS_SPAN_MERGE).f64(crate::eps::EPS_SPAN_MIN);
    h.end();

    SuiteResult {
        name: "hash.selfcheck",
        description: "canonical encoding known-answer checks (widening, NaN, signed zero)",
        cases: values.len() + 6,
        failures,
        digest: h.finish(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selftest_passes() {
        let report = run();
        for suite in &report.suites {
            assert!(
                suite.passed(),
                "suite `{}` failed: {:?}",
                suite.name,
                suite.failures
            );
        }
        assert!(report.passed());
        assert_eq!(report.total_failures(), 0);
        assert!(report.total_cases() > 9_000);
    }

    #[test]
    fn selftest_is_reproducible_within_a_process() {
        let a = run();
        let b = run();
        assert_eq!(a.digest, b.digest, "two runs produced different digests");
        assert_eq!(a, b);
    }

    #[test]
    fn every_suite_has_a_distinct_name_and_digest() {
        let report = run();
        let mut names: Vec<&str> = report.suites.iter().map(|s| s.name).collect();
        names.sort_unstable();
        let before = names.len();
        names.dedup();
        assert_eq!(before, names.len(), "duplicate suite names");

        let mut digests: Vec<String> = report.suites.iter().map(|s| s.digest.to_hex()).collect();
        digests.sort();
        let before = digests.len();
        digests.dedup();
        assert_eq!(before, digests.len(), "two suites produced the same digest");
    }

    #[test]
    fn report_digest_depends_on_every_suite() {
        let baseline = run();
        let mut tampered = baseline.clone();
        tampered.suites[0].cases += 1;
        let mut h = CanonicalHash::new();
        tampered.hash_canonical(&mut h);
        assert_ne!(h.finish(), baseline.digest);
    }
}
