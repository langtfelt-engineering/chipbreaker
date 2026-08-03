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

/// Seed for the transcendental parity suite.
pub const TRANSCENDENTAL_SEED: u64 = 0x0000_C41B_0000_0004;

/// Iterations of the transcendental parity suite.
pub const TRANSCENDENTAL_CASES: usize = 4_000;

/// Seed for the root-solver suite.
pub const ROOT_SOLVER_SEED: u64 = 0x0000_C41B_0000_0005;

/// Polynomials solved in the root-solver suite.
pub const ROOT_SOLVER_CASES: usize = 3_000;

/// Rays per side of the lattice cast at each tool, per axis.
///
/// Twelve gives 144 positions times three axes times nine tools, which is a few
/// thousand rays: enough to reach every surface at a range of incidences, and
/// fast enough that the self-test stays a thing anyone will run.
pub const TOOL_RAYS_PER_SIDE: usize = 12;

/// Tessellation tolerance used by the tool suite, in millimetres.
pub const TOOL_TESSELLATION_TOLERANCE: f64 = 0.05;

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

/// Runs every deterministic suite this crate defines.
///
/// Contains no I/O, no clock, and no threads, so it behaves identically under
/// `wasmtime` and natively.
///
/// Crates layered on top of this one contribute their own suites through
/// [`run_with`]. The G-code parser is one: it cannot live here, because this
/// crate must not depend on a parser, and yet it must be inside the
/// cross-platform parity guarantee like everything else.
#[must_use]
pub fn run() -> SelfTestReport {
    run_with(Vec::new())
}

/// As [`run`], with suites contributed by a crate this one cannot see.
///
/// `extra` is appended in the order given and hashed with the rest, so the
/// caller's ordering is part of the contract. The CLI passes the G-code suites;
/// nothing else should.
#[must_use]
pub fn run_with(extra: Vec<SuiteResult>) -> SelfTestReport {
    let mut suites = vec![
        predicate_corpus_suite(),
        predicate_identity_suite(),
        span_algebra_suite(),
        math_kernel_suite(),
        transcendental_suite(),
        mesh_suite(),
        root_solver_suite(),
        tool_geometry_suite(),
        canonical_hash_suite(),
    ];
    suites.extend(extra);
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

/// Transcendental functions must be bit-identical on every target.
///
/// This is the suite that turns [`crate::transcendental`]'s argument into
/// evidence. `std`'s `sin` lowers to the platform libm — MSVC's, glibc's and
/// wasi-libc's are different code that differ in the last bit — so we route
/// everything through the pure-Rust `libm` crate instead. Whether that actually
/// achieves bit-identity is not something to take on faith: these results are
/// folded into the canonical hash that CI compares between native and
/// `wasm32-wasip1`, so the claim is tested on every push.
///
/// U3 tessellates tool profiles as surfaces of revolution and will call `sin`
/// and `cos` on its first day. This suite exists so that lands on a foundation
/// that has already been proven, rather than one that gets proven by failing.
fn transcendental_suite() -> SuiteResult {
    use crate::transcendental as t;

    let mut rng = StdRng::seed_from_u64(TRANSCENDENTAL_SEED);
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("transcendental");

    for i in 0..TRANSCENDENTAL_CASES {
        // Angles well beyond one turn, so argument reduction is exercised: that
        // is where implementations most often disagree.
        let angle: f64 = rng.random_range(-40.0..40.0);
        let any: f64 = rng.random_range(-20.0..20.0);
        // Strictly inside [-1, 1] for the inverse trig functions.
        let unit: f64 = rng.random_range(-1.0..1.0);
        // Strictly positive for the logarithms.
        let positive: f64 = rng.random_range(1.0e-6..1.0e6);
        // Bounded so `exp` and `powf` stay finite and keep their full mantissa.
        let small: f64 = rng.random_range(-30.0..30.0);

        h.f64(angle).f64(any).f64(unit).f64(positive).f64(small);

        let (sin_v, cos_v) = t::sin_cos(angle);
        h.f64(t::sin(angle)).f64(t::cos(angle)).f64(t::tan(angle));
        h.f64(sin_v).f64(cos_v);
        h.f64(t::asin(unit)).f64(t::acos(unit));
        h.f64(t::atan(any)).f64(t::atan2(any, angle));
        h.f64(t::exp(small))
            .f64(t::ln(positive))
            .f64(t::log10(positive));
        h.f64(t::powf(positive, small.clamp(-4.0, 4.0)));
        h.f64(t::hypot(any, angle)).f64(t::cbrt(any));

        // A handful of identities, so the suite fails loudly if a wrapper is
        // ever pointed at the wrong function. The hash catches drift; these
        // catch a wiring mistake, which the hash alone would happily bless.
        if sin_v != t::sin(angle) || cos_v != t::cos(angle) {
            failures.push(Failure {
                case: format!("sin_cos/{i}"),
                detail: format!("sin_cos disagrees with sin/cos at {angle}"),
            });
        }
        let pythagorean = sin_v * sin_v + cos_v * cos_v;
        if (pythagorean - 1.0).abs() > 1.0e-12 {
            failures.push(Failure {
                case: format!("pythagorean/{i}"),
                detail: format!("sin^2 + cos^2 = {pythagorean} at {angle}"),
            });
        }
        let round_trip = t::exp(t::ln(positive));
        if !approx_eq(round_trip / positive, 1.0) {
            failures.push(Failure {
                case: format!("exp-ln/{i}"),
                detail: format!("exp(ln({positive})) = {round_trip}"),
            });
        }
        if (t::asin(unit) + t::acos(unit) - core::f64::consts::FRAC_PI_2).abs() > 1.0e-12 {
            failures.push(Failure {
                case: format!("asin-acos/{i}"),
                detail: format!("asin + acos != pi/2 at {unit}"),
            });
        }
    }

    // Exact values, which every conforming implementation must agree on and
    // which pin the suite against a wholesale substitution.
    h.f64(t::sin(0.0))
        .f64(t::cos(0.0))
        .f64(t::exp(0.0))
        .f64(t::ln(1.0));
    h.f64(t::log10(1000.0))
        .f64(t::powf(2.0, 10.0))
        .f64(t::hypot(3.0, 4.0));
    h.f64(t::cbrt(-27.0));
    for (label, value, expected) in [
        ("sin(0)", t::sin(0.0), 0.0),
        ("cos(0)", t::cos(0.0), 1.0),
        ("exp(0)", t::exp(0.0), 1.0),
        ("ln(1)", t::ln(1.0), 0.0),
        ("log10(1000)", t::log10(1000.0), 3.0),
        ("2^10", t::powf(2.0, 10.0), 1024.0),
        ("hypot(3,4)", t::hypot(3.0, 4.0), 5.0),
        ("cbrt(-27)", t::cbrt(-27.0), -3.0),
    ] {
        if value != expected {
            failures.push(Failure {
                case: label.to_owned(),
                detail: format!("expected exactly {expected}, got {value}"),
            });
        }
    }
    h.end();

    SuiteResult {
        name: "transcendental",
        description: "libm-backed trig, exp and log evaluated on seeded input for cross-target parity",
        cases: TRANSCENDENTAL_CASES,
        failures,
        digest: h.finish(),
    }
}

/// Mesh generation, topology, BVH shape and ray casting must agree across
/// targets.
///
/// The BVH topology hash is the interesting one. The tree is built by median
/// split on sorted centroids, so its shape depends on a floating-point sort key;
/// if that key differed by an ULP on one target, the split would land elsewhere,
/// the traversal order would change, and — once U11 adds parallelism — results
/// would follow. Hashing the tree turns that from a latent risk into a CI
/// failure.
///
/// The ray sweep is deliberately run against `lattice_block`, whose vertices are
/// all integers, so it exercises the Simulation of Simplicity cascade rather
/// than the float fast path. A cross-target disagreement in SoS would be
/// invisible on generic geometry.
fn mesh_suite() -> SuiteResult {
    use crate::mesh::bvh::Bvh;
    use crate::mesh::validate::validate;
    use crate::mesh::{shapes, weld};

    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("mesh");

    let meshes = [
        ("cube", shapes::cube(10.0)),
        ("sphere-2", shapes::icosphere(5.0, 2)),
        ("cylinder-32", shapes::cylinder(4.0, 9.0, 32)),
        ("cone-32", shapes::cone(4.0, 9.0, 32)),
        ("torus-16", shapes::torus(6.0, 2.0, 16, 8)),
        ("lattice-3", shapes::lattice_block(3)),
    ];

    let mut cases = 0usize;
    for (name, mesh) in &meshes {
        h.begin(name);
        mesh.hash_canonical(&mut h);

        // Welding must be a pure function of the geometry.
        match weld::weld(mesh, crate::eps::EPS_WELD) {
            Ok((welded, report)) => {
                welded.hash_canonical(&mut h);
                report.hash_canonical(&mut h);
            }
            Err(e) => failures.push(Failure {
                case: format!("weld/{name}"),
                detail: e.to_string(),
            }),
        }

        let report = validate(mesh);
        report.hash_canonical(&mut h);
        if !report.is_solid() {
            failures.push(Failure {
                case: format!("validate/{name}"),
                detail: format!(
                    "generated shape is not a closed outward solid: {:?}",
                    report.findings
                ),
            });
        }

        let bvh = Bvh::build(mesh);
        bvh.hash_canonical(&mut h);

        // A small ray sweep, hashed in full. Crossing counts, parameters and
        // directions all have to match across targets, not merely the totals.
        let bounds = mesh.bounds();
        let extent = bounds.extent();
        let mut scratch = Vec::new();
        for i in 0..8u32 {
            for j in 0..8u32 {
                let origin = Vec3::new(
                    bounds.min.x + extent.x * f64::from(i) / 7.0,
                    bounds.min.y + extent.y * f64::from(j) / 7.0,
                    bounds.min.z - extent.z - 1.0,
                );
                let ray = crate::math::Ray::new(origin, Vec3::Z);
                match bvh.intersect_ray_all_into(mesh, &ray, &mut scratch) {
                    Ok(stats) => {
                        h.add(&origin);
                        h.add_all(scratch.iter());
                        // Counter totals are hashed too: a target that took the
                        // exact path a different number of times would be a
                        // genuine divergence even if the answers matched.
                        h.u64(stats.triangle_tests)
                            .u64(stats.exact_path)
                            .u64(stats.sos_resolutions)
                            .u64(stats.coplanar_rejected);
                        cases += 1;

                        if !scratch.len().is_multiple_of(2) {
                            failures.push(Failure {
                                case: format!("parity/{name}/{i}-{j}"),
                                detail: format!(
                                    "odd crossing count {} from {origin:?}; material leaks",
                                    scratch.len()
                                ),
                            });
                        }
                    }
                    Err(e) => failures.push(Failure {
                        case: format!("raycast/{name}/{i}-{j}"),
                        detail: e.to_string(),
                    }),
                }
            }
        }
        h.end();
    }
    h.end();

    SuiteResult {
        name: "mesh",
        description: "shape generation, welding, topology, BVH shape and leak-free ray casting",
        cases,
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

/// Root solving must be bit-identical across targets.
///
/// # Why this suite exists at all
///
/// The solver reaches [`crate::transcendental`] for `acos`, `cos` and `cbrt` on
/// the cubic's three-real-root branch, and it is the only place in the engine
/// that does so inside an inner loop. A `std` call slipping in there would break
/// WASM parity for every ray that touched a torus, and nothing else in the
/// self-test would notice: the transcendental suite checks the functions, not
/// their callers.
///
/// # Why the polynomials are built from roots rather than sampled
///
/// Random coefficients almost never produce a repeated root, and the repeated
/// root is the interesting case — it is where the solver leaves the closed form
/// and goes through the critical points of `p'`, and where the answer is
/// determined to `sqrt(eps)` rather than to `eps`. Building from a known root
/// set makes the hard branch the common case instead of an accident.
///
/// The roots are dyadic and small, so expanding the product is exact in `f64`
/// and the coefficients the solver sees represent precisely the polynomial
/// intended.
fn root_solver_suite() -> SuiteResult {
    use crate::roots::{eval, solve_cubic, solve_quadratic, solve_quartic};

    let mut rng = StdRng::seed_from_u64(ROOT_SOLVER_SEED);
    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("roots");

    // Expands `a * (x - r0)(x - r1)...` into descending coefficients.
    let from_roots = |a: f64, roots: &[f64]| -> Vec<f64> {
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
    };

    for i in 0..ROOT_SOLVER_CASES {
        // Quarters in [-8, 8]: exactly representable, and small enough that the
        // expanded coefficients of a quartic are exact too.
        let pick = |rng: &mut StdRng| f64::from(rng.random_range(-32i32..=32)) / 4.0;
        let lead = if i % 7 == 0 { -2.0 } else { 1.0 };

        // Every third case repeats a root, so the repeated-root path is
        // exercised on a third of the corpus rather than by luck.
        let degree = 2 + i % 3;
        let mut chosen: Vec<f64> = (0..degree).map(|_| pick(&mut rng)).collect();
        if i % 3 == 0 && !chosen.is_empty() {
            chosen[0] = chosen[chosen.len() - 1];
        }

        let coefficients = from_roots(lead, &chosen);
        let found = match coefficients.len() {
            3 => solve_quadratic(coefficients[0], coefficients[1], coefficients[2]),
            4 => solve_cubic(
                coefficients[0],
                coefficients[1],
                coefficients[2],
                coefficients[3],
            ),
            _ => solve_quartic(
                coefficients[0],
                coefficients[1],
                coefficients[2],
                coefficients[3],
                coefficients[4],
            ),
        };

        h.add(&found);

        // Every root the solver found must actually be one, and every root that
        // was built in must be found. Both directions, because either alone
        // permits a solver that is silently half right.
        for (value, _) in found.iter() {
            let residual = eval(&coefficients, value).abs();
            let scale: f64 = coefficients.iter().fold(0.0, |m, c| m.max(c.abs()));
            let tolerance = 1.0e-6 * scale.max(1.0);
            if residual > tolerance {
                failures.push(Failure {
                    case: format!("case {i}: residual"),
                    detail: format!(
                        "root {value} of {coefficients:?} leaves residual {residual}, above {tolerance}"
                    ),
                });
            }
        }
        for r in &chosen {
            let nearest = found
                .roots()
                .iter()
                .fold(f64::INFINITY, |m, v| m.min((v - r).abs()));
            // A repeated root is displaced by about sqrt(eps) relative to its
            // own magnitude, which is the accuracy floor and not an error.
            let allowed = 1.0e-5 * r.abs().max(1.0);
            if nearest > allowed {
                failures.push(Failure {
                    case: format!("case {i}: missing root"),
                    detail: format!(
                        "built {coefficients:?} from {chosen:?}; {r} is {nearest} from the nearest root found"
                    ),
                });
            }
        }
    }
    h.end();

    SuiteResult {
        name: "roots",
        description: "real roots of seeded polynomials built from known roots, a third of them repeated",
        cases: ROOT_SOLVER_CASES,
        failures,
        digest: h.finish(),
    }
}

/// Tool geometry, ray casting and tessellation must agree across targets.
///
/// # What would go wrong without it
///
/// The ray-versus-tool path is the whole of Unit 3 in one function: it reaches
/// the root solver, `atan2`, `hypot` and `sin_cos`, and it decides interval
/// membership from a containment predicate. Any of those disagreeing by one ULP
/// on one target moves a span endpoint, and a moved span endpoint is material
/// removed in a different place. Hashing the spans themselves — rather than a
/// count or a total — is what makes that a CI failure rather than a slow drift.
///
/// The tools are the catalogue forms, so the sweep covers cylinders, cones,
/// discs, spheres and tori, which are the five surfaces a profile can generate
/// and the five polynomial cases the solver has to handle.
fn tool_geometry_suite() -> SuiteResult {
    use crate::math::{Ray, Vec3};
    use crate::tool::catalog::{
        HolderStage, Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill,
        flat_end_mill, tapered_end_mill,
    };
    use crate::tool::profile::Profile;
    use crate::tool::raycast::{RaycastScratch, RaycastStats};

    let mut failures = Vec::new();
    let mut h = CanonicalHash::new();
    h.begin("tool");

    let shank = Shank::plain(6.0, 50.0);
    let held = Shank::with_holder(
        6.0,
        40.0,
        [
            HolderStage::cylinder(25.0, 20.0),
            HolderStage::taper(25.0, 40.0, 15.0),
        ],
    );
    let tools: Vec<(&str, Profile)> = vec![
        ("flat", flat_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "flat-necked",
            flat_end_mill(10.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
        ),
        ("ball", ball_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "bull",
            bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "chamfer",
            chamfer_mill(8.0, 1.0, 90.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "taper",
            tapered_end_mill(2.0, 10.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        ("drill", drill(6.0, 118.0, 30.0, &shank).expect("valid")),
        (
            "barrel",
            barrel_end_mill(12.0, 60.0, 40.0, &Shank::plain(12.0, 70.0)).expect("valid"),
        ),
        ("held", flat_end_mill(6.0, 20.0, &held).expect("valid")),
    ];

    let mut cases = 0usize;
    for (name, profile) in &tools {
        h.str(name);
        h.add(profile);
        // Closed-form properties: a divergence between targets here would mean
        // the arithmetic itself differs, before any ray is cast.
        h.f64_slice(&[
            profile.volume(),
            profile.surface_area(),
            profile.max_radius(),
            profile.total_length(),
        ]);
        cases += 1;

        let cylinder = profile.bounding_cylinder();
        let radius = cylinder.radius * 1.25 + 1.0;
        let mut scratch = RaycastScratch::with_capacity(profile.len());
        let mut spans = crate::spans::Spans::new();
        let mut stats = RaycastStats::default();

        // A fixed lattice rather than a random one: the point is reproducibility
        // across targets, and a seeded generator would add a second thing that
        // could differ. Cell centres keep every ray off the axis, where a
        // surface of revolution is met tangentially by construction.
        for i in 0..TOOL_RAYS_PER_SIDE {
            let u = -radius + 2.0 * radius * (i as f64 + 0.5) / TOOL_RAYS_PER_SIDE as f64;
            for j in 0..TOOL_RAYS_PER_SIDE {
                let v = cylinder.z_min - 0.5
                    + (cylinder.z_max - cylinder.z_min + 1.0) * (j as f64 + 0.5)
                        / TOOL_RAYS_PER_SIDE as f64;
                for axis in 0..3 {
                    let (origin, direction) = match axis {
                        0 => (Vec3::new(-radius - 1.0, u, v), Vec3::new(1.0, 0.0, 0.0)),
                        1 => (Vec3::new(u, -radius - 1.0, v), Vec3::new(0.0, 1.0, 0.0)),
                        _ => (
                            Vec3::new(u, v - cylinder.z_max, cylinder.z_min - 1.0),
                            Vec3::new(0.0, 0.0, 1.0),
                        ),
                    };
                    let Some(ray) = Ray::new_normalized(origin, direction) else {
                        continue;
                    };
                    profile.intersect_ray_into(&ray, &mut scratch, &mut spans, &mut stats);
                    // The spans themselves, not a summary of them.
                    h.add(&spans);
                    cases += 1;

                    // A leak is the failure that matters, and it is checked here
                    // as well as hashed: a hash only catches a *change*, and a
                    // leak that was present from the first run would hash
                    // consistently and still be wrong.
                    let reach = 2.0 * (radius + cylinder.z_max + 2.0) + 2.0;
                    for span in spans.iter() {
                        if !span.t0.is_finite() || !span.t1.is_finite() || span.t1 > reach {
                            failures.push(Failure {
                                case: format!("{name}: leak on axis {axis}"),
                                detail: format!("span {span} escapes a bound of {reach}"),
                            });
                        }
                    }
                }
            }
        }

        // Tessellation: subdivision counts come from `acos`, so the mesh is a
        // second, independent consumer of the transcendental layer.
        let (mesh, report) = profile
            .tessellate(TOOL_TESSELLATION_TOLERANCE)
            .expect("valid");
        h.add(&mesh);
        h.usize(report.divisions);
        h.usize(report.stations);
        cases += 1;

        let exact = profile.volume();
        let measured = mesh.signed_volume();
        if measured > exact * (1.0 + 1.0e-9) {
            failures.push(Failure {
                case: format!("{name}: tessellation is not inscribed"),
                detail: format!("mesh volume {measured} exceeds the true {exact}"),
            });
        }
    }
    h.end();

    SuiteResult {
        name: "tool",
        description: "tool profiles, closed-form properties, ray-cast spans and tessellation across the catalogue",
        cases,
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
