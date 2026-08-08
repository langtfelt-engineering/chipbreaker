// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Long-running randomized stress on [`Spans`].
//!
//! Unlike `spans_properties.rs`, this asserts almost nothing about *what* the
//! operations compute. It applies a million random operations — including
//! deliberately hostile inputs the property tests exclude, such as NaN bounds,
//! infinities, sub-tolerance slivers and out-of-order pushes — and checks only
//! that the structural invariant survives and that nothing panics.
//!
//! That is the point. The property tests cover the well-behaved regime because
//! the algebraic laws only hold there; this covers the regime where the laws do
//! *not* hold, and where the only thing that must remain true is that the data
//! structure is never left in a corrupt state.
//!
//! `#[ignore]`d so it runs nightly rather than on every commit. Run it with:
//!
//! ```sh
//! cargo test -p chipbreaker-core --release --test spans_fuzz -- --ignored --nocapture
//! ```

use chipbreaker_core::eps::{EPS_SPAN_MERGE, EPS_SPAN_MIN};
use chipbreaker_core::spans::{Span, Spans};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Fixed and documented, so a nightly failure reproduces exactly.
const FUZZ_SEED: u64 = 0x0000_C41B_0000_0020;

/// Operations applied, as required by the unit specification.
const OPERATIONS: usize = 1_000_000;

/// Coordinate range the fuzz works over. Wide relative to the span lengths
/// below, so that a set genuinely accumulates many disjoint spans instead of
/// collapsing into one interval that covers everything.
const EXTENT: f64 = 5_000.0;

/// A bound drawn from the awkward parts of the `f64` line.
///
/// Weighted toward values that stress the tolerance policy: coordinates within a
/// merge threshold of each other, exact zeros, and the occasional infinity or
/// NaN, which `Spans` must discard rather than propagate.
fn hostile_bound(rng: &mut StdRng) -> f64 {
    match rng.random_range(0u32..100) {
        0..=49 => f64::from(rng.random_range(-50i32..=50)),
        50..=69 => f64::from(rng.random_range(-50i32..=50)) + EPS_SPAN_MERGE * 0.5,
        70..=84 => f64::from(rng.random_range(-50i32..=50)) + EPS_SPAN_MIN * 1.5,
        85..=91 => rng.random_range(-EXTENT..EXTENT),
        92..=94 => 0.0,
        95..=96 => -0.0,
        97 => f64::INFINITY,
        98 => f64::NEG_INFINITY,
        _ => f64::NAN,
    }
}

/// A span drawn from the awkward cases: degenerate, inverted, NaN-bearing,
/// infinite, or a sub-tolerance sliver.
fn hostile_span(rng: &mut StdRng) -> Span {
    let a = hostile_bound(rng);
    let b = if rng.random_bool(0.25) {
        // A span that is degenerate, inverted or a hair long.
        a
    } else {
        hostile_bound(rng)
    };
    if rng.random_bool(0.5) {
        Span::new(a, b)
    } else {
        Span::ordered(a, b)
    }
}

/// A short, well-formed span at a random position over [`EXTENT`].
///
/// Without these the set never accumulates: two random bounds over a bounded
/// range produce long overlapping intervals that fuse into one span, and the
/// fuzz then spends a million operations on a set of size one. Short spans over
/// a wide range are what actually exercises the merge-scan.
fn short_span(rng: &mut StdRng) -> Span {
    let t = rng.random_range(-EXTENT..EXTENT);
    let len = rng.random_range(0.05f64..3.0);
    Span::new(t, t + len)
}

/// A window that is usually valid, so `complement_within` does real work rather
/// than returning empty for a degenerate bound.
fn window(rng: &mut StdRng) -> Span {
    if rng.random_bool(0.9) {
        let t = rng.random_range(-EXTENT..0.0);
        Span::new(t, t + rng.random_range(1.0f64..2.0 * EXTENT))
    } else {
        hostile_span(rng)
    }
}

/// Everything that must hold after every single operation.
///
/// Deliberately narrow: this fuzz makes no claim about *what* the operations
/// compute — the property tests do that, in the regime where the algebraic laws
/// actually hold. Here the only requirements are that the structure is never
/// left corrupt, that no NaN is ever stored, and that nothing panics.
fn audit(
    a: &Spans,
    b: &Spans,
    op: usize,
    rng: &mut StdRng,
    largest: &mut usize,
    substantial: &mut usize,
) {
    *largest = (*largest).max(a.len()).max(b.len());
    if a.len() >= 16 {
        *substantial += 1;
    }

    // Reading must never panic either, including on hostile probes.
    let _ = a.measure();
    let _ = a.hull();
    let _ = a.contains(hostile_bound(rng));

    for (label, set) in [("a", a), ("b", b)] {
        if let Err(e) = set.check_invariant() {
            panic!("invariant broken on `{label}` after {op} operations: {e}\nset = {set}");
        }
    }

    for s in a.iter().chain(b.iter()) {
        assert!(
            s.is_valid(),
            "stored an invalid span {s} after {op} operations"
        );
        assert!(
            !s.t0.is_nan() && !s.t1.is_nan(),
            "stored NaN after {op} operations"
        );
    }

    // Unbounded growth would mean merging has stopped working; the generators
    // cannot produce more than a few thousand disjoint spans over EXTENT.
    assert!(
        a.len() < 100_000,
        "set grew to {} spans after {op} operations; merging has failed",
        a.len()
    );
}

#[test]
#[ignore = "one million operations; runs nightly, not on every commit"]
fn a_million_operations_never_corrupt_the_invariant_or_panic() {
    let mut rng = StdRng::seed_from_u64(FUZZ_SEED);
    let mut a = Spans::new();
    let mut b = Spans::new();
    let mut scratch = Spans::new();

    let mut ops_applied = 0usize;
    let mut resets = 0usize;
    let mut largest = 0usize;
    let mut substantial = 0usize;

    // Alternating build and mutate phases, rather than one uniformly random
    // operation mix. A uniform mix does not work here: every intersection, clip
    // or complement collapses the set, and with any appreciable share of those
    // the fuzz spends its million operations on sets of one or two spans. Long
    // build phases guarantee the merge-scan is actually exercised at width.
    while ops_applied < OPERATIONS {
        let build = rng.random_range(50usize..500);
        for _ in 0..build {
            if ops_applied >= OPERATIONS {
                break;
            }
            let target_a = rng.random_bool(0.65);
            let span = if rng.random_bool(0.8) {
                short_span(&mut rng)
            } else {
                hostile_span(&mut rng)
            };
            if target_a {
                a.push_merge(span);
            } else {
                b.push_merge(span);
            }
            ops_applied += 1;
            audit(
                &a,
                &b,
                ops_applied,
                &mut rng,
                &mut largest,
                &mut substantial,
            );
        }

        let mutations = rng.random_range(1usize..6);
        for _ in 0..mutations {
            if ops_applied >= OPERATIONS {
                break;
            }
            match rng.random_range(0u32..9) {
                0 => a = a.union(&b),
                1 => b = b.union(&a),
                2 => a = a.intersect(&b),
                3 => a = a.subtract(&b),
                4 => a = a.complement_within(window(&mut rng)),
                5 => a = a.clipped_to(window(&mut rng)),
                6 => {
                    a.subtract_into(&b, &mut scratch);
                    core::mem::swap(&mut a, &mut scratch);
                }
                7 => a.normalize(),
                _ => {
                    core::mem::swap(&mut a, &mut b);
                    if rng.random_bool(0.3) {
                        a.clear();
                        resets += 1;
                    }
                }
            }
            ops_applied += 1;
            audit(
                &a,
                &b,
                ops_applied,
                &mut rng,
                &mut largest,
                &mut substantial,
            );
        }
    }

    assert_eq!(ops_applied, OPERATIONS);
    assert!(
        resets > 0,
        "the fuzz never reset; it explored one shape only"
    );
    // Without these the test can pass while doing nothing: an operation mix that
    // collapses to the empty set checks the invariant of an empty vector a
    // million times and reports success.
    assert!(
        largest >= 100,
        "the largest set reached only {largest} spans; the fuzz is collapsing \
         instead of exercising the merge-scan"
    );
    assert!(
        substantial > OPERATIONS / 4,
        "only {substantial} of {OPERATIONS} operations ran against a set of 16 \
         or more spans; the fuzz is spending its time on trivial input"
    );
    eprintln!(
        "{OPERATIONS} operations survived: {resets} resets, largest set {largest} spans, \
         {substantial} operations on sets of 16+ spans, final |a| = {}",
        a.len()
    );
}

#[test]
fn the_fuzz_generators_actually_produce_hostile_input() {
    // A cheap guard that runs on every commit: if the generators stopped
    // producing NaN, infinities and slivers, the nightly fuzz would still pass
    // while testing nothing.
    let mut rng = StdRng::seed_from_u64(FUZZ_SEED);
    let mut nan = 0;
    let mut infinite = 0;
    let mut sliver = 0;
    let mut inverted = 0;
    for _ in 0..20_000 {
        let s = hostile_span(&mut rng);
        if s.t0.is_nan() || s.t1.is_nan() {
            nan += 1;
        } else if s.t0.is_infinite() || s.t1.is_infinite() {
            infinite += 1;
        } else if s.t1 > s.t0 && s.length() < EPS_SPAN_MIN {
            sliver += 1;
        } else if s.t1 < s.t0 {
            inverted += 1;
        }
    }
    assert!(nan > 0, "no NaN spans generated");
    assert!(infinite > 0, "no infinite spans generated");
    assert!(sliver > 0, "no sub-threshold slivers generated");
    assert!(inverted > 0, "no inverted spans generated");
}
