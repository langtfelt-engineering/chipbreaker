// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Property tests for the span set algebra.
//!
//! # The seed is fixed on purpose
//!
//! `proptest` seeds itself from entropy by default, which means a CI failure may
//! not reproduce locally and a passing run tells you nothing about what was
//! actually tried. Every runner here is built with an explicit
//! [`SEED`], so a failure reproduces on the first attempt on any machine, and
//! failure persistence is switched off so there is no `.proptest-regressions`
//! file to drift out of sync with the code.
//!
//! # Why the generators use a coarse integer grid
//!
//! The set-algebra identities hold **exactly** only when every endpoint is
//! separated from every other by more than [`chipbreaker_core::eps::EPS_SPAN_MERGE`].
//! In the sliver regime they legitimately fail, and no tolerance-based
//! implementation can make them hold — see the `spans` module documentation.
//! Generating on a grid of spacing `1.0`, nine orders of magnitude above the
//! tolerance, means these tests exercise the algebra rather than the epsilon
//! policy. The epsilon policy is pinned separately by
//! `tolerance_behaviour_is_characterised` in the `spans` module.

use chipbreaker_core::eps::approx_eq;
use chipbreaker_core::golden::Hashable;
use chipbreaker_core::spans::{Span, Spans};
use proptest::prelude::*;
use proptest::test_runner::{Config, FileFailurePersistence, RngAlgorithm, TestRng, TestRunner};

/// The documented proptest seed. Exactly 32 bytes.
const SEED: [u8; 32] = *b"chipbreaker.spans.proptest.seed1";

/// Cases per property, as required by the unit specification.
const CASES: u32 = 10_000;

fn runner() -> TestRunner {
    let config = Config {
        cases: CASES,
        // No regression file: the fixed seed already makes runs reproducible,
        // and a persisted file would silently change what CI tests.
        failure_persistence: Some(Box::new(FileFailurePersistence::Off)),
        ..Config::default()
    };
    TestRunner::new_with_rng(config, TestRng::from_seed(RngAlgorithm::ChaCha, &SEED))
}

/// Runs a property, turning a proptest failure into a test panic that names the
/// minimal counterexample.
fn check<S: Strategy>(strategy: S, body: impl Fn(S::Value) -> Result<(), TestCaseError>) {
    if let Err(e) = runner().run(&strategy, body) {
        panic!("{e}");
    }
}

/// Span sets on an integer grid: at most `max` spans, each 1 to 7 units long,
/// separated by gaps of 0 to 5 units. A gap of zero produces abutting spans,
/// which must fuse — that case is deliberately reachable.
fn spans_strategy(max: usize) -> impl Strategy<Value = Spans> {
    proptest::collection::vec((0i64..6, 1i64..8), 0..=max).prop_map(|steps| {
        let mut out = Spans::new();
        let mut t = -20i64;
        for (gap, len) in steps {
            t += gap;
            #[expect(
                clippy::cast_precision_loss,
                reason = "grid coordinates are small integers, exactly representable"
            )]
            out.push_merge(Span::new(t as f64, (t + len) as f64));
            t += len;
        }
        out.normalize();
        out
    })
}

fn pair() -> impl Strategy<Value = (Spans, Spans)> {
    (spans_strategy(8), spans_strategy(8))
}

fn triple() -> impl Strategy<Value = (Spans, Spans, Spans)> {
    (spans_strategy(6), spans_strategy(6), spans_strategy(6))
}

#[test]
fn union_is_idempotent() {
    check(spans_strategy(8), |a| {
        prop_assert_eq!(a.union(&a), a.clone());
        prop_assert_eq!(a.intersect(&a), a.clone());
        prop_assert!(a.subtract(&a).is_empty());
        Ok(())
    });
}

#[test]
fn union_and_intersection_commute() {
    check(pair(), |(a, b)| {
        prop_assert_eq!(a.union(&b), b.union(&a));
        prop_assert_eq!(a.intersect(&b), b.intersect(&a));
        Ok(())
    });
}

#[test]
fn union_and_intersection_associate() {
    check(triple(), |(a, b, c)| {
        prop_assert_eq!(a.union(&b).union(&c), a.union(&b.union(&c)));
        prop_assert_eq!(a.intersect(&b).intersect(&c), a.intersect(&b.intersect(&c)));
        Ok(())
    });
}

#[test]
fn difference_is_disjoint_from_the_subtrahend() {
    check(pair(), |(a, b)| {
        let d = a.subtract(&b);
        prop_assert!(
            d.intersect(&b).is_empty(),
            "(a - b) ∩ b = {} for a = {}, b = {}",
            d.intersect(&b),
            a,
            b
        );
        // And the difference never adds anything that was not in `a`.
        prop_assert_eq!(d.intersect(&a), d.clone());
        Ok(())
    });
}

#[test]
fn difference_and_intersection_partition_the_left_operand() {
    check(pair(), |(a, b)| {
        let d = a.subtract(&b);
        let i = a.intersect(&b);
        prop_assert_eq!(d.union(&i), a.clone(), "(a - b) ∪ (a ∩ b) != a");
        prop_assert!(d.intersect(&i).is_empty(), "the two parts must be disjoint");
        Ok(())
    });
}

#[test]
fn measure_obeys_inclusion_exclusion() {
    check(pair(), |(a, b)| {
        let lhs = a.union(&b).measure() + a.intersect(&b).measure();
        let rhs = a.measure() + b.measure();
        prop_assert!(
            approx_eq(lhs, rhs),
            "|a ∪ b| + |a ∩ b| = {lhs} but |a| + |b| = {rhs}"
        );
        // Measure is monotone and never negative.
        prop_assert!(a.measure() >= 0.0);
        prop_assert!(a.intersect(&b).measure() <= a.measure());
        prop_assert!(a.union(&b).measure() >= a.measure());
        Ok(())
    });
}

#[test]
fn complement_within_is_an_involution_on_subsets() {
    check(spans_strategy(8), |a| {
        // Any window that contains `a` will do; widen its hull so the endpoints
        // of `a` are strictly interior as well as flush cases.
        let bounds = a
            .hull()
            .map_or(Span::new(-1.0, 1.0), |h| Span::new(h.t0 - 3.0, h.t1 + 3.0));
        let once = a.complement_within(bounds);
        prop_assert_eq!(once.complement_within(bounds), a.clone());
        // The complement really is complementary, within the window.
        prop_assert!(once.intersect(&a).is_empty());
        prop_assert!(approx_eq(once.measure() + a.measure(), bounds.length()));
        Ok(())
    });
}

#[test]
fn complement_within_a_flush_window_still_round_trips() {
    // The window exactly equal to the hull is the awkward case: the complement
    // has to handle spans that touch the boundary rather than sitting inside it.
    check(spans_strategy(8), |a| {
        if let Some(hull) = a.hull() {
            let once = a.complement_within(hull);
            prop_assert_eq!(once.complement_within(hull), a.clone());
        }
        Ok(())
    });
}

#[test]
fn absorption_and_distributivity_hold() {
    check(triple(), |(a, b, c)| {
        prop_assert_eq!(a.union(&a.intersect(&b)), a.clone(), "a ∪ (a ∩ b) = a");
        prop_assert_eq!(a.intersect(&a.union(&b)), a.clone(), "a ∩ (a ∪ b) = a");
        prop_assert_eq!(
            a.intersect(&b.union(&c)),
            a.intersect(&b).union(&a.intersect(&c)),
            "∩ distributes over ∪"
        );
        prop_assert_eq!(
            a.union(&b.intersect(&c)),
            a.union(&b).intersect(&a.union(&c)),
            "∪ distributes over ∩"
        );
        Ok(())
    });
}

#[test]
fn de_morgan_holds_within_a_window() {
    check(pair(), |(a, b)| {
        let bounds = Span::new(-40.0, 80.0);
        let a = a.clipped_to(bounds);
        let b = b.clipped_to(bounds);
        let not = |s: &Spans| s.complement_within(bounds);
        prop_assert_eq!(not(&a.union(&b)), not(&a).intersect(&not(&b)));
        prop_assert_eq!(not(&a.intersect(&b)), not(&a).union(&not(&b)));
        Ok(())
    });
}

#[test]
fn the_structural_invariant_survives_every_operation() {
    check(pair(), |(a, b)| {
        let bounds = Span::new(-50.0, 50.0);
        let derived = [
            ("a", a.clone()),
            ("b", b.clone()),
            ("union", a.union(&b)),
            ("intersect", a.intersect(&b)),
            ("subtract", a.subtract(&b)),
            ("complement", a.complement_within(bounds)),
            ("clipped", a.clipped_to(bounds)),
        ];
        for (label, set) in derived {
            if let Err(e) = set.check_invariant() {
                return Err(TestCaseError::fail(format!("{label}: {e}")));
            }
            prop_assert!(set.is_normalized(), "{} is not normalized: {}", label, set);
            // Normalization is idempotent: running it again changes nothing.
            let mut again = set.clone();
            again.normalize();
            prop_assert_eq!(again, set, "normalize is not idempotent for {}", label);
        }
        Ok(())
    });
}

#[test]
fn contains_agrees_with_a_linear_scan() {
    check(spans_strategy(8), |a| {
        // Probe every endpoint and the points either side of it, which is where
        // the half-open convention actually bites.
        let mut probes: Vec<f64> = vec![-1000.0, 1000.0];
        for s in a.iter() {
            probes.extend_from_slice(&[
                s.t0 - 0.5,
                s.t0,
                s.midpoint(),
                s.t1 - 0.5,
                s.t1,
                s.t1 + 0.5,
            ]);
        }
        for t in probes {
            let linear = a.iter().any(|s| t >= s.t0 && t < s.t1);
            prop_assert_eq!(
                a.contains(t),
                linear,
                "contains({}) disagreed with a linear scan over {}",
                t,
                a
            );
        }
        prop_assert!(!a.contains(f64::NAN));
        Ok(())
    });
}

#[test]
fn equal_sets_hash_equally_regardless_of_construction() {
    check(pair(), |(a, b)| {
        let via_ops = a.union(&b);
        // The same set rebuilt one span at a time, in order.
        let mut rebuilt = Spans::new();
        for s in via_ops.iter() {
            rebuilt.push_merge(*s);
        }
        prop_assert_eq!(rebuilt.canonical_digest(), via_ops.canonical_digest());
        // And rebuilt from an unsorted pile.
        let mut shuffled: Vec<Span> = via_ops.iter().copied().collect();
        shuffled.reverse();
        prop_assert_eq!(
            Spans::from_unsorted(shuffled).canonical_digest(),
            via_ops.canonical_digest()
        );
        // Different sets must not collide.
        if a != b {
            prop_assert_ne!(a.canonical_digest(), b.canonical_digest());
        }
        Ok(())
    });
}

#[test]
fn scratch_buffer_variants_agree_with_the_allocating_ones() {
    check(pair(), |(a, b)| {
        // Seed the scratch buffer with unrelated content so a failure to clear
        // it would show up.
        let mut out = Spans::from_span(Span::new(-999.0, -900.0));
        a.union_into(&b, &mut out);
        prop_assert_eq!(&out, &a.union(&b));
        a.intersect_into(&b, &mut out);
        prop_assert_eq!(&out, &a.intersect(&b));
        a.subtract_into(&b, &mut out);
        prop_assert_eq!(&out, &a.subtract(&b));
        let bounds = Span::new(-50.0, 50.0);
        a.complement_within_into(bounds, &mut out);
        prop_assert_eq!(&out, &a.complement_within(bounds));
        Ok(())
    });
}
