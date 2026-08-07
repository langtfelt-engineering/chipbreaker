// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Metamorphic properties of cutting, every one asserted **bit-identically**.
//!
//! These do not check that the answer is right. They check that the answer does
//! not depend on things it must not depend on: where a path was split, where the
//! stock happens to sit, what order disjoint cuts were applied in, or how many
//! times the same cut was repeated.
//!
//! That makes them the tests most likely to find a real bug, because every one
//! of those independences is something an optimisation can quietly break — and
//! because a difference of one ULP fails them, they catch drift long before a
//! tolerance-based test would notice.

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Mat4, Vec3};
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::LinearMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, flat_end_mill};

const METHOD: SweepMethod = SweepMethod::Analytic { tolerance: 1.0e-3 };

fn mill() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn stock_at(offset: Vec3) -> TriDexelField {
    let mesh = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0));
    TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: 0.5,
            placement: Mat4::from_translation(offset),
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn stock() -> TriDexelField {
    stock_at(Vec3::new(0.0, 0.0, 0.0))
}

fn digest(field: &TriDexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

/// Applies a list of moves to fresh stock and returns the digest.
fn cut_all(moves: &[LinearMove]) -> (TriDexelField, String) {
    let profile = mill();
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);
    for motion in moves {
        cut_tri(&mut field, &profile, motion, METHOD, &mut scratch);
    }
    let d = digest(&field);
    (field, d)
}

#[test]
fn splitting_a_path_at_a_segment_boundary_changes_nothing() {
    let a = LinearMove {
        start: Vec3::new(4.0, 8.0, 4.0),
        end: Vec3::new(20.0, 8.0, 4.0),
    };
    let b = LinearMove {
        start: Vec3::new(20.0, 8.0, 4.0),
        end: Vec3::new(36.0, 22.0, 4.0),
    };
    let (_, together) = cut_all(&[a, b]);

    // The same two moves, applied in two separate passes over the field. The
    // field is the accumulator, so this is the same computation split in time.
    let profile = mill();
    let mut field = stock();
    let mut scratch = CutScratch::new(&profile);
    cut_tri(&mut field, &profile, &a, METHOD, &mut scratch);
    let mut scratch = CutScratch::new(&profile);
    cut_tri(&mut field, &profile, &b, METHOD, &mut scratch);

    assert_eq!(
        digest(&field),
        together,
        "splitting a path between segments must be bit-identical"
    );
}

/// Removed volume, span structure, and worst endpoint difference in ULP.
fn compare(a: &TriDexelField, b: &TriDexelField) -> (bool, i64) {
    let mut same_structure = true;
    let mut worst_ulp = 0i64;
    for axis in chipbreaker_core::dexel::tri::AXES {
        let (Some(x), Some(y)) = (a.bundle(axis), b.bundle(axis)) else {
            continue;
        };
        let rays = u32::try_from(x.arena().rays()).expect("small");
        for ray in 0..rays {
            let (p, q) = (x.arena().get(ray), y.arena().get(ray));
            if p.len() != q.len() {
                same_structure = false;
                continue;
            }
            for (u, v) in p.iter().zip(q) {
                for (m, n) in [(u.t0, v.t0), (u.t1, v.t1)] {
                    #[allow(
                        clippy::cast_possible_wrap,
                        reason = "both are ordinary positive lengths"
                    )]
                    let d = (m.to_bits() as i64 - n.to_bits() as i64).abs();
                    worst_ulp = worst_ulp.max(d);
                }
            }
        }
    }
    (same_structure, worst_ulp)
}

#[test]
fn splitting_a_segment_mid_way_agrees_to_within_a_few_ulp() {
    // **The specification asked for bit-identity here and it is not achievable.**
    //
    // A mid-segment split introduces a point that did not exist before, and that
    // point is rounded. The whole move's flank direction is
    // `(end - start) / |end - start|`; the second piece's is
    // `(end - mid) / |end - mid|`. Those are equal in the reals and differ by an
    // ULP in `f64`, because `mid` is not the exact point. Measured on a
    // (5,6) -> (35,24) move split at a quarter:
    //
    // ```text
    // whole  0.8574929257125442  0.5144957554275266
    // piece2 0.8574929257125442  0.5144957554275265
    // ```
    //
    // There is nothing for an implementation to recover: the two computations
    // have different inputs. So bit-identity is impossible, and demanding it
    // would mean either refusing to split paths or pretending.
    //
    // What IS achievable, and is asserted here, is the contract that matters:
    //
    // - **Removed volume bit-identical.** The quantity a customer is told. The
    //   comparison below is on `f64::to_bits` of the summed accumulator, not on
    //   a formatted string, so it really is bit equality. It holds by
    //   ABSORPTION rather than by cancellation: a 1e-16 mm endpoint shift is
    //   far below the ULP of an accumulator of order 100 mm^3, so the
    //   difference never survives into the sum. Do not read it as evidence that
    //   the two computations agreed exactly -- they did not, and the endpoint
    //   assertion below is what says by how much.
    // - **The same span structure.** No ray gains or loses a span, so no hairline
    //   sliver of material is left behind at the join -- which is the failure
    //   mode that would actually show on a part.
    // - **Endpoints within a few ULP.**
    //
    // Splitting at a segment BOUNDARY remains bit-identical, and the test above
    // asserts it, because there the shared point is exactly equal on both sides.
    //
    // The Z bundle is bit-identical even mid-segment, at 0 of 4800 rays: its
    // spans end on the swept prism's top and bottom, which do not depend on the
    // horizontal direction at all. Only the flank can see the difference.
    let profile = mill();
    let whole = LinearMove {
        start: Vec3::new(5.0, 6.0, 3.5),
        end: Vec3::new(35.0, 24.0, 3.5),
    };

    let mut once = stock();
    let mut scratch = CutScratch::new(&profile);
    let before = once.volume();
    cut_tri(&mut once, &profile, &whole, METHOD, &mut scratch);
    let removed_once = before - once.volume();

    for fraction in [0.5, 0.25, 1.0 / 3.0, 0.87] {
        let middle = whole.at(fraction);
        let mut split = stock();
        let mut scratch = CutScratch::new(&profile);
        let before = split.volume();
        for piece in [
            LinearMove {
                start: whole.start,
                end: middle,
            },
            LinearMove {
                start: middle,
                end: whole.end,
            },
        ] {
            cut_tri(&mut split, &profile, &piece, METHOD, &mut scratch);
        }
        let removed_split = before - split.volume();

        assert_eq!(
            removed_once.to_bits(),
            removed_split.to_bits(),
            "split at {fraction}: removed volume must be bit-identical, got              {removed_once} against {removed_split}"
        );
        let (structure, ulp) = compare(&once, &split);
        assert!(
            structure,
            "split at {fraction} changed the span structure, which means a sliver              of material was left at the join"
        );
        assert!(
            ulp <= 16,
            "split at {fraction}: endpoints differ by {ulp} ULP, far more than the              rounding of the split point can explain"
        );
    }
}

#[test]
fn translating_the_stock_and_the_path_translates_the_result() {
    // Not bit-identical on the digest -- the lattice origin moves with the
    // stock, so the field legitimately differs -- but the REMOVED VOLUME must
    // match on the bits, because it is a geometric quantity that translation
    // cannot change.
    let profile = mill();
    let offset = Vec3::new(17.0, -23.0, 9.0);
    let motion = LinearMove {
        start: Vec3::new(6.0, 7.0, 4.0),
        end: Vec3::new(33.0, 21.0, 4.0),
    };

    let mut here = stock();
    let mut scratch = CutScratch::new(&profile);
    let before_here = here.volume();
    cut_tri(&mut here, &profile, &motion, METHOD, &mut scratch);
    let removed_here = before_here - here.volume();

    let mut there = stock_at(offset);
    let moved = LinearMove {
        start: motion.start + offset,
        end: motion.end + offset,
    };
    let mut scratch = CutScratch::new(&profile);
    let before_there = there.volume();
    cut_tri(&mut there, &profile, &moved, METHOD, &mut scratch);
    let removed_there = before_there - there.volume();

    assert_eq!(
        removed_here.to_bits(),
        removed_there.to_bits(),
        "translating stock and path together changed the removed volume: \
         {removed_here} against {removed_there}"
    );
}

#[test]
fn two_disjoint_cuts_commute() {
    // Spatially disjoint, so neither can see the other's material. Order must
    // not matter, on the bits.
    let far_left = LinearMove {
        start: Vec3::new(3.0, 5.0, 4.0),
        end: Vec3::new(14.0, 5.0, 4.0),
    };
    let far_right = LinearMove {
        start: Vec3::new(26.0, 25.0, 4.0),
        end: Vec3::new(37.0, 25.0, 4.0),
    };

    let (_, forward) = cut_all(&[far_left, far_right]);
    let (_, backward) = cut_all(&[far_right, far_left]);
    assert_eq!(
        forward, backward,
        "disjoint cuts must commute bit-identically"
    );
}

#[test]
fn overlapping_cuts_also_commute_because_subtraction_does() {
    // Stronger than the specification asks, and worth pinning: set subtraction
    // is order-independent even when the sets overlap, so cuts that DO see each
    // other's material must commute too. If this ever fails, something has
    // started depending on what material was present when it ran -- which is
    // exactly the accumulation the unit is built to avoid.
    let a = LinearMove {
        start: Vec3::new(5.0, 12.0, 4.0),
        end: Vec3::new(30.0, 12.0, 4.0),
    };
    let b = LinearMove {
        start: Vec3::new(18.0, 4.0, 4.0),
        end: Vec3::new(18.0, 26.0, 4.0),
    };
    let (_, forward) = cut_all(&[a, b]);
    let (_, backward) = cut_all(&[b, a]);
    assert_eq!(forward, backward, "overlapping cuts must commute too");
}

#[test]
fn cutting_the_same_segment_twice_equals_cutting_it_once() {
    let motion = LinearMove {
        start: Vec3::new(7.0, 9.0, 3.0),
        end: Vec3::new(31.0, 19.0, 3.0),
    };
    let (_, once) = cut_all(&[motion]);
    let (_, twice) = cut_all(&[motion, motion]);
    let (_, five) = cut_all(&[motion, motion, motion, motion, motion]);
    assert_eq!(twice, once, "cutting twice must equal cutting once");
    assert_eq!(five, once, "and so must cutting five times");
}

#[test]
fn the_result_is_identical_across_repeated_runs() {
    let motions = [
        LinearMove {
            start: Vec3::new(4.0, 6.0, 3.0),
            end: Vec3::new(36.0, 6.0, 3.0),
        },
        LinearMove {
            start: Vec3::new(36.0, 6.0, 3.0),
            end: Vec3::new(36.0, 24.0, 3.0),
        },
        LinearMove {
            start: Vec3::new(20.0, 15.0, 11.0),
            end: Vec3::new(20.0, 15.0, 2.0),
        },
    ];
    let (_, first) = cut_all(&motions);
    for _ in 0..4 {
        let (_, again) = cut_all(&motions);
        assert_eq!(again, first, "the same job must produce the same field");
    }
}

#[test]
fn a_plunge_split_in_two_agrees_to_within_a_few_ulp() {
    // The same property on Case B, where the mechanism is entirely different:
    // the exact path dilates spans rather than unioning three pieces.
    //
    // I expected this one to be bit-identical, on the reasoning that a plunge
    // splits along its own axis so only `z` is rounded. That was wrong. The
    // second piece casts the static tool at the SPLIT height rather than the
    // original one, and `origin.z - 6.0` is not `(origin.z - 11.0) + 5.0` in
    // floating point. A new position is a new rounding wherever it sits.
    let profile = ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    let whole = LinearMove {
        start: Vec3::new(20.0, 15.0, 11.0),
        end: Vec3::new(20.0, 15.0, 1.0),
    };

    let run = |moves: &[LinearMove]| {
        let mut field = stock();
        let mut scratch = CutScratch::new(&profile);
        let before = field.volume();
        for m in moves {
            cut_tri(&mut field, &profile, m, METHOD, &mut scratch);
        }
        let removed = before - field.volume();
        (field, removed)
    };

    let (once, removed_once) = run(&[whole]);
    for fraction in [0.5, 0.3, 0.75] {
        let middle = whole.at(fraction);
        let (split, removed_split) = run(&[
            LinearMove {
                start: whole.start,
                end: middle,
            },
            LinearMove {
                start: middle,
                end: whole.end,
            },
        ]);
        assert_eq!(
            removed_once.to_bits(),
            removed_split.to_bits(),
            "plunge split at {fraction}: removed volume must be bit-identical"
        );
        let (structure, ulp) = compare(&once, &split);
        assert!(structure, "plunge split at {fraction} left a sliver");
        assert!(
            ulp <= 16,
            "plunge split at {fraction}: endpoints differ by {ulp} ULP"
        );
    }
}

#[test]
fn a_cut_entirely_outside_the_stock_changes_nothing_at_all() {
    // The rejection path, checked for correctness rather than speed: a move that
    // never touches the stock must leave the field bit-identical, not merely
    // close.
    let (_, untouched) = cut_all(&[]);
    let (_, missed) = cut_all(&[LinearMove {
        start: Vec3::new(-50.0, -50.0, 40.0),
        end: Vec3::new(-20.0, -50.0, 40.0),
    }]);
    assert_eq!(missed, untouched, "a cut that misses must change nothing");
}
