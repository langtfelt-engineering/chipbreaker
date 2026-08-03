// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The lattice invariant and the arena's contract.
//!
//! No field construction here — that is Increment B. These are the two
//! structures a field is made of, tested apart from it.

use chipbreaker_core::dexel::{Arena, INLINE_CAPACITY, Lattice, LatticeError};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Aabb3, Axis, Vec3};
use chipbreaker_core::spans::{Span, Spans};

fn workspace() -> Aabb3 {
    Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 60.0, 25.0))
}

// --- the invariant ---------------------------------------------------------

#[test]
fn origins_are_never_on_the_integer_lattice() {
    // **This test is the enforcement.** ADR 0001 Part 2 makes the half-cell
    // offset a required invariant rather than a tuning parameter, for two
    // reasons at once: 2.52 ms against 39.83 ms, and keeping coplanar
    // degeneracy unreachable for axis-aligned stock.
    //
    // Somebody will eventually simplify `min + (i + 0.5) * spacing` to
    // `min + i * spacing`, because the second is tidier and the reason for the
    // first is two documents away. Deleting the `+ 0.5` fails here.
    //
    // The bounds and spacing are chosen so that a corner-based lattice would
    // land on integers: origin at zero, spacing exactly 1 mm.
    let lattice = Lattice::new(
        Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 20.0)),
        1.0,
        Axis::Z,
    )
    .expect("valid");

    let [nx, ny] = lattice.counts();
    assert!(nx > 1 && ny > 1);
    for i in 0..nx {
        for j in 0..ny {
            let origin = lattice.origin_of(i, j).to_array();
            for (axis, value) in origin.iter().enumerate().take(2) {
                assert!(
                    (value - value.round()).abs() > 1e-12,
                    "ray ({i}, {j}) origin axis {axis} is at {value}, an integer coordinate. \
                     A stock mesh with integer vertices would then be coplanar with the ray, \
                     which construction treats as a hard error. The `+ 0.5` in \
                     Lattice::origin_of is load-bearing."
                );
            }
        }
    }
}

#[test]
fn origins_are_exactly_half_a_cell_in_from_the_corner() {
    let lattice = Lattice::new(workspace(), 0.5, Axis::Z).expect("valid");
    let first = lattice.origin_of(0, 0);
    assert!((first.x - 0.25).abs() < 1e-15, "{first:?}");
    assert!((first.y - 0.25).abs() < 1e-15, "{first:?}");
    // Rays start behind the workspace so a surface exactly on the lower bound
    // is crossed rather than begun upon.
    assert!(first.z < 0.0, "{first:?}");
}

// --- lattice ---------------------------------------------------------------

#[test]
fn counts_cover_the_workspace() {
    let lattice = Lattice::new(workspace(), 0.5, Axis::Z).expect("valid");
    assert_eq!(lattice.counts(), [200, 120]);
    assert_eq!(lattice.ray_count(), 24_000);
    assert!((lattice.cell_area() - 0.25).abs() < 1e-15);
}

#[test]
fn ray_indices_round_trip_through_lattice_coordinates() {
    let lattice = Lattice::new(workspace(), 2.0, Axis::Z).expect("valid");
    for ray in 0..u32::try_from(lattice.ray_count()).expect("small") {
        let (i, j) = lattice.coords(ray);
        assert_eq!(lattice.index(i, j), ray);
    }
}

#[test]
fn each_axis_puts_its_rays_along_the_right_direction() {
    for (axis, expected) in [
        (Axis::X, Vec3::new(1.0, 0.0, 0.0)),
        (Axis::Y, Vec3::new(0.0, 1.0, 0.0)),
        (Axis::Z, Vec3::new(0.0, 0.0, 1.0)),
    ] {
        let lattice = Lattice::new(workspace(), 5.0, axis).expect("valid");
        assert_eq!(lattice.ray_at(0).direction, expected);
        // And the lattice spans the other two axes, so the ray's own axis is
        // the one that does not vary between neighbouring rays.
        let [_, _, w] = axis.cyclic();
        let a = lattice.ray_at(0).origin.to_array();
        let b = lattice.ray_at(1).origin.to_array();
        assert!(
            (a[w] - b[w]).abs() < 1e-15,
            "{axis:?}: neighbouring rays must share the ray-axis coordinate"
        );
    }
}

#[test]
fn a_lattice_that_cannot_be_addressed_is_refused_rather_than_truncated() {
    let huge = Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0e6, 1.0e6, 10.0));
    match Lattice::new(huge, 0.01, Axis::Z) {
        Err(LatticeError::TooManyRays { wanted, .. }) => {
            assert!(wanted > u64::from(u32::MAX));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_bad_spacing_or_bounds_is_refused() {
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            Lattice::new(workspace(), bad, Axis::Z),
            Err(LatticeError::BadSpacing { .. })
        ));
    }
    assert!(matches!(
        Lattice::new(Aabb3::EMPTY, 0.5, Axis::Z),
        Err(LatticeError::BadBounds { .. })
    ));
}

#[test]
fn lattice_hashing_reflects_every_field() {
    let digest = |l: &Lattice| {
        let mut h = CanonicalHash::new();
        h.add(l);
        h.finish().to_hex()
    };
    let base = Lattice::new(workspace(), 0.5, Axis::Z).expect("valid");
    assert_eq!(digest(&base), digest(&base));
    assert_ne!(
        digest(&base),
        digest(&Lattice::new(workspace(), 0.25, Axis::Z).expect("valid"))
    );
    assert_ne!(
        digest(&base),
        digest(&Lattice::new(workspace(), 0.5, Axis::X).expect("valid"))
    );
}

// --- arena -----------------------------------------------------------------

fn span(a: f64, b: f64) -> Span {
    Span::ordered(a, b)
}

#[test]
fn a_new_arena_is_every_ray_empty() {
    let arena = Arena::new(100);
    assert_eq!(arena.rays(), 100);
    assert_eq!(arena.total_spans(), 0);
    assert_eq!(arena.filled_rays(), 0);
    assert_eq!(arena.spilled_rays(), 0);
    for ray in 0..100u32 {
        assert!(arena.get(ray).is_empty());
    }
}

#[test]
fn spans_within_the_inline_capacity_never_spill() {
    // The measured common case: one span on every ray, and two on a ray through
    // a cavity. Neither should touch the spill map.
    let mut arena = Arena::new(10);
    for ray in 0..10u32 {
        arena.set(ray, &[span(0.0, f64::from(ray) + 1.0)]);
    }
    assert_eq!(arena.spilled_rays(), 0, "one span must never spill");

    arena.set(3, &[span(0.0, 1.0), span(2.0, 3.0)]);
    assert_eq!(arena.spilled_rays(), 0, "two spans must never spill");
    assert_eq!(arena.span_count(3), 2);
    assert_eq!(arena.get(3), &[span(0.0, 1.0), span(2.0, 3.0)]);
}

#[test]
fn spans_beyond_the_inline_capacity_spill_and_read_back_whole() {
    let mut arena = Arena::new(4);
    let many: Vec<Span> = (0..7)
        .map(|k| span(f64::from(k) * 2.0, f64::from(k) * 2.0 + 1.0))
        .collect();
    arena.set(2, &many);
    assert_eq!(arena.spilled_rays(), 1);
    assert_eq!(arena.span_count(2), 7);
    assert_eq!(arena.get(2), many.as_slice());
    // Its neighbours are untouched.
    assert!(arena.get(1).is_empty());
    assert!(arena.get(3).is_empty());
}

#[test]
fn a_ray_that_shrinks_releases_its_spill() {
    // Otherwise the arena only ever grows: U7 subtracts, and a ray that splits
    // and later merges back would keep dead storage alive for the whole run.
    let mut arena = Arena::new(4);
    let many: Vec<Span> = (0..6)
        .map(|k| span(f64::from(k), f64::from(k) + 0.5))
        .collect();
    arena.set(1, &many);
    assert_eq!(arena.spilled_rays(), 1);

    arena.set(1, &[span(0.0, 10.0)]);
    assert_eq!(arena.spilled_rays(), 0, "the spill must be released");
    assert_eq!(arena.get(1), &[span(0.0, 10.0)]);
}

#[test]
fn read_into_reuses_the_callers_buffer() {
    // The shape U7 needs: one scratch buffer for a whole sweep.
    let mut arena = Arena::new(4);
    arena.set(0, &[span(0.0, 1.0), span(3.0, 4.0)]);
    arena.set(1, &[span(5.0, 6.0)]);

    let mut scratch = Spans::new();
    arena.read_into(0, &mut scratch);
    assert_eq!(scratch.len(), 2);
    arena.read_into(1, &mut scratch);
    assert_eq!(scratch.len(), 1, "the buffer is cleared, not appended to");
    arena.read_into(2, &mut scratch);
    assert!(scratch.is_empty());
}

#[test]
fn the_hash_depends_on_contents_and_not_on_history() {
    // Unused inline slots keep whatever was last written there. Hashing the raw
    // backing array would make two fields with identical geometry disagree
    // because one of them had been cut and restored.
    let digest = |a: &Arena| {
        let mut h = CanonicalHash::new();
        h.add(a);
        h.finish().to_hex()
    };

    let mut fresh = Arena::new(8);
    fresh.set(3, &[span(1.0, 2.0)]);

    let mut used = Arena::new(8);
    used.set(3, &[span(90.0, 91.0), span(95.0, 96.0)]);
    used.set(5, &[span(0.0, 1.0)]);
    used.set(5, &[]);
    used.set(3, &[span(1.0, 2.0)]);

    assert_eq!(
        digest(&fresh),
        digest(&used),
        "two arenas holding the same spans must hash identically however they got there"
    );

    // And a spilled ray that shrinks back must match one that never spilled.
    let mut spilled = Arena::new(8);
    spilled.set(
        3,
        &(0..9)
            .map(|k| span(f64::from(k), f64::from(k) + 0.5))
            .collect::<Vec<_>>(),
    );
    spilled.set(3, &[span(1.0, 2.0)]);
    assert_eq!(digest(&fresh), digest(&spilled));
}

#[test]
fn the_distribution_is_reported_for_the_arenas_own_justification() {
    let mut arena = Arena::new(10);
    for ray in 0..6u32 {
        arena.set(ray, &[span(0.0, 1.0)]);
    }
    for ray in 6..8u32 {
        arena.set(ray, &[span(0.0, 1.0), span(2.0, 3.0)]);
    }
    let distribution = arena.distribution();
    assert_eq!(distribution.get(&0), Some(&2));
    assert_eq!(distribution.get(&1), Some(&6));
    assert_eq!(distribution.get(&2), Some(&2));
    assert_eq!(arena.total_spans(), 6 + 4);
    assert_eq!(arena.filled_rays(), 8);
}

#[test]
fn the_inline_capacity_is_two_and_the_reason_is_recorded() {
    // A guard on the constant rather than on behaviour: changing it is allowed,
    // but it should be a deliberate act against a fresh measurement rather than
    // a tidy-up, and this makes the diff show up.
    assert_eq!(
        INLINE_CAPACITY, 2,
        "the measured distribution justifies two; see the arena module header \
         and examples/span_distribution.rs before changing it"
    );
}

#[test]
fn memory_is_proportional_to_rays_and_free_of_per_ray_allocation() {
    // The requirement: no per-ray heap allocation in the steady state. Two
    // arenas of the same size hold the same bytes whatever their contents,
    // until something spills.
    let mut a = Arena::new(1000);
    let b = Arena::new(1000);
    for ray in 0..1000u32 {
        a.set(ray, &[span(0.0, 1.0)]);
    }
    assert_eq!(a.bytes(), b.bytes(), "filling changes no allocation");

    let expected = 1000 * INLINE_CAPACITY * size_of::<Span>() + 1000 * size_of::<u16>();
    assert_eq!(a.bytes(), expected);
}

// --- the U6 amendment ------------------------------------------------------

#[test]
fn a_cell_centre_never_lands_on_the_workspace_boundary() {
    // The bug Unit 6 found, and it had been there since Unit 5.
    //
    // Anchoring cells at `min` puts centre i at `min + (i + 0.5) * h`. A 20 mm
    // box at 1.6 mm cells is 13 cells, and the last centre lands on EXACTLY
    // 20.0 -- the stock's own face. Every ray on it is coplanar with that face,
    // which construction treats as a hard error, so `dexel build` refused a
    // plain box at a perfectly ordinary spacing. Unit 5 never saw it because it
    // only ever cast along Z on meshes whose transverse extents happened not to
    // land that way.
    //
    // Centring the lattice fixes it provably rather than empirically: with
    // pad = (n*h - E)/2 in [0, h/2), the first centre exceeds min and the last
    // falls short of max, on every axis, for every extent and spacing.
    let awkward: [(f64, f64); 5] = [
        (20.0, 1.6), // the original failure: 12.5 cells
        (10.0, 0.8), // 12.5 again
        (7.5, 0.5),  // 15 exactly -- no slack at all
        (30.0, 4.0), // 7.5 cells
        (1.0, 0.3),  // 3.33 cells, large relative slack
    ];
    for (extent, spacing) in awkward {
        let bounds =
            Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(extent, extent, extent));
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            let lattice = Lattice::new(bounds, spacing, axis).expect("valid");
            let [u, v, _] = axis.cyclic();
            let [nx, ny] = lattice.counts();
            for (i, j) in [(0, 0), (nx - 1, ny - 1), (nx / 2, ny / 2)] {
                let origin = lattice.origin_of(i, j).to_array();
                for k in [u, v] {
                    assert!(
                        origin[k] > 0.0 && origin[k] < extent,
                        "extent {extent} at {spacing} mm on {axis:?}: ray ({i}, {j}) has \
                         transverse coordinate {} on axis {k}, which is on or outside the \
                         workspace boundary. A ray there is coplanar with the stock's own \
                         face and construction will refuse to build.",
                        origin[k]
                    );
                }
            }
        }
    }
}

#[test]
fn the_centring_pad_is_under_half_a_cell() {
    // The step the proof turns on. If `pad` could reach h/2, the first centre
    // would land exactly on `min` and the guarantee would be lost.
    for extent in [1.0, 7.5, 10.0, 20.0, 33.3, 99.99] {
        for spacing in [0.1, 0.3, 0.5, 0.8, 1.6, 4.0] {
            let bounds =
                Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(extent, extent, extent));
            let lattice = Lattice::new(bounds, spacing, Axis::Z).expect("valid");
            for pad in lattice.pad() {
                assert!(
                    (0.0..spacing / 2.0).contains(&pad),
                    "extent {extent} at {spacing}: pad {pad} is outside [0, h/2)"
                );
            }
        }
    }
}

#[test]
fn a_box_builds_at_the_spacing_that_used_to_abort() {
    use chipbreaker_core::dexel::{BuildOptions, DexelField};
    let mesh = chipbreaker_core::mesh::shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(30.0, 20.0, 10.0),
    );
    // 1.6 mm divides none of 30, 20 or 10, which is what made a cell centre
    // land on a face. Building at all is the regression this pins.
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let (_field, stats) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing: 1.6,
                axis,
                ..BuildOptions::default()
            },
        )
        .unwrap_or_else(|e| panic!("{axis:?} must build: {e}"));
        assert_eq!(stats.predicates.coplanar_rejected, 0, "{axis:?}");
        assert_eq!(stats.empty_rays, 0, "{axis:?}: every ray meets the box");
    }
}

#[test]
fn a_lattice_that_does_not_divide_the_stock_over_counts_volume_by_a_known_factor() {
    // NOT a defect, and worth a test so nobody "fixes" it into one.
    //
    // Every cell claims a full h^2 of cross-section. When the spacing does not
    // divide the transverse extent, `ceil` gives cells that stick out past the
    // stock -- and because the lattice is centred, their ray centres are still
    // INSIDE the stock, so they report a full chord. The volume is therefore
    // over-counted by exactly (covered area / true area).
    //
    // For a 30x20x10 box at 1.6 mm the Z bundle is 19x13 cells covering
    // 632.32 mm^2 against a true 600, so it reports 6323.2 against 6000 -- a
    // 5.4% bias that is arithmetic, not sampling. It jumps discontinuously
    // whenever `ceil` steps, which is a third reason volume is unfit as an
    // accuracy metric. See ADR 0005.
    use chipbreaker_core::dexel::{BuildOptions, DexelField};
    let mesh = chipbreaker_core::mesh::shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(30.0, 20.0, 10.0),
    );
    let extents = [30.0, 20.0, 10.0];
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let (field, _) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing: 1.6,
                axis,
                ..BuildOptions::default()
            },
        )
        .expect("builds");
        let [u, v, w] = axis.cyclic();
        let [nu, nv] = field.lattice().counts();
        let covered = f64::from(nu) * 1.6 * f64::from(nv) * 1.6;
        let predicted = covered * extents[w];
        assert!(
            (field.volume() - predicted).abs() / predicted < 1e-12,
            "{axis:?}: volume {} is not the covered area {covered} times the depth              {}; the over-count should be exactly explained by cell quantisation",
            field.volume(),
            extents[w]
        );
        // And it really is an over-count, never an under-count.
        let truth = extents[u] * extents[v] * extents[w];
        assert!(field.volume() >= truth, "{axis:?}");
    }
}

#[test]
fn a_lattice_that_divides_the_stock_is_exact() {
    // The other side: when the spacing divides the extents there is no slack,
    // the pad is zero, and the box is captured to machine precision.
    use chipbreaker_core::dexel::{BuildOptions, DexelField};
    let mesh = chipbreaker_core::mesh::shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(30.0, 20.0, 10.0),
    );
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let (field, _) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing: 0.5,
                axis,
                ..BuildOptions::default()
            },
        )
        .expect("builds");
        assert_eq!(field.lattice().pad(), [0.0, 0.0], "{axis:?}");
        let expected = 30.0 * 20.0 * 10.0;
        assert!(
            (field.volume() - expected).abs() / expected < 1e-12,
            "{axis:?}"
        );
    }
}
