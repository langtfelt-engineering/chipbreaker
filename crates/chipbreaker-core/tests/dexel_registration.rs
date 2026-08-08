// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The three bundles share one corner lattice, and endpoints carry normals.
//!
//! # Why registration is now an invariant
//!
//! It was once recorded that the three bundles need not be co-registered. That
//! was wrong, and extraction is where the debt came due. Dual contouring needs one grid
//! whose **corners** are ray positions, because the three bundles *are* that
//! grid's three edge directions: an X-directed edge from `(x_i, y_j, z_k)` to
//! `(x_{i+1}, y_j, z_k)` must be a sub-segment of the X-bundle ray at transverse
//! `(y_j, z_k)`. If the X-bundle and the Z-bundle disagreed about where `y_j`
//! is, that edge would belong to no ray and the cell would have an uncovered
//! edge.
//!
//! It already holds, and to the bit, because the centring computes `pad`
//! from the axis extent and the spacing alone — both shared across bundles — so
//! two bundles reach a shared ordinate by identical arithmetic on identical
//! inputs. These tests turn that from a happy consequence into a checked
//! property, ahead of any adaptive subdivision, which is the change most
//! likely to break it.
//!
//! **The half-cell offset is not violated.** The DC grid sits half a cell from
//! the dexel cell grid: dexel cell *centres* are DC grid *corners*. The rays are
//! exactly where they always were; only the grid we name has moved.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::{Axis, Vec3};
use chipbreaker_core::mesh::shapes;

fn build(size: Vec3, spacing: f64) -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), size),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

/// Stock sizes and spacings chosen so `pad` is zero in some and awkward in
/// others — a formula that only registered when the spacing divided the extent
/// would pass on the first row and fail on the last.
const CASES: [(&str, [f64; 3], f64); 6] = [
    ("divides exactly", [40.0, 30.0, 10.0], 0.5),
    ("pad non-zero on all three", [20.0, 20.0, 20.0], 1.6),
    ("awkward spacing", [40.0, 30.0, 10.0], 0.7),
    ("plate", [100.0, 60.0, 20.0], 0.4),
    ("irrational-ish", [37.3, 11.9, 5.1], 0.31),
    ("very thin in z", [30.0, 30.0, 1.05], 0.25),
];

#[test]
fn every_build_registers_its_three_bundles() {
    for (name, size, spacing) in CASES {
        let field = build(Vec3::new(size[0], size[1], size[2]), spacing);
        field
            .check_registration()
            .unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

#[test]
fn the_shared_corner_ordinates_agree_bit_for_bit() {
    // Stronger than `check_registration` reports, and the reason it can afford
    // to compare bits rather than use a tolerance: the two bundles compute the
    // same ordinate from the same inputs by the same arithmetic, so anything
    // other than 0 ULP means they were derived differently.
    for (name, size, spacing) in CASES {
        let field = build(Vec3::new(size[0], size[1], size[2]), spacing);
        for world in AXES {
            let mut seen: Vec<(Axis, Vec<f64>)> = Vec::new();
            for bundle_axis in AXES {
                let Some(bundle) = field.bundle(bundle_axis) else {
                    continue;
                };
                let lattice = bundle.lattice();
                let [u, v, _] = bundle_axis.cyclic();
                let which = if u == world.index() {
                    0
                } else if v == world.index() {
                    1
                } else {
                    continue;
                };
                let n = lattice.counts()[which];
                seen.push((
                    bundle_axis,
                    (0..n)
                        .map(|k| {
                            let (i, j) = if which == 0 { (k, 0) } else { (0, k) };
                            lattice.origin_of(i, j).to_array()[world.index()]
                        })
                        .collect(),
                ));
            }
            assert_eq!(
                seen.len(),
                2,
                "{name}/{}: exactly two bundles should have a transverse \
                 coordinate along any world axis",
                world.as_str()
            );
            let (first_axis, first) = &seen[0];
            let (other_axis, other) = &seen[1];
            assert_eq!(first.len(), other.len(), "{name}/{}: count", world.as_str());
            for (k, (a, b)) in first.iter().zip(other.iter()).enumerate() {
                assert_eq!(
                    a.to_bits(),
                    b.to_bits(),
                    "{name}: corner {k} along {} is {a} in bundle {} and {b} in \
                     bundle {}",
                    world.as_str(),
                    first_axis.as_str(),
                    other_axis.as_str()
                );
            }
        }
    }
}

#[test]
fn corner_coordinates_are_strictly_inside_the_workspace_and_ascending() {
    // The property the centring exists to guarantee, restated for the grid
    // extraction builds on: no corner lands on a stock face, where a ray would be
    // tangent to the surface and the crossing count ambiguous.
    for (name, size, spacing) in CASES {
        let field = build(Vec3::new(size[0], size[1], size[2]), spacing);
        for world in AXES {
            let coords = field
                .corner_coordinates(world)
                .unwrap_or_else(|| panic!("{name}: no coordinates along {}", world.as_str()));
            assert!(coords.len() >= 2, "{name}: too few corners");
            let (lo, hi) = (0.0, size[world.index()]);
            for w in coords.windows(2) {
                assert!(w[1] > w[0], "{name}: corners are not ascending");
            }
            assert!(
                coords[0] > lo && *coords.last().expect("non-empty") < hi,
                "{name}/{}: corners {} .. {} escape the workspace {lo} .. {hi}",
                world.as_str(),
                coords[0],
                coords.last().expect("non-empty")
            );
        }
    }
}

#[test]
fn the_dc_grid_is_offset_half_a_cell_from_the_dexel_grid() {
    // Documents the relabelling rather than asserting a new fact, so that
    // someone who remembers the "origins are never on the integer lattice"
    // does not read the offset as a violation of it.
    let field = build(Vec3::new(40.0, 30.0, 10.0), 0.5);
    let coords = field.corner_coordinates(Axis::X).expect("x corners");
    // Spacing divides the extent, so pad is zero and the first corner sits at
    // exactly half a cell.
    assert!(
        (coords[0] - 0.25).abs() < 1.0e-12,
        "first X corner at {}, expected half a cell in",
        coords[0]
    );
    for w in coords.windows(2) {
        assert!(
            (w[1] - w[0] - 0.5).abs() < 1.0e-12,
            "corner spacing is not the cell size"
        );
    }
}

#[test]
fn construction_records_a_real_normal_at_every_endpoint() {
    // A box has six faces, all axis-aligned, so every endpoint's normal must be
    // exactly one of the six axis directions. Anything else means the triangle
    // normal was not what reached the span.
    //
    // **This deliberately does not count placeholders**, because it cannot. The
    // placeholder is `(0, 0)`, which is a genuine `+Z`, and the encoding has no
    // reserved pattern by design — reserving one would cost a real direction.
    // A first draft asserted zero placeholders and found 4,800: exactly the
    // count of `+Z`-facing far endpoints on the Z bundle, every one of them
    // correct. The count was measuring the encoding's own ambiguity, not a
    // defect.
    //
    // What can be checked is that every normal is one of the six axis
    // directions, which a placeholder left over from an unpopulated endpoint
    // would also satisfy — so the real guard against that is the sign-convention
    // test below, where a missing normal on a `-Z` face shows up immediately.
    let field = build(Vec3::new(40.0, 30.0, 10.0), 0.5);
    let mut checked = 0u64;
    for axis in AXES {
        let bundle = field.bundle(axis).expect("built");
        let rays = u32::try_from(bundle.arena().rays()).expect("small");
        for ray in 0..rays {
            for span in bundle.arena().get(ray) {
                for n in [span.n0, span.n1] {
                    let d = n.decode();
                    let best = d.x.abs().max(d.y.abs()).max(d.z.abs());
                    assert!(
                        best > 0.999,
                        "a box face produced an off-axis normal {d:?} on bundle {}",
                        axis.as_str()
                    );
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 1000, "too few endpoints examined: {checked}");
}

#[test]
fn the_normals_on_a_box_point_out_of_the_material() {
    // The sign convention, on the one shape where it is unambiguous. Along a Z
    // ray through a box, the near endpoint's normal must point -Z and the far
    // one +Z: out of the solid at each end, not along the ray.
    let field = build(Vec3::new(40.0, 30.0, 10.0), 0.5);
    let bundle = field.bundle(Axis::Z).expect("z bundle");
    let rays = u32::try_from(bundle.arena().rays()).expect("small");
    let mut seen = 0u64;
    for ray in 0..rays {
        let spans = bundle.arena().get(ray);
        if spans.len() != 1 {
            continue;
        }
        let s = spans[0];
        let (near, far) = (s.n0.decode(), s.n1.decode());
        assert!(
            near.z < -0.999,
            "the near face of a Z ray must point -Z, got {near:?}. If this is +Z \
             the mesh will be inside out."
        );
        assert!(far.z > 0.999, "the far face must point +Z, got {far:?}");
        seen += 1;
    }
    assert!(seen > 1000, "too few single-span rays examined: {seen}");
}
