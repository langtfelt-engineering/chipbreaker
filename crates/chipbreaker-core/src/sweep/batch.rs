// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Cutting several motions in one traversal.
//!
//! # What batching buys
//!
//! Unit 7 measured 84.2% of rays rejected by the box test on a raster job. The
//! remaining cost is not the inner loop but re-walking the same rays for every
//! consecutive segment: a finishing raster spends its life in a narrow band, so
//! a batch of 32 or 64 moves rejects nearly as well as one move while paying the
//! traversal once.
//!
//! # It must not change the answer, and the loop order is why that is delicate
//!
//! Unbatched is motion-major: for each motion, walk every ray. Batched is
//! ray-major: for each ray, walk every motion. The **field** is identical either
//! way, because each ray's subtractions still happen in motion order and rays do
//! not interact.
//!
//! The **statistics** are not automatically identical, and that is the trap.
//! `removed_mm3` is a sum of floats, and a sum reordered is a different sum. So
//! removed volume is accumulated into a **per-motion** slot and summed only at
//! the end, in motion order — which reproduces the unbatched order exactly,
//! because within a motion the rays still arrive ascending in both schemes.
//!
//! ## Per-motion slots are necessary and not sufficient
//!
//! The first version of this module did exactly that and still disagreed with
//! the unbatched path by one ULP. Per-motion slots fix the order *within* a
//! batch; they do nothing about the **boundary between** batches. Unbatched
//! computes `((m1 + m2) + m3) + m4`. Batched in pairs computes
//! `(m1 + m2) + (m3 + m4)` — same order, different **grouping**, and floating
//! point addition is not associative.
//!
//! So the per-motion slots run the length of the whole motion list, not of one
//! batch, and [`cut_all`] sums them once at the very end. That is why `cut_all`
//! is the entry point and [`cut_batch`] is not: a caller that chunks the list
//! itself and adds up the chunk totals reintroduces the grouping, and gets a
//! reported volume that depends on the batch size. Which is a tuning knob, and
//! must be invisible.
//!
//! # One tool per batch
//!
//! A batch carries one profile. Splitting at a tool change is the caller's job
//! and [`split_runs`] does it, because a batch spanning two cutters would need a
//! scratch per tool and the sharing is not worth the tangle.

use crate::dexel::DexelField;
use crate::dexel::tri::{AXES, TriDexelField};
use crate::math::{Aabb3, Ray};
use crate::tool::Profile;

use super::Motion;
use super::cut::{CutScratch, CutStats, SweepMethod, cut_one_ray, transverse_overlaps};

/// Default motions per batch.
///
/// Measured, on 4,000 short segments of a posted finishing pass over a
/// 100 x 60 x 20 mm field at 0.5 mm — the workload batching exists for:
///
/// ```text
/// size      1       4      16      64     256    1024
/// time   1024ms  662ms   510ms   495ms   471ms   538ms
/// gain    1.00x  1.55x   2.01x   2.07x   2.17x   1.90x
/// ```
///
/// The turnover at 1024 is the union swelling: a batch rejects a ray against
/// the union of its boxes, so once a batch spans more ground than its segments
/// work over, the rejection that makes the field affordable stops working. The
/// curve is flat from 16 to 256 and 32 sits in the middle of that plateau,
/// which is why it is the default rather than the 256 that measured fastest —
/// the extra 5% is not worth being one workload away from the cliff.
///
/// Long full-width raster passes show none of this: each move's box already
/// spans the part, so the union of thirty-two is barely larger than one and the
/// speedup is inside the noise. Measuring those first was a mistake worth
/// recording, because it makes the shape of the win precise — batching pays for
/// **many small boxes**, not for many moves.
pub const DEFAULT_BATCH: usize = 32;

/// Cuts a whole motion list, batching internally.
///
/// **The entry point.** Produces a bit-identical field *and* bit-identical
/// statistics to cutting the same motions one at a time, for every `size`. The
/// caller must not chunk the list itself and add up the totals — see the module
/// header for the ULP that costs.
///
/// All motions share `profile`; group by tool before calling.
pub fn cut_all(
    field: &mut TriDexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    scratch: &mut CutScratch,
    size: usize,
) -> CutStats {
    let mut total = CutStats::default();
    if motions.is_empty() {
        return total;
    }
    // One slot per motion per bundle, for the whole list. Summed once, at the
    // end, in motion order.
    let mut removed = vec![[0.0f64; 3]; motions.len()];
    let tools = vec![0u32; motions.len()];
    for (lo, hi) in split_runs(motions, &tools, size) {
        let stats = cut_batch_per_motion(
            field,
            profile,
            &motions[lo..hi],
            method,
            scratch,
            &mut removed[lo..hi],
        );
        total.rays_tested += stats.rays_tested;
        total.rays_rejected += stats.rays_rejected;
        total.rays_changed += stats.rays_changed;
        total.substeps += stats.substeps;
        total.rays_exact += stats.rays_exact;
        total.rays_substepped += stats.rays_substepped;
        total.worst_bound_mm = total.worst_bound_mm.max(stats.worst_bound_mm);
        total.raycast.merge(&stats.raycast);
    }
    for slot in 0..3 {
        for per_motion in &removed {
            total.removed_mm3[slot] += per_motion[slot];
        }
    }
    total
}

/// Cuts one run of motions with one traversal per bundle.
///
/// The field is bit-identical to the unbatched result. `removed_mm3` is this
/// run's own flat sum, which is **not** additive across runs — use [`cut_all`]
/// unless you are accumulating per-motion yourself.
pub fn cut_batch(
    field: &mut TriDexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    scratch: &mut CutScratch,
) -> CutStats {
    let mut removed = vec![[0.0f64; 3]; motions.len()];
    let mut total = cut_batch_per_motion(field, profile, motions, method, scratch, &mut removed);
    for slot in 0..3 {
        for per_motion in &removed {
            total.removed_mm3[slot] += per_motion[slot];
        }
    }
    total
}

/// Cuts one run, writing removed volume **per motion** instead of summing it.
///
/// For a caller that chunks a longer list itself: accumulate `removed` across
/// every chunk and sum it once at the end, in motion order. Summing each chunk's
/// total instead reintroduces the grouping the module header warns about, and
/// the reported volume then depends on the batch size.
///
/// `removed` must be one entry per motion, indexed to match `motions`.
///
/// # Panics
/// Panics if `removed` is not the same length as `motions`.
pub fn cut_batch_per_motion(
    field: &mut TriDexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    scratch: &mut CutScratch,
    removed: &mut [[f64; 3]],
) -> CutStats {
    assert_eq!(
        motions.len(),
        removed.len(),
        "one removed-volume slot per motion"
    );
    let mut total = CutStats::default();
    for axis in AXES {
        let Some(bundle) = field.bundle_mut(axis) else {
            continue;
        };
        let stats = cut_batch_bundle(bundle, profile, motions, method, scratch, removed);
        total.rays_tested += stats.rays_tested;
        total.rays_rejected += stats.rays_rejected;
        total.rays_changed += stats.rays_changed;
        total.substeps += stats.substeps;
        total.rays_exact += stats.rays_exact;
        total.rays_substepped += stats.rays_substepped;
        total.worst_bound_mm = total.worst_bound_mm.max(stats.worst_bound_mm);
        total.raycast.merge(&stats.raycast);
    }
    total
}

/// One bundle's share of a batch.
fn cut_batch_bundle(
    bundle: &mut DexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    scratch: &mut CutScratch,
    removed: &mut [[f64; 3]],
) -> CutStats {
    let mut stats = CutStats::default();
    if motions.is_empty() {
        return stats;
    }

    let lattice = bundle.lattice().clone();
    let axis = lattice.axis();
    let slot = axis.index();
    let cell_area = lattice.cell_area();

    // Per-motion boxes, and their union. The union is tested once per ray
    // instead of once per motion, which is the whole saving; the individual
    // boxes still reject a ray from the motions it cannot meet.
    let boxes: Vec<Aabb3> = motions.iter().map(|m| m.swept_bounds(profile)).collect();
    let union = boxes.iter().skip(1).fold(boxes[0], |acc, b| acc.union(b));

    let rays = u32::try_from(bundle.arena().rays()).unwrap_or(u32::MAX);
    for ray_index in 0..rays {
        let (i, j) = lattice.coords(ray_index);
        let origin = lattice.origin_of(i, j);
        if !transverse_overlaps(&union, axis, origin, lattice.spacing()) {
            // Rejected against the whole batch at once, which is the point.
            stats.rays_rejected += motions.len() as u64;
            continue;
        }
        let ray = Ray {
            origin,
            direction: axis.direction(),
        };

        for (index, motion) in motions.iter().enumerate() {
            if !transverse_overlaps(&boxes[index], axis, origin, lattice.spacing())
                || bundle.arena().span_count(ray_index) == 0
            {
                stats.rays_rejected += 1;
                continue;
            }
            stats.rays_tested += 1;
            let cut = cut_one_ray(
                bundle, profile, motion, method, scratch, ray_index, &ray, cell_area, &mut stats,
            );
            if cut > 0.0 {
                // Into this motion's own slot, never into a running total. See
                // the module header.
                removed[index][slot] += cut;
                stats.rays_changed += 1;
            }
        }
    }
    stats
}

/// Splits motions into runs of at most `size` that share a tool and a kind.
///
/// A run never spans a tool change, because a batch carries one profile. It also
/// never mixes exact motions with sub-stepped ones, so a reported deviation
/// bound still belongs to the motions that earned it rather than to a batch.
///
/// Returns half-open `(start, end)` ranges.
///
/// # Panics
/// Panics if `tools` is not the same length as `motions`.
#[must_use]
pub fn split_runs(motions: &[Motion], tools: &[u32], size: usize) -> Vec<(usize, usize)> {
    assert_eq!(
        motions.len(),
        tools.len(),
        "every motion needs the tool it was programmed with"
    );
    let size = size.max(1);
    let mut out = Vec::new();
    let mut start = 0usize;
    while start < motions.len() {
        let tool = tools[start];
        let exact = motions[start].case().is_exact();
        let limit = (start + size).min(motions.len());
        let mut end = start + 1;
        while end < limit && tools[end] == tool && motions[end].case().is_exact() == exact {
            end += 1;
        }
        out.push((start, end));
        start = end;
    }
    out
}
