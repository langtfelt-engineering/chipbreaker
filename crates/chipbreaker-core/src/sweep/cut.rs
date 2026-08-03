// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Applying a swept volume to a field.
//!
//! # The rejection test is the performance story
//!
//! A finishing segment touches a vanishing fraction of a four-million-ray field.
//! What decides whether a 500,000-segment job takes minutes or days is not the
//! inner loop but how cheaply the other 99.99% of rays are skipped, so the
//! transverse box test comes first and everything else is behind it.
//!
//! # Order is contract
//!
//! Rays are visited in ascending index within a bundle, and bundles in
//! `AXES` order. Nothing here depends on the order — each ray's subtraction is
//! independent — but the **statistics** accumulate in it, and a float that
//! accumulates in a different order is a different float. Unit 11 will have to
//! respect that when it parallelises.

use crate::dexel::tri::{AXES, TriDexelField};
use crate::dexel::{Arena, DexelField};
use crate::math::{Aabb3, Axis, Ray};
use crate::spans::Spans;
use crate::tool::Profile;
use crate::tool::raycast::{RaycastScratch, RaycastStats};

use super::{LinearMove, SweepCase, horizontal, plunge, reference, spans_in_tool_at};

/// How a swept volume should be computed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepMethod {
    /// Dense sub-stepping with a fixed step count.
    ///
    /// The ground truth of [`reference`]. Slow and obviously correct.
    Reference {
        /// Sub-steps per motion.
        steps: u32,
    },
    /// Closed form where one exists, bounded sub-stepping otherwise.
    ///
    /// The shipping method. Horizontal moves take the three-piece
    /// decomposition and stationary ones the static tool, both exact; a general
    /// ramp falls back to sub-stepping with a computed bound. Unit 7 measured
    /// ramps at 0.75% of linear segments, so the fallback is rare and the cases
    /// that are not rare are exact.
    Analytic {
        /// Deviation tolerance for the moves that still need sub-stepping.
        tolerance: f64,
    },
    /// Sub-stepping refined until the deviation bound falls under a tolerance.
    ///
    /// The bound is computed per motion from its length, so a short move costs
    /// few steps and a long one costs many — which a fixed count cannot do.
    Bounded {
        /// Maximum deviation between the true sweep and the stepped
        /// approximation, in millimetres.
        tolerance: f64,
    },
}

impl SweepMethod {
    /// Steps and the deviation bound achieved, for one motion.
    #[must_use]
    pub fn plan(self, motion: &LinearMove) -> (u32, f64) {
        match self {
            Self::Reference { steps } => {
                let steps = steps.max(1);
                let distance = motion.delta().length();
                (steps, distance / (2.0 * f64::from(steps)))
            }
            Self::Bounded { tolerance } => reference::substeps_for_error(motion, tolerance),
            Self::Analytic { tolerance } => match motion.case() {
                // Exact: no sub-stepping, so no deviation at all.
                SweepCase::Stationary | SweepCase::Horizontal => (0, 0.0),
                // A plunge is usually exact too, but only for an axis ray
                // against a radially convex profile, and neither is known here.
                // The plan is the fallback's; `cut_bundle` records zero
                // sub-steps for the rays that took the exact path.
                SweepCase::Plunge | SweepCase::Ramp => {
                    reference::substeps_for_error(motion, tolerance)
                }
            },
        }
    }
}

/// What one cut did.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CutStats {
    /// Rays examined after the box rejection.
    pub rays_tested: u64,
    /// Rays skipped by the box rejection.
    pub rays_rejected: u64,
    /// Rays whose material actually changed.
    pub rays_changed: u64,
    /// Material removed, in cubic millimetres, per bundle in `AXES` order.
    ///
    /// Per bundle and never averaged: the three disagree at `O(h)` and
    /// reconciling them is Unit 9's job.
    pub removed_mm3: [f64; 3],
    /// Sub-steps used, summed over motions and bundles.
    pub substeps: u64,
    /// Worst deviation bound accepted across the motions in this cut.
    ///
    /// The number that makes a step count mean something. Reported beside it
    /// everywhere, because "we used 64 sub-steps" guarantees nothing.
    pub worst_bound_mm: f64,
    /// Predicate counters from the ray casts.
    pub raycast: RaycastStats,
}

impl CutStats {
    /// Fraction of rays the box test skipped, in `[0, 1]`.
    #[must_use]
    pub fn rejection_rate(&self) -> f64 {
        let total = self.rays_tested + self.rays_rejected;
        if total == 0 {
            0.0
        } else {
            #[allow(clippy::cast_precision_loss, reason = "a ratio of counts")]
            {
                self.rays_rejected as f64 / total as f64
            }
        }
    }

    /// Accumulates another cut's statistics.
    pub fn merge(&mut self, other: &Self) {
        self.rays_tested += other.rays_tested;
        self.rays_rejected += other.rays_rejected;
        self.rays_changed += other.rays_changed;
        for (a, b) in self.removed_mm3.iter_mut().zip(other.removed_mm3) {
            *a += b;
        }
        self.substeps += other.substeps;
        self.worst_bound_mm = self.worst_bound_mm.max(other.worst_bound_mm);
        self.raycast.merge(&other.raycast);
    }
}

/// Reusable buffers for cutting, so the inner loop allocates nothing.
#[derive(Debug, Default)]
pub struct CutScratch {
    raycast: RaycastScratch,
    swept: Spans,
    material: Spans,
    result: Spans,
}

impl CutScratch {
    /// Buffers sized for a profile.
    #[must_use]
    pub fn new(profile: &Profile) -> Self {
        Self {
            raycast: RaycastScratch::with_capacity(profile.len()),
            swept: Spans::new(),
            material: Spans::new(),
            result: Spans::new(),
        }
    }
}

/// Subtracts a swept tool from every bundle of a field.
///
/// Each bundle is cut independently and never compared, per the Unit 7 contract.
pub fn cut_tri(
    field: &mut TriDexelField,
    profile: &Profile,
    motion: &LinearMove,
    method: SweepMethod,
    scratch: &mut CutScratch,
) -> CutStats {
    let mut total = CutStats::default();
    for axis in AXES {
        let Some(bundle) = field.bundle_mut(axis) else {
            continue;
        };
        let stats = cut_bundle(bundle, profile, motion, method, scratch);
        total.rays_tested += stats.rays_tested;
        total.rays_rejected += stats.rays_rejected;
        total.rays_changed += stats.rays_changed;
        total.removed_mm3[axis.index()] += stats.removed_mm3[axis.index()];
        total.substeps += stats.substeps;
        total.worst_bound_mm = total.worst_bound_mm.max(stats.worst_bound_mm);
        total.raycast.merge(&stats.raycast);
    }
    total
}

/// Subtracts a swept tool from one bundle.
///
/// The removed volume is recorded in the slot for this bundle's own axis and
/// nowhere else.
pub fn cut_bundle(
    bundle: &mut DexelField,
    profile: &Profile,
    motion: &LinearMove,
    method: SweepMethod,
    scratch: &mut CutScratch,
) -> CutStats {
    let mut stats = CutStats::default();
    let (steps, bound) = method.plan(motion);
    // Set only when a ray actually sub-steps. A cut that took the exact path on
    // every ray reports no deviation, which is the truth and is what makes the
    // number worth printing.
    let planned_bound = bound;

    let lattice = bundle.lattice().clone();
    let axis = lattice.axis();
    let bounds = motion.swept_bounds(profile);
    let cell_area = lattice.cell_area();
    let slot = axis.index();

    let rays = u32::try_from(bundle.arena().rays()).unwrap_or(u32::MAX);
    // Ascending ray index. Independent per ray, but the statistics accumulate in
    // this order and a float summed differently is a different float.
    for ray_index in 0..rays {
        let (i, j) = lattice.coords(ray_index);
        let origin = lattice.origin_of(i, j);

        // The rejection that decides the runtime. A ray whose transverse
        // position is outside the swept box cannot meet the tool anywhere along
        // its length, so no cast is needed and no span is touched.
        if !transverse_overlaps(&bounds, axis, origin, lattice.spacing()) {
            stats.rays_rejected += 1;
            continue;
        }
        // Nothing to remove from an empty ray, and checking is far cheaper than
        // casting. This matters after the first few passes, when a large part of
        // the field is already air.
        if bundle.arena().span_count(ray_index) == 0 {
            stats.rays_rejected += 1;
            continue;
        }
        stats.rays_tested += 1;

        let ray = Ray {
            origin,
            direction: axis.direction(),
        };
        match method {
            SweepMethod::Analytic { .. } if matches!(motion.case(), SweepCase::Horizontal) => {
                horizontal::swept_spans_into(
                    profile,
                    motion,
                    &ray,
                    &mut scratch.raycast,
                    &mut scratch.swept,
                    &mut stats.raycast,
                );
            }
            SweepMethod::Analytic { .. } if matches!(motion.case(), SweepCase::Stationary) => {
                spans_in_tool_at(
                    profile,
                    motion.start,
                    &ray,
                    &mut scratch.raycast,
                    &mut scratch.swept,
                    &mut stats.raycast,
                );
            }
            // A plunge is exact only for an axis ray against a radially convex
            // profile. `swept_spans_into` says whether it took it, and anything
            // it declines falls through to sub-stepping rather than being
            // guessed at.
            SweepMethod::Analytic { .. }
                if matches!(motion.case(), SweepCase::Plunge)
                    && plunge::swept_spans_into(
                        profile,
                        motion,
                        &ray,
                        &mut scratch.raycast,
                        &mut scratch.swept,
                        &mut stats.raycast,
                    ) => {}
            _ => {
                reference::swept_spans_into(
                    profile,
                    motion,
                    steps.max(1),
                    &ray,
                    &mut scratch.raycast,
                    &mut scratch.swept,
                    &mut stats.raycast,
                );
                stats.substeps += u64::from(steps);
                stats.worst_bound_mm = stats.worst_bound_mm.max(planned_bound);
            }
        }
        if scratch.swept.is_empty() {
            continue;
        }

        bundle.arena().read_into(ray_index, &mut scratch.material);
        let before = scratch.material.measure();
        scratch
            .material
            .subtract_into(&scratch.swept, &mut scratch.result);
        let after = scratch.result.measure();
        if after == before && scratch.result.as_slice() == scratch.material.as_slice() {
            continue;
        }
        stats.rays_changed += 1;
        stats.removed_mm3[slot] += (before - after) * cell_area;
        bundle.arena_mut().set(ray_index, scratch.result.as_slice());
    }
    stats
}

/// True if a ray's cell could meet the swept box.
///
/// Tests the ray's transverse position against the box, widened by half a cell
/// so a ray whose cell straddles the boundary is not dropped. Along the ray axis
/// nothing is tested: a ray that overlaps transversely may meet the tool
/// anywhere on its length, and the span arithmetic will find out.
fn transverse_overlaps(
    bounds: &Aabb3,
    axis: Axis,
    origin: crate::math::Vec3,
    spacing: f64,
) -> bool {
    let [u, v, _] = axis.cyclic();
    let p = origin.to_array();
    let lo = bounds.min.to_array();
    let hi = bounds.max.to_array();
    let slack = 0.5 * spacing;
    for k in [u, v] {
        if p[k] < lo[k] - slack || p[k] > hi[k] + slack {
            return false;
        }
    }
    true
}

/// Span-count histogram across every bundle, for the arena measurement.
///
/// Unit 5 sized `INLINE_CAPACITY` on stock at rest, where the distribution is
/// nearly degenerate. Cutting splits spans, so the number that matters is this
/// one taken **after** a cut.
#[must_use]
pub fn distribution(field: &TriDexelField) -> std::collections::BTreeMap<usize, usize> {
    let mut out = std::collections::BTreeMap::new();
    for (_, bundle) in field.bundles() {
        for (spans, rays) in bundle.arena().distribution() {
            *out.entry(spans).or_default() += rays;
        }
    }
    out
}

/// Spilled rays across every bundle.
#[must_use]
pub fn spilled(field: &TriDexelField) -> usize {
    field.bundles().map(|(_, b)| b.arena().spilled_rays()).sum()
}

/// Total bytes of span storage, so growth under cutting is visible.
#[must_use]
pub fn bytes(field: &TriDexelField) -> usize {
    field.bundles().map(|(_, b)| b.arena().bytes()).sum()
}

/// The arena a bundle holds, for callers that only need to look.
#[must_use]
pub fn arena_of(bundle: &DexelField) -> &Arena {
    bundle.arena()
}
