// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Cutting on many threads, with the answer unchanged.
//!
//! # The distinction the whole design rests on
//!
//! > **Work assignment may be dynamic. Value combination may not.**
//!
//! Which thread computes a ray does not matter: rays are independent and each
//! result lands in a slot chosen by ray index, never in a shared accumulator.
//! Nondeterminism enters only where results are *combined* in completion order.
//!
//! So the compute phase is a chunked queue with stealing, and the reduction is a
//! separate sequential pass in fixed index order — ascending ray within a
//! motion, ascending motion, which is what ADR 0006 already requires of the
//! batched path. Dynamic balancing and bit-exact reproducibility at once, rather
//! than one at the cost of the other.
//!
//! # The arena is never written during compute
//!
//! Unit 7 established that spill is per bundle and can be sudden — the rib case
//! spilled all 4,500 rays of one bundle at once. A mutex on the spill path would
//! serialise exactly the workload that spills, which is the one that needs the
//! threads.
//!
//! There is no lock, because there is nothing to lock. **The arena is immutable
//! for the entire compute phase.** A worker reads a ray's spans, applies every
//! motion of the batch to a local copy, and keeps the result in its own buffer;
//! the arena is written afterwards, sequentially, in ascending ray order, by the
//! existing `Arena::set`. Zero contention during compute, one ordered pass after,
//! and all of the arena's spill and compaction logic reused rather than
//! reimplemented under a lock.
//!
//! This is the second of the two designs the plan suggested, and it was preferred
//! over addressing spill as a pure function of ray index because that one buys
//! the problem away at the cost of reserving spill capacity for rays that never
//! use it — and Unit 10 has just spent a unit making memory predictable.
//!
//! # Why the removed volume needs more care than the counters
//!
//! Every other statistic is an integer count or a maximum, and both reassociate
//! freely. `removed_mm3` is a float sum, and the sequential path accumulates it
//! **flat over rays** for each motion. Chunk-local partials summed in chunk order
//! would regroup that sum — `(r0+r1) + (r2+r3)` against `((r0+r1)+r2)+r3` — which
//! is the same ULP that Unit 8's batching had to be redesigned to avoid.
//!
//! So a worker does not sum. It records each ray's contribution as a
//! `(motion, value)` pair, in ray order, and the reduction replays them flat.
//! The list is sparse — only rays that actually removed material appear, which on
//! a real job is well under 1% — so this costs a few bytes per changed ray
//! rather than a slot per ray-motion pair.
//!
//! # Threads
//!
//! `std::thread::scope`, not a thread pool crate. It is in `std`, it needs no
//! dependency review, and the scheduling this needs is one atomic counter. On a
//! target without threads — `wasm32-wasip1` — [`Schedule::threads`] resolves to
//! one and the sequential path runs, which is the same code either way.

use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::dexel::tri::{AXES, TriDexelField};
use crate::math::Ray;
use crate::spans::Spans;
use crate::tool::Profile;

use super::Motion;
use super::batch::split_runs;
use super::cut::{
    CutScratch, CutStats, SweepMethod, apply_swept, swept_for_ray, transverse_overlaps,
};

/// Rays per chunk.
///
/// Small on purpose. Unit 7 measured 84% of rays rejected by the box test, so
/// the surviving work is **spatially clustered**: a chunk that covers a whole
/// raster row is either almost all work or almost all rejection. Chunks well
/// below that spread the clusters across workers and let stealing do its job.
pub const DEFAULT_CHUNK: usize = 256;

/// Smallest motion batch the parallel path will use.
///
/// The thread scope is entered **once per bundle per batch**, so a job cut into
/// many small batches pays the spawn cost many times over. Measured on a 500
/// motion raster at eight workers:
///
/// ```text
/// batch     32     128     512    2048
/// time   0.120   0.100   0.094   0.094
/// gain    1.00x   1.24x   1.32x   1.32x
/// ```
///
/// Raising it is safe because **batch size cannot change the answer** -- Unit 8
/// established that and `batch_size_is_invisible_to_the_parallel_path` re-checks
/// it here -- so this is a scheduling knob and nothing more. It applies only
/// when more than one worker is running; a single worker keeps the caller's
/// batch and so stays byte-for-byte on the sequential path's traversal.
pub const PARALLEL_BATCH: usize = 512;

/// How to schedule a parallel cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Schedule {
    /// Worker threads. `0` means one per available core.
    pub threads: usize,
    /// Rays per chunk.
    pub chunk: usize,
    /// Adversarial scheduling seed.
    ///
    /// `Some(seed)` shuffles chunk order and varies chunk size reproducibly, to
    /// perturb the interleaving far harder than changing the thread count does.
    /// See [`chaos_order`].
    pub chaos: Option<u64>,
}

impl Default for Schedule {
    fn default() -> Self {
        Self {
            threads: 0,
            chunk: DEFAULT_CHUNK,
            chaos: None,
        }
    }
}

impl Schedule {
    /// A sequential schedule.
    #[must_use]
    pub const fn sequential() -> Self {
        Self {
            threads: 1,
            chunk: DEFAULT_CHUNK,
            chaos: None,
        }
    }

    /// The worker count this resolves to on this machine.
    ///
    /// **Never hashed.** It is a property of the host, not of the answer, and a
    /// report generated on a different machine must still compare equal.
    #[must_use]
    pub fn workers(&self) -> usize {
        if self.threads > 0 {
            return self.threads;
        }
        std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get)
    }
}

/// One ray's contribution to one motion's removed volume.
///
/// Recorded rather than summed, so the reduction can replay the sequential
/// path's flat order exactly. See the module header.
#[derive(Debug, Clone, Copy)]
struct Contribution {
    motion: u32,
    value: f64,
}

/// What one chunk produced.
struct ChunkResult {
    /// First ray of the chunk, so results can be ordered without a sort.
    start: u32,
    /// Rays whose spans changed, ascending: `(ray, offset, length)` into
    /// [`Self::changed_spans`].
    ///
    /// A flat arena rather than a `Spans` per ray. One `Vec` allocation per
    /// changed ray puts every worker into the global allocator on the hot path,
    /// and allocator contention is a classic way for a parallel loop to stop
    /// scaling for reasons that have nothing to do with the work.
    changed: Vec<(u32, u32, u32)>,
    /// The changed rays' spans, back to back.
    changed_spans: Vec<crate::spans::Span>,
    /// Removed-volume contributions, in `(ray, motion)` order.
    contributions: Vec<Contribution>,
    /// Counters, all of which reassociate freely.
    stats: CutStats,
}

/// Cuts a whole motion list across threads.
///
/// Produces a **bit-identical** field and bit-identical statistics to
/// [`super::batch::cut_all`] at any thread count, chunk size, or chaos seed.
///
/// # Panics
/// Panics if a worker thread panics, which can only happen through a bug in the
/// sequential code it shares.
pub fn cut_all_parallel(
    field: &mut TriDexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    size: usize,
    schedule: Schedule,
) -> CutStats {
    let mut total = CutStats::default();
    if motions.is_empty() {
        return total;
    }
    let mut removed = vec![[0.0f64; 3]; motions.len()];
    let tools = vec![0u32; motions.len()];
    // See `PARALLEL_BATCH`. Only when actually threading, so a one-worker run
    // traverses exactly as the sequential path does.
    let size = if schedule.workers() > 1 {
        size.max(PARALLEL_BATCH)
    } else {
        size
    };

    for (lo, hi) in split_runs(motions, &tools, size) {
        let run = &motions[lo..hi];
        for axis in AXES {
            let Some(bundle) = field.bundle_mut(axis) else {
                continue;
            };
            let slot = axis.index();
            let results = compute_bundle(bundle, profile, run, method, schedule);

            // --- the reduction, sequential and in fixed order ----------------
            //
            // Chunks arrive in whatever order they finished; they are sorted by
            // their first ray so what follows is ascending ray regardless.
            let mut ordered: Vec<ChunkResult> = results;
            ordered.sort_by_key(|c| c.start);

            for chunk in &ordered {
                total.merge_without_volume(&chunk.stats);
            }
            // Removed volume, replayed flat over rays for each motion, which is
            // exactly what the sequential path does.
            //
            // Bucketed by a **stable counting sort**, not by rescanning the list
            // once per motion. The first version did the latter and it was
            // `O(motions x contributions)` on the one pass that cannot be
            // parallelised -- Amdahl's serial fraction, made quadratic. It held
            // eight-thread efficiency to 40.9% on the balanced raster; bucketing
            // is `O(n + motions)` and the same arithmetic in the same order,
            // because a stable sort preserves the ray order the chunks were
            // already in.
            let motions_here = hi - lo;
            let mut counts = vec![0usize; motions_here + 1];
            for chunk in &ordered {
                for c in &chunk.contributions {
                    counts[c.motion as usize] += 1;
                }
            }
            let mut offset = 0usize;
            for count in &mut counts {
                let n = *count;
                *count = offset;
                offset += n;
            }
            let mut bucketed = vec![0.0f64; offset];
            let mut cursor = counts.clone();
            for chunk in &ordered {
                for c in &chunk.contributions {
                    let at = &mut cursor[c.motion as usize];
                    bucketed[*at] = c.value;
                    *at += 1;
                }
            }
            for (m, slot_value) in removed.iter_mut().enumerate().take(hi).skip(lo) {
                let index = m - lo;
                for value in &bucketed[counts[index]..cursor[index]] {
                    slot_value[slot] += *value;
                }
            }
            // The arena, written once per changed ray in ascending order.
            let arena = bundle.arena_mut();
            for chunk in &ordered {
                for (ray, at, len) in &chunk.changed {
                    let from = *at as usize;
                    let to = from + *len as usize;
                    arena.set(*ray, &chunk.changed_spans[from..to]);
                }
            }
        }
    }

    for slot in 0..3 {
        for per_motion in &removed {
            total.removed_mm3[slot] += per_motion[slot];
        }
    }
    total
}

/// Runs one bundle's chunks, on as many threads as the schedule asks for.
fn compute_bundle(
    bundle: &crate::dexel::DexelField,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    schedule: Schedule,
) -> Vec<ChunkResult> {
    let lattice = bundle.lattice().clone();
    let rays = u32::try_from(bundle.arena().rays()).unwrap_or(u32::MAX);
    if rays == 0 {
        return Vec::new();
    }
    // The swept boxes, computed once per bundle rather than once per chunk.
    //
    // They were inside `run_chunk`, which meant `motions x chunks` bound
    // computations -- redundant work that grows with the chunk count and so
    // penalises exactly the small chunks the clustering argument asks for.
    let boxes: Vec<crate::math::Aabb3> = motions.iter().map(|m| m.swept_bounds(profile)).collect();
    let union = boxes
        .iter()
        .skip(1)
        .fold(boxes.first().copied().unwrap_or_default(), |a, b| {
            a.union(b)
        });

    let chunk = schedule.chunk.max(1);
    let count = (rays as usize).div_ceil(chunk);
    let order = chaos_order(count, schedule.chaos);

    let workers = schedule.workers().max(1).min(count.max(1));
    let next = AtomicUsize::new(0);
    let out: Mutex<Vec<ChunkResult>> = Mutex::new(Vec::with_capacity(count));

    // One thread is the sequential path, and taking it avoids spawning at all --
    // the plan asks that parallel machinery cost near zero when unused.
    if workers <= 1 {
        let mut scratch = CutScratch::new(profile);
        let mut local = Vec::with_capacity(count);
        for slot in &order {
            local.push(run_chunk(
                bundle,
                &lattice,
                profile,
                motions,
                method,
                &mut scratch,
                *slot,
                chunk,
                rays,
                &boxes,
                union,
            ));
        }
        return local;
    }

    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                // Per worker, so the inner loop still allocates nothing. This is
                // the `threads x scratch` the Unit 10 ceiling has to account for.
                let mut scratch = CutScratch::new(profile);
                let mut mine = Vec::new();
                loop {
                    let index = next.fetch_add(1, Ordering::Relaxed);
                    if index >= order.len() {
                        break;
                    }
                    mine.push(run_chunk(
                        bundle,
                        &lattice,
                        profile,
                        motions,
                        method,
                        &mut scratch,
                        order[index],
                        chunk,
                        rays,
                        &boxes,
                        union,
                    ));
                }
                out.lock().expect("no worker panicked").extend(mine);
            });
        }
    });

    out.into_inner().expect("no worker panicked")
}

/// One chunk of rays, against every motion of the run.
#[allow(
    clippy::too_many_arguments,
    reason = "a worker's whole context; a struct here would exist only to be destructured"
)]
fn run_chunk(
    bundle: &crate::dexel::DexelField,
    lattice: &crate::dexel::Lattice,
    profile: &Profile,
    motions: &[Motion],
    method: SweepMethod,
    scratch: &mut CutScratch,
    slot: usize,
    chunk: usize,
    rays: u32,
    boxes: &[crate::math::Aabb3],
    union: crate::math::Aabb3,
) -> ChunkResult {
    let axis = lattice.axis();
    let cell_area = lattice.cell_area();
    let start = u32::try_from(slot * chunk).unwrap_or(u32::MAX);
    let end = u32::try_from(((slot + 1) * chunk).min(rays as usize)).unwrap_or(rays);

    let mut result = ChunkResult {
        start,
        changed: Vec::new(),
        changed_spans: Vec::new(),
        contributions: Vec::new(),
        stats: CutStats::default(),
    };
    // The ray's spans, carried across the run's motions locally rather than
    // written back between them. The arena is not touched here at all.
    let mut material = Spans::new();
    let mut next_material = Spans::new();

    for ray_index in start..end {
        let (i, j) = lattice.coords(ray_index);
        let origin = lattice.origin_of(i, j);
        if !transverse_overlaps(&union, axis, origin, lattice.spacing_uv()) {
            result.stats.rays_rejected += motions.len() as u64;
            continue;
        }
        let ray = Ray {
            origin,
            direction: axis.direction(),
        };
        bundle.arena().read_into(ray_index, &mut material);
        let mut touched = false;

        for (index, motion) in motions.iter().enumerate() {
            if !transverse_overlaps(&boxes[index], axis, origin, lattice.spacing_uv())
                || material.is_empty()
            {
                result.stats.rays_rejected += 1;
                continue;
            }
            result.stats.rays_tested += 1;
            if !swept_for_ray(profile, motion, method, scratch, &ray, &mut result.stats) {
                continue;
            }
            let cut = apply_swept(&material, &scratch.swept, &mut next_material, cell_area);
            if cut == 0.0 && next_material.as_slice() == material.as_slice() {
                continue;
            }
            core::mem::swap(&mut material, &mut next_material);
            touched = true;
            // `> 0.0`, matching the sequential path exactly, and the two
            // conditions are deliberately different.
            //
            // A ray whose spans changed while its *measure* did not -- a cut
            // that split one span into two of the same total length -- must be
            // written back but must NOT count as changed, because the sequential
            // path counts on `cut > 0.0`. Mirroring the write-back condition
            // here instead over-counted by 8 rays out of 10,496 on the raster:
            // the field and the volume were already identical, and only the
            // statistic disagreed.
            if cut > 0.0 {
                result.stats.rays_changed += 1;
                result.contributions.push(Contribution {
                    motion: u32::try_from(index).unwrap_or(u32::MAX),
                    value: cut,
                });
            }
        }
        if touched {
            let at = u32::try_from(result.changed_spans.len()).unwrap_or(u32::MAX);
            let len = u32::try_from(material.len()).unwrap_or(u32::MAX);
            result.changed_spans.extend_from_slice(material.as_slice());
            result.changed.push((ray_index, at, len));
        }
    }
    result
}

/// The order chunks are handed out in.
///
/// Identity unless a chaos seed is given, in which case the chunks are shuffled
/// by a small deterministic generator. **Shuffling the order cannot change the
/// answer** — that is the property under test — so this is a scheduling
/// perturbation far stronger than varying the thread count, which on a machine
/// with spare cores may produce nearly the same interleaving at 4 and at 8.
#[must_use]
pub fn chaos_order(count: usize, seed: Option<u64>) -> Vec<usize> {
    let mut order: Vec<usize> = (0..count).collect();
    let Some(seed) = seed else {
        return order;
    };
    // SplitMix64: a few lines, no dependency, and identical on every target.
    let mut state = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut next = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    };
    // Fisher-Yates, descending, which is the standard unbiased form.
    for i in (1..order.len()).rev() {
        let j = (next() % (i as u64 + 1)) as usize;
        order.swap(i, j);
    }
    order
}
