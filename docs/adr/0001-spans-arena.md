# ADR 0001 — Dexel field construction: storage, and the ray lattice offset

- **Status:** Accepted, implementation deferred to U5
- **Date:** 2026-08-02 (storage), amended 2026-08-02 after U2 (lattice offset)
- **Unit:** raised in U1, recorded and amended in U2, to be acted on in U5

This ADR carries two decisions for U5. The first is about where a ray's spans
live. The second, added after Unit 2's measurements, is about **where the rays
themselves are placed**, and it is the more urgent of the two because it is a
correctness invariant as well as a performance one.

---

# Part 2 — The ray lattice must be offset to cell centres

**This is a required invariant, not a tuning parameter.**

## Decision

U5 must place dexel ray origins at **cell centres**, offset by half a cell from
the integer lattice. Any future change that moves them onto cell corners — for
symmetry, for simplicity, for "cleaner" indexing — is a correctness regression,
not a refactor.

Additionally: during dexel construction, `RayStats::coplanar_rejected > 0` must
be a **hard error that aborts the build**, not a statistic in a report.

## Why: one decision buys both correctness and 16x

Unit 2 measured the same choice from two directions.

**Performance.** 4,096 rays against a 5,292-triangle lattice block:

| ray origins | time |
|---|---:|
| offset to cell centres | 2.52 ms |
| on the integer lattice | 39.83 ms |

**15.8x**, on the innermost loop of the entire product, available for free by
choosing an offset. The cause is Simulation of Simplicity: the exact-fallback
rate goes from 3.94% on generic geometry to **65.80%** when every ray strikes a
vertex or an edge head on.

**Correctness.** `chipbreaker_core::mesh::bvh` is leak-free by construction for
rays through edges and vertices — SoS resolves those, and the antisymmetry
argument is a proof rather than a hope. But it has **one documented gap**: a ray
that is *coplanar* with a triangle has all three edge functions vanish, and that
is not a sign question at all. The intersection is a segment, so the crossing
parameter `t` is genuinely undetermined; no amount of symbolic perturbation
fixes it, because there is nothing to take the sign of.

Those triangles are currently rejected. Rejection is pragmatic, not principled,
and Unit 2 could measure that parity survives it (16k–40k rejections per lattice
sweep, zero leaks) without being able to *prove* it in general.

Do not try to prove it. Make it impossible instead, and make its occurrence
loud.

## Why a hard error rather than a warning

A ray coplanar with a face is only reachable when the lattice is aligned to the
model. Offsetting removes it. So during dexel construction the count should be
zero, and any non-zero value means the offset has stopped doing its job —
because the stock was modelled on a half-cell grid, because someone changed the
lattice, because a transform put a face exactly on a ray plane.

At that moment there are two options. Continue, and produce a field that may or
may not contain a leaked ray, which surfaces months later as a tunnel through a
customer's simulated part and takes days to trace back here. Or stop, and say
which ray and which triangle.

The second is obviously right, and it is only available if the check is an error.
A warning in a log is the same as no check: the failure mode this guards against
is precisely the one where nobody was looking.

This also converts the unproven-but-measured path into a *guarded* one: we do not
need a proof that rejection is parity-safe, because we refuse to rely on it.

## Consequences

- U5's dexel builder takes the offset as a documented constant with this
  reasoning attached, not as a configurable.
- `Bvh::intersect_ray_all_into` already returns `RayStats`; U5 checks
  `coplanar_rejected` per ray and fails the build on the first non-zero.
- The error must name the ray and the triangle, since the user's next question
  is "where".
- If a legitimate model ever trips it, the fix is to perturb the lattice offset,
  not to relax the check.

---

# Part 1 — `Spans` storage: per-ray `Vec` today, flat arena at U5

## Context

`chipbreaker_core::spans::Spans` currently owns a `Vec<Span>`:

```rust
pub struct Spans {
    spans: Vec<Span>,
}
```

This is the right shape for Unit 1, where a `Spans` is a standalone value that
tests construct, combine and compare. It is the wrong shape for Unit 5.

U5 builds a tri-dexel field: three orthogonal bundles of parallel rays, each ray
storing the material intervals along it. A 1000 x 1000 field in each of three
directions is **three million rays**, and under the current design that is:

- **Three million heap allocations** at construction, plus more on every growth.
  Allocation is not free and, worse, allocator behaviour is a source of
  timing variance that makes performance work harder to reason about.
- **72 MB of `Vec` headers alone** — 24 bytes of pointer, length and capacity per
  ray, before a single `Span` is stored. The payload for a typical ray is one or
  two spans, i.e. 16–32 bytes. **The bookkeeping outweighs the data.**
- **Terrible locality.** U5's access pattern is a coherent sweep: process ray
  *i*, then ray *i+1*, then *i+2*. With per-ray allocations those live wherever
  the allocator put them, so a sweep that should stream linearly through memory
  instead chases three million pointers.

None of this is hypothetical arithmetic dressed up as a problem — it is the
dominant cost of the data structure at the scale U5 operates at.

## Decision

**Keep `Vec<Span>` through U4. Replace it with a flat arena at U5.**

The intended shape:

```rust
/// All spans for all rays, contiguous.
pub struct SpanArena {
    spans: Vec<Span>,
    /// Ray `i` owns `spans[offsets[i] .. offsets[i + 1]]`.
    /// Length is ray_count + 1, so the last ray needs no special case.
    offsets: Vec<u32>,
}

/// A borrowed, mutable view of one ray's spans.
pub struct SpansMut<'a> { /* &'a mut [Span] plus growth policy */ }
```

Because cutting *shrinks or splits* a ray's span set rather than growing it
without bound, a per-ray capacity plus a spill list handles the rare case where a
cut increases the span count beyond its slot. That detail is U5's to settle; what
matters here is that the arena is the target.

## Why the current API already converges on this

This is the part worth recording, because it means U5 is a substitution rather
than a rewrite.

Unit 1 deliberately added `_into` variants to every set operation:

```rust
pub fn union_into(&self, other: &Self, out: &mut Self);
pub fn intersect_into(&self, other: &Self, out: &mut Self);
pub fn subtract_into(&self, other: &Self, out: &mut Self);
pub fn complement_within_into(&self, bounds: Span, out: &mut Self);
```

These take the output as a caller-owned buffer instead of returning a fresh
`Spans`. That is exactly the signature an arena needs: the caller already owns
the storage and passes a place to write. Migrating means changing what `out` *is*
— from `&mut Spans` to `&mut SpansMut<'_>` — not changing how callers are
written.

The benchmark supports the shape too: at n = 10, which is where a dexel ray
actually lives, `subtract_into` is 27% faster than `subtract` purely from not
allocating. The arena extends that saving from "no allocation per operation" to
"no allocation at all".

## Why not now

1. **No caller needs it.** U2–U4 handle meshes, tools and toolpaths, none of
   which hold millions of `Spans`. Building an arena before there is a consumer
   means guessing its access pattern, and guessing wrong is how you get an
   abstraction that has to be unpicked later.
2. **The growth policy depends on U5's cut algorithm**, which does not exist yet.
   How often does a cut split a span? What is the realistic maximum count per
   ray? Those answers determine the slot sizing, and they are measurements, not
   opinions.
3. **`Spans` as a standalone value is genuinely useful** for tests, for the CLI,
   and for U12's deviation fields. The arena should be an *additional* storage
   strategy behind the same operations, not a replacement that removes the simple
   case.

## Consequences

- U5 must budget for this work rather than discovering it. It is a known,
  scoped task, not a surprise.
- The `_into` variants must be preserved and preferred. A future contributor who
  "simplifies" them away because the allocating form reads better would be
  removing the migration path. Their doc comments say so.
- The structural invariant and the merge-scan are storage-independent: they
  operate on `&[Span]` and `&mut Vec<Span>` already. The arena changes where the
  slice comes from, not what the algorithm does.
- Benchmarks at U5 should compare arena against per-ray `Vec` on the same
  workload, so the claim in this document is measured rather than assumed.

## Alternatives considered

- **Small-vector optimisation** (inline capacity for 1–2 spans, spilling to the
  heap beyond that). Removes most allocations and needs no arena. Rejected as the
  primary plan because it does not fix locality — the rays are still scattered —
  and because every mainstream small-vector crate uses `unsafe`, which this
  workspace forbids. Worth revisiting only if the arena's growth handling turns
  out to be genuinely awkward.
- **Fixed maximum spans per ray, no spill.** Simplest and fastest, but it makes
  a geometric limit out of an implementation detail: a ray crossing a comb-like
  feature would silently lose material. Unacceptable in a verification tool,
  where the entire product claim is that we do not silently lose material.
- **Keeping `Vec<Span>` and accepting the cost.** Defensible if U5 turned out to
  be dominated by something else entirely. The numbers above say it will not be.
