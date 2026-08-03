# ADR 0001 — Dexel field construction: storage, and the ray lattice offset

- **Status:** Accepted, implementation deferred to U5
- **Date:** 2026-08-02 (storage), amended 2026-08-02 after U2 (lattice offset)
- **Unit:** raised in U1, recorded and amended in U2, to be acted on in U5

This ADR carries three decisions. Part 1 is about where a ray's spans live, as
scoped at U1. Part 2, added after Unit 2's measurements, is about **where the
rays themselves are placed**, and it is the most urgent of the three because it
is a correctness invariant as well as a performance one. Part 3, added at U5,
records the arena as actually built, and how the measurement changed the design
that Part 1 anticipated.

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
choosing an offset.

**Amended at U5 with a second measurement.** A number that decides a required
invariant should not rest on one measurement in one place, so
`benches/dexel.rs` repeats it against the same lattice block, cast the way
construction actually casts: 414 us with cell centres against 2.30 ms on the
integer lattice, or **5.5x**.

That is a third of the original figure, and the gap is recorded rather than
smoothed over. The two benchmarks sweep different ray counts over different
extents, so they are not measuring quite the same thing; what they agree on is
the sign and the order of magnitude. The decision does not turn on whether it is
5x or 16x -- it turns on correctness, and the performance is the bonus. Anyone
quoting a single number for this should quote 5.5x from the construction path,
because that is the path the product runs. The cause is Simulation of Simplicity: the exact-fallback
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

---

# Part 3 — The arena as built

- **Status:** Accepted, implemented
- **Date:** 2026-08-03
- **Unit:** 5

Part 1 scoped a flat arena and left the shape open. This part records what was
built, and it differs from Part 1's plan in one significant way, because the
measurement said so.

## The measurement came first

Part 1's reasoning was structural: per-ray `Vec` costs a 24-byte header and an
allocation per ray, and 4M rays is ~96 MB of headers before a single interval
exists. That argument is sound and unchanged. What it did *not* say is what the
replacement should look like, and that depends entirely on the distribution of
span counts — a fact nobody had measured. A spread-out distribution wants a
general allocator; a degenerate one wants a flat array. Guessing would have been
guessing at the central data structure of the product.

So `examples/span_distribution.rs` was written before `arena.rs`, deliberately
on top of the `Vec<Spans>` representation the arena is meant to replace, because
measuring with the thing being designed would be circular. Casting a `+Z` bundle
at 0.5 mm:

| mesh | 0 spans | 1 span | 2 spans | max |
|---|---:|---:|---:|---:|
| box, stock at rest | — | 100% | — | 1 |
| sphere r=20 | 21.8% | 78.2% | — | 1 |
| torus R=20 r=6, axis along the bundle | 44.3% | 55.7% | — | 1 |
| nested shells (a cavity) | 21.8% | 58.6% | 19.6% | 2 |
| lattice block, integer vertices | — | 100% | — | 1 |

The distribution is not merely skewed. It is nearly degenerate: **stock at rest
is exactly one span on every ray**, and the only case reaching two is a genuine
internal cavity. Inline capacity 2 covers 100% of every case measured; capacity
1 covers only 80.4% of the cavity case.

One result was a surprise and is recorded because it corrects a natural
assumption: a torus whose axis lies **along** the bundle does not produce
two-span rays. Its hole appears as 44% *empty* rays. A through hole gives two
spans only when it runs *transverse* to the bundle. Anyone sizing storage
against "holes give two spans" would be sizing against the wrong picture.

## Decision

```rust
pub struct Arena {
    inline: Vec<Span>,               // rays * INLINE_CAPACITY, flat
    len:    Vec<u16>,                // spans per ray, inline or spilled
    spill:  BTreeMap<u32, Vec<Span>>, // rays past capacity, holding ALL their spans
}
pub const INLINE_CAPACITY: usize = 2;
```

Two allocations for the whole field, both sized as a pure function of the ray
count. `set(ray, &[Span])` is the only mutation.

**This is not a general-purpose allocator and must not become one.** The
distribution above is its entire justification; a design that would be better
for a spread-out distribution would be worse for this one.

### Why 2, and not 1 or 4

One would cover every ray of stock at rest, and would spill on the first pocket
cut into a block. U7 subtracts the tool from these spans millions of times, and
subtraction **splits**: cutting a slot through a solid ray turns one span into
two. Growth is the common mutation, not a rare one.

Four would double the resting footprint — 4M rays at 16 bytes a span is 128 MB
against 64 MB — to buy a case that has not been observed.

### Why `u16` and not `u8`

A ray through a honeycomb or a lattice-work part can genuinely carry hundreds of
spans. A silent wrap at 256 would be a field that looks fine and is wrong, which
is the worst failure mode available to a verification tool. Two bytes per ray is
8 MB at 4M rays, against the 64 MB of inline slots; it is not worth being clever
about.

### Why `BTreeMap` for the spill

Determinism. The spill is iterated when hashing, and unordered iteration
reaching a float is exactly what the standing rules forbid. `HashMap` would be
faster and would make the field hash depend on hasher state.

### Two behaviours that are contract, not detail

**A ray that shrinks releases its spill.** If it did not, a ray that split under
cutting and later merged back would keep dead storage alive for the run, and the
arena would only ever grow. `a_ray_that_shrinks_releases_its_spill` covers it.

**The hash reads the spans a ray *has*, never the slots it was given.** Unused
inline slots keep whatever was last written there. Hashing the raw backing array
would make two fields with identical geometry disagree because one of them had
been cut and restored. `the_hash_depends_on_contents_and_not_on_history` covers
it.

## Deviation from Part 1

Part 1 rejected small-vector optimisation as the primary plan, on two grounds:
it does not fix locality, and every mainstream small-vector crate uses `unsafe`,
which this workspace forbids.

What was built is, in effect, a small-vector optimisation — *and both objections
still hold and are answered*, which is why the deviation is recorded rather than
quietly taken.

The locality objection was aimed at `Vec<SmallVec>`, where the rays are still
scattered because each ray owns its own inline buffer inside a per-ray struct.
Here the inline slots are **one flat array for the whole field**, so consecutive
rays are consecutive in memory. That is Part 1's arena; the inline capacity is a
property of the arena's layout, not a per-ray container.

The `unsafe` objection was aimed at the crates. This uses none: `Vec<Span>`
indexed by arithmetic, with `len` tracked alongside. The cost is that unused
slots hold initialised `Span::new(0.0, 0.0)` values rather than uninitialised
memory, which is a wasted memset at construction and nothing else.

So Part 1's conclusion stands and its framing was slightly off: the two designs
it presented as alternatives are the same design seen from different levels.

## Consequences

- Memory is a pure function of ray count until something spills, which
  `memory_is_proportional_to_rays_and_free_of_per_ray_allocation` asserts.
- `spilled_rays()` is the number that says whether `INLINE_CAPACITY` is still
  right. If it stops being near zero on real work, revisit it against a fresh
  measurement — not by intuition, and not by bumping the constant until the
  number looks better.
- `the_inline_capacity_is_two_and_the_reason_is_recorded` guards the constant
  itself. Changing it is allowed; changing it as a tidy-up should not slip
  through review unnoticed.
## The Part 1 benchmark obligation, discharged

Part 1 argued the arena would win from structure rather than measurement, and
left the benchmark as an obligation on U5. `benches/dexel.rs` runs it. Filling
and then scanning one span per ray, release build:

| rays | arena fill | `Vec<Spans>` fill | arena scan | `Vec<Spans>` scan |
|---:|---:|---:|---:|---:|
| 10,000 | 41.1 us | 242 us | 9.76 us | **7.46 us** |
| 100,000 | 813 us | 2.82 ms | 104 us | 135 us |
| 1,000,000 | 8.44 ms | 35.2 ms | 1.82 ms | 2.92 ms |

Filling is **4.2x** faster at a million rays, which is the allocation argument
holding up. Scanning is **1.6x**, which is the locality argument.

The 10,000-ray scan row is bolded because the arena **loses** there, and the row
is kept rather than dropped. At that size the whole working set fits in cache
either way, so locality buys nothing and the arena still pays for the indirection
through `len` on every ray. The arena is the right structure for the sizes this
product runs at, not for every size, and a future reader benchmarking a small
field should find that already written down rather than discover it and conclude
the design is wrong.

## Consequences

- Memory is a pure function of ray count until something spills, which
  `memory_is_proportional_to_rays_and_free_of_per_ray_allocation` asserts.


---

# Part 4 — The spill path, rebuilt at U7

- **Status:** Accepted, implemented
- **Date:** 2026-08-03
- **Unit:** 7
- **Amends:** Part 3's `BTreeMap<u32, Vec<Span>>` spill

## The measurement Part 3 could not have taken

Part 3 sized `INLINE_CAPACITY = 2` on **stock at rest**, and said so: the
distribution there is nearly degenerate, one span on every filled ray, and only
a genuine internal cavity reaches two. It also said that if `spilled_rays()`
stopped being near zero on real work, the number should be revisited against a
fresh measurement rather than by intuition.

Unit 7 took that measurement, on a 60x40x12 mm block at 0.4 mm cells:

| geometry | max spans | spilled rays |
|---|---:|---:|
| stock at rest | 1 | 0 |
| one slot (a pocket) | 2 | 0 |
| two slots either side of a rib | **3** | **4,500** |
| five slots (a comb) | **6** | **4,500** |

The aggregate figure hides the shape of it. Per bundle, on the rib:

```
x: 3000 rays, max 1 span,  0 spilled
y: 4500 rays, max 3 spans, 4500 spilled     <- every ray
z: 15000 rays, max 1 span, 0 spilled
```

**Spill is not a tail. It is per bundle.** When features run parallel to one
axis, every ray of the perpendicular bundle crosses every feature, so one bundle
spills completely while the other two never spill at all.

## Why not simply raise the capacity

Because it moves a threshold rather than removing one. Capacity 4 covers the
rib and not the comb; an eleven-slot part needs twelve. And it is paid on every
ray of every bundle:

| | bytes/ray | cube 100^3 at 0.05 mm |
|---|---:|---:|
| capacity 2 | 34 | 389 MiB |
| capacity 4 | 66 | 755 MiB |
| capacity 6 | 98 | 1121 MiB |

Ninety-four percent more memory, on all three bundles, to postpone a threshold
that two of them never approach.

## Decision

Keep `INLINE_CAPACITY = 2`. Replace the spill map with a chunked heap:

```rust
spill_at: Vec<u32>,   // offset into `heap`, or NO_SPILL -- LAZY
heap:     Vec<Span>,  // every spilled ray's spans, one allocation
garbage:  usize,      // spans no ray points at
```

- **No per-ray allocation.** 4,500 spilled rays cost one growing `Vec`, not
  4,500 of them. A spill path that allocated per ray was exactly the per-ray
  allocation Part 1 was written to eliminate, and Part 3 left it in place
  because the measurement said it would be rare.
- **The index is lazy.** `spill_at` is not allocated until a ray actually
  spills, so the two clean bundles keep paying exactly the 34 bytes a ray Part 3
  measured. Only the bundle that spills pays the extra 4.
- **Compaction is deterministic.** `compact()` walks rays in ascending index, so
  the compacted layout is a pure function of contents rather than of the order
  rays happened to grow. Two arenas holding the same spans compact to the same
  bytes, which is what keeps the field hash independent of history — the
  property `the_hash_depends_on_contents_and_not_on_history` has asserted since
  Part 3.
- Compaction triggers at half garbage, so growth is amortised `O(1)`.

## Consequences

- `spilled_rays()` remains the number to watch, but it is now a cost rather than
  a cliff: a field with every ray spilled is slower and larger, not broken.
- Part 3's inline-capacity justification stands **for the resting distribution
  only**, and now says so. The capacity was never the interesting number; the
  spill path was, and Part 3 got the cheap half right.
