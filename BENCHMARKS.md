# Benchmarks

**This file is append-only.** Add a new dated section for each measurement run;
never edit or delete an old one. Numbers taken on different machines are not
comparable in absolute terms, which is exactly why the machine description is
recorded alongside every table. What we are building here is a regression
record, and a regression record you can rewrite is not one.

Run them with:

```sh
cargo bench --bench predicates -- --warm-up-time 1 --measurement-time 3
cargo bench --bench spans      -- --warm-up-time 1 --measurement-time 3
cargo bench --bench mesh       -- --warm-up-time 1 --measurement-time 3
cargo bench --bench tool       -- --warm-up-time 1 --measurement-time 3
cargo bench --bench gcode      -- --warm-up-time 1 --measurement-time 3
cargo bench --bench deviation  -- --warm-up-time 1 --measurement-time 3
cargo bench --bench findings   -- --warm-up-time 1 --measurement-time 3
```

CI compiles the benchmarks (`cargo bench --no-run`) but does not time them.
Timing on shared CI runners produces numbers with more variance than the
regressions we would be looking for.

---

## 2026-08-08 — findings: clustering, attribution, and diffing

- **Commit:** `HEAD`
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** the pinned `rust-toolchain.toml`
- **Command:** `cargo bench --bench findings`

### Clustering scales with what is wrong, not with the part

A 40 x 30 x 12 mm block, raster-faced, compared against its own nominal at three
depths:

| cut | samples | above tolerance | findings | time |
|---|---|---|---|---|
| correct | 28,960 | 666 | 1 | 129 µs |
| 0.5 mm deep | 29,306 | 6,035 | 1 | 1.63 ms |
| 2.0 mm deep | 30,690 | 7,419 | 1 | 2.15 ms |

Throughput holds at 3.4–5.2 M samples/second. The field barely changes size and
the work changes by 17x, which is the shape wanted: a clean part costs almost
nothing to cluster however large it is, and the expensive case is a part
somebody was about to spend a long time on anyway.

> **These figures carry more variance than earlier entries in this file.**
> Repeat runs on the same binary moved by up to 30% — criterion reported the
> shifts as significant, and they are, of the machine rather than of the code.
> This measurement was taken with other work running. Treat the numbers as
> order-of-magnitude and the *ratios* as the finding; a regression baseline
> wants a quiet machine, and this one was not.

**One finding, not thousands**, because a raster gouged uniformly *is* one
connected region. That is the correct answer and a useless benchmark, so the
scale case below builds the pathological one directly.

### At scale: one finding per sample

Isolated samples a millimetre apart with a 0.05 mm radius, so nothing merges and
the finding count equals the sample count:

| findings | cluster | cluster + identify |
|---|---|---|
| 1,000 | 0.66–0.80 ms | 1.7–2.2 ms |
| 10,000 | 8.2–9.8 ms | 19.6–21.9 ms |

Linear in findings — 10x the input for roughly 12x the time — and **about 20 ms
for the ten-thousand-finding case**, which is the number that matters: a report
that large is one nobody should be machining, but the generator has to survive
it.

`identify` costs about as much again as clustering, which is the hashing. That
is a fair price for identities that make two reports diffable.

### Attribution, and diffing

| | |
|---|---|
| attribute 1 finding against 5 segments | 2.7–3.7 µs |
| diff two reports | 0.9–1.2 µs per finding |

Attribution is measured **with the box rejection in place**, because that is how
it runs: the rejection is what stops the segment count mattering, and measuring
without it would price an implementation nobody ships.

Diffing at a microsecond per thousand findings is the number that lets
`report-diff` sit in somebody's CI on every push. A diff slower than the
verification it compares would be a strange thing to ship.

### What this priced, and what it decided

The choice between storing a segment index per span endpoint and recomputing
attribution for findings alone. Recomputing costs a few microseconds per finding
and nothing in steady state; storing would have cost **eight bytes on every span
in every field the engine builds** — a third more memory, spent on regions that
are overwhelmingly not findings. The variance above is 30% and the decision
turns on a factor of thousands, so it is not close and does not need a quiet
machine to settle.

---

## 2026-08-07 — comparing a result against the part it was meant to be

- **Commit:** `HEAD` (the deviation field)
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** the pinned `rust-toolchain.toml`
- **Command:** `cargo bench --bench deviation`

### Per sample, because that is what transfers between jobs

`compare` visits every span endpoint of all three bundles, so its cost follows
the **surface area** of the cut result rather than the part's volume or the
program's length. A 40 x 30 x 12 mm block, raster-faced with a 6 mm flat mill,
compared against a mesh extracted from the same field:

| cell | samples | nominal triangles | total | per sample |
|---|---|---|---|---|
| 0.80 mm | 7,260 | 14,440 | 86.3 ms | 11.9 µs |
| 0.50 mm | 18,496 | 36,992 | 167.7 ms | 9.1 µs |
| 0.35 mm | 38,530 | 77,060 | 358.8 ms | 9.3 µs |

Flat in the cell size once the hierarchy is deep enough to matter, which is the
shape a `log n` traversal should have.

### The three queries, and the one that was not what I expected

Each sample runs a closest-point query for the metric, two ray casts for the
perpendicular diagnostic, and one more gathering every crossing for the
containment parity that decides the sign. Timed on 248 points drawn from the
field itself:

| query | before | after | share |
|---|---|---|---|
| `closest_point` | 10.4 µs | **0.64 µs** | 7% |
| two nearest-hit casts | 7.4 µs | 5.53 µs | 62% |
| all crossings, for parity | 4.0 µs | 2.72 µs | 31% |
| **sum** | 21.8 µs | **8.89 µs** | |

The sum tracks the end-to-end figure — 8.89 against 9.3 µs — so the three
queries account for essentially all of it and there is no fourth cost hiding.

**I predicted the parity cast would dominate**, on the reasoning that it is the
one query that cannot stop early: a nearest-point search abandons a subtree
when its bound exceeds the running best and a nearest-hit cast stops at the
first crossing, but a parity test has to find them all. It was the cheapest of
the three from the start, and after the fix below it is a third of the cost of
the two casts that *can* stop early.

### A 13x traversal win that was sitting behind an over-cautious comment

`closest_point` pushed BVH children in index order, with a comment saying that
sorting them by distance would make the traversal depend on floating-point
rounding. That is true and it does not matter: ties are broken by **triangle
index**, so the answer cannot depend on visit order at all — rounding changes
how much work is pruned, never which triangle comes back.

Descending the nearer child first establishes a tight bound immediately, and
the far subtree is then usually rejected whole rather than walked to its
leaves. **13x** on the query, and the reuse of a traversal buffer instead of a
`Vec` per call took a little more:

| | before | after | change |
|---|---|---|---|
| `compare`, h = 0.35 | 920.3 ms | 358.8 ms | **2.6x** |
| `compare`, h = 0.50 | 374.3 ms | 167.7 ms | 2.2x |
| `closest_point`, per query | 10.4 µs | 0.64 µs | 16x |

Every golden, ladder rung and recall figure is unchanged, which is the point:
the ordering affects what is pruned and nothing else.

### What the numbers argue for next

The **perpendicular diagnostic now costs 62% of a comparison** — more than the
metric it is a diagnostic for. It is genuinely useful (it is what makes the
step-edge artefact visible rather than silent), so it is not being removed,
but a `--no-perpendicular` flag would be a 2.5x saving for a customer who only
wants the verdict. Recorded here rather than implemented, because nothing has
asked for it yet.

Both casts go through `intersect_ray`, which calls `intersect_ray_all` — a
`Vec` allocation and a full sort of every crossing, to then take the nearest.
That is the obvious next target and it is mesh code used everywhere, so it
wants its own change rather than a drive-by.

### The tessellation floor

| | triangles | time | per triangle |
|---|---|---|---|
| `facet_size`, nominal | 36,992 | 6.53 ms | 177 ns |

Once per mesh rather than once per sample, so it is 4% of a comparison at
h = 0.5 and less at finer cells. Timed anyway, because it walks every edge and
builds a map to do it, and "expected to be invisible" is the kind of claim that
turns out to be a third of the runtime.

---

## 2026-08-03 — G-code parsing and the toolpath IR

- **Commit:** `HEAD` (parser and toolpath IR complete)
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** rustc 1.96.0 (ac68faa20 2026-05-25), `x86_64-pc-windows-msvc`
- **Profile:** `bench` — `lto = "fat"`, `codegen-units = 1`
- **Criterion:** 1 s warm-up, 3 s measurement.

```sh
cargo bench --bench gcode -- --warm-up-time 1 --measurement-time 3
cargo run --release -p chipbreaker-gcode --example ir_memory
```

### The number the dexel field has to budget around: IR memory

Measured from the real layout rather than added up from the struct definition,
because padding and the inline `Option<ArcData>` make the two differ.

| | bytes |
|---|---:|
| `MotionSegment` | **192** |
| ...of which `ArcData`, carried inline | 56 |
| `PathEvent` | 40 |

| segments | resident |
|---:|---:|
| 100,000 | 18.3 MB |
| **1,000,000** | **183.1 MB** |
| 10,000,000 | 1831.1 MB |

**A million segments cost 183 MB**, and a million-segment finishing pass is
ordinary. The IR is held beside the dexel field, so that is a real slice of a
working set rather than a footnote.

The arc payload is inline rather than boxed, so a program of pure linear moves
pays 56 bytes a segment for arcs it does not have — roughly 29% of the total.
That is the right trade while sweeping wants arc data resident, and it
is the first thing to revisit if the number becomes a problem. Boxing it would
take a linear-only program to about 136 bytes a segment.

### End to end, synthetic raster surfacing

| lines | time | throughput |
|---:|---:|---:|
| 1,004 | 882 µs | 1.14 M lines/s |
| 10,004 | 10.1 ms | 990 K lines/s |
| 100,004 | 111 ms | 898 K lines/s |

**The 100k-line file is synthetic.** It is generated from a realistic raster
pattern — long alternating passes of short `G1` moves, a step-over arc between
them, a full-precision coordinate every seventh line — rather than taken from a
CAM post, because a real post's output is somebody's copyrighted part program.
Nobody should read this as "Chipbreaker parses Mastercam at 900k lines a second".

The mild fall-off with size is allocation growth in the segment vector, not
anything super-linear.

### By stage, on the same 20k-line input

| stage | time | throughput | share |
|---|---:|---:|---:|
| lex | 12.7 ms | 1.58 M lines/s | 60% |
| assemble | 1.5 ms | 13.4 M lines/s | 7% |
| lex + assemble + resolve | 21.3 ms | 941 K lines/s | 100% |

**Lexing dominates at 60%**, which is worth knowing before anyone optimises the
resolver. It allocates a `Vec<char>` per line and a `String` per word; both are
straightforward to remove and neither is worth removing until a profile of a
whole simulation
says the parse is on a critical path. Resolution — the arithmetic, the arcs, the
cycles — is about a third.

### Arc forms

| form | time | throughput |
|---|---:|---:|
| I/J/K | 14.7 ms | 681 K arcs/s |
| R | 13.2 ms | 755 K arcs/s |

The `R` form is *faster* by about 10%, which is the opposite of the expectation
that deriving a centre through a square root would cost more than being given
one. The centre-offset path pays for reconciling the given centre against both
endpoints — two `hypot` calls and a projection onto the chord's perpendicular
bisector — and that costs more than one `sqrt`. Not a reason to prefer either;
recorded so the next person does not assume the wrong direction.

### Canned cycle expansion

| | value |
|---|---:|
| input | 5,010 lines |
| time | 13.5 ms |
| throughput | 370 K lines/s |
| expansion ratio | ~4.6 segments per cycle line |

A `G83` line with pecking expands to a rapid, a plunge, and two motions per peck
plus a retract. The ratio is what a field should budget from: a drilling program is
five times more IR than it looks.

### What this measurement changed

1. **183 MB per million segments.** Decide early whether the engine streams the IR or
   holds it. If it holds it, boxing `ArcData` is the obvious 29% saving on
   linear-dominated programs.
2. Lexing is 60% of parse time and is allocation-bound. Only worth attacking if
   parsing shows up in a whole-simulation profile.
3. Cycle expansion multiplies segment count by about 4.6. A drilling-heavy
   program has far more IR than its line count suggests.

---

## 2026-08-03 — root solving and ray versus tool

- **Commit:** `05c8c75` (tool geometry and the root solver complete)
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** rustc 1.96.0 (ac68faa20 2026-05-25), `x86_64-pc-windows-msvc`
- **Profile:** `bench` — `lto = "fat"`, `codegen-units = 1`
- **Criterion:** 1 s warm-up, 3 s measurement. Times are the median of the
  reported confidence interval.

```sh
cargo bench --bench tool -- --warm-up-time 1 --measurement-time 3
```

### Root solving, per solve

A quarter of the corpus has a repeated root deliberately, because that is the
branch that leaves the closed form.

| degree | per solve | relative |
|---|---:|---:|
| quadratic | 11.5 ns | 1x |
| cubic | 94.1 ns | 8.2x |
| quartic | 560.7 ns | **48.8x** |

### Ray against a tool, coherent bundle of 48 x 48

| tool | per ray | throughput | relative |
|---|---:|---:|---:|
| flat (quadratic) | 73.6 ns | 13.6 M/s | 1x |
| drill (quadratic) | 128 ns | 7.8 M/s | 1.7x |
| ball (quadratic) | 142 ns | 7.0 M/s | 1.9x |
| bull (quartic) | 321 ns | 3.1 M/s | 4.4x |
| barrel (quartic) | 1.39 µs | 0.72 M/s | **19x** |

### Per-ray cost with the confounds made visible

Measured separately from Criterion, with the hit rate and the torus share of the
profile printed alongside, because a bare ratio between two tools can be an
artefact of bundle geometry rather than a statement about the solver. Hit rates
are ~60% for every row, so the ratios below are comparable.

| tool | ns/ray | vs flat | torus share |
|---|---:|---:|---:|
| flat 6 mm | 61.1 | 1.00x | 0% |
| drill 6 mm/118 | 75.3 | 1.23x | 0% |
| ball 6 mm | 91.9 | 1.50x | 0% |
| **bull 10 mm r2** | **222.2** | **3.64x** | 5.7% |
| bull 10 mm r0.5 | 195.8 | 3.20x | 1.4% |
| barrel 12 mm R60 | 1246.8 | 20.4x | 38.2% |
| barrel 12 mm R200 | 1323.7 | 21.7x | 54.3% |

**The bull nose is the number that matters and it is 3.6x, not 20x.** A bull nose
is the workhorse of 3-axis finishing and appears in every job; a barrel is a
5-axis specialty that does not appear at all yet.

The mechanism is *not* per-solve conditioning. One quartic costs about the same
either way -- 1350 ns for the bull's torus, 1308 ns for the barrel's. What
differs is **how many rays actually cross a torus**. A bull nose's corner is 2 mm
of a 50 mm tool, so about 94% of rays meet its torus quartic with no real roots
and leave through the cheap no-sign-change path. A barrel's entire cutting length
*is* the torus.

### Correcting the quartic figure above

The 560 ns in the first table understates in-situ cost by about 2.4x. That corpus
uses dyadic roots in [-8, 8]; a real tool geometry quartic costs 0.6 to 1.35 us.
The cause is bracket width. Moving a ray's origin from -100 mm to -7 mm halves
the bull's quartic, 1350 ns to 645 ns, with no change to the algorithm at all:

| torus | ray starts at | ns/solve |
|---|---:|---:|
| bull R=3 rho=2 | -100 | 1350 |
| bull R=3 rho=2 | -20 | 913 |
| bull R=3 rho=2 | -7 | 645 |
| barrel R=194 rho=200 | -100 | 1308 |
| barrel R=194 rho=200 | -20 | 1281 |

Refinement is spending its time bisecting in from a Cauchy bound far wider than
the geometry warrants.

### Correcting the recovery plan

An earlier revision of this file proposed trying Ferrari first and checking each
root's residual, falling back only when the check failed. **That does not work,
and the reason is worth recording.** The failure Ferrari exhibited was reporting
*no* real roots where two exist -- and there is no residual to check for a root
that was never found. On those 41 cases the trigger never fires and the fast path
silently returns an empty set. Residual checking catches inaccurate roots; it is
blind to missing ones, which is the failure mode we actually have.

The sound trigger is a **root count**, available cheaply from machinery already
trusted. For a quartic normalised to a positive leading coefficient, `p` tends to
`+inf` at both ends, so the sign sequence `[+, p(c1), p(c2), p(c3), +]` over the
critical points gives the exact distinct real root count by alternation -- and the
critical points are the roots of `p'`, a cubic, which the repaired cubic solver
handles reliably. If Ferrari's count disagrees, fall back. That is a handful of
Horner evaluations plus one cubic solve.

This is the same phenomenon as the cubic discriminant finding, twice: the
closed-form quantity that is supposed to reveal the root structure is itself
catastrophically cancelled, so the structure has to come from the derivative
instead.

Note also that the bracket-width result above suggests the *larger* win is not a
Ferrari fast path at all. Clipping the initial bracket to the element's own
`t`-range -- which the ray caster knows and the general-purpose solver cannot --
cannot change which roots are found, so it needs no soundness argument.

### Allocation, bull nose, 32 x 32 coherent rays

| form | time | per ray |
|---|---:|---:|
| `intersect_ray` (allocates) | 333.8 µs | 326 ns |
| `intersect_ray_into` (reuses scratch) | 286.1 µs | 279 ns |

A 17% saving, which is smaller than it looks and worth stating plainly: at this
size the allocator is not the bottleneck, the quartic is. The scratch parameter
earns its place in the API on the strength of the field builder's call volume rather than on
this table, and if a later measurement at field-building scale does not show more than this,
the API should lose it.

### Closed-form properties

| operation | flat | ball | bull | barrel |
|---|---:|---:|---:|---:|
| `volume` | 2.72 ns | 24.3 ns | 25.7 ns | 28.6 ns |
| `contains_rz` | 12.6 ns | 21.2 ns | 29.3 ns | 62.4 ns |

`contains_rz` is on the ray caster's hot path — every candidate interval is
classified by testing its midpoint — so the barrel's 62 ns is part of why its
rays cost what they do, though the quartic still dominates.

### Tessellation, bull nose

| tolerance | time |
|---|---:|
| 0.1 mm | 1.10 µs |
| 0.01 mm | 4.27 µs |
| 0.001 mm | 31.1 µs |

Roughly `1/sqrt(tolerance)`, as expected: both the angular divisions and the
chords along an arc scale that way, and the triangle count is their product.

### What this measurement changed

1. Clip the quartic's initial bracket to the element's own `t`-range. Sound by
   construction, and worth up to 2x on its own.
2. If a closed-form fast path is still wanted after that, gate it on the
   **root count** from the critical-point sign sequence, never on residuals.
3. Re-measure the allocation table at real sweep sizes before trusting the 17%
   figure either way.
4. `contains_rz` is called once per candidate interval. If the quartic gets
   cheaper, this becomes the next thing to look at.
5. Barrel cutters are 20x and are nobody's concern yet. Bull noses are 3.6x and
   are not a blocker for building fields or sweeping them.

---

## 2026-08-02 — the mesh pipeline

- **Commit:** `550fcab` plus the completion work (self-intersection, 3MF,
  corpus, this table)
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** rustc 1.96.0 (ac68faa20 2026-05-25), `x86_64-pc-windows-msvc`
- **Profile:** `bench` — `lto = "fat"`, `codegen-units = 1`
- **Criterion:** 1 s warm-up, 3 s measurement; 10 samples for the sizes that run
  in hundreds of milliseconds. Times are the median of the reported interval.

### Parsing

An icosphere of 20,480 triangles, written out and read back.

| format | time | throughput |
|---|---:|---:|
| STL binary | 447.6 µs | **2.13 GiB/s** |
| STL ASCII | 8.032 ms | 773 MiB/s |
| OBJ | 2.677 ms | 332 MiB/s |

Binary STL is 18x faster than ASCII per triangle, which is the expected shape:
one is a `memcpy` with a widening, the other is 180,000 calls to
`str::parse::<f64>`. OBJ's lower byte-rate against a higher triangle-rate is the
indexed format doing less work per byte.

3MF is not in this table. It is a ZIP container, so its throughput is dominated
by `inflate` rather than by anything Chipbreaker does, and timing it would
measure `zlib-rs`.

### Welding, validation and BVH build

Lattice blocks, whose triangle count is `12n²`, at ~10k, ~100k and ~1M.

| operation | 10k | 100k | 1M | Melem/s at 1M |
|---|---:|---:|---:|---:|
| `weld` (from a triangle soup) | 1.634 ms | 18.99 ms | 213.0 ms | 4.71 |
| `validate` (topology only) | 4.381 ms | 50.04 ms | 580.3 ms | 1.73 |
| `Bvh::build` | 2.168 ms | 29.83 ms | 410.4 ms | 2.44 |

All three scale slightly worse than linearly — welding is 11.6x and 11.2x across
decades, validation 11.4x and 11.6x, BVH build 13.8x and 13.8x — which is the
`log n` of the `BTreeMap` and the sort, plus cache pressure. Nothing here is
super-linear in a way that would bite at a whole field's scale.

**Self-intersection is the outlier, and this is why it is opt-in:**

| check | 10k triangles | ratio |
|---|---:|---:|
| `validate` alone | 4.381 ms | 1x |
| `validate` + `check_self_intersections` | 35.99 ms | **8.2x** |

At 10k triangles that is 36 ms, which is fine. It grows with the candidate-pair
count rather than with the triangle count, so on a real part with thousands of
near-touching faces it is minutes. `--check-self-intersect` stays off by default.

### Ray queries

Two meshes of near-identical size so the comparison is about the *geometry* and
not about the tree: an icosphere (5,120 triangles) and a lattice block (5,292).
4,096 rays per batch.

| batch | time | throughput |
|---|---:|---:|
| coherent, generic | 2.316 ms | 1.769 Melem/s |
| incoherent, generic | 2.376 ms | 1.724 Melem/s |
| coherent, lattice, offset origins | 2.524 ms | 1.623 Melem/s |
| coherent, lattice, **origins on the integer lattice** | 39.83 ms | 0.103 Melem/s |

**The headline number for field building is the last row: 15.8x.**

Coherent and incoherent rays differ by only 2.6%, which is worth knowing on its
own — the BVH is not especially rewarding locality at this size, so a ray bundle should not
expect a large win from ray reordering, and correspondingly does not need to
worry about ray order hurting it.

The 15.8x is the cost of Simulation of Simplicity firing. The parity suite
measures the *rate* directly: 3.94% of triangle tests take the exact path on a
generic mesh against **65.80%** on a lattice-aligned one. The predicate benchmarks measured
`orient3d` at 16.9x the filtered path, and 0.658 × 16.9 ≈ 11, so the observed
15.8x is that plus the SoS cascade's own `orient2d` calls. The three
measurements corroborate each other.

**Action taken:** place the dexel ray lattice at cell *centres*, not cell
corners. Rays that miss every vertex and edge cost 2.5 ms per 4,096; rays that
hit them cost 39.8 ms. That is a factor of sixteen on the innermost loop of the
product, available for free by choosing an offset.

It is also the mitigation for the one deviation from strict SoS: a ray coplanar
with a triangle is rejected, and offsetting the lattice makes that case
unreachable for axis-aligned stock.

---

## 2026-08-02 — the numeric core, as a baseline

- **Commit:** `85a6f7e` (`spans` and predicates complete)
- **Machine:** Intel Core Ultra 7 270K Plus, 24 physical / 24 logical cores,
  31.5 GB RAM
- **OS:** Windows 11 Pro 10.0.26200
- **Toolchain:** rustc 1.96.0 (ac68faa20 2026-05-25), `x86_64-pc-windows-msvc`
- **Profile:** `bench` — `lto = "fat"`, `codegen-units = 1`
- **Criterion:** 1 s warm-up, 3 s measurement. Times are the median of the
  reported confidence interval.

### Predicates

Batches of 1024 evaluations; the per-call figure is the batch time divided by
1024.

| Benchmark | Batch time | Per call | Throughput |
|---|---:|---:|---:|
| `orient2d` generic | 1.048 µs | 1.02 ns | 977 Melem/s |
| `orient2d` degenerate | 6.234 µs | 6.09 ns | 164 Melem/s |
| `orient3d` generic | 7.029 µs | 6.86 ns | 146 Melem/s |
| `orient3d` degenerate | 118.8 µs | 116 ns | 8.6 Melem/s |

**Degeneracy cost — the number this benchmark exists to produce:**

| Predicate | Ratio (degenerate / generic) |
|---|---:|
| `orient2d` | **5.9x** |
| `orient3d` | **16.9x** |

Read this as: when the floating-point filter cannot decide and the predicate
escalates to exact expansion arithmetic, `orient2d` costs six times as much and
`orient3d` nearly seventeen times as much.

That is far better than the naive fear (exact arithmetic is not 1000x here) but
it is not free, and it matters during extraction. Dual contouring evaluates predicates on
*grid-aligned* data, where degeneracy is not a rare accident but the common
case — every sample that lands exactly on a cell boundary is degenerate by
construction. If a naive extractor puts most of its predicate calls on
the exact path, `orient3d` alone could dominate the contouring pass. The
mitigation to design in, not discover: perturb the sampling grid off the
canonical lattice, or cache orientation results per cell edge.

### Spans

Two interleaved sets of `n` disjoint spans each. One "element" is one span in
the left operand.

| Operation | n = 10 | n = 100 | n = 1000 | ns per span at n = 1000 |
|---|---:|---:|---:|---:|
| `union` | 177.0 ns | 1.651 µs | 15.82 µs | 15.8 |
| `intersect` | 233.1 ns | 1.999 µs | 19.19 µs | 19.2 |
| `subtract` | 202.3 ns | 1.650 µs | 15.90 µs | 15.9 |
| `subtract_into` | 147.5 ns | 1.529 µs | 15.43 µs | 15.4 |
| `measure` | 1.889 ns | 23.48 ns | 344.7 ns | 0.34 |
| `contains` | 3.026 ns | 5.201 ns | 8.375 ns | — (`O(log n)`) |

Scaling from n = 100 to n = 1000 is 9.6x for `union` and `subtract` against a
10x increase in input size: the merge-scan is linear, as designed, with no
hidden super-linear term.

`subtract_into` versus `subtract` is the allocation saving a real toolpath sweep
gets by keeping one scratch buffer per ray: **27% at n = 10**, falling to 3% at
n = 1000 where the scan itself dominates. Small sets are the common case on a
dexel ray, so this is worth having.

`contains` grows like `log n` (3.0 → 5.2 → 8.4 ns), confirming the binary search.

### `push_merge`

| Insertion order | n = 10 | n = 100 | n = 1000 |
|---|---:|---:|---:|
| ascending (hot path) | 30.32 ns | 165.2 ns | 1.563 µs |
| descending (slow path) | 107.5 ns | 19.61 µs | **1.812 ms** |
| `from_unsorted` | 55.59 ns | 165.9 ns | 1.337 µs |

**The headline risk in this table is the descending row.** Building a
1000-span set by repeatedly inserting *before* the end costs 1.81 ms against
1.56 µs for the same spans inserted in order — a **1160x** penalty, and it grows
quadratically because every out-of-order insert triggers a full re-sort and
re-normalize.

This is documented on `Spans::push_merge`, but documentation is not a defence.
A sweep that happens to walk a dexel ray back-to-front would be a thousand
times slower with no error and no warning, and the symptom would present as
"the dexel field is slow" rather than "the insert order is wrong".

`from_unsorted` is the right answer when order is not guaranteed: at n = 1000 it
is *faster* than ascending `push_merge` (1.34 µs against 1.56 µs), because one
sort of a `Vec` beats a thousand individually-checked appends. Callers that
cannot guarantee order should collect into a `Vec<Span>` and build once.

### What this measurement changed

1. Assert or enforce front-to-back ray traversal in the sweep, rather than
   trusting the convention.
2. Prefer `from_unsorted` over repeated `push_merge` wherever insertion order is
   not structurally guaranteed.
3. Budget for `orient3d` on the exact path before writing the extractor, not during it.
