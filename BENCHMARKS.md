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
```

CI compiles the benchmarks (`cargo bench --no-run`) but does not time them.
Timing on shared CI runners produces numbers with more variance than the
regressions we would be looking for.

---

## 2026-08-02 — Unit 2: mesh pipeline

- **Commit:** `550fcab` plus the Unit 2 completion work (self-intersection, 3MF,
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
super-linear in a way that would bite at U5's scale.

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

**The headline number for U5 is the last row: 15.8x.**

Coherent and incoherent rays differ by only 2.6%, which is worth knowing on its
own — the BVH is not especially rewarding locality at this size, so U5 should not
expect a large win from ray reordering, and correspondingly does not need to
worry about ray order hurting it.

The 15.8x is the cost of Simulation of Simplicity firing. The parity suite
measures the *rate* directly: 3.94% of triangle tests take the exact path on a
generic mesh against **65.80%** on a lattice-aligned one. Unit 1 measured
`orient3d` at 16.9x the filtered path, and 0.658 × 16.9 ≈ 11, so the observed
15.8x is that plus the SoS cascade's own `orient2d` calls. The three
measurements corroborate each other.

**Action for U5:** place the dexel ray lattice at cell *centres*, not cell
corners. Rays that miss every vertex and edge cost 2.5 ms per 4,096; rays that
hit them cost 39.8 ms. That is a factor of sixteen on the innermost loop of the
product, available for free by choosing an offset.

It is also the mitigation for the one deviation from strict SoS: a ray coplanar
with a triangle is rejected, and offsetting the lattice makes that case
unreachable for axis-aligned stock.

---

## 2026-08-02 — Unit 1 baseline

- **Commit:** `85a6f7e` (Unit 1, `spans` and predicates complete)
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
it is not free, and it matters at U9. Dual contouring evaluates predicates on
*grid-aligned* data, where degeneracy is not a rare accident but the common
case — every sample that lands exactly on a cell boundary is degenerate by
construction. If a naive U9 implementation puts most of its predicate calls on
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
A U5 sweep that happens to walk a dexel ray back-to-front would be a thousand
times slower with no error and no warning, and the symptom would present as
"the dexel field is slow" rather than "the insert order is wrong".

`from_unsorted` is the right answer when order is not guaranteed: at n = 1000 it
is *faster* than ascending `push_merge` (1.34 µs against 1.56 µs), because one
sort of a `Vec` beats a thousand individually-checked appends. Callers that
cannot guarantee order should collect into a `Vec<Span>` and build once.

### Action items carried into U5

1. Assert or enforce front-to-back ray traversal in the sweep, rather than
   trusting the convention.
2. Prefer `from_unsorted` over repeated `push_merge` wherever insertion order is
   not structurally guaranteed.
3. Budget for `orient3d` on the exact path before U9, not during it.
