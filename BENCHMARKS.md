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
