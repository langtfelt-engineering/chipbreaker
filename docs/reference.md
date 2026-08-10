# Accuracy and performance reference

Everything an evaluator asks in the first meeting, on one page: how accurate,
how fast, and what the numbers do not cover. Each figure says where it came
from, because a number without a method is an advertisement.

Measurements are on x86-64 Windows, `rustc 1.96.0`, release profile, unless
stated. Absolute times move between machines; the **ratios** are the part that
transfers.

---

## 1. What a deviation bound covers

**A bound is `d_H(computed stock, ideal geometric cutting model)`.** It is the
Hausdorff distance between what the engine computed and what the programmed
toolpath would remove from a perfect machine.

It says nothing about tool wear, deflection under cutting load, thermal growth,
spindle runout, backlash, workholding deflection, or how a particular control
interpolates between programmed points. **A part can match this bound exactly
and still be out of tolerance for any of those reasons.**

Chipbreaker verifies the program, not the machine and not the part. Every report
carries this in its own `exclusions` and `scope` fields, so a reader six months
later does not have to have read this page.

---

## 2. Accuracy

### 2.1 The measured quantity is surface distance, and two rulers are published

- **Surface distance** — the distance to the nearest point of the nominal. This
  is what `d_H` is defined as, and it is what the verdict uses.
- **Perpendicular distance** — the same thing read along the surface normal, by
  casting a ray. An **upper bound and nothing more**: at a step edge the ray
  leaves along the floor's normal, passes the wall beside it and strikes a face
  five millimetres away.

Both are published. `worst_projection_gap_mm` reports their largest
disagreement, which is large exactly where the perpendicular number is
describing the measurement instead of the part.

### 2.2 Detection floor, against 295 injected defects

There is no ground truth for "useful", so recall is measured against a corpus of
**295 injected defects**: eight kinds of operator error, seven locales, ten
depths from a fifth of a cell to eight cells, each perturbing exactly one
segment by a known amount in a known place.

| defect depth | recall |
|---|---|
| below ½ cell | 80% |
| ½ cell and above | **100%** |

Gouges invented on a correctly machined part, across that corpus: **none**. See
KI-1 in [known-issues.md](known-issues.md) for a configuration outside the
corpus where the engine does invent one — a stock mesh whose minimum corner sits
exactly on the grid origin.

Every case is verified to contain what it claims **before it counts**, by a
measurement sharing no code with the thing under test: the Hausdorff distance
between the two fields' span sets, ray by ray, with no mesh, extraction, normal
or containment test anywhere in it. All 295 inject; the weakest reaches 94% of
the depth it claims. This check exists because twice the corpus contained cases
that could never have been found, sitting silently in the denominator.

Reference: `tests/corpus/defect/expectations.json`.

### 2.3 Why volume is not the accuracy metric

Volume is a **global integral**, so boundary errors of opposite sign cancel: a
field can be more wrong everywhere and report a better volume. On a cylinder,
four times the rays gave *more than twice* the volume error —

| h/R | rays | relative volume error |
|---:|---:|---:|
| 1/80 | 25,600 | 1.90e-4 |
| 1/160 | 102,400 | **4.39e-4** |

— because that solid's volume error is exactly the error in counting lattice
points inside a disc, the Gauss circle problem, whose error term oscillates.

Volume also **floors out against tessellation**. On a sphere, below about
`h/R = 1/40` a finer field buys nothing a customer would see:

| h/R | vs mesh | vs analytic | tessellation floor |
|---:|---:|---:|---:|
| 1/10 | 2.251e-3 | 1.708e-3 | 5.419e-4 |
| 1/40 | 2.099e-4 | 3.321e-4 | 5.419e-4 |
| 1/320 | 1.626e-6 | 5.403e-4 | 5.419e-4 |

Full reasoning: [ADR 0005](adr/0005-deviation-not-volume.md).

### 2.4 The tolerance floor

A tolerance finer than the coarsest input describes the inputs rather than the
part. The engine computes that floor from the stock facet error, the nominal
facet error and the cell size, reports it as
`numerical_semantics.tolerance_floor_mm`, and sets `below_floor` when the
requested tolerance is under it. `--allow-below-floor` overrides deliberately.

### 2.5 Sweep error, per run

Reports state how many ray-cuts were computed **in closed form** and how many
were **sub-stepped**, with the worst bound among the sub-stepped ones only:

```json
"swept_volumes": {
  "available": true, "ray_cuts_exact": 72716, "ray_cuts_bounded": 0,
  "worst_bound_mm": 0.0,
  "worst_bound_applies_to": "the sub-stepped ray-cuts only, never the whole run"
}
```

Quoting a single worst bound for a mixed program would be a claim about segments
that carried no sweep error at all.

### 2.6 Composition across setups

Bounds across a multi-setup job compose by **plain sum, not root-sum-square**.
The boundaries are independent samplings of the same solid whose worst cases can
line up, so the pessimistic sum is the honest one.

An axis-aligned re-fixture contributes **exactly zero**: a 90° rotation is a
signed permutation with entries 0 and ±1, exact in `f64`, and rays map onto rays.
Verified against a directly-built reference over 40,000 normals with zero
difference. An oblique re-fixture resamples, and the report says which regime
each boundary used and what it cost.

---

## 3. Performance

### 3.1 Collision checking is amortised across a job, not paid per tool

The question a shop actually asks. Same total cutting work, split one way and
the other, 12 passes over an 80 × 50 × 30 block at 0.7 mm spacing:

| | time | vs cutting alone |
|---|---:|---:|
| cutting alone, 12 passes | 191.8 ms | — |
| checked, 1 tool × 12 passes | 314.0 ms | **+64%** |
| checked, 2 tools × 6 passes | 304.1 ms | +59% |
| checked, 4 tools × 3 passes | 264.3 ms | +38% |

Flat to falling as the tool count rises, so the overhead is **amortised across
the job**. If it were per-tool the last row would be roughly four times the
first. That is the difference between a check somebody leaves switched on and
one they switch off.

Source: `cargo bench --bench job`.

### 3.2 Crossing a setup boundary

| | time |
|---|---:|
| axis-aligned re-fixture | 1.153 ms |
| rebuilding the same field from the mesh | 91.4 ms |

**79× cheaper**, because the axis-aligned path is a relabelling: no rays are
cast and no intersections are solved, so it scales with the number of spans
rather than the volume of the field.

### 3.3 Comparison against a nominal

Per sample, which is what transfers between jobs:

| cell | samples | nominal triangles | total | per sample |
|---|---:|---:|---:|---:|
| 0.80 mm | 7,260 | 14,440 | 86.3 ms | 11.9 µs |
| 0.50 mm | 18,496 | 36,992 | 167.7 ms | 9.1 µs |
| 0.35 mm | 38,530 | 77,060 | 358.8 ms | 9.3 µs |

### 3.4 Findings scale with what is wrong, not with the part

| cut | samples | above tolerance | findings | time |
|---|---:|---:|---:|---:|
| correct | 28,960 | 666 | 1 | 129 µs |
| 0.5 mm deep | 29,306 | 6,035 | 1 | 1.63 ms |
| 2.0 mm deep | 30,690 | 7,419 | 1 | 2.15 ms |

### 3.5 Parsing

1.14 M lines/s on synthetic raster surfacing. The toolpath IR costs **192 bytes
per `MotionSegment`**, so a million-segment program is 183 MB before any field
exists — which is why the memory check runs after parsing and before the field
is built.

### 3.6 The browser build

| | |
|---|---|
| module size | 1.0 MB |
| cold run, including the one-off self-test | 2441 ms |
| warm run | 87 ms |
| memory ceiling | 256 MiB, mandatory |
| segment cap | 20,000 |

The self-test dominates the cold figure, so a page should run it at worker
start rather than on the visitor's first click.

---

## 4. Determinism

The same inputs produce **bit-identical output** across runs, thread counts,
platforms and WebAssembly. Enforced, not asserted:

- `f64` throughout; no `f32` outside the binary STL wire format
- no FMA, no `mul_add`
- no `std` transcendentals — a libm-backed module, enforced by a clippy
  `disallowed-methods` list
- no unordered iteration reaching a float: `BTreeMap` and `BTreeSet` only
- exact predicates rather than raw float sign tests
- canonical binary hashing

The self-test digest is identical on **five targets** — Linux, macOS, Windows,
`wasm32-wasip1` and `wasm32-unknown-unknown` — and CI fails if any of them
diverges:

```
17 suites, 27,054 cases
1ccc6660b31f67ad5092a0248a89fc1c38cf1b9c2cf1fe5e4c022803f689c038
```

A build sharing that digest will produce the same answers for the same inputs.
It identifies the build's **behaviour**, not the build.

---

## 5. Where these numbers live

| | |
|---|---|
| append-only performance record | [`BENCHMARKS.md`](../BENCHMARKS.md) |
| architecture decisions and their measurements | [`docs/adr/`](adr/) |
| defect corpus and expected recall | `tests/corpus/defect/expectations.json` |
| evaluation corpus with expected outputs | `tests/corpus/eval/expectations.json` |
| known defects | [`known-issues.md`](known-issues.md) |
| what the version numbers promise | [`versioning.md`](versioning.md) |
