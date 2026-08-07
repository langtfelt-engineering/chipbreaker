# ADR 0005: Volume is a diagnostic. Deviation is the metric.

- **Status:** accepted
- **Date:** 2026-08-03
- **Unit:** 6 (tri-dexel field), amended at Unit 9
- **Binds:** U9 (surface extraction), U12, U13 (customer-facing accuracy claims)
- **Related:** [ADR 0001](0001-spans-arena.md), [ADR 0004](0004-dexel-binary-format.md)

## The rule

**Material volume is a construction-time diagnostic. Maximum surface deviation
is the metric that accuracy assertions and customer-facing claims are made
against.**

Do not reintroduce a volume-based accuracy assertion. It is not a weaker version
of a deviation assertion; it is a different quantity with three properties that
make it unfit, every one of them measured rather than argued.

## Why: volume fails three times, and none of them is a bug

### 1. It does not fall monotonically

Unit 5 measured a cylinder whose axis runs along the bundle:

| h/R | rays | relative volume error |
|---:|---:|---:|
| 1/80 | 25,600 | 1.90e-4 |
| 1/160 | 102,400 | **4.39e-4** |

**Four times the rays, more than twice the error.** This is not a defect. For
that solid the chord length is a hard indicator — full height inside the
silhouette, zero outside — so the volume is *exactly*
`h^2 * H * (lattice points inside the disc)`, and the volume error is *exactly*
the error in counting lattice points inside a disc. That is the Gauss circle
problem, whose error term oscillates; the bound `(h/R)^(2 - 131/208)` bounds it
and does not describe its decay.

The smooth solids fail the other way. Their **signed** error crosses zero inside
the tested range, so `|error|` dips wherever the crossing happens to land and
reads as superconvergence. Cone and torus R=10 are both non-monotone for that
reason.

Underneath both is one cause: volume is a **global integral**, so boundary
errors of opposite sign cancel. A field can be more wrong everywhere and report
a better volume.

### 2. It floors out against tessellation

Also Unit 5, on a sphere:

| h/R | vs mesh | vs analytic | tessellation floor |
|---:|---:|---:|---:|
| 1/10 | 2.251e-3 | 1.708e-3 | 5.419e-4 |
| 1/40 | 2.099e-4 | 3.321e-4 | 5.419e-4 |
| 1/320 | 1.626e-6 | 5.403e-4 | 5.419e-4 |

The dexel error falls three orders of magnitude. The error a customer would
actually see *rises* and then parks on the mesh's own error. Below about
`h/R = 1/40` a finer field buys nothing measurable in volume, so volume cannot
distinguish a good tri-dexel field from a mediocre one on ordinary corpus
geometry.

### 3. It carries a quantisation bias that jumps discontinuously

Found at Unit 6, and the simplest of the three to state.

Every cell claims a full `h^2` of cross-section. When the spacing does not
divide the transverse extent, `ceil` produces cells that stick out past the
stock — and because the lattice is centred (see the U6 amendment in
`dexel::lattice`), their ray centres are still *inside* the stock, so each
reports a full chord. The volume is over-counted by exactly
`covered area / true area`.

A 30x20x10 mm box at 1.6 mm cells:

| bundle | cells | covered mm² | true mm² | reported | truth | bias |
|---|---|---:|---:|---:|---:|---:|
| X | 13x7 | 232.96 | 200 | 6988.8 | 6000 | 1.165x |
| Y | 7x19 | 340.48 | 300 | 6809.6 | 6000 | 1.135x |
| Z | 19x13 | 632.32 | 600 | 6323.2 | 6000 | 1.054x |

**A 16.5% volume error on a plain box, from arithmetic rather than sampling**,
and it jumps discontinuously every time `ceil` steps. It vanishes exactly when
the spacing divides the extents, which is why Unit 5 never saw it: every test
there used a spacing that happened to divide.

Deviation is untouched by this. The rays are in the right places and their
endpoints are exact; only the *area attributed to each ray* is quantised, and
deviation never attributes area to anything.

### Does averaging three bundles rescue it? Measured at U6: no.

Three independent oscillating terms might have cancelled. The upright cylinder,
relative error against truth:

| h/R | X bundle | Y bundle | Z bundle | mean of three |
|---:|---:|---:|---:|---:|
| 1/40 | 4.23e-4 | 4.23e-4 | 4.07e-4 | **1.46e-4** |
| 1/80 | 1.60e-4 | 1.60e-4 | 1.90e-4 | **1.70e-4** |
| 1/160 | 5.18e-5 | 5.18e-5 | 4.39e-4 | **1.81e-4** |
| 1/320 | 2.37e-5 | 2.37e-5 | 1.62e-5 | 2.12e-5 |

The mean rises across two successive refinements. Averaging a diagnostic three
times gives a better diagnostic, not a metric.

## Why deviation works

Maximum surface deviation is a **supremum over points**, not an integral. It has
no cancellation: a region that is badly sampled shows up at full size, and
cannot be offset by a region that is badly sampled the other way. It has no
oscillating lattice-counting term, because it never counts cells — it measures
distances.

And it is bounded, by the theorem Unit 6 delivers. For a plane with normal `n`
sampled by a bundle along `d` with cells of size `h`, the perpendicular
deviation is about `(h/2) * sin(theta)` where `theta` is the angle between `n`
and `d`. Over three orthogonal axes,

```
max( |n.x|, |n.y|, |n.z| )  >=  1/sqrt(3)
```

(if all three were below, the squares would sum to less than one), so the best
axis has `sin(theta) <= sqrt(2/3)` and

```
best-of-three deviation  <=  (h/2) * sqrt(2/3)  ~=  0.408 * h
```

Linear in `h`, with a constant, and monotone. That is a statement a customer can
act on: **a finer simulation is a safer one.** Volume cannot support that
sentence, and Unit 5's cylinder is the counterexample.

## What volume is still for

Kept, reported, and useful — as a **diagnostic**, which is a different job:

- A gross construction fault (inverted winding, a bundle that found nothing)
  shows up in volume immediately and cheaply.
- Per-bundle volumes that disagree by far more than `O(h^2)` mean something is
  wrong with a bundle, not with the metric.
- It is the one number that is cheap on every field, so it is what `dexel stat`
  leads with.

`TriDexelField::volume` says so in its own documentation, so a reader who arrives
by autocomplete rather than by this document still gets told.

## What this rules out

- **Asserting that the three bundles agree on volume.** They will disagree at
  `O(h^2)` with independent signs. Demanding tight agreement is demanding that
  three independent errors coincide, which is a test of luck. The U6 plan
  originally contained this and it was removed.
- **Reporting a single "accuracy" figure derived from volume** in U12 or U13.
- **Tightening a volume tolerance to make a test pass.** If a volume check is
  failing, the question is what changed geometrically, and the deviation harness
  is what answers it.

## Consequences

- U6's assertions are on best-of-three deviation: monotone in `h`, bounded by
  `C * h`.
- U9's extracted surface is judged the same way, against the same harness.
- U12 and U13 quote deviation, with the cell size and the mesh's own
  tessellation error alongside — an accuracy number without the ratio it was
  measured at is not a claim about anything.
- The tessellation floor is a first-class part of any accuracy statement. Unit 6
  records the estimate in `.tdx` provenance so a field carries the evidence with
  it, and `dexel build` warns when the requested cell size is finer than the
  input mesh supports.


## Amendment, Unit 9: every accuracy metric floors against its input

Volume was rejected above partly because it **floors out against tessellation**.
Unit 9 found the same floor in deviation, which is the metric that replaced it,
and the coincidence is not one — it is a general property that deserves stating
once rather than being rediscovered per unit.

**Any accuracy metric floors against the fidelity of its input. Past that point
you are no longer measuring the pipeline; you are measuring the mesher.**

Measured at Unit 9, extracting a subdivision-4 icosphere of radius 8 whose facets
are about 0.4 mm across:

| cell size | rms deviation from the ideal sphere |
|---:|---:|
| 0.8 mm | 0.012246 mm |
| 0.4 mm | 0.003464 mm |
| 0.2 mm | **0.005297 mm** |

The last row rises, and nothing is wrong. At 0.2 mm the grid has become finer
than the source mesh's own facets, so it is faithfully reproducing a faceted
polyhedron while the measurement compares against an ideal sphere. The number it
reports is the tessellation's error, not the field's. With a subdivision-5 sphere
all three rungs stay above the floor: 0.015229, 0.003099, 0.000851.

### What this binds

- **Unit 9 onward**, any convergence assertion must keep the grid coarser than
  the input geometry, or it is asserting something about the input.
- **Unit 12**, whose deviation fields compare an extracted mesh against a nominal
  part, inherits this twice over: once for the stock mesh and once for the
  nominal.
- **Unit 13's error budget** must say it in customer-facing terms. A customer
  supplying a coarse STL and asking for findings at 0.01 mm is making exactly
  this mistake in a different costume, and the honest answer is that the tolerance
  they can be given is bounded below by the tolerance their input carries.
