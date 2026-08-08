# Chipbreaker

A material-removal simulation and machining verification engine, in Rust.

Given a block of stock, a set of cutting tools and a CNC toolpath, Chipbreaker
answers: **what shape is left at the end, and does it match the part you
intended?** It finds gouges and leftover stock before a real machine drives into
a real workpiece. Tool-holder collision detection is next; see
[Roadmap](#roadmap).

> **Status: the verification layer has just landed.** Chipbreaker simulates a
> whole 3-axis NC program end to end — build a tri-dexel field from a stock mesh,
> cut it with linear moves, arcs and helices across a multi-tool program, contour
> the result back to a watertight mesh — and then **compares it against the part
> you meant**, reporting gouges and leftover stock as two separate numbers. On as
> many threads as you have, with the answer unchanged at every thread count.
>
> 500,000 segments over a 100 × 60 × 20 mm field at 0.5 mm takes **43 seconds**
> when every move is level and **103 seconds** when every move ramps. Extraction
> runs at about 1.7 M triangles/second and holds a working set of a few
> megabytes regardless of field size.
>
> Underneath: exact geometric predicates, interval spans, canonical binary
> hashing, a deterministic root solver validated against an exact Sturm oracle,
> ray casting against tool solids of revolution, an RS-274 parser producing a
> canonical toolpath IR, closed-form swept volumes for every motion case that has
> one, manifold dual contouring, and a memory ceiling that predicts a job's
> footprint exactly and refuses over-budget work before allocating. Native/WASM
> parity is proved in CI across all of it.
>
> A program plunged one millimetre too deep reports exactly `1.0000 mm` of
> gouge over 1102 samples and exits non-zero; the same program a millimetre
> shallow reports one millimetre of excess and passes, because material left
> standing is what a roughing pass is for.
>
> **Not there yet:** turning a field of deviations into a short list of
> *findings* a machinist can act on, tool-holder collision detection, and
> multi-setup work. See [Roadmap](#roadmap).

## Scope

What Chipbreaker does **not** do, collected here so nobody spends an afternoon
finding out. None of these is a bug report.

| Out of scope | Status |
|---|---|
| **5-axis and any tilted tool** | planned, last — see [Roadmap](#roadmap) |
| **Turning, mill-turn, lathe work** | not planned. The workpiece does not rotate; the field is a static solid the tool is subtracted from |
| **Cutter radius compensation (`G41`/`G42`)** | **refused by design**, see below |
| **Siemens 840D and Heidenhain Klartext** | refused, detected by name |
| **Macro and parametric programming, `o`-word subprograms** | refused, detected by name |
| **Flutes, helix angle, rake, relief, tooth count** | not modelled, and never will be |
| **A GUI** | not planned. Library plus CLI; a browser demo will consume the library, never be part of it |

The dialect Chipbreaker reads is RS-274 in its Fanuc-style form. Everything else
is **detected and named** rather than approximated, and that distinction is the
point:

> `G41` is the sharpest case. Simulating the uncompensated path produces a part
> that is wrong by the tool radius *everywhere*, and it looks entirely
> reasonable. A verification tool that is quietly wrong is worse than one that
> says it cannot answer.

So a Siemens program does not fail with "unexpected character at line 3"; it
fails with "this is Siemens 840D, which is a different language, not a dialect of
this one". Somebody who feeds the wrong file to the wrong parser has made a
category error, and a syntax error will not tell them so.

**Flutes are worth one more sentence**, because their absence looks like a gap
and is not. Material removal is determined by the tool's swept envelope, which is
a surface of revolution: a two-flute and a four-flute cutter of the same diameter
and corner radius remove exactly the same material. Modelling flutes would add no
accuracy and would turn every ray intersection from a quartic into something with
no closed form at all.

## The guarantee

**The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build.**

It is enforced from the first commit rather than bolted on later, and CI checks
it by running the self-test natively on Windows, Linux and macOS and under
`wasmtime`, then comparing the resulting hashes byte for byte. All four currently
agree on a single hash over 15 suites and 27,039 cases, at every thread count.

**All four targets run and gate on every push.** A commit that has only been
checked on one platform has not been checked.

The rules that make it hold — `f64` only, no FMA, no unordered iteration reaching
a float, no parallelism without a deterministic partition, exact predicates
instead of float sign tests, canonical *binary* hashing — are written down in
[CONTRIBUTING.md](CONTRIBUTING.md) and enforced in review and in CI.

The guarantee is load-bearing in ordinary use, not just a marketing line. Cutting
a program in one pass, in two halves split at a segment boundary, or in batches
of any size gives the same field and the same reported volume, bit for bit — so
`--segment-range` really does reproduce what the full job did to those segments,
and `--batch-size` is a performance knob that can never change an answer.

## Why a dexel field, not a B-rep kernel

The engine represents the workpiece as a **tri-dexel field**: three orthogonal
bundles of parallel rays through the workspace, each ray storing a sorted set of
disjoint intervals describing where material exists along that line. Cutting is
interval subtraction. Surface extraction is dual contouring back to a triangle
mesh.

This is a deliberate choice over a NURBS B-rep kernel. B-rep requires solving
surface–surface intersection robustly, which is the problem that has killed every
small-team CAD kernel attempt. A dexel field has no such problem: every ray is
independent, and every intersection is one-dimensional interval arithmetic.

### What that buys, and what it costs

The field is **exact along each ray and sampled across it**. Span endpoints are
continuous positions, not lattice points, so cutting is interval arithmetic on
exact intersection parameters rather than a resampling — a thousand cuts
accumulate no error. What the cell size quantises is *which rays exist*, never
*where a surface sits along one*.

That single asymmetry explains most of the engine's measured behaviour, and it
cuts both ways:

- A surface parallel to a bundle is invisible to it, which is why there are three
  bundles. No plane can be near-parallel to all three at once: the worst case is
  the body diagonal, where the best bundle still meets the surface at 54.74°.
- Accuracy is anisotropic. It depends on the ratio of cell size to the smallest
  feature that matters, not on the cell size alone, so the CLI refuses to default
  it.
- A 0.25 mm field can still tell a 1 µm change in toolpath geometry from a 10 µm
  one, because that change moves endpoints and endpoints are continuous. Grid
  resolution and toolpath resolution are independent; a coarse field does not
  blur away a difference the program actually contains.

### Every accuracy figure floors against its input

A deviation or volume figure is a statement about Chipbreaker only while the cell
size is coarser than the geometry it was given. Below that the grid reproduces
the source mesh's own facets faithfully, and the number reported is the mesher's
error rather than the engine's — measured, when surface extraction was built, as an rms deviation that
*fell* from 0.0122 mm to 0.0035 mm and then *rose* to 0.0053 mm once the grid
passed the source's 0.4 mm facets.

So a coarse STL puts a floor under any tolerance that can be claimed from it.
See [ADR 0005](docs/adr/0005-deviation-not-volume.md).

### Volume is a diagnostic; deviation is the metric

Removed volume is reported per bundle and never averaged, and accuracy claims are
never made against it. Volume is non-monotone under refinement, floors out
against tessellation, and carries a cell-quantisation bias — all three measured,
not assumed. The reasoning is in
[ADR 0005](docs/adr/0005-deviation-not-volume.md), and the surprising part is
that the volume error of an axis-parallel cylinder is *exactly* the error in
counting lattice points inside a disc, which is the Gauss circle problem.

## Building

Requires the toolchain pinned in [`rust-toolchain.toml`](rust-toolchain.toml);
`rustup` will fetch it automatically.

```sh
cargo build --release
cargo test --workspace
```

Run the deterministic self-test:

```sh
cargo run --release -p chipbreaker-cli -- selftest
cargo run --release -p chipbreaker-cli -- selftest --report json --out report.json
cargo run --release -p chipbreaker-cli -- version --json
```

`selftest` exits 0 when every suite passes and 1 otherwise. Its JSON report has
two sections: `results`, which is deterministic and carries its own canonical
hash, and `environment` — toolchain, target triple, timings, host CPU — which is
excluded from that hash. If timings leaked into the hashed section, every CI run
would disagree with every other one.

Check native/WASM parity locally:

```sh
cargo build --release --target wasm32-wasip1 -p chipbreaker-cli
wasmtime target/wasm32-wasip1/release/chipbreaker.wasm selftest --report json
```

The `results.hash` field must match the native run exactly.

## Simulating a program

Build a field from a stock mesh, then cut it with an NC program:

```sh
chipbreaker dexel build stock.stl --units mm --res 0.4 --axes xyz --out stock.tdx

chipbreaker run --stock stock.tdx \
                --path part.nc \
                --tools library.json \
                --out cut.tdx
```

`run` resolves each `T` number against the library, so a program that changes
tools simply works. It reports what it did rather than only that it finished:

```text
program   arc-full-circle-ijk, segments 0..5 of 5
tool      flat-6 (overriding every T number)
method    closed form where exact, otherwise bounded at 0.04 mm (a tenth of the cell size)
cases     1 horizontal, 3 plunge, 0 ramp, 1 arc, 0 helix, 0 stationary
batching  5 motions in 1 run(s) of at most 32, bit-identical at every size
removed   2042.767764864333 mm^3 (mean of three bundles; per bundle [2069.9116472966193, 2069.9116472966216, 1988.4799999999352])
rays      8680 tested, 66320 rejected (88.427% rejection), 4828 changed
sweep     8680 of 8680 ray-cuts exact (100.00%), 0 sub-stepped over 0 steps
          worst deviation bound 0 mm, and it applies ONLY to the sub-stepped ones
digest    29f6520c1aaaf70751514c6095e54ec4bf7bcefae1b5be98f24802d55bc2fa76
```

Three lines there are the whole design philosophy. **`rays`** shows that 88% of
the field was never examined — the box rejection, not the inner loop, is what
decides whether a 500,000-segment job takes minutes or days. **`sweep`** splits
exact ray-cuts from sub-stepped ones, because a mixed job's worst-case bound
belongs only to the segments that earned it; quoting one number for the whole
program would be a claim about work that carried no sweep error at all.
**`removed`** is given per bundle and the three disagree by design, which is why
it is labelled a diagnostic rather than a measurement.

Useful flags:

| Flag | Purpose |
|---|---|
| `--reference` | dense sub-stepping instead of the closed forms — the slow, obviously-correct path, exposed so a doubted result can be reproduced |
| `--max-swept-error MM` | absolute deviation bound; defaults to a tenth of the cell size |
| `--segment-range A:B` | simulate only segments `A..B`, for debugging one bad move |
| `--no-arc-native` | replace arcs with chords, as many CAM posts do |
| `--batch-size N` | traversal tuning only; the result is bit-identical at every value |

### Motion cases

Each segment is classified, and the classification decides whether the swept
volume is computed in closed form or sub-stepped with a bound. Nothing is
declared exact unless it is *provably* exact — a rule that exists because
violating it once produced a quarter-turn arc swept with a single sample of the
tool, silently ([ADR 0006](docs/adr/0006-arc-closed-form-scope-and-batch-invisibility.md)).

| Case | Geometry | Treatment |
|---|---|---|
| Stationary | no motion | exact — the static tool |
| Horizontal | `dz = 0` | exact — three-piece decomposition |
| Plunge | `dxy = 0` | exact — moving maximum of the profile |
| Arc | level `G17` arc | exact — the Case A′ collapse |
| Ramp | both non-zero, **and every `G18`/`G19` arc** | sub-stepped, with a bound |
| Helix | an arc that rises | sub-stepped, with a bound |

The step count for the sub-stepped cases comes from the **helical** path length,
never the chord: on a typical helix a chord under-states the path by 20.8%, so a
chord-derived bound would claim an accuracy the sweep does not have.

## Verifying a program

`run` says what shape is left. `compare` says whether it is the right shape.

```sh
chipbreaker compare cut.tdx --nominal part.stl --units mm --tolerance 0.5
```

```text
field      cut.tdx
nominal    part.stl
samples    11820
tolerance  0.5000 mm (floor 0.4000 mm: stock 0.0000, nominal 0.0000, lattice 0.4000)

GOUGE      worst 1.0000 mm over 1102 samples
EXCESS     worst 0.0000 mm over 0 samples
rms        0.2927 mm

verdict    GOUGED above tolerance
```

Both blocks above are real output with the file paths shortened; nothing else is
edited. The program is a 6 mm flat mill through a 24 x 18 x 10 mm block, plunged
one millimetre below the nominal channel floor.

**The two signs are never blended into one number.** A gouge is unambiguous:
material that should be there is not, and nothing downstream puts it back.
Excess stock is usually *expected* — it is what a roughing pass is supposed to
leave. Only gouges decide the exit code. A tool that failed a correct roughing
pass would be switched off within a day.

`--tolerance` is checked against the floor before it is used. Asking for 0.01 mm
against a visibly faceted nominal is refused, with the three inputs named and
which one is the limit, because "refine your mesh" without saying which mesh is
not actionable. `--allow-below-floor` overrides it deliberately.

### Two rulers, and why both are printed

`deviation-stat` gives the distribution behind the verdict: bands by depth, a
split by bundle, and the worst samples with coordinates.

```text
worst 2 samples:
    -1.0000 mm at (  21.000,   26.000,   25.000)  normal ( 0.000,  0.000,  1.000)  axis 1  perpendicular -5.0000
    -1.0000 mm at (  21.000,   26.200,   25.000)  normal ( 0.000,  0.000,  1.000)  axis 2  perpendicular -1.0000
```

The first sample sits on the corner where a channel wall meets the gouged floor,
and the two readings disagree by a factor of five. That is not a bug in either:

- **Surface distance**, the metric, is the distance to the nearest point of the
  nominal. It is what `d_H` is defined as.
- **Perpendicular distance** is the same thing measured along the surface normal,
  by casting a ray. At a step edge that ray leaves along the floor's normal,
  passes the wall beside it and strikes the top face five millimetres away.

The perpendicular reading is an upper bound and nothing more. It is published
beside the metric rather than discarded, and `worst_projection_gap_mm` reports
their largest disagreement — which is large exactly where a perpendicular number
describes the measurement instead of the part.

### The corpus is the oracle

There is no ground truth for "useful", so recall is measured against **295
injected defects**: eight kinds of operator error, seven locales, ten depths from
a fifth of a cell to eight cells, each perturbing exactly one segment by a known
amount at a known place.

| depth | recall |
|---|---|
| below ½ cell | 80% |
| ½ cell and above | **100%** |

Gouges invented on a correctly machined part: **none**.

A corpus like that is only an oracle if its cases genuinely contain what they
claim, and twice they did not — a locale anchored on the stock surface, and a
rapid clearing above it, each leaving cases that sat in the denominator and could
never be found. Every case is now checked, before it counts, by a measurement
that shares no code with the thing being tested: the Hausdorff distance between
the two fields' span sets, ray by ray, with no mesh, extraction, normal or
containment test anywhere in it. All 295 inject; the weakest reaches 94% of the
depth it claims.

## Layout

| Path | Contents |
|---|---|
| `crates/chipbreaker-core` | `math`, `predicates`, `transcendental`, `eps`, `spans`, `roots`, `mesh`, `tool`, `toolpath`, `dexel`, `sweep`, `contour`, `deviation`, `defect`, `budget`, `golden`, `selftest` |
| `crates/chipbreaker-gcode` | RS-274 parser: the only place that reads G-code text |
| `crates/chipbreaker-cli` | the `chipbreaker` binary |
| `docs/adr` | architecture decisions, and the measurements behind them |
| `tests/corpus` | versioned test inputs, grown alongside the engine |
| `tests/golden` | committed golden hashes |
| `BENCHMARKS.md` | append-only performance record |

Four modules are worth reading first.
[`spans`](crates/chipbreaker-core/src/spans.rs) holds sorted, disjoint,
normalized sets of half-open intervals, with union, intersection and difference
implemented as a single merge-scan; every material-removal operation in the
engine bottoms out there, so its tolerance policy is documented at length.
[`roots`](crates/chipbreaker-core/src/roots.rs) solves the polynomials that every
ray-versus-tool intersection reduces to, and its header explains why a double
root gives eight digits rather than sixteen — the fact that governs the
tolerances in both modules.
[`sweep`](crates/chipbreaker-core/src/sweep/mod.rs) computes swept volumes, one
module per motion case, each differential-tested against a dense reference.
[`dexel/tri`](crates/chipbreaker-core/src/dexel/tri.rs) is the field itself, and
carries the sampling theorem that justifies three bundles.

Module headers carry the reasoning and the measurements, not just the interface;
several of them record a wrong answer that was tried first and why it failed,
because that is usually the more useful half.

### Decisions

| ADR | Subject |
|---|---|
| [0001](docs/adr/0001-spans-arena.md) | Span storage and the chunked spill arena |
| [0002](docs/adr/0002-branch-protection.md) | Branch protection and required checks |
| [0003](docs/adr/0003-toolpath-ir-coordinate-frame.md) | Toolpath IR and its coordinate frame |
| [0004](docs/adr/0004-dexel-binary-format.md) | Why `.dexel` is binary and not text |
| [0005](docs/adr/0005-deviation-not-volume.md) | Volume is a diagnostic; deviation is the metric |
| [0006](docs/adr/0006-arc-closed-form-scope-and-batch-invisibility.md) | The arc closed form's scope, and batching's invisibility |
| [0007](docs/adr/0007-no-local-refinement.md) | A dexel ray is global, so local refinement is not available |
| [0008](docs/adr/0008-simd-is-autovectorisation-only.md) | SIMD means autovectorisation; intrinsics are ruled out |
| [0009](docs/adr/0009-debt-register.md) | The rolling debt register, closed |
| [0010](docs/adr/0010-unset-normals-are-not-distinguishable.md) | An unset normal is not distinguishable from `+Z`, and stays that way |

## Roadmap

| Capability | Status |
|---|---|
| Determinism harness, exact predicates, canonical hashing | **done** |
| Triangle mesh, I/O, validation, BVH | **done** |
| Tool and holder geometry, root solver, ray versus solid of revolution | **done** |
| G-code parser and toolpath IR | **done** |
| Dexel field, `.dexel` format, convergence measurement | **done** |
| Tri-dexel field, the sampling theorem, deviation harness | **done** |
| 3-axis material removal: linear moves | **done** |
| Arcs, helices, motion batching | **done** |
| Dual contouring to a watertight mesh | **done** |
| Memory ceiling, anisotropic resolution | **done** |
| Deterministic parallelism | **done** |
| Deviation fields: `compare`, the injected-defect corpus | **done** |
| Gouge classification, collision detection, multi-setup | next |
| WASM target and browser demo | planned |
| Commercial packaging: C ABI, bindings, evaluation kit | planned |
| Assurance package: SBOM, signed reproducible builds, error-budget specification | planned |
| 5-axis kinematics and tilted swept volumes | last, see below |

5-axis is last, and deliberately so. It is the largest single remaining piece of
work — orientation-aware kinematics, and swept volumes for a tool that tilts as
it moves, which has no closed form and needs bounded sub-stepping throughout. The
engine is built for it: the toolpath IR already carries an `orientation` field
that stays `None`, and the bounded sub-stepping machinery it needs was built and
validated for ramps and helices. Sequencing it last means the 3-axis verification
layer ships complete and proven rather than broad and thin.

Chipbreaker ships as a **library plus CLI**. There is no GUI in the core and there
will not be one; everything must be exercisable from the command line. The
eventual browser demo is a consumer of the library, never part of it.

## What this is for

Stock simulation — showing the shape a program leaves — is commoditised. It ships
with every CAM system and sells standalone from about ten dollars a month.
Chipbreaker is not aimed there.

The target is **assurance-grade verification**: not only an answer, but an answer
you can put in front of someone who has to sign for it. Reproducible execution,
tolerances stated rather than implied, machine-readable evidence, traceable
versions, and approximation that is bounded and says so. The exact predicates,
the canonical hashing and the cross-target parity all exist to make that claim
supportable rather than aspirational.

### What a deviation bound does and does not cover

A bound from this engine covers the distance between the **computed stock** and
the **ideal geometric cutting model**. That is all it covers.

It says nothing about tool wear, deflection under load, thermal growth, spindle
runout, backlash, or how a particular controller interpolates between the points
it was given. A part can match the simulation exactly and still be out of
tolerance for any of those reasons. Chipbreaker verifies the *program*, not the
machine and not the part.

That distinction is kept in the code, the documentation and the output, and it is
not modesty — a verification tool that lets a customer believe it covers physics
it never modelled is worse than one that admits its scope.

## Licence

Copyright (C) 2026 Langtfelt. Dual-licensed:

- **GPL-3.0-or-later** — see [LICENSE](LICENSE). Use it, study it, modify it,
  redistribute it, on the terms of the GPL.
- **Commercial** — for use in a proprietary product, where the GPL's terms do
  not suit. Write to
  [licensing@langtfelt.com](mailto:licensing@langtfelt.com).

**Code contributions are not open at present**, for licensing reasons set out in
[CONTRIBUTING.md](CONTRIBUTING.md). Issues, bug reports and questions are
welcome — a reproducer for a wrong answer is worth more here than a patch.

For anything with a security dimension, including an input that makes the engine
report a gouged part as clean, see [SECURITY.md](SECURITY.md) rather than opening
a public issue.
