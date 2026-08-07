# Chipbreaker

A material-removal simulation and machining verification engine, in Rust.

Given a block of stock, a set of cutting tools and a CNC toolpath, Chipbreaker
answers: **what shape is left at the end, and does it match the part we
intended?** It finds gouges, leftover stock and tool-holder collisions before a
real machine drives into a real workpiece.

> **Status: Unit 8 of 20 complete.** Chipbreaker can now simulate a whole
> 3-axis NC program end to end: build a tri-dexel field from a stock mesh, cut it
> with linear moves, arcs and helices across a multi-tool program, and write the
> result back out. 500,000 segments over a 100 × 60 × 20 mm field at 0.5 mm takes
> **43 seconds** when every move is level and **103 seconds** when every move
> ramps.
>
> Underneath: exact geometric predicates, interval spans, canonical binary
> hashing, a deterministic root solver validated against an exact Sturm oracle,
> ray casting against tool solids of revolution, an RS-274 parser producing a
> canonical toolpath IR, and closed-form swept volumes for every motion case that
> has one. Native/WASM parity is proved in CI across all of it.
>
> **Not there yet:** surface extraction. The field knows where material is; it
> cannot yet hand you a mesh of the cut part. That is Unit 9. See
> [Roadmap](#roadmap).

## The guarantee

**The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build.**

Neither major incumbent publishes such a guarantee. It is enforced from the first
commit rather than bolted on later, and CI proves it on every push by running the
self-test natively on Windows, Linux and macOS and under `wasmtime`, then
comparing the resulting hashes byte for byte. All four currently agree on a
single hash over 13 suites and 20,482 cases.

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
error rather than the engine's — measured, at Unit 9, as an rms deviation that
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

## Layout

| Path | Contents |
|---|---|
| `crates/chipbreaker-core` | `math`, `predicates`, `transcendental`, `eps`, `spans`, `roots`, `mesh`, `tool`, `toolpath`, `dexel`, `sweep`, `golden`, `selftest` |
| `crates/chipbreaker-gcode` | RS-274 parser: the only place that reads G-code text |
| `crates/chipbreaker-cli` | the `chipbreaker` binary |
| `docs/adr` | architecture decisions, and the measurements behind them |
| `tests/corpus` | versioned test inputs, growing every unit |
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

## Roadmap

| Units | Content | Status |
|---|---|---|
| U1 | Workspace, determinism harness, numeric core | **done** |
| U2 | Triangle mesh, I/O, validation, BVH | **done** |
| U3 | Tool and holder geometry, root solver, ray versus solid of revolution | **done** |
| U4 | G-code parser and toolpath IR | **done** |
| U5 | Dexel field, `.dexel` format, convergence measurement | **done** |
| U6 | Tri-dexel field, the sampling theorem, deviation harness | **done** |
| U7 | 3-axis material removal: linear moves | **done** |
| U8 | Arcs, helices, motion batching | **done** |
| U9 | Dual contouring to a watertight mesh | **done** |
| U10–U11 | Rectilinear graded resolution, deterministic parallelism | next |
| U12–U15 | Deviation fields, gouge classification, collision detection, multi-setup | |
| U16–U18 | 5-axis kinematics, tilted swept volumes, error-bounded sub-stepping | |
| U19–U20 | WASM target and demo, commercial packaging (C ABI, Python bindings) | |

Chipbreaker ships as a **library plus CLI**. There is no GUI in the core and there
will not be one; everything must be exercisable from the command line. The
eventual browser demo is a consumer of the library, never part of it.

## Licence

Dual-licensed:

- **GPL-3.0-or-later** — see [LICENSE](LICENSE).
- **Commercial** — for use in proprietary products. `TODO(legal)`: contact
  details once the legal entity is registered.

Contributions require a Contributor Licence Agreement; see
[CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request.
