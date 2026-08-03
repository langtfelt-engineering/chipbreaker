# Chipbreaker

A material-removal simulation and machining verification engine, in Rust.

Given a block of stock, a set of cutting tools and a CNC toolpath, Chipbreaker
answers: **what shape is left at the end, and does it match the part we
intended?** It finds gouges, leftover stock and tool-holder collisions before a
real machine drives into a real workpiece.

> **Status: Unit 3 of 20, in progress.** The numeric foundation, the determinism
> harness, the triangle mesh core with its I/O and BVH, and tool geometry are in
> place: exact geometric predicates, interval spans, canonical hashing, a
> deterministic root solver validated against an exact Sturm oracle, ray casting
> against tool solids of revolution, and native/WASM parity in CI. There is no
> G-code parser and no dexel field yet. See [Roadmap](#roadmap).

## The guarantee

**The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build.**

Neither major incumbent publishes such a guarantee. It is enforced from the first
commit rather than bolted on later, and CI proves it on every push by running the
self-test natively on Windows, Linux and macOS and under `wasmtime`, then
comparing the resulting hashes byte for byte.

The rules that make it hold — `f64` only, no FMA, no unordered iteration reaching
a float, no parallelism without a deterministic partition, exact predicates
instead of float sign tests, canonical *binary* hashing — are written down in
[CONTRIBUTING.md](CONTRIBUTING.md) and enforced in review and in CI.

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

## Building

Requires the toolchain pinned in [`rust-toolchain.toml`](rust-toolchain.toml);
`rustup` will fetch it automatically.

```sh
cargo build --release
cargo test --all
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

## Layout

| Path | Contents |
|---|---|
| `crates/chipbreaker-core` | `math`, `predicates`, `spans`, `golden`, `selftest` |
| `crates/chipbreaker-cli` | the `chipbreaker` binary |
| `tests/corpus` | versioned test inputs, growing every unit |
| `tests/golden` | committed golden hashes |
| `BENCHMARKS.md` | append-only performance record |

The interesting module is [`spans`](crates/chipbreaker-core/src/spans.rs): sorted,
disjoint, normalized sets of half-open intervals, with union, intersection and
difference implemented as a single merge-scan. Every material-removal operation
in the engine bottoms out there, so its tolerance policy is documented at length.

## Roadmap

| Units | Content | Status |
|---|---|---|
| U1 | Workspace, determinism harness, numeric core | **done** |
| U2 | Triangle mesh, I/O, validation, BVH | **done** |
| U3 | Tool and holder geometry, root solver, ray versus solid of revolution | in progress |
| U4 | G-code parser and toolpath IR | |
| U5–U8 | Dexel field, tri-dexel, 3-axis material removal, arcs | |
| U9–U11 | Dual contouring, adaptive resolution, deterministic parallelism | |
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
