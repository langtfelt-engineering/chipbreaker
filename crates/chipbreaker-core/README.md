# chipbreaker-core

The geometry and determinism core of
[Chipbreaker](https://github.com/spanwerk/chipbreaker), a material-removal
simulation and machining verification engine.

This crate holds everything that is not G-code parsing or the command line:

- **Exact geometric predicates** with Simulation of Simplicity, so a ray through
  a shared edge or a vertex is decided by sign information rather than by
  rounding.
- **Interval spans** — the sorted, disjoint interval algebra every cut is
  expressed in.
- **Triangle meshes**, STL/OBJ/3MF readers, validation, welding, and a BVH whose
  ray casting is leak-free by construction.
- **Tool geometry**: solids of revolution, a deterministic root solver, and ray
  intersection against cylinders, cones, spheres and tori.
- **Tri-dexel fields**, material removal for linear moves, arcs and helices, and
  dual contouring back to a watertight mesh.
- **Deviation fields**: comparing a simulated result against the part it was
  meant to be.
- `#![forbid(unsafe_code)]`, and a canonical binary hashing harness that makes
  the determinism guarantee checkable rather than asserted.

## The guarantee

**The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build.** `f64` only, no FMA, no unordered iteration that
can reach a float, transcendentals from a pinned pure-Rust `libm` rather than the
platform's, and every parallel reduction combined in a fixed order.

The rules and the reasoning behind each are in
[CONTRIBUTING.md](https://github.com/spanwerk/chipbreaker/blob/main/CONTRIBUTING.md).

## A note on the tests

Many of the integration tests read fixtures from `tests/corpus/` at the
repository root, which is outside this package. They are shipped because they are
a large part of the argument for why the code is the way it is — but to *run*
them, clone the repository rather than the crate.

## Licence

Copyright (C) 2026 Langtfelt. Dual-licensed: GPL-3.0-or-later, or a commercial
licence for use in a proprietary product. See the
[repository](https://github.com/spanwerk/chipbreaker) for details.
