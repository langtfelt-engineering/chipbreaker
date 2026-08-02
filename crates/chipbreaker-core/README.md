Numeric core for **Chipbreaker**, a material-removal simulation and machining
verification engine.

This crate provides the four foundations every later unit is built on:

- [`math`] — `f64` vector, matrix, AABB and ray types with no external linear
  algebra dependency.
- [`predicates`] — adaptive-precision exact geometric predicates behind a
  swappable trait, returning an explicit three-valued [`predicates::Orientation`]
  rather than a float sign.
- [`spans`] — sorted, disjoint, normalized sets of half-open intervals on a
  line, with union / intersection / difference implemented as a single
  merge-scan. Every material-removal operation in the engine bottoms out here.
- [`golden`] — the bit-exact determinism harness: canonical binary hashing and
  golden-file comparison.

## The determinism invariant

The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build. That guarantee is enforced from the first commit,
not retrofitted. In practice it means: `f64` only, no FMA, no unordered
iteration that can reach a float, no parallelism without a deterministic
partition, exact predicates instead of raw float sign tests, and canonical
*binary* serialization for hashing. See `CONTRIBUTING.md` at the repository root
for the full rules.
