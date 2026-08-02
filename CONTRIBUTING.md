# Contributing to Chipbreaker

## Before you open a pull request

**External pull requests cannot be merged until a Contributor Licence Agreement
is in place.** Chipbreaker is dual-licensed — GPL-3.0-or-later plus a commercial
licence — and the commercial licence requires clean copyright title to the whole
work. Without a signed CLA we cannot relicense your contribution, and a single
un-CLA'd commit taints the commercial offering permanently.

We are sorry about the friction. It is unavoidable and it is much cheaper to
handle before you write the patch than after.

> `TODO(legal)`: publish the CLA text and the signing process, and name the legal
> entity that holds copyright. Until then, every `.rs` file carries
> `Copyright (C) 2026 Chipbreaker Contributors` as a placeholder. Issues,
> discussion and bug reports are welcome now; code contributions are on hold.

---

## The determinism invariant

Chipbreaker guarantees that **the same input produces bit-identical output across
runs, thread counts, platforms, and the WASM build**. Neither major incumbent
publishes such a guarantee; it is the product's commercial differentiator, and it
is the single constraint that most shapes how this code is written.

It is enforced from the first commit rather than retrofitted, because retrofitting
it later would mean auditing every floating-point operation in the codebase.

The rules below are not style preferences. A violation is a bug of the same
severity as a wrong answer, because it *is* a wrong answer on somebody else's
machine.

### 1. `f64` everywhere

No `f32` in the core. Ever. Not for storage, not for interchange, not "just for
the display path". CI greps for it.

**One narrow exception**, because reality intrudes: binary STL *stores* `f32`,
so reading and writing that format has to name the type. Those lines carry an
`ALLOW-f32-WIRE-FORMAT` marker comment and CI skips them. The marker is
deliberately ugly and deliberately per-line: it makes each exception show up in
review rather than widening the rule to a whole file or module.

The exception is for the **wire format only**. A value read from an `f32` is
widened to `f64` immediately — which is exact, since every `f32` is an `f64` —
and no arithmetic ever happens at single precision. If you find yourself wanting
the marker on a line that computes something, the answer is no.

Note the knock-on effect this has on tolerances: `f32`'s 24-bit mantissa gives
about 6e-6 mm of resolution at a 100 mm coordinate, which is why
[`EPS_WELD`](crates/chipbreaker-core/src/eps.rs) is 1e-6 mm and not finer. A weld
lattice below the incoming noise floor fails to merge vertices that really are
the same point.

### 2. No FMA

Never call `f64::mul_add`, and never write anything that invites the compiler to
contract a multiply-add. Rust does not auto-contract, and we will not do it by
hand: a fused multiply-add rounds once where a separate multiply and add round
twice, and the hardware FMA that native x86-64 uses is not the software path that
WASM takes.

Write `a * b + c * d`. If a profiler ever makes this look expensive, the answer is
a better algorithm, not a fused instruction.

Clippy's `suboptimal_flops` and `imprecise_flops` lints suggest `mul_add`; both
are set to `allow` at the workspace root for exactly this reason.

### 3. No unordered iteration that can reach a float

Use `BTreeMap` and `BTreeSet`, not `HashMap` and `HashSet`, anywhere the iteration
order could influence a floating-point result — which includes any accumulation,
any "pick the first match", and any hash input.

Where a hash map is genuinely the right structure for pure lookup, document at the
declaration why its order cannot escape. "It's only used for lookup" is a claim
that has to be re-checked every time the surrounding code changes, so write down
the argument.

Sums have a documented order. `Spans::measure` sums ascending by `t0`; `Vec3::dot`
accumulates `x`, then `y`, then `z`; `Mat3::determinant` expands along row 0.
Floating-point addition is not associative, so "the obvious order" must be *the
written-down order*.

### 4. No parallelism (yet)

No `rayon`, no threads, no `std::thread::spawn`. Parallelism arrives in U11 behind
a deterministic partitioning scheme where each partition's result is combined in
a fixed order regardless of completion order. Adding it before then destroys the
invariant and hides the damage until much later.

### 5. Exact predicates, never raw float sign tests

Any question of the form "which side of this is that on" goes through
`chipbreaker_core::predicates`. Never `if determinant > 0.0`.

`Vec3::cross` and friends exist for computing directions and magnitudes. The sign
of one of their components is not an answer to a geometric question.

The predicates are exact only within a documented coordinate range
(`CoordRange`), which narrows as the determinant's degree rises. The bounds are
*measured*, not derived — see
`published_coord_ranges_are_inside_the_measured_exact_band`.

### 6. Canonical binary serialization for hashing

Never hash text. `f64` hashes as its little-endian IEEE-754 bit pattern via
`f64::to_le_bytes`, through `golden::CanonicalHash`. Text formatting of floats is
for humans and never feeds a hash.

`usize` is widened to `u64` before hashing. WASM is a 32-bit target; without the
widening, any hash containing a length or an index differs between the native and
WASM builds, and nobody notices until U19.

`NaN` collapses to one canonical payload and `-0.0` to `+0.0`, because two runs
that are numerically identical must hash identically.

### 7. `#![forbid(unsafe_code)]`

In every crate. If you need to do something the safe subset will not let you do,
that is a design conversation, not a patch.

Note the knock-on effect: `std::env::set_var` is `unsafe` in edition 2024, so
configuration is threaded through as values — see `golden::GoldenStore`, which
takes its root directory rather than reading the environment on every call.

### 8. Transcendental functions are not yet safe

`f64::sqrt` is fine: IEEE-754 requires it to be correctly rounded, so it is
bit-identical everywhere.

`sin`, `cos`, `tan`, `atan2`, `exp`, `ln` and friends are **not**. They are
correctly rounded by no standard, and the native platform libm and the Rust
`libm` used for WASM differ by an ULP on some inputs. The `math` module
deliberately exposes no trigonometric constructors for this reason.

This becomes a real problem at U16, where 5-axis kinematics needs rotations. The
fix is a vendored, bit-reproducible implementation — decided and landed *before*
the first rotation matrix is built, not after.

---

## Golden files

Committed digests live in `tests/golden/*.hash`, one lower-case hex digest per
file, LF-terminated, marked binary in `.gitattributes`.

To accept a change:

```sh
CHIPBREAKER_ACCEPT_GOLDEN=1 cargo test
```

**Any commit that touches a golden file must explain why in the commit message.**
A golden hash changes for exactly two kinds of reason: a deliberate change to what
we compute, or a determinism bug. The commit message is where you tell the next
person which one it was. "Update goldens" is not an explanation.

If you find yourself accepting a golden file you did not expect to change, stop.
That is the harness working.

## Changing a pinned dependency

`robust`, `blake3` and `rand` are pinned with `=` in the workspace manifest
because their output feeds a hash we publish as a guarantee. A patch release that
changes a random stream or a digest silently invalidates every golden file.

To bump one: change the pin, run the full suite on all three platforms *and*
under `wasmtime`, and say in the commit message what changed and why the new
goldens are correct.

The same applies to `rust-toolchain.toml`. Codegen changes between compiler
releases are a real source of last-bit drift.

## Tolerances

Every tolerance lives in `chipbreaker_core::eps` with a written rationale and its
units. A bare `1e-9` at a call site will be rejected in review: it carries no
units, no justification, and no way to audit what happens when the workspace scale
changes.

## Tests

- `cargo test --all` must pass on Windows, Linux and macOS.
- `cargo clippy --all-targets -- -D warnings` must be clean.
- Property tests use a fixed, documented seed. A CI failure has to reproduce
  locally on the first try.
- Long-running fuzz tests are `#[ignore]`d and run nightly, not on every commit.
- New geometry code comes with cases in `tests/corpus/`, which is versioned and
  grows every unit.

## Commit style

Small, logical commits. One deliverable per commit is about right. Explain *why*
in the body — the diff already says what.
