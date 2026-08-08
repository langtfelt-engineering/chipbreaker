# Contributing to Chipbreaker

## Code contributions are not open at present

**Chipbreaker is not accepting code contributions from outside Langtfelt.**

The reason is licensing rather than a judgement about anyone's patch.
Chipbreaker is dual-licensed — GPL-3.0-or-later plus a commercial licence — and
the commercial licence requires clean copyright title to the whole work. Holding
that title means every contribution must arrive under a signed Contributor
Licence Agreement, and a single commit without one taints the commercial
offering permanently. The CLA process is not in place yet, so the honest answer
is to say no rather than to accept patches we could not use.

**Issues, bug reports, questions and discussion are very welcome.** A reproducer
for a wrong answer is worth more to this project than a patch, and there is no
paperwork attached to filing one.

If you want to use Chipbreaker in a proprietary product, or to discuss
contributing under a CLA once one exists, write to
[licensing@langtfelt.com](mailto:licensing@langtfelt.com).

The rest of this document is the standard the code is held to. It is public
because it explains *why* the code looks the way it does, and because anyone
evaluating the engine should be able to check that the rules are real.

---

## The determinism invariant

Chipbreaker guarantees that **the same input produces bit-identical output across
runs, thread counts, platforms, and the WASM build**. It is the product's
commercial differentiator, and it is the single constraint that most shapes how
this code is written.

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

### 4. Parallelism only behind a deterministic partition

No bare `rayon`, no ad-hoc `std::thread::spawn`. Where cutting runs on many
threads it does so behind a partitioning scheme in which each partition's result
is combined in a **fixed order regardless of completion order**. Work assignment
may be dynamic; value combination may not.

Adding parallelism any other way destroys the invariant and hides the damage
until much later, when the only symptom is that two runs of the same job
disagree in the last few bits.

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
WASM builds, and the cross-target parity job is the only thing that would ever
tell you.

`NaN` collapses to one canonical payload and `-0.0` to `+0.0`, because two runs
that are numerically identical must hash identically.

### 7. `#![forbid(unsafe_code)]`

In every crate. If you need to do something the safe subset will not let you do,
that is a design conversation, not a patch.

Note the knock-on effect: `std::env::set_var` is `unsafe` in edition 2024, so
configuration is threaded through as values — see `golden::GoldenStore`, which
takes its root directory rather than reading the environment on every call.

### 8. Transcendentals come from `transcendental`, never from `std`

`f64::sqrt` is fine, and is the only exception: IEEE-754 requires it to be
correctly rounded, so it is bit-identical everywhere.

`sin`, `cos`, `tan`, `atan2`, `exp`, `ln` and friends are **not**. No standard
requires them to be correctly rounded, so the platform libm on x86-64 Linux, the
one on macOS, and the one WASM gets are all entitled to differ by an ULP — and
they do.

So every one of them goes through
[`chipbreaker_core::transcendental`](crates/chipbreaker-core/src/transcendental.rs),
which is the pure-Rust [`libm`](https://crates.io/crates/libm) crate, pinned by
exact version and compiled from the same source for every target. Same source,
same arithmetic, same bits.

`std`'s versions are **banned mechanically**, not by review:
[`clippy.toml`](clippy.toml) lists each in `disallowed-methods` with the
replacement named in the error message. A few — `exp2`, `exp_m1` — have no
reproducible replacement yet and are simply refused; if you need one, add it to
`transcendental` rather than reaching for `std`.

The `math` module exposes no trigonometric constructors, for the same reason: a
`Mat3::from_rotation` would be a supported way to make the guarantee false.

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

## Architecture decision records

Decisions that outlive the code that implements them live in [`docs/adr/`](docs/adr/),
one file each, numbered. A decision belongs there when reversing it later would
look like a reasonable simplification to somebody who was not present for it —
which is precisely when a comment in the code is not enough.

**The index is [the table in the README](README.md#decisions)**, and there is
exactly one of it. This document used to carry a second copy, which fell three
entries behind while sitting under a rule telling contributors to keep it up to
date — a good demonstration of why a duplicated index is worse than none.

Add the new file *and* a row in the README. An ADR nothing links to is a file
nobody reads.

## Fixtures that cross a serialization boundary

**Every fixture that is written out and read back must contain at least one
value requiring all 17 significant digits.**

This is not a style preference. `serde_json`'s default float parser reads
`2.0481555856608242` as `2.048155585660824` — one ULP low — and the tool library
fixture failed to notice for months because every tool in it was a flat,
a ball or a bull nose, whose coordinates are `3.0` and `20.0` and survive any
parser ever written. There was no bit to lose, so no test could lose it.

A parser that drops a bit on the *first* read is stable ever after, so a
write-read-write comparison agrees with itself while the data is already wrong.
Only a full-precision value can detect it, and only against the originally
constructed value.

Applies to: tool libraries, toolpath IR, NC corpus expectations, report schemas,
machine configuration. When adding a fixture, take the awkward number from real
geometry — `1 + 20 tan(3 deg)`, `sqrt(200^2 - 194^2)` — rather than inventing a
bit pattern, and assert in the test that it still needs 17 digits so a later edit
cannot quietly turn the check back into a test of `3.0`.

`crates/chipbreaker-core/tests/float_roundtrip_guard.rs` holds the standing check
that `serde_json` itself has not regressed.

## Tests

- `cargo test --all` must pass on Windows, Linux and macOS.
- `cargo clippy --all-targets -- -D warnings` must be clean.
- Property tests use a fixed, documented seed. A CI failure has to reproduce
  locally on the first try.
- Long-running fuzz tests are `#[ignore]`d and run nightly, not on every commit.
- New geometry code comes with cases in `tests/corpus/`, which is versioned and
  grows with the engine.

### A test that asserts an invariant must carry evidence it can fail

**Show that the assertion breaks when the invariant does.** Not in the pull
request description — in the repository, as something that runs.

A passing test that cannot fail is worse than a missing one, because a missing
test looks like a gap and a vacuous one looks like coverage. Two got through in a
single release:

- Counting placeholder normals to prove normals were set. `PLACEHOLDER` **is**
  `+Z` — the four-byte encoding has no reserved pattern, deliberately — so every
  up-facing endpoint of an uncut box already counted as one.
- Asserting that a slot's cut endpoints did not all share a normal. They did not:
  the *outer stock* faces carried correct normals from construction, and only the
  cut faces were wrong.

Both passed for a long time while every cut face in the engine carried
`(0, 0, -1)`.

Any of these counts as evidence, in rough order of preference:

1. A test in the same file that runs the check against a deliberately broken
   input and asserts it fails. `tests/sweep_normals.rs` does this: it substitutes
   the exact normal the defect produced and asserts the comparison rejects it.
2. A floor on the number of things checked, so a filter that quietly matches
   nothing fails instead of passing.
3. An oracle computed **independently of the code under test**. Comparing a
   sweep's stored normal against the function the sweep itself calls is a
   spelling check; comparing it against the Minkowski-sum geometry is a test.

If none of the three is practical, say so in the test's own documentation and say
what would make it possible.

## What a green CI run proves, and what it does not

Two cadences, and the difference is worth knowing before you trust a green tick.

### Every push

| job | covers |
|---|---|
| `test (ubuntu / windows / macos)` | fmt, clippy, the full test suite, benchmarks compile, self-test |
| `wasm parity` | the same self-test under `wasmtime` on `wasm32-wasip1` |
| `cross-platform parity` | all four `results` hashes compared byte for byte, per suite |
| `determinism rules` | no `f32`, FMA, `HashMap`, or ad-hoc threads; SPDX headers; golden file format |
| `corpus regenerates identically` | every generated corpus file matches its generator |
| `dependency licences` | `cargo deny` |

**All four targets gate every push.** This was Linux-only for a while, on a
billing argument that stopped applying when the repository became public.

### Nightly only

- **The two full-corpus statistical sweeps.** `every_case_injects_the_defect_it_claims`
  walks all 295 corpus cases, and `recall_against_depth` builds two fields and
  contours one per sampled case. 96 and 140 seconds respectively in the debug
  build `cargo test` uses; 15 and 18 in the release build nightly uses.
- **The fuzz suites**, against deliberately hostile input.

### So what does a green push prove?

**It proves the answer is identical on four targets**, that every rule in this
document holds, and that nothing in the fast suite regressed — including the
sign convention, the false-positive floor, the deviation ladder, and the
corpus's *identity*, which is pinned by digest and is what both empty-case bugs
would have tripped.

**It does not prove the recall figure still holds.** That is measured nightly.
A change that leaves every case injecting correctly but degrades detection would
pass a push and fail the following morning.

If you are about to publish a number from the recall curve, run the sweeps
yourself rather than trusting the last green tick:

```sh
cargo test --release --all -- --ignored --nocapture
```

## Commit style

Small, logical commits. One deliverable per commit is about right. Explain *why*
in the body — the diff already says what.
