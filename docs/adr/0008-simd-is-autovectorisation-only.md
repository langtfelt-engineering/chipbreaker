# ADR 0008: SIMD means autovectorisation, and hand-written intrinsics are ruled out

- **Status:** accepted
- **Date:** 2026-08-07
- **Governs:** any future performance work on the hot loops
- **Related:** [ADR 0006](0006-arc-closed-form-scope-and-batch-invisibility.md)

## The rule

**Vectorisation is obtained from LLVM by structuring the loops for it. Hand-written
SIMD intrinsics are not deferred — they are ruled out while
`#![forbid(unsafe_code)]` stands.**

The original plan promised "SIMD interval operations". This narrows that to
autovectorisation, and the two reasons are worth stating precisely so nobody
reopens it on the assumption it was merely unfinished.

## Why not intrinsics

### 1. They require unsafe, and the rule is worth more than the speedup

`core::arch` intrinsics are `unsafe` and `std::simd` is nightly-only. Every crate
in this workspace carries `#![forbid(unsafe_code)]`, which is a claim a customer
can check mechanically and an auditor can trust without reading the code.
Spending that for a constant factor on one loop is a bad trade at any point in
this project, and a worse one now that the determinism claim rests on the same
discipline.

### 2. Runtime CPU dispatch is a determinism hazard

The usual way to ship intrinsics is to detect CPU features at run time and
choose a code path. That is directly hostile to the guarantee this unit exists to
defend: if AVX-512 is chosen on one machine and AVX2 on another, and any
horizontal reduction differs in width, the two produce different bits. It would
fail on *some* hardware, silently, and the cross-target parity job would not
necessarily catch it — CI runs on the runners GitHub gives us, not on the
customer's machine.

A determinism claim that holds except on some CPUs is not a determinism claim.

## Why autovectorisation is safe for determinism

This is the part worth writing down, because it looks like it should be a hazard
and is not.

**Rust does not enable fast-math.** LLVM is therefore forbidden from
reassociating floating-point operations: it may not turn `(a + b) + c` into
`a + (b + c)`, because for IEEE-754 those are different values and without
`reassoc` fast-math flags LLVM must preserve the written order.

The consequence:

- **Elementwise operations vectorise freely and are bit-exact.** Four `f64`
  subtractions done in one instruction are the same four IEEE-754 subtractions,
  each independent. Nothing is reordered because nothing depends on order.
- **Horizontal reductions are the exception.** A vectorised sum accumulates into
  lanes and combines them at the end, which *is* a reassociation. LLVM will not
  do this to `f64` without fast-math, so the risk is not that it happens
  silently — it is that someone later writes a reduction expecting it to
  vectorise, finds it does not, and reaches for a flag or an intrinsic to force
  it.

**Every reduction in this engine stays scalar and in fixed order**, which the
parallel reduction pass already requires for its own reasons. The two constraints
coincide, and that is not a coincidence: both are the same fact about
floating-point addition.

## What this means in practice

- Structure hot loops over contiguous slices, without early exits in the inner
  body and with bounds checks hoisted, so LLVM can see the shape it needs.
- **Measure the effect.** Autovectorisation is a hope, not a guarantee: it turns
  on and off with inlining decisions, and a refactor that looks neutral can
  silence it. A claimed gain that was never benchmarked is worth nothing.
- Do not add `-C target-cpu=native` to the shipped build. It would vectorise more
  and produce binaries whose numerics depend on the build machine, which is the
  same hazard as runtime dispatch wearing a different hat.

## Consequences

- No `unsafe` enters the workspace for performance reasons.
- Cross-target parity does not depend on every target choosing the same
  instruction set, only on every target obeying IEEE-754 — which they do.
- If a hot loop is genuinely bounded by arithmetic throughput and
  autovectorisation will not take, the answer is a better algorithm or more
  threads, not intrinsics. Every profile so far has found the former
  available: the box rejection, batching, the slab sweep and the counting-sort
  reduction were each worth more than any constant factor a vector unit offers.
