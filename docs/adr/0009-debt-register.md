# ADR 0009: The rolling debt register, closed

- **Status:** accepted
- **Date:** 2026-08-07
- **Unit:** 12 (deviation field)
- **Binds:** nothing new; it disposes of items carried since Units 3, 5 and 11
- **Related:** [ADR 0007](0007-no-local-refinement.md), [ADR 0008](0008-simd-is-autovectorisation-only.md)

## Why this exists

Six items have been carried forward across units as "deferred". Deferred twice is
where an item quietly becomes forgotten: it stops being a decision anyone made
and becomes a thing nobody looked at. Each gets a disposition here, and ambient
tracking stops.

Three outcomes are used. **Declined** means it will not be done and the reasoning
is recorded so it is not silently reopened. **Carried** means it is still wanted,
with the condition that would trigger it. **Open** means it should be done and is
now scheduled.

---

## 1. `ArcData` boxing — **carried**

*Raised U5, deferred to U10, never done.*

`MotionSegment` carries `Option<ArcData>` inline. Arcs are 3.26% of segments in
the corpus, so the payload is dead weight in 97% of them and costs about 24% of
the toolpath IR's memory.

**Carried, not declined**, because the number that would trigger it is now known
precisely. Unit 10 measured the IR at 192 bytes per segment; boxing would take it
to roughly 146. On a three-million-segment program that is 550 MiB against 418 —
a real difference, and one the memory ceiling would report accurately either way.

**Trigger:** a customer program where the IR is the binding constraint in a
`mem-estimate` refusal. Until then the win is real and small, and the churn
touches every consumer of `MotionSegment`.

## 2. Ferrari fast path for barrel tools — **declined**

*Raised U3, deferred to U11.*

Barrel and toroidal cutters reach the quartic solver, which Unit 8 measured at
one solve per ray cast — about 20× the cost of a quadratic.

**Declined.** Barrel tools matter at Unit 20, and Phase F is conditional on
commercial evidence that may never arrive. Building a numerically delicate fast
path for a case that may never ship is the wrong order of work, and Unit 3
already established that Ferrari's method "destroys wide-magnitude quartics" —
the recovery would need its own oracle.

**Reopen if:** the 5-axis gate opens, or a customer profile shows barrel tools on
a hot path.

## 3. Autovectorisation measurement — **declined**

*Raised U11.*

[ADR 0008](0008-simd-is-autovectorisation-only.md) scoped SIMD to
autovectorisation and required the effect be measured rather than assumed. The
measurement was not taken.

**Declined as stated**, and the distinction matters: this is a **benchmark gap,
not a correctness gap**. Nothing claims a vectorisation speedup, so nothing
unmeasured is being asserted. ADR 0008's substantive content — that intrinsics
are ruled out, and why autovectorised `f64` is bit-exact without fast-math —
stands on its own and needed no benchmark.

**Reopen if:** a customer profile shows a hot loop that is arithmetic-bound. Then
the measurement is the first step, not the last.

## 4. Bundle-level parallelism past 16 threads — **carried**

*Raised U11.*

Efficiency falls to about 50% at 16 threads on a 24-core host. The remaining
serial work is the arena write-back and the per-bundle reduction, both
`O(changed rays)`. The three bundles are fully independent and could run
concurrently, tripling the parallel work per scope.

**Carried.** It is a real and identified next step, not a mystery. The reason it
is not urgent: a 500,000-segment job already runs in 43 seconds, and no customer
has yet asked for it to be 20 seconds.

**Trigger:** a job where wall time is the complaint, on a machine with more than
16 usable cores.

## 5. 64 KiB per worker, unmeasured — **carried**

*Raised U11.*

`BYTES_PER_WORKER` is a deliberately generous estimate, not a measurement. It is
charged to the memory ceiling and so is *conservative in the safe direction*: an
over-estimate refuses a job that would have fitted, which is annoying, where an
under-estimate would let one through that then fails.

**Carried.** Measuring it needs a high-span job — a ribbed pocket where rays
carry many spans — which the corpus does not yet contain and Unit 12's
injected-defect work may produce as a side effect.

**Trigger:** a refusal a customer disputes, or the first high-span corpus case.

## 6. Field streaming at 100M rays — **carried**

*Raised U9, restated U10.*

Extraction's working set is now `O(area)` and a 100M-ray field needs about 13 MiB
of sweep window. **The field itself, at roughly 4.7 GiB, is what bounds that
size**, and streaming it is a separate problem from anything Units 9 to 11 solved.

**Carried.** The memory ceiling already turns this from a crash into a refusal
that names a spacing which fits, which is the behaviour that matters
commercially. Streaming is an optimisation on top of a correct refusal, not a fix
for a defect.

**Trigger:** a customer part that genuinely needs 100M rays and cannot be served
by anisotropic spacing.

---

## Consequences

- Nothing on this list is tracked ambiently any more. Items 1, 4, 5 and 6 have a
  named trigger; items 2 and 3 need a reason to reopen, recorded above.
- Two of the six are declined on the same principle: **do not build for a phase
  that is conditional**. Units 19 and 20 may never happen, and work done for them
  now is work done for a door that may not open.
- Three of the four carried items are memory or performance, and all three are
  bounded by the Unit 10 ceiling — which means the failure mode in every case is
  a diagnosable refusal rather than a crash. That is why they can wait.
