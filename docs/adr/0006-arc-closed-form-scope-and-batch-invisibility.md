# ADR 0006: The arc closed form's scope, and batching's invisibility

- **Status:** accepted
- **Date:** 2026-08-06
- **Unit:** 8 (arcs, helices, batching)
- **Binds:** U9 (surface extraction), U11 (parallelism), U13 (customer-facing accuracy claims), U15 (chained-equals-monolithic)
- **Related:** [ADR 0003](0003-toolpath-ir-coordinate-frame.md), [ADR 0005](0005-deviation-not-volume.md)

## Two rules

**1. Case A′ applies only when the arc's axis is parallel to the tool's.** That
means `G17` with no axial rise, and nothing else. A `G18` or `G19` arc, and any
helix, is sub-stepped with a bound derived from the true helical path length.

**2. Batch size is invisible.** Cutting a list of motions must produce a
bit-identical field *and* bit-identical statistics at every batch size, including
one. Batching is a tuning knob; it is never allowed to become an answer.

## Why rule 1, and what it cost to learn

The Case A′ collapse rests on a single identity. For a tool centre travelling a
circle of radius `R` about `C`, and a point at distance `d` and bearing `phi`,

```text
|p - centre(theta)|^2 = d^2 + R^2 - 2 d R cos(phi - theta)
```

minimised at `theta = phi`, giving `(d - R)^2`. Inside the swept wedge the
membership test collapses to `|d - R| <= rho(w)`.

That identity is two-dimensional. It works because the distance from the point to
the tool axis is measured **in the plane the arc turns in**, and the tool's
radius `rho` is a function of the coordinate **along the tool's own axis**. When
the two axes coincide — a `G17` arc of a vertical tool — those are different
coordinates and the problem separates. When they do not, the point's distance
from the tool axis depends on the sweep parameter in a way `rho` cannot absorb,
and there is no collapse.

`Motion::case` originally read `!is_helix()` and returned `SweepCase::Arc` for
any arc without a rise, whatever its plane. The consequences chained:

- `SweepMethod::Analytic` reads `SweepCase::Arc` as *exact, plan zero sub-steps*.
- `arc::swept_spans_into` correctly declined the non-`G17` arc and returned
  `false`, because it checks the plane.
- The fall-through swept the motion with `steps.max(1)` — **one** sample of the
  tool, parked at the start point, for a whole quarter turn.

Every individual piece behaved as documented. Nothing warned. The cut was simply
almost entirely absent, and it took adding a `G18` case to the corpus to see it.

**The classification is therefore part of the contract, not an implementation
detail.** A non-`G17` arc is a `Ramp`; a rising arc is a `Helix`; both sub-step.
Anything that adds a motion kind must decide its case by what is *provably
exact*, never by what is merely not-obviously-inexact.

### The deviation bound for the sub-stepped cases

`deviation(N) = sqrt((2 R sin(delta/4))^2 + (h/2)^2)`, exact, with `delta` the
per-step angular extent and `h` the per-step rise. The step count is chosen from
`L / (2N)` with `L` the **helical** length, `sqrt((R * sweep)^2 + rise^2)`.

A chord-based length would be unsound, not merely loose: on a 2.4 radian sweep of
a 10 mm radius with a 6 mm rise, the chord under-states the path by 20.8%, so a
bound derived from it would claim an accuracy the sweep does not have.

### This is not the same quantity as a chord tolerance

Linearising an arc into chords — what `--no-arc-native` does, and what many CAM
posts do — has deviation `R (1 - cos(delta/2))`, the sagitta. The axial term is
absent because a chord through a helix interpolates the axial coordinate linearly
in the same parameter the helix does and both ends agree, so the axial component
of a chord is **exact**.

Sagitta is `O(delta^2)` where the sub-step bound is `O(delta)`. The two therefore
need quite different counts for the same tolerance and must never be compared or
substituted for one another.

## Why rule 2

Batching inverts the loop: unbatched walks motions outside and rays inside,
batched does the reverse. The field cannot notice, because rays do not interact
and each ray still sees its motions in order. `removed_mm3` can, because it is a
sum of floats.

Accumulating into per-motion slots fixes the order *within* a batch and does
nothing about the boundary *between* batches. Unbatched computes
`((m1 + m2) + m3) + m4`; batched in pairs computes `(m1 + m2) + (m3 + m4)`. Same
order, different grouping, and floating-point addition is not associative. That
cost one ULP, and it is why the per-motion slots run the length of the whole
motion list and are summed once at the end — and why `cut_all` is the entry point
while `cut_batch` is not.

### What this binds on Unit 11

Unit 11 parallelises. The same argument applies to it verbatim and is harder
there, because a thread pool reorders by construction:

- **Rays may be distributed freely.** Each ray's subtraction is independent.
- **Removed volume may not be accumulated across threads in completion order.**
  It must land in per-ray or per-motion slots and be reduced in a fixed order —
  ascending ray within a motion, ascending motion — or the reported volume
  becomes a function of thread scheduling.
- **`worst_bound_mm` is a maximum** and so reorders freely. The integer counters
  do too. Only the float sums are at risk, and there are only three of them.

A parallel run that produced the same field and a different volume would be the
same defect as the batching one, arriving by a route that is much harder to
reproduce.

## Consequences

- `SweepCase::is_exact` is true for `Stationary`, `Horizontal`, `Plunge` and
  `Arc` only. `Ramp` and `Helix` sub-step, and a `G18` arc is a `Ramp`.
- The corpus pins sub-step counts, so a case that silently stops taking its
  closed form — or starts claiming one it has not earned — fails as a dispatch
  change rather than passing as a small numerical difference.
- The selftest hashes arcs, helices and two batch sizes, so both rules are
  checked on all four targets rather than only on the developer's.
- `--batch-size` may be tuned, defaulted, or ignored without any customer-visible
  effect. If that ever stops being true it is a bug in Chipbreaker, not a
  tolerance to document.
