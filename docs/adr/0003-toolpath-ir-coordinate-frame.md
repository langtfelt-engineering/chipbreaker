# ADR 0003 — The toolpath IR stores machine coordinates

- **Status:** Accepted
- **Date:** 2026-08-03
- **Governs:** the toolpath IR, and everything downstream that consumes it

## Decision

`MotionSegment.start` and `.end` hold **machine coordinates**, in millimetres.

Every work offset in force anywhere in the program is recorded in
`ToolpathHeader.offsets` as a transform, and every activation is recorded as a
`PathEvent`. A consumer that wants workpiece coordinates applies one transform;
a consumer that wants to know *which* workpiece frame was active at a given
segment reads the events.

This **overrides** the original specification, which asked for workpiece
coordinates. The specification also required exact contiguity, and required a
corpus case exercising a mid-program work-offset change. Those three cannot all
be true at once.

## Why they cannot all be true

Consider:

```gcode
G54 G0 X10 Y10      ; work offset A
G55                 ; work offset B, no axis words
G1 X10 Y10 F100     ; the same programmed point, a different place
```

The `G55` block commands no motion. The tool does not move. But if segments hold
workpiece coordinates, the segment before `G55` ends at `(10, 10)` in A's frame
and the segment after begins at `(10, 10)` in B's frame — two different points in
space wearing the same numbers. Reverse the situation and the numbers differ
while the point does not. Either way, `start != previous end`.

The same discontinuity arises from `G92`, from `G10 L2` rewriting an offset
mid-program, and from a `G43 H1` → `G43 H2` tool-length change.

There is a second problem, prior to the arithmetic. "Workpiece coordinates" does
not name a frame. A program that machines two fixtures through `G54` and `G55`
has two workpiece frames, and the phrase is ambiguous exactly when it matters.

## Why machine coordinates rather than one chosen offset

The alternative considered was to name a frame — `--frame G54` — and reject
programs using more than one. It was rejected for three reasons.

**It cannot represent a real class of program.** Multi-fixture and pallet
programs machine several parts in one file, and a verification tool that refuses
them refuses the jobs where a crash is most expensive.

**`G53` stops being special.** `G53` is a non-modal move in machine coordinates.
In the machine frame it needs no handling at all; in a workpiece frame it is a
per-block exception to the resolution pipeline.

**Contiguity becomes unconditional.** Not "contiguous except across offset
changes", with every downstream consumer obliged to know the exception. Field
building can
assert `start == previous.end` with no tolerance and no cases, and any violation
is a bug rather than a possibility to be handled.

## What it costs

Field building applies one transform to place stock relative to the workpiece rather than
consuming coordinates directly. That is a single matrix per setup, applied once,
against a downstream simplification that everything after it benefits from.

Reports must render coordinates in a workpiece frame to be intelligible — an
operator thinks in G54, not in machine coordinates. That is a presentation
concern, and the header carries what is needed to do it.

## Consequences downstream

- `start == previous.end` holds **exactly**, everywhere, with no tolerance and no
  exceptions. Assert it.
- A `WorkOffsetChanged` event is a *label*, not a discontinuity. The geometry is
  continuous across it.
- To place stock: read `header.offsets[active]` and invert.
- Adding orientation later does not disturb this. A rotary axis moves the *part*
  relative to the machine, so the machine frame remains the one frame in which
  everything is expressible.
