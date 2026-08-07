# ADR 0010: An unset normal is not distinguishable from `+Z`, and stays that way

- **Status:** accepted
- **Date:** 2026-08-07
- **Unit:** 12 (verification), fixing a defect introduced in Unit 7
- **Binds:** the span format, `.tdx` versions 3 and later, and any future consumer
  of a single normal
- **Related:** [ADR 0001](0001-spans-arena.md),
  [ADR 0004](0004-dexel-binary-format.md),
  [ADR 0005](0005-deviation-not-volume.md)

## The rule

**`OctNormal::PLACEHOLDER` is `(0, 0)`, it decodes to `+Z`, and no bit pattern is
reserved to mean "unset". Correctness comes from the guarantee that every
endpoint is given a real normal at the moment it is created, not from a sentinel
that could be checked afterwards.**

## What happened

Unit 9 added normals to span endpoints and wrote that a normal is available for
free at both sites where an endpoint is born: the triangle normal during
construction, and the analytic tool surface normal during a cut. Only the first
was implemented. `tool::raycast` built every span with `Span::ordered`, which
leaves the placeholder, and the subtraction that removes material negated it —
so **every cut face in the engine carried `(0, 0, -1)`**, whichever way it
actually faced.

It survived five units, for three structural reasons:

1. Dual contouring solves a QEF over several crossings per cell, so one plane
   with a wrong normal is averaged against correct ones rather than seen.
2. Unit 9's own sharp-feature corpus used **uncut** boxes, spheres and tori,
   where every normal comes from construction and is correct.
3. Unit 12's deviation field is the first consumer to use a single normal on its
   own, with nothing to average it against.

The two tests written to catch it both passed vacuously first: one counted
placeholders, which fails because the placeholder *is* a legitimate `+Z`; the
other asked whether a slot's endpoints all shared a normal, which fails because
the outer stock faces are correct and only the cut faces were not.

## The decision, and why it is not "add a reserved bit"

The obvious repair is to reserve a pattern for "unset", so that a consumer can
tell a real `+Z` from a normal nobody wrote. It is declined.

**It would not have caught this.** The defect was not a consumer misreading a
sentinel. It was a producer never writing one. A reserved pattern turns a silent
wrong answer into a loud wrong answer only if somebody checks it, and the
extractor's honest response to "unset" is the same as its response to `+Z`: it
has no better information. The bug would have been a stream of warnings from a
correct-looking mesh rather than a mesh quietly built on `(0, 0, -1)`.

**It costs the exactness of negation.** `OctNormal::negated` is integer
arithmetic that loses nothing, precisely because the encoding is odd-symmetric
about `(0, 0)` with no holes in it. The subtraction that removes material negates
every cutter normal, so this runs on every cut face of every simulation.
Reserving a pattern puts a branch in that path and breaks the symmetry the
exactness rests on.

**It moves the guarantee to the wrong place.** What actually makes the field
correct is that both creation sites write a real normal. That is a property of
two functions — `dexel::build` and `tool::raycast::intersect_ray_into` — and it
is now enforced where it belongs: `tests/sweep_normals.rs` checks stored normals
against an independent Minkowski-sum oracle across six motion orientations, and
asserts that the check rejects the exact value the defect produced.

## Where "unset" genuinely has to be known

Two places, and in both the caller knows it from context rather than from the
value:

- Reading a `.tdx` at format version 2, which predates normals entirely. The
  version number says so.
- `extract --no-normals`, which discards them deliberately to produce the surface
  nets control the contour corpus compares against. The flag says so.

Nothing infers it from the bits, and nothing should start.

## Consequences

- The goldens recorded before this fix encoded the defect. They were re-accepted
  in the same change, and the commit says why.
- The Unit 9 claim that four bytes buy sharp features was measured on
  construction normals only. It is restated, and re-measured on cut geometry, in
  the same unit.
- Any future producer of a span endpoint — a new sweep case, a new import path —
  must set a normal at the point of creation. There is no check that will catch
  it later.
