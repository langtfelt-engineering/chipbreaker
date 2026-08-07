# ADR 0007: A dexel ray is global, so local refinement is not available

- **Status:** accepted
- **Date:** 2026-08-07
- **Unit:** 9 (dual contouring), ruling scope for Unit 10
- **Binds:** U10 (adaptive resolution), U16–U18 (5-axis), U19 (WASM demo memory budget)
- **Related:** [ADR 0005](0005-deviation-not-volume.md), [ADR 0006](0006-arc-closed-form-scope-and-batch-invisibility.md)

## The rule

**Refinement of a tri-dexel field is rectilinear and global per axis. There is no
local, octree-style refinement, and Unit 10 will not attempt one.**

Anisotropic per-axis spacing and inserted full coordinate planes are in scope.
Partial rays — rays that exist over a sub-interval of their axis — are
**explicitly deferred**, and this ADR exists so the next person understands they
were ruled out on representation grounds rather than overlooked.

## Why: a ray is global along its axis

A dexel ray is not a sample at a point. It runs the full extent of the workspace
along its axis and carries the spans of material it meets. It either exists or it
does not; there is no such thing as a ray that exists only near an interesting
feature.

That property, which is the source of the representation's strength — exact along
the ray, no accumulated error, cutting as interval arithmetic — is what forbids
local refinement.

Suppose the field is refined near a feature by inserting one transverse ordinate
`y*`. To be usable, `y*` must be a transverse coordinate of the bundles that
cross it:

- the **X bundle** needs a ray at `(y*, z_k)` for **every** `k`, and
- the **Z bundle** needs a ray at `(x_i, y*)` for **every** `i`.

A single locally-inserted coordinate therefore propagates into full planes of new
rays in the other two bundles, spanning the entire workspace. The refinement is
not local, and cannot be made local, because the objects being added are not
local.

## What made it visible, and what did not cause it

[ADR 0006](0006-arc-closed-form-scope-and-batch-invisibility.md) was followed at
Unit 9 by a registration invariant: the three bundles must share one corner
lattice, because they are the three edge directions of the dual contouring grid.

It would be easy to read the registration invariant as the obstacle and to
imagine that relaxing it would restore local refinement. **It would not.**
Registration made the problem visible one unit earlier than it would otherwise
have surfaced; the problem is the globality of a ray, and it is present whether
or not anything checks registration. Dropping the invariant would give up
watertight extraction and gain nothing.

## What is in scope instead

Three things, all registration-safe by construction, and the first of them is
probably the largest single win available:

### 1. Anisotropic per-axis spacing

Independent `h_x`, `h_y`, `h_z`, chosen from the part's geometry rather than one
number for all three. Registration is preserved trivially: each axis still has
one shared ordinate set, it is simply a different one per axis.

This attacks a measured problem. Unit 6's per-cubic-centimetre cost varies about
5× between a plate and a bar, and a plate at 1593 KiB/cm³ is paying for
resolution in a direction where it has almost no extent. Refining one axis by 2×
costs about 1.67× memory where refining all three costs 4×.

### 2. Rectilinear graded refinement

Insert **full** coordinate planes where curvature or feature density warrants.
Registration-safe by construction, since a full plane is exactly the object the
bundles need. Real wins where complexity is concentrated in slabs — which in
machining it usually is: fine near the finished surface, coarse through the bulk.

### 3. A hard memory ceiling with graceful degradation

Orthogonal to refinement and independently valuable. A cap, a documented strategy
on reaching it, and a clear refusal naming the number rather than an OOM in the
middle of a customer's job.

## What is deferred, and what it would cost

**Partial rays.** Rays existing over a sub-interval, so that refinement can be
genuinely local. This is a substantial redesign, not a feature:

- The arena, the `.tdx` format and the canonical hash all assume a ray is a
  full-extent object identified by its transverse position alone.
- Level boundaries introduce cracks — a fine ray ending where a coarse one
  continues — and stitching them watertight is the classic hard part of adaptive
  contouring, made harder here because the three bundles would have to agree
  about where the boundary is.
- Extraction's cell grid would become an octree whose corners are no longer a
  simple product of three ordinate sets, which is the assumption Unit 9's sweep
  rests on.

This is research rather than engineering, and Unit 10 is not on the critical path
to a sellable product. Deferring it is a scope decision made with the cost known.

## Consequences

- Unit 10's exit criterion becomes: **measure against Unit 6's per-cm³ baseline
  and publish what was achieved.** It does not promise an accuracy-versus-memory
  table from an octree, because there will not be one.
- Any future proposal for local refinement must address the globality of a ray
  first. A proposal that starts from the extraction side has not engaged with the
  problem.
- The memory ceiling should be built regardless of what refinement lands, since
  it is the only mechanism that turns "too big" into a diagnosable refusal.

## Amendment, Unit 10: graded planes declined, with the measurement

Unit 10 was told to build rectilinear graded planes **only if** anisotropic
spacing left enough on the table to justify the complexity. It does not.

Holding the sample-distance bound fixed — the ruling that `--auto-res` may buy
memory with anything except the guarantee — the measured saving at `--res 0.1`
is:

| part | isotropic KiB/cm³ | auto KiB/cm³ | saving |
|---|---:|---:|---:|
| cube 40×40×40 | 366.2 | 366.8 | 1.00× |
| block 100×60×20 | 374.3 | 356.4 | 1.05× |
| plate 200×200×6 | 862.6 | 673.8 | **1.28×** |
| bar 300×20×20 | 504.6 | 481.9 | 1.05× |

Unit 6 observed per-cm³ varying about 5× across shapes like these; measured here
at 2.36× isotropic. **That spread is the part's geometry, not waste.** Only the
portion of it that survives holding the bound fixed is recoverable, and that is
the 1.28×.

The cube is the informative row. It comes back isotropic — very slightly *worse*
than isotropic, by 0.2%, because the continuous optimum rounds up through a
`ceil` per axis — and that is correct rather than a defect: the constrained
problem is symmetric, so there is nothing to win, and a rule that produced
anisotropy there would be one that had stopped holding the bound.

### Why graded planes are not worth it on that evidence

Inserted planes would add **non-uniform indexing to every consumer**: the sweep's
transverse rejection, the extraction sweep's window sizing, the deviation
harness's nearest-endpoint search, and the contour grid's cell extents. Three of
those four were *already* wrong under mere anisotropy and had to be fixed in this
unit — including one, `nearest_endpoint`, where a single shared spacing made the
accuracy harness report six times the true deviation while the field itself was
perfectly correct.

Paying that cost again, across more code, for an increment on top of a 1.00–1.28×
base, is not a good trade. Graded planes are **declined**, not deferred pending
more thought: the measurement that would justify them is the one above, and it
says no.

If the picture changes — a customer part with feature density concentrated in a
slab, and a measurement showing anisotropy leaving much more than 1.28× on the
table for it — reopen this with that measurement attached.
