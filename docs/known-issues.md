# Known issues

Defects that are understood, reproducible, and not yet fixed. Each entry says
how to recognise it, how to avoid it, and what is actually known about the
cause — so that nobody has to rediscover any of that.

An issue leaves this page when it is fixed and a test pins it, not when it
stops being mentioned.

---

## KI-1 — A mesh whose minimum corner sits on the grid origin invents a gouge

**Status:** open. Found 2026-08-10 while building the evaluation corpus.

**Severity:** high for anyone whose stock happens to be modelled at the origin,
which is common. It produces a **false defect**, which is the worst direction
for a verification tool to be wrong in — a false clear is worse still, but a
tool that cries wolf gets switched off, and then it is not detecting anything.

### How to recognise it

A gouge reported as a tall, thin column along one vertical corner of the stock,
with **no attributed NC line**, in a region no tool went near. Depth is on the
order of a few millimetres and grows slightly as resolution is refined.

### The minimal reproduction

No program, and a nominal identical to the stock. Compared against itself, a
part cannot deviate from itself, so any finding at all is spurious:

```python
import chipbreaker
# a 60 x 40 x 25 block with its minimum corner at exactly (0, 0, 0)
r = chipbreaker.verify(program="nothing.nc", tools=..., stock="box.stl",
                       nominal="box.stl", resolution_mm=0.5)
r["summary"]["worst_gouge_mm"]   # 1.9843, and it should be 0.0
```

Move the same box off the origin and it disappears:

| stock | worst gouge |
|---|---|
| box at `(0, 0, 0)`–`(60, 40, 25)` | **1.9843 mm** |
| the same box translated to `(7.3, 3.1, 2.7)` | 0.0 mm |
| cube at `(0, 0, 0)`–`(40, 40, 40)` | **1.9843 mm** |

The identical value across two different box sizes says the magnitude is a
property of the sampling near the corner rather than of the part.

### What is known about the cause

It depends on the mesh's minimum corner coinciding with the field's grid
origin, not on the box's size or aspect. It is present with no tool involved at
all, so it is in the **comparison or the field build**, not in the sweep, the
cut, or the collision check. Refining resolution makes it slightly *worse*
(1.66 mm at h = 1.0, 1.98 mm at h = 0.5, 2.12 mm at h = 0.25), which rules out
an ordinary discretisation error and points at a boundary condition on the
first cell.

It has not been traced further than that.

### How to avoid it

Model stock so that it does not begin exactly at the machine origin. This is
good practice for an unrelated and equally real reason: a program starts with
the tool at `(0, 0, 0)`, and a block covering that point has the shank inside
the material before the first line runs — which the collision check will
correctly report as a crash.

The evaluation corpus places its block at `x 10..70, y 8..48` for both reasons.

### What it means for the published recall figures

The README reports **no gouges invented on a correctly machined part**, measured
over the 295-case defect corpus. That measurement stands: those cases do not sit
on the grid origin, so none of them exercises this. But the claim as *stated* is
broader than the evidence now supports, and the honest reading is "none invented
across the defect corpus", not "none invented, ever". The corpus should gain an
origin-aligned case, at which point this issue will be failing a test rather
than sitting on a page.
