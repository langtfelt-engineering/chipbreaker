# The verification report schema

**Version 1. Frozen.**

`chipbreaker verify --report r.json` writes this. It is a **public interface**:
integrators build against it, and later work extends it rather than reshaping
it.

## The stability contract

- **A new field is an addition.** Consumers that ignore unknown keys are
  unaffected, and consumers that do not ignore them were going to break on
  anything.
- **An existing field never changes meaning under its own name.** If the meaning
  has to change, the field gets a new name and the old one is deprecated in
  place.
- **`schema_version` bumps only for a breaking change.** Adding fields does not
  bump it.
- **Keys are sorted and floats are full precision**, so two reports of the same
  inputs are byte-identical and `diff` on the files themselves is meaningful.

## Top level

| key | type | meaning |
|---|---|---|
| `schema` | string | always `chipbreaker.verification-report` |
| `schema_version` | integer | `1` |
| `accepted` | bool | the verdict — see `verdict_rule` |
| `verdict_rule` | string | what `accepted` means, in the artifact |
| `manifest` | object | which inputs, at what settings |
| `numerical_semantics` | object | what the numbers are worth |
| `exclusions` | array of string | what the comparison does **not** model |
| `scope` | string | what was verified: the program, not the machine |
| `summary` | object | counts and worst depths |
| `findings` | array | the findings, in canonical order |
| `environment` | object | host and timing — **excluded from every digest** |

`environment` appears only on standard output, never in the written report file,
and is excluded from every hash. Two runs of the same inputs on two machines an
hour apart must agree, or the identity is measuring the clock.

## `manifest`

| key | type | meaning |
|---|---|---|
| `digest` | string | **the report's identity** |
| `inputs` | array | `{role, path, digest}`, sorted by role |
| `spacing_mm` | `[f64; 3]` | cell size per axis |
| `tolerance_mm` | f64 | what findings were judged against |
| `cluster_radius_mm` | f64 | what grouped samples into findings |
| `engine_version` | string | the crate version |
| `engine_selftest` | string | the engine's self-test digest |

`path` is for a human and is **not** part of the digest: two runs of the same
bytes from different directories are the same run.

`engine_selftest` identifies the build's *behaviour* rather than the build — it
is identical across all four targets, so a report produced on Linux and one
produced under `wasmtime` carry the same value.

**Same manifest digest implies byte-identical findings.** That is a test, not an
aspiration.

## `numerical_semantics`

| key | type | meaning |
|---|---|---|
| `spacing_mm` | `[f64; 3]` | repeated, so the section stands alone |
| `tolerance_mm` | f64 | as applied |
| `stock_facet_mm` | f64 | chord error of the stock mesh |
| `nominal_facet_mm` | f64 | chord error of the nominal mesh |
| `tolerance_floor_mm` | f64 | the coarsest of the three inputs |
| `below_floor` | bool | whether the tolerance is below that floor |
| `worst_projection_gap_mm` | f64 | how far the perpendicular reading overstated the metric |
| `swept_volumes` | object | how the swept volumes were computed |
| `detection_floor` | object | the measured recall curve, and where it lives |

`swept_volumes` carries `available: false` and a `why` when the run's statistics
were not supplied, because a field does not carry the statistics of the run that
cut it. **It never reports zeros in that case**: "no ray-cut was bounded" is a
claim, and an audited artifact should not make one by accident. Pass
`--run-report` from `chipbreaker run --json` to fill it in, giving
`ray_cuts_exact`, `ray_cuts_bounded` and `worst_bound_mm`.

## `findings[]`

| key | type | meaning |
|---|---|---|
| `id` | string | content-derived, sixteen hex characters |
| `class` | string | `gouge`, `excess-stock`, `undercut`, `unreachable` |
| `is_defect` | bool | true for `gouge` alone |
| `severity` | object | `worst_depth_mm`, `mean_depth_mm`, `area_mm2`, `volume_mm3`, `note` |
| `sample_count` | integer | exact |
| `at` | `[f64; 3]` | centroid |
| `worst_at` | `[f64; 3]` | position of the deepest sample |
| `bounds` | object | `{min, max}` |
| `attribution` | object | `{ambiguous, segments}` |

`attribution.segments` is an array of `{segment, file, line, block, cycle_step?}`.
`cycle_step` is present only for a segment expanded from a canned cycle — one
`G81` becomes rapid, plunge and retract, and a report naming line 42 three times
without distinguishing them makes the reader do the work.

### Identity

`id` is a hash of the finding's **class and quantised position**, never a
counter. A counter renumbers everything after an insertion and makes two reports
undiffable.

**Severity is deliberately not hashed.** A gouge that deepens from 1.0 to 1.2 mm
keeps its identity, so a diff reports "changed severity" rather than one finding
vanishing and another arriving.

The cost, stated rather than hidden: a finding whose centroid crosses a
quantisation boundary takes a new identity, and a diff shows that as one gone and
one arrived. The quantisation is the cluster radius, so this needs real movement
rather than rounding.

### Severity

Depth and extent are reported **separately and never combined**. A 2 mm gouge
over one cell and a 0.2 mm gouge over a whole face are different problems, and
one number cannot say which this is.

`worst_depth_mm`, `mean_depth_mm` and `sample_count` are **exact**. `area_mm2`
and `volume_mm3` are **estimates**: each surface patch is counted once, from
whichever bundle sees it most squarely, which bounds the weight by `sqrt(3)`.

### Classification

Only `gouge` is a defect on its own.

- **`excess-stock`** is what a roughing pass is *for*. Whether it is a defect
  depends on whether this was the last operation on that surface, which a single
  comparison cannot know.
- **`undercut`** is a nominal face pointing away from the tool's approach. No
  3-axis tool reaches it at this setup at any resolution; the fix is another
  setup, not a finer lattice.
- **`unreachable`** is nominal surface no sample mapped to. It is an absence of
  evidence, not a measurement, and its `worst_depth_mm` is `0.0` because
  inventing a depth there would put a number in the report that nothing measured.

## `report-diff`

```
chipbreaker report-diff old.json new.json [--json]
```

| exit | means |
|---|---|
| 0 | identical |
| non-zero | differs, or a file is not a report |

A file that is not a Chipbreaker report is **refused**, not read as an empty one.
Treating arbitrary JSON as "no findings" would exit zero and say "identical",
which is the most dangerous answer a CI gate can give.

Manifest differences are reported **first** and labelled as possibly explaining
everything below them: a resolution change can move every finding, and a reader
who starts with the finding list will hunt for a program bug that is not there.
