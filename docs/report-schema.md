# The verification report schema

**Version 3.**

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

## The version 2 break, and why it was made

Version 2 **removes `accepted`** and replaces it with `verdict`. This is the only
breaking change the schema has made. It is documented here rather than smoothed
over, because a schema page that explains a break honestly argues for the
contract's seriousness, and one that quietly widens a field argues against it.

`accepted` meant "no gouge above tolerance". Collision checking gave that bit a
job it could not do: a consumer reading `accepted` — the obvious thing to read —
would have passed a program that drives a holder into a fixture. Three options
existed and two were worse.

| option | why not |
|---|---|
| Keep `accepted`, add a second flag | Permits `accepted: true` beside a spindle crash. "They should have checked the other field" is not a defence that survives the incident report. |
| Widen `accepted` in place | Changes an existing field's meaning under its own name, breaking every version-1 consumer **silently** — the one thing the contract above promises never happens. |
| **Rename it** | Breaks version-1 consumers **loudly**, at the moment it can still be fixed. |

The rename is the load-bearing part. A version-1 consumer looking for `accepted`
finds nothing and fails, rather than reading a bit that no longer accounts for
everything that can condemn a program. `report-diff` refuses a version-1 file for
the same reason instead of reading it with version-2 code.

The cost was worth paying now: the schema had no installed base, so the price of
a break was zero and will never be this low again.

## The version 3 break: three names that promised too much

Three renames, all of one kind — a field whose **name promised more than its
value delivers**. Removed and replaced, never aliased.

| was | is | why |
|---|---|---|
| `penetration_mm` | `overlap_along_ray_mm` | it is the overlap measured **along a dexel ray**. For a holder wider than the stock it reports the stock's extent, not any chuck dimension. |
| `clearance_mm` *(measured)* | `clearance_along_ray_mm` | a one-dimensional gap, and an **upper bound** on the true separation |
| `area_mm2`, `volume_mm3` | `area_estimate_mm2`, `volume_estimate_mm3` | estimates, as the accompanying `note` always said |
| `at`, `worst_at`, `bounds` | `at_mm`, `worst_at_mm`, `bounds_mm` | positions, with no unit stated, beside a dozen siblings that all said `_mm` |

Each already carried a `note` in the same object explaining the caveat, and that
was not enough. A consumer extracts `severity.penetration_mm` into a field of
their own called `penetration`, and the note stays behind. **In a
machine-readable contract, an ugly name that resists misreading beats an elegant
one that invites it** — because the name is the part that travels.

The **threshold** a caller sets keeps its plain name: `--clearance`,
`clearance_mm` in a job file, `clearance_threshold_mm` echoed in a collision
report. It is a setting, not a measurement, and nothing about it is measured
along anything.

**No identity moved.** Finding and collision IDs hash class, position, role,
obstacle and motion — never a JSON key name — so reports written before this
change still diff against reports written after it. Only `schema_version` and
the three key names differ.

## What a consumer must do with something it has not seen

Every enumeration in this schema may grow. The rules are part of the contract:

| it meets | it must treat it as |
|---|---|
| an unknown **gate name** | a gate like any other — `pass` is a conjunction, so include it |
| an unknown **gate state** | **not-pass** |
| an unknown **contact kind** | a **defect**, unless `is_defect` says otherwise |
| an unknown **finding class** | not a defect, unless `is_defect` says otherwise |
| an unknown **key** anywhere | ignore it |

`is_defect` exists precisely so a consumer never has to enumerate the classes to
answer the only question that matters. Read it rather than switching on the name.

**Do not skip an unrecognised gate.** `pass` is a conjunction, so ignoring a gate
makes a consumer's own answer *laxer* than the report's — the one direction the
contract does not permit. Either read `pass` directly, or recompute over every
gate including ones you do not recognise.

## `verdict`

```json
"verdict": {
  "pass": false,
  "gates": {
    "gouge":     {"state": "pass"},
    "collision": {"state": "fail", "why": "1 collision"}
  }
}
```

`pass` is the **conjunction** over every gate. A gate is `pass`, `fail`, or
`unchecked`.

**`unchecked` does not pass.** A gate that could not run says so and carries a
`why`. This is the whole point of having three states: a holder that is not
modelled cannot be found hitting anything, and a tool reporting "clear" on that
basis would be manufacturing safety out of missing data.

The collision gate is `unchecked` when:

| condition | why it cannot be answered |
|---|---|
| the tool has no holder geometry | nothing above the shank is modelled, so nothing above the shank can be found hitting anything |
| `unmodelled_retracts` is non-zero | the machine makes motion this replay does not contain |
| no `--stock-field` was given | `verify` holds the **cut** field, and a collision is judged against the material present when each move ran |
| no `--path` was given | there is no program to replay |

That third row is worth stating plainly. A collision is a property of the
**trajectory**: at the moment a move executes, the material in its way is the
stock as it stood then. Judging against the final field would test every move
against the least material the job ever contains and would miss every collision
with material a later pass removes. So `verify` needs the field the program
started from, and `chipbreaker collide` takes it as its only positional
argument.

**Forward compatibility, which integrators may rely on:** every future gate is a
new key under `gates`, and adding one can only ever make a verdict *stricter* —
a conjunction can lose a true, never gain one.

That guarantee is about the **report**, not about a consumer who discards part of
it. Version 2 said "a consumer that ignores keys it does not recognise still gets
a correct answer", which reads two ways and is only true on one of them. Stated
precisely:

- Ignoring an unrecognised **key inside** a gate — a `why`, a future `detail` —
  is safe.
- Ignoring an unrecognised **gate** is *not*. If that gate failed, a consumer
  skipping it computes `pass` where the report says otherwise, which is laxer
  than the report and the one direction the contract forbids.

So: read `pass`, or recompute over **every** gate, treating an unrecognised state
as not-pass.

## Top level

| key | type | meaning |
|---|---|---|
| `schema` | string | always `chipbreaker.verification-report` |
| `schema_version` | integer | `3` |
| `verdict` | object | `pass`, and one entry per gate — see above |
| `verdict_rule` | string | what `pass` means, in the artifact |
| `manifest` | object | which inputs, at what settings |
| `numerical_semantics` | object | what the numbers are worth |
| `exclusions` | array of string | what the comparison does **not** model |
| `scope` | string | what was verified: the program, not the machine |
| `summary` | object | counts and worst depths |
| `findings` | array | the findings, in canonical order |
| `collisions` | array | collisions and near misses, in canonical order |
| `rapid_path` | object | which rapid policy was replayed, or why none was |
| `environment` | object | host and timing — **excluded from every digest** |

`rapid_path` is in the report because a dogleg rapid can collide where a linear
one does not, so a collision result is only as trustworthy as the path policy it
was computed against.

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
| `boundaries` | array or null | one entry per setup boundary crossed |
| `accumulated_transform_bound_mm` | f64 | the sum of those bounds |

`path` is for a human and is **not** part of the digest: two runs of the same
bytes from different directories are the same run.

`engine_selftest` identifies the build's *behaviour* rather than the build — it
is identical across all four targets, so a report produced on Linux and one
produced under `wasmtime` carry the same value.

**Same manifest digest implies byte-identical findings and collisions.** That is
a test, not an aspiration.

That promise is only as wide as the self-test behind it, and it was briefly
narrower than it claimed. Collision detection shipped before it had a self-test
suite, so two builds whose collision behaviour differed carried the *same*
`engine_selftest` — and a diff of two such reports showed collisions changing
under an identical manifest, which is exactly what the manifest exists to make
impossible. It was found by running the diff, not by reasoning about it. The
`collision` suite closes it; anything added later that a report can carry needs
one too.

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
| `severity` | object | `worst_depth_mm`, `mean_depth_mm`, `area_estimate_mm2`, `volume_estimate_mm3`, `note` |
| `sample_count` | integer | exact |
| `at_mm` | `[f64; 3]` | centroid |
| `worst_at_mm` | `[f64; 3]` | position of the deepest sample |
| `bounds_mm` | object | `{min, max}` |
| `attribution` | object | `{ambiguous, segments}` |

`attribution` carries a **`setup`** index beside its segments. A line number
alone is ambiguous across a job: two setups have two programs, each numbering
its own lines from one, so "line 47" names two different moves. It is `0` for a
single-setup run.

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

`worst_depth_mm`, `mean_depth_mm` and `sample_count` are **exact**.
`area_estimate_mm2` and `volume_estimate_mm3` are **estimates**, and now say so
in their names: each surface patch is counted once, from
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

## `collisions[]`

A collision is **not** a `class` of finding, and the separation is deliberate.

| key | type | meaning |
|---|---|---|
| `id` | string | content-derived, sixteen hex characters |
| `contact` | string | `collision`, `cutter-into-fixture`, or `near-miss` |
| `is_defect` | bool | true for `collision` and `cutter-into-fixture`; false for `near-miss` |
| `severity` | object | `overlap_along_ray_mm` **or** `clearance_along_ray_mm`, never both |
| `element` | object | `{role, index}` — which part of the tool stack |
| `obstacle` | object | `{kind}`, plus `{index, name}` for a fixture |
| `motion` | string | `rapid`, `linear`, `arc`, `helix` |
| `at_mm`, `bounds_mm`, `attribution` | | as for a finding |

A finding's severity is a depth into the **nominal surface**, with an area and a
volume measured over that surface. A collision's is penetration of a non-cutting
element into an **obstacle**: no nominal surface is involved, area and volume over
one do not exist, and `worst_depth_mm` would be one field name carrying two
different physical quantities — which the stability contract forbids.

They also come from different places. A finding is derived from the deviation
field and governed by its detection floor. A collision is a property of the
**trajectory**, computed as the program is replayed, and the deviation field never
enters into it.

**Penetration and clearance are separate keys.** A `-0.2` meaning "cleared by
0.2 mm" and a `0.2` meaning "buried by 0.2 mm" are different enough that a
consumer sorting on one number would rank a safe pass beside a crash. A near miss
is reported but is not a defect: it names the thing that will collide after a
small edit, which is often more useful than the crash itself.

## Exit codes

One contract across every verb, so a script does not have to remember which
command it called.

| code | means | what to do |
|---|---|---|
| 0 | the run completed and every gate passed | nothing |
| 1 | the run completed and the verdict does not pass — a gate failed, **or a gate could not run** | fix the program, or supply what the unchecked gate needed |
| 2 | the run could not be completed: bad input, a refusal, an I/O error | fix the invocation |

**One and two are different answers**, and they shared a code until this review
separated them. A CI gate wants to distinguish "the part is bad" from "I could
not look" — which is the same distinction the three-state verdict exists to make,
and it was being thrown away at the process boundary.

`report-diff` uses 0 for identical and 1 for differs, so it works as a gate
without parsing anything. A file that is not a report exits 2.

## `report-diff`

```
chipbreaker report-diff old.json new.json [--json]
```

Emits `chipbreaker.report-diff` version 1.

A file that is not a Chipbreaker report is **refused**, not read as an empty one.
Treating arbitrary JSON as "no findings" would exit zero and say "identical",
which is the most dangerous answer a CI gate can give.

Manifest differences are reported **first** and labelled as possibly explaining
everything below them: a resolution change can move every finding, and a reader
who starts with the finding list will hunt for a program bug that is not there.
