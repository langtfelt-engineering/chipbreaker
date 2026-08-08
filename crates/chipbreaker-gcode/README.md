# chipbreaker-gcode

The RS-274 parser for
[Chipbreaker](https://github.com/langtfelt-engineering/chipbreaker),
a material-removal simulation and machining verification engine.

**This is the only place in Chipbreaker that reads G-code text.** Everything
downstream works from the canonical toolpath IR this crate produces, which is
what keeps dialect quirks from leaking into the geometry.

Four stages, each separately testable, because when a real file goes wrong the
question is always "which stage got this wrong":

```text
lex  ->  block  ->  modal  ->  resolve
```

Canned cycles are expanded to longhand. Arcs are resolved from `I`/`J`/`K` or
`R`, with the radius disagreement between endpoints **measured and recorded**
rather than silently split — because attributing a surface deviation later means
knowing whether the geometry or the tolerance caused it.

## What it refuses, and why refusing beats approximating

Siemens 840D and Heidenhain Klartext programs, macro and parametric programming,
`o`-word subprograms, and `G41`/`G42` cutter radius compensation. Each is
detected and **named**, not approximated:

```text
line 1: this looks like Siemens 840D, which is a different language rather
than a dialect of RS-274 (found "DEF INT")
```

`G41` is the sharpest case. Simulating the uncompensated path produces a part
wrong by the tool radius *everywhere*, and it looks entirely reasonable. A
verification tool that is quietly wrong is worse than one that says it cannot
answer.

## A note on the tests

Some integration tests read fixtures from `tests/corpus/` at the repository root,
outside this package. Clone the repository to run them.

## Licence

Copyright (C) 2026 Langtfelt. Dual-licensed: GPL-3.0-or-later, or a commercial
licence for use in a proprietary product. See the
[repository](https://github.com/langtfelt-engineering/chipbreaker) for details.
