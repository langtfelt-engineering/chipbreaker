# G-code corpus

Every entry is an NC file plus a committed golden IR, or an NC file plus the
specific error it must be rejected with. Each carries a `_why` explaining what it
is for, in the manner of the Unit 2 mesh corpus.

## The arc form amendment — do not "fix" this

Unit 4's definition of done originally required that the `I`/`J`/`K` and `R`
forms of one arc **produce identical IR**. That is not achievable and the
requirement was amended.

The `I`/`J`/`K` form is *given* the centre. The `R` form *derives* it, through a
square root and two divisions. Those are two different computations of the same
quantity, and they agree to within a rounding rather than on the same bits. No
implementation choice changes this, because the two inputs do not carry the same
information.

Measured across 39 sweep angles at four coordinate scales — regenerate with
`cargo run -p chipbreaker-gcode --example form_agreement` — the worst
disagreement is **11.15 ULP** in the centre and 3.82 ULP in the sweep.

So:

* The two forms must resolve to the same arc **within 32 ULP**, which is the
  measured worst case with margin.
* Their golden IR hashes **legitimately differ**, in the last bit or two of the
  arc centre, because they came from different source files.

`arc-quarter-ijk.nc` and `arc-quarter-r.nc` are the pair. **Their goldens are
supposed to differ.** If a future change makes them equal, that change has
snapped a computed centre to a convenient value, and it is a bug rather than a
tidy-up.

## Entries

Files are named `<topic>-<case>.nc`. `expectations.json` says, for each, whether
it should parse or be rejected, and with what.
