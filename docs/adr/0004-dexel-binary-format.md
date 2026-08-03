# ADR 0004: `.dexel` stores raw IEEE-754 bit patterns, not text

- **Status:** accepted
- **Date:** 2026-08-03
- **Unit:** 5 (single-axis dexel field)
- **Supersedes:** nothing
- **Related:** [ADR 0001](0001-spans-arena.md) (span storage), `CONTRIBUTING.md` § fixture precision

## Context

Unit 5 introduces the first Chipbreaker file format that carries computed
floating-point geometry: `.dexel`, a serialized dexel field. Later units will
write it (U6 for three bundles, U9 for checkpoints, U14 for verification
evidence) and read it back expecting the reloaded field to be **the same field**
— not a field that agrees to fifteen digits.

Every earlier format we touch is text. The tool library is JSON, the toolpath
corpus is G-code, the goldens are hex digests in text files. Continuing that way
for `.dexel` would be the path of least resistance and would be a mistake, and
this ADR exists because that mistake has already been made once in this project.

At Unit 3 we found that `serde_json` reads floats one ULP low: the tool-library
fixture value `2.0481555856608242` came back as `2.048155585660824`. The parser
was not doing anything unreasonable — it was doing correctly-rounded shortest
decimal parsing without the `float_roundtrip` feature. The fix was a feature
flag. But the deeper lesson is that a text float format has a *correctness
dependency on the parser*, and that dependency is invisible until a value with
seventeen significant digits crosses it. Our own tests missed it for a week
because the fixture happened to contain only round numbers.

A dexel field is not a fixture. It is millions of computed span endpoints, every
one of them the output of a ray-triangle intersection, and essentially none of
them round. If the format is text, then:

- Every writer must emit shortest-round-trip decimal (17 significant digits is
  sufficient but not what most formatters produce by default).
- Every reader must parse with correct rounding.
- `-0.0`, subnormals, infinities and NaN each need an agreed spelling, and the
  agreement has to hold across Rust, whatever the WASM host is written in, and
  whatever a customer's Python post-processor does.
- Every one of those is a place where a future contributor, a dependency
  upgrade, or a different locale can silently change a value by one ULP.

One ULP is not a rounding detail here. The whole determinism contract of the
product is that two runs produce **bit-identical** output, enforced by BLAKE3
digests over canonical binary encodings. A field that round-trips to within one
ULP produces a different digest, which is indistinguishable from a real
regression. We would be manufacturing false alarms in the exact mechanism whose
value depends on having none.

## Decision

**`.dexel` stores `f64` values as their raw IEEE-754 bit patterns, in
little-endian byte order, and never as text.**

Concretely:

1. Every float in the format — span endpoints, lattice spacing, origin, stock
   placement transform — is written with `f64::to_bits().to_le_bytes()` and read
   with `f64::from_bits(u64::from_le_bytes(..))`. No formatting, no parsing, no
   locale.
2. Every index and count is `u32` little-endian, per the standing rule that
   anything hashed or serialized uses `u32` because WASM is 32-bit.
3. The header carries a magic, a format version, and the lattice description, so
   a reader can refuse a file it does not understand rather than misinterpret it.
4. The round-trip requirement is **bit-identical**, tested as such: write a
   field, read it back, and assert the canonical hashes are equal. Not
   `approx_eq`. If a future change makes that test need a tolerance, the change
   is wrong.
5. Little-endian is fixed by the format, not inherited from the host. Every
   platform we target (x86-64, aarch64, wasm32) is little-endian, so today this
   costs nothing; writing it down means a big-endian port byte-swaps instead of
   producing files that silently disagree.

### On NaN and `-0.0`

The canonical hashing layer already canonicalises NaN to a single bit pattern
and `-0.0` to `+0.0`, because two values that compare equal must hash equal.
The **file format does not do this**. It stores what is there.

That asymmetry is deliberate. Hashing answers "are these the same field?", where
`-0.0` and `+0.0` are the same field. Serialization answers "can I reconstruct
exactly what I had?", where silently rewriting a value is data loss. A field
containing `-0.0` should reload containing `-0.0`. Construction should not
produce NaN at all, and if it ever does, the file preserving it is what lets us
find out where it came from; a format that quietly normalised it away would
destroy the evidence.

## Consequences

**Good.**

- Round-trip is exact by construction rather than by care. There is no parser
  behaviour to depend on, so there is no `float_roundtrip` equivalent waiting to
  be forgotten.
- Files are roughly 8 bytes per float rather than 20-ish, and a large field is
  mostly floats. A 2000x2000 field with one span per ray is 64 MB binary against
  something like 170 MB of text, before any compression.
- Reading is a memory copy rather than millions of decimal conversions.
- The format is *checkable*: a file either has the expected byte length for its
  declared counts or it does not.

**Bad, and accepted.**

- The format is not human-readable. Mitigated by `dexel stat`, which prints the
  header and the span distribution, and by `dexel slice`, which emits a readable
  cross-section. Debuggability is a tooling problem, and tooling is cheaper than
  a correctness dependency on float formatting.
- It is not diffable, so `.dexel` files do not belong in the corpus as goldens.
  We store the **hash** of a field as the golden and regenerate the field. This
  is already the pattern for the toolpath IR.
- A hand-written third-party reader is more work than `json.load`. Accepted: the
  audience for this format is Chipbreaker and its embedders, and the CLI is the
  supported route for everyone else.

## Alternatives rejected

**Text with 17 significant digits.** Round-trips correctly *if* every writer and
reader is correct. Unit 3 demonstrated that a widely used, well-maintained JSON
library is not correct by default. We would be betting the determinism contract
on a property we cannot enforce at the boundary.

**Text plus a checksum.** Detects the problem instead of preventing it. It turns
a silent one-ULP drift into a loud failure, which is better, but the failure is
still unfixable by the person holding the file.

**A general binary format (CBOR, MessagePack, Protobuf).** All of these store
`f64` as IEEE bits, so they solve the actual problem. Rejected for the cost:
each brings a dependency and an encoding layer whose canonicality we would then
have to verify — map ordering, optional-field presence, integer width promotion
— for a format whose entire schema is a fixed header followed by a flat array.
The place we need certainty is exactly the place a general format gives us
someone else's defaults.

**Reusing the golden hashing encoder as the file format.** Tempting, since it is
already canonical binary. Rejected because it canonicalises NaN and `-0.0`,
which is right for hashing and wrong for storage, as above; and because a hash
encoder is free to be lossy in ways a format must not be. Two jobs, two
encoders, one of which is allowed to change without breaking files on disk.

## Enforcement

- `dexel_roundtrip_is_bit_identical` writes, reads, and compares canonical
  hashes. A tolerance appearing in that test is the signal that this ADR has
  been violated.
- A test writes the same field twice and asserts the two byte streams are equal,
  which catches any iteration order that reached the encoder.
- The format version in the header is bumped whenever the layout changes, and a
  reader refuses an unknown version.
