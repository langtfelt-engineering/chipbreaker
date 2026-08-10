# chipbreaker

Material-removal simulation and machining verification.

The engine computes what an NC program will leave behind, compares it against
the part you meant to make, and reports what it found together with the error
budget that makes the finding evidence rather than assertion.

```python
import chipbreaker

report = chipbreaker.verify(
    program="part.nc",
    tools="library.json",
    stock="stock.stl",
    nominal="part.stl",
    resolution_mm=0.4,
)

if report["verdict"]["pass"]:
    print("clear")
else:
    for gate, outcome in report["verdict"]["gates"].items():
        print(gate, outcome["state"], outcome.get("why", ""))
```

## Two things worth knowing before you start

**A refusal is an answer.** The engine declines jobs it cannot answer for, and
says why in a sentence written for a person:

```python
try:
    chipbreaker.verify(program="roughing.mpf", tools=..., stock=...)
except chipbreaker.Refused as exc:
    print(exc)
    # line 5: this looks like Siemens 840D, which is a different language
    # rather than a dialect of RS-274 (found "DEF REAL")
```

Show it and stop. Retrying produces the same sentence, because nothing about
the input has changed.

**An unchecked gate does not pass.** Leaving out `nominal` does not tell you the
part is good; it tells you the gouge gate never ran. `report["verdict"]["pass"]`
is `False`, and the gate says why. That is deliberate — the alternative is a
tool that reports success for a check it did not perform.

## Building from source

Wheels are not published yet. Building needs a Rust toolchain and
[maturin](https://www.maturin.rs/):

```sh
pip install maturin
cd crates/chipbreaker-python
maturin build --release          # produces a wheel in ../../target/wheels
pip install ../../target/wheels/chipbreaker-*.whl
```

Or, for a development install into the current environment:

```sh
maturin develop --release
```

The extension is built `abi3`, so one artifact serves every Python from 3.9
upward.

## Verifying the build

The engine's self-test digest is identical on every target it builds for,
including WebAssembly. It identifies the build's *behaviour* rather than the
build:

```python
>>> chipbreaker.selftest_digest()
'1ccc6660b31f67ad5092a0248a89fc1c38cf1b9c2cf1fe5e4c022803f689c038'
>>> chipbreaker.selftest_case_count()
27054
```

The first call runs the suites and takes about a second and a half; afterwards
it is free.

## Licence

GPL-3.0-or-later, or a commercial licence. See the repository root.

## What this does not cover

A deviation bound compares the computed stock against the **ideal geometric
cutting model**. It says nothing about tool wear, deflection under load,
thermal growth, spindle runout, backlash, or how a particular control
interpolates between programmed points. Chipbreaker verifies the program, not
the machine and not the part. Every report states this in its own `exclusions`
and `scope` sections.
