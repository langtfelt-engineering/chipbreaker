# chipbreaker

The command-line front end for
[Chipbreaker](https://github.com/spanwerk/chipbreaker), a material-removal
simulation and machining verification engine.

```sh
cargo install chipbreaker
```

Given a block of stock, a set of cutting tools and a CNC toolpath, Chipbreaker
answers: **what shape is left at the end, and does it match the part you
intended?**

```sh
chipbreaker dexel build stock.stl --units mm --res 0.4 --axes xyz --out stock.tdx
chipbreaker run --stock stock.tdx --path part.nc --tools library.json --out cut.tdx
chipbreaker compare cut.tdx --nominal part.stl --units mm --tolerance 0.05
```

```text
GOUGE      worst 1.0000 mm over 1102 samples
EXCESS     worst 0.0000 mm over 0 samples

verdict    GOUGED above tolerance
```

Gouges and excess stock are reported as **two numbers and never one**, and only
gouges decide the exit code — material left standing is what a roughing pass is
supposed to leave.

`chipbreaker --help` lists the rest: mesh inspection and validation, tool
tessellation and ray casting, toolpath parsing, dexel field building and slicing,
surface extraction, memory estimation, and a deterministic self-test.

## The guarantee

**The same input produces bit-identical output across runs, thread counts,
platforms, and the WASM build.** `chipbreaker selftest --report json` emits a
canonically hashed `results` section; the same command under `wasmtime` produces
the same hash, and CI checks that on Linux, macOS, Windows and `wasm32-wasip1`
on every push.

## What it does not do

5-axis or any tilted tool, turning and mill-turn, cutter radius compensation,
Siemens or Heidenhain dialects, macro programming, and flutes. The
[repository README](https://github.com/spanwerk/chipbreaker#scope) sets out each
and why.

Chipbreaker verifies a **program** against an **ideal geometric cutting model**.
It does not model tool wear, deflection, thermal growth, runout, backlash or
controller interpolation, and it is not a safety interlock.

## Licence

Copyright (C) 2026 Langtfelt. Dual-licensed: GPL-3.0-or-later, or a commercial
licence for use in a proprietary product. See the
[repository](https://github.com/spanwerk/chipbreaker) for details.
