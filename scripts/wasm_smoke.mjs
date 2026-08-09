// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// Exercises the browser build's job entry point the way a page would: a clean
// job, a gouged one, and a program the engine refuses.
//
// The refusal case is the point of this script as much as the working ones. A
// demo whose refusal path is untested will meet its first refusal in front of a
// visitor, and it will look like a crash.
//
//   node scripts/wasm_smoke.mjs [path/to/chipbreaker_wasm.wasm]

import { readFile } from 'node:fs/promises';
import process from 'node:process';

const path =
  process.argv[2] ??
  'target/wasm32-unknown-unknown/release/chipbreaker_wasm.wasm';
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
const wasm = instance.exports;

/** Writes a string into the module's memory and returns [ptr, len]. */
function put(text) {
  const bytes = new TextEncoder().encode(text);
  const ptr = wasm.alloc(bytes.length);
  new Uint8Array(wasm.memory.buffer, ptr, bytes.length).set(bytes);
  return [ptr, bytes.length];
}

/** Runs one request and returns the parsed result. */
function run(request) {
  const [ptr, len] = put(JSON.stringify(request));
  const started = performance.now();
  const ok = wasm.run(ptr, len);
  const ms = performance.now() - started;
  const out = new Uint8Array(
    wasm.memory.buffer,
    wasm.result_ptr(),
    wasm.result_len(),
  );
  const text = new TextDecoder().decode(out);
  wasm.dealloc(ptr, len);
  return { ok: ok === 1, ms, value: JSON.parse(text) };
}

/** A binary STL box, as base64. */
function box(lo, hi) {
  const [x0, y0, z0] = lo;
  const [x1, y1, z1] = hi;
  const v = [
    [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
    [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
  ];
  const t = [
    [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
    [2, 3, 7], [2, 7, 6], [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5],
  ];
  const buf = new ArrayBuffer(84 + t.length * 50);
  const view = new DataView(buf);
  view.setUint32(80, t.length, true);
  let o = 84;
  for (const tri of t) {
    o += 12;
    for (const i of tri) {
      for (const c of v[i]) {
        view.setFloat32(o, c, true);
        o += 4;
      }
    }
    o += 2;
  }
  return Buffer.from(new Uint8Array(buf)).toString('base64');
}

const tools = await readFile('tests/corpus/tool/standard-library.json', 'utf8');
const stock = box([0, 0, 0], [60, 40, 25]);

console.log(`segment cap    ${wasm.segment_cap()}`);
console.log(`memory ceiling ${(Number(wasm.memory_ceiling_bytes()) / 1048576).toFixed(0)} MiB`);
console.log();

const cases = [
  {
    name: 'clean pass (cold: includes the one-off self-test)',
    request: {
      stock_stl: stock,
      tools,
      tool: 'flat-6',
      resolution_mm: 0.6,
      program: 'G21 G90\nG0 Z50.\nG0 X-10. Y20.\nG0 Z18.\nG1 X70. F600.\nG0 Z50.\nM30\n',
    },
  },
  {
    name: 'cutter radius compensation',
    request: {
      stock_stl: stock,
      tools,
      tool: 'flat-6',
      program: 'G21 G90\nG41 D1\nG0 X0. Y20.\nG1 X60. F600.\nM30\n',
    },
  },
  {
    name: 'a dialect that is not this one',
    request: {
      stock_stl: stock,
      tools,
      tool: 'flat-6',
      program: 'N10 MSG("roughing")\nN20 CYCLE81(10, 0, 2, -15)\nM30\n',
    },
  },
  {
    name: 'resolution too fine for a tab',
    request: {
      stock_stl: stock,
      tools,
      tool: 'flat-6',
      resolution_mm: 0.02,
      program: 'G21 G90\nG0 Z50.\nG1 X60. F600.\nM30\n',
    },
  },
];

cases.push({ name: 'clean pass (warm)', request: cases[0].request });

for (const { name, request } of cases) {
  const { ok, ms, value } = run(request);
  console.log(`--- ${name} --- ${ms.toFixed(0)} ms`);
  if (ok) {
    console.log(`    ran: ${value.segments} segments, ${value.volume_mm3.toFixed(0)} mm3 left`);
  } else {
    console.log(`    REFUSED: ${value.message.replace(/\s+/g, ' ').slice(0, 220)}`);
  }
  console.log();
}
