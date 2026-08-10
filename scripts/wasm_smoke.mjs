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
  // Read before and after. If the counter did not advance by exactly one, the
  // bytes at result_ptr() belong to some other call and reading them would
  // present as a stale result rather than as an error -- the worst failure
  // this surface has, because it looks like an answer.
  const before = wasm.result_generation();
  const started = performance.now();
  const ok = wasm.run(ptr, len);
  const ms = performance.now() - started;
  const after = wasm.result_generation();
  if (after !== before + 1) {
    throw new Error(
      `result generation went ${before} -> ${after}; the buffer is not ours`,
    );
  }
  // The view is constructed after the call, never cached: a growing linear
  // memory detaches every ArrayBuffer taken before it grew.
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

const NL = String.fromCharCode(10);
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

cases.push({
  name: 'a stub tool in a bulky chuck, cutting a 25 mm deep slot',
  request: {
    stock_stl: stock,
    tools,
    // 10 mm of flute under a 50.8 mm ER32 chuck body. Reaching the bottom of
    // a 25 mm block puts the chuck into the material, which is the whole
    // point: the collision gate has to be able to *fail*, not only to report
    // `unchecked` because no tool in the library carries a holder.
    tool: 'er32-stub-6',
    resolution_mm: 0.6,
    program:
      'G21 G90' + NL + 'G0 Z50.' + NL + 'G0 X30. Y20.' + NL +
      'G1 Z-1. F200.' + NL + 'G1 X50. F600.' + NL + 'G0 Z50.' + NL + 'M30' + NL,
  },
});

cases.push({ name: 'clean pass (warm)', request: cases[0].request });

for (const { name, request } of cases) {
  const { ok, ms, value } = run(request);
  console.log(`--- ${name} --- ${ms.toFixed(0)} ms`);
  console.log(`    ${value.schema} v${value.schema_version}`);
  if (ok) {
    const gates = Object.entries(value.verdict.gates)
      .map(([g, o]) => `${g}=${o.state}`)
      .join(' ');
    const cmp = value.numerical_semantics.comparison;
    console.log(`    verdict pass=${value.verdict.pass}  ${gates}`);
    console.log(
      `    ${value.summary.total} finding(s), ${value.summary.collisions} collision(s), ` +
        `${value.summary.near_misses} near miss(es)`,
    );
    console.log(
      `    error budget: ${value.numerical_semantics.swept_volumes.ray_cuts_exact} exact / ` +
        `${value.numerical_semantics.swept_volumes.ray_cuts_bounded} bounded ray-cuts, ` +
        `worst ${value.numerical_semantics.swept_volumes.worst_bound_mm} mm`,
    );
    console.log(
      `    comparison: ${cmp.available ? 'ran' : 'unavailable -- ' + cmp.why.slice(0, 60) + '...'}`,
    );
    console.log(`    manifest ${value.manifest.digest.slice(0, 16)}...`);
  } else {
    console.log(`    verdict pass=${value.verdict.pass}`);
    console.log(`    REFUSED: ${value.message.replace(/\s+/g, ' ').slice(0, 200)}`);
  }
  console.log();
}
