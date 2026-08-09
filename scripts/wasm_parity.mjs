// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// Runs the engine's self-test in a browser-class WebAssembly engine (V8, via
// Node) and prints the digest.
//
// The point of this harness is that it is *crude*. It instantiates the module,
// calls four functions that return integers, and reassembles a hex string. It
// reads no memory, marshals no structures, and depends on nothing in the wasm
// crate except the four exports. If the digest it prints matches the published
// one, that agreement cannot be an artifact of the harness.
//
// Usage:
//   node scripts/wasm_parity.mjs [path/to/chipbreaker_wasm.wasm]

import { readFile } from 'node:fs/promises';
import process from 'node:process';

const path =
  process.argv[2] ??
  'target/wasm32-unknown-unknown/release/chipbreaker_wasm.wasm';

const bytes = await readFile(path);
const started = performance.now();
// No imports at all: `wasm32-unknown-unknown` has no WASI and this module asks
// the host for nothing. An import object here would be a sign something had
// crept in.
const { instance } = await WebAssembly.instantiate(bytes, {});
const instantiated = performance.now();

const {
  selftest_digest_word,
  selftest_suite_count,
  selftest_case_count,
  selftest_passed,
} = instance.exports;

const ran = performance.now();
let hex = '';
for (let w = 0; w < 4; w += 1) {
  hex += selftest_digest_word(w).toString(16).padStart(16, '0');
}
const finished = performance.now();

const suites = selftest_suite_count();
const cases = selftest_case_count();
const passed = selftest_passed() === 1;

console.log(`engine        ${process.version} / V8 ${process.versions.v8}`);
console.log(`module        ${(bytes.length / 1024).toFixed(0)} KiB`);
console.log(`instantiate   ${(instantiated - started).toFixed(1)} ms`);
console.log(`self-test     ${(finished - ran).toFixed(1)} ms`);
console.log(`suites        ${suites}`);
console.log(`cases         ${cases}`);
console.log(`passed        ${passed}`);
console.log(`digest        ${hex}`);

if (!passed) {
  console.error('a suite failed in the browser build');
  process.exit(1);
}

// The published digest, when given, is compared here so CI can gate on it.
const expected = process.env.CHIPBREAKER_EXPECTED_DIGEST;
if (expected) {
  if (expected === hex) {
    console.log('parity        MATCHES the published digest');
  } else {
    console.error(`parity        DIFFERS\n  expected ${expected}\n  browser  ${hex}`);
    process.exit(1);
  }
}
