// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// ALLOW-UNSAFE-ABI-BOUNDARY
//
// **The one crate in this workspace that cannot forbid unsafe.** Handing a raw
// pointer across a language edge is unsafe by construction, and a crate that
// wrote `#![forbid(unsafe_code)]` over the top of an `extern "C"` surface would
// be making a claim it cannot keep. Saying so is better than a rule that holds
// only because nobody looked.
//
// What still holds: `unsafe_op_in_unsafe_fn` is denied, so every unsafe
// operation is written out inside its own block rather than inherited from the
// function signature; the engine itself — every crate that computes anything —
// still forbids unsafe entirely; and the unsafe here is confined to four
// functions that move bytes in and out, none of which does arithmetic.
#![deny(unsafe_op_in_unsafe_fn)]

//! The browser build of the engine.
//!
//! # This is not an API
//!
//! It is the smallest surface that lets a page run the engine: hand it bytes,
//! get a report back. There is no versioning, no stability promise, and nothing
//! here is documented for an integrator to build against — the C ABI is the
//! integration surface, and designing a second one in JavaScript would be
//! committing to something nobody has decided to maintain.
//!
//! # Why the exports look like this
//!
//! `wasm32-unknown-unknown` has no WASI, so there is no allocator hook, no
//! `stdout`, and no host to ask for anything. Every export is therefore a plain
//! integer function or a pointer into this module's own linear memory, allocated
//! here and freed here. The caller reads bytes out; it never passes ownership
//! in either direction.
//!
//! # The digest exports exist to be doubted
//!
//! [`selftest_digest_word`] returns the self-test digest a quarter at a time, as
//! integers, needing no memory access at all. That makes it callable from the
//! crudest possible harness — `WebAssembly.instantiate` and four calls — so a
//! sceptic can check the browser build's digest against the published one
//! without trusting any of the marshalling below.

use std::sync::OnceLock;

/// The self-test report, computed once.
///
/// The suites are deterministic, so a second run would produce the same digest
/// at the same cost. A browser tab has better things to do.
fn report() -> &'static chipbreaker_core::selftest::SelfTestReport {
    static REPORT: OnceLock<chipbreaker_core::selftest::SelfTestReport> = OnceLock::new();
    REPORT
        .get_or_init(|| chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites()))
}

/// One quarter of the self-test digest, as a big-endian `u64`.
///
/// `word` is 0 to 3; anything else returns zero, which cannot be mistaken for a
/// digest quarter in practice and needs no error channel.
///
/// Returning integers rather than a string is deliberate: a caller can check
/// this build's digest against the published one with four calls and no memory
/// marshalling, so the parity claim does not rest on the rest of this file
/// being correct.
#[unsafe(no_mangle)]
pub extern "C" fn selftest_digest_word(word: u32) -> u64 {
    let bytes = report().digest.as_bytes();
    let Some(chunk) = bytes.get((word as usize) * 8..(word as usize) * 8 + 8) else {
        return 0;
    };
    let mut out = [0u8; 8];
    out.copy_from_slice(chunk);
    u64::from_be_bytes(out)
}

/// How many suites ran, so a caller can tell a truncated build from a divergent
/// one: a differing digest with a differing suite count is a build problem,
/// and a differing digest with the same count is a determinism problem.
#[unsafe(no_mangle)]
pub extern "C" fn selftest_suite_count() -> u32 {
    u32::try_from(report().suites.len()).unwrap_or(u32::MAX)
}

/// Total cases across every suite.
#[unsafe(no_mangle)]
pub extern "C" fn selftest_case_count() -> u32 {
    u32::try_from(report().suites.iter().map(|s| s.cases).sum::<usize>()).unwrap_or(u32::MAX)
}

/// Whether every suite passed.
#[unsafe(no_mangle)]
pub extern "C" fn selftest_passed() -> u32 {
    u32::from(report().passed())
}

// ---------------------------------------------------------------------------
// The demo entry point
// ---------------------------------------------------------------------------

/// The memory ceiling the browser build always applies.
///
/// **Mandatory here, unlike the CLI.** A native run that asks for too much gets
/// a refusal printed and an exit code; a browser tab that asks for too much
/// dies, taking the page and any explanation with it. There is no message to
/// read afterwards, so the check has to happen before anything is allocated.
///
/// 256 MiB against a tab's practical few hundred: deliberately conservative,
/// because the cost of refusing a job that would have fitted is a sentence
/// telling the visitor what resolution would, and the cost of the other mistake
/// is a blank tab.
const BROWSER_CEILING_BYTES: u64 = 256 * 1024 * 1024;

/// The largest program the browser build will replay.
///
/// Stated on the page **before** the first run rather than discovered by
/// waiting. A visitor watching a spinner is forming an opinion about the
/// engine's speed; one who was told the cap up front is forming an opinion
/// about its honesty.
const BROWSER_SEGMENT_CAP: usize = 20_000;

/// Reserves `len` bytes for the caller to write into.
///
/// The caller writes input here and passes the pointer back to [`run`]. Memory
/// is allocated and freed on this side only; the JavaScript side never owns
/// anything.
///
/// # Panics
/// If the allocator refuses, which in a browser means the tab is already out of
/// memory and there is nothing useful to return.
#[unsafe(no_mangle)]
pub extern "C" fn alloc(len: u32) -> *mut u8 {
    let mut buf = Vec::<u8>::with_capacity(len as usize);
    let ptr = buf.as_mut_ptr();
    core::mem::forget(buf);
    ptr
}

/// Releases what [`alloc`] returned.
///
/// # Safety
/// `ptr` must have come from [`alloc`] with the same `len`, and must not have
/// been freed already.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dealloc(ptr: *mut u8, len: u32) {
    if !ptr.is_null() {
        drop(unsafe { Vec::from_raw_parts(ptr, 0, len as usize) });
    }
}

/// Where the last result lives, so the caller can read it out.
///
/// A pair of statics rather than a returned struct: `wasm32-unknown-unknown`
/// has no stable multi-value ABI across every toolchain that might build this,
/// and two integer reads are simpler than agreeing on one.
static mut RESULT: (usize, usize) = (0, 0);

/// How many results have been published. See [`result_generation`].
static mut GENERATION: usize = 0;

/// Pointer to the last result produced by [`run`].
#[unsafe(no_mangle)]
pub extern "C" fn result_ptr() -> *const u8 {
    unsafe { RESULT.0 as *const u8 }
}

/// Length of the last result produced by [`run`].
#[unsafe(no_mangle)]
pub extern "C" fn result_len() -> u32 {
    u32::try_from(unsafe { RESULT.1 }).unwrap_or(0)
}

fn publish(text: String) {
    let bytes = text.into_bytes().into_boxed_slice();
    let len = bytes.len();
    let ptr = Box::into_raw(bytes).cast::<u8>();
    unsafe {
        // The previous result is released before the new one replaces it.
        // Leaking looked harmless for a demo -- one report is a few kilobytes
        // -- but a visitor cycling through presets pays for every one of them,
        // and a tab that grows without bound as somebody explores is exactly
        // the failure the memory ceiling exists to prevent.
        if RESULT.0 != 0 {
            drop(Vec::from_raw_parts(RESULT.0 as *mut u8, RESULT.1, RESULT.1));
        }
        RESULT = (ptr as usize, len);
        GENERATION = GENERATION.wrapping_add(1);
    }
}

/// Runs one job and publishes a report.
///
/// The input is a JSON envelope: the NC program and the tool library as text,
/// the stock and nominal meshes as arrays of bytes. That is the whole contract,
/// and it is deliberately not documented anywhere a stranger would find it —
/// the C ABI is the integration surface, and a second one grown here by
/// accident would be a maintenance promise nobody made.
///
/// Returns `1` on success and `0` on a refusal or an error; either way the
/// result is a JSON document, because **a refusal is a result**. The page
/// renders both, and a refusal renders like an answer rather than like a crash.
///
/// # Safety
/// `ptr` and `len` must describe a buffer this module allocated via [`alloc`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn run(ptr: *const u8, len: u32) -> u32 {
    let input = unsafe { core::slice::from_raw_parts(ptr, len as usize) };
    let (text, ok) = match run_inner(input) {
        Ok(report) => (report.to_json().to_string(), 1),
        // A refusal is a document of the same family, carrying the same
        // `verdict` at the same key, so a page that only wants the gate reads
        // it without branching and gets `false`.
        Err(why) => (
            chipbreaker_core::findings::Refusal::new(why)
                .to_json()
                .to_string(),
            0,
        ),
    };
    publish(text);
    ok
}

/// How many results this module has published, counting from one.
///
/// A page reads its result through [`result_ptr`] and [`result_len`], which
/// describe **the last** result and nothing else. Two jobs in flight, or a
/// render that outlives the run that produced it, and a caller can read a
/// buffer belonging to a different request while believing it read its own —
/// which presents as stale output rather than as an error, and a silent wrong
/// answer is the worst failure this surface can have.
///
/// So the counter is exported. Read it before the call and after, and if it did
/// not advance by exactly one, the bytes are not yours.
///
/// The real fix is one job at a time in a worker, which is what the page does.
/// This is how the page can *prove* it, and how anything else finds out cheaply
/// that it has not.
#[unsafe(no_mangle)]
pub extern "C" fn result_generation() -> u32 {
    u32::try_from(unsafe { GENERATION }).unwrap_or(u32::MAX)
}

/// The largest job this build will accept, so a page can say so before running.
#[unsafe(no_mangle)]
pub extern "C" fn segment_cap() -> u32 {
    u32::try_from(BROWSER_SEGMENT_CAP).unwrap_or(u32::MAX)
}

/// The memory ceiling this build applies, in bytes.
#[unsafe(no_mangle)]
pub extern "C" fn memory_ceiling_bytes() -> u64 {
    BROWSER_CEILING_BYTES
}

/// Decodes standard base64, ignoring whitespace.
///
/// Twenty lines rather than a dependency. The browser build is downloaded by a
/// visitor, so every crate that reaches it is bytes they pay for, and this is
/// the only place the engine needs base64 at all.
fn base64(text: &str) -> Result<Vec<u8>, String> {
    const BAD: u8 = 255;
    let value = |c: u8| -> u8 {
        match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => BAD,
        }
    };
    let mut out = Vec::with_capacity(text.len() / 4 * 3);
    let mut acc: u32 = 0;
    let mut bits = 0u32;
    for c in text.bytes() {
        if c.is_ascii_whitespace() || c == b'=' {
            continue;
        }
        let v = value(c);
        if v == BAD {
            return Err(format!(
                "the input is not base64: it contains {:?}",
                char::from(c)
            ));
        }
        acc = (acc << 6) | u32::from(v);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            #[allow(clippy::cast_possible_truncation, reason = "masked to a byte")]
            out.push(((acc >> bits) & 0xFF) as u8);
        }
    }
    Ok(out)
}

/// Runs one job, or explains why it will not.
///
/// Every early return here is a **refusal with a reason**, not an error code.
/// The engine already writes these to be read by a person; this function's job
/// is to carry them out intact rather than flattening them into "failed".
///
/// # The report is the real one
///
/// This emits `chipbreaker.verification-report` — the same schema, at the same
/// version, as `chipbreaker verify` writes. It briefly did not: an earlier
/// version of this file invented a `chipbreaker.browser-result` with eight
/// fields, no manifest, no numerical semantics and no verdict.
///
/// That was a mistake worth naming. The page this demo sits on argues that a
/// finding is worth what its error budget says it is worth. A reduced schema is
/// not a smaller version of that argument; it is a different artifact that
/// happens to share a name, and a reader who downloads one and finds a stub has
/// been handed a reason to disbelieve the rest of the page.
///
/// Where a browser run genuinely cannot fill a field it is marked absent with a
/// reason — the pattern `numerical_semantics.comparison` and `sweep` already
/// use. An honest report with stated gaps is the product; a second schema is a
/// second product.
fn run_inner(input: &[u8]) -> Result<chipbreaker_core::findings::Report, String> {
    let text = core::str::from_utf8(input).map_err(|e| format!("the request is not UTF-8: {e}"))?;
    let v: serde_json::Value =
        serde_json::from_str(text).map_err(|e| format!("the request is not valid JSON: {e}"))?;

    let get_str = |k: &str| -> Result<String, String> {
        v[k].as_str()
            .map(str::to_owned)
            .ok_or_else(|| format!("the request has no {k}"))
    };

    let stock_stl = base64(&get_str("stock_stl")?)?;
    let nominal_stl = match v["nominal_stl"].as_str() {
        Some(b64) => Some(base64(b64)?),
        None => None,
    };
    let program = get_str("program")?;
    let tools = get_str("tools")?;

    // Everything above this line is browser-specific: base64 in, a JSON
    // envelope, and the two limits a tab imposes. Everything below is the
    // *shared* assembly, which is the point -- the C ABI runs the identical
    // code, so the two cannot drift into answering the same question
    // differently while every determinism test still passes.
    chipbreaker_gcode::pipeline::run(&chipbreaker_gcode::pipeline::JobRequest {
        program: &program,
        tools: &tools,
        stock_stl: &stock_stl,
        nominal_stl: nominal_stl.as_deref(),
        tool_id: v["tool"].as_str(),
        source: v["source"].as_str(),
        resolution_mm: v["resolution_mm"].as_f64().unwrap_or(0.6),
        tolerance_mm: v["tolerance_mm"].as_f64().unwrap_or(0.1),
        clearance_mm: v["clearance_mm"].as_f64().unwrap_or(0.0),
        // Mandatory here, unlike the CLI: a tab that runs out of memory dies
        // with no message, so the refusal has to happen before anything is
        // allocated and has to name a resolution that would fit.
        memory_ceiling_bytes: Some(BROWSER_CEILING_BYTES),
        segment_cap: Some(BROWSER_SEGMENT_CAP),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_digest_words_reassemble_the_published_digest() {
        // The four words are the digest, in order, with nothing lost. If this
        // ever fails the parity harness is measuring something other than the
        // number the engine publishes.
        let mut bytes = Vec::new();
        for w in 0..4 {
            bytes.extend_from_slice(&selftest_digest_word(w).to_be_bytes());
        }
        let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(hex, report().digest.to_hex());
        assert_eq!(hex.len(), 64);
    }

    #[test]
    fn a_word_out_of_range_is_zero_rather_than_a_panic() {
        // A browser caller has no way to receive a panic usefully, so the
        // out-of-range answer is a value rather than a trap.
        assert_eq!(selftest_digest_word(4), 0);
        assert_eq!(selftest_digest_word(u32::MAX), 0);
    }

    #[test]
    fn the_counts_are_reported_and_the_suites_pass() {
        assert!(selftest_passed() == 1);
        assert!(selftest_suite_count() >= 15);
        assert!(selftest_case_count() > 20_000);
    }
}
