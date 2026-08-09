// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

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
        RESULT = (ptr as usize, len);
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
    match run_inner(input) {
        Ok(text) => {
            publish(text);
            1
        }
        Err(refusal) => {
            // A refusal is shaped like a report so the page never has to guess
            // which of two things it received.
            publish(
                serde_json::json!({
                    "schema": "chipbreaker.browser-refusal",
                    "schema_version": 1,
                    "refused": true,
                    "message": refusal,
                })
                .to_string(),
            );
            0
        }
    }
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

/// Reads an STL, binary or ASCII, in millimetres.
///
/// The demo accepts whatever a visitor drags in, and "binary or ASCII" is not a
/// question they should have to answer about their own file.
fn read_stl(bytes: &[u8]) -> Result<chipbreaker_core::mesh::TriMesh, String> {
    use chipbreaker_core::mesh::io::stl;
    use chipbreaker_core::mesh::units::Unit;
    if stl::looks_binary(bytes) {
        stl::read_binary(bytes, Unit::Millimetre).map_err(|e| e.to_string())
    } else {
        let text = core::str::from_utf8(bytes).map_err(|e| format!("not UTF-8: {e}"))?;
        stl::read_ascii(text, Unit::Millimetre).map_err(|e| e.to_string())
    }
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
#[allow(clippy::too_many_lines, reason = "one linear assembly of one job")]
fn run_inner(input: &[u8]) -> Result<String, String> {
    use chipbreaker_core::budget::{Budget, Spacing};
    use chipbreaker_core::deviation::compare;
    use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
    use chipbreaker_core::findings::cluster::{ClusterParams, cluster};
    use chipbreaker_core::findings::identify;
    use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
    use chipbreaker_core::tool::ToolLibrary;

    let text = core::str::from_utf8(input).map_err(|e| format!("the request is not UTF-8: {e}"))?;
    let v: serde_json::Value =
        serde_json::from_str(text).map_err(|e| format!("the request is not valid JSON: {e}"))?;

    let get_str = |k: &str| -> Result<String, String> {
        v[k].as_str()
            .map(str::to_owned)
            .ok_or_else(|| format!("the request has no {k}"))
    };
    let resolution = v["resolution_mm"].as_f64().unwrap_or(0.6);
    let tolerance = v["tolerance_mm"].as_f64().unwrap_or(0.1);
    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(format!(
            "resolution must be a positive length, got {resolution}"
        ));
    }

    // --- the stock mesh ---------------------------------------------------
    let stock_bytes = base64(&get_str("stock_stl")?)?;
    let stock =
        read_stl(&stock_bytes).map_err(|e| format!("the stock mesh could not be read: {e}"))?;

    // --- the ceiling, before anything is allocated ------------------------
    //
    // A tab that runs out of memory dies with no message. The refusal has to
    // happen here, and it has to name a resolution that would fit.
    let extents = stock.bounds().extent().to_array();
    let budget = Budget::bytes(BROWSER_CEILING_BYTES);
    if let Err(e) = budget.check(
        extents,
        Spacing::uniform(resolution),
        u64::try_from(BROWSER_SEGMENT_CAP).unwrap_or(u64::MAX),
        false,
    ) {
        return Err(e.to_string());
    }

    // --- the program ------------------------------------------------------
    let program = get_str("program")?;
    let library = ToolLibrary::from_json(&get_str("tools")?)
        .map_err(|e| format!("the tool library could not be read: {e}"))?;
    let (toolpath, _, _) = chipbreaker_gcode::resolve::parse(
        &program,
        "program",
        &chipbreaker_gcode::resolve::ParseOptions::default(),
        None,
    )
    .map_err(|e| e.to_string())?;

    if toolpath.segments.len() > BROWSER_SEGMENT_CAP {
        return Err(format!(
            "this program has {} segments and the browser build is capped at {}. \
             The cap is a property of running in a tab, not of the engine: the \
             native build has no such limit and is several times faster. Run it \
             locally, or try one of the bundled examples.",
            toolpath.segments.len(),
            BROWSER_SEGMENT_CAP
        ));
    }

    let tool_id = v["tool"].as_str();
    let profile = match tool_id {
        Some(id) => library
            .get(id)
            .ok_or_else(|| format!("no tool with id {id:?} in the library"))?
            .profile()
            .clone(),
        None => {
            let first = toolpath.segments.first().map_or(0, |s| s.tool);
            library
                .get_by_number(first)
                .ok_or_else(|| format!("no tool numbered {first} in the library"))?
                .profile()
                .clone()
        }
    };

    // --- cut --------------------------------------------------------------
    let (mut field, _) = TriDexelField::build(
        &stock,
        &TriBuildOptions {
            spacing: resolution,
            ..TriBuildOptions::default()
        },
    )
    .map_err(|e| format!("the stock field could not be built: {e}"))?;

    let motions: Vec<_> = toolpath
        .segments
        .iter()
        .filter_map(chipbreaker_core::toolpath::segment_motion)
        .collect();
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        &motions,
        SweepMethod::Analytic {
            tolerance: resolution / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );

    // --- compare, when a nominal was supplied -----------------------------
    let nominal = match v["nominal_stl"].as_str() {
        Some(b64) => {
            let bytes = base64(b64)?;
            Some(read_stl(&bytes).map_err(|e| format!("the nominal mesh could not be read: {e}"))?)
        }
        None => None,
    };

    let report = match &nominal {
        Some(n) => {
            let d = compare(&field, n, Some(&stock));
            let params = ClusterParams::for_spacing(resolution, tolerance);
            let findings = identify(cluster(&d.samples, &params, resolution), params.radius_mm);
            serde_json::json!({
                "schema": "chipbreaker.browser-result",
                "schema_version": 1,
                "refused": false,
                "engine_selftest": report().digest.to_hex(),
                "resolution_mm": resolution,
                "tolerance_mm": tolerance,
                "segments": toolpath.segments.len(),
                "volume_mm3": field.volume(),
                "findings": findings.iter().map(|f| serde_json::json!({
                    "id": f.id,
                    "class": f.class.as_str(),
                    "is_defect": f.is_defect(),
                    "worst_depth_mm": f.worst_depth_mm,
                    "sample_count": f.sample_count,
                    "at_mm": [f.at.x, f.at.y, f.at.z],
                })).collect::<Vec<_>>(),
            })
        }
        None => serde_json::json!({
            "schema": "chipbreaker.browser-result",
            "schema_version": 1,
            "refused": false,
            "engine_selftest": report().digest.to_hex(),
            "resolution_mm": resolution,
            "segments": toolpath.segments.len(),
            "volume_mm3": field.volume(),
            "findings": serde_json::Value::Null,
        }),
    };
    Ok(report.to_string())
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
