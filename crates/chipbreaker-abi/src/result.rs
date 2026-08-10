// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The result handle, and how a C caller reads an outcome.
//!
//! # One document, read one way
//!
//! A result always carries JSON, whether the run happened or was refused. The
//! two documents differ — `chipbreaker.verification-report` for a run,
//! `chipbreaker.refusal` for a decline — but they share `schema`,
//! `schema_version`, `verdict` and `verdict_rule` at the top level. A caller
//! that only wants the gate calls [`cb_result_pass`] and never learns which it
//! got; a caller that wants to explain the outcome reads
//! [`cb_result_message`], which is empty for a run that happened and the
//! engine's sentence for one that did not.
//!
//! The two accessors below exist so that the common cases need no JSON parser
//! at all. A host that wants to gate a build on the verdict can link this
//! library, call three functions, and never add a dependency.

use core::ffi::{c_char, c_int, c_void};

use crate::handle_ref;

/// The outcome of a run, owned by the caller until freed.
#[derive(Debug)]
pub struct CbResult {
    /// The document, NUL-terminated so a C caller can treat it as a string as
    /// well as a counted buffer. The NUL is not included in the reported
    /// length, matching every other string API a C programmer has used.
    json: std::ffi::CString,
    /// The refusal sentence, or empty.
    message: std::ffi::CString,
    /// Whether the verdict passed.
    pass: bool,
    /// Whether this was a refusal.
    refused: bool,
}

impl CbResult {
    /// Wraps a completed report.
    pub(crate) fn from_report(report: &chipbreaker_core::findings::Report) -> Self {
        let pass = report.verdict.pass();
        Self {
            json: cstring(report.to_json().to_string()),
            message: cstring(String::new()),
            pass,
            refused: false,
        }
    }

    /// Wraps a refusal.
    pub(crate) fn from_refusal(why: &str) -> Self {
        let refusal = chipbreaker_core::findings::Refusal::new(why);
        Self {
            json: cstring(refusal.to_json().to_string()),
            message: cstring(why.to_owned()),
            // A refusal never passes. Stated here as well as in the document
            // because this is the field a caller reads first.
            pass: false,
            refused: true,
        }
    }
}

/// Interior NULs cannot occur in JSON this engine produces, but a refusal
/// sentence is assembled from parser output, so the conversion is made total
/// rather than assumed safe.
fn cstring(s: String) -> std::ffi::CString {
    std::ffi::CString::new(s.replace('\0', " ")).unwrap_or_default()
}

/// The result document, as NUL-terminated UTF-8 JSON.
///
/// When `len` is not null it receives the length in bytes, **excluding** the
/// terminating NUL.
///
/// **Borrowed.** The pointer is valid until [`cb_result_free`] is called on
/// this handle, and must not be freed by the caller. Copy it if it needs to
/// outlive the handle.
///
/// Returns null if `result` is null.
///
/// # Safety
/// `result` must be null or a live handle from `cb_job_run`, and `len` must be
/// null or a valid pointer to a single writeable `size_t`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_result_json(result: *const c_void, len: *mut usize) -> *const c_char {
    let Some(r) = (unsafe { handle_ref::<CbResult>(result) }) else {
        if !len.is_null() {
            unsafe { *len = 0 };
        }
        return core::ptr::null();
    };
    if !len.is_null() {
        unsafe { *len = r.json.as_bytes().len() };
    }
    r.json.as_ptr()
}

/// Whether the verdict passed: `1` for yes, `0` for no.
///
/// **A conjunction over every gate, and an unchecked gate does not pass.** A
/// job with no nominal does not pass the gouge gate by default — it fails to
/// have been checked, which is a different and more useful thing to be told.
///
/// Returns `0` for a null handle, which is the safe direction: a caller that
/// mishandles the pointer is told the job did not pass rather than that it did.
///
/// # Safety
/// `result` must be null or a live handle from `cb_job_run`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_result_pass(result: *const c_void) -> c_int {
    unsafe { handle_ref::<CbResult>(result) }.map_or(0, |r| c_int::from(r.pass))
}

/// Whether this result is a refusal: `1` for yes.
///
/// Equivalent to having received [`CbStatus::Refused`] from the run, and
/// available here so a result can be interpreted after being passed around
/// without its status.
///
/// # Safety
/// As [`cb_result_pass`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_result_refused(result: *const c_void) -> c_int {
    unsafe { handle_ref::<CbResult>(result) }.map_or(0, |r| c_int::from(r.refused))
}

/// The refusal sentence, as NUL-terminated UTF-8.
///
/// Empty — a valid pointer to a single NUL, never null — for a run that
/// happened. **This is the string to show a user.** It names what was declined
/// and usually what to do instead: which resolution would fit, which dialect
/// the file is written in, why the control rather than the program decides an
/// offset path.
///
/// **Borrowed**, on the same terms as [`cb_result_json`].
///
/// # Safety
/// As [`cb_result_json`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_result_message(
    result: *const c_void,
    len: *mut usize,
) -> *const c_char {
    let Some(r) = (unsafe { handle_ref::<CbResult>(result) }) else {
        if !len.is_null() {
            unsafe { *len = 0 };
        }
        return core::ptr::null();
    };
    if !len.is_null() {
        unsafe { *len = r.message.as_bytes().len() };
    }
    r.message.as_ptr()
}

/// Releases a result.
///
/// Passing null is allowed and does nothing. Every pointer previously returned
/// by [`cb_result_json`] or [`cb_result_message`] for this handle is invalid
/// afterwards.
///
/// # Safety
/// `result` must be null, or a handle from `cb_job_run` that has not been
/// freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_result_free(result: *mut c_void) {
    if !result.is_null() {
        drop(unsafe { Box::from_raw(result.cast::<CbResult>()) });
    }
}

/// Runs a job through the shared assembly.
///
/// Thin on purpose: the assembly lives in one place so that the C ABI, the
/// browser build and the Python bindings cannot drift into answering the same
/// question differently.
#[allow(clippy::too_many_arguments, reason = "the builder's fields, in order")]
pub(crate) fn run_job(
    program: &str,
    tools: &str,
    stock_stl: &[u8],
    nominal_stl: Option<&[u8]>,
    tool_id: Option<&str>,
    source: Option<&str>,
    resolution_mm: f64,
    tolerance_mm: f64,
    clearance_mm: f64,
    memory_ceiling_bytes: Option<u64>,
) -> Result<chipbreaker_core::findings::Report, String> {
    chipbreaker_gcode::pipeline::run(&chipbreaker_gcode::pipeline::JobRequest {
        program,
        tools,
        stock_stl,
        nominal_stl,
        tool_id,
        source,
        resolution_mm,
        tolerance_mm,
        clearance_mm,
        memory_ceiling_bytes,
        // No cap. A native host runs on a workstation, and the browser's cap
        // exists because a tab does not.
        segment_cap: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refusal_carries_its_sentence_and_does_not_pass() {
        let r = CbResult::from_refusal("G41 is not supported because the control decides the path");
        assert!(!r.pass);
        assert!(r.refused);
        assert!(r.message.to_str().expect("utf-8").contains("G41"));
        let doc: serde_json::Value =
            serde_json::from_str(r.json.to_str().expect("utf-8")).expect("valid JSON");
        assert_eq!(doc["schema"], "chipbreaker.refusal");
        assert_eq!(doc["verdict"]["pass"], false);
    }

    #[test]
    fn a_null_result_reads_as_not_passing_rather_than_as_passing() {
        // The safe direction. A caller that mishandles the pointer must not be
        // told its program is clear.
        assert_eq!(unsafe { cb_result_pass(core::ptr::null()) }, 0);
        assert_eq!(unsafe { cb_result_refused(core::ptr::null()) }, 0);
        assert!(unsafe { cb_result_json(core::ptr::null(), core::ptr::null_mut()) }.is_null());
    }

    #[test]
    fn the_reported_length_excludes_the_terminator() {
        let r = CbResult::from_refusal("no");
        let boxed = Box::into_raw(Box::new(r)).cast::<c_void>();
        let mut len = 0usize;
        let ptr = unsafe { cb_result_json(boxed, &raw mut len) };
        assert!(!ptr.is_null());
        // A C caller doing `memcpy(dst, ptr, len)` must not copy the NUL, and
        // one doing `strlen(ptr)` must get the same number.
        let via_strlen = unsafe { core::ffi::CStr::from_ptr(ptr) }.to_bytes().len();
        assert_eq!(len, via_strlen);
        unsafe { cb_result_free(boxed) };
    }
}
