// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Building a job and running it.
//!
//! # Why a builder rather than one wide function
//!
//! A single `cb_run(stock, stock_len, program, program_len, tools, tools_len,
//! nominal, nominal_len, tool_id, resolution, tolerance, clearance, ceiling,
//! &out)` has thirteen parameters, four of which are optional, and a caller who
//! transposes two pointers of the same type gets a runtime error rather than a
//! compile one.
//!
//! Setters cost more calls and are worth it: each one names what it takes, each
//! one validates immediately, and adding an input later is an addition rather
//! than a signature change — which is exactly the difference between bumping
//! the ABI version and not.

use core::ffi::{c_char, c_double, c_void};

use crate::{CbResult, CbStatus, borrow, borrow_str, handle};

/// A job being assembled, then run.
///
/// Opaque to C. The fields are owned copies rather than borrowed pointers: a
/// caller that hands us a buffer and frees it before calling `cb_job_run` is
/// making a mistake this design simply does not have.
#[derive(Debug, Default)]
pub struct CbJob {
    program: String,
    tools: String,
    stock_stl: Vec<u8>,
    nominal_stl: Option<Vec<u8>>,
    tool_id: Option<String>,
    source: Option<String>,
    resolution_mm: Option<f64>,
    tolerance_mm: Option<f64>,
    clearance_mm: Option<f64>,
    ceiling_bytes: Option<u64>,
}

/// Creates an empty job.
///
/// Returns null only if allocation fails, which on a host large enough to run
/// this engine means the process is already in trouble.
///
/// The caller owns the result and must release it with [`cb_job_free`].
#[unsafe(no_mangle)]
pub extern "C" fn cb_job_new() -> *mut c_void {
    Box::into_raw(Box::new(CbJob::default())).cast()
}

/// Releases a job.
///
/// Passing null is allowed and does nothing, so a cleanup path need not test
/// first. Passing the same handle twice is undefined behaviour, as it is for
/// `free`.
///
/// # Safety
/// `job` must be null, or a handle from [`cb_job_new`] that has not been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_free(job: *mut c_void) {
    if !job.is_null() {
        drop(unsafe { Box::from_raw(job.cast::<CbJob>()) });
    }
}

/// Sets the NC program, as UTF-8 text.
///
/// Copied. The caller may free its buffer immediately.
///
/// # Safety
/// `text` must be valid for reads of `len` bytes, or null when `len` is zero.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_program(
    job: *mut c_void,
    text: *const c_char,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow_str(text, len) } {
        Ok(s) => {
            j.program = s.to_owned();
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Sets the tool library, as the JSON this engine's tool format uses.
///
/// # Safety
/// As [`cb_job_set_program`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_tools(
    job: *mut c_void,
    text: *const c_char,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow_str(text, len) } {
        Ok(s) => {
            j.tools = s.to_owned();
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Sets the stock mesh, as STL bytes. Binary or ASCII; the engine tells them
/// apart, because which one a customer's CAM system wrote is not a question
/// worth asking a caller.
///
/// # Safety
/// `bytes` must be valid for reads of `len` bytes, or null when `len` is zero.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_stock_stl(
    job: *mut c_void,
    bytes: *const u8,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow(bytes, len) } {
        Ok(b) => {
            j.stock_stl = b.to_vec();
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Sets the nominal part, as STL bytes.
///
/// **Optional, and its absence is not a defect.** Without it the gouge gate
/// reports `unchecked` rather than passing, because a gate that did not run has
/// certified nothing. A host doing collision checking alone can leave it unset
/// and read a report that says exactly that.
///
/// # Safety
/// As [`cb_job_set_stock_stl`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_nominal_stl(
    job: *mut c_void,
    bytes: *const u8,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow(bytes, len) } {
        Ok(b) => {
            j.nominal_stl = Some(b.to_vec());
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Selects a tool from the library by its `id`.
///
/// When unset, the tool named by the program's first `T` word is used.
///
/// # Safety
/// As [`cb_job_set_program`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_tool_id(
    job: *mut c_void,
    text: *const c_char,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow_str(text, len) } {
        Ok(s) => {
            j.tool_id = Some(s.to_owned());
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Records where these inputs came from, for a human reading the manifest.
///
/// **Not part of the report's identity.** The manifest hashes input *content*,
/// never paths, so that two runs of the same bytes from different directories
/// are recognisably the same run. This string is a courtesy to a reader six
/// months later, and nothing depends on it.
///
/// # Safety
/// As [`cb_job_set_program`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_source(
    job: *mut c_void,
    text: *const c_char,
    len: usize,
) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    match unsafe { borrow_str(text, len) } {
        Ok(s) => {
            j.source = Some(s.to_owned());
            CbStatus::Ok
        }
        Err(e) => e,
    }
}

/// Sets the dexel spacing in millimetres. Must be finite and positive.
///
/// # Safety
/// `job` must be a live handle from [`cb_job_new`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_resolution_mm(job: *mut c_void, mm: c_double) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    if !mm.is_finite() || mm <= 0.0 {
        return CbStatus::InvalidArgument;
    }
    j.resolution_mm = Some(mm);
    CbStatus::Ok
}

/// Sets the tolerance findings are judged against, in millimetres.
///
/// # Safety
/// As [`cb_job_set_resolution_mm`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_tolerance_mm(job: *mut c_void, mm: c_double) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    if !mm.is_finite() || mm <= 0.0 {
        return CbStatus::InvalidArgument;
    }
    j.tolerance_mm = Some(mm);
    CbStatus::Ok
}

/// Sets the clearance below which a pass is reported as a near miss.
///
/// Zero is legitimate and means "report contact only". Negative is not.
///
/// # Safety
/// As [`cb_job_set_resolution_mm`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_clearance_mm(job: *mut c_void, mm: c_double) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    if !mm.is_finite() || mm < 0.0 {
        return CbStatus::InvalidArgument;
    }
    j.clearance_mm = Some(mm);
    CbStatus::Ok
}

/// Caps how much memory a run may need, in bytes.
///
/// The check happens **before anything is allocated**, and exceeding it is a
/// refusal that names a resolution which would fit — not an allocation failure.
/// A host embedding this engine in a long-lived process should set it; the
/// alternative is discovering the limit through the operating system.
///
/// Zero means no ceiling.
///
/// # Safety
/// As [`cb_job_set_resolution_mm`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_set_memory_ceiling_bytes(job: *mut c_void, bytes: u64) -> CbStatus {
    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };
    j.ceiling_bytes = if bytes == 0 { None } else { Some(bytes) };
    CbStatus::Ok
}

/// Runs the job.
///
/// **Returns [`CbStatus::Refused`] as a success.** On both `Ok` and `Refused`,
/// `*out` receives a result handle carrying a JSON document; the caller owns it
/// and must release it with [`cb_result_free`]. On `InvalidArgument` and
/// `Internal`, `*out` is set to null and there is nothing to free.
///
/// The job handle is not consumed and may be run again, with or without
/// changing settings first. Re-running produces an independent result handle.
///
/// # Safety
/// `job` must be a live handle from [`cb_job_new`], and `out` must be a valid
/// pointer to a single writeable pointer.
///
/// [`cb_result_free`]: crate::result::cb_result_free
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cb_job_run(job: *mut c_void, out: *mut *mut c_void) -> CbStatus {
    if out.is_null() {
        return CbStatus::InvalidArgument;
    }
    // Null first, so that a caller who ignores the status and reads `*out`
    // finds nothing rather than whatever was on their stack.
    unsafe { *out = core::ptr::null_mut() };

    let Some(j) = (unsafe { handle::<CbJob>(job) }) else {
        return CbStatus::InvalidArgument;
    };

    let (result, status) = match j.execute() {
        Ok(report) => (CbResult::from_report(&report), CbStatus::Ok),
        Err(why) => (CbResult::from_refusal(&why), CbStatus::Refused),
    };
    unsafe { *out = Box::into_raw(Box::new(result)).cast() };
    status
}

impl CbJob {
    /// Runs the job, or explains why it will not.
    ///
    /// Every early return is a refusal carrying a sentence, never a code. This
    /// is the same assembly the browser build performs, and deliberately so:
    /// two entry points that assembled a job differently would eventually
    /// disagree about an answer, and the disagreement would be invisible.
    fn execute(&self) -> Result<chipbreaker_core::findings::Report, String> {
        crate::result::run_job(
            &self.program,
            &self.tools,
            &self.stock_stl,
            self.nominal_stl.as_deref(),
            self.tool_id.as_deref(),
            self.source.as_deref(),
            self.resolution_mm.unwrap_or(0.5),
            self.tolerance_mm.unwrap_or(0.1),
            self.clearance_mm.unwrap_or(0.0),
            self.ceiling_bytes,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_null_handle_is_rejected_rather_than_dereferenced() {
        let s = unsafe { cb_job_set_resolution_mm(core::ptr::null_mut(), 0.5) };
        assert_eq!(s, CbStatus::InvalidArgument);
    }

    #[test]
    fn freeing_null_is_allowed() {
        // So a cleanup path can be unconditional, the way `free(NULL)` is.
        unsafe { cb_job_free(core::ptr::null_mut()) };
    }

    #[test]
    fn a_resolution_that_is_not_a_length_is_a_caller_bug() {
        let job = cb_job_new();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                unsafe { cb_job_set_resolution_mm(job, bad) },
                CbStatus::InvalidArgument,
                "{bad} must be refused"
            );
        }
        assert_eq!(unsafe { cb_job_set_resolution_mm(job, 0.5) }, CbStatus::Ok);
        unsafe { cb_job_free(job) };
    }

    #[test]
    fn zero_clearance_is_legitimate_but_negative_is_not() {
        let job = cb_job_new();
        assert_eq!(unsafe { cb_job_set_clearance_mm(job, 0.0) }, CbStatus::Ok);
        assert_eq!(
            unsafe { cb_job_set_clearance_mm(job, -0.1) },
            CbStatus::InvalidArgument
        );
        unsafe { cb_job_free(job) };
    }

    #[test]
    fn run_nulls_the_out_pointer_before_it_can_fail() {
        // A caller that ignores the status and reads `*out` must find null
        // rather than an uninitialised value that looks like a handle.
        let mut out = core::ptr::dangling_mut::<c_void>();
        let s = unsafe { cb_job_run(core::ptr::null_mut(), &raw mut out) };
        assert_eq!(s, CbStatus::InvalidArgument);
        assert!(out.is_null());
    }
}
