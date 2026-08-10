// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

// ALLOW-UNSAFE-ABI-BOUNDARY
//
// A C ABI cannot forbid unsafe: the whole surface receives raw pointers from a
// caller this crate cannot see, and `#![forbid(unsafe_code)]` written over it
// would be a claim the code cannot keep.
//
// What still holds, and what makes the exception narrow rather than a hole:
//
// * `unsafe_op_in_unsafe_fn` is denied, so every unsafe operation appears in
//   its own block rather than being inherited from a function signature.
// * **Nothing here computes a geometric answer.** This crate moves bytes,
//   checks pointers for null, and calls the engine. Every crate that decides
//   anything still carries `#![forbid(unsafe_code)]`, and that is the property
//   the rule exists to protect.
// * The unsafe is confined to converting caller pointers into slices and to
//   the handle lifecycle. Both are written once and shared.
//
// Same shape as ALLOW-f32-WIRE-FORMAT and the browser build's marker. See
// CONTRIBUTING.md, "Unsafe is forbidden, except at an ABI boundary that says
// so".
#![deny(unsafe_op_in_unsafe_fn)]

//! The C ABI: the integration surface for a host application.
//!
//! # The refusal convention, which is the design decision everything follows
//!
//! This engine's distinguishing behaviour is that it **declines by name, with a
//! reason a person can act on**: `G41` hands the offset path to the control,
//! this file is Siemens rather than a dialect of RS-274, this resolution needs
//! 748 MiB and 0.035 mm would fit. The reason is the product. An ABI that
//! returned `-1` would throw away the only part of a refusal anybody wanted.
//!
//! So:
//!
//! * Every call returns [`CbStatus`]. **[`CbStatus::Refused`] is a success**:
//!   the engine did its job and the answer is "no, because".
//!   [`CbStatus::InvalidArgument`] means the *caller* made a mistake — a null
//!   pointer, a length that overflows — and is the only status that indicates a
//!   bug on the far side of the boundary.
//! * A run produces a [`CbResult`] handle **whether it succeeded or was
//!   refused**, and that handle always carries a JSON document. There is one
//!   document family: [`chipbreaker.verification-report`] for a run that
//!   happened, [`chipbreaker.refusal`] for one that did not, and both carry
//!   `schema`, `schema_version`, `verdict` and `verdict_rule` at the top level.
//!   A caller that only wants the gate reads `verdict.pass` from either without
//!   branching, and a refusal reads `false`.
//! * Ownership has **one** convention, used everywhere: this library allocates,
//!   the caller releases with the matching `_free`. Strings handed out by a
//!   getter are borrowed from the handle and stay valid until that handle is
//!   freed. There is no two-call length protocol anywhere, because mixing the
//!   two is how a caller ends up freeing something it does not own.
//! * **No `errno`, no thread-local state, no implicit initialisation.** Across
//!   a language edge, a value a caller has to remember to fetch before the next
//!   call is a value somebody will read stale.
//!
//! # Thread safety, precisely
//!
//! * Distinct handles may be used concurrently from different threads without
//!   external synchronisation.
//! * **A single handle may not.** Two threads calling `cb_job_set_*` on one
//!   `cb_job`, or one thread freeing a handle while another reads it, is
//!   undefined behaviour. Handles are `!Sync` by intent, not by accident.
//! * The functions that take no handle — [`cb_abi_version`],
//!   [`cb_engine_version`], [`cb_selftest_digest`], [`cb_selftest_passed`] —
//!   are safe to call from any thread at any time.
//! * The library holds no mutable global state. The self-test report is
//!   computed once behind a `OnceLock` and never changes afterwards.
//!
//! # Versioning
//!
//! [`cb_abi_version`] returns an integer a host can check at load time. See its
//! documentation for what a bump means; the policy is in `docs/versioning.md`.
//!
//! [`chipbreaker.verification-report`]: chipbreaker_core::findings::report::SCHEMA
//! [`chipbreaker.refusal`]: chipbreaker_core::findings::refusal::SCHEMA

use core::ffi::{c_char, c_int, c_void};
use std::sync::OnceLock;

mod job;
mod result;

// Re-exported at the root so the crate's own tests can call the ABI as
// ordinary Rust. An ABI whose tests all have to go through `dlopen` is an ABI
// that gets tested once, by hand, on the day it is written.
pub use job::*;
pub use result::*;

/// The outcome of a call.
///
/// `repr(i32)` because a C caller stores this in an `int`, and because a value
/// this library does not define must be readable as "something newer than me"
/// rather than as garbage.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CbStatus {
    /// The call did what was asked.
    Ok = 0,
    /// **A success.** The engine declined the job and said why.
    ///
    /// The result handle is populated, and its JSON is a
    /// `chipbreaker.refusal` document whose `message` is the sentence to show
    /// a user. Treating this as an error is the single most likely
    /// integration mistake, and it turns the engine's most useful behaviour
    /// into a crash dialog.
    Refused = 1,
    /// The caller passed something this library cannot use: a null pointer
    /// where one is required, a length that is not a valid size, text that is
    /// not UTF-8. **This is the only status that means a bug on the caller's
    /// side**, and no result handle is produced.
    InvalidArgument = 2,
    /// The engine failed in a way it does not have a sentence for.
    ///
    /// Distinct from [`Self::Refused`] on purpose: a refusal is a designed
    /// answer, and this is not. If this is ever returned, it is a defect worth
    /// reporting.
    Internal = 3,
}

/// The ABI version this library implements.
///
/// A host should call this at load time and refuse to continue if it does not
/// recognise the answer. Checking costs one call and turns a silent
/// misinterpretation of a struct layout into a clear message at startup.
///
/// **What a bump means.** This number changes only when an existing declaration
/// changes meaning or disappears: a function removed or renamed, a parameter
/// added or reordered, an enumerator's value changed, a struct's layout
/// altered. **Adding** a new function, or a new enumerator at the end of
/// [`CbStatus`], does not bump it — a caller compiled against an older header
/// keeps working, which is the whole point of the guarantee.
///
/// This is versioned independently of the crate version and of the report
/// schemas. They move for different reasons and tying them together would make
/// every release look like a break in all three.
#[unsafe(no_mangle)]
pub extern "C" fn cb_abi_version() -> u32 {
    1
}

/// The engine version, as a NUL-terminated string.
///
/// Static storage. **Never freed**, valid for the life of the process.
#[unsafe(no_mangle)]
pub extern "C" fn cb_engine_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr().cast()
}

/// The self-test digest, as a 64-character NUL-terminated hex string.
///
/// The same value on every target the engine builds for, including WebAssembly,
/// which is what makes it a statement about behaviour rather than about a
/// build. A host that records this beside a report can say later which
/// engine behaviour produced it.
///
/// Static storage after the first call. **Never freed.**
#[unsafe(no_mangle)]
pub extern "C" fn cb_selftest_digest() -> *const c_char {
    static DIGEST: OnceLock<std::ffi::CString> = OnceLock::new();
    DIGEST
        .get_or_init(|| std::ffi::CString::new(selftest().digest.to_hex()).unwrap_or_default())
        .as_ptr()
}

/// Whether every self-test suite passes in this build. `1` for yes.
///
/// The first call runs the suites, which takes on the order of a second and a
/// half; afterwards it is free. A host that wants that cost at a moment of its
/// choosing should call this during start-up rather than before the first job.
#[unsafe(no_mangle)]
pub extern "C" fn cb_selftest_passed() -> c_int {
    c_int::from(selftest().passed())
}

/// How many self-test cases ran.
///
/// Beside the digest so a differing digest can be told apart: a different count
/// means a different build, and the same count with a different digest means a
/// determinism problem, which is a far more serious thing.
#[unsafe(no_mangle)]
pub extern "C" fn cb_selftest_case_count() -> u32 {
    u32::try_from(selftest().suites.iter().map(|s| s.cases).sum::<usize>()).unwrap_or(u32::MAX)
}

fn selftest() -> &'static chipbreaker_core::selftest::SelfTestReport {
    static REPORT: OnceLock<chipbreaker_core::selftest::SelfTestReport> = OnceLock::new();
    REPORT
        .get_or_init(|| chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites()))
}

// ---------------------------------------------------------------------------
// Shared pointer handling
// ---------------------------------------------------------------------------

/// Borrows a caller's byte buffer.
///
/// Written once and used by every setter, so that "what counts as a valid
/// buffer" is decided in one place rather than eight.
///
/// A null pointer with a zero length is **not** an error: it is an empty
/// buffer, which is a thing a caller may legitimately have. A null pointer with
/// a non-zero length is a caller bug and says so.
///
/// # Safety
/// `ptr` must be valid for reads of `len` bytes, or null when `len` is zero.
unsafe fn borrow<'a>(ptr: *const u8, len: usize) -> Result<&'a [u8], CbStatus> {
    if len == 0 {
        return Ok(&[]);
    }
    if ptr.is_null() {
        return Err(CbStatus::InvalidArgument);
    }
    Ok(unsafe { core::slice::from_raw_parts(ptr, len) })
}

/// The same, for text that must be UTF-8.
///
/// # Safety
/// As [`borrow`].
unsafe fn borrow_str<'a>(ptr: *const c_char, len: usize) -> Result<&'a str, CbStatus> {
    let bytes = unsafe { borrow(ptr.cast::<u8>(), len)? };
    core::str::from_utf8(bytes).map_err(|_| CbStatus::InvalidArgument)
}

/// Casts an opaque handle back to its type, refusing null.
///
/// # Safety
/// `ptr` must be null or a handle this library produced and the caller has not
/// yet freed.
unsafe fn handle<'a, T>(ptr: *mut c_void) -> Option<&'a mut T> {
    if ptr.is_null() {
        return None;
    }
    Some(unsafe { &mut *ptr.cast::<T>() })
}

/// The same, for a handle that is only read.
///
/// # Safety
/// As [`handle`].
unsafe fn handle_ref<'a, T>(ptr: *const c_void) -> Option<&'a T> {
    if ptr.is_null() {
        return None;
    }
    Some(unsafe { &*ptr.cast::<T>() })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_abi_version_is_stated() {
        assert_eq!(cb_abi_version(), 1);
    }

    #[test]
    fn a_null_buffer_with_a_length_is_a_caller_bug_and_an_empty_one_is_not() {
        // The distinction matters: a host that legitimately has no nominal
        // passes (null, 0) and must not be told it made a mistake.
        assert_eq!(unsafe { borrow(core::ptr::null(), 0) }, Ok(&[][..]));
        assert_eq!(
            unsafe { borrow(core::ptr::null(), 4) },
            Err(CbStatus::InvalidArgument)
        );
    }

    #[test]
    fn refused_is_not_an_error_code() {
        // Written as a test because it is the thing an integrator gets wrong.
        // `Ok` and `Refused` are both outcomes the engine intends; only the two
        // above them mean something went wrong.
        assert_eq!(CbStatus::Ok as i32, 0);
        assert_eq!(CbStatus::Refused as i32, 1);
        assert!((CbStatus::InvalidArgument as i32) > (CbStatus::Refused as i32));
    }
}
