/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Langtfelt
 *
 * GENERATED FILE -- do not edit by hand.
 *
 * Regenerate with:
 *     cbindgen --config crates/chipbreaker-abi/cbindgen.toml \
 *              --crate chipbreaker-abi --output include/chipbreaker.h
 *
 * CI fails if this file differs from what the Rust source generates, so an
 * edit here is reverted rather than kept.
 *
 * ---------------------------------------------------------------------------
 * READ THIS FIRST: CB_REFUSED IS NOT AN ERROR
 * ---------------------------------------------------------------------------
 *
 * This engine declines jobs it cannot answer for, by name and with a reason
 * written for a person to read. A refusal means the engine did its job. Treat
 * CB_REFUSED as a result to show the user, never as a failure to log and
 * retry -- the sentence in cb_result_message() is the most valuable thing this
 * library produces.
 *
 * Ownership, one rule everywhere: this library allocates, you release with the
 * matching _free. Strings returned by getters are borrowed from their handle
 * and are invalid once that handle is freed.
 *
 * Threads: distinct handles may be used concurrently; a single handle may not.
 */

#ifndef CHIPBREAKER_H
#define CHIPBREAKER_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * The outcome of a call.
 *
 * `repr(i32)` because a C caller stores this in an `int`, and because a value
 * this library does not define must be readable as "something newer than me"
 * rather than as garbage.
 */
enum cb_status
#if defined(__cplusplus) || __STDC_VERSION__ >= 202311L
  : int32_t
#endif // defined(__cplusplus) || __STDC_VERSION__ >= 202311L
 {
    /**
     * The call did what was asked.
     */
    CB_STATUS_OK = 0,
    /**
     * **A success.** The engine declined the job and said why.
     *
     * The result handle is populated, and its JSON is a
     * `chipbreaker.refusal` document whose `message` is the sentence to show
     * a user. Treating this as an error is the single most likely
     * integration mistake, and it turns the engine's most useful behaviour
     * into a crash dialog.
     */
    CB_STATUS_REFUSED = 1,
    /**
     * The caller passed something this library cannot use: a null pointer
     * where one is required, a length that is not a valid size, text that is
     * not UTF-8. **This is the only status that means a bug on the caller's
     * side**, and no result handle is produced.
     */
    CB_STATUS_INVALID_ARGUMENT = 2,
    /**
     * The engine failed in a way it does not have a sentence for.
     *
     * Distinct from [`Self::Refused`] on purpose: a refusal is a designed
     * answer, and this is not. If this is ever returned, it is a defect worth
     * reporting.
     */
    CB_STATUS_INTERNAL = 3,
};
#ifndef __cplusplus
#if __STDC_VERSION__ >= 202311L
typedef enum cb_status cb_status;
#else
typedef int32_t cb_status;
#endif // __STDC_VERSION__ >= 202311L
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * The ABI version this library implements.
 *
 * A host should call this at load time and refuse to continue if it does not
 * recognise the answer. Checking costs one call and turns a silent
 * misinterpretation of a struct layout into a clear message at startup.
 *
 * **What a bump means.** This number changes only when an existing declaration
 * changes meaning or disappears: a function removed or renamed, a parameter
 * added or reordered, an enumerator's value changed, a struct's layout
 * altered. **Adding** a new function, or a new enumerator at the end of
 * [`CbStatus`], does not bump it — a caller compiled against an older header
 * keeps working, which is the whole point of the guarantee.
 *
 * This is versioned independently of the crate version and of the report
 * schemas. They move for different reasons and tying them together would make
 * every release look like a break in all three.
 */
uint32_t cb_abi_version(void);

/**
 * The engine version, as a NUL-terminated string.
 *
 * Static storage. **Never freed**, valid for the life of the process.
 */
const char *cb_engine_version(void);

/**
 * The self-test digest, as a 64-character NUL-terminated hex string.
 *
 * The same value on every target the engine builds for, including WebAssembly,
 * which is what makes it a statement about behaviour rather than about a
 * build. A host that records this beside a report can say later which
 * engine behaviour produced it.
 *
 * Static storage after the first call. **Never freed.**
 */
const char *cb_selftest_digest(void);

/**
 * Whether every self-test suite passes in this build. `1` for yes.
 *
 * The first call runs the suites, which takes on the order of a second and a
 * half; afterwards it is free. A host that wants that cost at a moment of its
 * choosing should call this during start-up rather than before the first job.
 */
int cb_selftest_passed(void);

/**
 * How many self-test cases ran.
 *
 * Beside the digest so a differing digest can be told apart: a different count
 * means a different build, and the same count with a different digest means a
 * determinism problem, which is a far more serious thing.
 */
uint32_t cb_selftest_case_count(void);

/**
 * Creates an empty job.
 *
 * Returns null only if allocation fails, which on a host large enough to run
 * this engine means the process is already in trouble.
 *
 * The caller owns the result and must release it with [`cb_job_free`].
 */
void *cb_job_new(void);

/**
 * Releases a job.
 *
 * Passing null is allowed and does nothing, so a cleanup path need not test
 * first. Passing the same handle twice is undefined behaviour, as it is for
 * `free`.
 *
 * # Safety
 * `job` must be null, or a handle from [`cb_job_new`] that has not been freed.
 */
void cb_job_free(void *job);

/**
 * Sets the NC program, as UTF-8 text.
 *
 * Copied. The caller may free its buffer immediately.
 *
 * # Safety
 * `text` must be valid for reads of `len` bytes, or null when `len` is zero.
 */
cb_status cb_job_set_program(void *job, const char *text, size_t len);

/**
 * Sets the tool library, as the JSON this engine's tool format uses.
 *
 * # Safety
 * As [`cb_job_set_program`].
 */
cb_status cb_job_set_tools(void *job, const char *text, size_t len);

/**
 * Sets the stock mesh, as STL bytes. Binary or ASCII; the engine tells them
 * apart, because which one a customer's CAM system wrote is not a question
 * worth asking a caller.
 *
 * # Safety
 * `bytes` must be valid for reads of `len` bytes, or null when `len` is zero.
 */
cb_status cb_job_set_stock_stl(void *job, const uint8_t *bytes, size_t len);

/**
 * Sets the nominal part, as STL bytes.
 *
 * **Optional, and its absence is not a defect.** Without it the gouge gate
 * reports `unchecked` rather than passing, because a gate that did not run has
 * certified nothing. A host doing collision checking alone can leave it unset
 * and read a report that says exactly that.
 *
 * # Safety
 * As [`cb_job_set_stock_stl`].
 */
cb_status cb_job_set_nominal_stl(void *job, const uint8_t *bytes, size_t len);

/**
 * Selects a tool from the library by its `id`.
 *
 * When unset, the tool named by the program's first `T` word is used.
 *
 * # Safety
 * As [`cb_job_set_program`].
 */
cb_status cb_job_set_tool_id(void *job, const char *text, size_t len);

/**
 * Records where these inputs came from, for a human reading the manifest.
 *
 * **Not part of the report's identity.** The manifest hashes input *content*,
 * never paths, so that two runs of the same bytes from different directories
 * are recognisably the same run. This string is a courtesy to a reader six
 * months later, and nothing depends on it.
 *
 * # Safety
 * As [`cb_job_set_program`].
 */
cb_status cb_job_set_source(void *job, const char *text, size_t len);

/**
 * Sets the dexel spacing in millimetres. Must be finite and positive.
 *
 * # Safety
 * `job` must be a live handle from [`cb_job_new`].
 */
cb_status cb_job_set_resolution_mm(void *job, double mm);

/**
 * Sets the tolerance findings are judged against, in millimetres.
 *
 * # Safety
 * As [`cb_job_set_resolution_mm`].
 */
cb_status cb_job_set_tolerance_mm(void *job, double mm);

/**
 * Sets the clearance below which a pass is reported as a near miss.
 *
 * Zero is legitimate and means "report contact only". Negative is not.
 *
 * # Safety
 * As [`cb_job_set_resolution_mm`].
 */
cb_status cb_job_set_clearance_mm(void *job, double mm);

/**
 * Caps how much memory a run may need, in bytes.
 *
 * The check happens **before anything is allocated**, and exceeding it is a
 * refusal that names a resolution which would fit — not an allocation failure.
 * A host embedding this engine in a long-lived process should set it; the
 * alternative is discovering the limit through the operating system.
 *
 * Zero means no ceiling.
 *
 * # Safety
 * As [`cb_job_set_resolution_mm`].
 */
cb_status cb_job_set_memory_ceiling_bytes(void *job, uint64_t bytes);

/**
 * Runs the job.
 *
 * **Returns [`CbStatus::Refused`] as a success.** On both `Ok` and `Refused`,
 * `*out` receives a result handle carrying a JSON document; the caller owns it
 * and must release it with [`cb_result_free`]. On `InvalidArgument` and
 * `Internal`, `*out` is set to null and there is nothing to free.
 *
 * The job handle is not consumed and may be run again, with or without
 * changing settings first. Re-running produces an independent result handle.
 *
 * # Safety
 * `job` must be a live handle from [`cb_job_new`], and `out` must be a valid
 * pointer to a single writeable pointer.
 *
 * [`cb_result_free`]: crate::result::cb_result_free
 */
cb_status cb_job_run(void *job, void **out);

/**
 * The result document, as NUL-terminated UTF-8 JSON.
 *
 * When `len` is not null it receives the length in bytes, **excluding** the
 * terminating NUL.
 *
 * **Borrowed.** The pointer is valid until [`cb_result_free`] is called on
 * this handle, and must not be freed by the caller. Copy it if it needs to
 * outlive the handle.
 *
 * Returns null if `result` is null.
 *
 * # Safety
 * `result` must be null or a live handle from `cb_job_run`, and `len` must be
 * null or a valid pointer to a single writeable `size_t`.
 */
const char *cb_result_json(const void *result, size_t *len);

/**
 * Whether the verdict passed: `1` for yes, `0` for no.
 *
 * **A conjunction over every gate, and an unchecked gate does not pass.** A
 * job with no nominal does not pass the gouge gate by default — it fails to
 * have been checked, which is a different and more useful thing to be told.
 *
 * Returns `0` for a null handle, which is the safe direction: a caller that
 * mishandles the pointer is told the job did not pass rather than that it did.
 *
 * # Safety
 * `result` must be null or a live handle from `cb_job_run`.
 */
int cb_result_pass(const void *result);

/**
 * Whether this result is a refusal: `1` for yes.
 *
 * Equivalent to having received [`CbStatus::Refused`] from the run, and
 * available here so a result can be interpreted after being passed around
 * without its status.
 *
 * # Safety
 * As [`cb_result_pass`].
 */
int cb_result_refused(const void *result);

/**
 * The refusal sentence, as NUL-terminated UTF-8.
 *
 * Empty — a valid pointer to a single NUL, never null — for a run that
 * happened. **This is the string to show a user.** It names what was declined
 * and usually what to do instead: which resolution would fit, which dialect
 * the file is written in, why the control rather than the program decides an
 * offset path.
 *
 * **Borrowed**, on the same terms as [`cb_result_json`].
 *
 * # Safety
 * As [`cb_result_json`].
 */
const char *cb_result_message(const void *result, size_t *len);

/**
 * Releases a result.
 *
 * Passing null is allowed and does nothing. Every pointer previously returned
 * by [`cb_result_json`] or [`cb_result_message`] for this handle is invalid
 * afterwards.
 *
 * # Safety
 * `result` must be null, or a handle from `cb_job_run` that has not been
 * freed.
 */
void cb_result_free(void *result);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* CHIPBREAKER_H */
