// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The C ABI and the shared assembly must produce the same document.
//!
//! # What this is actually guarding
//!
//! Three surfaces now answer the same question: the C ABI, the browser build,
//! and the Python bindings. They call one assembly, so they *should* be
//! incapable of disagreeing — but every one of them reads its inputs first, and
//! that is where they can differ without any of them touching the engine.
//!
//! That is not theoretical. The Python wrapper opened files in text mode, which
//! on Windows translates CRLF to LF, so the same program file produced a
//! different content digest in Python than in C. Every determinism test still
//! passed, because the engine was behaving identically; it was being handed
//! different bytes. The manifest is content-addressed precisely so that this
//! kind of difference is visible, and here it is being made visible on purpose.
//!
//! So the test compares the **whole document**, not a summary. A digest match
//! with a differing gate would be a worse outcome than a clean failure.

use std::ffi::c_void;

use chipbreaker::{CbStatus, cb_job_free, cb_job_new, cb_job_run};

/// A binary STL box, built here rather than read, so the test has no fixture
/// file whose line endings could become the very thing under test.
fn box_stl(lo: [f32; 3], hi: [f32; 3]) -> Vec<u8> {
    let v = [
        [lo[0], lo[1], lo[2]],
        [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]],
        [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]],
        [hi[0], lo[1], hi[2]],
        [hi[0], hi[1], hi[2]],
        [lo[0], hi[1], hi[2]],
    ];
    let t = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&u32::try_from(t.len()).expect("small").to_le_bytes());
    for tri in t {
        for _ in 0..3 {
            b.extend_from_slice(&0f32.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
        }
        for i in tri {
            for c in v[i] {
                b.extend_from_slice(&c.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
            }
        }
        b.extend_from_slice(&0u16.to_le_bytes());
    }
    b
}

fn tools() -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/corpus/tool/standard-library.json");
    std::fs::read_to_string(path).expect("the standard tool library must be readable")
}

const PROGRAM: &str =
    "G21 G90\nT11 M6\nG0 Z50.\nG0 X30. Y20.\nG1 Z-1. F200.\nG1 X50. F600.\nG0 Z50.\nM30\n";

/// Runs the job through the C ABI and returns its JSON.
fn through_the_c_abi(program: &str, stock: &[u8], tools: &str) -> serde_json::Value {
    let job = cb_job_new();
    assert!(!job.is_null());
    unsafe {
        assert_eq!(
            chipbreaker::cb_job_set_program(job, program.as_ptr().cast(), program.len()),
            CbStatus::Ok
        );
        assert_eq!(
            chipbreaker::cb_job_set_tools(job, tools.as_ptr().cast(), tools.len()),
            CbStatus::Ok
        );
        assert_eq!(
            chipbreaker::cb_job_set_stock_stl(job, stock.as_ptr(), stock.len()),
            CbStatus::Ok
        );
        assert_eq!(
            chipbreaker::cb_job_set_resolution_mm(job, 0.6),
            CbStatus::Ok
        );
        assert_eq!(chipbreaker::cb_job_set_tolerance_mm(job, 0.1), CbStatus::Ok);
    }

    let mut out: *mut c_void = core::ptr::null_mut();
    let status = unsafe { cb_job_run(job, &raw mut out) };
    unsafe { cb_job_free(job) };
    assert_eq!(status, CbStatus::Ok, "this program is not refused");
    assert!(!out.is_null());

    let mut len = 0usize;
    let ptr = unsafe { chipbreaker::cb_result_json(out, &raw mut len) };
    let text = unsafe { core::ffi::CStr::from_ptr(ptr) }
        .to_str()
        .expect("utf-8")
        .to_owned();
    assert_eq!(
        len,
        text.len(),
        "the reported length excludes the terminator"
    );
    unsafe { chipbreaker::cb_result_free(out) };
    serde_json::from_str(&text).expect("valid JSON")
}

/// The same job through the shared assembly, which is what Python calls.
fn through_the_pipeline(program: &str, stock: &[u8], tools: &str) -> serde_json::Value {
    chipbreaker_gcode::pipeline::run(&chipbreaker_gcode::pipeline::JobRequest {
        program,
        tools,
        stock_stl: stock,
        nominal_stl: None,
        tool_id: None,
        source: None,
        resolution_mm: 0.6,
        tolerance_mm: 0.1,
        clearance_mm: 0.0,
        memory_ceiling_bytes: None,
        segment_cap: None,
    })
    .expect("this program is not refused")
    .to_json()
}

#[test]
fn the_c_abi_and_the_shared_assembly_produce_the_same_document() {
    let stock = box_stl([0.0, 0.0, 0.0], [60.0, 40.0, 25.0]);
    let tools = tools();
    let a = through_the_c_abi(PROGRAM, &stock, &tools);
    let b = through_the_pipeline(PROGRAM, &stock, &tools);
    assert_eq!(
        serde_json::to_string(&a).expect("serialises"),
        serde_json::to_string(&b).expect("serialises"),
        "two surfaces over one assembly must not produce two documents"
    );
}

#[test]
fn the_same_program_with_windows_line_endings_hashes_differently() {
    // The mistake the Python wrapper made, pinned as a property of the engine
    // so nobody is surprised by it again: content addressing means CRLF and LF
    // are **different inputs**, and a binding that silently normalises one to
    // the other has changed the identity of its caller's file.
    //
    // This is correct behaviour, not a defect. What was a defect was a wrapper
    // doing the translation without knowing it.
    let stock = box_stl([0.0, 0.0, 0.0], [60.0, 40.0, 25.0]);
    let tools = tools();
    let crlf = PROGRAM.replace('\n', "\r\n");

    let a = through_the_pipeline(PROGRAM, &stock, &tools);
    let b = through_the_pipeline(&crlf, &stock, &tools);

    assert_ne!(
        a["manifest"]["digest"], b["manifest"]["digest"],
        "different bytes are a different input, and the manifest must say so"
    );
    // The geometry is identical all the same: the difference is in what was
    // hashed, not in what the engine did.
    assert_eq!(a["verdict"], b["verdict"]);
    assert_eq!(a["summary"], b["summary"]);
}

#[test]
fn a_refusal_reaches_a_c_caller_with_its_sentence() {
    let stock = box_stl([0.0, 0.0, 0.0], [60.0, 40.0, 25.0]);
    let tools = tools();
    let g41 = "G21 G90\nT1 M6\nG41 D1\nG0 X0. Y20.\nG1 X60. F600.\nM30\n";

    let job = cb_job_new();
    unsafe {
        chipbreaker::cb_job_set_program(job, g41.as_ptr().cast(), g41.len());
        chipbreaker::cb_job_set_tools(job, tools.as_ptr().cast(), tools.len());
        chipbreaker::cb_job_set_stock_stl(job, stock.as_ptr(), stock.len());
    }
    let mut out: *mut c_void = core::ptr::null_mut();
    let status = unsafe { cb_job_run(job, &raw mut out) };
    unsafe { cb_job_free(job) };

    // A success, and it produces a handle. A caller that treated this as an
    // error would discard the only part of the answer worth reading.
    assert_eq!(status, CbStatus::Refused);
    assert!(!out.is_null(), "a refusal still produces a result");
    assert_eq!(unsafe { chipbreaker::cb_result_refused(out) }, 1);
    assert_eq!(
        unsafe { chipbreaker::cb_result_pass(out) },
        0,
        "a refusal does not pass"
    );

    let msg = unsafe {
        core::ffi::CStr::from_ptr(chipbreaker::cb_result_message(out, std::ptr::null_mut()))
    }
    .to_str()
    .expect("utf-8");
    assert!(
        msg.contains("G41"),
        "the sentence names the construct: {msg}"
    );
    assert!(msg.contains("G40"), "and says what to do instead: {msg}");

    let text = unsafe {
        core::ffi::CStr::from_ptr(chipbreaker::cb_result_json(out, std::ptr::null_mut()))
    }
    .to_str()
    .expect("utf-8");
    let doc: serde_json::Value = serde_json::from_str(text).expect("valid JSON");
    assert_eq!(doc["schema"], "chipbreaker.refusal");
    // The contract that lets a consumer read the gate without branching.
    for key in ["schema", "schema_version", "verdict", "verdict_rule"] {
        assert!(doc.get(key).is_some(), "{key} must be present on a refusal");
    }
    assert_eq!(doc["verdict"]["pass"], false);

    unsafe { chipbreaker::cb_result_free(out) };
}
