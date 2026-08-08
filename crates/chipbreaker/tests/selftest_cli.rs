// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! End-to-end tests that run the real `chipbreaker` binary.
//!
//! The unit-level tests in `report.rs` check the rendering in-process. These
//! check the thing CI actually invokes: a separate process, its exit code, and
//! its stdout. The stability test in particular is the local stand-in for the
//! native/WASM parity job — same binary, two processes, identical `results`.

use std::path::PathBuf;
use std::process::Command;

use serde_json::Value;

fn binary() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_chipbreaker"))
}

fn run(args: &[&str]) -> (i32, String, String) {
    let output = Command::new(binary())
        .args(args)
        .output()
        .expect("the chipbreaker binary must be runnable");
    (
        output.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&output.stdout).into_owned(),
        String::from_utf8_lossy(&output.stderr).into_owned(),
    )
}

#[test]
fn selftest_exits_zero_and_reports_pass() {
    let (code, stdout, stderr) = run(&["selftest"]);
    assert_eq!(code, 0, "selftest failed:\n{stdout}\n{stderr}");
    assert!(stdout.contains("PASS"), "{stdout}");
    assert!(stdout.contains("results hash:"), "{stdout}");
    assert!(
        stderr.is_empty(),
        "nothing should reach stderr on success: {stderr}"
    );
}

#[test]
fn selftest_json_is_well_formed_and_complete() {
    let (code, stdout, _) = run(&["selftest", "--report", "json"]);
    assert_eq!(code, 0);
    let value: Value = serde_json::from_str(&stdout).expect("stdout must be valid JSON");

    let results = &value["results"];
    assert_eq!(results["passed"], Value::Bool(true));
    assert_eq!(results["total_failures"], Value::from(0));
    assert!(results["total_cases"].as_u64().expect("a number") > 9_000);
    let hash = results["hash"].as_str().expect("a hash string");
    assert_eq!(hash.len(), 64, "expected a 256-bit digest, got `{hash}`");
    assert!(
        hash.chars()
            .all(|c| c.is_ascii_hexdigit() && !c.is_uppercase())
    );

    let suites = results["suites"].as_array().expect("an array");
    assert!(
        suites.len() >= 5,
        "expected every core suite, got {}",
        suites.len()
    );
    for suite in suites {
        assert_eq!(suite["passed"], Value::Bool(true), "{suite}");
        assert!(!suite["name"].as_str().expect("a name").is_empty());
        assert_eq!(suite["hash"].as_str().expect("a hash").len(), 64);
    }

    // The environment section exists, and is where the volatile facts live.
    let environment = &value["environment"];
    assert!(environment["target"].is_string());
    assert!(environment["rustc"].is_string());
    assert!(environment["elapsed_ms"].is_number());
}

#[test]
fn the_results_hash_is_identical_across_two_processes() {
    // This is the property the whole determinism story rests on, checked the
    // hard way: two separate operating-system processes.
    let (code_a, a, _) = run(&["selftest", "--report", "json"]);
    let (code_b, b, _) = run(&["selftest", "--report", "json"]);
    assert_eq!(code_a, 0);
    assert_eq!(code_b, 0);

    let va: Value = serde_json::from_str(&a).expect("valid JSON");
    let vb: Value = serde_json::from_str(&b).expect("valid JSON");

    assert_eq!(
        va["results"], vb["results"],
        "the entire results section must be identical between runs"
    );
    assert_eq!(va["results"]["hash"], vb["results"]["hash"]);

    // If the environment section were also identical, this test would prove
    // nothing about the split — it would be consistent with hashing everything.
    // Timings are the field guaranteed to differ.
    assert!(
        va["environment"]["elapsed_ms"].is_number() && vb["environment"]["elapsed_ms"].is_number(),
        "timings must be reported, just not hashed"
    );
}

#[test]
fn out_writes_the_report_to_a_file() {
    let dir = std::env::temp_dir().join(format!("chipbreaker-cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("report.json");

    let (code, stdout, _) = run(&[
        "selftest",
        "--report",
        "json",
        "--out",
        path.to_str().expect("utf-8 path"),
    ]);
    assert_eq!(code, 0);
    assert!(stdout.contains("wrote"), "{stdout}");
    assert!(
        stdout.contains("results hash:"),
        "the log must not be silent"
    );

    let written = std::fs::read_to_string(&path).expect("the file must exist");
    let value: Value = serde_json::from_str(&written).expect("valid JSON on disk");
    assert_eq!(value["results"]["passed"], Value::Bool(true));
    // The hash on stdout must be the hash in the file.
    let hash = value["results"]["hash"].as_str().expect("a hash");
    assert!(stdout.contains(hash), "stdout hash disagrees with the file");

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn writing_to_an_impossible_path_fails_cleanly() {
    let (code, _, stderr) = run(&[
        "selftest",
        "--out",
        "no-such-directory-exists-here/report.txt",
    ]);
    assert_eq!(code, 1, "a write failure must be a failure exit");
    assert!(stderr.contains("could not write"), "{stderr}");
}

#[test]
fn version_reports_both_forms() {
    let (code, stdout, _) = run(&["version"]);
    assert_eq!(code, 0);
    assert!(stdout.contains(env!("CARGO_PKG_VERSION")), "{stdout}");

    let (code, stdout, _) = run(&["version", "--json"]);
    assert_eq!(code, 0);
    let value: Value = serde_json::from_str(&stdout).expect("valid JSON");
    assert_eq!(value["version"], Value::from(env!("CARGO_PKG_VERSION")));
    assert_eq!(value["name"], Value::from("chipbreaker"));
    assert!(value["target"].is_string());
    assert!(value["encoding_version"].is_number());
}

#[test]
fn a_missing_subcommand_is_an_error() {
    let (code, _, stderr) = run(&[]);
    assert_ne!(code, 0);
    assert!(stderr.contains("Usage"), "{stderr}");
}
