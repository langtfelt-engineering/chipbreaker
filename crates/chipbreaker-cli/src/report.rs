// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Rendering of the self-test report.
//!
//! # The two sections, and why the split matters
//!
//! The JSON report has exactly two top-level sections:
//!
//! - **`results`** — everything deterministic. Carries its own canonical hash,
//!   and that hash is what the CI parity job compares between the native run and
//!   the `wasmtime` run.
//! - **`environment`** — target triple, toolchain, crate version, host CPU, and
//!   **timings**. Excluded from the hash.
//!
//! The exclusion is not cosmetic. A duration in a hashed structure makes every
//! run differ from every other run, and the resulting CI failure looks exactly
//! like a determinism bug — which is a day of somebody's life to diagnose the
//! first time and an eroded trust in the harness thereafter.
//!
//! Keys are sorted: `serde_json::Map` is a `BTreeMap` unless the `preserve_order`
//! feature is enabled, which it is not.

use std::time::Duration;

use chipbreaker_core::selftest::{SelfTestReport, SuiteResult};
use serde_json::{Map, Value, json};

/// Facts about where and how this binary was built and run. Never hashed.
pub struct Environment {
    /// Target triple, from the build script.
    pub target: &'static str,
    /// `rustc --version` at build time.
    pub rustc: &'static str,
    /// Version of `chipbreaker-core`.
    pub crate_version: &'static str,
    /// Architecture, as `std::env::consts::ARCH`.
    pub arch: &'static str,
    /// Operating system, as `std::env::consts::OS`.
    pub os: &'static str,
    /// Available parallelism, or `None` where the platform cannot report it.
    pub cpu_threads: Option<usize>,
    /// Wall-clock time for the whole suite.
    pub elapsed: Duration,
}

impl Environment {
    /// Collects the environment. `elapsed` is supplied by the caller because
    /// only it knows what was being timed.
    #[must_use]
    pub fn collect(elapsed: Duration) -> Self {
        Self {
            target: env!("CHIPBREAKER_TARGET"),
            rustc: env!("CHIPBREAKER_RUSTC"),
            crate_version: chipbreaker_core::VERSION,
            arch: std::env::consts::ARCH,
            os: std::env::consts::OS,
            // Fails under some WASM runtimes; that is a missing value, not an
            // error, and it must not abort a self-test.
            cpu_threads: std::thread::available_parallelism().ok().map(Into::into),
            elapsed,
        }
    }

    fn to_json(&self) -> Value {
        json!({
            "arch": self.arch,
            "core_version": self.crate_version,
            "cpu_threads": self.cpu_threads,
            "elapsed_ms": duration_ms(self.elapsed),
            "note": "This section is excluded from results.hash. Timings and host \
                     details differ between every run and every platform by design.",
            "os": self.os,
            "rustc": self.rustc,
            "target": self.target,
        })
    }
}

/// Milliseconds, rounded to three decimals so the JSON stays readable.
///
/// Never feeds a hash, so lossy formatting is fine here — and only here.
fn duration_ms(d: Duration) -> f64 {
    (d.as_secs_f64() * 1e6).round() / 1e3
}

fn suite_json(suite: &SuiteResult) -> Value {
    let failures: Vec<Value> = suite
        .failures
        .iter()
        .map(|f| json!({ "case": f.case, "detail": f.detail }))
        .collect();
    json!({
        "cases": suite.cases,
        "description": suite.description,
        "failures": failures,
        "hash": suite.digest.to_hex(),
        "name": suite.name,
        "passed": suite.passed(),
    })
}

/// The deterministic half of the report.
///
/// `hash` is the canonical digest of the underlying [`SelfTestReport`] — computed
/// from its binary encoding, not from this JSON. It is reported here rather than
/// recomputed from the text precisely so that JSON formatting can never
/// influence it.
fn results_json(report: &SelfTestReport) -> Value {
    let suites: Vec<Value> = report.suites.iter().map(suite_json).collect();
    json!({
        "encoding_version": chipbreaker_core::CANONICAL_ENCODING_VERSION,
        "hash": report.digest.to_hex(),
        "passed": report.passed(),
        "suites": suites,
        "total_cases": report.total_cases(),
        "total_failures": report.total_failures(),
    })
}

/// Renders the full JSON report, keys sorted, with a trailing newline.
#[must_use]
pub fn to_json(report: &SelfTestReport, env: &Environment) -> String {
    let mut root = Map::new();
    root.insert("schema".to_owned(), json!(SCHEMA));
    root.insert("results".to_owned(), results_json(report));
    root.insert("environment".to_owned(), env.to_json());
    let mut out = serde_json::to_string_pretty(&Value::Object(root))
        .unwrap_or_else(|e| unreachable!("report JSON is always serializable: {e}"));
    out.push('\n');
    out
}

/// Identifies the report format, so a consumer can detect a breaking change.
pub const SCHEMA: &str = "chipbreaker.selftest/1";

/// Renders the human-readable report.
#[must_use]
pub fn to_text(report: &SelfTestReport, env: &Environment) -> String {
    let mut out = String::new();
    out.push_str("chipbreaker selftest\n");
    out.push_str("====================\n\n");

    let name_width = report
        .suites
        .iter()
        .map(|s| s.name.len())
        .max()
        .unwrap_or(0)
        .max(5);

    for suite in &report.suites {
        out.push_str(&format!(
            "{:<name_width$}  {:>7} cases  {}  {}\n",
            suite.name,
            suite.cases,
            if suite.passed() { "ok  " } else { "FAIL" },
            &suite.digest.to_hex()[..16],
        ));
        out.push_str(&format!("{:<name_width$}  {}\n", "", suite.description));
        for failure in &suite.failures {
            out.push_str(&format!(
                "{:<name_width$}    ! {}: {}\n",
                "", failure.case, failure.detail
            ));
        }
    }

    out.push_str(&format!(
        "\n{} suites, {} cases, {} failures\n",
        report.suites.len(),
        report.total_cases(),
        report.total_failures()
    ));
    out.push_str(&format!("results hash: {}\n", report.digest));
    out.push_str(&format!(
        "\nenvironment (not hashed): {} / {} / {:.3} ms\n",
        env.target,
        env.rustc,
        duration_ms(env.elapsed)
    ));
    out.push_str(if report.passed() {
        "\nPASS\n"
    } else {
        "\nFAIL\n"
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> (SelfTestReport, Environment) {
        (
            chipbreaker_core::selftest::run(),
            Environment::collect(Duration::from_millis(7)),
        )
    }

    #[test]
    fn json_has_exactly_two_sections_plus_a_schema() {
        let (report, env) = sample();
        let text = to_json(&report, &env);
        let value: Value = serde_json::from_str(&text).expect("valid JSON");
        let obj = value.as_object().expect("object");
        let mut keys: Vec<&str> = obj.keys().map(String::as_str).collect();
        keys.sort_unstable();
        assert_eq!(keys, ["environment", "results", "schema"]);
        assert_eq!(obj["schema"], json!(SCHEMA));
    }

    #[test]
    fn the_results_section_carries_no_timing_or_host_detail() {
        // The whole native/WASM parity check rests on this.
        let (report, env) = sample();
        let value: Value = serde_json::from_str(&to_json(&report, &env)).expect("valid JSON");
        let results = serde_json::to_string(&value["results"]).expect("serializable");
        for forbidden in [
            "elapsed", "ms", "time", "target", "rustc", "arch", "cpu", "os",
        ] {
            assert!(
                !results.contains(forbidden),
                "the hashed `results` section mentions `{forbidden}`: {results}"
            );
        }
        // And the environment section really does carry them.
        let environment = serde_json::to_string(&value["environment"]).expect("serializable");
        assert!(environment.contains("elapsed_ms"));
        assert!(environment.contains("target"));
    }

    #[test]
    fn json_keys_are_sorted() {
        let (report, env) = sample();
        let text = to_json(&report, &env);
        // `schema` sorts after `results` and `environment`; if serde_json ever
        // gained `preserve_order` through a feature unification, insertion order
        // would put `schema` first and this would catch it.
        let env_at = text.find("\"environment\"").expect("present");
        let results_at = text.find("\"results\"").expect("present");
        let schema_at = text.find("\"schema\"").expect("present");
        assert!(
            env_at < results_at && results_at < schema_at,
            "keys not sorted"
        );
    }

    #[test]
    fn the_results_section_is_identical_across_runs() {
        let a = to_json(
            &chipbreaker_core::selftest::run(),
            &Environment::collect(Duration::from_millis(1)),
        );
        let b = to_json(
            &chipbreaker_core::selftest::run(),
            &Environment::collect(Duration::from_secs(999)),
        );
        let va: Value = serde_json::from_str(&a).expect("valid JSON");
        let vb: Value = serde_json::from_str(&b).expect("valid JSON");
        assert_eq!(va["results"], vb["results"], "results must not vary");
        assert_ne!(
            va["environment"]["elapsed_ms"], vb["environment"]["elapsed_ms"],
            "the environment section is supposed to vary; otherwise this test proves nothing"
        );
    }

    #[test]
    fn text_report_names_every_suite_and_the_hash() {
        let (report, env) = sample();
        let text = to_text(&report, &env);
        for suite in &report.suites {
            assert!(
                text.contains(suite.name),
                "text report omits {}",
                suite.name
            );
        }
        assert!(text.contains(&report.digest.to_hex()));
        assert!(text.ends_with("PASS\n"));
    }

    #[test]
    fn duration_rendering_is_stable_and_rounded() {
        assert!((duration_ms(Duration::from_millis(1500)) - 1500.0).abs() < 1e-9);
        assert!((duration_ms(Duration::from_micros(1)) - 0.001).abs() < 1e-9);
        assert_eq!(duration_ms(Duration::ZERO), 0.0);
    }
}
