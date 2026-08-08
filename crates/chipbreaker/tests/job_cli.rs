// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! End-to-end tests for `chipbreaker job`.
//!
//! # What a job report has to get right that a single report does not
//!
//! Three things. **Which setup** a result came from, because two setups have two
//! programs with their own line numbering. **What the boundary between them
//! cost**, because that is the only place in the engine a transform can lose
//! anything. And **a verdict over the whole job**, computed strictly: a part is
//! not acceptable because two of its three setups were.
//!
//! The first setup is the control in every test here. If a fault in setup 1
//! showed up as a fault in setup 0 as well, the per-setup detail would be
//! decoration rather than information.

use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

fn binary() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_chipbreaker"))
}

fn run(args: &[&str]) -> (i32, String, String) {
    let out = Command::new(binary())
        .args(args)
        .output()
        .expect("the chipbreaker binary must be runnable");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

fn tool_library() -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/corpus/tool/standard-library.json")
        .to_str()
        .expect("utf-8")
        .to_owned()
        .replace('\\', "/")
}

fn write_box(path: &Path, lo: [f64; 3], hi: [f64; 3]) {
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
                // ALLOW-f32-WIRE-FORMAT
                #[allow(clippy::cast_possible_truncation, reason = "the format is 32-bit")]
                b.extend_from_slice(&(c as f32).to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
            }
        }
        b.extend_from_slice(&0u16.to_le_bytes());
    }
    std::fs::write(path, b).expect("writes");
}

struct Fixture {
    dir: PathBuf,
}

impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("chipbreaker-job-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("creates");
        let f = Self { dir };
        // Clear of the machine origin: a program starts with the tool at
        // (0, 0, 0), and a block covering that has the shank inside it before
        // the first line runs.
        write_box(&f.path("stock.stl"), [10.0, 10.0, 0.0], [70.0, 50.0, 30.0]);
        // Where the block lands after a quarter turn: x -50..-10, y 10..70.
        write_box(&f.path("clamp.stl"), [-8.0, 20.0, 0.0], [4.0, 36.0, 46.0]);
        let (code, _, err) = run(&[
            "dexel",
            "build",
            &f.s("stock.stl"),
            "--units",
            "mm",
            "--res",
            "0.8",
            "--axes",
            "xyz",
            "--out",
            &f.s("stock.tdx"),
        ]);
        assert_eq!(code, 0, "building the stock field failed: {err}");
        f.nc("op1.nc", "G0 X0. Y30.", 24.0, "G1 X80. F600.");
        f.nc("op2.nc", "G0 X-60. Y40.", 22.0, "G1 X0. F600.");
        f
    }

    fn path(&self, n: &str) -> PathBuf {
        self.dir.join(n)
    }

    fn s(&self, n: &str) -> String {
        self.path(n).to_str().expect("utf-8").to_owned()
    }

    fn nc(&self, name: &str, position: &str, z: f64, feed: &str) {
        std::fs::write(
            self.path(name),
            format!("G21 G90\nG0 Z60.\n{position}\nG0 Z{z:.3}\n{feed}\nG0 Z60.\nM30\n"),
        )
        .expect("writes");
    }

    /// Writes a job file and runs it. `tool1` is setup 1's cutter.
    fn job(&self, name: &str, tool1: &str, fixtures: bool) -> (i32, Value) {
        self.job_with(name, tool1, fixtures, None)
    }

    /// The same, optionally comparing against a nominal.
    fn job_with(
        &self,
        name: &str,
        tool1: &str,
        fixtures: bool,
        nominal: Option<&str>,
    ) -> (i32, Value) {
        let fx = if fixtures { "\"clamp.stl\"" } else { "" };
        let text = format!(
            r#"{{
  "schema": "chipbreaker.job",
  "version": 1,
  "stock": "stock.tdx",
  "units": "mm",
  "tolerance_mm": 0.2,
  "clearance_mm": 3.0,
  "setups": [
    {{ "name": "first-face", "program": "op1.nc", "tools": "{tools}",
       "tool": "long-reach-6", "fixtures": [] }},
    {{ "name": "turned", "transform": "rotate-z-90", "program": "op2.nc",
       "tools": "{tools}", "tool": "{tool1}", "fixtures": [{fx}] }}
  ]
}}
"#,
            tools = tool_library()
        );
        std::fs::write(self.path(name), text).expect("writes");
        let path = self.s(name);
        let mut args = vec!["job", "--setups", &path, "--json"];
        let n;
        if let Some(x) = nominal {
            n = self.s(x);
            args.push("--nominal");
            args.push(&n);
        }
        let (code, out, err) = run(&args);
        assert!(!out.trim().is_empty(), "the job produced no JSON: {err}");
        let top: Value = serde_json::from_str(&out).expect("valid JSON");
        (code, top["results"].clone())
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn a_two_setup_job_crosses_its_boundary_exactly() {
    let f = Fixture::new("clean");
    let (_, v) = f.job("job.json", "long-reach-6", true);

    let boundaries = v["boundaries"].as_array().expect("boundaries");
    assert_eq!(
        boundaries.len(),
        1,
        "two setups have one boundary between them"
    );
    assert_eq!(
        boundaries[0]["regime"], "exact",
        "a quarter turn must cross exactly rather than resample"
    );
    assert_eq!(boundaries[0]["bound_mm"], 0.0);
    assert_eq!(v["accumulated_transform_bound_mm"], 0.0);
    assert_eq!(
        v["verdict"]["gates"]["collision"]["state"], "pass",
        "no tool in this job can reach anything it should not"
    );

    // Both setups reported, in order, each named.
    let setups = v["setups"].as_array().expect("setups");
    assert_eq!(setups.len(), 2);
    assert_eq!(setups[0]["index"], 0);
    assert_eq!(setups[1]["index"], 1);
    assert_eq!(setups[1]["name"], "turned");
    assert!(
        setups.iter().all(|s| s["checked"] == Value::Bool(true)),
        "a job with holders on every tool must be checkable throughout"
    );
}

#[test]
fn a_fixture_is_checked_in_the_setup_it_belongs_to() {
    // The clamp exists only in setup 1, and the near misses must appear there
    // and nowhere else. If they showed up in setup 0 as well, the per-setup
    // attribution would be decoration.
    let f = Fixture::new("fixture");
    let (_, with) = f.job("job.json", "long-reach-6", true);
    let (_, without) = f.job("bare.json", "long-reach-6", false);

    let near = |v: &Value, i: usize| v["setups"][i]["near_misses"].as_u64().unwrap_or(0);
    assert_eq!(
        near(&with, 0),
        0,
        "setup 0 has no fixture and must report none"
    );
    assert!(
        near(&with, 1) > 0,
        "the clamp in setup 1 was not reported at a 3 mm clearance"
    );
    assert_eq!(
        near(&without, 1),
        0,
        "removing the clamp left the near misses behind, so they came from \
         something other than the fixture"
    );
}

#[test]
fn a_failure_in_the_second_setup_fails_the_whole_job() {
    // **The conjunction.** A part is not acceptable because one of its two
    // setups was, and the detail has to say which.
    let f = Fixture::new("fail");
    let (code, v) = f.job("job.json", "er32-stub-6", true);
    assert_eq!(code, 1, "a collision in setup 1 must fail the job");
    assert_eq!(v["verdict"]["pass"], Value::Bool(false));
    assert_eq!(v["verdict"]["gates"]["collision"]["state"], "fail");

    let setups = v["setups"].as_array().expect("setups");
    assert_eq!(
        setups[0]["collisions"], 0,
        "setup 0 is the control and must stay clean; if it did not, the \
         per-setup detail would be telling the reader nothing"
    );
    assert!(
        setups[1]["collisions"].as_u64().expect("a count") > 0,
        "the stub cutter in setup 1 must collide"
    );
}

#[test]
fn every_input_including_fixtures_is_content_addressed() {
    // Two clamps sharing a file stem would collide on identity if inputs were
    // named rather than hashed. A path is not an input.
    let f = Fixture::new("digests");
    let (_, v) = f.job("job.json", "long-reach-6", true);
    let inputs = v["inputs"].as_array().expect("inputs");
    assert!(
        inputs.len() >= 5,
        "expected stock, two programs, two tool libraries and a fixture, got {}",
        inputs.len()
    );
    for i in inputs {
        let d = i["digest"].as_str().expect("a digest");
        assert_eq!(d.len(), 64, "an input digest must be a full hash: {i}");
    }
    assert!(
        inputs
            .iter()
            .any(|i| i["role"].as_str().unwrap_or_default().contains("fixture")),
        "the fixture is missing from the manifest inputs"
    );
}

#[test]
fn an_arbitrary_rotation_is_refused_with_a_reason() {
    // The general resample is classified and bounded but not implemented, and
    // the refusal has to say so rather than falling back to something that
    // quietly claims a zero bound.
    let f = Fixture::new("oblique");
    let text = format!(
        r#"{{
  "schema": "chipbreaker.job",
  "version": 1,
  "stock": "stock.tdx",
  "units": "mm",
  "setups": [
    {{ "name": "a", "program": "op1.nc", "tools": "{t}", "tool": "long-reach-6" }},
    {{ "name": "b", "transform": [[0.8,-0.6,0,0],[0.6,0.8,0,0],[0,0,1,0],[0,0,0,1]],
       "program": "op2.nc", "tools": "{t}", "tool": "long-reach-6" }}
  ]
}}
"#,
        t = tool_library()
    );
    std::fs::write(f.path("oblique.json"), text).expect("writes");
    let (code, _, err) = run(&["job", "--setups", &f.s("oblique.json")]);
    assert_ne!(code, 0, "an unimplemented path must not report success");
    assert!(
        err.contains("not axis-aligned") && err.contains("mm"),
        "the refusal must say what it cannot do and what it would have cost: {err}"
    );
    assert!(
        err.contains("quarter turn") || err.contains("flip"),
        "the refusal should say what would work instead: {err}"
    );
}

#[test]
fn a_transform_on_the_first_setup_is_refused() {
    // There is no previous setup for it to move the stock from, so it would
    // suggest the stock arrives rotated -- which is not what the field says.
    let f = Fixture::new("firstmove");
    let text = format!(
        r#"{{
  "schema": "chipbreaker.job",
  "version": 1,
  "stock": "stock.tdx",
  "setups": [
    {{ "name": "a", "transform": "rotate-z-90", "program": "op1.nc",
       "tools": "{t}", "tool": "long-reach-6" }}
  ]
}}
"#,
        t = tool_library()
    );
    std::fs::write(f.path("bad.json"), text).expect("writes");
    let (code, _, err) = run(&["job", "--setups", &f.s("bad.json")]);
    assert_ne!(code, 0);
    assert!(
        err.contains("no previous setup"),
        "the refusal should explain why a transform there is meaningless: {err}"
    );
}

#[test]
fn a_file_that_is_not_a_job_is_refused() {
    let f = Fixture::new("notajob");
    std::fs::write(f.path("nope.json"), "{\"hello\": 1}\n").expect("writes");
    let (code, _, err) = run(&["job", "--setups", &f.s("nope.json")]);
    assert_ne!(code, 0);
    assert!(
        err.contains("not a Chipbreaker job file"),
        "the refusal should say what the file is not: {err}"
    );
}

#[test]
fn without_a_nominal_the_gouge_gate_is_unchecked_and_the_job_does_not_pass() {
    // **The same fail-safe rule the single-setup report follows.** A gate that
    // did not run has not passed, and a job verb that returned success while
    // comparing nothing would be certifying a part it never looked at.
    let f = Fixture::new("nonominal");
    let (code, v) = f.job("job.json", "long-reach-6", true);
    assert_eq!(v["verdict"]["gates"]["gouge"]["state"], "unchecked");
    assert_eq!(
        v["verdict"]["pass"],
        Value::Bool(false),
        "a job with an unchecked gate must not pass, however clean the rest is"
    );
    assert_eq!(code, 1, "the exit code must follow the whole verdict");
    let why = v["verdict"]["gates"]["gouge"]["why"]
        .as_str()
        .expect("an unchecked gate must say why");
    assert!(
        why.contains("nominal"),
        "the reason must name what is missing: {why}"
    );
    // And the collision gate, which *did* run, still reports its own answer.
    assert_eq!(v["verdict"]["gates"]["collision"]["state"], "pass");
}

#[test]
fn the_gouge_gate_runs_and_is_sensitive_to_the_nominal() {
    // Comparing the finished part against the **uncut block** must report the
    // slots as gouges: they are material missing relative to that nominal. A
    // deliberately wrong nominal, chosen because it makes the sensitivity
    // unmistakable -- a gate that passed here would be looking at nothing.
    //
    // The block has to be drawn where the finished stock *is*, which after a
    // quarter turn is x -50..-10, y 10..70. That is the whole point of the
    // frame check below.
    let f = Fixture::new("gouge");
    write_box(
        &f.path("turned-block.stl"),
        [-50.0, 10.0, 0.0],
        [-10.0, 70.0, 30.0],
    );
    let (code, v) = f.job_with("job.json", "long-reach-6", true, Some("turned-block.stl"));
    assert_eq!(
        v["verdict"]["gates"]["gouge"]["state"], "fail",
        "two slots against an uncut block must register as gouges: {v}"
    );
    assert_eq!(v["verdict"]["pass"], Value::Bool(false));
    assert_eq!(code, 1);
    let why = v["verdict"]["gates"]["gouge"]["why"]
        .as_str()
        .expect("a failing gate must say why");
    assert!(
        why.contains("gouge"),
        "the reason should name the finding kind and its depth: {why}"
    );
}

#[test]
fn a_nominal_in_the_wrong_frame_is_refused_rather_than_passed() {
    // **The rough edge this check exists for.** The stock is carried through
    // every setup transform; the nominal is not, because it is an input rather
    // than a result. A nominal drawn in the first setup's frame therefore does
    // not overlap the finished stock at all -- and the comparison used to
    // sample nothing and report a clean part, which is the worst answer
    // available.
    let f = Fixture::new("wrongframe");
    let text = format!(
        r#"{{
  "schema": "chipbreaker.job",
  "version": 1,
  "stock": "stock.tdx",
  "units": "mm",
  "setups": [
    {{ "name": "a", "program": "op1.nc", "tools": "{t}", "tool": "long-reach-6" }},
    {{ "name": "b", "transform": "rotate-z-90", "program": "op2.nc",
       "tools": "{t}", "tool": "long-reach-6" }}
  ]
}}
"#,
        t = tool_library()
    );
    std::fs::write(f.path("job.json"), text).expect("writes");
    // `stock.stl` is the first setup's frame, and the job ends in the second.
    let (code, out, err) = run(&[
        "job",
        "--setups",
        &f.s("job.json"),
        "--nominal",
        &f.s("stock.stl"),
    ]);
    assert_ne!(code, 0, "a nominal in the wrong frame must not pass: {out}");
    assert!(
        err.contains("does not overlap"),
        "the refusal must say the two do not line up: {err}"
    );
    assert!(
        err.contains("last setup"),
        "and must say which frame the nominal belongs in: {err}"
    );
}

#[test]
fn a_collision_names_the_setup_as_well_as_the_line() {
    // **Line 47 names two different moves in a two-setup job.** Each program
    // numbers its own lines from one, so a line number without a setup beside
    // it is ambiguous, and a machinist sent to the wrong file is worse off than
    // one sent nowhere.
    let f = Fixture::new("setupindex");
    let (_, v) = f.job("job.json", "er32-stub-6", true);
    let detail = v["setups"][1]["detail"].as_array().expect("detail");
    assert!(
        !detail.is_empty(),
        "the stub cutter in setup 1 must produce something to attribute"
    );
    for d in detail {
        assert_eq!(d["setup"], 1, "a contact found in setup 1 must say so: {d}");
        assert!(
            d["lines"].as_array().is_some_and(|l| !l.is_empty()),
            "and must still name a line: {d}"
        );
    }
}
