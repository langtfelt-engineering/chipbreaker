// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! End-to-end tests for `chipbreaker collide`.
//!
//! The exit code is the product here. A CI job wires this command to a gate and
//! never reads the JSON, so the code has to carry the whole answer — including
//! the case where the answer is "I could not look", which must not be zero.

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
        let dir = std::env::temp_dir().join(format!("chipbreaker-collide-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("creates");
        let f = Self { dir };
        // The block sits away from the machine origin on purpose. A program
        // begins with the tool at (0, 0, 0), and a block with a corner there
        // would have the shank inside it before the first line runs -- a real
        // collision, but one that says more about the fixture than the checker.
        write_box(&f.path("stock.stl"), [20.0, 20.0, 0.0], [95.0, 70.0, 40.0]);
        // Tall enough to reach the height the chuck actually sweeps at. A clamp
        // stopping at the height of the stock sits entirely under the nut.
        write_box(&f.path("clamp.stl"), [98.0, 38.0, 0.0], [112.0, 52.0, 80.0]);
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
        f
    }

    fn path(&self, n: &str) -> PathBuf {
        self.dir.join(n)
    }

    fn s(&self, n: &str) -> String {
        self.path(n).to_str().expect("utf-8").to_owned()
    }

    /// A pocket down to `z`.
    fn program(&self, name: &str, z: f64) -> String {
        std::fs::write(
            self.path(&format!("{name}.nc")),
            format!("G21 G90\nG0 Z70.\nG0 X35. Y45.\nG0 Z{z:.3}\nG1 X80. F500.\nG0 Z70.\nM30\n"),
        )
        .expect("writes");
        self.s(&format!("{name}.nc"))
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn a_buried_chuck_fails_and_names_the_line() {
    let f = Fixture::new("buried");
    let prog = f.program("deep", 6.0);
    let (code, out, err) = run(&[
        "collide",
        &f.s("stock.tdx"),
        "--path",
        &prog,
        "--tools",
        &tool_library(),
        "--tool",
        "er32-stub-6",
        "--json",
    ]);
    assert_eq!(code, 1, "a buried chuck must fail: {err}");
    // The shared emitter nests the payload under `results` and puts the host
    // and timing beside it, unhashed.
    let top: Value = serde_json::from_str(&out).expect("valid JSON");
    let v = &top["results"];
    assert_eq!(v["checked"], Value::Bool(true));
    assert!(
        v["summary"]["collisions"].as_u64().expect("a count") > 0,
        "no collisions reported for a 34 mm pocket cut with a 10 mm flute"
    );
    // Naming the line is the point. "Something collided" is not actionable.
    let first = &v["collisions"][0];
    assert!(
        first["attribution"]["lines"][0].as_u64().is_some(),
        "a collision must name the NC line that caused it: {first}"
    );
    assert_eq!(v["rapid_path"], "linear", "the policy must be stated");
}

#[test]
fn the_same_pocket_with_enough_reach_passes() {
    // The mutation check: the failure above must come from the reach of the
    // tool and not from the program, which is fine with a cutter that clears.
    let f = Fixture::new("reach");
    let prog = f.program("shallow", 36.0);
    let (code, out, err) = run(&[
        "collide",
        &f.s("stock.tdx"),
        "--path",
        &prog,
        "--tools",
        &tool_library(),
        "--tool",
        "long-reach-6",
        "--json",
    ]);
    assert_eq!(
        code, 0,
        "a shallow pocket with a long tool must pass: {err}"
    );
    // The shared emitter nests the payload under `results` and puts the host
    // and timing beside it, unhashed.
    let top: Value = serde_json::from_str(&out).expect("valid JSON");
    let v = &top["results"];
    assert_eq!(v["summary"]["collisions"], 0);
    assert_eq!(v["checked"], Value::Bool(true));
}

#[test]
fn a_tool_without_a_holder_exits_non_zero() {
    // **The most important exit code in this file.** A CI job reads the code and
    // nothing else, so "I could not check" must not look like "nothing found".
    let f = Fixture::new("noholder");
    let prog = f.program("shallow", 36.0);
    let (code, out, err) = run(&[
        "collide",
        &f.s("stock.tdx"),
        "--path",
        &prog,
        "--tools",
        &tool_library(),
        "--tool",
        "flat-6",
        "--json",
    ]);
    assert_eq!(
        code, 1,
        "an unchecked result must not exit zero: {out} {err}"
    );
    // The shared emitter nests the payload under `results` and puts the host
    // and timing beside it, unhashed.
    let top: Value = serde_json::from_str(&out).expect("valid JSON");
    let v = &top["results"];
    assert_eq!(v["checked"], Value::Bool(false));
    let why = v["unchecked_because"].as_str().expect("a reason");
    assert!(
        why.contains("holder"),
        "the reason must name what is missing: {why}"
    );
    assert_eq!(
        v["summary"]["collisions"], 0,
        "an unchecked run reports no collisions, which is exactly why the exit \
         code and not the count is the answer"
    );
}

#[test]
fn a_clamp_in_the_path_is_found_and_named() {
    let f = Fixture::new("clamp");
    let prog = f.program("shallow", 36.0);
    let (code, out, err) = run(&[
        "collide",
        &f.s("stock.tdx"),
        "--path",
        &prog,
        "--tools",
        &tool_library(),
        "--tool",
        "er32-stub-6",
        "--fixtures",
        &f.s("clamp.stl"),
        "--units",
        "mm",
        "--json",
    ]);
    assert_eq!(code, 1, "a clamp in the path of the chuck must fail: {err}");
    // The shared emitter nests the payload under `results` and puts the host
    // and timing beside it, unhashed.
    let top: Value = serde_json::from_str(&out).expect("valid JSON");
    let v = &top["results"];
    let against_fixture = v["collisions"]
        .as_array()
        .expect("collisions")
        .iter()
        .find(|c| c["obstacle"]["kind"] == "fixture")
        .expect("a fixture collision");
    assert_eq!(
        against_fixture["obstacle"]["name"], "clamp",
        "the fixture must be named rather than merely counted: a report saying \
         something was hit does not say which clamp to move"
    );
}

#[test]
fn a_negative_clearance_is_refused() {
    let f = Fixture::new("badclearance");
    let prog = f.program("shallow", 36.0);
    let (code, _, err) = run(&[
        "collide",
        &f.s("stock.tdx"),
        "--path",
        &prog,
        "--tools",
        &tool_library(),
        "--tool",
        "long-reach-6",
        // `=` rather than a separate argument: clap reads a bare `-1.0` as a
        // flag, which would test the parser instead of the validation.
        "--clearance=-1.0",
    ]);
    assert_ne!(code, 0);
    assert!(
        err.contains("clearance"),
        "the refusal should name the argument: {err}"
    );
}
