// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! End-to-end tests for `chipbreaker verify` and `report-diff`.
//!
//! The artifact is the deliverable, so this file tests the artifact: that it is
//! byte-stable, that its identity means what it claims, that a diff of two runs
//! shows exactly the intended change, and that a roughing pass is not reported
//! as a failure.
//!
//! Every test carries the mutation check `CONTRIBUTING.md` requires. It matters
//! more here than in the numeric layers: these assertions are about judgements,
//! and a judgement test passes vacuously in ways a numeric one cannot — by
//! comparing two empty finding lists, or by asserting a diff is empty when both
//! reports failed to parse.

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

/// Writes a binary STL.
fn write_stl(path: &Path, vertices: &[[f64; 3]], triangles: &[[usize; 3]]) {
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&u32::try_from(triangles.len()).expect("small").to_le_bytes());
    for t in triangles {
        for _ in 0..3 {
            b.extend_from_slice(&0f32.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
        }
        for i in t {
            for c in vertices[*i] {
                // ALLOW-f32-WIRE-FORMAT
                #[allow(clippy::cast_possible_truncation, reason = "the format is 32-bit")]
                b.extend_from_slice(&(c as f32).to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
            }
        }
        b.extend_from_slice(&0u16.to_le_bytes());
    }
    std::fs::write(path, b).expect("writes");
}

const ORIGIN: [f64; 3] = [20.0, 20.0, 20.0];
const SIZE: [f64; 3] = [24.0, 18.0, 10.0];
const SLOT: (f64, f64) = (26.0, 32.0);
const FLOOR: f64 = 26.0;

fn write_stock(path: &Path) {
    let (x0, y0, z0) = (ORIGIN[0], ORIGIN[1], ORIGIN[2]);
    let (x1, y1, z1) = (x0 + SIZE[0], y0 + SIZE[1], z0 + SIZE[2]);
    let v = [
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1],
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
    write_stl(path, &v, &t);
}

/// The slotted block, as a hand-written twelve-sided prism.
fn write_nominal(path: &Path, floor: f64) {
    let (x0, y0, z0) = (ORIGIN[0], ORIGIN[1], ORIGIN[2]);
    let (x1, y1, z1) = (x0 + SIZE[0], y0 + SIZE[1], z0 + SIZE[2]);
    let (ya, yb) = SLOT;
    let sec = [
        (y0, z0),
        (ya, z0),
        (yb, z0),
        (y1, z0),
        (y1, floor),
        (y1, z1),
        (yb, z1),
        (yb, floor),
        (ya, floor),
        (ya, z1),
        (y0, z1),
        (y0, floor),
    ];
    let n = sec.len();
    let mut v: Vec<[f64; 3]> = sec.iter().map(|(y, z)| [x0, *y, *z]).collect();
    v.extend(sec.iter().map(|(y, z)| [x1, *y, *z]));
    let mut t: Vec<[usize; 3]> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        t.push([i, j, j + n]);
        t.push([i, j + n, i + n]);
    }
    for q in [
        [0, 1, 8, 11],
        [1, 2, 7, 8],
        [2, 3, 4, 7],
        [11, 8, 9, 10],
        [7, 4, 5, 6],
    ] {
        t.push([q[2], q[1], q[0]]);
        t.push([q[3], q[2], q[0]]);
        t.push([q[0] + n, q[1] + n, q[2] + n]);
        t.push([q[0] + n, q[2] + n, q[3] + n]);
    }
    write_stl(path, &v, &t);
}

fn write_program(path: &Path, z: f64) {
    std::fs::write(
        path,
        format!("G21 G90\nG0 Z60.\nG0 X14. Y29.\nG0 Z{z:.3}\nG1 X50. F600.\nG0 Z60.\nM30\n"),
    )
    .expect("writes");
}

struct Fixture {
    dir: PathBuf,
}

impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("chipbreaker-verify-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("creates");
        let f = Self { dir };
        write_stock(&f.path("stock.stl"));
        write_nominal(&f.path("nominal.stl"), FLOOR);
        let (code, _, err) = run(&[
            "dexel",
            "build",
            &f.s("stock.stl"),
            "--units",
            "mm",
            "--res",
            "0.4",
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

    fn cut(&self, name: &str, z: f64) -> String {
        write_program(&self.path(&format!("{name}.nc")), z);
        let (code, _, err) = run(&[
            "run",
            "--stock",
            &self.s("stock.tdx"),
            "--path",
            &self.s(&format!("{name}.nc")),
            "--tools",
            &tool_library(),
            "--tool",
            "flat-6",
            "--out",
            &self.s(&format!("{name}.tdx")),
        ]);
        assert_eq!(code, 0, "cutting {name} failed: {err}");
        self.s(&format!("{name}.tdx"))
    }

    /// Runs `verify` and returns the exit code and the parsed report.
    fn verify(&self, name: &str, field: &str, extra: &[&str]) -> (i32, Value) {
        let report = self.s(&format!("{name}-report.json"));
        let nominal = self.s("nominal.stl");
        let stock = self.s("stock.stl");
        let program = self.s(&format!("{name}.nc"));
        let tools = tool_library();
        let mut args = vec![
            "verify",
            field,
            "--nominal",
            &nominal,
            "--stock",
            &stock,
            "--path",
            &program,
            "--tools",
            &tools,
            "--tool",
            "flat-6",
            "--units",
            "mm",
            "--tol",
            "0.5",
            "--report",
            &report,
        ];
        args.extend(extra);
        let (code, _, err) = run(&args);
        assert!(
            self.path(&format!("{name}-report.json")).exists(),
            "no report was written: {err}"
        );
        let text = std::fs::read_to_string(&report).expect("reads");
        (code, serde_json::from_str(&text).expect("valid JSON"))
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn a_report_is_byte_stable_across_runs() {
    // The property everything else rests on. If two runs of one input disagree,
    // a diff is noise and a manifest identity is a lie.
    let f = Fixture::new("stable");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, a) = f.verify("deep", &field, &[]);
    let first = std::fs::read_to_string(f.path("deep-report.json")).expect("reads");
    let (_, b) = f.verify("deep", &field, &[]);
    let second = std::fs::read_to_string(f.path("deep-report.json")).expect("reads");

    assert_eq!(
        first, second,
        "two runs of the same inputs produced different report bytes"
    );
    assert_eq!(a["manifest"]["digest"], b["manifest"]["digest"]);
    assert!(
        !a["findings"].as_array().expect("findings").is_empty(),
        "the fixture produced no findings, so byte-stability was compared \
         between two empty lists and proves nothing"
    );
}

#[test]
fn the_same_manifest_implies_the_same_findings() {
    // The manifest's whole claim. Two runs sharing a manifest digest must
    // produce identical findings; if they could differ, the digest identifies
    // nothing and the report cannot be audited.
    let f = Fixture::new("manifest");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, a) = f.verify("deep", &field, &[]);
    let (_, b) = f.verify("deep", &field, &[]);
    assert_eq!(a["manifest"]["digest"], b["manifest"]["digest"]);
    assert_eq!(a["findings"], b["findings"]);

    // And the converse: change a setting and the digest must move, or it is not
    // covering the settings it claims to.
    let (_, c) = f.verify("deep", &field, &["--cluster-radius", "3.0"]);
    assert_ne!(
        a["manifest"]["digest"], c["manifest"]["digest"],
        "changing the cluster radius left the manifest digest unchanged, so the \
         digest does not cover the settings that produced the findings"
    );
}

#[test]
fn a_roughing_pass_is_not_reported_as_a_failure() {
    // **The ruling this unit turns on.** A pass that stops short of the nominal
    // leaves excess stock, which is what roughing is for. Reporting it as a
    // defect would fail every roughing operation ever simulated, and a tool that
    // does that is switched off within a day.
    let f = Fixture::new("roughing");
    let field = f.cut("shallow", FLOOR + 1.0);
    let (code, r) = f.verify("shallow", &field, &[]);

    let by_class = &r["summary"]["by_class"];
    println!(
        "roughing pass: {} excess, {} gouge, gouge gate={}",
        by_class["excess-stock"], by_class["gouge"], r["verdict"]["gates"]["gouge"]["state"]
    );

    // The *gouge* gate is what this test is about, and it must pass. The overall
    // verdict does not, because `verify` does not replay the program and so
    // cannot speak for the collision gate -- which is asserted separately in
    // `verify_alone_cannot_certify_collisions`. Reading `pass` here would
    // conflate "roughing is fine" with "everything was checked".
    assert_eq!(r["verdict"]["gates"]["gouge"]["state"], "pass");
    assert_eq!(
        code, 1,
        "an unchecked collision gate must keep the exit code non-zero"
    );
    assert!(
        by_class["excess-stock"].as_u64().expect("a count") > 0,
        "a pass a millimetre shallow must leave excess stock to report; with \
         none, this test asserts nothing about how excess is treated"
    );
    assert_eq!(
        by_class["gouge"].as_u64().expect("a count"),
        0,
        "stopping short gouges nothing"
    );
    // Reported, and marked as not a defect.
    let excess = r["findings"]
        .as_array()
        .expect("findings")
        .iter()
        .find(|x| x["class"] == "excess-stock")
        .expect("an excess finding");
    assert_eq!(
        excess["is_defect"],
        Value::Bool(false),
        "excess stock must be reported as information, not as a defect"
    );
}

#[test]
fn a_gouge_fails_the_run_and_names_the_line() {
    // The other side of the same ruling, and the mutation check for the test
    // above: if a gouge also passed, "excess does not fail" would be vacuous.
    let f = Fixture::new("gouge");
    let field = f.cut("deep", FLOOR - 1.0);
    let (code, r) = f.verify("deep", &field, &[]);

    assert_eq!(code, 1, "a gouged part must fail the run");
    assert_eq!(r["verdict"]["gates"]["gouge"]["state"], "fail");
    assert_eq!(r["verdict"]["pass"], Value::Bool(false));
    assert!(
        r["accepted"].is_null(),
        "`accepted` was removed in schema version 2; leaving it in place beside \
         `verdict` would let a version-1 consumer keep reading a bit that no \
         longer accounts for collisions"
    );

    let gouge = r["findings"]
        .as_array()
        .expect("findings")
        .iter()
        .find(|x| x["class"] == "gouge")
        .expect("a gouge");
    assert_eq!(gouge["is_defect"], Value::Bool(true));
    let depth = gouge["severity"]["worst_depth_mm"]
        .as_f64()
        .expect("a depth");
    assert!(
        (depth - 1.0).abs() < 1.0e-6,
        "a millimetre too deep must report a millimetre, got {depth:.9}"
    );

    // The line, which is what makes it actionable. `G1 X50.` is line 5.
    let segs = gouge["attribution"]["segments"]
        .as_array()
        .expect("segments");
    assert!(
        !segs.is_empty(),
        "a gouge with no attribution names no line"
    );
    let lines: Vec<u64> = segs
        .iter()
        .map(|s| s["line"].as_u64().unwrap_or(0))
        .collect();
    assert!(
        lines.contains(&5),
        "the cutting pass is on line 5, and the attribution named {lines:?}"
    );
}

#[test]
fn severity_reports_depth_and_extent_separately() {
    // A 2 mm gouge over one cell and a 0.2 mm gouge over a whole face are
    // different problems. Collapsing them into one score destroys exactly the
    // information a machinist needs, so both are present and neither is a
    // combination of the other.
    let f = Fixture::new("severity");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, r) = f.verify("deep", &field, &[]);
    let s = &r["findings"].as_array().expect("findings")[0]["severity"];
    for key in [
        "worst_depth_mm",
        "mean_depth_mm",
        "area_estimate_mm2",
        "volume_estimate_mm3",
    ] {
        assert!(
            s[key].is_number(),
            "severity is missing {key}, so depth and extent are not both reported"
        );
    }
    assert!(
        s["note"].as_str().is_some_and(|n| n.contains("separately")),
        "the severity block must say that depth and area are deliberately apart"
    );
}

#[test]
fn every_report_carries_its_semantics_and_its_exclusions() {
    // A finding without its error budget is not evidence. Checked mechanically
    // rather than trusted to review, because this is the section that makes the
    // artifact worth more than a number.
    let f = Fixture::new("semantics");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, r) = f.verify("deep", &field, &[]);

    let n = &r["numerical_semantics"];
    for key in [
        "spacing_mm",
        "tolerance_mm",
        "stock_facet_mm",
        "nominal_facet_mm",
        "tolerance_floor_mm",
        "below_floor",
        "swept_volumes",
        "worst_projection_gap_mm",
        "detection_floor",
    ] {
        assert!(!n[key].is_null(), "numerical_semantics is missing {key}");
    }

    // Without a run report the split is genuinely unknown, and the artifact
    // must say so rather than emit zeros. "No ray-cut was bounded" is a strong
    // claim to make by accident in a document somebody audits.
    assert_eq!(
        n["swept_volumes"]["available"],
        Value::Bool(false),
        "with no --run-report the swept split must be marked unavailable, not          reported as zero"
    );
    assert!(
        n["swept_volumes"]["why"]
            .as_str()
            .is_some_and(|w| w.contains("--run-report")),
        "an unavailable section must say how to make it available"
    );

    let ex = r["exclusions"].as_array().expect("exclusions");
    let joined = ex
        .iter()
        .filter_map(|v| v.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    for term in [
        "wear",
        "deflection",
        "thermal",
        "runout",
        "backlash",
        "interpolation",
    ] {
        assert!(
            joined.contains(term),
            "the report does not exclude {term}; the exclusions were {joined:?}"
        );
    }
    assert!(
        r["scope"]
            .as_str()
            .is_some_and(|s| s.contains("not the machine")),
        "the report must say what it verified, in the artifact"
    );
}

#[test]
fn a_run_report_supplies_the_swept_volume_split() {
    // The mutation check for the assertion above: with the run's own statistics
    // supplied, the section must become available and carry real counts. If it
    // stayed unavailable, "unavailable when absent" would be trivially true.
    let f = Fixture::new("swept");
    write_program(&f.path("deep.nc"), FLOOR - 1.0);
    let (code, out, err) = run(&[
        "run",
        "--stock",
        &f.s("stock.tdx"),
        "--path",
        &f.s("deep.nc"),
        "--tools",
        &tool_library(),
        "--tool",
        "flat-6",
        "--out",
        &f.s("deep.tdx"),
        "--json",
    ]);
    assert_eq!(code, 0, "the run failed: {err}");
    std::fs::write(f.path("run.json"), &out).expect("writes");

    let field = f.s("deep.tdx");
    let run_report = f.s("run.json");
    let (_, r) = f.verify("deep", &field, &["--run-report", &run_report]);
    let sw = &r["numerical_semantics"]["swept_volumes"];
    println!("swept split: {sw}");

    assert_eq!(sw["available"], Value::Bool(true));
    let exact = sw["ray_cuts_exact"].as_u64().expect("a count");
    let bounded = sw["ray_cuts_bounded"].as_u64().expect("a count");
    assert!(
        exact + bounded > 0,
        "the split is available but empty, so it carries no information"
    );
    assert!(
        sw["worst_bound_applies_to"]
            .as_str()
            .is_some_and(|s| s.contains("sub-stepped")),
        "the worst bound must say it belongs to the bounded ray-cuts alone"
    );
}

#[test]
fn report_diff_finds_exactly_the_intended_change() {
    // The operational benefit. One segment perturbed, and the diff must contain
    // that finding and nothing else.
    let f = Fixture::new("diff");
    let clean = f.cut("clean", FLOOR);
    let deep = f.cut("deep", FLOOR - 1.0);
    let (_, before) = f.verify("clean", &clean, &[]);
    let (_, after) = f.verify("deep", &deep, &[]);
    assert_ne!(
        before["findings"], after["findings"],
        "the fixture did not change"
    );

    let (code, out, err) = run(&[
        "report-diff",
        &f.s("clean-report.json"),
        &f.s("deep-report.json"),
        "--json",
    ]);
    assert_ne!(code, 0, "a differing pair must exit non-zero: {err}");
    let d: Value = serde_json::from_str(&out).expect("valid JSON");
    let r = &d["results"];
    println!(
        "{}",
        serde_json::to_string_pretty(&r["summary"]).unwrap_or_default()
    );

    assert_eq!(
        r["summary"]["appeared"], 1,
        "exactly one finding should appear"
    );
    assert_eq!(r["summary"]["disappeared"], 0);
    assert_eq!(r["summary"]["changed"], 0);
    let appeared = &r["changes"].as_array().expect("changes")[0];
    assert_eq!(appeared["class"], "gouge");

    // The manifest difference is reported, and reported as an explanation.
    let m = r["manifest_differences"].as_array().expect("manifest");
    assert!(
        !m.is_empty(),
        "the field and program both changed; a diff that reports no manifest \
         difference would let a settings change masquerade as a program bug"
    );
}

#[test]
fn report_diff_on_identical_reports_is_empty_and_exits_zero() {
    // The mutation check for the test above: if a diff of two *identical*
    // reports also reported changes, "exactly one appeared" would prove nothing.
    let f = Fixture::new("diffsame");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, _) = f.verify("deep", &field, &[]);
    std::fs::copy(f.path("deep-report.json"), f.path("copy.json")).expect("copies");

    let (code, out, err) = run(&[
        "report-diff",
        &f.s("deep-report.json"),
        &f.s("copy.json"),
        "--json",
    ]);
    assert_eq!(code, 0, "identical reports must exit zero: {err}");
    let d: Value = serde_json::from_str(&out).expect("valid JSON");
    assert_eq!(d["results"]["identical"], Value::Bool(true));
    assert_eq!(d["results"]["summary"]["appeared"], 0);
    assert_eq!(d["results"]["summary"]["disappeared"], 0);
    assert_eq!(d["results"]["summary"]["changed"], 0);
}

#[test]
fn report_diff_refuses_a_file_that_is_not_a_report() {
    // A diff that silently treated arbitrary JSON as an empty report would exit
    // zero and say "identical", which is the most dangerous possible answer for
    // a CI gate.
    let f = Fixture::new("diffbad");
    std::fs::write(f.path("not-a-report.json"), "{\"hello\": 1}\n").expect("writes");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, _) = f.verify("deep", &field, &[]);

    let (code, _, err) = run(&[
        "report-diff",
        &f.s("not-a-report.json"),
        &f.s("deep-report.json"),
    ]);
    assert_ne!(
        code, 0,
        "a non-report must be refused, not treated as empty"
    );
    assert!(
        err.contains("not a Chipbreaker verification report"),
        "the refusal should say what the file is not: {err}"
    );
}

#[test]
fn a_version_1_report_is_refused_rather_than_misread() {
    // **The reason the field was renamed rather than widened.**
    //
    // Every version of this schema has *removed* what it replaced rather than
    // aliasing it: version 2 dropped `accepted`, version 3 renamed three
    // severity fields. A reader that accepted an older file would find those
    // keys absent and report values it invented -- which is exactly the silent
    // misreading that removing rather than aliasing exists to make impossible.
    //
    // So this asserts the loud failure, not merely that the new shape works.
    let f = Fixture::new("v1");
    let field = f.cut("deep", FLOOR - 1.0);
    let (_, r) = f.verify("deep", &field, &[]);

    // A version-1-shaped report: the old field, the old version number.
    let mut v1 = r.clone();
    let map = v1.as_object_mut().expect("an object");
    map.insert("schema_version".to_owned(), Value::from(1));
    map.insert("accepted".to_owned(), Value::Bool(true));
    map.remove("verdict");
    std::fs::write(
        f.path("v1-report.json"),
        serde_json::to_string_pretty(&v1).expect("renders") + "\n",
    )
    .expect("writes");

    let (code, out, err) = run(&[
        "report-diff",
        &f.s("v1-report.json"),
        &f.s("deep-report.json"),
    ]);
    assert_ne!(
        code, 0,
        "a version-1 report must be refused, not read with version-2 code"
    );
    assert!(
        err.contains("schema version 1") && err.contains("Regenerate"),
        "the refusal must name the version it was handed and say what to do, \
         rather than merely reporting that something went wrong: {err}"
    );
    assert!(
        !out.contains("identical"),
        "refusing must not also claim the two reports agree"
    );
}

#[test]
fn verify_alone_cannot_certify_collisions() {
    // `verify` compares a cut field against a nominal. It never replays the
    // program, so it has looked at no trajectory and no holder -- and a
    // collision gate reporting `pass` on that basis would be manufacturing
    // safety out of work that was never done.
    let f = Fixture::new("uncertified");
    let field = f.cut("shallow", FLOOR + 1.0);
    let (code, r) = f.verify("shallow", &field, &[]);

    assert_eq!(r["verdict"]["gates"]["gouge"]["state"], "pass");
    assert_eq!(
        r["verdict"]["gates"]["collision"]["state"], "unchecked",
        "verify must not claim a collision gate it did not run"
    );
    assert_eq!(
        r["verdict"]["pass"],
        Value::Bool(false),
        "an unchecked gate must not certify the run"
    );
    assert_eq!(code, 1, "the exit code must follow the whole verdict");

    let why = r["verdict"]["gates"]["collision"]["why"]
        .as_str()
        .expect("an unchecked gate must say why");
    assert!(
        why.contains("collide"),
        "the reason must name what to run to get the gate: {why}"
    );
}
