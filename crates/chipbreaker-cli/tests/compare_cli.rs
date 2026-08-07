// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! End-to-end tests for `chipbreaker compare` and `deviation-stat`.
//!
//! The whole pipeline in one process: a stock mesh, a field built from it, an NC
//! program cut into it, and a nominal part written out by hand. Every unit up to
//! eleven tested one stage; this is the first that a customer would recognise as
//! the product.
//!
//! # Everything is built here rather than committed
//!
//! The fixtures are a box and a slotted box, both a few dozen triangles of
//! coordinates. Writing them out costs less than a corpus entry and makes the
//! arithmetic checkable by inspection: a 24 x 18 x 10 block less a 6 x 24 x 4
//! channel is 3744 mm^3, and a program that plunges a millimetre deeper than the
//! nominal must report exactly a millimetre of gouge and no excess at all.

use std::path::{Path, PathBuf};
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

fn tool_library() -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/corpus/tool/standard-library.json")
        .to_str()
        .expect("utf-8 path")
        .to_owned()
}

/// Writes a binary STL from vertices and triangles.
///
/// The one place `f32` appears: the STL wire format has no other option, and
/// every fixture coordinate here is a small integer or a half, exactly
/// representable, so the round trip is lossless.
fn write_stl(path: &Path, vertices: &[[f64; 3]], triangles: &[[usize; 3]]) {
    let mut bytes = vec![0u8; 80];
    bytes.extend_from_slice(&u32::try_from(triangles.len()).expect("small").to_le_bytes());
    for tri in triangles {
        // A zero normal: every reader recomputes it from the winding, which is
        // the only source that cannot disagree with the geometry.
        for _ in 0..3 {
            bytes.extend_from_slice(&0f32.to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
        }
        for index in tri {
            for component in vertices[*index] {
                #[allow(clippy::cast_possible_truncation, reason = "STL is f32 by definition")]
                bytes.extend_from_slice(&(component as f32).to_le_bytes()); // ALLOW-f32-WIRE-FORMAT
            }
        }
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    std::fs::write(path, bytes).expect("writes");
}

/// The stock: a 24 x 18 x 10 block, placed clear of the origin.
///
/// Clear of it because a program's first move starts from the machine origin,
/// and a block with a corner there is cut by the very first rapid. That is
/// correct behaviour and a confusing fixture.
const ORIGIN: [f64; 3] = [20.0, 20.0, 20.0];
const SIZE: [f64; 3] = [24.0, 18.0, 10.0];
/// The channel: 6 mm wide, floor 4 mm below the top face.
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

/// The nominal: the same block with the channel milled through it.
///
/// A twelve-sided cross-section extruded along `x`. Twelve rather than eight so
/// the caps triangulate without T-junctions; the reasoning is set out in
/// `chipbreaker-core/tests/deviation_ladder.rs`, which builds the same solid for
/// the same reason.
fn write_nominal(path: &Path, floor: f64) {
    let (x0, y0, z0) = (ORIGIN[0], ORIGIN[1], ORIGIN[2]);
    let (x1, y1, z1) = (x0 + SIZE[0], y0 + SIZE[1], z0 + SIZE[2]);
    let (ya, yb) = SLOT;
    let section = [
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
    let n = section.len();
    let mut v: Vec<[f64; 3]> = section.iter().map(|(y, z)| [x0, *y, *z]).collect();
    v.extend(section.iter().map(|(y, z)| [x1, *y, *z]));

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

/// A single pass right through the block at the given depth.
fn write_program(path: &Path, z: f64) {
    let text = format!(
        "G21 G90\n\
         G0 Z60.\n\
         G0 X14. Y29.\n\
         G0 Z{z:.3}\n\
         G1 X50. F600.\n\
         G0 Z60.\n\
         M30\n"
    );
    std::fs::write(path, text).expect("writes");
}

/// Everything a comparison needs, built into a fresh temporary directory.
struct Fixture {
    dir: PathBuf,
}

impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("chipbreaker-compare-{name}"));
        // Removed first, so a previous run's files can never stand in for this
        // one's -- a stale `.tdx` would make a failing case pass.
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("creates");
        let f = Self { dir };

        write_stock(&f.path("stock.stl"));
        write_nominal(&f.path("nominal.stl"), FLOOR);
        let (code, _, err) = run(&[
            "dexel",
            "build",
            f.str("stock.stl").as_str(),
            "--units",
            "mm",
            "--res",
            "0.4",
            "--axes",
            "xyz",
            "--out",
            f.str("stock.tdx").as_str(),
        ]);
        assert_eq!(code, 0, "building the stock field failed: {err}");
        f
    }

    fn path(&self, name: &str) -> PathBuf {
        self.dir.join(name)
    }

    fn str(&self, name: &str) -> String {
        self.path(name).to_str().expect("utf-8 path").to_owned()
    }

    /// Cuts a program at the given depth and returns the resulting field's path.
    fn cut(&self, name: &str, z: f64) -> String {
        write_program(&self.path(&format!("{name}.nc")), z);
        let (code, _, err) = run(&[
            "run",
            "--stock",
            self.str("stock.tdx").as_str(),
            "--path",
            self.str(&format!("{name}.nc")).as_str(),
            "--tools",
            tool_library().as_str(),
            "--tool",
            "flat-6",
            "--out",
            self.str(&format!("{name}.tdx")).as_str(),
        ]);
        assert_eq!(code, 0, "cutting {name} failed: {err}");
        self.str(&format!("{name}.tdx"))
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn a_correct_program_compares_clean_and_exits_zero() {
    let f = Fixture::new("clean");
    let field = f.cut("clean", FLOOR);
    let (code, out, err) = run(&[
        "compare",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--stock",
        f.str("stock.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.5",
        "--json",
    ]);
    assert_eq!(code, 0, "a correct program must exit zero: {err}\n{out}");

    let v: Value = serde_json::from_str(&out).expect("valid JSON");
    let r = &v["results"];
    assert_eq!(r["accepted"], Value::Bool(true));
    assert_eq!(r["gouge_samples"], 0, "a correct program gouges nothing");
    assert!(
        r["samples"].as_u64().expect("a count") > 10_000,
        "too few samples for the answer to mean anything: {}",
        r["samples"]
    );
    // The sweep of a flat mill straight through a block IS the slotted solid,
    // so the two surfaces coincide exactly rather than nearly.
    let worst = r["worst_gouge_mm"].as_f64().expect("a number");
    assert!(worst < 1.0e-9, "expected an exact match, got {worst:.9} mm");
}

#[test]
fn a_plunge_one_millimetre_deep_reports_one_millimetre_of_gouge() {
    // The whole product in one assertion: an error of known size in, the same
    // size out, with the sign that says which way.
    let f = Fixture::new("deep");
    let field = f.cut("deep", FLOOR - 1.0);
    let (code, out, err) = run(&[
        "compare",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--stock",
        f.str("stock.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.5",
        "--json",
    ]);
    assert_eq!(code, 1, "a gouged part must exit non-zero: {err}\n{out}");

    let v: Value = serde_json::from_str(&out).expect("valid JSON");
    let r = &v["results"];
    assert_eq!(r["accepted"], Value::Bool(false));
    let gouge = r["worst_gouge_mm"].as_f64().expect("a number");
    let excess = r["worst_excess_mm"].as_f64().expect("a number");
    assert!(
        (gouge - 1.0).abs() < 1.0e-9,
        "a millimetre too deep must report a millimetre of gouge, got {gouge:.9}"
    );
    assert!(
        excess < 1.0e-9,
        "cutting too deep leaves no material standing, but {excess:.9} mm of \
         excess was reported. The sign is inverted."
    );
    assert!(
        r["gouge_samples"].as_u64().expect("a count") > 100,
        "a channel the width of the part should gouge many samples, not a few"
    );
}

#[test]
fn a_pass_one_millimetre_shallow_reports_excess_and_still_passes() {
    // The other sign, and the ruling behind it. Material left standing is what a
    // roughing pass is supposed to leave, so it is reported at equal prominence
    // and does not fail the part. A tool that called a roughing pass a failure
    // would be turned off within a day.
    let f = Fixture::new("shallow");
    let field = f.cut("shallow", FLOOR + 1.0);
    let (code, out, err) = run(&[
        "compare",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--stock",
        f.str("stock.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.5",
        "--json",
    ]);
    assert_eq!(code, 0, "excess stock is not a failure: {err}\n{out}");

    let v: Value = serde_json::from_str(&out).expect("valid JSON");
    let r = &v["results"];
    let gouge = r["worst_gouge_mm"].as_f64().expect("a number");
    let excess = r["worst_excess_mm"].as_f64().expect("a number");
    assert!(
        (excess - 1.0).abs() < 1.0e-9,
        "a millimetre short must report a millimetre of excess, got {excess:.9}"
    );
    assert!(
        gouge < 1.0e-9,
        "stopping short gouges nothing, but {gouge:.9} mm was reported"
    );
}

#[test]
fn a_tolerance_below_the_floor_is_refused_with_the_number_that_would_be_honest() {
    // ADR 0005. The refusal names the three inputs and which one is the limit,
    // because "refine your mesh" without saying which one is not actionable.
    let f = Fixture::new("floor");
    let field = f.cut("floor", FLOOR);
    let (code, _, err) = run(&[
        "compare",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--stock",
        f.str("stock.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.001",
    ]);
    assert_ne!(code, 0, "a tolerance below the floor must be refused");
    for expected in [
        "floor",
        "stock facets",
        "nominal facets",
        "lattice",
        "0.4000",
    ] {
        assert!(
            err.contains(expected),
            "the refusal must name {expected}; it said: {err}"
        );
    }

    // And the override works, because the customer may know something the tool
    // does not.
    let (code, _, err) = run(&[
        "compare",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.001",
        "--allow-below-floor",
    ]);
    assert_eq!(code, 0, "--allow-below-floor must report anyway: {err}");
}

#[test]
fn every_report_states_what_the_comparison_does_not_cover() {
    // The binding constraint, checked mechanically rather than trusted to
    // reviewers. A deviation bound covers the ideal geometric cutting model and
    // nothing physical, and a report that lets a customer forget which model it
    // verified is worse than no report.
    let f = Fixture::new("scope");
    let field = f.cut("scope", FLOOR);
    let nominal = f.str("nominal.stl");
    for command in ["compare", "deviation-stat"] {
        for extra in [None, Some("--json")] {
            let mut args = vec![
                command,
                field.as_str(),
                "--nominal",
                nominal.as_str(),
                "--units",
                "mm",
                "--tolerance",
                "0.5",
            ];
            args.extend(extra);
            let (_, out, err) = run(&args);
            for term in ["tool wear", "deflection", "thermal", "runout", "backlash"] {
                assert!(
                    out.contains(term),
                    "{command} did not say it does not model {term}: {out}{err}"
                );
            }
        }
    }
}

#[test]
fn deviation_stat_localises_the_defect_and_publishes_both_rulers() {
    let f = Fixture::new("stat");
    let field = f.cut("stat", FLOOR - 1.0);
    let (code, out, err) = run(&[
        "deviation-stat",
        field.as_str(),
        "--nominal",
        f.str("nominal.stl").as_str(),
        "--units",
        "mm",
        "--tolerance",
        "0.5",
        "--worst",
        "5",
        "--json",
    ]);
    assert_eq!(
        code, 0,
        "a distribution is a description, not a verdict: {err}"
    );

    let v: Value = serde_json::from_str(&out).expect("valid JSON");
    let r = &v["results"];
    let worst = r["worst_samples"].as_array().expect("an array");
    assert_eq!(worst.len(), 5, "--worst 5 must return five");

    // Every one of them is on the gouged floor, one millimetre down.
    for s in worst {
        let signed = s["signed_mm"].as_f64().expect("a number");
        assert!(
            (signed + 1.0).abs() < 1.0e-9,
            "the worst samples should all be the 1 mm floor, got {signed:.9}"
        );
        let z = s["at"].as_array().expect("a point")[2]
            .as_f64()
            .expect("a number");
        assert!(
            (z - (FLOOR - 1.0)).abs() < 1.0e-9,
            "the defect is at z = {}, but a worst sample sits at {z}",
            FLOOR - 1.0
        );
    }

    // The two rulers disagree somewhere, and that is reported rather than
    // hidden: at the corner where the channel wall meets the gouged floor, the
    // perpendicular cast leaves along the floor's normal, misses the wall and
    // strikes the top face five millimetres away.
    let gap = r["worst_projection_gap_mm"].as_f64().expect("a number");
    assert!(
        gap > 1.0,
        "the perpendicular ruler must be seen to overstate at a step edge; the \
         report says {gap:.4} mm, which suggests it is not being computed"
    );
}
