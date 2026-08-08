// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does a job run in pieces equal the same job run in one go?
//!
//! # The headline property, and why it is achievable at all
//!
//! Cutting is interval subtraction on each ray, and subtraction is associative:
//! removing A then B leaves the same set as removing `A ∪ B`. Nothing in a field
//! accumulates error between operations, because a span endpoint is an exact
//! root of the swept surface rather than a value carried forward and refined.
//!
//! That is what makes **bit-identical** the right bar here rather than "close".
//! A tool that answered slightly differently depending on where an operation
//! boundary fell would make the boundary itself a variable a user has to reason
//! about, and there would be no principled place to put it.
//!
//! So this file asserts equality of bytes, and the mutation checks below exist
//! because byte equality is exactly the kind of assertion that passes for the
//! wrong reason — two empty files are also bit-identical.

use std::path::{Path, PathBuf};
use std::process::Command;

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

/// The three passes, as separate programs and as one concatenation.
const PASSES: [(&str, f64, f64); 3] = [("p1", 10.0, 18.0), ("p2", 20.0, 15.0), ("p3", 30.0, 12.0)];

impl Fixture {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("chipbreaker-chain-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("creates");
        let f = Self { dir };
        write_box(&f.path("stock.stl"), [0.0; 3], [70.0, 40.0, 25.0]);
        let (code, _, err) = run(&[
            "dexel",
            "build",
            &f.s("stock.stl"),
            "--units",
            "mm",
            "--res",
            "0.5",
            "--axes",
            "xyz",
            "--out",
            &f.s("stock.tdx"),
        ]);
        assert_eq!(code, 0, "building the stock field failed: {err}");

        // One pass per operation, and the same three concatenated.
        let mut all = String::from("G21 G90\n");
        for (name, y, z) in PASSES {
            let body = format!("G0 Z50.\nG0 X-10. Y{y:.1}\nG0 Z{z:.1}\nG1 X80. F600.\nG0 Z50.\n");
            std::fs::write(
                f.path(&format!("{name}.nc")),
                format!("G21 G90\n{body}M30\n"),
            )
            .expect("writes");
            all.push_str(&body);
        }
        all.push_str("M30\n");
        std::fs::write(f.path("all.nc"), all).expect("writes");
        f
    }

    fn path(&self, n: &str) -> PathBuf {
        self.dir.join(n)
    }

    fn s(&self, n: &str) -> String {
        self.path(n).to_str().expect("utf-8").to_owned()
    }

    /// Cuts `program` from `stock_in`, writing `out`.
    fn cut(&self, stock_in: &str, program: &str, out: &str) {
        let (code, _, err) = run(&[
            "run",
            "--stock",
            &self.s(stock_in),
            "--path",
            &self.s(program),
            "--tools",
            &tool_library(),
            "--tool",
            "flat-6",
            "--out",
            &self.s(out),
        ]);
        assert_eq!(code, 0, "cutting {program} failed: {err}");
    }

    fn bytes(&self, n: &str) -> Vec<u8> {
        std::fs::read(self.path(n)).expect("the field must exist")
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn a_chained_job_is_bit_identical_to_the_monolithic_one() {
    // **The headline test of the unit.**
    let f = Fixture::new("headline");

    // Three operations, each consuming the previous field.
    f.cut("stock.tdx", "p1.nc", "s1.tdx");
    f.cut("s1.tdx", "p2.nc", "s2.tdx");
    f.cut("s2.tdx", "p3.nc", "chained.tdx");

    // The same three passes as one program.
    f.cut("stock.tdx", "all.nc", "mono.tdx");

    let (chained, mono) = (f.bytes("chained.tdx"), f.bytes("mono.tdx"));
    assert_eq!(
        chained.len(),
        mono.len(),
        "the two fields are different sizes, so the comparison below would be \
         meaningless even if the prefixes matched"
    );
    assert!(
        chained == mono,
        "a job cut in three operations differs from the same job cut in one. \
         Cutting is interval subtraction and subtraction is associative, so this \
         is a bug in how state crosses an operation boundary rather than a \
         tolerance to be widened"
    );
}

#[test]
fn the_comparison_would_notice_a_missing_operation() {
    // The mutation check, and it earns its place: byte equality passes for the
    // wrong reason more easily than most assertions. Two fields that were never
    // cut at all are also bit-identical.
    let f = Fixture::new("mutation");
    f.cut("stock.tdx", "p1.nc", "s1.tdx");
    f.cut("s1.tdx", "p2.nc", "two.tdx");
    f.cut("two.tdx", "p3.nc", "three.tdx");
    f.cut("stock.tdx", "all.nc", "mono.tdx");

    // Stopping one operation early must NOT match.
    assert_ne!(
        f.bytes("two.tdx"),
        f.bytes("mono.tdx"),
        "a chain missing its last operation matched the full job, so the \
         headline assertion is not sensitive to the cutting at all"
    );
    // And the uncut stock must not match either.
    assert_ne!(
        f.bytes("stock.tdx"),
        f.bytes("mono.tdx"),
        "the uncut stock matched the finished job, so the programs remove nothing"
    );
    // While the full chain does.
    assert_eq!(f.bytes("three.tdx"), f.bytes("mono.tdx"));
}

#[test]
fn the_order_of_operations_is_preserved_across_the_boundary() {
    // Splitting at a *different* point must still give the same answer: the
    // boundary is not a variable the user has to reason about.
    let f = Fixture::new("split");
    // Split after one pass.
    f.cut("stock.tdx", "p1.nc", "a1.tdx");
    f.cut("a1.tdx", "p2.nc", "a2.tdx");
    f.cut("a2.tdx", "p3.nc", "split_early.tdx");

    // Split after two, by running the first two as one program.
    let two = std::fs::read_to_string(f.path("p1.nc")).expect("reads");
    let second = std::fs::read_to_string(f.path("p2.nc")).expect("reads");
    let joined = format!(
        "{}{}",
        two.trim_end_matches("M30\n"),
        second.trim_start_matches("G21 G90\n")
    );
    std::fs::write(f.path("p12.nc"), joined).expect("writes");
    f.cut("stock.tdx", "p12.nc", "b1.tdx");
    f.cut("b1.tdx", "p3.nc", "split_late.tdx");

    assert_eq!(
        f.bytes("split_early.tdx"),
        f.bytes("split_late.tdx"),
        "moving the operation boundary changed the result, which would make the \
         boundary itself a variable with no principled place to put it"
    );
}
