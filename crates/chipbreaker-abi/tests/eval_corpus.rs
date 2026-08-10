// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The evaluation corpus, run against its own expectations.
//!
//! # Why the shipped corpus is also a test
//!
//! `tests/corpus/eval/expectations.json` is handed to integrators so they can
//! prove their integration is correct rather than merely running. A file like
//! that is only worth having if it is true, and the way to keep it true is to
//! check it in the same build that ships it.
//!
//! This has gone wrong here before. Twice, in the defect corpus, cases sat in
//! the denominator claiming to contain a defect they did not contain. The rule
//! that came out of it applies exactly here: **a corpus is an oracle only if
//! something independent confirms its cases contain what they claim.**

use std::path::{Path, PathBuf};

use serde_json::Value;

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/corpus/eval")
}

fn read(name: &str) -> Vec<u8> {
    std::fs::read(corpus_dir().join(name))
        .unwrap_or_else(|e| panic!("the corpus must contain {name}: {e}"))
}

fn tools() -> String {
    let p =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/corpus/tool/standard-library.json");
    std::fs::read_to_string(p).expect("the standard tool library must be readable")
}

fn expectations() -> Value {
    let text = std::fs::read_to_string(corpus_dir().join("expectations.json"))
        .expect("expectations.json must be readable");
    serde_json::from_str(&text).expect("expectations.json must be valid JSON")
}

#[test]
fn every_case_produces_what_the_corpus_says_it_does() {
    let spec = expectations();
    let tools = tools();
    let stock = read(
        spec["settings"]["stock"]
            .as_str()
            .expect("a stock mesh is named"),
    );
    let resolution = spec["settings"]["resolution_mm"]
        .as_f64()
        .expect("a resolution is stated");
    let tolerance = spec["settings"]["tolerance_mm"]
        .as_f64()
        .expect("a tolerance is stated");

    let cases = spec["cases"].as_array().expect("cases is an array");
    assert!(!cases.is_empty(), "an empty corpus proves nothing");

    for case in cases {
        let id = case["id"].as_str().expect("every case is named");
        let program_bytes = read(case["program"].as_str().expect("every case has a program"));
        let program = String::from_utf8(program_bytes).expect("programs are UTF-8");
        let nominal = case["nominal"].as_str().map(read);
        let expect = &case["expect"];

        let outcome = chipbreaker_gcode::pipeline::run(&chipbreaker_gcode::pipeline::JobRequest {
            program: &program,
            tools: &tools,
            stock_stl: &stock,
            nominal_stl: nominal.as_deref(),
            tool_id: None,
            source: Some(id),
            resolution_mm: resolution,
            tolerance_mm: tolerance,
            clearance_mm: 0.0,
            memory_ceiling_bytes: None,
            segment_cap: None,
        });

        match expect["outcome"].as_str().expect("an outcome is stated") {
            "refused" => {
                let why = match outcome {
                    Ok(_) => panic!("{id}: the corpus says this is refused, and it ran"),
                    Err(why) => why,
                };
                for needle in expect["message_contains"]
                    .as_array()
                    .expect("a refusal states what its message contains")
                {
                    let needle = needle.as_str().expect("needles are strings");
                    assert!(
                        why.contains(needle),
                        "{id}: the refusal must contain {needle:?}, and says: {why}"
                    );
                }
            }
            "ran" => {
                let report = match outcome {
                    Ok(r) => r,
                    Err(why) => {
                        panic!("{id}: the corpus says this runs, and it was refused: {why}")
                    }
                };
                let json = report.to_json();

                for (gate, state) in expect["gates"]
                    .as_object()
                    .expect("gate states are stated")
                    .iter()
                {
                    assert_eq!(
                        json["verdict"]["gates"][gate]["state"], *state,
                        "{id}: gate {gate}"
                    );
                }

                assert_eq!(
                    json["summary"]["total"], expect["findings"],
                    "{id}: finding count"
                );
                assert_eq!(
                    json["summary"]["collisions"], expect["collisions"],
                    "{id}: collision count"
                );

                if let Some(want) = expect["worst_gouge_mm"].as_f64() {
                    let got = json["summary"]["worst_gouge_mm"]
                        .as_f64()
                        .expect("worst_gouge_mm is a number");
                    // A tolerance where one is stated, exact where it is not:
                    // a case claiming 0.0 gouges means *none*, and a slack
                    // comparison there would let an invented gouge through.
                    let slack = expect["worst_gouge_tolerance_mm"].as_f64().unwrap_or(0.0);
                    assert!(
                        (got - want).abs() <= slack,
                        "{id}: worst gouge {got} is not {want} within {slack}"
                    );
                }
            }
            other => panic!("{id}: unknown outcome {other:?}"),
        }
    }
}

#[test]
fn the_injected_gouge_is_attributed_to_the_line_that_caused_it() {
    // The corpus checks counts and depths. This checks the thing an integrator
    // actually shows a machinist: which line to go and look at. A finding with
    // the right depth and no attribution is half a result.
    let tools = tools();
    let stock = read("stock.stl");
    let nominal = read("nominal-faced.stl");
    let program = String::from_utf8(read("faced-gouge.nc")).expect("UTF-8");

    let report = chipbreaker_gcode::pipeline::run(&chipbreaker_gcode::pipeline::JobRequest {
        program: &program,
        tools: &tools,
        stock_stl: &stock,
        nominal_stl: Some(&nominal),
        tool_id: None,
        source: Some("faced-gouge"),
        resolution_mm: 0.5,
        tolerance_mm: 0.1,
        clearance_mm: 0.0,
        memory_ceiling_bytes: None,
        segment_cap: None,
    })
    .expect("this program runs");

    let finding = report.findings.first().expect("one finding");
    assert!(
        !finding.attribution.segments.is_empty(),
        "a finding with no attribution names no line, and a machinist cannot act on it"
    );

    // The lane cut 1 mm deep is the sixth of eleven, and the file writes four
    // lines per lane after a four-line preamble. Pinning the line rather than
    // merely "some line" is what makes this a test of attribution rather than
    // of its presence.
    let lines: Vec<u32> = finding
        .attribution
        .provenance
        .iter()
        .map(|p| p.line)
        .collect();
    let text: Vec<&str> = program.lines().collect();
    for line in &lines {
        let source = text[(*line as usize) - 1];
        assert!(
            source.contains("Z22.000") || source.contains("G1 X"),
            "line {line} ({source:?}) is not part of the lane that was cut too deep"
        );
    }
}
