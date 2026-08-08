// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Byte mutations of valid programs must parse or error cleanly.
//!
//! Never panic, never hang, never admit a NaN into the IR. A parser reads
//! whatever a user hands it, and a user hands it whatever their post-processor
//! produced — including files truncated by a full disk and files that went
//! through a text editor that helpfully re-encoded them.
//!
//! `#[ignore]`d and run nightly, like the mesh fuzz: too slow for every commit,
//! too valuable to drop. Run with:
//!
//! ```text
//! cargo test -p chipbreaker-gcode --release -- --ignored --nocapture
//! ```

use chipbreaker_gcode::resolve::{ParseOptions, parse};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Fixed, so a failure reproduces on the first try.
const FUZZ_SEED: u64 = 0x0000_C41B_0000_0050;

/// Programs to mutate. Between them they reach every stage of the parser.
const SEEDS: &[&str] = &[
    "%\nO1000\nG21 G90 G17 G54\nG0 X0. Y0. Z10.\nS8000 M3\nG1 Z-1. F250.\n\
     G1 X20.5 Y-3.25\nG3 X0. Y10. I-10. J0.\nG0 Z10.\nM5\nM30\n%\n",
    "G21 G90\nG0 X0. Y0. Z10.\nF250.\nG98 G81 X20. Y30. Z-5. R2.\nX40.\nG80\nM30\n",
    "G21 G90\nG10 L2 P1 X-250.5 Y-100.25 Z-2.0481555856608242\n\
     G54 G0 X0. Y0.\nG53 G0 X-10.\nG92 X0.\nM30\n",
    "G20 G90 G93 G1 X1. F4.\nG94 G21\nM98 P100\nM30\nO100\nG91 G1 X10.\nG90\nM99\n",
];

/// One mutation of `text`.
fn mutate(rng: &mut StdRng, text: &str) -> String {
    let mut bytes = text.as_bytes().to_vec();
    if bytes.is_empty() {
        return String::new();
    }
    match rng.random_range(0..6) {
        // Flip a byte to another printable one. The commonest real corruption.
        0 => {
            let at = rng.random_range(0..bytes.len());
            bytes[at] = rng.random_range(32u8..127);
        }
        // Delete a run: a truncated write, or a line lost in transfer.
        1 => {
            let at = rng.random_range(0..bytes.len());
            // `random_range` panics on an empty range, and `1..1` is empty --
            // which happens whenever the cut lands on the last byte. The fuzz
            // harness crashing is not a finding about the parser.
            let most = 16.min(bytes.len() - at);
            let len = if most <= 1 {
                1
            } else {
                rng.random_range(1..most)
            };
            bytes.drain(at..at + len);
        }
        // Insert a run of digits, which is how a number becomes enormous.
        2 => {
            let at = rng.random_range(0..bytes.len());
            let len = rng.random_range(1..12);
            for i in 0..len {
                bytes.insert(at + i, rng.random_range(b'0'..=b'9'));
            }
        }
        // Truncate. A full disk, or a killed post-processor.
        3 => {
            let at = rng.random_range(0..bytes.len());
            bytes.truncate(at);
        }
        // Duplicate a line, which subprogram handling could loop on.
        4 => {
            let text: String = String::from_utf8_lossy(&bytes).into_owned();
            let lines: Vec<&str> = text.lines().collect();
            if !lines.is_empty() {
                let at = rng.random_range(0..lines.len());
                let mut out = lines.clone();
                out.insert(at, lines[at]);
                return out.join("\n");
            }
        }
        // A byte outside ASCII, since NC files travel through text editors.
        _ => {
            let at = rng.random_range(0..bytes.len());
            bytes[at] = rng.random_range(128u8..=255);
        }
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

#[test]
#[ignore = "slow; run nightly with --ignored"]
fn mutations_of_valid_programs_never_panic_or_produce_a_nan() {
    const ROUNDS: usize = 40_000;
    let mut rng = StdRng::seed_from_u64(FUZZ_SEED);
    let options = ParseOptions::default();

    let mut parsed = 0usize;
    let mut refused = 0usize;
    let mut segments = 0usize;

    for round in 0..ROUNDS {
        let seed = SEEDS[round % SEEDS.len()];
        // Layer up to three mutations, so the input drifts further from valid
        // than a single edit would take it.
        let mut text = seed.to_owned();
        for _ in 0..rng.random_range(1..4) {
            text = mutate(&mut rng, &text);
        }

        match parse(&text, "fuzz", &options, None) {
            Ok((path, _, _)) => {
                parsed += 1;
                segments += path.segments.len();
                for (index, segment) in path.segments.iter().enumerate() {
                    assert!(
                        segment.start.is_finite() && segment.end.is_finite(),
                        "round {round}: segment {index} carries a non-finite coordinate.\n\
                         Orientation::from_determinant panics on NaN in release by design, \
                         so this would abort the process downstream rather than misbehave.\n\
                         input was:\n{text}"
                    );
                    if let Some(arc) = &segment.arc {
                        assert!(
                            arc.is_finite(),
                            "round {round}: segment {index} has a non-finite arc\n{text}"
                        );
                    }
                }
                // Contiguity holds even for nonsense input, because it is a
                // property of how segments are built rather than of the file.
                for pair in path.segments.windows(2) {
                    assert_eq!(
                        pair[0].end, pair[1].start,
                        "round {round}: a mutated file broke contiguity\n{text}"
                    );
                }
            }
            Err(error) => {
                refused += 1;
                // An error the user cannot place is nearly worthless, even for
                // a corrupted file.
                assert!(
                    !error.to_string().is_empty(),
                    "round {round}: an error with no message"
                );
            }
        }
    }

    eprintln!(
        "{ROUNDS} mutations: {parsed} parsed ({segments} segments), {refused} refused, 0 panics"
    );
    assert!(
        parsed > 0,
        "every mutation was refused; the fuzz is not reaching the parser"
    );
    assert!(
        refused > 0,
        "no mutation was refused; the fuzz is too gentle"
    );
}

#[test]
#[ignore = "slow; run nightly with --ignored"]
fn deeply_nested_and_pathological_input_terminates() {
    // Not mutations but constructions: the shapes that would hang a parser
    // rather than crash it.
    let options = ParseOptions::default();

    let cases = [
        // A subprogram calling itself, which must hit the depth cap.
        (
            "self-recursive subprogram",
            "M98 P100\nM30\nO100\nM98 P100\nM99\n".to_owned(),
        ),
        // Mutual recursion, which a naive depth counter on one name would miss.
        (
            "mutually recursive subprograms",
            "M98 P100\nM30\nO100\nM98 P200\nM99\nO200\nM98 P100\nM99\n".to_owned(),
        ),
        // A very large repeat count on a subprogram.
        (
            "huge repeat count",
            "G21 G90 G0 X0.\nM98 P100 L999999\nM30\nO100\nM99\n".to_owned(),
        ),
        // A cycle whose peck depth is tiny against its depth: the loop must
        // terminate on the depth rather than on the count.
        (
            "microscopic peck",
            "G21 G90 G0 X0. Y0. Z10.\nF250.\nG98 G83 X10. Y10. Z-100. R2. Q0.0001\nG80\nM30\n"
                .to_owned(),
        ),
        // A peck of zero, which must not divide or loop forever.
        (
            "zero peck",
            "G21 G90 G0 X0. Y0. Z10.\nF250.\nG98 G83 X10. Y10. Z-5. R2. Q0.\nG80\nM30\n".to_owned(),
        ),
        // Many blocks, to be sure nothing is quadratic in a way that matters.
        (
            "many blocks",
            std::iter::once("G21 G90 G0 X0. Y0.\nF250.\n".to_owned())
                .chain((0..20_000).map(|i| format!("G1 X{}.5\n", i % 100)))
                .chain(std::iter::once("M30\n".to_owned()))
                .collect::<String>(),
        ),
    ];

    for (name, text) in cases {
        // The assertion is simply that this returns at all.
        match parse(&text, name, &options, None) {
            Ok((path, _, _)) => {
                eprintln!("{name}: parsed, {} segments", path.segments.len());
                assert!(path.segments.iter().all(|s| s.start.is_finite()));
            }
            Err(error) => eprintln!("{name}: refused, {error}"),
        }
    }
}
