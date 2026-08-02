// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

#![forbid(unsafe_code)]

//! Command-line front end for Chipbreaker.
//!
//! Everything the engine does must be reachable from here. There is no GUI in
//! the core and there will not be one; the eventual browser demo is a consumer
//! of the library, never a part of it.

mod mesh;
mod report;

use std::io::Write;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::{Parser, Subcommand, ValueEnum};

use report::Environment;

/// Chipbreaker: material-removal simulation and machining verification.
#[derive(Debug, Parser)]
#[command(name = "chipbreaker", version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Run every deterministic self-test suite and report the results.
    ///
    /// Exits 0 when every suite passes and 1 otherwise. The JSON report's
    /// `results` section is bit-identical across runs, platforms and the WASM
    /// build; the `environment` section is not, and is excluded from the hash.
    Selftest {
        /// Output format.
        #[arg(long, value_enum, default_value_t = ReportFormat::Text)]
        report: ReportFormat,
        /// Write the report to a file instead of standard output.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Inspect, validate and ray-cast triangle meshes.
    Mesh {
        #[command(subcommand)]
        command: mesh::MeshCommand,
    },
    /// Print version information.
    Version {
        /// Emit JSON instead of a single line of text.
        #[arg(long)]
        json: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum ReportFormat {
    /// Human-readable summary.
    Text,
    /// Machine-readable, with a `results` section that is canonically hashed.
    Json,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.command {
        Command::Selftest { report, out } => run_selftest(report, out.as_deref()),
        Command::Mesh { command } => run_mesh(&command),
        Command::Version { json } => {
            print_version(json);
            ExitCode::SUCCESS
        }
    }
}

fn run_selftest(format: ReportFormat, out: Option<&std::path::Path>) -> ExitCode {
    // The clock starts here and its reading goes only into `environment`. If it
    // ever reaches the hashed section, every CI run disagrees with every other
    // one and the failure looks like a determinism bug.
    let started = Instant::now();
    let results = chipbreaker_core::selftest::run();
    let env = Environment::collect(started.elapsed());

    let rendered = match format {
        ReportFormat::Text => report::to_text(&results, &env),
        ReportFormat::Json => report::to_json(&results, &env),
    };

    if let Some(path) = out {
        if let Err(e) = std::fs::write(path, &rendered) {
            eprintln!("chipbreaker: could not write {}: {e}", path.display());
            return ExitCode::FAILURE;
        }
        // Still say something on stdout, so a CI log is not silent.
        println!(
            "wrote {} ({} suites, {} cases, {} failures)",
            path.display(),
            results.suites.len(),
            results.total_cases(),
            results.total_failures()
        );
        println!("results hash: {}", results.digest);
    } else {
        print!("{rendered}");
        let _ = std::io::stdout().flush();
    }

    if results.passed() {
        ExitCode::SUCCESS
    } else {
        // Name the specific suite and case on stderr, so a failing CI job is
        // diagnosable from the log alone.
        eprintln!();
        for suite in &results.suites {
            for failure in &suite.failures {
                eprintln!("FAIL {}: {} — {}", suite.name, failure.case, failure.detail);
            }
        }
        eprintln!(
            "\n{} of {} cases failed across {} suites",
            results.total_failures(),
            results.total_cases(),
            results.suites.iter().filter(|s| !s.passed()).count()
        );
        ExitCode::FAILURE
    }
}

fn run_mesh(command: &mesh::MeshCommand) -> ExitCode {
    let as_json = match command {
        mesh::MeshCommand::Inspect(i)
        | mesh::MeshCommand::Validate { input: i }
        | mesh::MeshCommand::Convert { input: i, .. }
        | mesh::MeshCommand::Bvh { input: i, .. }
        | mesh::MeshCommand::Raycast { input: i, .. }
        | mesh::MeshCommand::Parity { input: i, .. } => i.json,
    };
    let (outcome, elapsed) = mesh::timed(|| mesh::run(command));
    match outcome {
        Ok((results, text, ok)) => {
            print!("{}", mesh::render(&results, &text, elapsed, as_json));
            let _ = std::io::stdout().flush();
            if ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Err(message) => {
            eprintln!("chipbreaker: {message}");
            ExitCode::FAILURE
        }
    }
}

fn print_version(json: bool) {
    let name = env!("CARGO_PKG_NAME");
    let version = env!("CARGO_PKG_VERSION");
    let target = env!("CHIPBREAKER_TARGET");
    let rustc = env!("CHIPBREAKER_RUSTC");
    if json {
        let value = serde_json::json!({
            "core_version": chipbreaker_core::VERSION,
            "encoding_version": chipbreaker_core::CANONICAL_ENCODING_VERSION,
            "name": name,
            "rustc": rustc,
            "target": target,
            "version": version,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&value)
                .unwrap_or_else(|e| unreachable!("version JSON is always serializable: {e}"))
        );
    } else {
        println!("{name} {version} ({target}, {rustc})");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_definition_is_valid() {
        Cli::command().debug_assert();
    }

    #[test]
    fn selftest_parses_with_and_without_options() {
        let cli = Cli::try_parse_from(["chipbreaker", "selftest"]).expect("bare selftest");
        assert!(matches!(
            cli.command,
            Command::Selftest {
                report: ReportFormat::Text,
                out: None
            }
        ));

        let cli = Cli::try_parse_from([
            "chipbreaker",
            "selftest",
            "--report",
            "json",
            "--out",
            "r.json",
        ])
        .expect("selftest with options");
        match cli.command {
            Command::Selftest { report, out } => {
                assert_eq!(report, ReportFormat::Json);
                assert_eq!(out, Some(PathBuf::from("r.json")));
            }
            other => panic!("wrong subcommand: {other:?}"),
        }
    }

    #[test]
    fn mesh_subcommands_parse_and_require_units() {
        let cli = Cli::try_parse_from([
            "chipbreaker",
            "mesh",
            "inspect",
            "part.stl",
            "--units",
            "in",
        ])
        .expect("mesh inspect");
        assert!(matches!(cli.command, Command::Mesh { .. }));

        // The central CLI rule: STL and OBJ carry no units, so one must be given.
        assert!(
            Cli::try_parse_from(["chipbreaker", "mesh", "inspect", "part.stl"]).is_err(),
            "--units must be mandatory"
        );
        assert!(
            Cli::try_parse_from([
                "chipbreaker",
                "mesh",
                "inspect",
                "part.stl",
                "--units",
                "furlong"
            ])
            .is_err(),
            "unknown units must be rejected rather than defaulted"
        );

        for sub in ["validate", "bvh", "parity"] {
            assert!(
                Cli::try_parse_from(["chipbreaker", "mesh", sub, "p.stl", "--units", "mm"]).is_ok(),
                "{sub} should parse"
            );
        }
        assert!(
            Cli::try_parse_from([
                "chipbreaker",
                "mesh",
                "raycast",
                "p.stl",
                "--units",
                "mm",
                "--origin",
                "0,0,-1",
                "--dir",
                "0,0,1",
            ])
            .is_ok()
        );
        // A malformed vector is a parse error, not a silent zero.
        assert!(
            Cli::try_parse_from([
                "chipbreaker",
                "mesh",
                "raycast",
                "p.stl",
                "--units",
                "mm",
                "--origin",
                "0,0",
                "--dir",
                "0,0,1",
            ])
            .is_err()
        );
    }

    #[test]
    fn version_parses() {
        let cli = Cli::try_parse_from(["chipbreaker", "version", "--json"]).expect("version");
        assert!(matches!(cli.command, Command::Version { json: true }));
        let cli = Cli::try_parse_from(["chipbreaker", "version"]).expect("version");
        assert!(matches!(cli.command, Command::Version { json: false }));
    }

    #[test]
    fn unknown_input_is_rejected() {
        assert!(
            Cli::try_parse_from(["chipbreaker"]).is_err(),
            "a subcommand is required"
        );
        assert!(Cli::try_parse_from(["chipbreaker", "polish"]).is_err());
        assert!(
            Cli::try_parse_from(["chipbreaker", "selftest", "--report", "xml"]).is_err(),
            "report format is a closed set"
        );
    }
}
