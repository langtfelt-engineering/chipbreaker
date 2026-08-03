// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `chipbreaker path` subcommands.
//!
//! Same convention as `mesh` and `tool`: a deterministic `results` section that
//! is canonically hashed, and an `environment` section carrying timings that is
//! not.
//!
//! `path bounds` exists as a verb of its own because U5 sizes its dexel field
//! from the toolpath's extents, and wants that number before it has written any
//! of itself.

use std::path::PathBuf;

use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::tool::ToolLibrary;
use chipbreaker_core::toolpath::RapidPath;
use chipbreaker_core::toolpath::Toolpath;
use chipbreaker_gcode::diag::Diagnostics;
use chipbreaker_gcode::modal::Units;
use chipbreaker_gcode::resolve::{ParseOptions, ParseStats, parse};
use clap::{Args, Subcommand};
use serde_json::{Value, json};

/// Options shared by every `path` subcommand.
#[derive(Debug, Args)]
pub struct Input {
    /// NC file to read.
    pub file: PathBuf,
    /// Tool library, for resolving `T` and `G43 H` numbers.
    #[arg(long, value_name = "FILE")]
    pub tools: Option<PathBuf>,
    /// Units assumed before the program says otherwise.
    #[arg(long, value_parser = parse_units, default_value = "mm")]
    pub units: Units,
    /// Arc radius mismatch tolerance, in millimetres.
    #[arg(long, default_value_t = chipbreaker_gcode::arcs::DEFAULT_ARC_TOLERANCE)]
    pub arc_tolerance: f64,
    /// How rapids are represented in the IR.
    #[arg(long, value_parser = parse_rapid_path, default_value = "linear")]
    pub rapid_path: RapidPath,
    /// Treat an axis word with no decimal point as this many units.
    ///
    /// Without it such a word is refused, because `X10` means 0.010 mm on a
    /// legacy control and 10 mm everywhere else.
    #[arg(long, value_name = "VALUE")]
    pub legacy_increment: Option<f64>,
    /// Skip blocks marked with a leading `/`.
    #[arg(long)]
    pub skip_optional_blocks: bool,
    /// G73's chip-break retract distance, in millimetres.
    ///
    /// A machine parameter, absent from the NC file. Without it G73 expands as a
    /// straight plunge and the omission is counted in the IR header, so a
    /// collision check can refuse to certify a path it knows is incomplete.
    #[arg(long, value_name = "MM")]
    pub chip_break_clearance: Option<f64>,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

/// Parses `--units`.
///
/// # Errors
/// Returns a message listing the accepted names.
pub fn parse_units(s: &str) -> Result<Units, String> {
    match s.to_ascii_lowercase().as_str() {
        "mm" | "millimetre" | "millimeter" | "metric" => Ok(Units::Millimetres),
        "in" | "inch" | "imperial" => Ok(Units::Inches),
        other => Err(format!("unknown unit `{other}`; use mm or inch")),
    }
}

/// Parses `--rapid-path`.
///
/// # Errors
/// Returns a message listing the accepted names.
pub fn parse_rapid_path(s: &str) -> Result<RapidPath, String> {
    match s.to_ascii_lowercase().as_str() {
        "linear" => Ok(RapidPath::Linear),
        "dogleg" => Ok(RapidPath::Dogleg),
        other => Err(format!(
            "unknown rapid path `{other}`; use linear or dogleg. \
             dogleg is the conservative choice for collision checking"
        )),
    }
}

/// `chipbreaker path ...`
#[derive(Debug, Subcommand)]
pub enum PathCommand {
    /// Parse a program and report what it contains.
    Parse {
        #[command(flatten)]
        input: Input,
        /// Include per-stage counts.
        #[arg(long)]
        stats: bool,
    },
    /// Write every motion segment, one JSON object per line.
    Dump {
        #[command(flatten)]
        input: Input,
        /// Output format. Only `jsonl` is defined.
        #[arg(long, default_value = "jsonl")]
        format: String,
    },
    /// Report diagnostics without producing a toolpath.
    Lint {
        #[command(flatten)]
        input: Input,
        /// Treat warnings as errors.
        #[arg(long)]
        strict: bool,
    },
    /// Show what canned cycles expanded to, for diffing against longhand.
    Expand {
        #[command(flatten)]
        input: Input,
        /// Show only the segments that came from a cycle.
        #[arg(long)]
        cycles_only: bool,
    },
    /// Report the workspace extents, which is what U5 sizes stock from.
    Bounds {
        #[command(flatten)]
        input: Input,
    },
}

impl PathCommand {
    /// The shared options, whichever variant this is.
    #[must_use]
    pub fn input(&self) -> &Input {
        match self {
            Self::Parse { input, .. }
            | Self::Dump { input, .. }
            | Self::Lint { input, .. }
            | Self::Expand { input, .. }
            | Self::Bounds { input } => input,
        }
    }
}

fn options_from(input: &Input, strict: bool) -> ParseOptions {
    ParseOptions {
        default_units: input.units,
        arc_tolerance: input.arc_tolerance,
        rapid_path: input.rapid_path,
        legacy_increment: input.legacy_increment,
        execute_block_skip: !input.skip_optional_blocks,
        chip_break_clearance: input.chip_break_clearance,
        strict,
        ..ParseOptions::default()
    }
}

fn load_tools(input: &Input) -> Result<Option<ToolLibrary>, String> {
    let Some(path) = &input.tools else {
        return Ok(None);
    };
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    ToolLibrary::from_json(&text)
        .map(Some)
        .map_err(|e| e.to_string())
}

fn read_program(input: &Input) -> Result<(String, String), String> {
    let text = std::fs::read_to_string(&input.file)
        .map_err(|e| format!("cannot read {}: {e}", input.file.display()))?;
    let name = input
        .file
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("program")
        .to_owned();
    Ok((text, name))
}

fn run_parse(input: &Input, strict: bool) -> Result<(Toolpath, Diagnostics, ParseStats), String> {
    let (text, name) = read_program(input)?;
    let tools = load_tools(input)?;
    parse(&text, &name, &options_from(input, strict), tools.as_ref()).map_err(|e| e.to_string())
}

fn diagnostics_json(diagnostics: &Diagnostics) -> Vec<Value> {
    diagnostics
        .warnings()
        .iter()
        .map(|w| {
            json!({
                "kind": w.kind(),
                "line": w.site().line,
                "message": w.to_string(),
            })
        })
        .collect()
}

fn segment_json(index: usize, segment: &chipbreaker_core::toolpath::MotionSegment) -> Value {
    let mut value = json!({
        "end_mm": segment.end.to_array(),
        "feed": {
            "mode": segment.feed.mode.as_str(),
            "value": segment.feed.value,
        },
        "index": index,
        "kind": segment.kind.as_str(),
        "length_mm": segment.length(),
        "source": {
            "block": segment.source.block,
            "cycle_step": if segment.source.is_from_cycle() {
                json!(segment.source.cycle_step)
            } else {
                Value::Null
            },
            "file": segment.source.file,
            "line": segment.source.line,
        },
        "start_mm": segment.start.to_array(),
        "tool": segment.tool,
    });
    if let Some(arc) = &segment.arc
        && let Some(map) = value.as_object_mut()
    {
        map.insert(
            "arc".to_owned(),
            json!({
                "center_mm": arc.center.to_array(),
                "form": arc.form.as_str(),
                "plane": arc.plane.as_str(),
                "radius_mm": arc.radius,
                "radius_residual_mm": arc.radius_residual,
                "sweep_rad": arc.sweep,
            }),
        );
    }
    value
}

/// Runs a `path` subcommand.
///
/// # Errors
/// Returns a message suitable for stderr.
#[allow(clippy::too_many_lines, reason = "one arm per verb, each short")]
pub fn run(command: &PathCommand) -> Result<(Value, String, bool), String> {
    match command {
        PathCommand::Parse { input, stats } => {
            let (path, diagnostics, counts) = run_parse(input, false)?;
            let hash = {
                let mut h = CanonicalHash::new();
                h.add(&path);
                h.finish().to_hex()
            };
            let bounds = path.tip_bounds();
            let mut results = json!({
                "command": "parse",
                "events": path.events.len(),
                "length_mm": path.length(),
                "program": path.header.program,
                "schema_version": path.header.schema_version,
                "segments": path.segment_count(),
                "toolpath_hash": hash,
                "tools_used": path.tools_used(),
                "unmodelled_retracts": path.header.unmodelled_retracts,
                "warnings": diagnostics.len(),
            });
            if *stats && let Some(map) = results.as_object_mut() {
                map.insert(
                    "stats".to_owned(),
                    json!({
                        "blocks": counts.blocks,
                        "cycle_segments": counts.cycle_segments,
                        "dropped_zero_length": counts.dropped_zero_length,
                        "segments": counts.segments,
                        "skipped_blocks": counts.skipped_blocks,
                        "subprogram_calls": counts.subprogram_calls,
                    }),
                );
            }
            let text = format!(
                "{} segments, {} events, {} mm of commanded motion\n\
                 tools    {:?}\n\
                 bounds   {:?} .. {:?} mm (tool tip)\n\
                 warnings {}\n\
                 hash     {hash}\n{}",
                path.segment_count(),
                path.events.len(),
                path.length(),
                path.tools_used(),
                bounds.min.to_array(),
                bounds.max.to_array(),
                diagnostics.len(),
                if path.header.unmodelled_retracts > 0 {
                    format!(
                        "\nNOTE  {} G73 cycle(s) expanded without their chip-break retract, \
                         so this\n      path is missing motion the machine makes. Pass \
                         --chip-break-clearance\n      to include it.\n",
                        path.header.unmodelled_retracts
                    )
                } else {
                    String::new()
                },
            );
            Ok((results, text, true))
        }

        PathCommand::Dump { input, format } => {
            if format != "jsonl" {
                return Err(format!("unknown format `{format}`; only jsonl is defined"));
            }
            let (path, _, _) = run_parse(input, false)?;
            let mut text = String::new();
            for (index, segment) in path.segments.iter().enumerate() {
                text.push_str(
                    &serde_json::to_string(&segment_json(index, segment)).unwrap_or_default(),
                );
                text.push('\n');
            }
            let results = json!({
                "command": "dump",
                "segments": path.segment_count(),
            });
            Ok((results, text, true))
        }

        PathCommand::Lint { input, strict } => {
            let outcome = run_parse(input, *strict);
            match outcome {
                Ok((_, diagnostics, _)) => {
                    let entries = diagnostics_json(&diagnostics);
                    let mut text = format!("{} warning(s)\n", diagnostics.len());
                    for warning in diagnostics.warnings() {
                        text.push_str(&format!("  {warning}\n"));
                    }
                    let results = json!({
                        "command": "lint",
                        "errors": Vec::<Value>::new(),
                        "warnings": entries,
                    });
                    Ok((results, text, true))
                }
                Err(message) => {
                    // A lint that cannot parse still reports, rather than
                    // refusing to say anything.
                    let results = json!({
                        "command": "lint",
                        "errors": [ { "message": message.clone() } ],
                        "warnings": Vec::<Value>::new(),
                    });
                    Ok((results, format!("error: {message}\n"), false))
                }
            }
        }

        PathCommand::Expand { input, cycles_only } => {
            let (path, _, counts) = run_parse(input, false)?;
            let chosen: Vec<(usize, &chipbreaker_core::toolpath::MotionSegment)> = path
                .segments
                .iter()
                .enumerate()
                .filter(|(_, s)| !*cycles_only || s.source.is_from_cycle())
                .collect();
            let mut text = format!(
                "{} of {} segments came from canned cycles\n",
                counts.cycle_segments,
                path.segment_count()
            );
            for (index, segment) in &chosen {
                text.push_str(&format!(
                    "  {index:>5}  line {:>5} step {:>3}  {:<7} {:?} -> {:?}\n",
                    segment.source.line,
                    if segment.source.is_from_cycle() {
                        segment.source.cycle_step.to_string()
                    } else {
                        "-".to_owned()
                    },
                    segment.kind.as_str(),
                    segment.start.to_array(),
                    segment.end.to_array(),
                ));
            }
            let results = json!({
                "command": "expand",
                "cycle_segments": counts.cycle_segments,
                "segments": chosen
                    .iter()
                    .map(|(i, s)| segment_json(*i, s))
                    .collect::<Vec<_>>(),
            });
            Ok((results, text, true))
        }

        PathCommand::Bounds { input } => {
            let (path, _, _) = run_parse(input, false)?;
            let tip = path.tip_bounds();
            let tools = load_tools(input)?;
            // The tip bounds are not the swept bounds: the tool has a body. U5
            // must expand by the largest radius in play or it will size a dexel
            // field that the tool reaches outside of.
            let radius = tools.as_ref().map_or(0.0, |library| {
                path.tools_used()
                    .iter()
                    .filter_map(|n| library.get(&n.to_string()))
                    .map(chipbreaker_core::tool::Tool::max_radius)
                    .fold(0.0f64, f64::max)
            });
            let swept = tip.expand(radius);
            let results = json!({
                "command": "bounds",
                "swept_max_mm": swept.max.to_array(),
                "swept_min_mm": swept.min.to_array(),
                "tip_max_mm": tip.max.to_array(),
                "tip_min_mm": tip.min.to_array(),
                "tool_radius_mm": radius,
            });
            let text = format!(
                "tip     {:?} .. {:?} mm\n\
                 swept   {:?} .. {:?} mm  (tip expanded by a {radius} mm tool radius)\n\
                 extent  {:?} mm\n",
                tip.min.to_array(),
                tip.max.to_array(),
                swept.min.to_array(),
                swept.max.to_array(),
                swept.extent().to_array(),
            );
            Ok((results, text, true))
        }
    }
}
