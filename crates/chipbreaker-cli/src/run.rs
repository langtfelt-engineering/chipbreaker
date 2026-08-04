// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! `chipbreaker run` and `chipbreaker cut-stat`: simulating material removal.
//!
//! # `--segment-range` is a debugging tool and is meant to be used
//!
//! When segment 41,332 of a job produces a wrong result, the only tractable next
//! step is running that segment alone against the same stock. Splitting a path
//! is bit-identical at segment boundaries — the metamorphic suite asserts it —
//! so a range really does reproduce what the full job did to those segments,
//! rather than something merely similar.
//!
//! # `--reference` is the ground truth, exposed
//!
//! The dense sub-stepping reference is what every fast path in `sweep` is
//! differential-tested against. Putting it behind a flag means a customer who
//! doubts a result can reproduce it the slow, obvious way on their own geometry,
//! rather than taking the fast path on trust.
//!
//! # T numbers are not yet bound to tools
//!
//! The toolpath IR carries the `T` number a segment was programmed with, and the
//! tool library is keyed by name. Nothing in the project maps between them,
//! because that binding belongs to a machine setup that no unit has defined yet.
//! Until one does, `--tool` names the tool explicitly and a job that changes
//! tools is refused rather than silently simulated with the wrong one.

use std::path::PathBuf;

use chipbreaker_core::dexel::tri::TriDexelField;
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::sweep::cut::{CutScratch, CutStats, SweepMethod, cut_tri, distribution};
use chipbreaker_core::sweep::{LinearMove, SweepCase};
use chipbreaker_core::tool::{Profile, ToolLibrary};
use chipbreaker_core::toolpath::{MotionKind, Toolpath};
use chipbreaker_gcode::resolve::{ParseOptions, parse};
use clap::Args;
use serde_json::{Value, json};

/// Default sweep tolerance as a fraction of the cell size.
///
/// **Not an absolute figure, and that is the point.** The sweep deviation and
/// the lattice sampling are independent error sources, and resolving the sweep
/// far below what the lattice can represent is pure waste: the sweep error
/// disappears behind the sampling error long before it reaches a micrometre.
///
/// Measured on a 15 mm rapid over a 0.4 mm lattice: a fixed 1 um tolerance asks
/// for 7,500 sub-steps per ray and 8.9 million across the segment. A tenth of a
/// cell asks for 188 -- forty times fewer -- and the difference is invisible in
/// the field, because a tenth of a cell is an order below the sampling error the
/// lattice already carries.
///
/// A customer who wants an absolute bound passes `--max-swept-error` and gets
/// exactly that.
const SWEPT_ERROR_PER_CELL: f64 = 0.1;

/// `chipbreaker run ...`
#[derive(Debug, Args)]
pub struct RunArgs {
    /// Stock field to cut, as `.tdx` or `.dexel`.
    #[arg(long, value_name = "FILE")]
    pub stock: PathBuf,
    /// NC program to simulate.
    #[arg(long, value_name = "FILE")]
    pub path: PathBuf,
    /// Tool library.
    #[arg(long, value_name = "FILE")]
    pub tools: Option<PathBuf>,
    /// Which tool to cut with, by library id.
    ///
    /// Required when a library is given: see the module note on `T` numbers.
    #[arg(long, value_name = "ID")]
    pub tool: Option<String>,
    /// Where to write the resulting field.
    #[arg(long, value_name = "FILE")]
    pub out: Option<PathBuf>,
    /// Print progress as segments are consumed.
    #[arg(long)]
    pub progress: bool,
    /// Simulate only segments `A:B`, half-open, for debugging one bad segment.
    #[arg(long, value_name = "A:B", value_parser = parse_range)]
    pub segment_range: Option<(usize, usize)>,
    /// Maximum swept-volume deviation, in millimetres.
    ///
    /// Defaults to a tenth of the stock's cell size, because a sweep resolved far
    /// below the lattice is work nobody can see. See `SWEPT_ERROR_PER_CELL`.
    #[arg(long, value_name = "MM")]
    pub max_swept_error: Option<f64>,
    /// Use the dense sub-stepping reference instead of the closed forms.
    #[arg(long)]
    pub reference: bool,
    /// Sub-steps per motion when `--reference` is given.
    #[arg(long, default_value_t = 64)]
    pub substeps: u32,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

/// `chipbreaker cut-stat ...`
#[derive(Debug, Args)]
pub struct CutStatArgs {
    /// The field to describe.
    pub file: PathBuf,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

fn parse_range(s: &str) -> Result<(usize, usize), String> {
    let (a, b) = s
        .split_once(':')
        .ok_or_else(|| format!("expected A:B, such as 41332:41333; got {s:?}"))?;
    let parse = |t: &str, what: &str| {
        t.trim()
            .parse::<usize>()
            .map_err(|e| format!("{what} of the segment range: {e}"))
    };
    let (a, b) = (parse(a, "start")?, parse(b, "end")?);
    if a >= b {
        return Err(format!(
            "the segment range is half-open, so the start must be below the end; got {a}:{b}"
        ));
    }
    Ok((a, b))
}

fn read_stock(file: &std::path::Path) -> Result<TriDexelField, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
    match dexel_io::detect(&bytes) {
        Some(FieldFormat::Tri) => {
            dexel_io::tri_from_bytes(&bytes).map_err(|e| format!("{}: {e}", file.display()))
        }
        Some(FieldFormat::Single) => Err(format!(
            "{} is a single-bundle .dexel field. Cutting needs all three bundles, \
             because a surface parallel to the only bundle would be sampled by \
             nothing. Rebuild it with `dexel build --axes xyz`.",
            file.display()
        )),
        None => Err(format!(
            "{} is not a Chipbreaker field file",
            file.display()
        )),
    }
}

/// Resolves the one tool a job may use.
fn resolve_tool(args: &RunArgs, toolpath: &Toolpath) -> Result<Profile, String> {
    let Some(library_path) = &args.tools else {
        return Err(
            "--tools is required: cutting needs a tool, and guessing one would be \
             worse than refusing"
                .to_owned(),
        );
    };
    let text = std::fs::read_to_string(library_path)
        .map_err(|e| format!("cannot read {}: {e}", library_path.display()))?;
    let library = ToolLibrary::from_json(&text).map_err(|e| e.to_string())?;

    // Every distinct T number the program actually cuts with.
    let mut numbers: Vec<u32> = toolpath.segments.iter().map(|s| s.tool).collect();
    numbers.sort_unstable();
    numbers.dedup();

    let Some(id) = &args.tool else {
        return Err(format!(
            "--tool is required. This program uses T{numbers:?}, and nothing in the \
             project maps a T number to a library entry -- that binding belongs to a \
             machine setup no unit has defined yet. Name the tool explicitly. The \
             library holds: {}",
            library
                .tools()
                .iter()
                .map(|t| t.id().as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    };
    if numbers.len() > 1 {
        return Err(format!(
            "this program changes tools (T{numbers:?}) but --tool names only {id}. \
             Simulating every segment with one tool would produce a confident wrong \
             answer, so this is refused until T numbers are bound to tools."
        ));
    }
    library
        .get(id)
        .map(|t| t.profile().clone())
        .ok_or_else(|| format!("the library has no tool called {id:?}"))
}

/// Runs `chipbreaker run`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn run(args: &RunArgs) -> Result<(Value, String, bool), String> {
    if let Some(tolerance) = args.max_swept_error
        && (!tolerance.is_finite() || tolerance <= 0.0)
    {
        return Err(format!(
            "--max-swept-error must be a positive length in millimetres, got {tolerance}"
        ));
    }

    let mut field = read_stock(&args.stock)?;
    // Derived from the stock rather than fixed, unless the caller insisted.
    let spacing = field
        .bundles()
        .next()
        .map_or(1.0, |(_, b)| b.lattice().spacing());
    let tolerance = args
        .max_swept_error
        .unwrap_or(spacing * SWEPT_ERROR_PER_CELL);
    let text = std::fs::read_to_string(&args.path)
        .map_err(|e| format!("cannot read {}: {e}", args.path.display()))?;
    let name = args
        .path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("program");
    let (toolpath, diagnostics, _) =
        parse(&text, name, &ParseOptions::default(), None).map_err(|e| e.to_string())?;
    let profile = resolve_tool(args, &toolpath)?;

    let method = if args.reference {
        SweepMethod::Reference {
            steps: args.substeps.max(1),
        }
    } else {
        SweepMethod::Analytic { tolerance }
    };

    let (lo, hi) = args.segment_range.unwrap_or((0, toolpath.segments.len()));
    let hi = hi.min(toolpath.segments.len());
    if lo >= hi {
        return Err(format!(
            "the segment range {lo}:{hi} selects nothing; the program has {} segments",
            toolpath.segments.len()
        ));
    }

    let before = field.volume();
    let mut scratch = CutScratch::new(&profile);
    let mut totals = CutStats::default();
    let mut cases = [0u64; 4];
    let mut arcs_skipped = 0u64;

    for (index, segment) in toolpath.segments[lo..hi].iter().enumerate() {
        // Arcs are Unit 8. Counted and reported rather than silently treated as
        // their chord, which would remove the wrong material and look fine.
        if matches!(segment.kind, MotionKind::Arc | MotionKind::Helix) {
            arcs_skipped += 1;
            continue;
        }
        let motion = LinearMove {
            start: segment.start,
            end: segment.end,
        };
        cases[case_index(motion.case())] += 1;
        let stats = cut_tri(&mut field, &profile, &motion, method, &mut scratch);
        totals.merge(&stats);

        if args.progress && index % 500 == 0 {
            eprintln!(
                "  segment {}/{}, {:.1}% of rays rejected",
                lo + index,
                hi,
                totals.rejection_rate() * 100.0
            );
        }
    }

    let removed = before - field.volume();
    let mut written = None;
    if let Some(path) = &args.out {
        let bytes = dexel_io::tri_to_bytes(&field).map_err(|e| e.to_string())?;
        std::fs::write(path, &bytes)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        written = Some((path.display().to_string(), bytes.len()));
    }

    let mut digest = CanonicalHash::new();
    digest.add(&field);
    let digest = digest.finish().to_hex();

    let mut report = format!(
        "program   {name}, segments {lo}..{hi} of {}\n\
         tool      {}\n\
         method    {}\n",
        toolpath.segments.len(),
        args.tool.as_deref().unwrap_or("?"),
        if args.reference {
            format!("dense reference, {} sub-steps per motion", args.substeps)
        } else {
            format!(
                "closed form where exact, otherwise bounded at {tolerance} mm{}",
                if args.max_swept_error.is_some() {
                    ""
                } else {
                    " (a tenth of the cell size)"
                }
            )
        },
    );
    report.push_str(&format!(
        "cases     {} horizontal, {} plunge, {} ramp, {} stationary\n",
        cases[1], cases[2], cases[3], cases[0]
    ));
    if arcs_skipped > 0 {
        report.push_str(&format!(
            "SKIPPED   {arcs_skipped} arc or helix segment(s): arcs are Unit 8. This \
             result is NOT a simulation of the whole program.\n"
        ));
    }
    report.push_str(&format!(
        "removed   {removed} mm^3 (mean of three bundles; per bundle {:?})\n",
        totals.removed_mm3
    ));
    report.push_str(&format!(
        "rays      {} tested, {} rejected ({:.3}% rejection), {} changed\n",
        totals.rays_tested,
        totals.rays_rejected,
        totals.rejection_rate() * 100.0,
        totals.rays_changed,
    ));
    report.push_str(&format!(
        "sweep     {} sub-steps, worst deviation bound {} mm\n",
        totals.substeps, totals.worst_bound_mm
    ));
    report.push_str(&format!("digest    {digest}\n"));
    if let Some((path, bytes)) = &written {
        report.push_str(&format!(
            "wrote     {path} ({bytes} bytes, {:.2} MiB)\n",
            *bytes as f64 / (1024.0 * 1024.0)
        ));
    }
    if !diagnostics.is_empty() {
        report.push_str(&format!(
            "\n{} parser diagnostic(s); run `chipbreaker path resolve` to see them\n",
            diagnostics.len()
        ));
    }

    let results = json!({
        "arcs_skipped": arcs_skipped,
        "cases": {
            "horizontal": cases[1],
            "plunge": cases[2],
            "ramp": cases[3],
            "stationary": cases[0],
        },
        "command": "run",
        "digest": digest,
        "method": if args.reference { "reference" } else { "analytic" },
        "rays": {
            "changed": totals.rays_changed,
            "rejected": totals.rays_rejected,
            "rejection_rate": totals.rejection_rate(),
            "tested": totals.rays_tested,
        },
        "removed_mm3": removed,
        "removed_mm3_per_bundle": totals.removed_mm3,
        "segments": { "from": lo, "to": hi, "total": toolpath.segments.len() },
        "sweep": {
            "substeps": totals.substeps,
            "worst_bound_mm": totals.worst_bound_mm,
        },
        "written": written.map(|(p, n)| json!({ "bytes": n, "path": p })),
    });
    // An arc encountered is not a failure, but it is not a success either: the
    // exit code has to say the program was not fully simulated.
    Ok((results, report, arcs_skipped == 0))
}

const fn case_index(case: SweepCase) -> usize {
    match case {
        SweepCase::Stationary => 0,
        SweepCase::Horizontal => 1,
        SweepCase::Plunge => 2,
        SweepCase::Ramp => 3,
    }
}

/// Runs `chipbreaker cut-stat`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn cut_stat(args: &CutStatArgs) -> Result<(Value, String, bool), String> {
    let field = read_stock(&args.file)?;
    let mut text = format!(
        "volume    {} mm^3 (mean of three bundles)\n",
        field.volume()
    );
    for (axis, bundle) in field.bundles() {
        text.push_str(&format!(
            "bundle {}  {} rays, {} spans, {} spilled, {:.2} MiB, volume {} mm^3\n",
            axis.as_str(),
            bundle.arena().rays(),
            bundle.total_spans(),
            bundle.arena().spilled_rays(),
            bundle.arena().bytes() as f64 / (1024.0 * 1024.0),
            bundle.volume(),
        ));
    }
    text.push_str("\nspan distribution across all bundles\n");
    let d = distribution(&field);
    let rays: usize = d.values().sum();
    for (spans, count) in &d {
        #[allow(clippy::cast_precision_loss, reason = "a percentage of counts")]
        let share = *count as f64 / rays.max(1) as f64 * 100.0;
        text.push_str(&format!(
            "  {spans:>3} span(s)  {count:>10} rays  {share:>6.2}%\n"
        ));
    }
    text.push_str(
        "\nThe bundles will not agree on volume and are not meant to. Volume is a\n\
         construction diagnostic; `dexel deviation` is the accuracy metric (ADR 0005).\n",
    );

    let mut digest = CanonicalHash::new();
    digest.add(&field);
    let results = json!({
        "bytes": field.bytes(),
        "command": "cut-stat",
        "digest": digest.finish().to_hex(),
        "distribution": d.iter().map(|(k, v)| json!([k, v])).collect::<Vec<_>>(),
        "per_bundle": field
            .bundles()
            .map(|(axis, b)| json!({
                "axis": axis.as_str(),
                "bytes": b.arena().bytes(),
                "rays": b.arena().rays(),
                "spans": b.total_spans(),
                "spilled": b.arena().spilled_rays(),
                "volume_mm3": b.volume(),
            }))
            .collect::<Vec<_>>(),
        "volume_mm3": field.volume(),
    });
    Ok((results, text, true))
}
