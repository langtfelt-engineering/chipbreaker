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
//! # T numbers resolve against the library
//!
//! A program says `T5`, so the library's primary key is the number and the name
//! is metadata. Unit 7 could not run a multi-tool program because Unit 3's
//! library had drifted to keying by name; the number is restored, and a job that
//! changes tools now simply works.
//!
//! `--tool` remains, overriding the resolution for every segment. It is for
//! answering "what would this program do with a different cutter", not for
//! papering over a library that is missing a number -- a missing number is an
//! error naming the tools that are present.

use std::path::PathBuf;

use chipbreaker_core::budget::Budget;
use chipbreaker_core::dexel::tri::TriDexelField;
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_batch_per_motion, split_runs};
use chipbreaker_core::sweep::cut::{CutScratch, CutStats, SweepMethod, distribution};
use chipbreaker_core::sweep::parallel::{Schedule, cut_all_parallel};
use chipbreaker_core::sweep::{LinearMove, Motion, SweepCase};
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
    /// Motions per batch traversal. `1` disables batching.
    ///
    /// A tuning knob and nothing else: the result is bit-identical at every
    /// value, which the test suite asserts across the corpus.
    #[arg(long, value_name = "N", default_value_t = DEFAULT_BATCH)]
    pub batch_size: usize,
    /// Replace arcs with chords, as many CAM posts do, instead of sweeping them.
    ///
    /// For comparing against a post-processed program, and for confirming that
    /// the native arc converges to the linearised one as the tolerance tightens.
    #[arg(long)]
    pub no_arc_native: bool,
    /// Chord tolerance for `--no-arc-native`, in millimetres.
    ///
    /// Defaults to the sweep tolerance, so `--no-arc-native` alone asks for the
    /// same accuracy by a different route.
    #[arg(long, value_name = "MM")]
    pub linearise_tol: Option<f64>,
    /// Refuse if the field grows past this while cutting, e.g. `512M`.
    ///
    /// Checked **as the job runs**, not only at the start. Cutting splits spans,
    /// so a field that fitted when built can exceed its budget once pockets are
    /// cut -- Unit 7 measured a rib that spilled every ray of one bundle. The
    /// refusal names the operation and the segment so the answer to "how far did
    /// it get" is in the message.
    #[arg(long, value_name = "BYTES", value_parser = crate::dexel::parse_bytes)]
    pub mem_limit: Option<u64>,
    /// Worker threads. `0` uses one per available core, `1` is sequential.
    ///
    /// **The result is identical at every value.** Thread count is recorded in
    /// the unhashed environment section of a report, never in the hashed
    /// results, so a report produced on a different machine still compares equal.
    #[arg(long, value_name = "N", default_value_t = 1)]
    pub threads: usize,
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

/// The tool each distinct `T` number in the program resolves to.
fn resolve_tools(
    args: &RunArgs,
    toolpath: &Toolpath,
) -> Result<std::collections::BTreeMap<u32, Profile>, String> {
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

    // An override applies to every segment, which is a question ("what would a
    // different cutter do here?") rather than a resolution.
    if let Some(id) = &args.tool {
        let profile = library
            .get(id)
            .map(|t| t.profile().clone())
            .ok_or_else(|| {
                format!(
                    "the library has no tool called {id:?}. It holds: {}",
                    library
                        .tools()
                        .iter()
                        .map(|t| format!("{} (T{})", t.id().as_str(), t.number()))
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            })?;
        return Ok(numbers.into_iter().map(|n| (n, profile.clone())).collect());
    }

    let mut resolved = std::collections::BTreeMap::new();
    for number in numbers {
        let tool = library.get_by_number(number).ok_or_else(|| {
            // Named, not numbered, because the person reading this is holding a
            // library file and needs to know what to add.
            format!(
                "the program uses T{number} and the library has no tool with that \
                 number. It holds: {}",
                library
                    .tools()
                    .iter()
                    .map(|t| format!("T{} = {}", t.number(), t.id().as_str()))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })?;
        resolved.insert(number, tool.profile().clone());
    }
    Ok(resolved)
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
    let linearise_tol = args.linearise_tol.unwrap_or(tolerance);
    if !linearise_tol.is_finite() || linearise_tol <= 0.0 {
        return Err(format!(
            "--linearise-tol must be a positive length in millimetres, got {linearise_tol}"
        ));
    }
    let text = std::fs::read_to_string(&args.path)
        .map_err(|e| format!("cannot read {}: {e}", args.path.display()))?;
    let name = args
        .path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("program");
    let (toolpath, diagnostics, _) =
        parse(&text, name, &ParseOptions::default(), None).map_err(|e| e.to_string())?;
    let profiles = resolve_tools(args, &toolpath)?;

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
    // One scratch per tool: `CutScratch` caches the profile's radial convexity,
    // so sharing one across tools would answer for the wrong cutter.
    let mut scratches: std::collections::BTreeMap<u32, CutScratch> = profiles
        .iter()
        .map(|(number, profile)| (*number, CutScratch::new(profile)))
        .collect();
    let mut totals = CutStats::default();
    let mut cases = [0u64; 6];
    let mut arcs_skipped = 0u64;

    // Built up front rather than cut as they are read, because batching needs to
    // see a run of motions at once. The tool number rides along so `split_runs`
    // can break a run before a tool change.
    let mut motions: Vec<Motion> = Vec::with_capacity(hi - lo);
    let mut motion_tools: Vec<u32> = Vec::with_capacity(hi - lo);
    let mut linearised_arcs = 0u64;
    let mut linearised_chords = 0u64;
    for (index, segment) in toolpath.segments[lo..hi].iter().enumerate() {
        let Some(motion) = segment_motion(segment) else {
            // An arc whose data the parser did not attach. Counted and reported
            // rather than quietly treated as its chord, which for a full circle
            // is no motion at all.
            arcs_skipped += 1;
            continue;
        };
        if !profiles.contains_key(&segment.tool) {
            return Err(format!(
                "segment {} uses T{} which did not resolve; this should have been \
                 caught before cutting started",
                lo + index,
                segment.tool
            ));
        }
        match (&motion, args.no_arc_native) {
            (Motion::Arc(arc), true) => {
                linearised_arcs += 1;
                for chord in arc.linearise(linearise_tol) {
                    linearised_chords += 1;
                    let chord = Motion::Linear(chord);
                    cases[case_index(chord.case())] += 1;
                    motions.push(chord);
                    motion_tools.push(segment.tool);
                }
            }
            _ => {
                cases[case_index(motion.case())] += 1;
                motions.push(motion);
                motion_tools.push(segment.tool);
            }
        }
    }

    // One slot per motion, summed once at the end in motion order. Chunking the
    // list and adding up the chunk totals would make the reported volume depend
    // on the batch size; see `sweep::batch`.
    let mut removed_per_motion = vec![[0.0f64; 3]; motions.len()];
    let runs = split_runs(&motions, &motion_tools, args.batch_size.max(1));
    for (run, (from, to)) in runs.iter().copied().enumerate() {
        let tool = motion_tools[from];
        let (Some(profile), Some(scratch)) = (profiles.get(&tool), scratches.get_mut(&tool)) else {
            return Err(format!("T{tool} did not resolve"));
        };
        let stats = if args.threads == 1 {
            cut_batch_per_motion(
                &mut field,
                profile,
                &motions[from..to],
                method,
                scratch,
                &mut removed_per_motion[from..to],
            )
        } else {
            // The parallel path owns its own batching and reduction, so it takes
            // the whole run and reports the removed volume itself; the caller's
            // per-motion slots stay zero for it and the totals below add the
            // volume once.
            let s = cut_all_parallel(
                &mut field,
                profile,
                &motions[from..to],
                method,
                args.batch_size.max(1),
                Schedule {
                    threads: args.threads,
                    ..Schedule::default()
                },
            );
            for slot in 0..3 {
                totals.removed_mm3[slot] += s.removed_mm3[slot];
            }
            s
        };
        totals.merge_without_volume(&stats);

        // The ceiling, checked as spill grows rather than only at the start.
        if let Some(limit) = args.mem_limit {
            Budget::bytes(limit)
                .check_growth(field.bytes() as u64, "cutting", (lo + from) as u64)
                .map_err(|e| e.to_string())?;
        }

        if args.progress && run % 100 == 0 {
            eprintln!(
                "  motion {from}/{}, {:.1}% of rays rejected",
                motions.len(),
                totals.rejection_rate() * 100.0
            );
        }
    }
    for slot in 0..3 {
        for per_motion in &removed_per_motion {
            totals.removed_mm3[slot] += per_motion[slot];
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
        if let Some(id) = args.tool.as_deref() {
            format!("{id} (overriding every T number)")
        } else {
            profiles
                .keys()
                .map(|n| format!("T{n}"))
                .collect::<Vec<_>>()
                .join(", ")
        },
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
        "cases     {} horizontal, {} plunge, {} ramp, {} arc, {} helix, {} stationary
",
        cases[1], cases[2], cases[3], cases[4], cases[5], cases[0]
    ));
    report.push_str(&format!(
        "batching  {} motions in {} run(s) of at most {}, bit-identical at every size
",
        motions.len(),
        runs.len(),
        args.batch_size.max(1),
    ));
    if args.no_arc_native {
        report.push_str(&format!(
            "LINEARISED {linearised_arcs} arc(s) -> {linearised_chords} chord(s) at {linearise_tol} mm. This is the post-processed program, NOT the arc.
"
        ));
    }
    if arcs_skipped > 0 {
        report.push_str(&format!(
            "SKIPPED   {arcs_skipped} arc segment(s) carried no arc data. This result \
             is NOT a simulation of the whole program.\n"
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
    // Split, per 1c: the worst bound belongs only to the rays that sub-stepped.
    let total_rays = totals.rays_exact + totals.rays_substepped;
    #[allow(clippy::cast_precision_loss, reason = "a percentage of counts")]
    let exact_share = if total_rays == 0 {
        100.0
    } else {
        totals.rays_exact as f64 / total_rays as f64 * 100.0
    };
    report.push_str(&format!(
        "sweep     {} of {total_rays} ray-cuts exact ({exact_share:.2}%), {} sub-stepped \
         over {} steps\n",
        totals.rays_exact, totals.rays_substepped, totals.substeps
    ));
    report.push_str(&format!(
        "          worst deviation bound {} mm, and it applies ONLY to the \
         sub-stepped ones\n",
        totals.worst_bound_mm
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
        "batching": {
            "runs": runs.len(),
            "size": args.batch_size.max(1),
            "motions": motions.len(),
        },
        "cases": {
            "arc": cases[4],
            "helix": cases[5],
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
        "linearised": if args.no_arc_native {
            json!({ "arcs": linearised_arcs, "chords": linearised_chords, "tolerance_mm": linearise_tol })
        } else {
            Value::Null
        },
        "removed_mm3": removed,
        "removed_mm3_per_bundle": totals.removed_mm3,
        "segments": { "from": lo, "to": hi, "total": toolpath.segments.len() },
        "sweep": {
            "exact_share": exact_share,
            "rays_exact": totals.rays_exact,
            "rays_substepped": totals.rays_substepped,
            "substeps": totals.substeps,
            "worst_bound_mm": totals.worst_bound_mm,
        },
        "written": written.map(|(p, n)| json!({ "bytes": n, "path": p })),
    });
    // An arc encountered is not a failure, but it is not a success either: the
    // exit code has to say the program was not fully simulated.
    Ok((results, report, arcs_skipped == 0))
}

/// Turns a parsed segment into a motion this unit can sweep.
///
/// `None` only when an arc segment carries no arc data, which the parser should
/// never produce -- but the caller counts and reports it rather than assuming,
/// because silently treating an arc as its chord deletes a full circle entirely.
fn segment_motion(segment: &chipbreaker_core::toolpath::MotionSegment) -> Option<Motion> {
    if !matches!(segment.kind, MotionKind::Arc | MotionKind::Helix) {
        return Some(Motion::Linear(LinearMove {
            start: segment.start,
            end: segment.end,
        }));
    }
    let data = segment.arc.as_ref()?;
    let [u, v, w] = data.plane.axes();
    let centre = data.center.to_array();
    let start = segment.start.to_array();
    let end = segment.end.to_array();
    Some(Motion::Arc(ArcMove {
        center: data.center,
        radius: data.radius,
        // The bearing of the start about the arc's own axis, in that plane's
        // own axis order -- which for `G18` is Z then X, not X then Z.
        start_angle: chipbreaker_core::transcendental::atan2(
            start[v] - centre[v],
            start[u] - centre[u],
        ),
        sweep: data.sweep,
        z: start[w],
        rise: end[w] - start[w],
        plane: data.plane,
    }))
}

const fn case_index(case: SweepCase) -> usize {
    match case {
        SweepCase::Stationary => 0,
        SweepCase::Horizontal => 1,
        SweepCase::Plunge => 2,
        SweepCase::Ramp => 3,
        SweepCase::Arc => 4,
        SweepCase::Helix => 5,
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
