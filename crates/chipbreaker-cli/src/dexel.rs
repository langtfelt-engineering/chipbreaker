// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `chipbreaker dexel` subcommands.
//!
//! A dexel field is the structure the rest of the product operates on, and it is
//! a binary blob of several hundred megabytes that nobody can read. ADR 0004
//! accepted that deliberately — the alternative was making the determinism
//! contract depend on float formatting — on the understanding that debuggability
//! becomes a tooling problem. This module is that tooling.
//!
//! `stat` answers "what is in this file", `slice` answers "what does it look
//! like there", and `volume` answers "how much material". `convergence` runs the
//! accuracy measurement so a customer can reproduce the published table on their
//! own geometry rather than taking ours on faith.

use std::path::PathBuf;

use chipbreaker_core::dexel::convergence::{
    ErrorModel, GAUSS_CIRCLE_EXPONENT, measure, standard_cases, standard_ratios,
};
use chipbreaker_core::dexel::{BuildOptions, BuildStats, DexelField, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use clap::{Args, Subcommand};
use serde_json::{Value, json};

use crate::mesh::Input;

/// Options shared by every subcommand that builds a field.
#[derive(Debug, Args)]
pub struct BuildArgs {
    #[command(flatten)]
    pub input: Input,
    /// Cell size, in millimetres.
    ///
    /// No default. Accuracy depends on the ratio of this to the smallest feature
    /// that matters, not on the number itself, so a default would be a guess
    /// about the customer's part.
    #[arg(long, value_name = "MM")]
    pub spacing: f64,
    /// Axis the rays run along.
    #[arg(long, default_value = "z", value_parser = parse_axis)]
    pub axis: Axis,
    /// Where the stock sits in machine coordinates, as `x,y,z` millimetres.
    #[arg(long, value_parser = parse_vec3)]
    pub at: Option<Vec3>,
    /// Extra room around the stock bounds, in millimetres.
    #[arg(long, default_value_t = 0.0, value_name = "MM")]
    pub margin: f64,
}

impl BuildArgs {
    fn options(&self) -> BuildOptions {
        BuildOptions {
            spacing: self.spacing,
            axis: self.axis,
            placement: self.at.map_or(Mat4::IDENTITY, Mat4::from_translation),
            margin: self.margin,
        }
    }
}

/// `chipbreaker dexel ...`
#[derive(Debug, Subcommand)]
pub enum DexelCommand {
    /// Build a field from a stock mesh and write it as `.dexel`.
    Build {
        #[command(flatten)]
        build: BuildArgs,
        /// Where to write the field.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Describe a `.dexel` file: lattice, occupancy, span distribution.
    Stat {
        /// The field to read.
        file: PathBuf,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Report the material volume of a field.
    Volume {
        /// A `.dexel` file to read.
        file: PathBuf,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Print one row of the lattice as text, so a field can be eyeballed.
    ///
    /// The answer to "the binary format is not human-readable". A slice shows
    /// where material starts and stops along each ray in a single row, which is
    /// enough to see a hole in the wrong place.
    Slice {
        /// A `.dexel` file to read.
        file: PathBuf,
        /// Which row of the lattice, along the first lattice axis.
        ///
        /// Defaults to the middle, which is where the interesting geometry
        /// usually is.
        #[arg(long)]
        row: Option<u32>,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Run the convergence measurement and print the accuracy table.
    Convergence {
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
}

impl DexelCommand {
    /// Whether this invocation asked for JSON.
    #[must_use]
    pub const fn json(&self) -> bool {
        match self {
            Self::Build { build, .. } => build.input.json,
            Self::Stat { json, .. }
            | Self::Volume { json, .. }
            | Self::Slice { json, .. }
            | Self::Convergence { json } => *json,
        }
    }
}

fn parse_axis(s: &str) -> Result<Axis, String> {
    Axis::from_name(s).ok_or_else(|| format!("expected x, y or z; got {s:?}"))
}

fn parse_vec3(s: &str) -> Result<Vec3, String> {
    let parts: Vec<&str> = s.split(',').collect();
    let [x, y, z] = parts.as_slice() else {
        return Err(format!("expected three comma-separated numbers, got {s:?}"));
    };
    let parse = |t: &str| t.trim().parse::<f64>().map_err(|e| format!("{t:?}: {e}"));
    Ok(Vec3::new(parse(x)?, parse(y)?, parse(z)?))
}

/// Runs a subcommand.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn run(command: &DexelCommand) -> Result<(Value, String, bool), String> {
    match command {
        DexelCommand::Build { build, out } => run_build(build, out.as_deref()),
        DexelCommand::Stat { file, .. } => run_stat(file),
        DexelCommand::Volume { file, .. } => run_volume(file),
        DexelCommand::Slice { file, row, .. } => run_slice(file, *row),
        DexelCommand::Convergence { .. } => run_convergence(),
    }
}

fn read_field(file: &std::path::Path) -> Result<DexelField, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
    dexel_io::from_bytes(&bytes).map_err(|e| format!("{}: {e}", file.display()))
}

fn digest_of(field: &DexelField) -> String {
    let mut h = CanonicalHash::new();
    h.add(field);
    h.finish().to_hex()
}

fn lattice_json(field: &DexelField) -> Value {
    let lattice = field.lattice();
    let [nx, ny] = lattice.counts();
    json!({
        "axis": lattice.axis().as_str(),
        "cell_area_mm2": lattice.cell_area(),
        "counts": [nx, ny],
        "origin_mm": lattice.origin().to_array(),
        "ray_count": lattice.ray_count(),
        "ray_length_mm": lattice.ray_length(),
        "spacing_mm": lattice.spacing(),
    })
}

fn run_build(
    args: &BuildArgs,
    out: Option<&std::path::Path>,
) -> Result<(Value, String, bool), String> {
    if !args.spacing.is_finite() || args.spacing <= 0.0 {
        return Err(format!(
            "--spacing must be a positive length in millimetres, got {}",
            args.spacing
        ));
    }
    let (mesh, mesh_summary) = crate::mesh::load(&args.input)?;
    let (field, stats) = DexelField::build(&mesh, &args.options()).map_err(|e| e.to_string())?;

    let mut written = None;
    if let Some(path) = out {
        let bytes = dexel_io::to_bytes(&field).map_err(|e| e.to_string())?;
        std::fs::write(path, &bytes)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        written = Some((path.display().to_string(), bytes.len()));
    }

    let text = format!(
        "{}\n\n{}{}",
        describe_lattice(&field),
        describe_occupancy(&field, Some(&stats)),
        written.as_ref().map_or_else(String::new, |(p, n)| format!(
            "\nwrote {p} ({n} bytes, {:.2} MiB)\n",
            *n as f64 / (1024.0 * 1024.0)
        ))
    );
    let results = json!({
        "command": "build",
        "digest": digest_of(&field),
        "lattice": lattice_json(&field),
        "mesh": mesh_summary,
        "occupancy": occupancy_json(&field, Some(&stats)),
        "written": written.map(|(p, n)| json!({ "bytes": n, "path": p })),
    });
    Ok((results, text, true))
}

fn run_stat(file: &std::path::Path) -> Result<(Value, String, bool), String> {
    let field = read_field(file)?;
    let text = format!(
        "{}\n\n{}\n{}",
        describe_lattice(&field),
        describe_occupancy(&field, None),
        describe_distribution(&field)
    );
    let results = json!({
        "command": "stat",
        "digest": digest_of(&field),
        "file": file.display().to_string(),
        "lattice": lattice_json(&field),
        "occupancy": occupancy_json(&field, None),
    });
    Ok((results, text, true))
}

fn run_volume(file: &std::path::Path) -> Result<(Value, String, bool), String> {
    let field = read_field(file)?;
    let volume = field.volume();
    let text = format!(
        "{volume} mm^3   ({:.6} cm^3)\n\n\
         Accuracy depends on the ratio of cell size to the smallest feature that\n\
         matters, not on the cell size alone. This field's cells are {} mm.\n",
        volume / 1000.0,
        field.lattice().spacing()
    );
    let results = json!({
        "command": "volume",
        "digest": digest_of(&field),
        "spacing_mm": field.lattice().spacing(),
        "volume_mm3": volume,
    });
    Ok((results, text, true))
}

fn run_slice(file: &std::path::Path, row: Option<u32>) -> Result<(Value, String, bool), String> {
    let field = read_field(file)?;
    let lattice = field.lattice();
    let [nx, ny] = lattice.counts();
    let row = row.unwrap_or(nx / 2);
    if row >= nx {
        return Err(format!(
            "row {row} is outside the lattice, which has {nx} rows along its first axis"
        ));
    }

    let mut text = format!(
        "row {row} of {nx}, {ny} rays along the second lattice axis, rays run along {}\n\n",
        lattice.axis().as_str()
    );
    let mut entries = Vec::new();
    for column in 0..ny {
        let ray = lattice.index(row, column);
        let spans = field.arena().get(ray);
        let origin = lattice.origin_of(row, column);
        let rendered = if spans.is_empty() {
            "empty".to_owned()
        } else {
            spans
                .iter()
                .map(|s| format!("[{:.4}, {:.4}]", s.t0, s.t1))
                .collect::<Vec<_>>()
                .join(" ")
        };
        text.push_str(&format!("  {column:>5}  {rendered}\n"));
        entries.push(json!({
            "column": column,
            "origin_mm": origin.to_array(),
            "spans": spans.iter().map(|s| json!([s.t0, s.t1])).collect::<Vec<_>>(),
        }));
    }

    let results = json!({
        "command": "slice",
        "digest": digest_of(&field),
        "rays": entries,
        "row": row,
    });
    Ok((results, text, true))
}

fn run_convergence() -> Result<(Value, String, bool), String> {
    let ratios = standard_ratios();
    let mut text = String::from(
        "Two error columns. Against the MESH isolates dexel sampling error and is\n\
         what the tests assert on; against the ANALYTIC solid adds tessellation\n\
         error and is the total distance from reality.\n\n",
    );
    let mut cases = Vec::new();
    let mut ok = true;

    for case in standard_cases() {
        let result = measure(&case, &ratios);
        text.push_str(&format!("=== {} ===\n", result.name));
        text.push_str(&format!(
            "  {:>8}  {:>10}  {:>12}  {:>12}\n",
            "h/R", "h (mm)", "vs mesh", "vs analytic"
        ));
        let mut samples = Vec::new();
        for sample in &result.samples {
            let analytic = sample
                .analytic_error()
                .map_or_else(|| "          --".to_owned(), |e| format!("{e:>12.3e}"));
            text.push_str(&format!(
                "  {:>8.5}  {:>10.4}  {:>12.3e}  {analytic}\n",
                sample.ratio,
                sample.spacing,
                sample.mesh_error()
            ));
            samples.push(json!({
                "analytic_error": sample.analytic_error(),
                "mesh_error": sample.mesh_error(),
                "rays": sample.rays,
                "ratio": sample.ratio,
                "signed_mesh_error": sample.signed_mesh_error(),
                "spacing_mm": sample.spacing,
                "volume_mm3": sample.measured,
            }));
        }

        let (model, claim) = match result.model {
            ErrorModel::Quadrature => {
                let p = result.exponent().unwrap_or(f64::NAN);
                let c = result.envelope_constant(1.5);
                text.push_str(&format!(
                    "  quadrature; fitted p = {p:.3}; error <= {c:.4} * (h/R)^1.5\n"
                ));
                (
                    "quadrature",
                    json!({ "envelope_1_5": c, "fitted_exponent": p }),
                )
            }
            ErrorModel::LatticeCount => {
                let c = result.envelope_constant(GAUSS_CIRCLE_EXPONENT);
                text.push_str(&format!(
                    "  lattice-point counting (Gauss circle); no meaningful fitted rate;\n  \
                     error <= {c:.4} * (h/R)^{GAUSS_CIRCLE_EXPONENT:.5}\n"
                ));
                (
                    "lattice_count",
                    json!({ "envelope_gauss": c, "fitted_exponent": Value::Null }),
                )
            }
        };
        if !result.is_monotone() {
            text.push_str("  NOT monotone: a finer lattice made the answer worse at least once\n");
        }
        if let Some(finest) = result.finest_within(1.0 / 200.0) {
            text.push_str(&format!(
                "  at h <= R/200 (h/R = {:.5}): {:.5}%\n",
                finest.ratio,
                finest.mesh_error() * 100.0
            ));
            if finest.mesh_error() >= 1e-3 {
                ok = false;
            }
        }
        text.push('\n');

        cases.push(json!({
            "claim": claim,
            "model": model,
            "monotone": result.is_monotone(),
            "name": result.name,
            "observed_orders": result.observed_orders(),
            "samples": samples,
        }));
    }

    text.push_str(
        "The two cylinder rows are the argument for Unit 6. An axis-parallel\n\
         cylinder's chord is a hard indicator, so its volume is exactly a count of\n\
         lattice points inside a disc, and that error is erratic -- refining the\n\
         lattice is not reliably an improvement. Lying down, the same cylinder is a\n\
         smooth quadrature and converges predictably. The fix for a vertical wall is\n\
         a bundle along another axis, not a finer lattice.\n",
    );

    Ok((
        json!({ "cases": cases, "command": "convergence" }),
        text,
        ok,
    ))
}

// --- shared rendering ------------------------------------------------------

fn describe_lattice(field: &DexelField) -> String {
    let lattice = field.lattice();
    let [nx, ny] = lattice.counts();
    format!(
        "lattice   {nx} x {ny} = {} rays along {}, {} mm cells\n\
         origin    {:?} mm\n\
         ray span  {} mm",
        lattice.ray_count(),
        lattice.axis().as_str(),
        lattice.spacing(),
        lattice.origin().to_array(),
        lattice.ray_length(),
    )
}

fn describe_occupancy(field: &DexelField, stats: Option<&BuildStats>) -> String {
    let rays = field.arena().rays();
    let filled = field.filled_rays();
    let mut out = format!(
        "rays      {rays}, {filled} carrying material ({:.1}%)\n\
         spans     {}, {} spilled past the inline capacity\n\
         volume    {} mm^3\n\
         bytes     {:.2} MiB\n",
        filled as f64 / rays.max(1) as f64 * 100.0,
        field.total_spans(),
        field.arena().spilled_rays(),
        field.volume(),
        field.arena().bytes() as f64 / (1024.0 * 1024.0),
    );
    if let Some(stats) = stats {
        out.push_str(&format!(
            "predicates {} tests, {:.4}% took the exact path, {} SoS resolutions\n",
            stats.predicates.triangle_tests,
            stats.predicates.exact_fraction() * 100.0,
            stats.predicates.sos_resolutions,
        ));
    }
    out
}

fn occupancy_json(field: &DexelField, stats: Option<&BuildStats>) -> Value {
    json!({
        "bytes": field.arena().bytes(),
        "filled_rays": field.filled_rays(),
        "predicates": stats.map(|s| json!({
            "exact_path": s.predicates.exact_path,
            "sos_resolutions": s.predicates.sos_resolutions,
            "triangle_tests": s.predicates.triangle_tests,
        })),
        "spilled_rays": field.arena().spilled_rays(),
        "total_spans": field.total_spans(),
        "volume_mm3": field.volume(),
    })
}

fn describe_distribution(field: &DexelField) -> String {
    let mut out = String::from("span distribution\n");
    let total = field.arena().rays().max(1);
    for (spans, rays) in field.arena().distribution() {
        let share = rays as f64 / total as f64 * 100.0;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "a bar length, clamped to 50"
        )]
        let bar = "#".repeat(((share / 2.0).round() as usize).min(50));
        out.push_str(&format!(
            "  {spans:>3} span(s)  {rays:>10} rays  {share:>6.2}%  {bar}\n"
        ));
    }
    out
}
