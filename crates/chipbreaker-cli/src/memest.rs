// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! `chipbreaker mem-estimate`: will this job fit?
//!
//! Its own verb rather than a flag on `build`, because "will this fit" is a
//! question asked *before* committing to a run, and an OEM integrator wants it
//! available programmatically without having to start something they intend to
//! cancel. It reads the stock mesh's bounds and the program's segment count, and
//! allocates neither a field nor an extraction window.
//!
//! The three contributors are reported separately for the same reason the
//! refusal does it: a user whose problem is a three-million-segment program
//! needs to be told that, not sent off to coarsen a lattice that was never the
//! issue.

use std::path::PathBuf;

use chipbreaker_core::budget::{Budget, BudgetError, Spacing, auto_spacing, human, ray_counts};
use chipbreaker_gcode::resolve::{ParseOptions, parse};
use clap::Args;
use serde_json::{Value, json};

/// `chipbreaker mem-estimate ...`
#[derive(Debug, Args)]
pub struct MemEstimateArgs {
    /// Stock mesh, to take the extents from.
    #[command(flatten)]
    pub stock: crate::mesh::Input,
    /// Cell size, in millimetres.
    #[arg(long, value_name = "MM")]
    pub res: f64,
    /// Cell size along X, overriding `--res` for that axis.
    #[arg(long, value_name = "MM")]
    pub res_x: Option<f64>,
    /// Cell size along Y.
    #[arg(long, value_name = "MM")]
    pub res_y: Option<f64>,
    /// Cell size along Z.
    #[arg(long, value_name = "MM")]
    pub res_z: Option<f64>,
    /// Choose the three spacings automatically, holding the accuracy of `--res`.
    #[arg(long)]
    pub auto_res: bool,
    /// NC program whose toolpath IR will be held alongside the field.
    #[arg(long, value_name = "FILE")]
    pub path: Option<PathBuf>,
    /// Include the extraction sweep's window in the estimate.
    #[arg(long)]
    pub extract: bool,
    /// Report against this ceiling, e.g. `512M`.
    #[arg(long, value_name = "BYTES", value_parser = crate::dexel::parse_bytes)]
    pub mem_limit: Option<u64>,
}

impl MemEstimateArgs {
    /// Whether to emit JSON.
    ///
    /// Comes from the flattened [`crate::mesh::Input`], which already carries a
    /// `--json` flag. Declaring a second one here made clap see two arguments of
    /// the same name and refuse every invocation.
    #[must_use]
    pub const fn json(&self) -> bool {
        self.stock.json
    }
}

/// Runs `chipbreaker mem-estimate`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn mem_estimate(args: &MemEstimateArgs) -> Result<(Value, String, bool), String> {
    if !args.res.is_finite() || args.res <= 0.0 {
        return Err(format!(
            "--res must be a positive length in millimetres, got {}",
            args.res
        ));
    }
    let mesh = crate::dexel::load_mesh_for_estimate(&args.stock)?;
    let extents = mesh.bounds().extent().to_array();

    let spacing = if args.auto_res {
        auto_spacing(extents, args.res)
    } else if args.res_x.is_some() || args.res_y.is_some() || args.res_z.is_some() {
        Spacing {
            x: args.res_x.unwrap_or(args.res),
            y: args.res_y.unwrap_or(args.res),
            z: args.res_z.unwrap_or(args.res),
        }
    } else {
        Spacing::uniform(args.res)
    };

    let segments = match &args.path {
        Some(p) => {
            let text = std::fs::read_to_string(p)
                .map_err(|e| format!("cannot read {}: {e}", p.display()))?;
            let name = p.file_stem().and_then(|s| s.to_str()).unwrap_or("program");
            let (toolpath, _, _) =
                parse(&text, name, &ParseOptions::default(), None).map_err(|e| e.to_string())?;
            toolpath.segments.len() as u64
        }
        None => 0,
    };

    let budget = args.mem_limit.map_or_else(Budget::unlimited, Budget::bytes);
    let outcome = budget.check(extents, spacing, segments, args.extract);
    let footprint = match &outcome {
        Ok(f) => *f,
        Err(BudgetError::TooLarge { footprint, .. }) => *footprint,
        Err(e) => return Err(e.to_string()),
    };

    let counts = ray_counts(extents, spacing);
    let bound = spacing.sample_distance_bound();
    let fits = outcome.is_ok();

    let mut text = format!(
        "stock     {:.3} x {:.3} x {:.3} mm\n\
         spacing   {:.6} x {:.6} x {:.6} mm{}\n\
         bound     {bound:.6} mm worst-case sample distance\n\
         rays      {} + {} + {} = {}\n",
        extents[0],
        extents[1],
        extents[2],
        spacing.x,
        spacing.y,
        spacing.z,
        if args.auto_res {
            " (chosen automatically)"
        } else {
            ""
        },
        counts[0],
        counts[1],
        counts[2],
        counts.iter().sum::<u64>(),
    );
    text.push_str(&format!("field     {}\n", human(footprint.field_bytes)));
    text.push_str(&format!(
        "spill     {} held back for span splitting under the cutter\n",
        human(footprint.spill_headroom_bytes)
    ));
    if footprint.extraction_bytes > 0 {
        text.push_str(&format!(
            "extract   {} sweep window\n",
            human(footprint.extraction_bytes)
        ));
    }
    if footprint.ir_bytes > 0 {
        text.push_str(&format!(
            "toolpath  {} for {segments} segments\n",
            human(footprint.ir_bytes)
        ));
    }
    text.push_str(&format!("total     {}\n", human(footprint.total_bytes())));
    match &outcome {
        Ok(_) => {
            if let Some(limit) = args.mem_limit {
                #[allow(clippy::cast_precision_loss, reason = "a percentage")]
                let used = footprint.total_bytes() as f64 / limit as f64 * 100.0;
                text.push_str(&format!(
                    "budget    {} -- FITS, {used:.1}% used\n",
                    human(limit)
                ));
            }
        }
        Err(e) => text.push_str(&format!("REFUSED   {e}\n")),
    }

    let results = json!({
        "command": "mem-estimate",
        "extents_mm": extents,
        "fits": fits,
        "limit_bytes": args.mem_limit,
        "memory": {
            "extraction_bytes": footprint.extraction_bytes,
            "field_bytes": footprint.field_bytes,
            "ir_bytes": footprint.ir_bytes,
            "spill_headroom_bytes": footprint.spill_headroom_bytes,
            "total_bytes": footprint.total_bytes(),
        },
        "rays": counts,
        "sample_distance_bound_mm": bound,
        "segments": segments,
        "spacing_mm": [spacing.x, spacing.y, spacing.z],
    });
    Ok((results, text, fits))
}
