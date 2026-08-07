// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! `chipbreaker extract`: a cut field back to a triangle mesh.
//!
//! # `--no-normals` is a measurement, not a fallback
//!
//! Discarding the stored normals leaves the quadratic error function with points
//! and no planes, so every direction is unconstrained and each vertex resolves
//! to the centroid of its crossings. That is plain surface nets: still manifold,
//! still watertight, and with every sharp edge rounded off.
//!
//! It exists so the cost of storing normals can be checked rather than asserted.
//! At 0.5 mm, on the edges a machinist measures first:
//!
//! | geometry | with normals | without |
//! |---|---|---|
//! | an uncut 16 x 12 x 8 mm block | **exact** | 0.167 mm worst, 0.126 rms |
//! | a slot cut through a 24 x 18 x 10 mm block | **exact** | 0.125 mm worst, 0.125 rms |
//!
//! **The second row is the one that was missing, and it was missing for five
//! units.** The first measures only *construction* normals — a field built from a
//! mesh takes each endpoint's normal from the triangle its ray crossed, and that
//! path was always right. A cut face takes its normal from the tool, and until
//! Unit 12 the sweep set none at all: every cut face in the engine carried
//! `(0, 0, -1)`, whichever way it faced. So the claim that four bytes an endpoint
//! buy sharp features had been demonstrated only where it was never in doubt.
//!
//! It holds on cut geometry too, and holds exactly. Both rows are published
//! because only together do they say what the four bytes buy. ADR 0010 records
//! why the defect survived so long, and `tests/contour_accuracy.rs` is where both
//! numbers are measured.
//!
//! `--no-normals` is also the honest way to read a version 2 `.tdx`, which
//! predates normals.
//!
//! # `--stats` reports the disagreement rather than hiding it
//!
//! The three bundles were cut independently and can classify a grid corner
//! differently. Extraction resolves that by majority, and publishes how often it
//! had to: the rate is a direct reading of how far the three fields differ on
//! real geometry, which no bound derived at Unit 6 provides.

use std::path::PathBuf;

use chipbreaker_core::contour::{ContourOptions, DEFAULT_CLAMP_EXPAND, extract as contour_extract};
use chipbreaker_core::dexel::tri::TriDexelField;
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::mesh::validate::validate;
use clap::Args;
use serde_json::{Value, json};

use crate::mesh::write_mesh;

/// `chipbreaker extract ...`
#[derive(Debug, Args)]
pub struct ExtractArgs {
    /// The field to contour, as `.tdx`.
    pub file: PathBuf,
    /// Where to write the mesh: `.stl`, `.stla` or `.obj`.
    #[arg(long, value_name = "FILE")]
    pub out: Option<PathBuf>,
    /// Discard the stored normals and fall back to surface nets.
    #[arg(long)]
    pub no_normals: bool,
    /// How far outside its cell a vertex may sit, in cells.
    #[arg(long, value_name = "F", default_value_t = DEFAULT_CLAMP_EXPAND)]
    pub clamp_expand: f64,
    /// Report counts and rates without writing a mesh.
    #[arg(long)]
    pub stats: bool,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

fn read_field(file: &std::path::Path) -> Result<TriDexelField, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
    match dexel_io::detect(&bytes) {
        Some(FieldFormat::Tri) => {
            dexel_io::tri_from_bytes(&bytes).map_err(|e| format!("{}: {e}", file.display()))
        }
        Some(FieldFormat::Single) => Err(format!(
            "{} is a single-bundle .dexel field. Dual contouring needs all three \
             bundles, because they are the three edge directions of one grid. \
             Rebuild it with `dexel build --axes xyz`.",
            file.display()
        )),
        None => Err(format!(
            "{} is not a Chipbreaker field file",
            file.display()
        )),
    }
}

/// Runs `chipbreaker extract`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn extract(args: &ExtractArgs) -> Result<(Value, String, bool), String> {
    if !args.clamp_expand.is_finite() || args.clamp_expand < 0.0 {
        return Err(format!(
            "--clamp-expand must be a non-negative number of cells, got {}",
            args.clamp_expand
        ));
    }
    let field = read_field(&args.file)?;
    let options = ContourOptions {
        clamp_expand: args.clamp_expand,
        use_normals: !args.no_normals,
    };
    let (mesh, stats) = contour_extract(&field, &options).map_err(|e| e.to_string())?;
    let report_mesh = validate(&mesh);

    let mut digest = CanonicalHash::new();
    digest.add(&mesh);
    let digest = digest.finish().to_hex();

    #[allow(clippy::cast_precision_loss, reason = "a ratio of counts")]
    let disagreement_rate = if stats.corners == 0 {
        0.0
    } else {
        stats.corner_disagreements as f64 / stats.corners as f64
    };
    #[allow(clippy::cast_precision_loss, reason = "a ratio of counts")]
    let split_rate = if stats.cells_with_vertices == 0 {
        0.0
    } else {
        stats.cells_with_multiple_vertices as f64 / stats.cells_with_vertices as f64
    };

    let mut written = None;
    if let Some(path) = &args.out {
        let (bytes, format) = write_mesh(&mesh, path)?;
        written = Some((path.display().to_string(), bytes, format));
    }

    let mut text = format!(
        "field     {}\n\
         method    manifold dual contouring, {}\n\
         mesh      {} vertices, {} triangles\n",
        args.file.display(),
        if args.no_normals {
            "normals DISCARDED (surface nets; sharp edges will be rounded off)"
        } else {
            "using stored normals"
        },
        mesh.vertex_count(),
        mesh.triangle_count(),
    );
    // The exit criterion, printed whether or not anyone asked for stats. A mesh
    // that is not manifold is not a result, it is a defect.
    text.push_str(&format!(
        "sound     manifold {}, watertight {}, oriented {}, volume {:.4} mm^3\n",
        yes_no(report_mesh.is_manifold),
        yes_no(report_mesh.is_watertight),
        yes_no(report_mesh.is_orientation_consistent),
        report_mesh.signed_volume,
    ));
    text.push_str(&format!(
        "corners   {} classified, {} disagreed ({:.4}%), {} short of three votes\n",
        stats.corners,
        stats.corner_disagreements,
        disagreement_rate * 100.0,
        stats.corners_short_of_three_votes,
    ));
    text.push_str(&format!(
        "edges     {} with a crossing, {} carrying more than one (a feature \
         thinner than a cell)\n",
        stats.crossing_edges, stats.multi_crossing_edges,
    ));
    text.push_str(&format!(
        "cells     {} with a vertex, {} needing more than one ({:.4}%), which \
         plain DC would have got wrong\n",
        stats.cells_with_vertices,
        stats.cells_with_multiple_vertices,
        split_rate * 100.0,
    ));
    text.push_str(&format!(
        "features  {} flat, {} edge, {} corner; {} vertices clamped\n",
        stats.rank_histogram[1],
        stats.rank_histogram[2],
        stats.rank_histogram[3],
        stats.clamped_vertices,
    ));
    if stats.sign_change_without_crossing > 0 {
        text.push_str(&format!(
            "NOTE      {} sign changes had no crossing on their own bundle's ray. \
             That is the majority vote disagreeing with the bundle that owns the \
             edge; the midpoint was used.\n",
            stats.sign_change_without_crossing
        ));
    }
    text.push_str(&format!("digest    {digest}\n"));
    if let Some((path, bytes, format)) = &written {
        text.push_str(&format!(
            "wrote     {path} as {format} ({bytes} bytes, {:.2} MiB)\n",
            *bytes as f64 / (1024.0 * 1024.0)
        ));
    }

    let results = json!({
        "command": "extract",
        "cells": {
            "with_vertices": stats.cells_with_vertices,
            "with_multiple_vertices": stats.cells_with_multiple_vertices,
            "split_rate": split_rate,
        },
        "corners": {
            "classified": stats.corners,
            "disagreed": stats.corner_disagreements,
            "disagreement_rate": disagreement_rate,
            "short_of_three_votes": stats.corners_short_of_three_votes,
        },
        "digest": digest,
        "edges": {
            "with_crossing": stats.crossing_edges,
            "multi_crossing": stats.multi_crossing_edges,
            "sign_change_without_crossing": stats.sign_change_without_crossing,
        },
        "features": {
            "clamped": stats.clamped_vertices,
            "corner": stats.rank_histogram[3],
            "edge": stats.rank_histogram[2],
            "flat": stats.rank_histogram[1],
        },
        "mesh": {
            "triangles": mesh.triangle_count(),
            "vertices": mesh.vertex_count(),
        },
        "normals": !args.no_normals,
        "sound": {
            "is_manifold": report_mesh.is_manifold,
            "is_orientation_consistent": report_mesh.is_orientation_consistent,
            "is_watertight": report_mesh.is_watertight,
            "signed_volume": report_mesh.signed_volume,
        },
        "written": written.map(|(p, n, f)| json!({ "bytes": n, "format": f, "path": p })),
    });

    // A mesh that fails the exit criterion is a failure, not a warning. Unit 12
    // compares this against the nominal part and a hole becomes a phantom gouge,
    // so it must not be possible to pipe one onward without noticing.
    let sound = report_mesh.is_manifold
        && report_mesh.is_watertight
        && report_mesh.is_orientation_consistent
        && report_mesh.signed_volume > 0.0;
    Ok((results, text, sound))
}

const fn yes_no(b: bool) -> &'static str {
    if b { "yes" } else { "NO" }
}
