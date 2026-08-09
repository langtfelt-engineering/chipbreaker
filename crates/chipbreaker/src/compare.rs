// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! `chipbreaker compare`: a simulated result against the part it was meant to be.
//!
//! # What this reports, exactly
//!
//! `d_H(computed stock, ideal geometric cutting model)`. That is the whole of it.
//!
//! It says nothing about tool wear, deflection under load, thermal growth,
//! spindle runout, backlash, or how a controller interpolates between the points
//! it was given. A part can match this comparison exactly and still be out of
//! tolerance for any of those reasons. The text output says so on every run, and
//! that line is not decoration: a verification tool that lets a customer forget
//! which model it verified is worse than no tool.
//!
//! # The two signs are never blended
//!
//! **A gouge is unambiguous.** Material that should be there is not, and nothing
//! downstream puts it back.
//!
//! **Excess stock is often expected.** It is what a roughing pass leaves on
//! purpose, and a finishing pass removes it.
//!
//! So they are reported as two numbers and never combined into one "deviation".
//! A single figure would let 0.2 mm of intended roughing allowance cancel against
//! 0.2 mm of scrap, and the two are not the same event.
//!
//! # `--tolerance` is checked against the floor before it is used
//!
//! ADR 0005: any accuracy metric floors against the fidelity of its input. Here
//! there are three — the stock mesh, the nominal mesh, and the lattice — so a
//! tolerance below the coarsest of them describes the inputs rather than the
//! engine. Asking for 0.01 mm against a 1 mm-faceted nominal is refused with the
//! number that would be honest, rather than obliged.
//!
//! # `deviation-stat` is the distribution behind the headline
//!
//! `compare` answers "is this part acceptable". `deviation-stat` answers "where
//! and how is it wrong": a histogram by depth, a split by bundle, and the worst
//! samples with their coordinates. Same inputs, same arithmetic, different
//! question — which is why they share a module rather than a flag.

use std::path::PathBuf;

use chipbreaker_core::deviation::{DeviationField, compare as deviation_compare};
use chipbreaker_core::dexel::tri::TriDexelField;
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::mesh::TriMesh;
use clap::Args;
use serde_json::{Value, json};

use crate::mesh::Input;

/// The sentence that must appear wherever a deviation is reported.
const SCOPE: &str = "This compares the computed stock against the ideal geometric cutting \
                     model. It does not model tool wear, deflection, thermal growth, runout, \
                     backlash or controller interpolation.";

/// `chipbreaker compare ...`
#[derive(Debug, Args)]
pub struct CompareArgs {
    /// The cut field to judge, as `.tdx`.
    pub file: PathBuf,
    /// The nominal part, as a mesh.
    #[arg(long, value_name = "FILE")]
    pub nominal: PathBuf,
    /// The stock the field was built from, as a mesh.
    ///
    /// Optional, and only used to report the tessellation floor: a comparison
    /// cannot be finer than the coarser of its two meshes, and without this one
    /// the floor is reported from the nominal and the lattice alone.
    #[arg(long, value_name = "FILE")]
    pub stock: Option<PathBuf>,
    /// Unit the meshes' coordinates are in.
    ///
    /// Required for STL and OBJ, which carry no unit information at all.
    #[arg(long, value_name = "UNIT")]
    pub units: Option<String>,
    /// The tolerance to judge against, in millimetres.
    #[arg(long, value_name = "MM", default_value_t = 0.1)]
    pub tolerance: f64,
    /// Report below the tessellation floor anyway, with the floor stated.
    ///
    /// The refusal exists because a number finer than the inputs support is a
    /// wrong answer with a plausible face. Overriding it is a deliberate act.
    #[arg(long)]
    pub allow_below_floor: bool,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

/// `chipbreaker deviation-stat ...`
#[derive(Debug, Args)]
pub struct DeviationStatArgs {
    #[command(flatten)]
    pub compare: CompareArgs,
    /// How many of the worst samples to list, with their coordinates.
    #[arg(long, value_name = "N", default_value_t = 10)]
    pub worst: usize,
}

fn read_field(file: &std::path::Path) -> Result<TriDexelField, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
    match dexel_io::detect(&bytes) {
        Some(FieldFormat::Tri) => {
            dexel_io::tri_from_bytes(&bytes).map_err(|e| format!("{}: {e}", file.display()))
        }
        Some(FieldFormat::Single) => Err(format!(
            "{} is a single-bundle .dexel field. A deviation is measured at every \
             span endpoint of all three bundles, because a surface one bundle \
             grazes is one another sees squarely. Rebuild it with \
             `dexel build --axes xyz`.",
            file.display()
        )),
        None => Err(format!(
            "{} is not a Chipbreaker field file",
            file.display()
        )),
    }
}

fn read_mesh(path: &std::path::Path, units: Option<&str>) -> Result<TriMesh, String> {
    let unit = match units {
        Some(u) => Some(crate::mesh::parse_unit(u)?),
        None => None,
    };
    let input = Input {
        file: path.to_path_buf(),
        units: unit,
        weld_tol: chipbreaker_core::eps::EPS_WELD,
        json: false,
    };
    crate::mesh::load(&input).map(|(m, _)| m)
}

/// Runs the comparison, shared by both subcommands.
fn run_comparison(args: &CompareArgs) -> Result<(DeviationField, TriMesh), String> {
    if !args.tolerance.is_finite() || args.tolerance <= 0.0 {
        return Err(format!(
            "--tolerance must be a positive number of millimetres, got {}",
            args.tolerance
        ));
    }
    let field = read_field(&args.file)?;
    let nominal = read_mesh(&args.nominal, args.units.as_deref())?;
    let stock = match &args.stock {
        Some(p) => Some(read_mesh(p, args.units.as_deref())?),
        None => None,
    };
    let deviation = deviation_compare(&field, &nominal, stock.as_ref());
    Ok((deviation, nominal))
}

/// The floor check, which both subcommands apply before reporting anything.
fn check_floor(d: &DeviationField, args: &CompareArgs) -> Result<(), String> {
    if !d.below_floor(args.tolerance) || args.allow_below_floor {
        return Ok(());
    }
    Err(format!(
        "a tolerance of {:.4} mm is below the {:.4} mm floor these inputs support, \
         so any finding at that scale would describe the inputs and not the part.\n\
         \x20 stock facets   {:.4} mm\n\
         \x20 nominal facets {:.4} mm\n\
         \x20 lattice        {:.4} mm\n\
         Refine the coarsest of the three, or pass --allow-below-floor to report \
         anyway with the floor stated.",
        args.tolerance,
        d.tolerance_floor_mm(),
        d.stock_facet_mm,
        d.nominal_facet_mm,
        d.spacing_mm,
    ))
}

fn digest_of(d: &DeviationField) -> String {
    let mut h = CanonicalHash::new();
    h.add(d);
    h.finish().to_hex()
}

/// Runs `chipbreaker compare`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn compare(args: &CompareArgs) -> Result<(Value, String, bool), String> {
    let (d, _) = run_comparison(args)?;
    check_floor(&d, args)?;

    let gouges = d
        .samples
        .iter()
        .filter(|s| s.signed_mm < -args.tolerance)
        .count();
    let excess = d
        .samples
        .iter()
        .filter(|s| s.signed_mm > args.tolerance)
        .count();
    // The verdict is on gouges alone. Excess stock is reported at equal
    // prominence and does not decide it: a roughing pass that leaves two
    // millimetres everywhere is correct, and a tool that called it a failure
    // would be turned off within a day.
    let accepted = gouges == 0;

    let text = format!(
        "field      {}\n\
         nominal    {}\n\
         samples    {}\n\
         tolerance  {:.4} mm (floor {:.4} mm: stock {:.4}, nominal {:.4}, lattice {:.4})\n\
         \n\
         GOUGE      worst {:.4} mm over {gouges} samples\n\
         EXCESS     worst {:.4} mm over {excess} samples\n\
         rms        {:.4} mm\n\
         \n\
         verdict    {}\n\
         \n\
         {SCOPE}\n",
        args.file.display(),
        args.nominal.display(),
        d.samples.len(),
        args.tolerance,
        d.tolerance_floor_mm(),
        d.stock_facet_mm,
        d.nominal_facet_mm,
        d.spacing_mm,
        d.worst_gouge_mm,
        d.worst_excess_mm,
        d.rms_mm,
        if accepted {
            "no gouge above tolerance"
        } else {
            "GOUGED above tolerance"
        },
    );

    let value = json!({
        "field": args.file.display().to_string(),
        "nominal": args.nominal.display().to_string(),
        "samples": d.samples.len(),
        "tolerance_mm": args.tolerance,
        "tolerance_floor_mm": d.tolerance_floor_mm(),
        "stock_facet_mm": d.stock_facet_mm,
        "nominal_facet_mm": d.nominal_facet_mm,
        "spacing_mm": d.spacing_mm,
        "worst_gouge_mm": d.worst_gouge_mm,
        "worst_excess_mm": d.worst_excess_mm,
        "gouge_samples": gouges,
        "excess_samples": excess,
        "rms_mm": d.rms_mm,
        "worst_projection_gap_mm": d.worst_projection_gap_mm,
        "accepted": accepted,
        "scope": SCOPE,
        "digest": digest_of(&d),
    });
    Ok((value, text, accepted))
}

/// The depth bands the histogram uses, as multiples of the tolerance.
const BANDS: [f64; 6] = [0.5, 1.0, 2.0, 4.0, 8.0, f64::INFINITY];

/// Runs `chipbreaker deviation-stat`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn deviation_stat(args: &DeviationStatArgs) -> Result<(Value, String, bool), String> {
    let (d, _) = run_comparison(&args.compare)?;
    check_floor(&d, &args.compare)?;
    let tolerance = args.compare.tolerance;

    // Histogram, gouges and excess held apart the whole way down.
    let mut gouge_bands = [0usize; BANDS.len()];
    let mut excess_bands = [0usize; BANDS.len()];
    let mut by_axis = [0usize; 3];
    for s in &d.samples {
        let magnitude = s.signed_mm.abs();
        if magnitude <= tolerance {
            continue;
        }
        let band = BANDS
            .iter()
            .position(|b| magnitude <= tolerance * b)
            .unwrap_or(BANDS.len() - 1);
        if s.signed_mm < 0.0 {
            gouge_bands[band] += 1;
        } else {
            excess_bands[band] += 1;
        }
        if let Some(slot) = by_axis.get_mut(s.axis) {
            *slot += 1;
        }
    }

    let mut worst: Vec<_> = d.samples.iter().collect();
    worst.sort_by(|a, b| {
        b.signed_mm
            .abs()
            .partial_cmp(&a.signed_mm.abs())
            .unwrap_or(core::cmp::Ordering::Equal)
            // Ties broken by position, so the list is the same on every run and
            // on every platform. Sorting floats alone leaves equal magnitudes in
            // whatever order the walk produced.
            .then(a.at.x.total_cmp(&b.at.x))
            .then(a.at.y.total_cmp(&b.at.y))
            .then(a.at.z.total_cmp(&b.at.z))
    });
    worst.truncate(args.worst);

    let mut text = format!(
        "field      {}\n\
         nominal    {}\n\
         samples    {}, tolerance {tolerance:.4} mm, floor {:.4} mm\n\
         \n\
         depth distribution, in multiples of the tolerance:\n\
         {:>14}{:>10}{:>10}\n",
        args.compare.file.display(),
        args.compare.nominal.display(),
        d.samples.len(),
        d.tolerance_floor_mm(),
        "band",
        "gouge",
        "excess",
    );
    let mut low = 1.0;
    for (index, high) in BANDS.iter().enumerate() {
        if gouge_bands[index] == 0 && excess_bands[index] == 0 {
            low = *high;
            continue;
        }
        let label = if high.is_finite() {
            format!("{low:.1}-{high:.1}x")
        } else {
            format!("{low:.1}x+")
        };
        text.push_str(&format!(
            "{label:>14}{:>10}{:>10}\n",
            gouge_bands[index], excess_bands[index]
        ));
        low = *high;
    }

    text.push_str(&format!(
        "\nby bundle: X {}, Y {}, Z {}\n",
        by_axis[0], by_axis[1], by_axis[2]
    ));
    text.push_str(&format!(
        "\nperpendicular reading overstates the metric by up to {:.4} mm somewhere, \
         \nwhich is where the two surfaces meet at a steep angle and a perpendicular \
         \nnumber describes the measurement rather than the part.\n",
        d.worst_projection_gap_mm
    ));

    text.push_str(&format!("\nworst {} samples:\n", worst.len()));
    for s in &worst {
        text.push_str(&format!(
            "  {:+9.4} mm at ({:8.3}, {:8.3}, {:8.3})  normal ({:6.3}, {:6.3}, {:6.3})  \
             axis {}  perpendicular {:+.4}\n",
            s.signed_mm,
            s.at.x,
            s.at.y,
            s.at.z,
            s.normal.x,
            s.normal.y,
            s.normal.z,
            s.axis,
            s.perpendicular_mm,
        ));
    }
    text.push_str(&format!("\n{SCOPE}\n"));

    let bands: Vec<Value> = BANDS
        .iter()
        .enumerate()
        .map(|(index, high)| {
            json!({
                "upper_multiple": if high.is_finite() { json!(high) } else { Value::Null },
                "gouge": gouge_bands[index],
                "excess": excess_bands[index],
            })
        })
        .collect();
    let worst_json: Vec<Value> = worst
        .iter()
        .map(|s| {
            json!({
                "signed_mm": s.signed_mm,
                "perpendicular_mm": s.perpendicular_mm,
                "at_mm": [s.at.x, s.at.y, s.at.z],
                "normal": [s.normal.x, s.normal.y, s.normal.z],
                "axis": s.axis,
            })
        })
        .collect();

    let value = json!({
        "field": args.compare.file.display().to_string(),
        "nominal": args.compare.nominal.display().to_string(),
        "samples": d.samples.len(),
        "tolerance_mm": tolerance,
        "tolerance_floor_mm": d.tolerance_floor_mm(),
        "bands": bands,
        "by_axis": by_axis,
        "worst_gouge_mm": d.worst_gouge_mm,
        "worst_excess_mm": d.worst_excess_mm,
        "rms_mm": d.rms_mm,
        "worst_projection_gap_mm": d.worst_projection_gap_mm,
        "worst_samples": worst_json,
        "scope": SCOPE,
        "digest": digest_of(&d),
    });
    // A distribution is a description, not a verdict; `compare` is where a
    // pass or fail is decided.
    Ok((value, text, true))
}
