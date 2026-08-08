// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! `chipbreaker collide`.
//!
//! # Why this takes the stock and not the result
//!
//! Every other command that reads a field reads the field a run produced.
//! This one reads the field a run **started from**, and the difference is not
//! cosmetic.
//!
//! A collision is a property of the trajectory. At the moment a move executes,
//! the material in its way is the stock as it stood then — not the stock at the
//! end, from which the offending material may since have been cut. Handing this
//! command a result field would make it test every move against the least
//! material the job ever contains, and it would report "clear" for a program
//! that buries the spindle. So the program is replayed.
//!
//! # The exit code
//!
//! | code | means |
//! |---|---|
//! | 0 | checked, and no collision |
//! | 1 | a collision, or the check could not run |
//!
//! **Unchecked exits non-zero.** A gate that could not run has not passed, and a
//! CI job that treats "I could not look" as "nothing there" is worse than one
//! that has no collision check at all — it reports safety it never established.

use std::path::PathBuf;

use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::findings::collide::collision_count;
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use clap::Args;
use serde_json::{Value, json};

use crate::mesh::Input;

/// `chipbreaker collide ...`
#[derive(Debug, Args)]
pub struct CollideArgs {
    /// The **stock** field the program starts from, as `.tdx`.
    ///
    /// Not the cut result: a collision is judged against the material present
    /// when each move runs, so the program is replayed from the beginning.
    pub stock: PathBuf,
    /// The NC program to replay.
    #[arg(long, value_name = "FILE")]
    pub path: PathBuf,
    /// The tool library, which is where the holder geometry comes from.
    #[arg(long, value_name = "FILE")]
    pub tools: PathBuf,
    /// Which tool, by library id.
    #[arg(long, value_name = "ID")]
    pub tool: Option<String>,
    /// Static obstacles: clamps, vises, the table. Comma separated.
    #[arg(long, value_name = "FILES", value_delimiter = ',')]
    pub fixtures: Vec<PathBuf>,
    /// Unit the fixture meshes are in.
    #[arg(long, value_name = "UNIT")]
    pub units: Option<String>,
    /// Report a near miss when the gap is below this, in millimetres.
    #[arg(long, value_name = "MM", default_value_t = 0.0)]
    pub clearance: f64,
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
            "{} is a single-bundle field; collision checking needs all three bundles",
            file.display()
        )),
        None => Err(format!(
            "{} is not a Chipbreaker field file",
            file.display()
        )),
    }
}

/// Runs `chipbreaker collide`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn collide(args: &CollideArgs) -> Result<(Value, String, bool), String> {
    if !args.clearance.is_finite() || args.clearance < 0.0 {
        return Err(format!(
            "--clearance must be a non-negative number of millimetres, got {}",
            args.clearance
        ));
    }
    let started = std::time::Instant::now();

    let mut field = read_field(&args.stock)?;
    // The lattice pitch, taken from a bundle rather than assumed. A fixture
    // built at a different resolution than the stock would sample the same
    // geometry two ways and disagree with itself about a grazing contact.
    let spacing = field.bundles().next().map_or(0.4, |(_, b)| {
        let uv = b.lattice().spacing_uv();
        uv[0].min(uv[1])
    });
    let replay = crate::run::resolve_for_collision(
        &args.path,
        Some(args.tools.as_path()),
        args.tool.as_deref(),
    )?;

    // Each fixture becomes a field of its own, at the stock's resolution and
    // over its own bounds. A clamp is not inside the stock's lattice and a
    // shared one would either miss it or waste most of its cells on air.
    let unit = match &args.units {
        Some(u) => Some(crate::mesh::parse_unit(u)?),
        None => None,
    };
    let mut fixtures = Vec::with_capacity(args.fixtures.len());
    for f in &args.fixtures {
        let (mesh, _) = crate::mesh::load(&Input {
            file: f.clone(),
            units: unit,
            weld_tol: chipbreaker_core::eps::EPS_WELD,
            json: false,
        })?;
        let (built, _) = TriDexelField::build(
            &mesh,
            &TriBuildOptions {
                spacing,
                ..TriBuildOptions::default()
            },
        )
        .map_err(|e| format!("cannot build a field for {}: {e}", f.display()))?;
        let name = f
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("fixture")
            .to_owned();
        fixtures.push((name, built));
    }

    let params = CollideParams {
        clearance_mm: args.clearance,
        // Two cells, as for findings: the smallest quantisation that does not
        // split one contact across the three interleaved bundle lattices.
        grid_mm: 2.0 * spacing,
        method: SweepMethod::Analytic {
            tolerance: spacing / 10.0,
        },
    };
    let mut scratch = CutScratch::new(&replay.profile);
    let outcome = collide_with_stock(
        &mut field,
        &replay.profile,
        &replay.motions,
        &replay.kinds,
        &replay.provenance,
        replay.unmodelled_retracts,
        &fixtures,
        &params,
        &mut scratch,
    );
    let elapsed = started.elapsed().as_secs_f64() * 1000.0;

    let (collisions, unchecked) = match outcome {
        Ok(c) => (c, None),
        Err(u) => (Vec::new(), Some(u)),
    };
    let hard = collision_count(&collisions);
    let near = collisions.len() - hard;
    // Unchecked is not clear. See the module header.
    let ok = unchecked.is_none() && hard == 0;

    let mut text = format!(
        "stock      {}\n\
         program    {}\n\
         rapids     {}\n\
         fixtures   {}\n\n",
        args.stock.display(),
        args.path.display(),
        replay.rapid_path.as_str(),
        if fixtures.is_empty() {
            "none".to_owned()
        } else {
            fixtures
                .iter()
                .map(|(n, _)| n.clone())
                .collect::<Vec<_>>()
                .join(", ")
        }
    );
    if let Some(u) = &unchecked {
        text.push_str(&format!("UNCHECKED  {u}\n"));
    } else {
        text.push_str(&format!("collisions {hard}\nnear miss  {near}\n\n"));
        for c in &collisions {
            text.push_str(&format!(
                "  {} {:<10} {:<11} {:<9} vs {:<8} {:>9.4} mm  line {}\n",
                c.id,
                c.contact.as_str(),
                c.role.as_str(),
                c.motion.as_str(),
                c.obstacle.kind(),
                c.contact.magnitude(),
                c.attribution.provenance.first().map_or(0, |p| p.line)
            ));
        }
    }

    let value = json!({
        "schema": "chipbreaker.collision-report",
        "checked": unchecked.is_none(),
        "unchecked_because": unchecked.as_ref().map(std::string::ToString::to_string),
        "rapid_path": replay.rapid_path.as_str(),
        "clearance_mm": args.clearance,
        "summary": {
            "collisions": hard,
            "near_misses": near,
        },
        "collisions": collisions.iter().map(collision_json).collect::<Vec<_>>(),
        "environment": chipbreaker_core::findings::report::environment("local", elapsed),
    });
    Ok((value, text, ok))
}

fn collision_json(c: &chipbreaker_core::findings::Collision) -> Value {
    use chipbreaker_core::findings::{Contact, Obstacle};
    let mut severity = serde_json::Map::new();
    match c.contact {
        Contact::Collision { penetration_mm } => {
            severity.insert("penetration_mm".to_owned(), json!(penetration_mm));
        }
        Contact::NearMiss { clearance_mm } => {
            severity.insert("clearance_mm".to_owned(), json!(clearance_mm));
        }
    }
    let mut obstacle = serde_json::Map::new();
    obstacle.insert("kind".to_owned(), json!(c.obstacle.kind()));
    if let Obstacle::Fixture { index, name } = &c.obstacle {
        obstacle.insert("index".to_owned(), json!(index));
        obstacle.insert("name".to_owned(), json!(name));
    }
    json!({
        "id": c.id,
        "contact": c.contact.as_str(),
        "is_defect": c.is_defect(),
        "severity": Value::Object(severity),
        "element": { "role": c.role.as_str(), "index": c.element_index },
        "obstacle": Value::Object(obstacle),
        "motion": c.motion.as_str(),
        "at": [c.at.x, c.at.y, c.at.z],
        "attribution": {
            "segments": c.attribution.segments,
            "lines": c.attribution.provenance.iter().map(|p| p.line).collect::<Vec<_>>(),
        },
    })
}
