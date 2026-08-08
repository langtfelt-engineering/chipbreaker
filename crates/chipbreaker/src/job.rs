// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! `chipbreaker job`: a whole part, across setups.
//!
//! # What a setup is, and why it is the unit
//!
//! A setup is a **transform, a fixture set, and a program**. Everything inside
//! one is exact: operations chain by interval subtraction and nothing
//! accumulates, so ten operations in one setup are bit-identical to their
//! concatenation. Crossing between setups is the only place a transform can lose
//! anything, which is exactly why the boundary is the unit the report accounts
//! for.
//!
//! # The job file is content-addressed like everything else
//!
//! Every input a job names — program, tools, fixtures — reaches the manifest by
//! digest. Fixtures used to be identified by file stem, and two clamps sharing a
//! name would have collided on identity; a path is not an input, it is a place
//! an input used to be.
//!
//! # The verdict is a conjunction over setups
//!
//! A gate that fails in any setup fails the job. Per-setup detail sits beside
//! it so a reader can see *where*, but the job-wide answer is the strict one: a
//! part is not acceptable because two of its three setups were.

use std::path::{Path, PathBuf};

use chipbreaker_core::dexel::tri::TriDexelField;
use chipbreaker_core::findings::report::digest_bytes;
use chipbreaker_core::math::Mat4;
use chipbreaker_core::refixture::{Regime, classify, refixture_exact};
use clap::Args;
use serde_json::Value;

/// `chipbreaker job ...`
#[derive(Debug, Args)]
pub struct JobArgs {
    /// The job description: an ordered list of setups.
    #[arg(long, value_name = "FILE")]
    pub setups: PathBuf,
    /// The nominal part, for the gouge gate.
    #[arg(long, value_name = "FILE")]
    pub nominal: Option<PathBuf>,
    /// Where to write the report.
    #[arg(long, value_name = "FILE")]
    pub report: Option<PathBuf>,
    /// Emit JSON to standard output instead of text.
    #[arg(long)]
    pub json: bool,
}

/// One setup, as read from the job file.
#[derive(Debug, Clone)]
pub struct Setup {
    /// Where this setup sits in the job, counting from zero.
    pub index: u32,
    /// A name for the reader.
    pub name: String,
    /// The rigid motion carrying the previous setup's stock into this one.
    ///
    /// Absent, and taken as the identity, for the first setup — which has
    /// nothing to be carried from.
    pub transform: Option<Mat4>,
    /// The NC program.
    pub program: PathBuf,
    /// The tool library.
    pub tools: PathBuf,
    /// Which tool, by library id.
    pub tool: Option<String>,
    /// Static obstacles for this setup.
    pub fixtures: Vec<PathBuf>,
}

/// A job: an ordered list of setups over one piece of stock.
#[derive(Debug, Clone)]
pub struct Job {
    /// The stock field the first setup starts from.
    pub stock: PathBuf,
    /// Unit for meshes named by the job.
    pub units: Option<String>,
    /// The tolerance to judge findings against.
    ///
    /// Read from the job file and carried, but not yet consumed: the job verb
    /// runs the collision gate and does not compare against a nominal. Kept
    /// rather than dropped so the file format does not have to change when it
    /// does, and marked so nobody assumes it is in force.
    #[allow(
        dead_code,
        reason = "read from the job file; the gouge gate is not wired yet"
    )]
    pub tolerance_mm: f64,
    /// Report a near miss below this gap.
    pub clearance_mm: f64,
    /// The setups, in order.
    pub setups: Vec<Setup>,
}

/// The schema name a job file must carry.
pub const JOB_SCHEMA: &str = "chipbreaker.job";
/// The job file version this build reads.
pub const JOB_VERSION: u32 = 1;

fn rows_to_mat4(v: &Value) -> Option<Mat4> {
    let rows = v.as_array()?;
    if rows.len() != 4 {
        return None;
    }
    let mut m = [[0.0f64; 4]; 4];
    for (i, row) in rows.iter().enumerate() {
        let cells = row.as_array()?;
        if cells.len() != 4 {
            return None;
        }
        for (j, c) in cells.iter().enumerate() {
            m[i][j] = c.as_f64()?;
        }
    }
    Some(Mat4::from_rows_array(m))
}

/// A quarter turn and friends, by name.
///
/// Written as exact matrices rather than through a cosine, because these are
/// the transforms that must classify as axis-aligned and a cosine of a right
/// angle is a few ulps from zero. A job file may still give a matrix directly.
fn named_transform(name: &str) -> Option<Mat4> {
    let m = match name {
        "identity" => [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "rotate-z-90" => [
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "rotate-z-180" => [
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "flip-x" => [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "flip-y" => [
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        _ => return None,
    };
    Some(Mat4::from_rows_array(m))
}

/// Reads a job file.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn load_job(path: &Path) -> Result<Job, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text)
        .map_err(|e| format!("{} is not valid JSON: {e}", path.display()))?;
    if v["schema"].as_str() != Some(JOB_SCHEMA) {
        return Err(format!(
            "{} is not a Chipbreaker job file (schema is {:?})",
            path.display(),
            v["schema"].as_str().unwrap_or("absent")
        ));
    }
    let version = v["version"].as_u64();
    if version != Some(u64::from(JOB_VERSION)) {
        return Err(format!(
            "{} is a job file version {} and this build reads version {JOB_VERSION}",
            path.display(),
            version.map_or_else(|| "absent".to_owned(), |n| n.to_string())
        ));
    }

    // Paths in a job file resolve against the file, not the working directory.
    // A job that only ran from its own folder would be a job somebody has to
    // remember how to run.
    let base = path.parent().unwrap_or(Path::new(".")).to_path_buf();
    let rel = |p: &str| base.join(p);

    let stock = v["stock"]
        .as_str()
        .ok_or_else(|| format!("{}: no stock field named", path.display()))?;
    let mut setups = Vec::new();
    let list = v["setups"]
        .as_array()
        .ok_or_else(|| format!("{}: no setups", path.display()))?;
    for (index, s) in list.iter().enumerate() {
        let transform = if let Some(name) = s["transform"].as_str() {
            Some(named_transform(name).ok_or_else(|| {
                format!(
                    "{}: setup {index} names transform {name:?}, which is not one of \
                     identity, rotate-z-90, rotate-z-180, flip-x, flip-y",
                    path.display()
                )
            })?)
        } else if s["transform"].is_array() {
            Some(rows_to_mat4(&s["transform"]).ok_or_else(|| {
                format!(
                    "{}: setup {index} has a malformed transform",
                    path.display()
                )
            })?)
        } else {
            None
        };
        setups.push(Setup {
            index: u32::try_from(index).unwrap_or(u32::MAX),
            name: s["name"]
                .as_str()
                .map_or_else(|| format!("setup-{index}"), str::to_owned),
            transform,
            program: rel(s["program"]
                .as_str()
                .ok_or_else(|| format!("{}: setup {index} names no program", path.display()))?),
            tools: rel(s["tools"]
                .as_str()
                .ok_or_else(|| format!("{}: setup {index} names no tools", path.display()))?),
            tool: s["tool"].as_str().map(str::to_owned),
            fixtures: s["fixtures"]
                .as_array()
                .map(|a| a.iter().filter_map(|f| f.as_str()).map(rel).collect())
                .unwrap_or_default(),
        });
    }
    if setups.is_empty() {
        return Err(format!(
            "{}: a job needs at least one setup",
            path.display()
        ));
    }
    // The first setup has nothing to be carried from, so a transform on it is a
    // mistake rather than a no-op: it would suggest the stock arrives rotated,
    // which is not what the field says.
    if setups[0].transform.is_some() {
        return Err(format!(
            "{}: setup 0 carries a transform, but there is no previous setup for it \
             to move the stock from. Place the stock where the first program expects it.",
            path.display()
        ));
    }

    Ok(Job {
        stock: rel(stock),
        units: v["units"].as_str().map(str::to_owned),
        tolerance_mm: v["tolerance_mm"].as_f64().unwrap_or(0.1),
        clearance_mm: v["clearance_mm"].as_f64().unwrap_or(0.0),
        setups,
    })
}

/// What one setup boundary cost, and how it was crossed.
#[derive(Debug, Clone)]
pub struct Crossing {
    /// Which setup the boundary leads into.
    pub into_setup: u32,
    /// How it was crossed.
    pub regime: Regime,
}

/// Carries a field into the next setup.
///
/// # Errors
///
/// Refuses an arbitrary rotation rather than resampling it. The general
/// resample is classified and bounded but not implemented, and the whole value
/// of the classification is that it can say no: falling back to something that
/// quietly claimed a zero bound would be the failure this design exists to
/// prevent.
pub fn carry(
    field: &TriDexelField,
    transform: &Mat4,
    spacing_mm: f64,
    into_setup: u32,
) -> Result<(TriDexelField, Crossing), String> {
    let regime = classify(transform, spacing_mm).ok_or_else(|| {
        format!(
            "setup {into_setup}: the transform is not a rigid motion, so \"the same \
             stock in a new orientation\" is not true of it and no bound would mean \
             anything"
        )
    })?;
    match regime {
        Regime::Exact { .. } => {
            let moved = refixture_exact(field, transform).ok_or_else(|| {
                format!(
                    "setup {into_setup}: the transform classified as axis-aligned but \
                     the field would not move across it. This is an internal \
                     inconsistency rather than a fault in the job file."
                )
            })?;
            Ok((moved, Crossing { into_setup, regime }))
        }
        Regime::Resampled { bound_mm } => Err(format!(
            "setup {into_setup}: this transform is not axis-aligned, so carrying the \
             stock across it would resample the field and cost up to {bound_mm:.4} mm. \
             That path is not implemented, and guessing at it would put a number in \
             the report that nothing measured. Use a quarter turn, a half turn or a \
             flip, or run the setups separately and accept two reports."
        )),
    }
}

/// Every input of a job, by content.
///
/// Fixtures included, and by digest rather than by file stem: two clamps sharing
/// a name are two clamps, and an identity that could not tell them apart would
/// be claiming otherwise.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn input_digests(job: &Job) -> Result<Vec<(String, String, String)>, String> {
    let mut out = Vec::new();
    let mut add = |role: String, p: &Path| -> Result<(), String> {
        let bytes = std::fs::read(p).map_err(|e| format!("cannot read {}: {e}", p.display()))?;
        out.push((role, p.display().to_string(), digest_bytes(&bytes)));
        Ok(())
    };
    add("stock".to_owned(), &job.stock)?;
    for s in &job.setups {
        add(format!("setup{}.program", s.index), &s.program)?;
        add(format!("setup{}.tools", s.index), &s.tools)?;
        for (n, f) in s.fixtures.iter().enumerate() {
            add(format!("setup{}.fixture{n}", s.index), f)?;
        }
    }
    Ok(out)
}

/// What one setup produced.
pub struct SetupOutcome {
    /// Which setup.
    pub index: u32,
    /// Its name, for the reader.
    pub name: String,
    /// Collisions found while replaying it.
    pub collisions: Vec<chipbreaker_core::findings::Collision>,
    /// Whether the collision gate could run at all, and why not.
    pub unchecked: Option<String>,
    /// How rapids were represented.
    pub rapid_path: chipbreaker_core::toolpath::RapidPath,
}

/// Runs a whole job: every setup in order, carrying the stock between them.
///
/// # Errors
/// Returns a message suitable for stderr.
#[allow(clippy::too_many_lines, reason = "one linear pass over a job")]
pub fn job(args: &JobArgs) -> Result<(Value, String, bool), String> {
    use chipbreaker_core::dexel::tri::TriBuildOptions;
    use chipbreaker_core::findings::collide::collision_count;
    use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
    use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};

    let started = std::time::Instant::now();
    let plan = load_job(&args.setups)?;
    let mut field = crate::run::read_stock_public(&plan.stock)?;
    let spacing = field.bundles().next().map_or(0.4, |(_, b)| {
        let uv = b.lattice().spacing_uv();
        uv[0].min(uv[1])
    });

    let mut crossings: Vec<Crossing> = Vec::new();
    let mut outcomes: Vec<SetupOutcome> = Vec::new();

    for setup in &plan.setups {
        // Carry the stock into this setup, if it is not the first.
        if let Some(t) = &setup.transform {
            let (moved, crossing) = carry(&field, t, spacing, setup.index)?;
            field = moved;
            crossings.push(crossing);
        }

        let replay = crate::run::resolve_for_collision(
            &setup.program,
            Some(setup.tools.as_path()),
            setup.tool.as_deref(),
        )?;

        // Fixtures are rebuilt per setup from their meshes, at this setup's
        // spacing. A fixture carried across by transform would have to be
        // resampled exactly as the stock does, and its mesh is right there.
        let unit = match &plan.units {
            Some(u) => Some(crate::mesh::parse_unit(u)?),
            None => None,
        };
        let mut fixtures = Vec::with_capacity(setup.fixtures.len());
        for f in &setup.fixtures {
            let (mesh, _) = crate::mesh::load(&crate::mesh::Input {
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
            fixtures.push((
                f.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("fixture")
                    .to_owned(),
                built,
            ));
        }

        let mut scratch = CutScratch::new(&replay.profile);
        let mut working = field.clone();
        let outcome = collide_with_stock(
            &mut working,
            &replay.profile,
            &replay.motions,
            &replay.kinds,
            &replay.provenance,
            replay.unmodelled_retracts,
            &fixtures,
            &CollideParams {
                clearance_mm: plan.clearance_mm,
                grid_mm: 2.0 * spacing,
                method: SweepMethod::Analytic {
                    tolerance: spacing / 10.0,
                },
            },
            &mut scratch,
        );
        let (mut collisions, unchecked) = match outcome {
            Ok(c) => (c, None),
            Err(u) => (Vec::new(), Some(u.to_string())),
        };
        // Stamp which setup named these lines. A line number alone is
        // ambiguous across a job, since each setup's program numbers its own
        // lines from one.
        for c in &mut collisions {
            c.attribution.setup = setup.index;
        }
        // The replay consumed the field as the program consumed it, which is
        // exactly the state the next setup starts from.
        field = working;
        outcomes.push(SetupOutcome {
            index: setup.index,
            name: setup.name.clone(),
            collisions,
            unchecked,
            rapid_path: replay.rapid_path,
        });
    }

    // The gouge gate runs once, on the finished stock, in the frame of the last
    // setup -- which is where the nominal part is expressed. Comparing after
    // each setup would flag every surface a later operation has still to reach,
    // and call an unfinished part defective.
    let gouge = match &args.nominal {
        Some(path) => {
            let unit = match &plan.units {
                Some(u) => Some(crate::mesh::parse_unit(u)?),
                None => None,
            };
            let (nominal, _) = crate::mesh::load(&crate::mesh::Input {
                file: path.clone(),
                units: unit,
                weld_tol: chipbreaker_core::eps::EPS_WELD,
                json: false,
            })?;
            // **The nominal must be expressed in the last setup's frame.** The
            // stock has been carried through every transform; the nominal has
            // not, because it is an input rather than a result.
            //
            // Supplying it in the first setup's frame is an easy mistake and it
            // used to produce a *clean pass*: the two solids do not overlap, so
            // nothing is sampled and nothing is found. A comparison that finds
            // nothing because it was looking somewhere else is the worst answer
            // available, so it is refused here instead.
            let (a, b) = (field.material_bounds(), nominal.bounds());
            if !a.intersects(&b) {
                return Err(format!(
                    "the nominal does not overlap the finished stock at all, so the                      comparison would sample nothing and report a clean part.
                       stock    {:?} .. {:?}
                       nominal  {:?} .. {:?}
                     The stock is carried through every setup transform and the                      nominal is not, so a nominal drawn in the first setup's frame                      will not line up. Express it in the last setup's frame.",
                    a.min.to_array(),
                    a.max.to_array(),
                    b.min.to_array(),
                    b.max.to_array()
                ));
            }
            let d = chipbreaker_core::deviation::compare(&field, &nominal, None);
            let params = chipbreaker_core::findings::cluster::ClusterParams::for_spacing(
                spacing,
                plan.tolerance_mm,
            );
            let clusters =
                chipbreaker_core::findings::cluster::cluster(&d.samples, &params, spacing);
            let findings = chipbreaker_core::findings::identify(clusters, params.radius_mm);
            let defects = findings.iter().filter(|f| f.is_defect()).count();
            let worst = findings
                .iter()
                .filter(|f| f.is_defect())
                .map(|f| f.worst_depth_mm)
                .fold(0.0f64, f64::max);
            Some((defects, worst))
        }
        None => None,
    };

    let elapsed = started.elapsed().as_secs_f64() * 1000.0;
    let total: usize = outcomes
        .iter()
        .map(|o| collision_count(&o.collisions))
        .sum();
    let any_unchecked = outcomes.iter().any(|o| o.unchecked.is_some());
    let accumulated: f64 = crossings.iter().map(|c| c.regime.bound_mm()).sum();
    // A conjunction over gates *and* over setups. A part is not acceptable
    // because two of its three setups were, and an unchecked gate has certified
    // nothing -- including the gouge gate, which is unchecked when no nominal
    // was given rather than quietly passing.
    let gouge_state = match gouge {
        Some((0, _)) => "pass",
        Some(_) => "fail",
        None => "unchecked",
    };
    let pass = total == 0 && !any_unchecked && gouge_state == "pass";

    let mut text = format!(
        "job        {}\nsetups     {}\nstock      {}\n\n",
        args.setups.display(),
        plan.setups.len(),
        plan.stock.display()
    );
    for o in &outcomes {
        if let Some(c) = crossings.iter().find(|c| c.into_setup == o.index) {
            text.push_str(&format!(
                "  boundary into setup {}: {}, bound {:.4} mm\n",
                c.into_setup,
                c.regime.as_str(),
                c.regime.bound_mm()
            ));
        }
        let hard = collision_count(&o.collisions);
        let near = o.collisions.len() - hard;
        match &o.unchecked {
            Some(w) => text.push_str(&format!(
                "  setup {} {:<14} UNCHECKED -- {w}\n",
                o.index, o.name
            )),
            None => text.push_str(&format!(
                "  setup {} {:<14} {hard} collision(s), {near} near miss(es), rapids {}\n",
                o.index,
                o.name,
                o.rapid_path.as_str()
            )),
        }
    }
    match gouge {
        Some((n, worst)) => text.push_str(&format!(
            "\ngouge      {n} finding(s), worst {worst:.4} mm\n"
        )),
        None => text.push_str("\ngouge      UNCHECKED -- no --nominal, so nothing was compared\n"),
    }
    text.push_str(&format!(
        "transform bound accumulated {accumulated:.4} mm over {} boundary(ies)\nverdict    {}\n",
        crossings.len(),
        if pass { "PASS" } else { "DOES NOT PASS" }
    ));

    let state = if any_unchecked {
        "unchecked"
    } else if total > 0 {
        "fail"
    } else {
        "pass"
    };
    let value = serde_json::json!({
        "schema": "chipbreaker.job-report",
        "schema_version": 1,
        "setups": outcomes.iter().map(|o| serde_json::json!({
            "index": o.index,
            "name": o.name,
            "checked": o.unchecked.is_none(),
            "unchecked_because": o.unchecked,
            "collisions": collision_count(&o.collisions),
            "near_misses": o.collisions.len() - collision_count(&o.collisions),
            "rapid_path": o.rapid_path.as_str(),
            "detail": o.collisions.iter().map(|c| serde_json::json!({
                "id": c.id,
                "contact": c.contact.as_str(),
                "role": c.role.as_str(),
                "obstacle": c.obstacle.kind(),
                "motion": c.motion.as_str(),
                "magnitude_mm": c.contact.magnitude(),
                "setup": c.attribution.setup,
                "lines": c.attribution.provenance.iter().map(|p| p.line).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
        "boundaries": crossings.iter().map(|c| serde_json::json!({
            "into_setup": c.into_setup,
            "regime": c.regime.as_str(),
            "bound_mm": c.regime.bound_mm(),
        })).collect::<Vec<_>>(),
        "accumulated_transform_bound_mm": accumulated,
        "inputs": input_digests(&plan)?.into_iter().map(|(role, path, digest)| {
            serde_json::json!({ "role": role, "path": path, "digest": digest })
        }).collect::<Vec<_>>(),
        "verdict": {
            "pass": pass,
            "gates": {
                "collision": { "state": state },
                "gouge": {
                    "state": gouge_state,
                    "why": match gouge {
                        Some((0, _)) => None,
                        Some((n, worst)) => Some(format!(
                            "{n} gouge(s) above tolerance, worst {worst:.4} mm"
                        )),
                        None => Some(
                            "no --nominal was given, so no comparison ran; a gate that \
                             did not run has not passed"
                                .to_owned(),
                        ),
                    },
                },
            },
            "rule": "a gate that fails in any setup fails the job, and an unchecked \
                     setup certifies nothing",
        },
        "environment": chipbreaker_core::findings::report::environment("local", elapsed),
    });
    Ok((value, text, pass))
}
