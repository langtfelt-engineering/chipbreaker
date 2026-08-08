// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! `chipbreaker verify` and `chipbreaker report-diff`.
//!
//! # `compare` answers a question; `verify` produces an artifact
//!
//! `compare` prints the worst gouge and the worst excess. That is the right
//! answer to "is this part acceptable" and the wrong shape for anything else:
//! it cannot be diffed, it does not say which line caused what, and six months
//! later it cannot be checked against the inputs that produced it.
//!
//! `verify` produces a report — findings with identities, each attributed to the
//! lines that could have caused it, a content-addressed manifest of every input,
//! and a statement of what the numbers are worth. It is the thing a quality
//! engineer reads during an audit, and it is designed for that reader first and
//! for the terminal second.
//!
//! # The exit code contract
//!
//! | code | means |
//! |---|---|
//! | 0 | no gouge above tolerance |
//! | 1 | a gouge above tolerance, or the run could not be completed |
//!
//! `report-diff` uses the same two codes for "identical" and "differs", which
//! is what makes it usable as a CI gate without parsing anything.

use std::path::PathBuf;

use chipbreaker_core::deviation::compare as deviation_compare;
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::dexel::{FieldFormat, io as dexel_io};
use chipbreaker_core::findings::cluster::{ClusterParams, cluster, unsampled};
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::findings::report::{
    InputHash, Manifest, Report, SCHEMA, SCHEMA_VERSION, SweptSplit, digest_bytes, environment,
    semantics_from,
};
use chipbreaker_core::findings::verdict::{self, Gate, GateOutcome, Verdict};
use chipbreaker_core::findings::{
    Attribution, Collision, Contact, Obstacle, attribute_finding, identify,
};
use chipbreaker_core::mesh::TriMesh;
use chipbreaker_core::sweep::Motion;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::profile::ElementRole;
use chipbreaker_core::toolpath::Provenance;
use chipbreaker_core::toolpath::{MotionKind, RapidPath};
use clap::Args;
use serde_json::{Value, json};

use crate::mesh::Input;

/// `chipbreaker verify ...`
#[derive(Debug, Args)]
pub struct VerifyArgs {
    /// The cut field to judge, as `.tdx`.
    pub file: PathBuf,
    /// The nominal part, as a mesh.
    #[arg(long, value_name = "FILE")]
    pub nominal: PathBuf,
    /// The stock the field was built from, for the tessellation floor.
    #[arg(long, value_name = "FILE")]
    pub stock: Option<PathBuf>,
    /// The NC program, so findings can name the line that caused them.
    #[arg(long, value_name = "FILE")]
    pub path: Option<PathBuf>,
    /// The tool library the program was cut with.
    #[arg(long, value_name = "FILE")]
    pub tools: Option<PathBuf>,
    /// Which tool, by library id.
    #[arg(long, value_name = "ID")]
    pub tool: Option<String>,
    /// Unit the meshes' coordinates are in.
    #[arg(long, value_name = "UNIT")]
    pub units: Option<String>,
    /// The tolerance to judge against, in millimetres.
    #[arg(long, value_name = "MM", default_value_t = 0.1)]
    pub tol: f64,
    /// How far apart two samples may be and still be one finding.
    ///
    /// Defaults to two cells, which is the smallest radius that can join
    /// samples from different bundles — three bundles put their samples on
    /// three interleaved lattices, and anything under one cell splits a single
    /// physical gouge into one finding per bundle.
    #[arg(long, value_name = "MM")]
    pub cluster_radius: Option<f64>,
    /// Report below the tessellation floor anyway.
    #[arg(long)]
    pub allow_below_floor: bool,
    /// The JSON report from `chipbreaker run`, for the swept-volume split.
    ///
    /// A field does not carry the statistics of the run that cut it, so without
    /// this the report says the split is unavailable rather than inventing
    /// zeros for it.
    #[arg(long, value_name = "FILE")]
    pub run_report: Option<PathBuf>,
    /// The **stock** field the program started from, for the collision gate.
    ///
    /// `verify` holds the cut field, and a collision is judged against the
    /// material present when each move runs. Without this the collision gate
    /// reports `unchecked` rather than a clear it never established.
    #[arg(long, value_name = "FILE")]
    pub stock_field: Option<PathBuf>,
    /// Static obstacles for the collision gate. Comma separated.
    #[arg(long, value_name = "FILES", value_delimiter = ',')]
    pub fixtures: Vec<PathBuf>,
    /// Report a near miss when the gap is below this, in millimetres.
    #[arg(long, value_name = "MM", default_value_t = 0.0)]
    pub clearance: f64,
    /// Where to write the report.
    #[arg(long, value_name = "FILE")]
    pub report: Option<PathBuf>,
    /// Emit JSON to standard output instead of text.
    #[arg(long)]
    pub json: bool,
}

/// `chipbreaker report-diff ...`
#[derive(Debug, Args)]
pub struct ReportDiffArgs {
    /// The earlier report.
    pub old: PathBuf,
    /// The later report.
    pub new: PathBuf,
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
            "{} is a single-bundle field; verification needs all three bundles",
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
    crate::mesh::load(&Input {
        file: path.to_path_buf(),
        units: unit,
        weld_tol: chipbreaker_core::eps::EPS_WELD,
        json: false,
    })
    .map(|(m, _)| m)
}

fn hash_of(path: &std::path::Path) -> Result<String, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    Ok(digest_bytes(&bytes))
}

/// The motions and their provenance, when a program was supplied.
fn load_program(
    path: &std::path::Path,
    tools: Option<&std::path::Path>,
    tool: Option<&str>,
) -> Result<(Vec<Motion>, Vec<Provenance>, Profile), String> {
    let (motions, provenance, profile) = crate::run::resolve_for_attribution(path, tools, tool)?;
    Ok((motions, provenance, profile))
}

/// Runs `chipbreaker verify`.
///
/// # Errors
/// Returns a message suitable for stderr.
#[allow(clippy::too_many_lines, reason = "one linear assembly of one artifact")]
pub fn verify(args: &VerifyArgs) -> Result<(Value, String, bool), String> {
    if !args.tol.is_finite() || args.tol <= 0.0 {
        return Err(format!(
            "--tol must be a positive number of millimetres, got {}",
            args.tol
        ));
    }
    let started = std::time::Instant::now();

    let field = read_field(&args.file)?;
    let nominal_mesh = read_mesh(&args.nominal, args.units.as_deref())?;
    let stock_mesh = match &args.stock {
        Some(p) => Some(read_mesh(p, args.units.as_deref())?),
        None => None,
    };

    let d = deviation_compare(&field, &nominal_mesh, stock_mesh.as_ref());
    if d.below_floor(args.tol) && !args.allow_below_floor {
        return Err(format!(
            "a tolerance of {:.4} mm is below the {:.4} mm floor these inputs support, \
             so any finding at that scale would describe the inputs and not the part.\n\
             \x20 stock facets   {:.4} mm\n\
             \x20 nominal facets {:.4} mm\n\
             \x20 lattice        {:.4} mm\n\
             Refine the coarsest of the three, or pass --allow-below-floor.",
            args.tol,
            d.tolerance_floor_mm(),
            d.stock_facet_mm,
            d.nominal_facet_mm,
            d.spacing_mm,
        ));
    }

    let spacing = d.spacing_mm;
    let params = ClusterParams {
        radius_mm: args.cluster_radius.unwrap_or(2.0 * spacing),
        tolerance_mm: args.tol,
    };

    let mut clusters = cluster(&d.samples, &params, spacing);
    clusters.extend(unsampled(&nominal_mesh, &d.samples, &params));
    chipbreaker_core::findings::cluster::sort_canonically(&mut clusters);
    let mut findings = identify(clusters, params.radius_mm);

    // Attribution, only where there is a finding to attribute. This is the
    // choice recorded in `findings::attribute`: recompute for the rare regions
    // that matter rather than carry four bytes per endpoint through every field
    // the engine ever builds.
    let mut attributed = 0usize;
    let mut ambiguous = 0usize;
    if let Some(program) = &args.path {
        let (motions, provenance, profile) =
            load_program(program, args.tools.as_deref(), args.tool.as_deref())?;
        let bounds: Vec<_> = motions.iter().map(|m| m.swept_bounds(&profile)).collect();
        let method = SweepMethod::Analytic {
            tolerance: spacing / 10.0,
        };
        let mut scratch = CutScratch::new(&profile);
        for f in &mut findings {
            // Attribution answers "which segment cut this", so it applies to
            // findings the *cut* produced. An undercut is a property of the part
            // and no segment caused it; saying otherwise would name a line at
            // random.
            if !matches!(
                f.class,
                chipbreaker_core::findings::Classification::Gouge
                    | chipbreaker_core::findings::Classification::ExcessStock
            ) {
                continue;
            }
            // Several points across the finding, unioned. A centroid is an
            // average and need not lie on any swept surface at all; a single
            // deepest point lies on one, but not always the segment a user needs
            // to edit.
            let a = attribute_finding(
                &profile,
                &motions,
                &bounds,
                &provenance,
                method,
                &mut scratch,
                &f.probes,
            );
            if !a.is_empty() {
                attributed += 1;
            }
            if a.is_ambiguous() {
                ambiguous += 1;
            }
            f.attribution = a;
        }
    } else {
        for f in &mut findings {
            f.attribution = Attribution::none();
        }
    }

    let mut inputs = vec![
        InputHash {
            role: "field".to_owned(),
            path: args.file.display().to_string(),
            digest: hash_of(&args.file)?,
        },
        InputHash {
            role: "nominal".to_owned(),
            path: args.nominal.display().to_string(),
            digest: hash_of(&args.nominal)?,
        },
    ];
    for (role, p) in [
        ("stock", args.stock.as_ref()),
        ("program", args.path.as_ref()),
        ("tools", args.tools.as_ref()),
    ] {
        if let Some(p) = p {
            inputs.push(InputHash {
                role: role.to_owned(),
                path: p.display().to_string(),
                digest: hash_of(p)?,
            });
        }
    }
    inputs.sort_by(|a, b| a.role.cmp(&b.role));

    let selftest = {
        let report = chipbreaker_core::selftest::run_with(chipbreaker_gcode::selftest::suites());
        let mut h = chipbreaker_core::golden::CanonicalHash::new();
        chipbreaker_core::golden::Hashable::hash_canonical(&report, &mut h);
        h.finish().to_hex()
    };

    let manifest = Manifest {
        inputs,
        spacing_mm: [spacing, spacing, spacing],
        tolerance_mm: args.tol,
        cluster_radius_mm: params.radius_mm,
        engine_version: env!("CARGO_PKG_VERSION").to_owned(),
        engine_selftest: selftest,
    };
    let sweep = match &args.run_report {
        Some(p) => Some(read_sweep_split(p)?),
        None => None,
    };
    let semantics = semantics_from(&d, manifest.spacing_mm, args.tol, sweep);

    // The collision gate needs the **stock** field, because a collision is
    // judged against the material present when each move runs. `verify` holds
    // the cut field, so without `--stock-field` there is nothing here to replay
    // against and the gate says so rather than reporting a clear it never
    // established.
    let (collisions, collision_gate, rapid_path) =
        collision_gate(args, spacing).unwrap_or_else(|e| {
            (
                Vec::new(),
                GateOutcome::unchecked(format!("collision checking could not run: {e}")),
                None,
            )
        });

    let verdict = Verdict::new()
        .with(verdict::GATE_GOUGE, Report::gouge_gate(&findings))
        .with(verdict::GATE_COLLISION, collision_gate);
    let report = Report {
        manifest,
        semantics,
        findings,
        collisions,
        verdict,
        rapid_path,
    };

    if let Some(out) = &args.report {
        let text = serde_json::to_string_pretty(&report.to_json())
            .map_err(|e| format!("cannot render the report: {e}"))?;
        std::fs::write(out, text + "\n")
            .map_err(|e| format!("cannot write {}: {e}", out.display()))?;
    }

    let c = report.counts();
    let elapsed = started.elapsed().as_secs_f64() * 1000.0;
    let text = format!(
        "field      {}\n\
         nominal    {}\n\
         manifest   {}\n\
         \n\
         GOUGE          {} finding(s)\n\
         EXCESS STOCK   {} finding(s)   (expected on a roughing pass; not a defect on its own)\n\
         UNDERCUT       {} finding(s)   (unreachable at this setup, whatever the program)\n\
         UNREACHABLE    {} finding(s)   (no ray sampled it; absence of evidence)\n\
         \n\
         attributed {attributed} of {} cut findings, {ambiguous} ambiguous\n\
         \n\
         {}\
         \n\
         {}\n",
        args.file.display(),
        args.nominal.display(),
        report.manifest.digest(),
        c[0],
        c[1],
        c[2],
        c[3],
        c[0] + c[1],
        gate_lines(&report.verdict),
        chipbreaker_core::findings::report::SCOPE_STATEMENT,
    );

    let mut value = report.to_json();
    if let Value::Object(map) = &mut value {
        map.insert("environment".to_owned(), environment("local", elapsed));
        map.insert("schema".to_owned(), json!(SCHEMA));
    }
    Ok((value, text, report.verdict.pass()))
}

/// Runs `chipbreaker report-diff`.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn report_diff(args: &ReportDiffArgs) -> Result<(Value, String, bool), String> {
    let old = crate::verify::load_report(&args.old)?;
    let new = crate::verify::load_report(&args.new)?;
    let d = chipbreaker_core::findings::diff::diff(&old, &new);
    let (appeared, disappeared, changed) = d.tally();

    let mut text = format!(
        "old        {}\nnew        {}\n\n",
        args.old.display(),
        args.new.display()
    );
    if d.manifest.is_empty() {
        text.push_str("manifest   identical\n\n");
    } else {
        text.push_str("manifest   DIFFERS -- this may explain every finding below\n");
        for (k, a, b) in &d.manifest {
            text.push_str(&format!("             {k}: {a} -> {b}\n"));
        }
        text.push('\n');
    }
    text.push_str(&format!(
        "findings   {appeared} appeared, {disappeared} disappeared, {changed} changed\n"
    ));
    for c in &d.changes {
        match c {
            chipbreaker_core::findings::Change::Appeared(f) => text.push_str(&format!(
                "  + {} {:<12} {:.4} mm\n",
                f.id,
                f.class.as_str(),
                f.worst_depth_mm
            )),
            chipbreaker_core::findings::Change::Disappeared(f) => text.push_str(&format!(
                "  - {} {:<12} {:.4} mm\n",
                f.id,
                f.class.as_str(),
                f.worst_depth_mm
            )),
            chipbreaker_core::findings::Change::Changed { before, after } => {
                text.push_str(&format!(
                    "  ~ {} {:<12} {:.4} -> {:.4} mm\n",
                    after.id,
                    after.class.as_str(),
                    before.worst_depth_mm,
                    after.worst_depth_mm
                ));
            }
        }
    }
    // Gates before collisions and after findings: a gate change with no finding
    // behind it -- pass to unchecked, say -- would otherwise show as nothing at
    // all, which is the most misleading possible diff.
    if !d.gates.is_empty() {
        text.push_str("\ngates      CHANGED\n");
        for (k, a, b) in &d.gates {
            text.push_str(&format!("             {k}: {a} -> {b}\n"));
        }
    }
    if !d.collisions.is_empty() {
        text.push_str(&format!("\ncollisions {} changed\n", d.collisions.len()));
        for c in &d.collisions {
            use chipbreaker_core::findings::diff::CollisionChange as Cc;
            let line = |mark: char, x: &chipbreaker_core::findings::Collision| {
                format!(
                    "  {mark} {} {:<10} {:<9} {:.4} mm\n",
                    x.id,
                    x.contact.as_str(),
                    x.motion.as_str(),
                    x.contact.magnitude()
                )
            };
            match c {
                Cc::Appeared(x) => text.push_str(&line('+', x)),
                Cc::Disappeared(x) => text.push_str(&line('-', x)),
                Cc::Changed { before, after } => text.push_str(&format!(
                    "  ~ {} {:<10} {:<9} {:.4} -> {:.4} mm\n",
                    after.id,
                    after.contact.as_str(),
                    after.motion.as_str(),
                    before.contact.magnitude(),
                    after.contact.magnitude()
                )),
            }
        }
    }
    if d.is_empty() {
        text.push_str("\nidentical\n");
    }
    Ok((
        chipbreaker_core::findings::diff::to_json(&d),
        text,
        d.is_empty(),
    ))
}

/// Reads the swept-volume split out of a `chipbreaker run --json` report.
fn read_sweep_split(path: &std::path::Path) -> Result<SweptSplit, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text)
        .map_err(|e| format!("{} is not valid JSON: {e}", path.display()))?;
    let s = &v["results"]["sweep"];
    if s.is_null() {
        return Err(format!(
            "{} has no `results.sweep` section, so it is not a report from              `chipbreaker run --json`",
            path.display()
        ));
    }
    Ok(SweptSplit {
        ray_cuts_exact: s["rays_exact"].as_u64().unwrap_or(0),
        ray_cuts_bounded: s["rays_substepped"].as_u64().unwrap_or(0),
        worst_bound_mm: s["worst_bound_mm"].as_f64().unwrap_or(0.0),
    })
}

/// Reads a report back from JSON.
///
/// Only the fields a diff needs are reconstructed. A report is written for a
/// reader and consumed for a comparison, and the comparison does not need the
/// prose.
/// Runs the collision gate for `verify`, when it has what it needs.
///
/// Returns the collisions, the gate, and the rapid policy that was replayed.
/// Every path that cannot produce an answer returns `unchecked` with a reason —
/// never a pass.
fn collision_gate(
    args: &VerifyArgs,
    spacing: f64,
) -> Result<(Vec<Collision>, GateOutcome, Option<RapidPath>), String> {
    let Some(stock_field) = &args.stock_field else {
        return Ok((
            Vec::new(),
            GateOutcome::unchecked(
                "verify holds the cut field, and a collision is judged against the material \
                 present when each move runs; pass --stock-field with the field the program \
                 started from, or run `chipbreaker collide`",
            ),
            None,
        ));
    };
    let Some(program) = &args.path else {
        return Ok((
            Vec::new(),
            GateOutcome::unchecked("no --path, so there is no program to replay"),
            None,
        ));
    };

    let mut field = read_field(stock_field)?;
    let replay =
        crate::run::resolve_for_collision(program, args.tools.as_deref(), args.tool.as_deref())?;
    let rapid = replay.rapid_path;

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
        fixtures.push((
            f.file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("fixture")
                .to_owned(),
            built,
        ));
    }

    let mut scratch = CutScratch::new(&replay.profile);
    match collide_with_stock(
        &mut field,
        &replay.profile,
        &replay.motions,
        &replay.kinds,
        &replay.provenance,
        replay.unmodelled_retracts,
        &fixtures,
        &CollideParams {
            clearance_mm: args.clearance,
            grid_mm: 2.0 * spacing,
            method: SweepMethod::Analytic {
                tolerance: spacing / 10.0,
            },
        },
        &mut scratch,
    ) {
        Ok(c) => {
            let gate = Report::collision_gate(&c);
            Ok((c, gate, Some(rapid)))
        }
        Err(u) => Ok((
            Vec::new(),
            GateOutcome::unchecked(u.to_string()),
            Some(rapid),
        )),
    }
}

/// Every gate on its own line, with the reason when there is one.
///
/// One line per gate rather than a single summary word. A reader who sees only
/// "FAIL" has to open the JSON to find out which gate failed, and a reader who
/// sees only "PASS" cannot tell a run where everything was checked from one
/// where half of it was skipped — which is the distinction this whole schema
/// change exists to make visible.
fn gate_lines(v: &Verdict) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    for (name, g) in v.gates() {
        let label = match g.state {
            Gate::Pass => "pass     ",
            Gate::Fail => "FAIL     ",
            Gate::Unchecked => "unchecked",
        };
        let _ = write!(out, "gate       {label} {name}");
        if let Some(w) = &g.why {
            let _ = write!(out, " -- {w}");
        }
        out.push('\n');
    }
    out.push_str(if v.pass() {
        "verdict    PASS\n"
    } else {
        "verdict    DOES NOT PASS\n"
    });
    out
}

pub fn load_report(path: &std::path::Path) -> Result<Report, String> {
    use chipbreaker_core::findings::{Classification, Finding};
    use chipbreaker_core::math::{Aabb3, Vec3};

    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text)
        .map_err(|e| format!("{} is not valid JSON: {e}", path.display()))?;
    if v["schema"].as_str() != Some(SCHEMA) {
        return Err(format!(
            "{} is not a Chipbreaker verification report (schema is {:?})",
            path.display(),
            v["schema"].as_str().unwrap_or("absent")
        ));
    }

    // The version check is not politeness. Version 1 carried `accepted`, which
    // version 2 removed; reading a version-1 file with version-2 code would find
    // no verdict and quietly report one it invented. Refusing is the entire
    // point of having renamed the field -- a consumer that cannot read a report
    // must say so, not guess.
    let version = v["schema_version"].as_u64();
    if version != Some(u64::from(SCHEMA_VERSION)) {
        return Err(format!(
            "{} is a schema version {} report and this build reads version {}.              Version 1 carried `accepted`, which version 2 replaced with `verdict.gates`;              regenerate the report rather than reading it with the wrong reader.",
            path.display(),
            version.map_or_else(|| "absent".to_owned(), |n| n.to_string()),
            SCHEMA_VERSION,
        ));
    }

    let num = |x: &Value| x.as_f64().unwrap_or(0.0);
    let m = &v["manifest"];
    let inputs = m["inputs"]
        .as_array()
        .map(|a| {
            a.iter()
                .map(|i| InputHash {
                    role: i["role"].as_str().unwrap_or_default().to_owned(),
                    path: i["path"].as_str().unwrap_or_default().to_owned(),
                    digest: i["digest"].as_str().unwrap_or_default().to_owned(),
                })
                .collect()
        })
        .unwrap_or_default();
    let sp = m["spacing_mm"]
        .as_array()
        .map_or([0.0; 3], |a| [num(&a[0]), num(&a[1]), num(&a[2])]);

    let mut findings = Vec::new();
    for f in v["findings"].as_array().into_iter().flatten() {
        let class = match f["class"].as_str().unwrap_or_default() {
            "gouge" => Classification::Gouge,
            "excess-stock" => Classification::ExcessStock,
            "undercut" => Classification::Undercut,
            _ => Classification::Unreachable,
        };
        let at = f["at"].as_array().map_or(Vec3::ZERO, |a| {
            Vec3::new(num(&a[0]), num(&a[1]), num(&a[2]))
        });
        let seg = f["attribution"]["segments"]
            .as_array()
            .map(|a| {
                a.iter()
                    .map(|s| {
                        (
                            u32::try_from(s["segment"].as_u64().unwrap_or(0)).unwrap_or(0),
                            Provenance {
                                file: u32::try_from(s["file"].as_u64().unwrap_or(0)).unwrap_or(0),
                                line: u32::try_from(s["line"].as_u64().unwrap_or(0)).unwrap_or(0),
                                block: u32::try_from(s["block"].as_u64().unwrap_or(0)).unwrap_or(0),
                                cycle_step: u32::try_from(s["cycle_step"].as_u64().unwrap_or(
                                    u64::from(chipbreaker_core::toolpath::NOT_A_CYCLE_STEP),
                                ))
                                .unwrap_or(chipbreaker_core::toolpath::NOT_A_CYCLE_STEP),
                            },
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let worst_at = f["worst_at"]
            .as_array()
            .map_or(at, |a| Vec3::new(num(&a[0]), num(&a[1]), num(&a[2])));
        findings.push(Finding {
            id: f["id"].as_str().unwrap_or_default().to_owned(),
            class,
            worst_depth_mm: num(&f["severity"]["worst_depth_mm"]),
            mean_depth_mm: num(&f["severity"]["mean_depth_mm"]),
            area_mm2: num(&f["severity"]["area_mm2"]),
            volume_mm3: num(&f["severity"]["volume_mm3"]),
            sample_count: usize::try_from(f["sample_count"].as_u64().unwrap_or(0)).unwrap_or(0),
            at,
            worst_at,
            // A report does not carry probe positions: they exist to attribute a
            // finding, and a finding read back from JSON has already been
            // attributed. Reconstructing them would invite somebody to re-run
            // attribution against a report rather than against a field.
            probes: Vec::new(),
            bounds: Aabb3::EMPTY,
            attribution: Attribution {
                segments: seg.iter().map(|(s, _)| *s).collect(),
                provenance: seg.iter().map(|(_, p)| *p).collect(),
            },
        });
    }

    let parse_segments = |a: &Value| -> (Vec<u32>, Vec<Provenance>) {
        let pairs: Vec<_> = a
            .as_array()
            .map(|a| {
                a.iter()
                    .map(|s| {
                        (
                            u32::try_from(s["segment"].as_u64().unwrap_or(0)).unwrap_or(0),
                            Provenance {
                                file: u32::try_from(s["file"].as_u64().unwrap_or(0)).unwrap_or(0),
                                line: u32::try_from(s["line"].as_u64().unwrap_or(0)).unwrap_or(0),
                                block: u32::try_from(s["block"].as_u64().unwrap_or(0)).unwrap_or(0),
                                cycle_step: u32::try_from(s["cycle_step"].as_u64().unwrap_or(
                                    u64::from(chipbreaker_core::toolpath::NOT_A_CYCLE_STEP),
                                ))
                                .unwrap_or(chipbreaker_core::toolpath::NOT_A_CYCLE_STEP),
                            },
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        (
            pairs.iter().map(|(s, _)| *s).collect(),
            pairs.iter().map(|(_, p)| *p).collect(),
        )
    };

    let mut collisions = Vec::new();
    for c in v["collisions"].as_array().into_iter().flatten() {
        let sev = &c["severity"];
        // The two quantities live under different keys precisely so that a
        // reader cannot mistake one for the other, and this is the reader.
        let contact = if sev["penetration_mm"].is_null() {
            Contact::NearMiss {
                clearance_mm: num(&sev["clearance_mm"]),
            }
        } else {
            Contact::Collision {
                penetration_mm: num(&sev["penetration_mm"]),
            }
        };
        let at = c["at"].as_array().map_or(Vec3::ZERO, |a| {
            Vec3::new(num(&a[0]), num(&a[1]), num(&a[2]))
        });
        let (segments, provenance) = parse_segments(&c["attribution"]["segments"]);
        collisions.push(Collision {
            id: c["id"].as_str().unwrap_or_default().to_owned(),
            contact,
            role: match c["element"]["role"].as_str().unwrap_or_default() {
                "cutting" => ElementRole::Cutting,
                "holder" => ElementRole::Holder,
                _ => ElementRole::NonCutting,
            },
            element_index: u32::try_from(c["element"]["index"].as_u64().unwrap_or(0)).unwrap_or(0),
            obstacle: if c["obstacle"]["kind"].as_str() == Some("fixture") {
                Obstacle::Fixture {
                    index: u32::try_from(c["obstacle"]["index"].as_u64().unwrap_or(0)).unwrap_or(0),
                    name: c["obstacle"]["name"]
                        .as_str()
                        .unwrap_or_default()
                        .to_owned(),
                }
            } else {
                Obstacle::Stock
            },
            at,
            bounds: Aabb3::EMPTY,
            motion: match c["motion"].as_str().unwrap_or_default() {
                "rapid" => MotionKind::Rapid,
                "arc" => MotionKind::Arc,
                "helix" => MotionKind::Helix,
                _ => MotionKind::Linear,
            },
            attribution: Attribution {
                segments,
                provenance,
            },
        });
    }

    let mut verdict = Verdict::new();
    for (name, g) in v["verdict"]["gates"].as_object().into_iter().flatten() {
        let raw = g["state"].as_str().unwrap_or_default();
        // An unreadable state is not a pass. A newer writer may use a word this
        // build has never seen, and guessing "fine" about it is how a tool
        // certifies what it did not check.
        let state = Gate::parse(raw).unwrap_or(Gate::Unchecked);
        let why = g["why"].as_str().map(str::to_owned).or_else(|| {
            (state != Gate::Pass && Gate::parse(raw).is_none())
                .then(|| format!("unrecognised gate state {raw:?}, read as unchecked"))
        });
        verdict = verdict.with(name, GateOutcome { state, why });
    }

    let semantics = semantics_from(
        &chipbreaker_core::deviation::DeviationField::default(),
        sp,
        num(&m["tolerance_mm"]),
        None,
    );
    Ok(Report {
        manifest: Manifest {
            inputs,
            spacing_mm: sp,
            tolerance_mm: num(&m["tolerance_mm"]),
            cluster_radius_mm: num(&m["cluster_radius_mm"]),
            engine_version: m["engine_version"].as_str().unwrap_or_default().to_owned(),
            engine_selftest: m["engine_selftest"].as_str().unwrap_or_default().to_owned(),
        },
        semantics,
        verdict,
        findings,
        collisions,
        rapid_path: v["rapid_path"]["policy"]
            .as_str()
            .and_then(RapidPath::parse),
    })
}
