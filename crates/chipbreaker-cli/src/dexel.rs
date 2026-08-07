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
//! `stat` answers "what is in this file", `slice` draws a cross-section, and
//! `volume` answers "how much material" — though ADR 0005 is emphatic that
//! volume is a **diagnostic**, not an accuracy metric. `deviation` and
//! `coverage` are the accuracy commands: the first regenerates the deviation
//! table, the second checks the `1/sqrt(3)` sampling guarantee directly.

use std::path::PathBuf;

use chipbreaker_core::budget::{Budget, Footprint, Spacing, auto_spacing, human, ray_counts};
use chipbreaker_core::dexel::convergence::{
    ErrorModel, GAUSS_CIRCLE_EXPONENT, measure as measure_convergence, standard_cases,
    standard_ratios,
};
use chipbreaker_core::dexel::deviation::{
    coverage as measure_coverage, measure as measure_deviation, sample_mesh_budget,
};
use chipbreaker_core::dexel::tri::{
    AXES, AxisSet, TriBuildOptions, TriDexelField, WORST_CASE_COSINE,
};
use chipbreaker_core::dexel::{DexelField, FieldFormat, TriBuildStats, io as dexel_io};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::{Axis, Mat4, Vec3};
use chipbreaker_core::mesh::TriMesh;
use clap::{Args, Subcommand};
use serde_json::{Value, json};

use crate::mesh::Input;

/// Surface points used by `deviation`. Enough to find the worst region.
const DEVIATION_SAMPLES: usize = 20_000;

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
    #[arg(long = "res", alias = "spacing", value_name = "MM")]
    pub res: f64,
    /// Which bundles to build, as a subset of `xyz`.
    ///
    /// Only `xyz` carries the `1/sqrt(3)` sampling guarantee: with two bundles a
    /// surface normal perpendicular to both is sampled by neither.
    #[arg(long, default_value = "xyz", value_parser = parse_axes)]
    pub axes: AxisSet,
    /// Where the stock sits in machine coordinates, as `x,y,z` millimetres.
    #[arg(long, value_parser = parse_vec3)]
    pub at: Option<Vec3>,
    /// Extra room around the stock bounds, in millimetres.
    #[arg(long, default_value_t = 0.0, value_name = "MM")]
    pub margin: f64,
    /// Cell size along X, overriding `--res` for that axis alone.
    #[arg(long, value_name = "MM")]
    pub res_x: Option<f64>,
    /// Cell size along Y.
    #[arg(long, value_name = "MM")]
    pub res_y: Option<f64>,
    /// Cell size along Z.
    #[arg(long, value_name = "MM")]
    pub res_z: Option<f64>,
    /// Choose the three spacings automatically, holding the accuracy of `--res`.
    ///
    /// Picks the spacings that minimise memory **subject to the worst-case
    /// sample distance being no worse than `--res` would have given**. So the
    /// guarantee is never quietly weakened: a part that cannot benefit, such as
    /// a cube, simply comes back isotropic, and one that can — a plate, a bar —
    /// gets the saving for free.
    #[arg(long)]
    pub auto_res: bool,
    /// Refuse the job if it would need more than this, e.g. `512M` or `2G`.
    #[arg(long, value_name = "BYTES", value_parser = parse_bytes)]
    pub mem_limit: Option<u64>,
    /// Predict the footprint and exit without building.
    #[arg(long)]
    pub mem_dry_run: bool,
}

/// Parses `512M`, `2G`, `1048576`.
pub fn parse_bytes(s: &str) -> Result<u64, String> {
    let t = s.trim();
    let (digits, scale) = match t.chars().last() {
        Some('k' | 'K') => (&t[..t.len() - 1], 1024u64),
        Some('m' | 'M') => (&t[..t.len() - 1], 1024 * 1024),
        Some('g' | 'G') => (&t[..t.len() - 1], 1024 * 1024 * 1024),
        _ => (t, 1),
    };
    let n: f64 = digits
        .trim()
        .parse()
        .map_err(|_| format!("expected a byte count such as 512M or 2G; got {s:?}"))?;
    if !n.is_finite() || n <= 0.0 {
        return Err(format!("a memory limit must be positive; got {s:?}"));
    }
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "checked positive and finite"
    )]
    Ok((n * scale as f64) as u64)
}

impl BuildArgs {
    /// The explicit per-axis spacings, if any were given.
    ///
    /// `--auto-res` is resolved later, once the stock's extents are known, so it
    /// is not visible here.
    fn spacing_xyz(&self) -> Option<Spacing> {
        if self.res_x.is_none() && self.res_y.is_none() && self.res_z.is_none() {
            return None;
        }
        Some(Spacing {
            x: self.res_x.unwrap_or(self.res),
            y: self.res_y.unwrap_or(self.res),
            z: self.res_z.unwrap_or(self.res),
        })
    }

    fn options(&self) -> TriBuildOptions {
        TriBuildOptions {
            spacing: self.res,
            spacing_xyz: self.spacing_xyz(),
            axes: self.axes,
            placement: self.at.map_or(Mat4::IDENTITY, Mat4::from_translation),
            margin: self.margin,
        }
    }
}

/// `chipbreaker dexel ...`
#[derive(Debug, Subcommand)]
pub enum DexelCommand {
    /// Build a field from a stock mesh and write it as `.tdx` (or `.dexel`).
    Build {
        #[command(flatten)]
        build: BuildArgs,
        /// Where to write the field.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Describe a field file: lattice, occupancy, span distribution.
    Stat {
        /// The field to read.
        file: PathBuf,
        /// Break the report down per bundle.
        #[arg(long)]
        per_axis: bool,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Report the material volume of a field.
    ///
    /// **A diagnostic, not an accuracy metric.** See ADR 0005 and `deviation`.
    Volume {
        /// A field file to read.
        file: PathBuf,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Draw a cross-section of a field as SVG.
    ///
    /// The answer to "the binary format is not human-readable".
    Slice {
        /// A field file to read.
        file: PathBuf,
        /// The cutting plane, as `Z=12.5`.
        #[arg(long, value_name = "AXIS=MM", value_parser = parse_plane)]
        at: (Axis, f64),
        /// Which bundle to draw from. Defaults to the plane's own axis.
        #[arg(long, value_parser = parse_axis)]
        axis: Option<Axis>,
        /// Where to write the SVG.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Measure surface deviation against the mesh the field was built from.
    ///
    /// **The accuracy metric.** Regenerates the published table rather than
    /// asserting it, so the numbers are reproducible on a customer's own part.
    Deviation {
        /// A field file to read.
        file: PathBuf,
        /// The mesh to measure against.
        #[arg(long, value_name = "FILE")]
        mesh: PathBuf,
        /// Unit the mesh's coordinates are in.
        #[arg(long)]
        units: Option<String>,
        /// Show the per-bundle columns, which make the anisotropy visible.
        #[arg(long)]
        per_axis: bool,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Report the worst-case sampling angle over a mesh's surface.
    ///
    /// The direct check on the `1/sqrt(3)` guarantee: no surface normal should
    /// be sampled worse than 54.7356 degrees by the best of three bundles.
    Coverage {
        /// A field file to read, for its axis set.
        file: PathBuf,
        /// The mesh whose surface to check.
        #[arg(long, value_name = "FILE")]
        mesh: PathBuf,
        /// Unit the mesh's coordinates are in.
        #[arg(long)]
        units: Option<String>,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Run the volume convergence measurement.
    ///
    /// Kept because it is a useful diagnostic and because its non-monotonicity
    /// is the evidence behind ADR 0005.
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
            | Self::Deviation { json, .. }
            | Self::Coverage { json, .. }
            | Self::Convergence { json } => *json,
        }
    }
}

fn parse_axis(s: &str) -> Result<Axis, String> {
    Axis::from_name(s).ok_or_else(|| format!("expected x, y or z; got {s:?}"))
}

fn parse_axes(s: &str) -> Result<AxisSet, String> {
    AxisSet::parse(s)
        .ok_or_else(|| format!("expected a subset of xyz, such as xyz or xz; got {s:?}"))
}

fn parse_plane(s: &str) -> Result<(Axis, f64), String> {
    let (name, value) = s
        .split_once('=')
        .ok_or_else(|| format!("expected AXIS=VALUE, such as Z=12.5; got {s:?}"))?;
    let axis = parse_axis(name.trim())?;
    let at = value
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("{value:?}: {e}"))?;
    if !at.is_finite() {
        return Err(format!(
            "the cutting plane must be at a finite coordinate, got {at}"
        ));
    }
    Ok((axis, at))
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
        DexelCommand::Stat { file, per_axis, .. } => run_stat(file, *per_axis),
        DexelCommand::Volume { file, .. } => run_volume(file),
        DexelCommand::Slice {
            file,
            at,
            axis,
            out,
            ..
        } => run_slice(file, *at, *axis, out.as_deref()),
        DexelCommand::Deviation {
            file,
            mesh,
            units,
            per_axis,
            ..
        } => run_deviation(file, mesh, units.as_deref(), *per_axis),
        DexelCommand::Coverage {
            file, mesh, units, ..
        } => run_coverage(file, mesh, units.as_deref()),
        DexelCommand::Convergence { .. } => run_convergence(),
    }
}

/// Either kind of field file, so both formats stay usable everywhere.
enum Loaded {
    Single(Box<DexelField>),
    Tri(Box<TriDexelField>),
}

impl Loaded {
    fn as_tri(&self) -> Option<&TriDexelField> {
        match self {
            Self::Tri(f) => Some(f),
            Self::Single(_) => None,
        }
    }

    fn bundles(&self) -> Vec<(Axis, &DexelField)> {
        match self {
            Self::Single(f) => vec![(f.lattice().axis(), f.as_ref())],
            Self::Tri(f) => f.bundles().collect(),
        }
    }

    fn axes(&self) -> AxisSet {
        match self {
            Self::Single(f) => AxisSet::parse(f.lattice().axis().as_str()).unwrap_or(AxisSet::XYZ),
            Self::Tri(f) => f.axes(),
        }
    }

    fn digest(&self) -> String {
        let mut h = CanonicalHash::new();
        match self {
            Self::Single(f) => h.add(f.as_ref()),
            Self::Tri(f) => h.add(f.as_ref()),
        };
        h.finish().to_hex()
    }
}

fn read_field(file: &std::path::Path) -> Result<Loaded, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
    match dexel_io::detect(&bytes) {
        Some(FieldFormat::Single) => dexel_io::from_bytes(&bytes)
            .map(|f| Loaded::Single(Box::new(f)))
            .map_err(|e| format!("{}: {e}", file.display())),
        Some(FieldFormat::Tri) => dexel_io::tri_from_bytes(&bytes)
            .map(|f| Loaded::Tri(Box::new(f)))
            .map_err(|e| format!("{}: {e}", file.display())),
        None => Err(format!(
            "{} is not a Chipbreaker field file: it starts with neither the .dexel \
             nor the .tdx magic",
            file.display()
        )),
    }
}

fn load_mesh(path: &std::path::Path, units: Option<&str>) -> Result<TriMesh, String> {
    let input = Input {
        file: path.to_path_buf(),
        units: units
            .map(|u| {
                chipbreaker_core::mesh::units::Unit::from_name(u)
                    .ok_or_else(|| format!("unknown unit {u:?}"))
            })
            .transpose()?,
        weld_tol: chipbreaker_core::eps::EPS_WELD,
        json: false,
    };
    crate::mesh::load(&input).map(|(mesh, _)| mesh)
}

/// `--mem-dry-run`: the prediction, and nothing else.
fn dry_run_report(
    args: &BuildArgs,
    extents: [f64; 3],
    spacing: Spacing,
    footprint: &Footprint,
) -> (Value, String, bool) {
    let counts = ray_counts(extents, spacing);
    let bound = spacing.sample_distance_bound();
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
    text.push_str(&format!(
        "memory    {} total = field {} + spill headroom {}\n",
        human(footprint.total_bytes()),
        human(footprint.field_bytes),
        human(footprint.spill_headroom_bytes),
    ));
    if let Some(limit) = args.mem_limit {
        #[allow(clippy::cast_precision_loss, reason = "a percentage")]
        let used = footprint.total_bytes() as f64 / limit as f64 * 100.0;
        text.push_str(&format!("budget    {} ({used:.1}% used)\n", human(limit)));
    }
    text.push_str("dry run   nothing was allocated\n");
    let results = serde_json::json!({
        "command": "dexel build --mem-dry-run",
        "extents_mm": extents,
        "spacing_mm": [spacing.x, spacing.y, spacing.z],
        "sample_distance_bound_mm": bound,
        "rays": counts,
        "memory": {
            "field_bytes": footprint.field_bytes,
            "spill_headroom_bytes": footprint.spill_headroom_bytes,
            "total_bytes": footprint.total_bytes(),
        },
        "limit_bytes": args.mem_limit,
    });
    (results, text, true)
}

/// Loads a mesh for `mem-estimate`, which needs only its bounds.
///
/// Takes the shared [`Input`] rather than rebuilding one, so unit handling and
/// welding stay in exactly one place.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn load_mesh_for_estimate(input: &Input) -> Result<TriMesh, String> {
    crate::mesh::load(input).map(|(mesh, _)| mesh)
}

// --- build -----------------------------------------------------------------

fn run_build(
    args: &BuildArgs,
    out: Option<&std::path::Path>,
) -> Result<(Value, String, bool), String> {
    if !args.res.is_finite() || args.res <= 0.0 {
        return Err(format!(
            "--res must be a positive length in millimetres, got {}",
            args.res
        ));
    }
    let (mesh, mesh_summary) = crate::mesh::load(&args.input)?;

    // **Predicted before anything is allocated.** The extents come from the
    // mesh, so this is the real footprint of the job about to run rather than an
    // estimate of a similar one.
    let extents = mesh.bounds().extent().to_array();
    let spacing = if args.auto_res {
        auto_spacing(extents, args.res)
    } else {
        args.spacing_xyz().unwrap_or(Spacing::uniform(args.res))
    };
    let budget = args.mem_limit.map_or_else(Budget::unlimited, Budget::bytes);
    let footprint = budget
        .check(extents, spacing, 0, false)
        .map_err(|e| e.to_string())?;

    if args.mem_dry_run {
        return Ok(dry_run_report(args, extents, spacing, &footprint));
    }

    let mut options = args.options();
    options.spacing_xyz = Some(spacing);
    let (field, stats) = TriDexelField::build(&mesh, &options).map_err(|e| e.to_string())?;

    // The tessellation adequacy warning. A customer asking for 0.05 mm on a
    // coarse STL is buying precision their data cannot carry, and delivering it
    // silently produces a confident-looking answer whose real error is a
    // hundred times the cell size.
    let advice = field.provenance().tessellation.advice(args.res);

    let mut written = None;
    if let Some(path) = out {
        let bytes = if field.is_complete() || field.axes().len() > 1 {
            dexel_io::tri_to_bytes(&field).map_err(|e| e.to_string())?
        } else {
            // A single bundle round-trips through the older, simpler format, so
            // `--axes z` still produces something a .dexel reader understands.
            let (axis, bundle) = field.bundles().next().ok_or("no bundle was built")?;
            let _ = axis;
            dexel_io::to_bytes(bundle).map_err(|e| e.to_string())?
        };
        std::fs::write(path, &bytes)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        written = Some((path.display().to_string(), bytes.len()));
    }

    let mut text = describe_tri(&field, Some(&stats));
    if let Some((path, bytes)) = &written {
        text.push_str(&format!(
            "\nwrote {path} ({bytes} bytes, {:.2} MiB)\n",
            *bytes as f64 / (1024.0 * 1024.0)
        ));
    }
    if let Some(advice) = &advice {
        text.push_str(&format!("\nWARNING: {advice}\n"));
    }

    let t = &field.provenance().tessellation;
    let digest = {
        let mut h = CanonicalHash::new();
        h.add(&field);
        h.finish().to_hex()
    };
    let results = json!({
        "axes": field.axes().as_str(),
        "command": "build",
        "digest": digest,
        "mesh": mesh_summary,
        "per_axis": per_axis_json(&field, Some(&stats)),
        "tessellation": {
            "advice": advice,
            "edges": t.edges,
            "max_sagitta_mm": t.max_sagitta_mm,
            "mean_edge_mm": t.mean_edge_mm,
            "percentile_sagitta_mm": t.percentile_sagitta_mm,
            "sharp_edges": t.sharp_edges,
        },
        "totals": {
            "bytes": field.bytes(),
            "degenerate_triangles": stats.degenerate_triangles,
            "rays": stats.rays,
            "spans": stats.spans,
            "volume_mm3_mean": field.volume(),
        },
        "written": written.map(|(p, n)| json!({ "bytes": n, "path": p })),
    });
    Ok((results, text, true))
}

// --- stat, volume ----------------------------------------------------------

fn run_stat(file: &std::path::Path, per_axis: bool) -> Result<(Value, String, bool), String> {
    let loaded = read_field(file)?;
    let mut text = String::new();
    for (axis, bundle) in loaded.bundles() {
        let lattice = bundle.lattice();
        let [nx, ny] = lattice.counts();
        text.push_str(&format!(
            "bundle {}   {nx} x {ny} = {} rays, {} mm cells, origin {:?}\n",
            axis.as_str(),
            lattice.ray_count(),
            lattice.spacing(),
            lattice.origin().to_array(),
        ));
        text.push_str(&format!(
            "            {} filled, {} spans, {} spilled, {:.2} MiB, volume {} mm^3\n",
            bundle.filled_rays(),
            bundle.total_spans(),
            bundle.arena().spilled_rays(),
            bundle.arena().bytes() as f64 / (1024.0 * 1024.0),
            bundle.volume(),
        ));
        if per_axis {
            for (spans, rays) in bundle.arena().distribution() {
                text.push_str(&format!(
                    "              {spans:>3} span(s)  {rays:>10} rays\n"
                ));
            }
        }
    }
    if let Some(tri) = loaded.as_tri() {
        let p = tri.provenance();
        text.push_str(&format!(
            "\nprovenance  source {} triangles, digest {}\n            requested {} mm cells\n",
            p.source_triangles,
            &p.source_digest.to_hex()[..16],
            p.requested_spacing_mm,
        ));
        text.push_str(&format!(
            "            mesh's own deviation ~{:.5} mm (95th pct of {} edges, worst {:.5})\n",
            p.tessellation.percentile_sagitta_mm,
            p.tessellation.edges,
            p.tessellation.max_sagitta_mm
        ));
        if !tri.is_complete() {
            text.push_str(
                "\nNOTE: this field has fewer than three bundles, so the 1/sqrt(3)\n\
                 sampling guarantee does NOT hold. A surface normal perpendicular to\n\
                 both missing axes is sampled by neither.\n",
            );
        }
    }
    text.push_str(
        "\nvolume is a DIAGNOSTIC, not an accuracy metric (ADR 0005). Use `deviation`.\n",
    );

    let results = json!({
        "command": "stat",
        "complete": loaded.as_tri().is_some_and(TriDexelField::is_complete),
        "digest": loaded.digest(),
        "file": file.display().to_string(),
        "per_axis": bundles_json(&loaded),
    });
    Ok((results, text, true))
}

fn run_volume(file: &std::path::Path) -> Result<(Value, String, bool), String> {
    let loaded = read_field(file)?;
    let volumes: Vec<(Axis, f64)> = loaded
        .bundles()
        .into_iter()
        .map(|(a, b)| (a, b.volume()))
        .collect();
    let mean = volumes.iter().map(|(_, v)| *v).sum::<f64>() / volumes.len().max(1) as f64;

    let mut text = String::new();
    for (axis, volume) in &volumes {
        text.push_str(&format!("  {} bundle   {volume} mm^3\n", axis.as_str()));
    }
    text.push_str(&format!(
        "  mean        {mean} mm^3   ({:.6} cm^3)\n",
        mean / 1000.0
    ));
    text.push_str(
        "\nThe bundles will NOT agree closely, and they are not supposed to. They\n\
         disagree at O(h^2) with independent signs, and every cell claims a full\n\
         h^2 of cross-section, so a spacing that does not divide the stock\n\
         over-counts by exactly the covered-to-true area ratio. Volume is a\n\
         construction diagnostic; `deviation` is the accuracy metric (ADR 0005).\n",
    );

    let results = json!({
        "command": "volume",
        "digest": loaded.digest(),
        "mean_mm3": mean,
        "per_axis": volumes
            .iter()
            .map(|(a, v)| json!({ "axis": a.as_str(), "volume_mm3": v }))
            .collect::<Vec<_>>(),
    });
    Ok((results, text, true))
}

// --- slice -----------------------------------------------------------------

fn run_slice(
    file: &std::path::Path,
    at: (Axis, f64),
    axis: Option<Axis>,
    out: Option<&std::path::Path>,
) -> Result<(Value, String, bool), String> {
    let (plane_axis, plane_at) = at;
    let loaded = read_field(file)?;
    // Default to the plane's own axis: those rays cross the plane exactly once
    // per span, so occupancy is a simple in-span test per cell and the picture
    // is a clean raster of the cross-section.
    let want = axis.unwrap_or(plane_axis);
    let bundle = loaded
        .bundles()
        .into_iter()
        .find(|(a, _)| *a == want)
        .map(|(_, b)| b)
        .ok_or_else(|| {
            format!(
                "this field has no {} bundle; it carries {}",
                want.as_str(),
                loaded.axes().as_str()
            )
        })?;

    let lattice = bundle.lattice();
    let [u, v, w] = lattice.axis().cyclic();
    let [nu, nv] = lattice.counts();
    let spacing = lattice.spacing();
    let base_w = lattice.origin().to_array()[w] - spacing;

    let mut cells: Vec<(u32, u32)> = Vec::new();
    if want == plane_axis {
        // The plane is perpendicular to the rays: test each ray for material at
        // the plane's coordinate.
        let t = plane_at - base_w;
        for i in 0..nu {
            for j in 0..nv {
                let ray = lattice.index(i, j);
                if bundle
                    .arena()
                    .get(ray)
                    .iter()
                    .any(|s| s.t0 <= t && t <= s.t1)
                {
                    cells.push((i, j));
                }
            }
        }
    } else {
        // The plane is parallel to the rays: keep the row of rays whose own
        // transverse coordinate lands in the plane's cell.
        let axes = lattice.axis().cyclic();
        let which = if axes[0] == plane_axis.index() { 0 } else { 1 };
        for i in 0..nu {
            for j in 0..nv {
                let origin = lattice.origin_of(i, j).to_array();
                let coordinate = origin[if which == 0 { u } else { v }];
                if (coordinate - plane_at).abs() <= spacing / 2.0
                    && !bundle.arena().get(lattice.index(i, j)).is_empty()
                {
                    cells.push((i, j));
                }
            }
        }
    }

    let text = format!(
        "slice {}={} from the {} bundle: {} of {} cells carry material\n",
        plane_axis.as_str(),
        plane_at,
        want.as_str(),
        cells.len(),
        lattice.ray_count(),
    );

    let mut written = None;
    if let Some(path) = out {
        let svg = render_svg(bundle, &cells, plane_axis, plane_at);
        std::fs::write(path, svg.as_bytes())
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        written = Some(path.display().to_string());
    }

    let results = json!({
        "at": plane_at,
        "axis": want.as_str(),
        "cells_with_material": cells.len(),
        "command": "slice",
        "digest": loaded.digest(),
        "plane_axis": plane_axis.as_str(),
        "total_cells": lattice.ray_count(),
        "written": written,
    });
    Ok((results, text, true))
}

/// A self-contained SVG of the occupied cells.
///
/// No external references of any kind: this is a debugging artefact that has to
/// open in a browser with no network and no stylesheet.
fn render_svg(bundle: &DexelField, cells: &[(u32, u32)], plane_axis: Axis, at: f64) -> String {
    let lattice = bundle.lattice();
    let [u, v, _] = lattice.axis().cyclic();
    let [nu, nv] = lattice.counts();
    let spacing = lattice.spacing();
    let names = ["x", "y", "z"];

    // A pixel scale that keeps the picture readable whatever the lattice size.
    let scale = (900.0 / f64::from(nu.max(nv)).max(1.0)).clamp(0.5, 12.0);
    let width = f64::from(nu) * scale;
    let height = f64::from(nv) * scale;

    let mut out = String::with_capacity(cells.len() * 48 + 512);
    out.push_str(&format!(
        // The viewBox must include the caption strip, or the label renders
        // outside the visible area and the picture looks fine while silently
        // losing which plane it is of.
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.0}\" height=\"{h:.0}\" \
         viewBox=\"0 0 {w:.3} {h:.3}\">\n",
        w = width.max(1.0),
        h = height.max(1.0) + 26.0,
    ));
    out.push_str("<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n");
    out.push_str("<g fill=\"#2b6cb0\" shape-rendering=\"crispEdges\">\n");
    for (i, j) in cells {
        // Y is flipped so the picture reads the way the axes do, with the
        // lattice origin at the bottom left rather than the top left.
        let x = f64::from(*i) * scale;
        let y = height - f64::from(*j + 1) * scale;
        out.push_str(&format!(
            "<rect x=\"{x:.3}\" y=\"{y:.3}\" width=\"{scale:.3}\" height=\"{scale:.3}\"/>\n"
        ));
    }
    out.push_str("</g>\n");
    out.push_str(&format!(
        "<text x=\"2\" y=\"{:.1}\" font-family=\"monospace\" font-size=\"12\" \
         fill=\"#333333\">{}={} | {} bundle | {} mm cells | horizontal {}, vertical {}</text>\n",
        height + 16.0,
        names[plane_axis.index()],
        at,
        names[lattice.axis().index()],
        spacing,
        names[u],
        names[v],
    ));
    out.push_str("</svg>\n");
    out
}

// --- deviation, coverage ---------------------------------------------------

fn run_deviation(
    file: &std::path::Path,
    mesh_path: &std::path::Path,
    units: Option<&str>,
    per_axis: bool,
) -> Result<(Value, String, bool), String> {
    let loaded = read_field(file)?;
    let tri = loaded
        .as_tri()
        .ok_or("deviation needs a .tdx field; a single bundle has no best-of-three")?;
    let mesh = load_mesh(mesh_path, units)?;
    let (samples, sample_spacing) = sample_mesh_budget(&mesh, DEVIATION_SAMPLES);
    let report = measure_deviation(tri, &samples);

    let mut text = format!(
        "deviation against {} ({} surface samples, ~{:.4} mm apart)\n\n",
        mesh_path.display(),
        report.samples,
        sample_spacing,
    );
    if per_axis {
        for axis in AXES {
            if let Some(max) = report.per_axis_max[axis.index()] {
                text.push_str(&format!(
                    "  bundle {}   worst {max:.6} mm   rms {:.6} mm\n",
                    axis.as_str(),
                    report.per_axis_rms[axis.index()].unwrap_or(0.0),
                ));
            }
        }
        text.push('\n');
    }
    text.push_str(&format!(
        "  BEST-OF-3  worst {:.6} mm   rms {:.6} mm   C = worst/h = {:.4}\n",
        report.best_max,
        report.best_rms,
        report.constant(),
    ));
    text.push_str(&format!(
        "  coverage   worst sampling cosine {:.9} (bound {WORST_CASE_COSINE:.9}) {}\n",
        report.worst_cosine,
        if report.coverage_holds() {
            "OK"
        } else {
            "VIOLATED"
        }
    ));
    text.push_str(
        "\nThis is a COVERAGE deviation: the distance from a point on the true\n\
         surface to the nearest place the field sampled it. Span endpoints are\n\
         exact ray-surface intersections, so this carries no error of its own.\n\
         It is not a reconstruction deviation -- extracting a surface is U9, and\n\
         U9 must re-measure against its own output.\n",
    );

    let results = json!({
        "best_max_mm": report.best_max,
        "best_rms_mm": report.best_rms,
        "command": "deviation",
        "constant": report.constant(),
        "coverage_holds": report.coverage_holds(),
        "digest": loaded.digest(),
        "per_axis": AXES
            .iter()
            .map(|a| json!({
                "axis": a.as_str(),
                "max_mm": report.per_axis_max[a.index()],
                "rms_mm": report.per_axis_rms[a.index()],
            }))
            .collect::<Vec<_>>(),
        "samples": report.samples,
        "spacing_mm": report.spacing,
        "worst_cosine": report.worst_cosine,
    });
    Ok((results, text, report.coverage_holds()))
}

fn run_coverage(
    file: &std::path::Path,
    mesh_path: &std::path::Path,
    units: Option<&str>,
) -> Result<(Value, String, bool), String> {
    let loaded = read_field(file)?;
    let axes = loaded.axes();
    let mesh = load_mesh(mesh_path, units)?;
    let (worst, normal) = measure_coverage(&mesh, axes);
    let holds = worst >= WORST_CASE_COSINE - 1e-12;
    let degrees =
        chipbreaker_core::transcendental::acos(worst.min(1.0)) * 180.0 / core::f64::consts::PI;

    let text = format!(
        "axes {}\nworst sampling cosine {worst:.9} ({degrees:.4} deg) at n = \
         [{:.6}, {:.6}, {:.6}]\nbound 1/sqrt(3) = {WORST_CASE_COSINE:.9} (54.7356 deg)\n\n{}\n",
        axes.as_str(),
        normal[0],
        normal[1],
        normal[2],
        if holds {
            "OK: every surface is met by some bundle at 54.7356 degrees or better."
        } else if axes.len() < 3 {
            "VIOLATED, and expected to be: fewer than three bundles carries no such \
             guarantee. A normal perpendicular to every present axis is sampled by none."
        } else {
            "VIOLATED with three bundles present. This should be impossible -- the bound \
             follows from |n| = 1 -- so either the normals are not unit or something is \
             very wrong."
        }
    );

    let results = json!({
        "axes": axes.as_str(),
        "bound": WORST_CASE_COSINE,
        "command": "coverage",
        "degrees": degrees,
        "holds": holds,
        "worst_cosine": worst,
        "worst_normal": normal,
    });
    Ok((results, text, holds))
}

// --- convergence -----------------------------------------------------------

fn run_convergence() -> Result<(Value, String, bool), String> {
    let ratios = standard_ratios();
    let mut text = String::from(
        "Volume convergence. A DIAGNOSTIC (ADR 0005): non-monotone, floored by\n\
         tessellation, and biased by cell quantisation. Use `deviation` for accuracy.\n\n",
    );
    let mut cases = Vec::new();
    let mut ok = true;

    for case in standard_cases() {
        let result = measure_convergence(&case, &ratios);
        text.push_str(&format!("=== {} ===\n", result.name));
        for sample in &result.samples {
            text.push_str(&format!(
                "  h/R {:>8.5}   vs mesh {:>11.3e}   vs analytic {}\n",
                sample.ratio,
                sample.mesh_error(),
                sample
                    .analytic_error()
                    .map_or_else(|| "--".to_owned(), |e| format!("{e:.3e}")),
            ));
        }
        let (model, envelope) = match result.model {
            ErrorModel::Quadrature => ("quadrature", result.envelope_constant(1.5)),
            ErrorModel::LatticeCount => (
                "lattice_count",
                result.envelope_constant(GAUSS_CIRCLE_EXPONENT),
            ),
        };
        if !result.is_monotone() {
            text.push_str("  NOT monotone: a finer lattice made the answer worse\n");
        }
        if let Some(finest) = result.finest_within(1.0 / 200.0) {
            text.push_str(&format!(
                "  at h <= R/200: {:.5}%\n",
                finest.mesh_error() * 100.0
            ));
            if finest.mesh_error() >= 1e-3 {
                ok = false;
            }
        }
        text.push('\n');
        cases.push(json!({
            "envelope": envelope,
            "model": model,
            "monotone": result.is_monotone(),
            "name": result.name,
        }));
    }
    Ok((
        json!({ "cases": cases, "command": "convergence" }),
        text,
        ok,
    ))
}

// --- shared rendering ------------------------------------------------------

fn describe_tri(field: &TriDexelField, stats: Option<&TriBuildStats>) -> String {
    let mut out = format!("axes      {}\n", field.axes().as_str());
    for (axis, bundle) in field.bundles() {
        let lattice = bundle.lattice();
        let [nx, ny] = lattice.counts();
        out.push_str(&format!(
            "bundle {}  {nx} x {ny} = {} rays, {} spans, {:.2} MiB, volume {} mm^3\n",
            axis.as_str(),
            lattice.ray_count(),
            bundle.total_spans(),
            bundle.arena().bytes() as f64 / (1024.0 * 1024.0),
            bundle.volume(),
        ));
    }
    out.push_str(&format!(
        "total     {} rays, {} spans, {:.2} MiB\n",
        field.rays(),
        field.total_spans(),
        field.bytes() as f64 / (1024.0 * 1024.0),
    ));
    if let Some(stats) = stats
        && stats.degenerate_triangles > 0
    {
        out.push_str(&format!(
            "note      dropped {} degenerate triangle(s) before casting\n",
            stats.degenerate_triangles
        ));
    }
    out
}

fn per_axis_json(field: &TriDexelField, stats: Option<&TriBuildStats>) -> Value {
    let entries: Vec<Value> = field
        .bundles()
        .map(|(axis, bundle)| {
            let lattice = bundle.lattice();
            let [nx, ny] = lattice.counts();
            json!({
                "axis": axis.as_str(),
                "bytes": bundle.arena().bytes(),
                "counts": [nx, ny],
                "filled_rays": bundle.filled_rays(),
                "origin_mm": lattice.origin().to_array(),
                "rays": lattice.ray_count(),
                "spacing_mm": lattice.spacing(),
                "spans": bundle.total_spans(),
                "spilled_rays": bundle.arena().spilled_rays(),
                "volume_mm3": bundle.volume(),
                "coplanar_rejected": stats
                    .and_then(|s| s.per_axis[axis.index()])
                    .map(|s| s.predicates.coplanar_rejected),
            })
        })
        .collect();
    Value::Array(entries)
}

fn bundles_json(loaded: &Loaded) -> Value {
    let entries: Vec<Value> = loaded
        .bundles()
        .into_iter()
        .map(|(axis, bundle)| {
            let lattice = bundle.lattice();
            let [nx, ny] = lattice.counts();
            json!({
                "axis": axis.as_str(),
                "bytes": bundle.arena().bytes(),
                "counts": [nx, ny],
                "filled_rays": bundle.filled_rays(),
                "origin_mm": lattice.origin().to_array(),
                "rays": lattice.ray_count(),
                "spacing_mm": lattice.spacing(),
                "spans": bundle.total_spans(),
                "spilled_rays": bundle.arena().spilled_rays(),
                "volume_mm3": bundle.volume(),
            })
        })
        .collect();
    Value::Array(entries)
}
