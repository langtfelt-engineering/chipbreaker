// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `chipbreaker mesh` subcommands.
//!
//! Every command follows the Unit 1 report convention: a `results` section that
//! is deterministic and canonically hashed, and an `environment` section that
//! carries timings and host details and is excluded from that hash.

use std::path::{Path, PathBuf};
use std::time::Instant;

use chipbreaker_core::eps::EPS_WELD;
use chipbreaker_core::golden::{CanonicalHash, Hashable};
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::TriMesh;
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::io::{self, Format};
use chipbreaker_core::mesh::units::{Unit, accepted_names};
use chipbreaker_core::mesh::validate::{MeshReport, validate};
use chipbreaker_core::mesh::weld::weld;
use clap::{Args, Subcommand};
use serde_json::{Value, json};

use crate::report::Environment;

/// Parses `--units`, refusing to guess.
///
/// # Errors
/// Returns a message listing the accepted names.
pub fn parse_unit(s: &str) -> Result<Unit, String> {
    Unit::from_name(s).ok_or_else(|| {
        format!("unknown unit `{s}`; STL and OBJ carry no unit information, so one must be given. Accepted: {}", accepted_names())
    })
}

/// Parses an `x,y,z` triple.
///
/// # Errors
/// Returns a message describing what was wrong.
pub fn parse_vec3(s: &str) -> Result<Vec3, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 {
        return Err(format!("expected `x,y,z`, got `{s}`"));
    }
    let mut out = [0.0f64; 3];
    for (i, p) in parts.iter().enumerate() {
        out[i] = p
            .trim()
            .parse()
            .map_err(|_| format!("component {i} of `{s}` is not a number"))?;
    }
    Ok(Vec3::new(out[0], out[1], out[2]))
}

/// Options shared by every mesh subcommand that reads a file.
#[derive(Debug, Args)]
pub struct Input {
    /// Mesh file to read.
    pub file: PathBuf,
    /// Unit the file's coordinates are in.
    ///
    /// Required for STL and OBJ, which carry no unit information at all. 3MF
    /// declares its own, so passing this for a 3MF file is only checked for
    /// agreement — a contradiction is an error, not an override.
    #[arg(long, value_parser = parse_unit)]
    pub units: Option<Unit>,
    /// Vertex welding lattice, in millimetres.
    #[arg(long, default_value_t = EPS_WELD)]
    pub weld_tol: f64,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

/// `chipbreaker mesh ...`
#[derive(Debug, Subcommand)]
pub enum MeshCommand {
    /// Report what a mesh file contains.
    Inspect(Input),
    /// Check a mesh's topology and report every defect.
    Validate {
        #[command(flatten)]
        input: Input,
        /// Also look for triangles that intersect but share no vertex.
        ///
        /// Opt-in: it costs O(n log n) with a large constant.
        #[arg(long)]
        check_self_intersect: bool,
    },
    /// Convert between mesh formats.
    Convert {
        #[command(flatten)]
        input: Input,
        /// Output file; the format is chosen from its extension.
        out: PathBuf,
    },
    /// Build a bounding volume hierarchy and report its shape.
    Bvh {
        #[command(flatten)]
        input: Input,
        /// Print node statistics.
        #[arg(long)]
        stats: bool,
    },
    /// Cast a single ray and report the crossings.
    Raycast {
        #[command(flatten)]
        input: Input,
        /// Ray origin as `x,y,z`, in millimetres.
        #[arg(long, value_parser = parse_vec3)]
        origin: Vec3,
        /// Ray direction as `x,y,z`.
        #[arg(long, value_parser = parse_vec3)]
        dir: Vec3,
        /// Report every crossing rather than only the nearest.
        #[arg(long)]
        all: bool,
    },
    /// Cast a dense lattice of rays and check for material leaks.
    ///
    /// This is the Unit 5 contract, runnable from the command line: every ray
    /// through a closed surface must cross it an even number of times, with a
    /// running depth that never goes negative.
    Parity {
        #[command(flatten)]
        input: Input,
        /// Rays per side; the sweep casts `lattice^2` per direction.
        #[arg(long, default_value_t = 64)]
        lattice: u32,
        /// Snap ray origins to integer coordinates, so they pass through
        /// vertices and along edges. This is the case that finds tie-break bugs.
        #[arg(long)]
        align_to_vertices: bool,
    },
}

impl MeshCommand {
    fn input(&self) -> &Input {
        match self {
            Self::Inspect(i)
            | Self::Validate { input: i, .. }
            | Self::Convert { input: i, .. }
            | Self::Bvh { input: i, .. }
            | Self::Raycast { input: i, .. }
            | Self::Parity { input: i, .. } => i,
        }
    }
}

/// Loads, welds and returns a mesh along with what welding did.
fn load(input: &Input) -> Result<(TriMesh, Value), String> {
    let bytes = std::fs::read(&input.file)
        .map_err(|e| format!("cannot read {}: {e}", input.file.display()))?;
    let name = input.file.file_name().and_then(|s| s.to_str());
    let format = io::detect(&bytes, name);

    // 3MF states its unit; everything else has none, so the caller must.
    // Refusing to guess is the point: see chipbreaker_core::mesh::units.
    let unit = match (format.declares_units(), input.units) {
        (true, _) => None,
        (false, Some(u)) => Some(u),
        (false, None) => {
            return Err(format!(
                "{} carries no unit information, so --units is required. Accepted: {}",
                format.name(),
                accepted_names()
            ));
        }
    };
    let required = || unit.unwrap_or(Unit::Millimetre);

    let raw = match format {
        Format::StlBinary => io::stl::read_binary(&bytes, required()).map_err(|e| e.to_string()),
        Format::StlAscii => {
            let text = String::from_utf8_lossy(&bytes);
            io::stl::read_ascii(&text, required()).map_err(|e| e.to_string())
        }
        Format::Obj => {
            let text = String::from_utf8_lossy(&bytes);
            io::obj::read(&text, required()).map_err(|e| e.to_string())
        }
        Format::ThreeMf => io::threemf::read(&bytes, input.units).map_err(|e| e.to_string()),
    }?;
    let source_unit = raw.meta().source_unit;

    let (welded, weld_report) = weld(&raw, input.weld_tol).map_err(|e| e.to_string())?;
    let summary = json!({
        "format": format.name(),
        "lattice_mm": weld_report.lattice,
        "source_unit": source_unit.name(),
        "triangles_collapsed_by_welding": weld_report.triangles_collapsed,
        "vertices_after_weld": weld_report.vertices_after,
        "vertices_before_weld": weld_report.vertices_before,
    });
    Ok((welded, summary))
}

fn report_json(mesh_report: &MeshReport) -> Value {
    let findings: Vec<Value> = mesh_report
        .findings
        .iter()
        .map(|f| {
            json!({
                "detail": f.detail,
                "id": f.id,
                "kind": f.kind.name(),
                "triangles": f.triangles,
                "vertices": f.vertices,
            })
        })
        .collect();
    let components: Vec<Value> = mesh_report
        .components
        .iter()
        .map(|c| {
            json!({
                "boundary_edges": c.boundary_edges,
                "edges": c.edges,
                "euler_characteristic": c.euler_characteristic,
                "genus": c.genus,
                "signed_volume_mm3": c.signed_volume,
                "triangles": c.triangles,
                "vertices": c.vertices,
            })
        })
        .collect();
    json!({
        "components": components,
        "edges": mesh_report.edges,
        "findings": findings,
        "is_manifold": mesh_report.is_manifold,
        "is_orientation_consistent": mesh_report.is_orientation_consistent,
        "is_solid": mesh_report.is_solid(),
        "is_watertight": mesh_report.is_watertight,
        "self_intersection_checked": mesh_report.self_intersection_checked,
        "signed_volume_mm3": mesh_report.signed_volume,
        "surface_area_mm2": mesh_report.surface_area,
        "triangles": mesh_report.triangles,
        "version": mesh_report.version,
        "vertices": mesh_report.vertices,
    })
}

/// Runs a mesh subcommand, returning `(results, human_text, ok)`.
///
/// # Errors
/// Returns a human-readable message for any I/O or parse failure.
pub fn run(command: &MeshCommand) -> Result<(Value, String, bool), String> {
    let input = command.input();
    let (mesh, load_summary) = load(input)?;

    match command {
        MeshCommand::Inspect(_) => {
            let bounds = mesh.bounds();
            let results = json!({
                "bounds_max_mm": bounds.max.to_array(),
                "bounds_min_mm": bounds.min.to_array(),
                "command": "inspect",
                "ignored_records": mesh.meta().ignored_records,
                "load": load_summary,
                "mesh_hash": mesh.canonical_digest().to_hex(),
                "polygons_triangulated": mesh.meta().polygons_triangulated,
                "signed_volume_mm3": mesh.signed_volume(),
                "surface_area_mm2": mesh.surface_area(),
                "triangles": mesh.triangle_count(),
                "vertices": mesh.vertex_count(),
            });
            let text = format!(
                "{} triangles, {} vertices, read as {}\n\
                 bounds  {:?} .. {:?} mm\n\
                 volume  {} mm^3\n\
                 area    {} mm^2\n\
                 welded  {} -> {} vertices at a {} mm lattice\n",
                mesh.triangle_count(),
                mesh.vertex_count(),
                // The unit actually used, which for 3MF comes from the file
                // rather than from the command line.
                load_summary["source_unit"],
                bounds.min.to_array(),
                bounds.max.to_array(),
                mesh.signed_volume(),
                mesh.surface_area(),
                load_summary["vertices_before_weld"],
                load_summary["vertices_after_weld"],
                input.weld_tol,
            );
            Ok((results, text, true))
        }

        MeshCommand::Validate {
            check_self_intersect,
            ..
        } => {
            let mut report = validate(&mesh);
            if *check_self_intersect {
                chipbreaker_core::mesh::validate::check_self_intersections(&mesh, &mut report);
            }
            let ok = report.is_solid()
                && report.count_of(chipbreaker_core::mesh::validate::FindingKind::SelfIntersection)
                    == 0;
            let results = json!({
                "command": "validate",
                "load": load_summary,
                "report": report_json(&report),
                "report_hash": report.digest().to_hex(),
            });
            let mut text = format!(
                "manifold {}  watertight {}  consistent {}  solid {}\n\
                 volume {} mm^3, area {} mm^2, {} component(s)\n",
                report.is_manifold,
                report.is_watertight,
                report.is_orientation_consistent,
                ok,
                report.signed_volume,
                report.surface_area,
                report.components.len(),
            );
            for c in &report.components {
                text.push_str(&format!(
                    "  component: {} triangles, chi = {}, genus {:?}, volume {}\n",
                    c.triangles, c.euler_characteristic, c.genus, c.signed_volume
                ));
            }
            if report.findings.is_empty() {
                text.push_str("no findings\n");
            } else {
                text.push_str(&format!("{} finding(s):\n", report.findings.len()));
                for f in &report.findings {
                    text.push_str(&format!("  [{}] {}: {}\n", f.id, f.kind, f.detail));
                }
            }
            Ok((results, text, ok))
        }

        MeshCommand::Convert { out, .. } => {
            let written = write_mesh(&mesh, out)?;
            let results = json!({
                "bytes": written.0,
                "command": "convert",
                "load": load_summary,
                "mesh_hash": mesh.canonical_digest().to_hex(),
                "output_format": written.1,
            });
            let text = format!(
                "wrote {} ({} bytes, {})\n",
                out.display(),
                written.0,
                written.1
            );
            Ok((results, text, true))
        }

        MeshCommand::Bvh { stats, .. } => {
            let bvh = Bvh::build(&mesh);
            let results = json!({
                "command": "bvh",
                "leaves": bvh.leaf_count(),
                "load": load_summary,
                "max_depth": bvh.max_depth(),
                "nodes": bvh.nodes().len(),
                "topology_hash": bvh.topology_digest().to_hex(),
                "triangles": mesh.triangle_count(),
            });
            let mut text = format!(
                "{} nodes, {} leaves, depth {}\ntopology hash {}\n",
                bvh.nodes().len(),
                bvh.leaf_count(),
                bvh.max_depth(),
                bvh.topology_digest()
            );
            if *stats {
                let leaf_sizes: Vec<u32> = bvh
                    .nodes()
                    .iter()
                    .filter(|n| n.is_leaf())
                    .map(|n| n.count)
                    .collect();
                let total: u32 = leaf_sizes.iter().sum();
                text.push_str(&format!(
                    "leaf triangles: total {total}, max {}, mean {:.2}\n",
                    leaf_sizes.iter().copied().max().unwrap_or(0),
                    f64::from(total) / leaf_sizes.len().max(1) as f64,
                ));
            }
            Ok((results, text, true))
        }

        MeshCommand::Raycast {
            origin, dir, all, ..
        } => {
            let bvh = Bvh::build(&mesh);
            let ray = Ray::new(*origin, *dir);
            let (hits, stats) = bvh
                .intersect_ray_all(&mesh, &ray)
                .map_err(|e| e.to_string())?;
            let shown: Vec<&chipbreaker_core::mesh::bvh::Hit> = if *all {
                hits.iter().collect()
            } else {
                hits.iter().filter(|h| h.t >= 0.0).take(1).collect()
            };
            let crossings: Vec<Value> = shown
                .iter()
                .map(|h| {
                    json!({
                        "entering": h.entering,
                        "point_mm": ray.at(h.t).to_array(),
                        "t": h.t,
                        "triangle": h.triangle,
                    })
                })
                .collect();
            let results = json!({
                "command": "raycast",
                "coplanar_rejected": stats.coplanar_rejected,
                "crossings": crossings,
                "exact_path": stats.exact_path,
                "fast_path": stats.fast_path,
                "load": load_summary,
                "sos_resolutions": stats.sos_resolutions,
                "total_crossings": hits.len(),
                "triangle_tests": stats.triangle_tests,
            });
            let mut text = format!("{} crossing(s)\n", hits.len());
            for h in &shown {
                text.push_str(&format!(
                    "  t = {}  triangle {}  {}\n",
                    h.t,
                    h.triangle,
                    if h.entering { "entering" } else { "leaving" }
                ));
            }
            text.push_str(&format!(
                "{} triangle tests, {:.2}% exact path, {} SoS resolutions\n",
                stats.triangle_tests,
                stats.exact_fraction() * 100.0,
                stats.sos_resolutions
            ));
            Ok((results, text, true))
        }

        MeshCommand::Parity {
            lattice,
            align_to_vertices,
            ..
        } => {
            let outcome = run_parity(&mesh, *lattice, *align_to_vertices);
            let ok = outcome.leaks == 0;
            let results = json!({
                "aligned_to_vertices": align_to_vertices,
                "command": "parity",
                "coplanar_rejected": outcome.coplanar,
                "exact_fraction": outcome.exact_fraction,
                "leaks": outcome.leaks,
                "load": load_summary,
                "rays_cast": outcome.cast,
                "sos_resolutions": outcome.sos,
                "triangle_tests": outcome.tests,
            });
            let text = format!(
                "{} rays cast, {} leaks\n\
                 {} triangle tests, {:.3}% exact path, {} SoS resolutions, \
                 {} coplanar rejects\n{}",
                outcome.cast,
                outcome.leaks,
                outcome.tests,
                outcome.exact_fraction * 100.0,
                outcome.sos,
                outcome.coplanar,
                outcome.first_leak.as_deref().unwrap_or(""),
            );
            Ok((results, text, ok))
        }
    }
}

struct ParityOutcome {
    cast: u64,
    leaks: u64,
    tests: u64,
    sos: u64,
    coplanar: u64,
    exact_fraction: f64,
    first_leak: Option<String>,
}

fn run_parity(mesh: &TriMesh, lattice: u32, aligned: bool) -> ParityOutcome {
    let bvh = Bvh::build(mesh);
    let b = mesh.bounds();
    let extent = b.extent();
    let n = f64::from(lattice.max(1));
    let mut hits = Vec::new();
    let mut cast = 0u64;
    let mut leaks = 0u64;
    let mut first_leak = None;
    let mut total = chipbreaker_core::mesh::bvh::RayStats::default();

    for i in 0..lattice {
        for j in 0..lattice {
            let (u, v) = (f64::from(i), f64::from(j));
            let (x, y) = if aligned {
                (
                    (b.min.x + extent.x * u / n).round(),
                    (b.min.y + extent.y * v / n).round(),
                )
            } else {
                (
                    b.min.x + extent.x * (u + 0.5) / n,
                    b.min.y + extent.y * (v + 0.5) / n,
                )
            };
            let ray = Ray::new(Vec3::new(x, y, b.min.z - extent.z - 1.0), Vec3::Z);
            cast += 1;
            match bvh.intersect_ray_all_into(mesh, &ray, &mut hits) {
                Ok(s) => total.merge(&s),
                Err(e) => {
                    leaks += 1;
                    first_leak.get_or_insert(format!("ray rejected: {e}\n"));
                    continue;
                }
            }
            let mut depth = 0i32;
            let mut k = 0usize;
            let mut bad = hits.len() % 2 != 0;
            while k < hits.len() {
                let t = hits[k].t;
                let mut delta = 0i32;
                while k < hits.len() && hits[k].t == t {
                    delta += if hits[k].entering { 1 } else { -1 };
                    k += 1;
                }
                depth += delta;
                if depth < 0 {
                    bad = true;
                }
            }
            if depth != 0 {
                bad = true;
            }
            if bad {
                leaks += 1;
                first_leak.get_or_insert(format!(
                    "first leak: origin {:?}, {} crossings, final depth {depth}\n",
                    ray.origin.to_array(),
                    hits.len()
                ));
            }
        }
    }
    ParityOutcome {
        cast,
        leaks,
        tests: total.triangle_tests,
        sos: total.sos_resolutions,
        coplanar: total.coplanar_rejected,
        exact_fraction: total.exact_fraction(),
        first_leak,
    }
}

fn write_mesh(mesh: &TriMesh, out: &Path) -> Result<(usize, &'static str), String> {
    let extension = out
        .extension()
        .and_then(|s| s.to_str())
        .map(str::to_ascii_lowercase)
        .unwrap_or_default();
    let (bytes, format): (Vec<u8>, &'static str) = match extension.as_str() {
        "stl" => (io::stl::write_binary(mesh), "stl-binary"),
        "stla" => (
            io::stl::write_ascii(mesh, "chipbreaker").into_bytes(),
            "stl-ascii",
        ),
        "obj" => (io::obj::write(mesh).into_bytes(), "obj"),
        other => {
            return Err(format!(
                "cannot write `{other}`; supported output extensions are \
                 .stl (binary), .stla (ASCII) and .obj"
            ));
        }
    };
    std::fs::write(out, &bytes).map_err(|e| format!("cannot write {}: {e}", out.display()))?;
    Ok((bytes.len(), format))
}

/// Renders a mesh command's output, hashing the `results` section.
#[must_use]
pub fn render(results: &Value, text: &str, elapsed: std::time::Duration, as_json: bool) -> String {
    if !as_json {
        return text.to_owned();
    }
    // Hash the canonical JSON text rather than a bespoke binary encoding: unlike
    // the self-test, these results are a report *about* a file rather than a
    // determinism claim in themselves, and serde_json's Map is a BTreeMap so the
    // rendering is already canonical and sorted.
    let canonical = serde_json::to_string(results).unwrap_or_default();
    let mut h = CanonicalHash::new();
    h.begin("mesh.results").str(&canonical).end();
    let env = Environment::collect(elapsed);
    let mut root = serde_json::Map::new();
    root.insert("schema".to_owned(), json!("chipbreaker.mesh/1"));
    let mut with_hash = results.clone();
    if let Some(map) = with_hash.as_object_mut() {
        map.insert("hash".to_owned(), json!(h.finish().to_hex()));
    }
    root.insert("results".to_owned(), with_hash);
    root.insert("environment".to_owned(), env.to_json());
    let mut out = serde_json::to_string_pretty(&Value::Object(root)).unwrap_or_default();
    out.push('\n');
    out
}

/// Times a closure, for the unhashed environment section.
pub fn timed<T>(f: impl FnOnce() -> T) -> (T, std::time::Duration) {
    let start = Instant::now();
    let value = f();
    (value, start.elapsed())
}
