// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The `chipbreaker tool` subcommands.
//!
//! Same convention as `mesh`: a deterministic `results` section that is
//! canonically hashed, and an `environment` section carrying timings that is
//! not.
//!
//! # Where a tool comes from
//!
//! Either a library file (`--library FILE --id NAME`) or a catalogue
//! specification built on the spot (`--define KIND:key=value,...`). The second
//! exists because the first is tedious for a one-off question — "how much does a
//! 10 mm bull nose with a 2 mm corner actually weigh" should not require
//! authoring a JSON file — and because every parity and convergence check in
//! this unit wants to sweep over shapes rather than over files.

use std::path::PathBuf;

use chipbreaker_core::golden::{CanonicalHash, Hashable};
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::tool::catalog::{
    self, HolderStage, Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill,
    flat_end_mill, tapered_end_mill,
};
use chipbreaker_core::tool::io::ToolLibrary;
use chipbreaker_core::tool::profile::{Profile, ProfileElement};
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};
use chipbreaker_core::tool::{ElementRole, TOP_CAP};
use clap::{Args, Subcommand};
use serde_json::{Value, json};

use crate::mesh::parse_vec3;

/// Where the tool being operated on comes from.
#[derive(Debug, Args)]
pub struct ToolSource {
    /// Tool library file to read.
    #[arg(long, value_name = "FILE", requires = "id")]
    pub library: Option<PathBuf>,
    /// Identifier of the tool within the library.
    #[arg(long, value_name = "NAME")]
    pub id: Option<String>,
    /// Build a tool from the catalogue instead of reading one.
    ///
    /// `KIND:key=value,...`. Run `chipbreaker tool describe --define help` for
    /// the grammar and the keys each kind takes.
    #[arg(long, value_name = "SPEC", conflicts_with = "library")]
    pub define: Option<String>,
    /// Emit JSON instead of text.
    #[arg(long)]
    pub json: bool,
}

/// `chipbreaker tool ...`
#[derive(Debug, Subcommand)]
pub enum ToolCommand {
    /// Report a tool's dimensions and closed-form properties.
    Describe {
        #[command(flatten)]
        source: ToolSource,
    },
    /// List the generating profile, element by element.
    Profile {
        #[command(flatten)]
        source: ToolSource,
    },
    /// Tessellate the tool and write a mesh file.
    Mesh {
        #[command(flatten)]
        source: ToolSource,
        /// Maximum deviation between the mesh and the true surface, in mm.
        #[arg(long, default_value_t = 0.01)]
        tolerance: f64,
        /// Where to write. The extension picks the format.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Cast one ray at the tool and print the intervals inside it.
    Raycast {
        #[command(flatten)]
        source: ToolSource,
        /// Ray origin, `x,y,z`, in tool coordinates.
        ///
        /// `allow_hyphen_values` because a ray usually starts at a negative
        /// coordinate and fires toward the origin; without it clap reads
        /// `-100,0,5` as a flag it does not recognise.
        #[arg(long, value_parser = parse_vec3, default_value = "-100,0,5", allow_hyphen_values = true)]
        origin: Vec3,
        /// Ray direction, `x,y,z`. Normalised before use.
        #[arg(long, value_parser = parse_vec3, default_value = "1,0,0", allow_hyphen_values = true)]
        direction: Vec3,
    },
    /// Cast dense bundles of rays and check that none of them leaks.
    ///
    /// Exits non-zero if any span is unbounded, escapes the bounding cylinder,
    /// or disagrees with the containment predicate.
    Parity {
        #[command(flatten)]
        source: ToolSource,
        /// Rays per side of each bundle. Three bundles are cast, one per axis.
        #[arg(long, default_value_t = 64)]
        rays: usize,
    },
    /// Refine a ray bundle and a tessellation, and show both converging.
    Convergence {
        #[command(flatten)]
        source: ToolSource,
        /// Coarsest bundle; each step doubles until `--finest`.
        #[arg(long, default_value_t = 16)]
        coarsest: usize,
        /// Finest bundle.
        #[arg(long, default_value_t = 256)]
        finest: usize,
    },
}

impl ToolCommand {
    /// The source options, whichever variant this is.
    #[must_use]
    pub fn source(&self) -> &ToolSource {
        match self {
            Self::Describe { source }
            | Self::Profile { source }
            | Self::Mesh { source, .. }
            | Self::Raycast { source, .. }
            | Self::Parity { source, .. }
            | Self::Convergence { source, .. } => source,
        }
    }
}

/// The `--define` grammar, printed on request and on error.
const DEFINE_HELP: &str = "\
--define KIND:key=value,key=value

  flat     d, flute
  ball     d, flute
  bull     d, corner, flute
  chamfer  d, tip, angle, flute
  vbit     d, angle, flute            (a chamfer mill with a pointed tip)
  taper    tip, angle, flute
  drill    d, angle, flute
  barrel   d, barrel, flute

  every kind also takes:
    shank    shank diameter, mm            (default: the cutting diameter)
    overall  tip to top of shank, mm       (default: flute + 30)
    holder   d1xL1/d2xL2/...  stages above the shank, bottom first;
             a stage may taper as d1:d2xL

  all lengths are millimetres, all angles are degrees and are the full
  included angle across the axis, as catalogues give them.

examples
  flat:d=6,flute=20
  bull:d=10,corner=2,flute=30,shank=8,overall=70
  drill:d=6.8,angle=118,flute=40
  barrel:d=12,barrel=200,flute=60,overall=100
  flat:d=6,flute=20,overall=45,holder=25x30/25:45x20
";

/// Parses `d1xL1/d2:d3xL2/...` into holder stages.
fn parse_holder(spec: &str) -> Result<Vec<HolderStage>, String> {
    let mut stages = Vec::new();
    for (i, part) in spec.split('/').enumerate() {
        let (diameters, length) = part.split_once('x').ok_or_else(|| {
            format!("holder stage {i} `{part}` should look like `25x30` or `25:45x30`")
        })?;
        let length: f64 = length
            .trim()
            .parse()
            .map_err(|_| format!("holder stage {i}: `{length}` is not a length"))?;
        let (bottom, top) = match diameters.split_once(':') {
            Some((a, b)) => (a, b),
            None => (diameters, diameters),
        };
        let parse = |s: &str| -> Result<f64, String> {
            s.trim()
                .parse()
                .map_err(|_| format!("holder stage {i}: `{s}` is not a diameter"))
        };
        stages.push(HolderStage::taper(parse(bottom)?, parse(top)?, length));
    }
    Ok(stages)
}

/// Builds a profile from a `--define` specification.
fn parse_define(spec: &str) -> Result<Profile, String> {
    if spec.trim() == "help" {
        return Err(DEFINE_HELP.to_owned());
    }
    let (kind, rest) = spec
        .split_once(':')
        .ok_or_else(|| format!("expected `KIND:key=value,...`, got `{spec}`\n\n{DEFINE_HELP}"))?;

    let mut keys: Vec<(String, f64)> = Vec::new();
    let mut holder = Vec::new();
    for field in rest.split(',') {
        let field = field.trim();
        if field.is_empty() {
            continue;
        }
        let (key, value) = field
            .split_once('=')
            .ok_or_else(|| format!("`{field}` is not `key=value`\n\n{DEFINE_HELP}"))?;
        let key = key.trim().to_ascii_lowercase();
        if key == "holder" {
            holder = parse_holder(value.trim())?;
            continue;
        }
        let value: f64 = value
            .trim()
            .parse()
            .map_err(|_| format!("`{value}` is not a number, for key `{key}`"))?;
        keys.push((key, value));
    }

    let get = |name: &str| -> Option<f64> { keys.iter().find(|(k, _)| k == name).map(|(_, v)| *v) };
    let need = |name: &'static str| -> Result<f64, String> {
        get(name)
            .ok_or_else(|| format!("`{kind}` needs `{name}`; it was not given\n\n{DEFINE_HELP}"))
    };

    let flute = need("flute")?;
    // The shank defaults to the cutting diameter, and the overall length to a
    // stick-out that clears the flutes. Both are conveniences for the common
    // question; a real library file states them.
    let shank_diameter = get("shank")
        .or_else(|| get("d"))
        .unwrap_or_else(|| 2.0 * get("tip").unwrap_or(3.0));
    let overall = get("overall").unwrap_or(flute + 30.0);
    let shank = if holder.is_empty() {
        Shank::plain(shank_diameter, overall)
    } else {
        Shank::with_holder(shank_diameter, overall, holder)
    };

    let profile = match kind.trim().to_ascii_lowercase().as_str() {
        "flat" => flat_end_mill(need("d")?, flute, &shank),
        "ball" => ball_end_mill(need("d")?, flute, &shank),
        "bull" => bull_end_mill(need("d")?, need("corner")?, flute, &shank),
        "chamfer" => chamfer_mill(
            need("d")?,
            get("tip").unwrap_or(0.0),
            need("angle")?,
            flute,
            &shank,
        ),
        "vbit" => chamfer_mill(need("d")?, 0.0, need("angle")?, flute, &shank),
        "taper" => tapered_end_mill(get("tip").unwrap_or(0.0), need("angle")?, flute, &shank),
        "drill" => drill(need("d")?, need("angle")?, flute, &shank),
        "barrel" => barrel_end_mill(need("d")?, need("barrel")?, flute, &shank),
        other => {
            return Err(format!("unknown tool kind `{other}`\n\n{DEFINE_HELP}"));
        }
    };
    profile.map_err(|e: catalog::CatalogError| e.to_string())
}

/// Resolves the tool the command is to operate on.
fn resolve(source: &ToolSource) -> Result<(String, Profile), String> {
    if let Some(spec) = &source.define {
        return Ok((
            "(defined on the command line)".to_owned(),
            parse_define(spec)?,
        ));
    }
    let Some(path) = &source.library else {
        return Err(format!(
            "give either --library FILE --id NAME or --define SPEC\n\n{DEFINE_HELP}"
        ));
    };
    let id = source.id.as_deref().unwrap_or_default();
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let library = ToolLibrary::from_json(&text).map_err(|e| e.to_string())?;
    let tool = library.get(id).ok_or_else(|| {
        let known: Vec<&str> = library.tools().iter().map(|t| t.id().as_str()).collect();
        format!(
            "{} has no tool `{id}`. It holds: {}",
            path.display(),
            if known.is_empty() {
                "nothing".to_owned()
            } else {
                known.join(", ")
            }
        )
    })?;
    Ok((tool.description().to_owned(), tool.profile().clone()))
}

fn element_json(index: usize, element: &ProfileElement, role: ElementRole) -> Value {
    let mut value = json!({
        "end_rz_mm": [element.end().x, element.end().y],
        "index": index,
        "length_mm": element.length(),
        "role": role.as_str(),
        "start_rz_mm": [element.start().x, element.start().y],
    });
    if let ProfileElement::Arc {
        center, direction, ..
    } = element
    {
        let map = value.as_object_mut().expect("an object");
        map.insert("center_rz_mm".to_owned(), json!([center.x, center.y]));
        map.insert("direction".to_owned(), json!(direction.as_str()));
        map.insert("kind".to_owned(), json!("arc"));
        map.insert("radius_mm".to_owned(), json!(element.radius()));
        if let Some((_, _, sweep)) = element.angles() {
            map.insert("sweep_deg".to_owned(), json!(sweep.to_degrees()));
        }
    } else if let Some(map) = value.as_object_mut() {
        map.insert("kind".to_owned(), json!("segment"));
    }
    value
}

/// Casts one bundle of `n` by `n` parallel rays along `axis` and returns the
/// volume implied, the statistics, and how many rays leaked.
fn bundle(profile: &Profile, axis: usize, n: usize) -> (f64, RaycastStats, u64) {
    let cylinder = profile.bounding_cylinder();
    let radius = cylinder.radius * 1.25 + 1.0;
    let z_lo = cylinder.z_min - 1.0;
    let z_hi = cylinder.z_max + 1.0;
    let (v_lo, v_hi) = if axis == 2 {
        (-radius, radius)
    } else {
        (z_lo, z_hi)
    };
    let du = 2.0 * radius / n as f64;
    let dv = (v_hi - v_lo) / n as f64;
    // Everything must lie within the ray's own travel through the bounds.
    let reach = 2.0 * (radius + cylinder.z_max + 2.0) + 2.0;

    let mut scratch = RaycastScratch::with_capacity(profile.len());
    let mut spans = chipbreaker_core::spans::Spans::new();
    let mut stats = RaycastStats::default();
    let mut total = 0.0f64;
    let mut leaks = 0u64;

    for i in 0..n {
        let u = -radius + (i as f64 + 0.5) * du;
        for j in 0..n {
            let v = v_lo + (j as f64 + 0.5) * dv;
            let (origin, direction) = match axis {
                0 => (Vec3::new(-radius - 1.0, u, v), Vec3::new(1.0, 0.0, 0.0)),
                1 => (Vec3::new(u, -radius - 1.0, v), Vec3::new(0.0, 1.0, 0.0)),
                _ => (Vec3::new(u, v, z_lo - 1.0), Vec3::new(0.0, 0.0, 1.0)),
            };
            let Some(ray) = Ray::new_normalized(origin, direction) else {
                continue;
            };
            profile.intersect_ray_into(&ray, &mut scratch, &mut spans, &mut stats);
            for span in spans.iter() {
                if !span.t0.is_finite() || !span.t1.is_finite() || span.t1 > reach {
                    leaks += 1;
                }
            }
            total += spans.measure();
        }
    }
    (total * du * dv, stats, leaks)
}

fn stats_json(s: &RaycastStats) -> Value {
    json!({
        "collapsed": s.collapsed,
        "crossings": s.crossings,
        "grazes": s.grazes,
        "rays": s.rays,
        "spans": s.spans,
        "tangencies": s.tangencies,
    })
}

/// Runs a `tool` subcommand.
///
/// # Errors
/// Returns a message suitable for stderr.
#[allow(clippy::too_many_lines)]
pub fn run(command: &ToolCommand) -> Result<(Value, String, bool), String> {
    let (description, profile) = resolve(command.source())?;

    match command {
        ToolCommand::Describe { .. } => {
            let cylinder = profile.bounding_cylinder();
            let profile_hash = {
                let mut h = CanonicalHash::new();
                h.add(&profile);
                h.finish().to_hex()
            };
            let results = json!({
                "bounding_cylinder": {
                    "radius_mm": cylinder.radius,
                    "z_max_mm": cylinder.z_max,
                    "z_min_mm": cylinder.z_min,
                },
                "command": "describe",
                "cutting_length_mm": profile.top_of_role(ElementRole::Cutting),
                "description": description,
                "diameter_mm": 2.0 * profile.max_radius(),
                "elements": profile.len(),
                "profile_hash": profile_hash,
                "surface_area_mm2": profile.surface_area(),
                "total_length_mm": profile.total_length(),
                "volume_by_role_mm3": {
                    "cutting": profile.volume_of_role(ElementRole::Cutting),
                    "holder": profile.volume_of_role(ElementRole::Holder),
                    "non_cutting": profile.volume_of_role(ElementRole::NonCutting),
                },
                "volume_mm3": profile.volume(),
            });
            let text = format!(
                "{description}\n\
                 diameter    {} mm\n\
                 length      {} mm (cutting to {})\n\
                 elements    {}\n\
                 volume      {} mm^3   (closed form, not tessellated)\n\
                 area        {} mm^2\n\
                 bounds      r <= {} mm, z in [{}, {}] mm\n",
                2.0 * profile.max_radius(),
                profile.total_length(),
                profile
                    .top_of_role(ElementRole::Cutting)
                    .map_or_else(|| "nothing".to_owned(), |z| format!("{z} mm")),
                profile.len(),
                profile.volume(),
                profile.surface_area(),
                cylinder.radius,
                cylinder.z_min,
                cylinder.z_max,
            );
            Ok((results, text, true))
        }

        ToolCommand::Profile { .. } => {
            let elements: Vec<Value> = profile
                .elements()
                .iter()
                .enumerate()
                .map(|(i, e)| element_json(i, &e.element, e.role))
                .collect();
            let mut text = format!("{} elements, tip to top\n", profile.len());
            for (i, e) in profile.elements().iter().enumerate() {
                let s = e.element.start();
                let d = e.element.end();
                let kind = match e.element {
                    ProfileElement::Segment { .. } => "segment".to_owned(),
                    ProfileElement::Arc { .. } => {
                        format!("arc r={:.4}", e.element.radius().unwrap_or(0.0))
                    }
                };
                text.push_str(&format!(
                    "  {i:>2}  {:<11}  ({:>9.4}, {:>9.4}) -> ({:>9.4}, {:>9.4})  {kind}\n",
                    e.role.as_str(),
                    s.x,
                    s.y,
                    d.x,
                    d.y,
                ));
            }
            Ok((
                json!({ "command": "profile", "elements": elements }),
                text,
                true,
            ))
        }

        ToolCommand::Mesh { tolerance, out, .. } => {
            let (mesh, report) = profile.tessellate(*tolerance).map_err(|e| e.to_string())?;
            let check = chipbreaker_core::mesh::validate::validate(&mesh);
            let mut results = json!({
                "command": "mesh",
                "is_solid": check.is_solid(),
                "mesh_hash": mesh.canonical_digest().to_hex(),
                "mesh_volume_mm3": mesh.signed_volume(),
                "shortfall_fraction": (profile.volume() - mesh.signed_volume()) / profile.volume(),
                "tessellation": {
                    "angular_divisions": report.divisions,
                    "deviation_bound_mm": report.bound,
                    "profile_stations": report.stations,
                    "tolerance_mm": report.tolerance,
                },
                "triangles": mesh.triangle_count(),
                "true_volume_mm3": profile.volume(),
                "vertices": mesh.vertex_count(),
            });

            let mut text = format!(
                "{} triangles, {} vertices ({} stations x {} divisions)\n\
                 mesh volume {} mm^3 against a true {} mm^3, short by {:.4}%\n\
                 the mesh is inscribed, so it never claims material the tool lacks\n",
                mesh.triangle_count(),
                mesh.vertex_count(),
                report.stations,
                report.divisions,
                mesh.signed_volume(),
                profile.volume(),
                100.0 * (profile.volume() - mesh.signed_volume()) / profile.volume(),
            );

            if let Some(path) = out {
                let (bytes, format) = crate::mesh::write_mesh(&mesh, path)?;
                if let Some(map) = results.as_object_mut() {
                    map.insert("bytes_written".to_owned(), json!(bytes));
                    map.insert("format".to_owned(), json!(format));
                    map.insert("out".to_owned(), json!(path.display().to_string()));
                }
                text.push_str(&format!("wrote {} ({bytes} bytes)\n", path.display()));
            }
            Ok((results, text, check.is_solid()))
        }

        ToolCommand::Raycast {
            origin, direction, ..
        } => {
            let ray = Ray::new_normalized(*origin, *direction)
                .ok_or_else(|| "the ray direction has no length".to_owned())?;
            let mut scratch = RaycastScratch::with_capacity(profile.len());
            let mut spans = chipbreaker_core::spans::Spans::new();
            let mut stats = RaycastStats::default();
            profile.intersect_ray_into(&ray, &mut scratch, &mut spans, &mut stats);

            let entries: Vec<Value> = spans
                .iter()
                .map(|s| {
                    let contact = |t: f64| {
                        let p = ray.at(t);
                        let c = profile.nearest_surface(
                            chipbreaker_core::transcendental::hypot(p.x, p.y),
                            p.z,
                        );
                        json!({
                            "element": if c.element == TOP_CAP {
                                json!("top-cap")
                            } else {
                                json!(c.element)
                            },
                            "point_mm": p.to_array(),
                            "role": c.role.as_str(),
                            "t_mm": t,
                        })
                    };
                    json!({
                        "enter": contact(s.t0),
                        "exit": contact(s.t1),
                        "length_mm": s.length(),
                    })
                })
                .collect();

            let mut text = format!(
                "{} span(s), {} mm of material along the ray\n",
                spans.len(),
                spans.measure()
            );
            for s in spans.iter() {
                let p = ray.at(s.t0);
                let c =
                    profile.nearest_surface(chipbreaker_core::transcendental::hypot(p.x, p.y), p.z);
                text.push_str(&format!(
                    "  t {:>10.5} .. {:>10.5}   {:>9.5} mm   enters {}\n",
                    s.t0,
                    s.t1,
                    s.length(),
                    c.role.as_str()
                ));
            }
            let results = json!({
                "command": "raycast",
                "direction": ray.direction.to_array(),
                "measure_mm": spans.measure(),
                "origin": ray.origin.to_array(),
                "spans": entries,
                "stats": stats_json(&stats),
            });
            Ok((results, text, true))
        }

        ToolCommand::Parity { rays, .. } => {
            let exact = profile.volume();
            let mut leaks_total = 0u64;
            let mut all = RaycastStats::default();
            let mut per_axis = Vec::new();
            let mut worst = 0.0f64;

            for axis in 0..3 {
                let (measured, stats, leaks) = bundle(&profile, axis, *rays);
                all.merge(&stats);
                leaks_total += leaks;
                let error = (measured - exact).abs() / exact;
                worst = worst.max(error);
                let name = ["x", "y", "z"][axis];
                per_axis.push(json!({
                    "axis": name,
                    "leaks": leaks,
                    "relative_error": error,
                    "stats": stats_json(&stats),
                    "volume_mm3": measured,
                }));
            }

            let ok = leaks_total == 0;
            let text = format!(
                "{} rays in three bundles of {rays} x {rays}\n\
                 {} spans, {} crossings, {} tangencies, {} collapsed, {} grazes\n\
                 closed-form volume {exact} mm^3, worst bundle {:.4}% out\n\
                 leaks: {leaks_total}{}\n",
                all.rays,
                all.spans,
                all.crossings,
                all.tangencies,
                all.collapsed,
                all.grazes,
                worst * 100.0,
                if ok {
                    "  (a leaking ray is a column of stock removed from nothing)"
                } else {
                    "  FAILED"
                },
            );
            let results = json!({
                "bundles": per_axis,
                "command": "parity",
                "leaks": leaks_total,
                "rays_per_side": rays,
                "stats": stats_json(&all),
                "true_volume_mm3": exact,
                "worst_relative_error": worst,
            });
            Ok((results, text, ok))
        }

        ToolCommand::Convergence {
            coarsest, finest, ..
        } => {
            let exact = profile.volume();
            let mut bundles = Vec::new();
            let mut text = format!(
                "closed-form volume {exact} mm^3\n\n\
                 ray bundle along +x\n     n    measured volume     error     O(1/n) bound\n"
            );
            let mut n = (*coarsest).max(2);
            while n <= *finest {
                let (measured, _, _) = bundle(&profile, 0, n);
                let error = (measured - exact).abs() / exact;
                let bound = 2.0 / n as f64;
                text.push_str(&format!(
                    "  {n:>4}  {measured:>16.4}  {:>8.4}%  {:>10.4}%\n",
                    error * 100.0,
                    bound * 100.0
                ));
                bundles.push(json!({
                    "bound": bound,
                    "n": n,
                    "relative_error": error,
                    "volume_mm3": measured,
                }));
                n *= 2;
            }

            let mut meshes = Vec::new();
            text.push_str("\ntessellation\n  tolerance     mesh volume   shortfall   solid\n");
            for tolerance in [0.2f64, 0.1, 0.05, 0.025, 0.0125, 0.00625] {
                let (mesh, _) = profile.tessellate(tolerance).map_err(|e| e.to_string())?;
                let measured = mesh.signed_volume();
                let solid = chipbreaker_core::mesh::validate::validate(&mesh).is_solid();
                let shortfall = (exact - measured) / exact;
                text.push_str(&format!(
                    "  {tolerance:>9}  {measured:>14.4}  {:>9.4}%   {}\n",
                    shortfall * 100.0,
                    if solid { "yes" } else { "NO" }
                ));
                meshes.push(json!({
                    "is_solid": solid,
                    "shortfall_fraction": shortfall,
                    "tolerance_mm": tolerance,
                    "triangles": mesh.triangle_count(),
                    "volume_mm3": measured,
                }));
            }

            let results = json!({
                "bundles": bundles,
                "command": "convergence",
                "meshes": meshes,
                "true_volume_mm3": exact,
            });
            Ok((results, text, true))
        }
    }
}
