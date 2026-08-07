// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The numbers Unit 6 and Unit 10 have to budget against.
//!
//! Three reports, run together because they answer one question between them:
//! is a single-axis field safe, and what does it cost?
//!
//! 1. **The safety gate.** Coplanar rejections and odd crossing counts across
//!    every corpus mesh. Both must be zero. ADR 0001 Part 2 makes either one a
//!    hard error that aborts construction, and the cell-centre ray offset is what
//!    makes them unreachable for axis-aligned stock. A non-zero count here means
//!    the invariant has stopped holding.
//!
//! 2. **Memory.** Bytes per cubic centimetre at a range of cell sizes. Unit 6 is
//!    three bundles, so it is three times this. Unit 10's adaptive resolution has
//!    to beat it.
//!
//! 3. **Both error columns.** Against `TriMesh::signed_volume`, which isolates
//!    dexel sampling error, and against the analytic solid, which adds Unit 3's
//!    tessellation error. The total pipeline budget, visible in one place.
//!
//! Run with:
//! `cargo run --release -p chipbreaker-core --example dexel_budget`

#![allow(missing_docs, reason = "an example binary, not API")]

use std::path::PathBuf;

use chipbreaker_core::dexel::convergence::{measure, standard_cases, standard_ratios};
use chipbreaker_core::dexel::{Arena, BuildOptions, DexelField, Lattice};
use chipbreaker_core::math::{Aabb3, Axis, Mat4, Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::validate::validate;
use chipbreaker_core::mesh::{TriMesh, io, shapes, weld};

/// Cell sizes for the memory budget, in millimetres.
const SPACINGS: [f64; 4] = [0.4, 0.2, 0.1, 0.05];

// --- 1. the safety gate ----------------------------------------------------

/// Mirrors what `DexelField::build` removes before casting.
fn drop_degenerate_like_build(mesh: &TriMesh) -> TriMesh {
    use chipbreaker_core::mesh::validate::collinear_exact;
    let degenerate = |i: u32| {
        let t = mesh.triangles()[i as usize];
        t[0] == t[1] || t[1] == t[2] || t[2] == t[0] || {
            let [a, b, c] = mesh.triangle(i);
            collinear_exact(a, b, c)
        }
    };
    let kept: Vec<[u32; 3]> = (0..mesh.triangle_count())
        .filter(|i| !degenerate(*i))
        .map(|i| mesh.triangles()[i as usize])
        .collect();
    TriMesh::new(mesh.vertices().to_vec(), kept, mesh.meta().clone()).expect("subset")
}

struct Gate {
    rays: u64,
    coplanar: u64,
    odd: u64,
}

/// Casts a bundle and **counts** rather than aborting.
///
/// Construction treats either condition as a hard error, so it cannot report a
/// distribution. This is the same cast with the abort removed, purely so the
/// numbers can be stated.
fn gate(mesh: &TriMesh, spacing: f64, align_to_vertices: bool) -> Gate {
    // Degenerate triangles are dropped, exactly as construction drops them.
    // Without this the gate measures a different thing from the code it is a
    // gate on -- which is how the discrepancy in this file was found.
    let mesh = &drop_degenerate_like_build(mesh);
    let bvh = Bvh::build(mesh);
    let bounds = mesh.bounds();
    let extent = bounds.extent();
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "counts are small and positive by construction"
    )]
    let nx = ((extent.x / spacing).ceil() as u32).max(1);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "counts are small and positive by construction"
    )]
    let ny = ((extent.y / spacing).ceil() as u32).max(1);

    // The whole point of the comparison: `0.5` is the invariant, `0.0` is what
    // somebody would write if they "simplified" it away.
    let offset = if align_to_vertices { 0.0 } else { 0.5 };
    let mut out = Gate {
        rays: 0,
        coplanar: 0,
        odd: 0,
    };
    let mut hits = Vec::new();
    for i in 0..nx {
        for j in 0..ny {
            let ray = Ray {
                origin: Vec3::new(
                    bounds.min.x + (f64::from(i) + offset) * spacing,
                    bounds.min.y + (f64::from(j) + offset) * spacing,
                    bounds.min.z - spacing,
                ),
                direction: Vec3::new(0.0, 0.0, 1.0),
            };
            let Ok(stats) = bvh.intersect_ray_all_into(mesh, &ray, &mut hits) else {
                continue;
            };
            out.rays += 1;
            out.coplanar += stats.coplanar_rejected;
            if !hits.len().is_multiple_of(2) {
                out.odd += 1;
            }
        }
    }
    out
}

fn corpus_meshes() -> Vec<(String, TriMesh)> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("tests/corpus/mesh");
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| {
                    p.extension()
                        .and_then(|x| x.to_str())
                        .is_some_and(|x| matches!(x, "stl" | "obj" | "3mf"))
                })
                .collect()
        })
        .unwrap_or_default();
    // Sorted, or the report would depend on the directory's iteration order.
    paths.sort();

    let mut out = Vec::new();
    for path in paths {
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("?")
            .to_owned();
        let Ok(bytes) = std::fs::read(&path) else {
            continue;
        };
        let format = io::detect(&bytes, path.file_name().and_then(|s| s.to_str()));
        let unit = chipbreaker_core::mesh::units::Unit::Millimetre;
        let raw = match format {
            io::Format::StlBinary => io::stl::read_binary(&bytes, unit).ok(),
            io::Format::StlAscii => {
                io::stl::read_ascii(&String::from_utf8_lossy(&bytes), unit).ok()
            }
            io::Format::Obj => io::obj::read(&String::from_utf8_lossy(&bytes), unit).ok(),
            io::Format::ThreeMf => io::threemf::read(&bytes, None).ok(),
        };
        let Some(raw) = raw else { continue };
        let Ok((welded, _)) = weld::weld(&raw, chipbreaker_core::eps::EPS_WELD) else {
            continue;
        };
        // Only closed solids: the corpus deliberately contains broken meshes,
        // and a mesh with a hole in it will of course produce odd crossings.
        // Those are Unit 2's business, not this gate's.
        if validate(&welded).is_solid() {
            out.push((name, welded));
        }
    }
    out
}

fn safety_gate() {
    println!("=== 1. SAFETY GATE: coplanar rejections and odd crossing counts ===");
    println!();
    println!("Construction aborts on either. Both must be zero across the corpus.");
    println!();
    println!(
        "  {:<34} {:>10} {:>10} {:>8}",
        "mesh", "rays", "coplanar", "odd"
    );

    let mut total_rays = 0u64;
    let mut total_coplanar = 0u64;
    let mut total_odd = 0u64;
    let mut meshes = corpus_meshes();

    // The synthetic shapes too, including the ones the dexel corpus uses.
    meshes.push(("synth:lattice-block-5".to_owned(), shapes::lattice_block(5)));
    meshes.push(("synth:lattice-block-9".to_owned(), shapes::lattice_block(9)));
    meshes.push((
        "synth:box".to_owned(),
        shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0)),
    ));
    meshes.push(("synth:sphere".to_owned(), shapes::icosphere(12.0, 4)));
    meshes.push((
        "synth:cylinder".to_owned(),
        shapes::cylinder(10.0, 20.0, 128),
    ));
    meshes.push(("synth:torus".to_owned(), shapes::torus(15.0, 4.0, 96, 48)));
    meshes.push(("synth:cone".to_owned(), shapes::cone(10.0, 20.0, 128)));

    for (name, mesh) in &meshes {
        let g = gate(mesh, 0.5, false);
        total_rays += g.rays;
        total_coplanar += g.coplanar;
        total_odd += g.odd;
        println!(
            "  {name:<34} {:>10} {:>10} {:>8}{}",
            g.rays,
            g.coplanar,
            g.odd,
            if g.coplanar > 0 || g.odd > 0 {
                "   <-- GATE FAILED"
            } else {
                ""
            }
        );
    }
    println!();
    println!(
        "  {:<34} {total_rays:>10} {total_coplanar:>10} {total_odd:>8}",
        format!("TOTAL ({} meshes)", meshes.len())
    );
    println!();
    println!(
        "  gate: {}",
        if total_coplanar == 0 && total_odd == 0 {
            "PASS -- zero coplanar rejections, zero odd crossing counts"
        } else {
            "FAIL"
        }
    );

    // And the other half: the invariant has to be load-bearing, not merely
    // satisfied. On a mesh whose vertices are all integers, moving ray origins
    // onto the integer lattice must break it.
    println!();
    println!("  Does the invariant matter? The same lattice block, origins moved to");
    println!("  cell CORNERS instead of cell centres:");
    println!();
    println!(
        "  {:<34} {:>10} {:>10} {:>8}",
        "mesh (origins on integers)", "rays", "coplanar", "odd"
    );
    for n in [5u32, 9] {
        let mesh = shapes::lattice_block(n);
        let g = gate(&mesh, 1.0, true);
        println!(
            "  {:<34} {:>10} {:>10} {:>8}{}",
            format!("synth:lattice-block-{n}"),
            g.rays,
            g.coplanar,
            g.odd,
            if g.coplanar > 0 {
                "   <-- would abort the build"
            } else {
                ""
            }
        );
    }

    // Confirm the hard error actually fires, through the real construction path.
    println!();
    let left = shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.5, 4.0, 4.0));
    let right = shapes::box_solid(Vec3::new(2.5, 0.0, 0.0), Vec3::new(4.0, 4.0, 4.0));
    let offset = left.vertex_count();
    let mut vertices = left.vertices().to_vec();
    let mut triangles = left.triangles().to_vec();
    vertices.extend_from_slice(right.vertices());
    triangles.extend(
        right
            .triangles()
            .iter()
            .map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]),
    );
    let aligned =
        TriMesh::new(vertices, triangles, left.meta().clone()).expect("indices unchanged");
    let result = DexelField::build(
        &aligned,
        &BuildOptions {
            spacing_xyz: None,
            spacing: 1.0,
            ..BuildOptions::default()
        },
    );
    println!("  Deliberately lattice-aligned mesh (two boxes meeting at x = 2.5,");
    println!("  1 mm cells, so ray origins land exactly on the shared face):");
    match result {
        Err(e) => println!("    build refused, as required: {e}"),
        Ok(_) => println!("    !! BUILD SUCCEEDED -- the hard error did not fire"),
    }
    println!();
}

// --- 2. memory -------------------------------------------------------------

/// Arena bytes for a workspace at a given spacing.
///
/// Computed from the lattice rather than by building, because 0.05 mm over a
/// 100 mm part is four million rays and the answer does not depend on what the
/// rays find: the inline slots are allocated per ray whatever is on them. Spill
/// is the exception, and the measured cases below confirm it stays at zero.
fn arena_bytes(bounds: Aabb3, spacing: f64) -> Option<(usize, usize)> {
    let lattice = Lattice::new(bounds, spacing, Axis::Z).ok()?;
    let arena = Arena::new(lattice.ray_count());
    Some((lattice.ray_count(), arena.bytes()))
}

fn memory_budget() {
    println!("=== 2. MEMORY BUDGET ===");
    println!();
    println!("Unit 6 is three bundles, so it is 3x every figure here.");
    println!("Unit 10's adaptive resolution has to beat it.");
    println!();

    // Memory per cm^3 depends on the part's DEPTH along the bundle, not only on
    // the spacing: the lattice is two-dimensional, so a deeper part amortises the
    // same rays over more volume. Three shapes make that visible.
    let parts: [(&str, Aabb3); 3] = [
        (
            "plate 100 x 100 x 10 mm",
            Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 10.0)),
        ),
        (
            "block 100 x 100 x 50 mm",
            Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 50.0)),
        ),
        (
            "bar 100 x 100 x 200 mm",
            Aabb3::from_min_max(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 200.0)),
        ),
    ];

    for (name, bounds) in parts {
        let extent = bounds.extent();
        let cm3 = extent.x * extent.y * extent.z / 1000.0;
        println!("  {name}  ({cm3:.1} cm^3 of workspace)");
        println!(
            "    {:>8}  {:>14}  {:>12}  {:>14}  {:>14}",
            "h (mm)", "rays", "MiB", "KiB per cm^3", "3 bundles MiB"
        );
        for spacing in SPACINGS {
            match arena_bytes(bounds, spacing) {
                Some((rays, bytes)) => println!(
                    "    {spacing:>8}  {rays:>14}  {:>12.1}  {:>14.1}  {:>14.1}",
                    bytes as f64 / (1024.0 * 1024.0),
                    bytes as f64 / 1024.0 / cm3,
                    3.0 * bytes as f64 / (1024.0 * 1024.0),
                ),
                None => println!("    {spacing:>8}  refused: more rays than a u32 can address"),
            }
        }
        println!();
    }

    println!(
        "  Per ray: {} bytes ({} inline spans at {} bytes, plus a u16 length).",
        chipbreaker_core::dexel::INLINE_CAPACITY * size_of::<chipbreaker_core::spans::Span>()
            + size_of::<u16>(),
        chipbreaker_core::dexel::INLINE_CAPACITY,
        size_of::<chipbreaker_core::spans::Span>(),
    );
    println!("  Memory scales as 1/h^2, not 1/h^3: the lattice is two-dimensional and");
    println!("  the third dimension is exact. Halving the cell size quadruples memory.");
    println!("  Per cm^3 it also falls with depth along the bundle, because the same");
    println!("  rays cover more volume -- a bar is far cheaper per cm^3 than a plate.");
    println!();

    // And confirm the assumption the table rests on: real parts do not spill.
    println!("  Spill check on real geometry (spill is the only allocation the table");
    println!("  does not account for):");
    println!(
        "    {:<28} {:>10} {:>12} {:>10}",
        "mesh", "rays", "total spans", "spilled"
    );
    for (name, mesh, spacing) in [
        (
            "box at rest",
            shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 40.0, 20.0)),
            0.2,
        ),
        ("torus", shapes::torus(15.0, 4.0, 96, 48), 0.2),
        ("lattice block", shapes::lattice_block(9), 0.2),
    ] {
        let (field, _) = DexelField::build(
            &mesh,
            &BuildOptions {
                spacing_xyz: None,
                spacing,
                ..BuildOptions::default()
            },
        )
        .expect("builds");
        println!(
            "    {name:<28} {:>10} {:>12} {:>10}",
            field.arena().rays(),
            field.total_spans(),
            field.arena().spilled_rays()
        );
    }
    println!();
}

// --- 3. both error columns -------------------------------------------------

fn error_budget() {
    println!("=== 3. THE FULL PIPELINE ERROR BUDGET ===");
    println!();
    println!("vs MESH      dexel sampling error alone. The mesh is exactly what the");
    println!("             rays met, so this is the transverse sum and nothing else.");
    println!("vs ANALYTIC  sampling PLUS Unit 3's tessellation error. This is the whole");
    println!("             distance from reality, and it does NOT converge with h --");
    println!("             refining the lattice does nothing about the tessellation.");
    println!();

    for case in standard_cases() {
        let result = measure(&case, &standard_ratios());
        println!("  {}", result.name);
        println!(
            "    {:>8}  {:>12}  {:>12}  {:>12}",
            "h/R", "vs mesh", "vs analytic", "tessellation"
        );
        for sample in &result.samples {
            // What is left when the sampling error is taken out: the floor the
            // analytic column converges to.
            let floor = sample
                .analytic_volume
                .map(|v| (sample.mesh_volume - v).abs() / v);
            println!(
                "    {:>8.5}  {:>12.3e}  {:>12}  {:>12}",
                sample.ratio,
                sample.mesh_error(),
                sample
                    .analytic_error()
                    .map_or_else(|| "--".to_owned(), |e| format!("{e:.3e}")),
                floor.map_or_else(|| "--".to_owned(), |e| format!("{e:.3e}")),
            );
        }
        println!();
    }
    println!("  The `tessellation` column is constant down each table by construction:");
    println!("  it is the mesh's own error against the ideal solid, and no lattice");
    println!("  refinement touches it. Once the dexel error drops below it, the");
    println!("  analytic column stops improving. That is the point at which a finer");
    println!("  field buys nothing and a finer TESSELLATION is what is needed instead.");
    println!();
}

fn main() {
    // Placement is not varied here: all three reports are about the field's own
    // behaviour, and an off-origin placement would only move the numbers around.
    let _ = Mat4::IDENTITY;
    safety_gate();
    memory_budget();
    error_budget();
}
