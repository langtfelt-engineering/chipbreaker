// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! How many spans does a dexel ray actually carry?
//!
//! This measurement comes **before** the arena design, not after, because the
//! arena's whole justification is the shape of this distribution. A general
//! allocator would be the right answer if span counts were spread out; a packed
//! inline representation with a spill path is the right answer if they are
//! brutally skewed. Guessing which would be guessing at the central data
//! structure of the product.
//!
//! Deliberately built on a plain `Vec<Spans>` — the representation the arena is
//! meant to replace. Measuring with the thing being designed would be circular.
//!
//! Run with: `cargo run --release -p chipbreaker-core --example span_distribution`

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::mesh::bvh::Bvh;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::spans::{Span, Spans};

/// Casts one bundle of `+Z` rays at cell centres and returns the span count of
/// every ray, plus how many coplanar rejections the caster reported.
fn span_counts(mesh: &TriMesh, spacing: f64) -> (Vec<usize>, u64, u64) {
    let bounds = mesh.bounds();
    let bvh = Bvh::build(mesh);
    let extent = bounds.extent();

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "counts are small and positive by construction"
    )]
    let nx = ((extent.x / spacing).ceil() as usize).max(1);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "counts are small and positive by construction"
    )]
    let ny = ((extent.y / spacing).ceil() as usize).max(1);

    let mut counts = Vec::with_capacity(nx * ny);
    let mut hits = Vec::new();
    let mut coplanar = 0u64;
    let mut odd = 0u64;
    let below = bounds.min.z - 1.0;

    for i in 0..nx {
        for j in 0..ny {
            // Cell centres. ADR 0001 Part 2: never cell corners.
            let x = bounds.min.x + (i as f64 + 0.5) * spacing;
            let y = bounds.min.y + (j as f64 + 0.5) * spacing;
            let Some(ray) = Ray::new_normalized(Vec3::new(x, y, below), Vec3::new(0.0, 0.0, 1.0))
            else {
                continue;
            };
            let Ok(stats) = bvh.intersect_ray_all_into(mesh, &ray, &mut hits) else {
                continue;
            };
            coplanar += stats.coplanar_rejected;
            if !hits.len().is_multiple_of(2) {
                odd += 1;
                continue;
            }
            let mut spans = Spans::with_capacity(hits.len() / 2);
            for pair in hits.chunks_exact(2) {
                spans.push_merge(Span::ordered(pair[0].t, pair[1].t));
            }
            counts.push(spans.len());
        }
    }
    (counts, coplanar, odd)
}

fn histogram(counts: &[usize]) -> Vec<(usize, usize)> {
    let mut buckets: std::collections::BTreeMap<usize, usize> = std::collections::BTreeMap::new();
    for &c in counts {
        *buckets.entry(c).or_default() += 1;
    }
    buckets.into_iter().collect()
}

fn report(name: &str, mesh: &TriMesh, spacing: f64) {
    let (counts, coplanar, odd) = span_counts(mesh, spacing);
    let filled = counts.iter().filter(|c| **c > 0).count();
    let total_spans: usize = counts.iter().sum();
    let max = counts.iter().copied().max().unwrap_or(0);

    println!(
        "--- {name} ({} triangles, spacing {spacing}) ---",
        mesh.triangle_count()
    );
    println!(
        "  {} rays, {filled} filled, {} empty, {total_spans} spans, max {max} on one ray",
        counts.len(),
        counts.len() - filled
    );
    if coplanar > 0 || odd > 0 {
        println!("  !! coplanar rejections {coplanar}, odd crossing counts {odd}");
    }
    for (spans, rays) in histogram(&counts) {
        let share = rays as f64 / counts.len() as f64 * 100.0;
        let bar = "#".repeat(((share / 2.0).round() as usize).min(50));
        println!("  {spans:>3} span(s)  {rays:>8} rays  {share:>6.2}%  {bar}");
    }
    // The number that decides the arena: what inline capacity covers almost
    // everything?
    for cap in [1usize, 2, 4] {
        let within = counts.iter().filter(|c| **c <= cap).count();
        println!(
            "  inline capacity {cap}: covers {:.3}% of rays",
            within as f64 / counts.len() as f64 * 100.0
        );
    }
    println!();
}

fn main() {
    // Stock at rest: the case that dominates before anything is cut.
    report(
        "box (stock at rest)",
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 60.0, 25.0)),
        0.5,
    );

    // A sphere: still one span per filled ray, but a large empty fraction.
    report("sphere r=20", &shapes::icosphere(20.0, 3), 0.5);

    // A torus: the hole means two spans through the middle band, which is the
    // arena's tail.
    report("torus R=20 r=6", &shapes::torus(20.0, 6.0, 64, 32), 0.5);

    // Nested shells: the sphere-in-a-sphere from the mesh corpus. Four crossings, two
    // spans, on every ray through the middle.
    let mut nested = shapes::icosphere(20.0, 3);
    let inner = shapes::icosphere(10.0, 3);
    let offset = nested.vertex_count();
    let mut vertices = nested.vertices().to_vec();
    let mut triangles = nested.triangles().to_vec();
    vertices.extend_from_slice(inner.vertices());
    // The inner shell is reversed, so it bounds a cavity rather than a solid.
    triangles.extend(
        inner
            .triangles()
            .iter()
            .map(|t| [t[0] + offset, t[2] + offset, t[1] + offset]),
    );
    nested = TriMesh::new(vertices, triangles, nested.meta().clone()).expect("valid");
    report("nested shells (cavity)", &nested, 0.5);

    // The lattice block: every vertex an integer, which is the mesh that would
    // trigger coplanar rejections if ray origins ever moved to cell corners.
    report(
        "lattice block (integer vertices)",
        &shapes::lattice_block(9),
        0.5,
    );
}
