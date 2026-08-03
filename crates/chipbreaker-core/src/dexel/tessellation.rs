// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! How far is this mesh from the smooth surface it approximates?
//!
//! # Why a field builder cares
//!
//! Unit 5's error table made the point numerically. A sphere's dexel sampling
//! error falls from 2.25e-3 to 1.63e-6 as the lattice refines — three orders of
//! magnitude — while its error against the *ideal* sphere stops improving at
//! 5.42e-4 and parks there. Below about `h/R = 1/40` a finer field bought
//! nothing, because the mesh itself had become the limit.
//!
//! So a customer who asks for 0.05 mm cells on a coarse STL is buying precision
//! their data cannot carry. Delivering it silently is not neutral: it produces a
//! confident-looking answer whose real error is a hundred times the cell size,
//! and nothing on the screen says so.
//!
//! # The estimate
//!
//! A tessellated smooth surface deviates from it by roughly the **sagitta** of
//! each chord. Across an edge of length `L` where the surface turns through a
//! dihedral angle `phi`, the underlying curvature radius is about `R = L / phi`,
//! and the chord's sagitta is
//!
//! ```text
//! s = R * (1 - cos(phi / 2))
//! ```
//!
//! This is a **proxy and is documented as one**. It assumes the mesh
//! approximates something smooth, which is false for a genuinely faceted part —
//! a cube's 90-degree edges are the model, not an approximation of one. So the
//! estimate is reported as a distribution rather than a single number, and the
//! adequacy check uses a high percentile rather than the maximum: one sharp
//! feature should not condemn a mesh that is fine everywhere else.
//!
//! Sharp edges are excluded outright above [`SHARP_EDGE_DEGREES`], on the
//! reasoning that nobody tessellates a smooth surface at 40 degrees per facet.
//! That threshold is a judgement, not a measurement, and is the first thing to
//! revisit if the warning turns out to fire on parts it should not.

use std::collections::BTreeMap;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec3;
use crate::mesh::TriMesh;
use crate::transcendental::{acos, cos};

/// Dihedral angles above this are treated as design intent, not tessellation.
///
/// A judgement rather than a measurement. Chosen because a mesher asked for a
/// smooth surface does not emit 40-degree facets; anything sharper is far more
/// likely to be a real edge.
pub const SHARP_EDGE_DEGREES: f64 = 40.0;

/// Percentile of the per-edge sagitta used for the adequacy check.
///
/// Not the maximum: a single sliver or one missed sharp edge should not condemn
/// a mesh that is well tessellated everywhere else.
pub const ADEQUACY_PERCENTILE: f64 = 0.95;

/// What the mesh's own fidelity looks like.
#[derive(Debug, Clone, PartialEq)]
pub struct TessellationEstimate {
    /// Interior edges considered, after sharp edges were excluded.
    pub edges: u64,
    /// Edges excluded as design intent rather than tessellation.
    pub sharp_edges: u64,
    /// Largest per-edge sagitta, in millimetres.
    pub max_sagitta_mm: f64,
    /// [`ADEQUACY_PERCENTILE`] of the per-edge sagitta, in millimetres.
    ///
    /// The number the adequacy check uses.
    pub percentile_sagitta_mm: f64,
    /// Mean edge length, in millimetres.
    pub mean_edge_mm: f64,
    /// Largest dihedral angle that was still counted as tessellation, degrees.
    pub max_dihedral_deg: f64,
}

impl TessellationEstimate {
    /// True if `spacing` is materially finer than the mesh can support.
    ///
    /// "Materially" is a factor of two, because a cell size within the same
    /// order as the mesh's own error is a reasonable ask — the field is then
    /// roughly as good as its input, which is the best anyone can do.
    #[must_use]
    pub fn is_finer_than_the_mesh_supports(&self, spacing: f64) -> bool {
        self.percentile_sagitta_mm > 0.0 && spacing * 2.0 < self.percentile_sagitta_mm
    }

    /// A sentence to put in front of a customer, if there is one to say.
    #[must_use]
    pub fn advice(&self, spacing: f64) -> Option<String> {
        if !self.is_finer_than_the_mesh_supports(spacing) {
            return None;
        }
        Some(format!(
            "the requested {spacing} mm cells are finer than this mesh supports: its own \
             deviation from the smooth surface it approximates is about {:.4} mm (95th \
             percentile over {} edges, worst {:.4} mm). The field will be more precise \
             than the input data, which is not the same as more accurate. Re-export the \
             mesh at a finer chord tolerance, or use cells near {:.3} mm and spend the \
             time elsewhere.",
            self.percentile_sagitta_mm,
            self.edges,
            self.max_sagitta_mm,
            self.percentile_sagitta_mm / 2.0,
        ))
    }
}

impl Hashable for TessellationEstimate {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("TessellationEstimate");
        h.u64(self.edges);
        h.u64(self.sharp_edges);
        h.f64(self.max_sagitta_mm);
        h.f64(self.percentile_sagitta_mm);
        h.f64(self.mean_edge_mm);
        h.f64(self.max_dihedral_deg);
        h.end();
    }
}

/// Estimates a mesh's deviation from the smooth surface it approximates.
///
/// Returns an all-zero estimate for a mesh with no interior edges, which is the
/// honest answer: nothing can be inferred from a triangle soup.
#[must_use]
pub fn estimate(mesh: &TriMesh) -> TessellationEstimate {
    // Edge -> the faces on it. A `BTreeMap` because the per-edge sagittas are
    // collected in its iteration order and then summed; an unordered map would
    // put a float sum behind a hasher.
    let mut edges: BTreeMap<[u32; 2], Vec<u32>> = BTreeMap::new();
    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let key = if a <= b { [a, b] } else { [b, a] };
            edges.entry(key).or_default().push(t);
        }
    }

    let mut sagittas: Vec<f64> = Vec::new();
    let mut lengths: Vec<f64> = Vec::new();
    let mut sharp = 0u64;
    let mut max_dihedral = 0.0f64;
    let sharp_limit = SHARP_EDGE_DEGREES * core::f64::consts::PI / 180.0;

    for (key, faces) in &edges {
        // Only manifold interior edges. A boundary or non-manifold edge says
        // something about the mesh's validity, which is Unit 2's report, not
        // this one's.
        if faces.len() != 2 {
            continue;
        }
        let a = mesh.vertices()[key[0] as usize];
        let b = mesh.vertices()[key[1] as usize];
        let length = (b - a).length();
        if length <= 0.0 {
            continue;
        }
        let (Some(n0), Some(n1)) = (face_normal(mesh, faces[0]), face_normal(mesh, faces[1]))
        else {
            continue;
        };

        // Clamped before `acos`: a dot product of two unit vectors can leave the
        // domain by an ulp, and `acos(1.0000000000000002)` is NaN.
        let dot = n0.dot(n1).clamp(-1.0, 1.0);
        let phi = acos(dot);
        if !phi.is_finite() || phi <= 0.0 {
            // Coplanar neighbours: a flat region says nothing about curvature.
            lengths.push(length);
            continue;
        }
        if phi > sharp_limit {
            sharp += 1;
            lengths.push(length);
            continue;
        }
        max_dihedral = max_dihedral.max(phi);

        // R = L / phi is the curvature radius the chord implies; the sagitta is
        // the height of the arc over the chord.
        let radius = length / phi;
        sagittas.push(radius * (1.0 - cos(phi / 2.0)));
        lengths.push(length);
    }

    // `total_cmp`, not `partial_cmp`: a total order over every f64, and no
    // unwrap. The sort is what makes the percentile reproducible.
    sagittas.sort_by(f64::total_cmp);
    let percentile = if sagittas.is_empty() {
        0.0
    } else {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "an index into a vector whose length is known finite"
        )]
        let index = ((sagittas.len() as f64 - 1.0) * ADEQUACY_PERCENTILE).round() as usize;
        sagittas[index.min(sagittas.len() - 1)]
    };

    TessellationEstimate {
        edges: sagittas.len() as u64,
        sharp_edges: sharp,
        max_sagitta_mm: sagittas.last().copied().unwrap_or(0.0),
        percentile_sagitta_mm: percentile,
        mean_edge_mm: if lengths.is_empty() {
            0.0
        } else {
            lengths.iter().sum::<f64>() / lengths.len() as f64
        },
        max_dihedral_deg: max_dihedral * 180.0 / core::f64::consts::PI,
    }
}

/// Unit normal of a triangle, or `None` if it is degenerate.
fn face_normal(mesh: &TriMesh, triangle: u32) -> Option<Vec3> {
    let [a, b, c] = mesh.triangle(triangle);
    let n = (b - a).cross(c - a);
    let length = n.length();
    if length > 0.0 {
        Some(n * (1.0 / length))
    } else {
        None
    }
}
