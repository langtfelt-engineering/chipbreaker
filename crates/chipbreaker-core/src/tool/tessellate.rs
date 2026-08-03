// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Turning a tool into a triangle mesh, with a stated error bound.
//!
//! # What the tolerance means
//!
//! `tolerance` is a **maximum deviation, in length units, between the mesh and
//! the true surface** — not a subdivision count, not a fraction, and not a hint.
//! Every mesh this module produces is inscribed in the solid, so the deviation
//! is one-sided: the mesh is never larger than the tool.
//!
//! That is deliberate, and it is the property the rest of the engine depends on.
//! A mesh that is a subset of the tool cannot report material removed that the
//! tool did not reach, so a preview built from it is conservative in the safe
//! direction. Splitting the deviation either side of the true surface would
//! halve the triangle count for the same tolerance and would make every
//! downstream guarantee two-sided.
//!
//! # Where the deviation comes from
//!
//! Two independent approximations, each bounded by the same tolerance, so the
//! total is bounded by twice it and typically much less:
//!
//! * **Along the profile.** An arc of radius `rho` spanned by a chord over angle
//!   `phi` deviates by the sagitta `rho (1 - cos(phi/2))`. Solving for `phi`
//!   gives the number of chords. Straight segments are exact and get one chord.
//! * **Around the axis.** A circle of radius `r` approximated by an inscribed
//!   `m`-gon deviates by `r (1 - cos(PI/m))`. The largest radius on the tool
//!   sets `m`, so the tolerance holds everywhere rather than on average.
//!
//! # Determinism
//!
//! The subdivision counts come from [`crate::transcendental`], and the vertex
//! order is fixed: profile station outer, angular station inner, tip first. Two
//! runs on two platforms produce byte-identical meshes, which is what makes the
//! mesh hashable and the golden files meaningful.

use crate::eps::EPS_LENGTH;
use crate::math::Vec3;
use crate::mesh::{MeshError, MeshMeta, TriMesh};
use crate::transcendental as t;

use super::Tool;
use super::profile::{Profile, ProfileElement};

use core::f64::consts::PI;

/// Fewest angular divisions, however coarse the tolerance.
///
/// Below this the solid is not recognisably a solid of revolution: three
/// divisions give a triangular prism, whose volume is 41% of the cylinder it
/// claims to approximate. A tolerance loose enough to permit that is a tolerance
/// the caller has got wrong, and clamping is friendlier than obeying.
pub const MIN_ANGULAR_DIVISIONS: usize = 8;

/// Most angular divisions, whatever the tolerance.
///
/// A guard against a tolerance of zero, or one so small that the subdivision
/// count overflows before the mesh does.
pub const MAX_ANGULAR_DIVISIONS: usize = 4096;

/// Most chords used for a single arc.
pub const MAX_ARC_CHORDS: usize = 4096;

/// Chords needed to hold an arc of radius `rho` sweeping `sweep` radians to
/// within `tolerance`.
///
/// From the sagitta of a chord subtending `phi`: `rho (1 - cos(phi/2))`. A
/// tolerance at or above the radius is satisfied by a single chord, and the
/// `acos` argument is clamped so that it cannot leave its domain.
#[must_use]
pub fn arc_chords(rho: f64, sweep: f64, tolerance: f64) -> usize {
    let sweep = sweep.abs();
    if rho <= 0.0 || sweep <= 0.0 {
        return 1;
    }
    if tolerance >= rho {
        return 1;
    }
    let half = t::acos((1.0 - tolerance / rho).clamp(-1.0, 1.0));
    if half <= 0.0 {
        return MAX_ARC_CHORDS;
    }
    let count = (sweep / (2.0 * half)).ceil();
    if count.is_finite() {
        (count as usize).clamp(1, MAX_ARC_CHORDS)
    } else {
        MAX_ARC_CHORDS
    }
}

/// Angular divisions needed to hold a circle of radius `radius` to within
/// `tolerance`.
#[must_use]
pub fn angular_divisions(radius: f64, tolerance: f64) -> usize {
    if radius <= 0.0 || tolerance >= radius {
        return MIN_ANGULAR_DIVISIONS;
    }
    let half = t::acos((1.0 - tolerance / radius).clamp(-1.0, 1.0));
    if half <= 0.0 {
        return MAX_ANGULAR_DIVISIONS;
    }
    let count = (PI / half).ceil();
    if count.is_finite() {
        (count as usize).clamp(MIN_ANGULAR_DIVISIONS, MAX_ANGULAR_DIVISIONS)
    } else {
        MAX_ANGULAR_DIVISIONS
    }
}

/// Why a tessellation was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum TessellateError {
    /// The tolerance must be a positive, finite length.
    BadTolerance {
        /// What was supplied.
        found: f64,
    },
    /// The mesh would exceed what a `u32` index can address.
    TooLarge {
        /// Vertices the settings would have produced.
        vertices: usize,
    },
    /// The assembled mesh failed validation.
    Mesh(MeshError),
}

impl core::fmt::Display for TessellateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BadTolerance { found } => {
                write!(f, "tolerance must be a positive finite length, got {found}")
            }
            Self::TooLarge { vertices } => write!(
                f,
                "the requested tolerance needs {vertices} vertices, more than a u32 index \
                 can address; ask for a looser one"
            ),
            Self::Mesh(e) => write!(f, "{e}"),
        }
    }
}

impl core::error::Error for TessellateError {}

impl From<MeshError> for TessellateError {
    fn from(e: MeshError) -> Self {
        Self::Mesh(e)
    }
}

/// How a mesh was built, and how far it can be from the truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TessellationReport {
    /// The tolerance asked for.
    pub tolerance: f64,
    /// Angular divisions used.
    pub divisions: usize,
    /// Stations along the profile, including both ends.
    pub stations: usize,
    /// The largest deviation the construction admits: the sum of the two
    /// one-sided bounds, in length units.
    pub bound: f64,
}

/// The `(r, z)` stations a profile is sampled at, from the tip upward.
fn stations(profile: &Profile, tolerance: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    out.push((0.0, 0.0));
    for e in profile.elements() {
        match e.element {
            ProfileElement::Segment { end, .. } => out.push((end.x, end.y)),
            ProfileElement::Arc { .. } => {
                let rho = e.element.radius().unwrap_or(0.0);
                let sweep = e.element.angles().map_or(0.0, |(_, _, s)| s);
                let chords = arc_chords(rho, sweep, tolerance);
                for k in 1..=chords {
                    let p = e.element.point_at(k as f64 / chords as f64);
                    out.push((p.x, p.y));
                }
            }
        }
    }
    // Drop stations that repeat the previous one; a degenerate ring would make
    // a band of zero-area triangles that validation would then have to report.
    out.dedup_by(|a, b| (a.0 - b.0).abs() <= EPS_LENGTH && (a.1 - b.1).abs() <= EPS_LENGTH);
    out
}

impl Profile {
    /// Tessellates the solid to within `tolerance` of its true surface.
    ///
    /// The mesh is inscribed: every vertex is exactly on the surface and every
    /// triangle lies inside the solid, so it never claims material the tool does
    /// not have. See the module header for what the tolerance bounds.
    ///
    /// # Errors
    ///
    /// [`TessellateError::BadTolerance`] for a tolerance that is not a positive
    /// finite length, and [`TessellateError::TooLarge`] when the resulting mesh
    /// would not fit `u32` indices.
    pub fn tessellate(
        &self,
        tolerance: f64,
    ) -> Result<(TriMesh, TessellationReport), TessellateError> {
        if !tolerance.is_finite() || tolerance <= 0.0 {
            return Err(TessellateError::BadTolerance { found: tolerance });
        }

        let stations = stations(self, tolerance);
        let divisions = angular_divisions(self.max_radius(), tolerance);

        // One vertex per (station, division), except where a station sits on the
        // axis and collapses to a single point.
        let estimate = stations.len() * divisions + stations.len();
        if u32::try_from(estimate).is_err() {
            return Err(TessellateError::TooLarge { vertices: estimate });
        }

        let mut cos_sin = Vec::with_capacity(divisions);
        for k in 0..divisions {
            let angle = 2.0 * PI * (k as f64) / (divisions as f64);
            let (sin, cos) = t::sin_cos(angle);
            cos_sin.push((cos, sin));
        }

        let mut vertices: Vec<Vec3> = Vec::with_capacity(estimate);
        let mut triangles: Vec<[u32; 3]> = Vec::new();
        // Where each station's ring begins, and whether it is a single point.
        let mut ring_start: Vec<u32> = Vec::with_capacity(stations.len());
        let mut ring_is_point: Vec<bool> = Vec::with_capacity(stations.len());

        for &(r, z) in &stations {
            ring_start.push(vertices.len() as u32);
            if r <= EPS_LENGTH {
                ring_is_point.push(true);
                vertices.push(Vec3::new(0.0, 0.0, z));
            } else {
                ring_is_point.push(false);
                for &(cos, sin) in &cos_sin {
                    vertices.push(Vec3::new(r * cos, r * sin, z));
                }
            }
        }

        // Bands between consecutive stations.
        //
        // # Winding
        //
        // Every triangle must face outward, or U2's validator reports the mesh
        // as inconsistently oriented and its signed volume comes out wrong. The
        // side bands and the two degenerate cases do *not* share a winding, and
        // the difference is easy to get backwards, so each is derived here.
        //
        // Take `k` at angle zero, `k1` a hair further round, radius `r`, the
        // lower ring at `z0` and the upper at `z1`. For the full quad,
        // `[lo+k, lo+k1, hi+k1]` has edges `(0, r eps, 0)` and `(0, r eps, h)`,
        // whose cross product is `(r eps h, 0, 0)` — radially outward at angle
        // zero, which is correct.
        //
        // For a ring collapsed to a point *below* a ring — the tip of a ball
        // nose, and also the flat bottom of an end mill, where the two stations
        // share a `z` — the outward normal points down and out, and it is
        // `[lo, hi+k1, hi+k]` that gives it. The obvious `[lo, hi+k, hi+k1]`
        // points inward.
        for band in 0..stations.len().saturating_sub(1) {
            let (lo, hi) = (ring_start[band], ring_start[band + 1]);
            let (lo_point, hi_point) = (ring_is_point[band], ring_is_point[band + 1]);
            let n = divisions as u32;
            for k in 0..n {
                let k1 = (k + 1) % n;
                match (lo_point, hi_point) {
                    (true, true) => {}
                    (true, false) => triangles.push([lo, hi + k1, hi + k]),
                    (false, true) => triangles.push([lo + k, lo + k1, hi]),
                    (false, false) => {
                        triangles.push([lo + k, lo + k1, hi + k1]);
                        triangles.push([lo + k, hi + k1, hi + k]);
                    }
                }
            }
        }

        // The disc that closes the top. Its outward normal is `+Z`, which needs
        // `[centre, k, k1]` — the opposite hand from the bottom, because the two
        // caps face opposite ways.
        let top = self.top();
        if top.x > EPS_LENGTH {
            let last = *ring_start.last().unwrap_or(&0);
            let centre = vertices.len() as u32;
            vertices.push(Vec3::new(0.0, 0.0, top.y));
            let n = divisions as u32;
            for k in 0..n {
                let k1 = (k + 1) % n;
                triangles.push([centre, last + k, last + k1]);
            }
        }

        let bound = 2.0 * tolerance;
        let report = TessellationReport {
            tolerance,
            divisions,
            stations: stations.len(),
            bound,
        };
        let mesh = TriMesh::new(vertices, triangles, MeshMeta::synthetic())?;
        Ok((mesh, report))
    }
}

impl Tool {
    /// Tessellates the tool to within `tolerance` of its true surface.
    ///
    /// # Errors
    /// See [`Profile::tessellate`].
    pub fn tessellate(
        &self,
        tolerance: f64,
    ) -> Result<(TriMesh, TessellationReport), TessellateError> {
        self.profile().tessellate(tolerance)
    }
}
