// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Surface deviation and sampling coverage: the project's real accuracy metric.
//!
//! ADR 0005 is the rule this module exists to serve — **volume is a
//! construction-time diagnostic, deviation is the assertion metric** — and the
//! reasons are cancellation and oscillation, both measured at Unit 5.
//!
//! # What "deviation" means here, and how it differs from the plan's wording
//!
//! The unit plan asked for "the nearest point on the surface **reconstructed**
//! from each bundle". Reconstructing a surface is Unit 9, and the same plan
//! forbids doing it here. So this measures something that needs no
//! reconstruction and is arguably the better question anyway:
//!
//! > For a point on the true surface, how far away is the nearest place the
//! > field actually **sampled** that surface?
//!
//! A bundle's span endpoints are exact ray-surface intersections — they lie on
//! the true surface to machine precision, carrying no error of their own. So the
//! one-sided Hausdorff distance from densely sampled surface points to that
//! endpoint set is a clean measure of **sampling adequacy**, which is exactly
//! what §2's `1/sqrt(3)` theorem bounds.
//!
//! Two consequences worth being explicit about, because they are the cost of the
//! substitution:
//!
//! - This is a **coverage** deviation, not a reconstruction deviation. It says
//!   where the field knows the surface, not how well an extracted mesh would
//!   interpolate between those places. U9 must re-measure against its own
//!   output; this number does not bound that one.
//! - It cannot go below zero even for a perfect reconstruction, because a finite
//!   sample set never contains every surface point. What it does do is fall like
//!   `h`, which is the property under test.
//!
//! # Why best-of-three is the number that matters
//!
//! A surface parallel to one bundle is sampled sparsely by it and densely by
//! another. Reporting per-bundle deviation alone would make a tri-dexel field
//! look as bad as its worst bundle, which is precisely the anisotropy the third
//! bundle removes. The per-bundle columns are published so the anisotropy stays
//! visible; the assertion is on the best of the three.

use std::collections::BTreeMap;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Axis, Vec3};
use crate::mesh::TriMesh;
use crate::spans::Span;

use super::field::DexelField;
use super::tri::{AXES, AxisSet, TriDexelField, WORST_CASE_COSINE, best_cosine};

/// A point on the true surface, with the normal there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePoint {
    /// Where it is.
    pub position: Vec3,
    /// Unit outward normal at that point.
    pub normal: Vec3,
}

/// Deviation and coverage for one field against one surface sampling.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviationReport {
    /// Cell size the field was built at.
    pub spacing: f64,
    /// Surface points tested.
    pub samples: u64,
    /// Per-bundle worst distance, in [`AXES`] order. `None` if not built.
    pub per_axis_max: [Option<f64>; 3],
    /// Per-bundle root-mean-square distance.
    pub per_axis_rms: [Option<f64>; 3],
    /// Worst best-of-three distance. **The assertion metric.**
    pub best_max: f64,
    /// Root-mean-square best-of-three distance.
    pub best_rms: f64,
    /// Worst sampling cosine observed over the surface.
    ///
    /// Must never fall below [`WORST_CASE_COSINE`] for a complete field.
    pub worst_cosine: f64,
    /// The normal that achieved [`Self::worst_cosine`].
    pub worst_normal: [f64; 3],
}

impl DeviationReport {
    /// `best_max / spacing`: the constant in `deviation <= C * h`.
    #[must_use]
    pub fn constant(&self) -> f64 {
        if self.spacing > 0.0 {
            self.best_max / self.spacing
        } else {
            f64::INFINITY
        }
    }

    /// True if the coverage guarantee held.
    #[must_use]
    pub fn coverage_holds(&self) -> bool {
        // A tolerance of one ulp-ish, because the sample normals are computed
        // from float cross products and can land a hair under the exact bound
        // even when the geometry is exactly a body diagonal.
        self.worst_cosine >= WORST_CASE_COSINE - 1e-12
    }
}

impl Hashable for DeviationReport {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("DeviationReport");
        h.f64(self.spacing);
        h.u64(self.samples);
        for value in self.per_axis_max.iter().chain(&self.per_axis_rms) {
            match value {
                Some(v) => {
                    h.bool(true);
                    h.f64(*v);
                }
                None => {
                    h.bool(false);
                }
            }
        }
        h.f64(self.best_max);
        h.f64(self.best_rms);
        h.f64(self.worst_cosine);
        h.f64_slice(&self.worst_normal);
        h.end();
    }
}

/// Samples a mesh's surface to a fixed point budget.
///
/// Deviation is a **supremum over the surface**, so what matters is covering
/// every region, not sampling any one of them finely. A budget keeps the harness
/// usable: tying the sample spacing to the cell size instead makes the point
/// count grow as `1/h^2` exactly as the field does, and a 0.1 mm run on a
/// 20,480-triangle sphere then generates millions of queries and never finishes.
///
/// Returns the points and the sample spacing actually used.
///
/// # Panics
/// Panics if `budget` is zero.
#[must_use]
pub fn sample_mesh_budget(mesh: &TriMesh, budget: usize) -> (Vec<SurfacePoint>, f64) {
    assert!(budget > 0, "a sample budget of zero samples nothing");
    let mut area = 0.0;
    for t in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(t);
        area += (b - a).cross(c - a).length() / 2.0;
    }
    if area <= 0.0 {
        return (Vec::new(), 0.0);
    }
    // One sample per `target^2` of surface, near enough -- but the per-triangle
    // subdivision is clamped at 64, so a mesh of a few large triangles blows
    // straight through the budget. A 256-segment cylinder asked for 20,000 came
    // back with 2.5 million and the harness took twenty-four seconds a row.
    // So the target is grown until the count actually lands under budget.
    let mut target = (area / budget as f64).sqrt();
    for _ in 0..24 {
        let points = sample_mesh(mesh, target);
        if points.len() <= budget || target > 1.0e6 {
            return (points, target);
        }
        // Overshoot is roughly quadratic in the target, so correct by the
        // square root of the overshoot rather than doubling blindly.
        target *= (points.len() as f64 / budget as f64).sqrt().max(1.25);
    }
    let points = sample_mesh(mesh, target);
    (points, target)
}

/// Samples points across a mesh's surface, deterministically.
///
/// Each triangle is subdivided on a barycentric lattice fine enough that the
/// sample spacing is near `target`, then sampled at the sub-triangle centroids.
/// Area-weighted by construction, since a bigger triangle gets more
/// subdivisions.
///
/// Centroids rather than vertices on purpose: vertex samples would sit on edges
/// shared by two triangles, so the same point would be tested twice with two
/// different normals, and the coverage statistic would be weighted towards
/// creases.
#[must_use]
pub fn sample_mesh(mesh: &TriMesh, target: f64) -> Vec<SurfacePoint> {
    let mut out = Vec::new();
    for t in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(t);
        let cross = (b - a).cross(c - a);
        let length = cross.length();
        if length <= 0.0 {
            // A degenerate triangle has no surface to sample and no normal to
            // report. Construction drops these too.
            continue;
        }
        let normal = cross * (1.0 / length);

        let longest = (b - a).length().max((c - b).length()).max((a - c).length());
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to 1..=64 immediately"
        )]
        let n = if target > 0.0 {
            ((longest / target).ceil() as usize).clamp(1, 64)
        } else {
            1
        };

        // Barycentric sub-triangle centroids. The upward and downward triangles
        // of a subdivided triangle are enumerated separately.
        let step = 1.0 / n as f64;
        for i in 0..n {
            for j in 0..(n - i) {
                let (fi, fj) = (i as f64, j as f64);
                let up = ((fi + 1.0 / 3.0) * step, (fj + 1.0 / 3.0) * step);
                out.push(SurfacePoint {
                    position: barycentric(a, b, c, up.0, up.1),
                    normal,
                });
                if j + 1 < n - i {
                    let down = ((fi + 2.0 / 3.0) * step, (fj + 2.0 / 3.0) * step);
                    out.push(SurfacePoint {
                        position: barycentric(a, b, c, down.0, down.1),
                        normal,
                    });
                }
            }
        }
    }
    out
}

fn barycentric(a: Vec3, b: Vec3, c: Vec3, u: f64, v: f64) -> Vec3 {
    a + (b - a) * u + (c - a) * v
}

/// Distance from `p` to the nearest span endpoint in `field`.
///
/// Searches outward from the ray whose cell contains `p`, in Chebyshev rings,
/// and stops as soon as the next ring cannot contain anything closer. Exact, and
/// in practice it examines a handful of cells rather than the whole field — a
/// brute-force scan over millions of endpoints per query would make the harness
/// unusable at the resolutions that matter.
#[must_use]
pub fn nearest_endpoint(field: &DexelField, p: Vec3) -> f64 {
    let lattice = field.lattice();
    let [u, v, w] = lattice.axis().cyclic();
    let spacing = lattice.spacing_max();
    let [nu, nv] = lattice.counts();
    if nu == 0 || nv == 0 || spacing <= 0.0 {
        return f64::INFINITY;
    }

    let point = p.to_array();
    let origin = lattice.origin().to_array();
    // The ray axis origin: rays start one cell behind the workspace, and span
    // parameters are measured from there.
    let base_w = origin[w] - spacing;

    #[allow(
        clippy::cast_possible_truncation,
        reason = "clamped into the lattice immediately below"
    )]
    let centre_i = ((point[u] - origin[u]) / spacing - 0.5).round() as i64;
    #[allow(
        clippy::cast_possible_truncation,
        reason = "clamped into the lattice immediately below"
    )]
    let centre_j = ((point[v] - origin[v]) / spacing - 0.5).round() as i64;

    let mut best = f64::INFINITY;
    let max_radius = i64::from(nu.max(nv));
    for radius in 0..=max_radius {
        // Nothing in this ring or beyond can beat `best`: the closest a cell at
        // Chebyshev radius r can be, transversely, is (r - 1) * spacing.
        let floor = (radius as f64 - 1.0).max(0.0) * spacing;
        if floor > best {
            break;
        }
        let mut examined = false;
        for i in (centre_i - radius)..=(centre_i + radius) {
            for j in (centre_j - radius)..=(centre_j + radius) {
                // Ring only: the interior was covered by a smaller radius.
                if radius > 0 && (i - centre_i).abs() != radius && (j - centre_j).abs() != radius {
                    continue;
                }
                if i < 0 || j < 0 || i >= i64::from(nu) || j >= i64::from(nv) {
                    continue;
                }
                examined = true;
                #[allow(
                    clippy::cast_sign_loss,
                    clippy::cast_possible_truncation,
                    reason = "bounds-checked against the lattice counts above"
                )]
                let ray = lattice.index(i as u32, j as u32);
                let ray_origin = lattice
                    .origin_of(u32::try_from(i).unwrap_or(0), u32::try_from(j).unwrap_or(0))
                    .to_array();
                let du = point[u] - ray_origin[u];
                let dv = point[v] - ray_origin[v];
                let transverse = du * du + dv * dv;
                if transverse > best * best {
                    continue;
                }
                for span in field.arena().get(ray) {
                    for t in [span.t0, span.t1] {
                        let dw = point[w] - (base_w + t);
                        let d = (transverse + dw * dw).sqrt();
                        if d < best {
                            best = d;
                        }
                    }
                }
            }
        }
        if !examined && radius > max_radius / 2 && best.is_finite() {
            break;
        }
    }
    best
}

/// Measures deviation and coverage of a tri-dexel field against a surface.
///
/// `samples` should come from [`sample_mesh`] on the **source mesh** to isolate
/// sampling error, or from an analytic sampler to include tessellation error.
/// The caller chooses, and the two are reported as separate columns.
#[must_use]
pub fn measure(field: &TriDexelField, samples: &[SurfacePoint]) -> DeviationReport {
    let axes = field.axes();
    let spacing = field
        .bundles()
        .next()
        .map_or(0.0, |(_, b)| b.lattice().spacing_max());

    let mut per_axis_max: [Option<f64>; 3] = [None, None, None];
    let mut per_axis_sq: [f64; 3] = [0.0; 3];
    let mut best_max = 0.0f64;
    let mut best_sq = 0.0f64;
    let mut worst_cosine = f64::INFINITY;
    let mut worst_normal = [0.0, 0.0, 1.0];

    for point in samples {
        let mut best = f64::INFINITY;
        for axis in AXES {
            let Some(bundle) = field.bundle(axis) else {
                continue;
            };
            let d = nearest_endpoint(bundle, point.position);
            let slot = axis.index();
            per_axis_max[slot] = Some(per_axis_max[slot].unwrap_or(0.0).max(d));
            per_axis_sq[slot] += d * d;
            if d < best {
                best = d;
            }
        }
        if best.is_finite() {
            best_max = best_max.max(best);
            best_sq += best * best;
        }

        let cosine = best_cosine(point.normal, axes);
        if cosine < worst_cosine {
            worst_cosine = cosine;
            worst_normal = point.normal.to_array();
        }
    }

    let n = samples.len().max(1) as f64;
    let rms = |sum: f64| (sum / n).sqrt();
    DeviationReport {
        spacing,
        samples: samples.len() as u64,
        per_axis_max,
        per_axis_rms: [
            per_axis_max[0].map(|_| rms(per_axis_sq[0])),
            per_axis_max[1].map(|_| rms(per_axis_sq[1])),
            per_axis_max[2].map(|_| rms(per_axis_sq[2])),
        ],
        best_max,
        best_rms: rms(best_sq),
        worst_cosine: if worst_cosine.is_finite() {
            worst_cosine
        } else {
            0.0
        },
        worst_normal,
    }
}

/// Worst sampling cosine over a mesh's surface, and where it occurred.
///
/// The direct check on §2: over any closed surface this must never fall below
/// [`WORST_CASE_COSINE`], and a mesh containing a face normal to `(1,1,1)`
/// should come close to attaining it.
#[must_use]
pub fn coverage(mesh: &TriMesh, axes: AxisSet) -> (f64, [f64; 3]) {
    let mut worst = f64::INFINITY;
    let mut normal = [0.0, 0.0, 1.0];
    for t in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(t);
        let cross = (b - a).cross(c - a);
        let length = cross.length();
        if length <= 0.0 {
            continue;
        }
        let n = cross * (1.0 / length);
        let cosine = best_cosine(n, axes);
        if cosine < worst {
            worst = cosine;
            normal = n.to_array();
        }
    }
    (if worst.is_finite() { worst } else { 0.0 }, normal)
}

/// Span-count histogram across every bundle, for the report.
#[must_use]
pub fn combined_distribution(field: &TriDexelField) -> BTreeMap<usize, usize> {
    let mut out: BTreeMap<usize, usize> = BTreeMap::new();
    for (_, bundle) in field.bundles() {
        for (spans, rays) in bundle.arena().distribution() {
            *out.entry(spans).or_default() += rays;
        }
    }
    out
}

/// Sample points on an analytic solid, for the second error column.
///
/// Deliberately small: a sphere, a cylinder and a torus are enough to show what
/// tessellation costs, and a general analytic sampler would be a geometry
/// kernel this unit does not need.
pub enum Analytic {
    /// Sphere of the given radius, centred at the origin.
    Sphere {
        /// Radius, in millimetres.
        radius: f64,
    },
    /// Cylinder of the given radius and height, axis along `axis`.
    ///
    /// Spans `0 ..= height` along its axis, **matching
    /// [`crate::mesh::shapes::cylinder`]**. Not centred: assuming it was cost an
    /// afternoon, because a 12 mm offset shows up as a suspiciously constant
    /// deviation rather than an obvious failure.
    Cylinder {
        /// Radius, in millimetres.
        radius: f64,
        /// Height, in millimetres.
        height: f64,
        /// Which way the axis runs.
        axis: Axis,
    },
    /// Torus in the XY plane, centred at the origin.
    Torus {
        /// Distance from the centre to the tube centre.
        major: f64,
        /// Tube radius.
        minor: f64,
    },
}

impl Analytic {
    /// Samples the exact surface on a regular parametric grid.
    ///
    /// # Panics
    /// Panics if `steps` is zero.
    #[must_use]
    pub fn sample(&self, steps: u32) -> Vec<SurfacePoint> {
        use crate::transcendental::sin_cos;
        assert!(steps > 0, "an analytic sampler needs at least one step");
        let mut out = Vec::new();
        let tau = 2.0 * core::f64::consts::PI;
        match self {
            Self::Sphere { radius } => {
                for i in 0..=steps {
                    let theta = core::f64::consts::PI * f64::from(i) / f64::from(steps);
                    let (st, ct) = sin_cos(theta);
                    for j in 0..steps {
                        let phi = tau * f64::from(j) / f64::from(steps);
                        let (sp, cp) = sin_cos(phi);
                        let n = Vec3::new(st * cp, st * sp, ct);
                        out.push(SurfacePoint {
                            position: n * *radius,
                            normal: n,
                        });
                    }
                }
            }
            Self::Cylinder {
                radius,
                height,
                axis,
            } => {
                let [u, v, w] = axis.cyclic();
                for j in 0..steps {
                    let phi = tau * f64::from(j) / f64::from(steps);
                    let (sp, cp) = sin_cos(phi);
                    for i in 0..=steps {
                        let t = f64::from(i) / f64::from(steps);
                        let mut p = [0.0; 3];
                        let mut n = [0.0; 3];
                        p[u] = radius * cp;
                        p[v] = radius * sp;
                        p[w] = t * height;
                        n[u] = cp;
                        n[v] = sp;
                        out.push(SurfacePoint {
                            position: Vec3::from_array(p),
                            normal: Vec3::from_array(n),
                        });
                    }
                    // The end caps, whose normals are along the axis: without
                    // them the sampling would omit exactly the orientation the
                    // bundle captures best, and flatter the result.
                    for cap in [-1.0, 1.0] {
                        for k in 1..steps {
                            let r = radius * f64::from(k) / f64::from(steps);
                            let mut p = [0.0; 3];
                            let mut n = [0.0; 3];
                            p[u] = r * cp;
                            p[v] = r * sp;
                            p[w] = if cap < 0.0 { 0.0 } else { *height };
                            n[w] = cap;
                            out.push(SurfacePoint {
                                position: Vec3::from_array(p),
                                normal: Vec3::from_array(n),
                            });
                        }
                    }
                }
            }
            Self::Torus { major, minor } => {
                for i in 0..steps {
                    let u = tau * f64::from(i) / f64::from(steps);
                    let (su, cu) = sin_cos(u);
                    for j in 0..steps {
                        let v = tau * f64::from(j) / f64::from(steps);
                        let (sv, cv) = sin_cos(v);
                        let n = Vec3::new(cu * cv, su * cv, sv);
                        out.push(SurfacePoint {
                            position: Vec3::new(
                                (major + minor * cv) * cu,
                                (major + minor * cv) * su,
                                minor * sv,
                            ),
                            normal: n,
                        });
                    }
                }
            }
        }
        out
    }
}

/// A span endpoint's position in space, for a bundle.
#[must_use]
pub fn endpoint_position(field: &DexelField, ray: u32, span: Span, upper: bool) -> Vec3 {
    let lattice = field.lattice();
    let [_, _, w] = lattice.axis().cyclic();
    let (i, j) = lattice.coords(ray);
    let mut p = lattice.origin_of(i, j).to_array();
    p[w] += if upper { span.t1 } else { span.t0 };
    Vec3::from_array(p)
}
