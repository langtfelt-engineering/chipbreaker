// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! How fast does the field's volume converge as cells get smaller?
//!
//! # Two error measures, and only one of them is an assertion
//!
//! A dexel volume differs from the truth for two independent reasons, and
//! reporting them together would hide which one is which.
//!
//! **Against the mesh** (`TriMesh::signed_volume`). This isolates *dexel
//! sampling error*: the mesh is exactly what the rays were cast at, so any
//! difference is the transverse sum and nothing else. This is the basis for
//! every convergence assertion, because it is the only one that measures the
//! thing being tested.
//!
//! **Against the analytic solid.** This includes tessellation error too — an
//! icosphere is a polyhedron inscribed in a sphere and is genuinely smaller. It
//! does **not** converge as the cells shrink, because refining the lattice does
//! nothing about the tessellation; in the sphere's table it flattens out at
//! 5.4e-4 and stays there. It is still the number a customer cares about,
//! because it is the whole distance between the answer and reality.
//!
//! Both columns are published so the entire budget is visible. Only the first is
//! asserted on. Asserting on the second would be asserting on the mesh
//! generator.
//!
//! # There are two error models, not one, and the difference is the whole unit
//!
//! Which model applies depends on how a ray's **chord length** behaves as the
//! ray approaches the silhouette.
//!
//! ## Model 1: quadrature, when the chord vanishes continuously
//!
//! For a sphere, a cone, a torus, or a cylinder lying across the bundle, a ray
//! that just misses the solid contributes nearly nothing, because the chord
//! shrinks to zero as the silhouette is approached. Summing chord lengths on a
//! lattice is then a midpoint quadrature of a continuous function, the boundary
//! cells are only slightly wrong, and the error decreases **monotonically** at a
//! rate set by the smoothness of the chord profile:
//!
//! - a square-root profile (a lying cylinder: chord `2*sqrt(R^2 - x^2)`) gives
//!   about `h^1.5`, and measures 1.46;
//! - the smoother two-dimensional cases measure 2.1 to 2.5.
//!
//! These are honest fitted exponents over a monotone sequence, and a finer
//! lattice always helps.
//!
//! ## Model 2: lattice-point counting, when the chord is a hard indicator
//!
//! For a cylinder whose axis runs **along** the bundle, the chord is the full
//! height everywhere inside the silhouette and zero outside, with a jump at the
//! edge. The volume is then *exactly*
//!
//! ```text
//! V = h^2 * H * #{rays whose centre lies inside the disc}
//! ```
//!
//! and the volume error is *exactly* the error in counting lattice points inside
//! a disc. That is the **Gauss circle problem**, and this is not an analogy: the
//! measured relative errors reproduce a direct lattice-point count to five
//! significant figures at every spacing tested.
//!
//! # A correction, recorded because getting this wrong once was expensive
//!
//! The unit specification predicted `(h/R)^1.37` from lattice-point counting.
//! An earlier review of this work — mine — reported that as the wrong model,
//! on the grounds that a dexel field sums exact chord lengths rather than
//! counting points.
//!
//! **That correction was itself wrong, and in a way worth spelling out.** The
//! specification's model is exactly right for the case above, and the exponent
//! is not approximately right, it is the number:
//!
//! ```text
//! N(r) = pi*r^2 + E(r),  |E(r)| = O(r^theta),  theta = 131/208 = 0.62981
//! with r = R/h:  relative area error = (h/R)^(2 - theta)
//! 2 - 131/208 = 1.37019
//! ```
//!
//! The specification's error was **scope, not model**: it stated a worst-case
//! bound for the hard-indicator case as if it were a universal convergence rate.
//! The right statement is that there are two regimes, this bound governs one of
//! them, and quadrature governs the rest.
//!
//! And `1.37` is an **upper bound on the error**, not an asymptotic decay rate.
//! The Gauss circle error is famously erratic. Fitting a power law to six
//! samples of it produces a number, but the number is noise — which is exactly
//! what an earlier pass at this measurement did, reporting a "cylinder exponent"
//! of 1.91 that a rerun on a different ratio grid turned into 1.57. Neither
//! meant anything. So for this regime the code asserts an **envelope**,
//! `error <= C * (h/R)^1.37`, which is the claim the theory actually supports,
//! and [`Convergence::exponent`] is deliberately not used.
//!
//! # Refining the lattice can make the answer worse
//!
//! Measured, on the axis-parallel cylinder against its own mesh:
//!
//! | h/R | error |
//! |---:|---:|
//! | 1/80 | 1.90e-4 |
//! | 1/160 | **4.39e-4** |
//!
//! Four times the rays, more than twice the error. That is not a defect in the
//! implementation; it is what lattice-point counting does, and no amount of
//! refinement removes it.
//!
//! This is the strongest argument in the project for Unit 6. A single-axis field
//! does not merely capture a vertical wall *badly* — it captures it
//! **unpredictably**, and a verification tool cannot tell a customer that a
//! finer simulation is a safer one. The fix is not a finer lattice. It is a
//! bundle along another axis, where that same wall is a horizontal surface the
//! rays meet analytically.

use crate::math::{Axis, Mat4};
use crate::mesh::{TriMesh, shapes};
use crate::transcendental::{ln, powf};

use super::field::{BuildOptions, DexelField};

/// The Gauss circle exponent, `2 - 131/208`.
///
/// The specification's `1.37`, and it is exactly this. See the module header.
pub const GAUSS_CIRCLE_EXPONENT: f64 = 2.0 - 131.0 / 208.0;

/// Which error model governs a solid, given the bundle direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorModel {
    /// The chord vanishes continuously at the silhouette.
    ///
    /// Midpoint quadrature of a continuous profile: monotone, and a fitted
    /// exponent means something.
    Quadrature,
    /// The chord is a hard indicator — full inside, zero outside.
    ///
    /// The Gauss circle problem. Erratic, non-monotone, and only an envelope is
    /// a defensible claim. This is the vertical-wall case, which is to say it is
    /// the anisotropy Unit 6 exists to remove.
    LatticeCount,
}

/// One solid, at one spacing.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Cell size, in millimetres.
    pub spacing: f64,
    /// Characteristic radius of the solid.
    pub radius: f64,
    /// `spacing / radius`. Accuracy depends on this ratio, not on spacing alone.
    pub ratio: f64,
    /// What the field measured.
    pub measured: f64,
    /// What the mesh actually is.
    pub mesh_volume: f64,
    /// What the ideal solid is, where there is a closed form.
    pub analytic_volume: Option<f64>,
    /// Rays cast.
    pub rays: u64,
}

impl Sample {
    /// Relative error against the mesh: **dexel sampling error alone**.
    ///
    /// Every assertion uses this one, because the mesh is exactly what the rays
    /// met.
    #[must_use]
    pub fn mesh_error(&self) -> f64 {
        (self.measured - self.mesh_volume).abs() / self.mesh_volume
    }

    /// Signed relative error against the mesh.
    ///
    /// Published because the sign flips, and a sign flip is why the absolute
    /// error passes through near-zero at some spacings. Anyone reading only the
    /// magnitudes would take those dips for superconvergence.
    #[must_use]
    pub fn signed_mesh_error(&self) -> f64 {
        (self.measured - self.mesh_volume) / self.mesh_volume
    }

    /// Relative error against the ideal solid: sampling **plus** tessellation.
    ///
    /// Reported, never asserted on. It stops converging once tessellation
    /// dominates.
    #[must_use]
    pub fn analytic_error(&self) -> Option<f64> {
        self.analytic_volume.map(|v| (self.measured - v).abs() / v)
    }
}

/// One solid, across a range of spacings.
#[derive(Debug, Clone, PartialEq)]
pub struct Convergence {
    /// What was measured.
    pub name: String,
    /// Which model governs it.
    pub model: ErrorModel,
    /// Coarse to fine.
    pub samples: Vec<Sample>,
}

impl Convergence {
    /// Fitted exponent `p` in `error ~ (h/R)^p`, by least squares on the logs.
    ///
    /// **Meaningful only for [`ErrorModel::Quadrature`].** For a lattice-count
    /// solid this fits noise; use [`Self::envelope_constant`] instead. `None` if
    /// fewer than two usable samples survive.
    #[must_use]
    pub fn exponent(&self) -> Option<f64> {
        let points: Vec<(f64, f64)> = self
            .samples
            .iter()
            .filter(|s| s.mesh_error() > 0.0 && s.ratio > 0.0)
            .map(|s| (ln(s.ratio), ln(s.mesh_error())))
            .collect();
        if points.len() < 2 {
            return None;
        }
        let n = points.len() as f64;
        let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
        let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for (x, y) in &points {
            numerator += (x - mean_x) * (y - mean_y);
            denominator += (x - mean_x) * (x - mean_x);
        }
        if denominator == 0.0 {
            return None;
        }
        Some(numerator / denominator)
    }

    /// Observed order between each consecutive pair of refinements.
    ///
    /// `log(e_k / e_k+1) / log(r_k / r_k+1)`. Published alongside the fit because
    /// a single fitted number cannot show that a sequence is erratic, and
    /// erratic is the finding for the lattice-count case.
    #[must_use]
    pub fn observed_orders(&self) -> Vec<f64> {
        self.samples
            .windows(2)
            .map(|pair| {
                let (a, b) = (&pair[0], &pair[1]);
                if a.mesh_error() <= 0.0 || b.mesh_error() <= 0.0 || a.ratio == b.ratio {
                    return f64::NAN;
                }
                ln(a.mesh_error() / b.mesh_error()) / ln(a.ratio / b.ratio)
            })
            .collect()
    }

    /// True if every refinement reduced the error.
    ///
    /// The property a customer would assume without being told, and the one the
    /// axis-parallel cylinder does not have.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        self.samples
            .windows(2)
            .all(|pair| pair[1].mesh_error() <= pair[0].mesh_error())
    }

    /// The largest `error / (h/R)^exponent` over the samples.
    ///
    /// The constant in an envelope claim `error <= C * (h/R)^p`. This is what a
    /// lattice-count solid supports, where a fitted rate is noise.
    #[must_use]
    pub fn envelope_constant(&self, exponent: f64) -> f64 {
        self.samples
            .iter()
            .filter(|s| s.ratio > 0.0)
            .map(|s| s.mesh_error() / powf(s.ratio, exponent))
            .fold(0.0, f64::max)
    }

    /// The finest sample whose ratio is at most `limit`, if any.
    ///
    /// Used for the absolute accuracy bound, which only applies where cells are
    /// fine relative to the feature — `h <= R/200`. Stating the ratio alongside
    /// the bound is the point: an accuracy claim without the ratio it was
    /// measured at is not a claim about anything.
    #[must_use]
    pub fn finest_within(&self, limit: f64) -> Option<&Sample> {
        self.samples
            .iter()
            .filter(|s| s.ratio <= limit)
            .min_by(|a, b| a.ratio.total_cmp(&b.ratio))
    }
}

/// A solid to measure, and what it should be.
pub struct Case {
    /// Name for the table.
    pub name: &'static str,
    /// Characteristic radius: the length that `spacing` is compared against.
    pub radius: f64,
    /// Builds the mesh.
    pub mesh: fn() -> TriMesh,
    /// The ideal solid's volume, where there is a closed form.
    pub analytic: Option<f64>,
    /// How the stock is placed. Used to lay the cylinder on its side.
    pub placement: Mat4,
    /// Which error model governs it, and therefore which claim is defensible.
    pub model: ErrorModel,
}

/// Ninety degrees about X: an upright axis moves from `Z` to `Y`.
const LIE_DOWN: Mat4 = Mat4 {
    m: [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
};

/// The standard set of cases.
///
/// Deliberately includes the **same cylinder twice**, upright and lying down.
/// That pair is the clearest demonstration in the project that a single-axis
/// field is anisotropic, and therefore of why Unit 6 exists — not because the
/// upright one is less accurate, but because its error obeys a different and
/// unpredictable law. See the module header.
#[must_use]
pub fn standard_cases() -> Vec<Case> {
    let pi = core::f64::consts::PI;
    vec![
        Case {
            name: "cylinder r=10 h=20, axis ALONG the bundle",
            radius: 10.0,
            mesh: || shapes::cylinder(10.0, 20.0, 256),
            analytic: Some(pi * 100.0 * 20.0),
            placement: Mat4::IDENTITY,
            // The vertical-wall case: the chord is a hard indicator, so the
            // volume is a lattice-point count of the disc.
            model: ErrorModel::LatticeCount,
        },
        Case {
            name: "cylinder r=10 h=20, axis ACROSS the bundle",
            radius: 10.0,
            mesh: || shapes::cylinder(10.0, 20.0, 256),
            analytic: Some(pi * 100.0 * 20.0),
            placement: LIE_DOWN,
            // The same solid, now a midpoint quadrature of 2*sqrt(R^2 - x^2).
            model: ErrorModel::Quadrature,
        },
        Case {
            name: "sphere r=10",
            radius: 10.0,
            mesh: || shapes::icosphere(10.0, 5),
            analytic: Some(4.0 / 3.0 * pi * 1000.0),
            placement: Mat4::IDENTITY,
            model: ErrorModel::Quadrature,
        },
        Case {
            name: "cone r=10 h=20",
            radius: 10.0,
            mesh: || shapes::cone(10.0, 20.0, 256),
            analytic: Some(pi * 100.0 * 20.0 / 3.0),
            placement: Mat4::IDENTITY,
            model: ErrorModel::Quadrature,
        },
        Case {
            name: "torus R=10 r=3",
            radius: 3.0,
            mesh: || shapes::torus(10.0, 3.0, 256, 128),
            analytic: Some(2.0 * pi * pi * 10.0 * 9.0),
            placement: Mat4::IDENTITY,
            model: ErrorModel::Quadrature,
        },
        Case {
            name: "torus R=20 r=2",
            radius: 2.0,
            mesh: || shapes::torus(20.0, 2.0, 256, 128),
            analytic: Some(2.0 * pi * pi * 20.0 * 4.0),
            placement: Mat4::IDENTITY,
            model: ErrorModel::Quadrature,
        },
    ]
}

/// Measures one case at each of `ratios`, where a ratio is `spacing / radius`.
///
/// # Panics
/// Panics if a field fails to build, which for these meshes would be a defect
/// rather than a possibility to handle.
#[must_use]
pub fn measure(case: &Case, ratios: &[f64]) -> Convergence {
    let mesh = (case.mesh)();
    let mesh_volume = transformed_volume(&mesh, &case.placement);
    let mut samples = Vec::with_capacity(ratios.len());
    for &ratio in ratios {
        let spacing = ratio * case.radius;
        let options = BuildOptions {
            spacing_xyz: None,
            spacing,
            axis: Axis::Z,
            placement: case.placement,
            margin: 0.0,
        };
        let (field, stats) =
            DexelField::build(&mesh, &options).unwrap_or_else(|e| panic!("{}: {e}", case.name));
        samples.push(Sample {
            spacing,
            radius: case.radius,
            ratio,
            measured: field.volume(),
            mesh_volume,
            analytic_volume: case.analytic,
            rays: stats.rays,
        });
    }
    Convergence {
        name: case.name.to_owned(),
        model: case.model,
        samples,
    }
}

/// Ratios `h/R` for the standard table: 1/10 down to 1/320.
///
/// Geometric, because the fit is a straight line in log-log and evenly spaced
/// logs weight every octave equally.
#[must_use]
pub fn standard_ratios() -> Vec<f64> {
    (0..6)
        .map(|k| 1.0 / (10.0 * powf(2.0, f64::from(k))))
        .collect()
}

/// A cheaper grid for tests, which run in debug as well as release.
#[must_use]
pub fn test_ratios() -> Vec<f64> {
    (0..5)
        .map(|k| 1.0 / (10.0 * powf(2.0, f64::from(k))))
        .collect()
}

/// The mesh's own volume under a placement.
///
/// A rigid or mirroring placement scales volume by `|det|`, so the reference has
/// to move with the stock. Without this, laying the cylinder down would look
/// like a volume error rather than a rotation.
fn transformed_volume(mesh: &TriMesh, placement: &Mat4) -> f64 {
    mesh.signed_volume().abs() * placement.determinant().abs()
}
