// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Where the simulated result differs from the part that was intended.
//!
//! # The sign convention, in one place
//!
//! **Positive is excess stock. Negative is a gouge.**
//!
//! Material that should have been removed and was not reads positive; material
//! that should have stayed and was cut away reads negative. The convention lives
//! here and nowhere else, and it is tested from both ends, because a sign error
//! at this line inverts every finding the product will ever emit — turning
//! "there is metal left on the face" into "you have cut into it", which is the
//! difference between a part that needs another pass and a part that is scrap.
//!
//! # Three distances, and which one is the metric
//!
//! Unit 6 established that **sample distance** — how far a query point is from
//! the nearest place the field sampled — is a property of the lattice and not of
//! the part, that it exceeds the real error by up to `3/sqrt(2)`, and that
//! pointwise the ratio `1/(sin θ · cos θ)` is unbounded. That argument stands and
//! nothing here reports sample distance.
//!
//! But it leaves two candidates, not one, and the first version of this module
//! conflated them:
//!
//! - **Surface distance.** How far the result's surface is from the nominal
//!   surface, to the nearest point of it. This is what `d_H` is *defined* as, so
//!   it is the metric. Reported as `signed_mm`.
//! - **Perpendicular distance.** The same thing measured along the stored
//!   normal, by casting a ray. Reported as `perpendicular_mm`.
//!
//! Where the two surfaces are locally parallel the two agree exactly, which is
//! most of any part. Where they are not, the perpendicular one is an **upper
//! bound and nothing more**: the cast point lies on the nominal, so the nearest
//! point can only be closer.
//!
//! ## What the difference looks like
//!
//! At a step edge the perpendicular ray misses the wall beside it and travels on
//! until it strikes something else. A slot's floor 0.06 mm out laterally reads as
//! **5 mm** out perpendicularly, because the ray leaves along the floor's normal,
//! passes the wall, and hits the top face of the part. That is not a 5 mm defect;
//! it is a 0.06 mm one measured with the wrong ruler, and it appears at every
//! step edge in every program.
//!
//! So the metric is the surface distance, and the perpendicular one is published
//! beside it rather than discarded — the same discipline Unit 8 settled on for
//! its two error measures. Their disagreement is itself diagnostic:
//! [`DeviationField::worst_projection_gap_mm`] is large exactly where the result
//! meets the nominal at a steep angle, which is where a customer should be told
//! that a perpendicular reading is not meaningful.
//!
//! Both use the endpoint normals stored at Unit 9 — the perpendicular one for its
//! direction, and **both** for their sign. That is the second of the three units
//! the four-byte decision was justified by.
//!
//! # What a deviation bound covers
//!
//! `d_H(computed stock, ideal geometric cutting model)`. That is the whole of
//! it.
//!
//! It says nothing about tool wear, deflection under load, thermal growth,
//! spindle runout, backlash, or how a controller interpolates between the points
//! it was given. A part can match this field exactly and still be out of
//! tolerance for any of those reasons. Nothing in this module, its output, or
//! anything built on it may imply otherwise.
//!
//! # The tessellation floor applies twice
//!
//! ADR 0005 established that any accuracy metric floors against the fidelity of
//! its input. Here there are **two** inputs — the stock mesh the field was built
//! from, and the nominal part being compared against — so the floor is the
//! coarser of the two. A customer supplying a 1 mm-faceted nominal and asking
//! for findings at 0.01 mm is making that mistake in a different costume, and
//! the report says so rather than quietly obliging.

use crate::dexel::tri::{AXES, TriDexelField};
use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Ray, Vec3};
use crate::mesh::TriMesh;
use crate::mesh::bvh::Bvh;

/// One sampled disagreement between the result and the nominal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deviation {
    /// Where on the result's surface this was measured.
    pub at: Vec3,
    /// Outward normal of the result there, from the stored endpoint normal.
    pub normal: Vec3,
    /// **The metric: signed distance to the nearest point of the nominal
    /// surface, in millimetres.** Positive is excess stock, negative is a gouge.
    /// See the module header.
    pub signed_mm: f64,
    /// The same deviation measured along `normal` instead, by casting a ray.
    ///
    /// A diagnostic, never the finding. Equal to `signed_mm` wherever the two
    /// surfaces are locally parallel, and an upper bound on it everywhere.
    pub perpendicular_mm: f64,
    /// Which bundle's ray this endpoint came from.
    pub axis: usize,
}

impl Deviation {
    /// How much the perpendicular reading overstates the metric here.
    ///
    /// Zero on parallel surfaces; large at a step edge, where the cast leaves
    /// along the normal and misses the wall beside it.
    #[must_use]
    pub fn projection_gap_mm(&self) -> f64 {
        (self.perpendicular_mm.abs() - self.signed_mm.abs()).max(0.0)
    }
}

impl Hashable for Deviation {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Deviation");
        h.f64_slice(&self.at.to_array());
        h.f64_slice(&self.normal.to_array());
        h.f64(self.signed_mm);
        h.f64(self.perpendicular_mm);
        h.u64(self.axis as u64);
        h.end();
    }
}

/// The whole comparison.
#[derive(Debug, Clone, Default)]
pub struct DeviationField {
    /// Every sampled deviation, in bundle then ray then span order.
    pub samples: Vec<Deviation>,
    /// Deepest gouge, as a positive depth. Zero if there are none.
    pub worst_gouge_mm: f64,
    /// Thickest excess stock. Zero if there is none.
    pub worst_excess_mm: f64,
    /// Root-mean-square of the signed values, reduced in sample order.
    pub rms_mm: f64,
    /// The largest amount by which the perpendicular reading overstated the
    /// metric at any sample.
    ///
    /// Published rather than asserted on. A large value does not mean the
    /// comparison is wrong — it means the two surfaces meet at a steep angle
    /// somewhere, so a perpendicular reading there describes the geometry of the
    /// measurement rather than the geometry of the part.
    pub worst_projection_gap_mm: f64,
    /// Estimated facet size of the stock mesh, in millimetres.
    pub stock_facet_mm: f64,
    /// Estimated facet size of the nominal mesh.
    pub nominal_facet_mm: f64,
    /// The lattice this was sampled on.
    pub spacing_mm: f64,
}

impl DeviationField {
    /// The floor under any tolerance claimable from this comparison.
    ///
    /// The coarser of the two input meshes' facets and the lattice. Below this,
    /// a reported number describes the inputs rather than the engine — see ADR
    /// 0005, and the module header for why there are two meshes here rather than
    /// one.
    #[must_use]
    pub fn tolerance_floor_mm(&self) -> f64 {
        self.stock_facet_mm
            .max(self.nominal_facet_mm)
            .max(self.spacing_mm)
    }

    /// Whether a requested tolerance is below what the inputs can support.
    #[must_use]
    pub fn below_floor(&self, tolerance_mm: f64) -> bool {
        tolerance_mm < self.tolerance_floor_mm()
    }

    /// How many samples exceed `tolerance_mm` in magnitude.
    #[must_use]
    pub fn findings(&self, tolerance_mm: f64) -> usize {
        self.samples
            .iter()
            .filter(|d| d.signed_mm.abs() > tolerance_mm)
            .count()
    }
}

impl Hashable for DeviationField {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("DeviationField");
        h.add_all(self.samples.iter());
        h.f64(self.worst_gouge_mm);
        h.f64(self.worst_excess_mm);
        h.f64(self.rms_mm);
        h.f64(self.worst_projection_gap_mm);
        // The facet estimates and the spacing describe the inputs, and a report
        // that omitted them from the digest could claim a tolerance its inputs
        // never supported without the digest noticing.
        h.f64(self.stock_facet_mm);
        h.f64(self.nominal_facet_mm);
        h.f64(self.spacing_mm);
        h.end();
    }
}

/// Estimates a mesh's characteristic facet size.
///
/// The square root of the mean triangle area, which is the side of the
/// equal-area equilateral triangle — a single number for "how finely is this
/// tessellated". Crude on purpose: it is used to warn, not to bound.
#[must_use]
pub fn facet_size(mesh: &TriMesh) -> f64 {
    let n = mesh.triangle_count();
    if n == 0 {
        return 0.0;
    }
    let mut total = 0.0f64;
    for t in 0..n {
        total += mesh.double_area(t) / 2.0;
    }
    #[allow(clippy::cast_precision_loss, reason = "a triangle count")]
    let mean = total / f64::from(n);
    if mean > 0.0 { mean.sqrt() } else { 0.0 }
}

/// Compares a cut field against the nominal part.
///
/// Walks every span endpoint of every ray, projects the distance to the nominal
/// surface onto that endpoint's stored normal, and signs it by which side of the
/// nominal the material sits on.
///
/// `placement` maps the nominal into machine coordinates; the caller resolves it
/// from the toolpath's work offset or an explicit transform, exactly as Unit 5's
/// stock placement does, rather than a second mechanism being invented here.
#[must_use]
pub fn compare(field: &TriDexelField, nominal: &TriMesh, stock: Option<&TriMesh>) -> DeviationField {
    let bvh = Bvh::build(nominal);
    let mut samples: Vec<Deviation> = Vec::new();

    // Bundle, then ray, then span: the same order everything else in the engine
    // walks a field, so the reduction below is in a fixed order by construction.
    for axis in AXES {
        let Some(bundle) = field.bundle(axis) else {
            continue;
        };
        let lattice = bundle.lattice().clone();
        let rays = u32::try_from(bundle.arena().rays()).unwrap_or(u32::MAX);
        let direction = axis.direction();
        for ray_index in 0..rays {
            let (i, j) = lattice.coords(ray_index);
            let origin = lattice.origin_of(i, j);
            for span in bundle.arena().get(ray_index) {
                for (t, code) in [(span.t0, span.n0), (span.t1, span.n1)] {
                    let at = Vec3::new(
                        origin.x + direction.x * t,
                        origin.y + direction.y * t,
                        origin.z + direction.z * t,
                    );
                    let normal = code.decode();
                    if let Some((surface, perpendicular)) =
                        signed_deviation(nominal, &bvh, at, normal)
                    {
                        samples.push(Deviation {
                            at,
                            normal,
                            signed_mm: surface,
                            perpendicular_mm: perpendicular,
                            axis: axis.index(),
                        });
                    }
                }
            }
        }
    }

    reduce(
        samples,
        stock.map_or(0.0, facet_size),
        facet_size(nominal),
        field
            .bundles()
            .next()
            .map_or(0.0, |(_, b)| b.lattice().spacing_max()),
    )
}

/// Combines samples in the order they were collected.
///
/// Separate for the reason Unit 11 made [`crate::dexel::deviation::reduce`]
/// separate: the maxima reassociate freely and **the RMS does not**.
#[must_use]
pub fn reduce(
    samples: Vec<Deviation>,
    stock_facet_mm: f64,
    nominal_facet_mm: f64,
    spacing_mm: f64,
) -> DeviationField {
    let mut worst_gouge = 0.0f64;
    let mut worst_excess = 0.0f64;
    let mut worst_gap = 0.0f64;
    let mut sum_sq = 0.0f64;
    for d in &samples {
        if d.signed_mm < 0.0 {
            worst_gouge = worst_gouge.max(-d.signed_mm);
        } else {
            worst_excess = worst_excess.max(d.signed_mm);
        }
        worst_gap = worst_gap.max(d.projection_gap_mm());
        sum_sq += d.signed_mm * d.signed_mm;
    }
    #[allow(clippy::cast_precision_loss, reason = "a sample count")]
    let n = samples.len().max(1) as f64;
    DeviationField {
        worst_gouge_mm: worst_gouge,
        worst_excess_mm: worst_excess,
        rms_mm: (sum_sq / n).sqrt(),
        worst_projection_gap_mm: worst_gap,
        stock_facet_mm,
        nominal_facet_mm,
        spacing_mm,
        samples,
    }
}

/// Signed deviation from `at` to the nominal surface, both ways of measuring it.
///
/// Returns `(surface, perpendicular)`, both signed, in millimetres.
///
/// # Sign from containment, not from which direction hit first
///
/// The sign is decided by whether `at` lies **inside** the nominal solid.
///
/// The first version decided it from *which direction* found the nominal first,
/// and it was wrong in a way worth recording. Consider the side wall of a slot
/// that should not exist: the wall is deep inside the nominal solid, so casting
/// inward finds the far side of the part before casting outward finds anything,
/// and the nearer hit is the inward one. That reads as excess stock when the
/// truth is a gouge — the result's surface is inside the nominal because material
/// was removed that should have stayed.
///
/// Containment does not have that failure mode. A result surface inside the
/// nominal means material is missing there, wherever the nearest face happens to
/// lie; a result surface outside it means material is left over. The corpus
/// caught this as a plunge-too-deep case reporting 2.8 mm of *excess*.
///
/// One sign serves both magnitudes. They measure the distance to the same
/// surface along different paths, so they cannot disagree about which side of it
/// the point is on.
///
/// # Two magnitudes
///
/// The surface distance is the metric and always exists for a non-empty nominal.
/// The perpendicular one is a cast along `+/-normal` and can miss entirely, in
/// which case it falls back to the surface distance rather than to nothing: a
/// missing cast means the normal points into open space, not that there is no
/// deviation.
fn signed_deviation(nominal: &TriMesh, bvh: &Bvh, at: Vec3, normal: Vec3) -> Option<(f64, f64)> {
    // Far enough to cross any plausible part, short enough that a stray hit on
    // the far side of the model is not mistaken for a local deviation.
    const REACH: f64 = 200.0;

    let (nearest, _) = bvh.closest_point(nominal, at)?;
    let surface = nearest.distance(at);

    let cast = |direction: Vec3| -> Option<f64> {
        bvh.intersect_ray(
            nominal,
            &Ray {
                origin: at,
                direction,
            },
        )
        .ok()
        .flatten()
        .filter(|h| h.t >= 0.0 && h.t <= REACH)
        .map(|h| h.t)
    };
    let back = Vec3::new(-normal.x, -normal.y, -normal.z);
    let perpendicular = match (cast(normal), cast(back)) {
        (Some(a), Some(b)) => a.min(b),
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => surface,
    };

    // Inside the nominal: material that should be here is gone.
    let sign = if contains(nominal, bvh, at, normal) {
        -1.0
    } else {
        1.0
    };
    Some((sign * surface, sign * perpendicular))
}

/// Whether `at` lies inside the nominal solid.
///
/// By crossing parity along one ray. The direction is taken from the surface
/// normal rather than a fixed axis so that a point sitting exactly on a nominal
/// face -- which every sample of a correctly machined part does -- resolves the
/// same way each time rather than according to which axis it happens to be
/// parallel to.
fn contains(nominal: &TriMesh, bvh: &Bvh, at: Vec3, normal: Vec3) -> bool {
    let mut hits = Vec::new();
    let query = Ray {
        origin: at,
        direction: normal,
    };
    if bvh
        .intersect_ray_all_into(nominal, &query, &mut hits)
        .is_err()
    {
        return false;
    }
    // Strictly ahead: a hit at the origin means the point is ON the surface, not
    // through it, and counting it would flip every on-surface sample.
    hits.iter().filter(|h| h.t > crate::eps::EPS_LENGTH).count() % 2 == 1
}
