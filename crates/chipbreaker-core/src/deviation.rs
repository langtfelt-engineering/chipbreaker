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
use crate::transcendental as t;

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

/// Above this dihedral angle an edge is a design feature, not a sampled curve.
///
/// The distinction is the whole of [`facet_size`]. Thirty degrees is well above
/// any tessellation a customer would ship — a curve sampled that coarsely would
/// be visible as facets to the naked eye — and well below the shallowest edge
/// anyone deliberately puts on a part.
const FEATURE_ANGLE: f64 = core::f64::consts::PI / 6.0;

/// How far a mesh can be from the surface it represents, in millimetres.
///
/// # Big triangles are not the same as a coarse mesh
///
/// The first version of this returned the square root of the mean triangle area:
/// the side of the equal-area equilateral triangle, as a single number for "how
/// finely is this tessellated". It refused to compare anything against a box.
///
/// A box's faces are *planes*, and twelve triangles represent them **exactly**.
/// No refinement would improve them, so a floor derived from their size is a
/// refusal to answer a question that has an exact answer — and refusing to
/// answer is worse than answering with a caveat. Every prismatic part, which is
/// most machined parts, hit it.
///
/// # What is actually being estimated
///
/// The **chord error**: how far the true smooth surface departs from the flat
/// facet standing in for it. That is not a property of a triangle on its own; it
/// is a property of how a triangle sits relative to its neighbours.
///
/// Two facets meeting across an edge at a dihedral angle `theta` are sampling a
/// surface whose radius of curvature is set by **how far they reach across that
/// edge**, not by how long the edge is:
///
/// ```text
///   rho  =  (w_a + w_b) / (2 theta)          w = the facet's reach, perpendicular
///   s    =  rho * (1 - cos(theta / 2))       to the shared edge
/// ```
///
/// which is exact for a regular polygon inscribed in a circle: `N` facets around
/// radius `R` reach `L = 2R sin(pi/N)` each and turn by `theta = 2 pi / N`, so
/// `rho` comes back as `R` and `s` as the sagitta `R (1 - cos(pi/N))`.
///
/// The reach is what matters and the edge length is not, which the first version
/// of this had backwards. It used the edge length directly, so a cylinder
/// tessellated into `N` segments reported a chord error proportional to its
/// **height** — the vertical edges are the ones that carry the dihedral, and
/// their length says nothing at all about how well the circle is sampled. A
/// torus happened to come out near enough to hide it.
///
/// Splitting a quad with a diagonal costs little under this form: the diagonal's
/// two triangles are nearly coplanar, so its `theta` is small and its
/// contribution falls off as `theta`.
///
/// Two angles contribute nothing, for opposite reasons. **Coplanar** facets have
/// `theta = 0`: they are one plane, sampled twice, with no error between them.
/// **Feature** edges above [`FEATURE_ANGLE`] are a crease the part really has,
/// and refining the mesh will never soften them — measuring a 90 degree corner
/// as though it were a coarsely sampled fillet is the same mistake as before,
/// wearing the opposite hat.
///
/// A box therefore returns zero, a coarsely tessellated sphere returns its chord
/// error, and a part that is mostly flat with one rough fillet returns the
/// fillet's.
///
/// # It is an estimate, and is used to warn rather than to bound
///
/// The formula is exact for a regular polygon inscribed in a circle and nothing
/// else. A triangulated torus is not one, and the estimate comes back at a
/// steady **1.23 times** the closed form across three tessellations of the same
/// torus, and 1.5 to 1.6 on an icosphere: a consistent, slightly conservative
/// offset that depends on how the shape was triangulated. Measured in
/// `tests/facet_floor.rs`.
///
/// That is enough for the job. A customer asking for 0.01 mm against a mesh
/// whose facets are half a millimetre out needs to be told the scale of the
/// problem. Calling this a bound would be the more comfortable claim and the
/// wrong one.
///
/// # The limitation worth knowing
///
/// Below roughly twelve segments per revolution the dihedral exceeds
/// [`FEATURE_ANGLE`] and the mesh reads as a faceted *design* rather than a
/// coarse sampling — a bare icosahedron standing in for a sphere returns zero.
/// Nothing in a mesh file distinguishes the two, and the choice here is to
/// believe the part: treating a genuine 45 degree chamfer as sampling error
/// would inflate the floor on every chamfered part and refuse comparisons that
/// are exactly answerable, which is the failure this function was rewritten to
/// remove.
#[must_use]
pub fn facet_size(mesh: &TriMesh) -> f64 {
    use std::collections::BTreeMap;

    // Undirected edge -> the triangles on it. `u32` keys, and a BTreeMap rather
    // than a hash map, because the iteration below reaches a float.
    let mut edges: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
    for (index, tri) in mesh.triangles().iter().enumerate() {
        let t = u32::try_from(index).unwrap_or(u32::MAX);
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let key = if a <= b { (a, b) } else { (b, a) };
            edges.entry(key).or_default().push(t);
        }
    }

    let mut worst = 0.0f64;
    for ((a, b), tris) in &edges {
        // Boundary and non-manifold edges have no single dihedral angle. The
        // validator reports those; here they simply contribute nothing.
        let [ta, tb] = tris[..] else { continue };
        let (Some(na), Some(nb)) = (mesh.face_normal(ta), mesh.face_normal(tb)) else {
            continue;
        };
        let cos = (na.x * nb.x + na.y * nb.y + na.z * nb.z).clamp(-1.0, 1.0);
        let theta = t::acos(cos);
        if theta <= 0.0 || theta >= FEATURE_ANGLE {
            continue;
        }
        let reach = facet_reach(mesh, ta, *a, *b) + facet_reach(mesh, tb, *a, *b);
        if reach <= 0.0 {
            continue;
        }
        let rho = reach / (2.0 * theta);
        // `1 - cos(theta/2)` written as `2 sin^2(theta/4)`, which does not
        // cancel to nothing when theta is small -- and on a finely tessellated
        // mesh theta is always small.
        let quarter = t::sin(0.25 * theta);
        worst = worst.max(rho * 2.0 * quarter * quarter);
    }
    worst
}

/// How far triangle `tri` reaches from the line through `a` and `b`.
///
/// The perpendicular distance from its third vertex to that line: the extent of
/// the facet across the shared edge, which is what sets the curvature the two
/// facets are sampling.
fn facet_reach(mesh: &TriMesh, tri: u32, a: u32, b: u32) -> f64 {
    let Some(indices) = mesh.triangles().get(tri as usize) else {
        return 0.0;
    };
    let Some(&opposite) = indices.iter().find(|v| **v != a && **v != b) else {
        return 0.0;
    };
    let vertices = mesh.vertices();
    let (Some(pa), Some(pb), Some(pc)) = (
        vertices.get(a as usize),
        vertices.get(b as usize),
        vertices.get(opposite as usize),
    ) else {
        return 0.0;
    };
    let edge = *pb - *pa;
    let length = edge.length();
    if length <= 0.0 {
        return 0.0;
    }
    // Twice the triangle's area over the edge length is the height on that edge.
    edge.cross(*pc - *pa).length() / length
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
pub fn compare(
    field: &TriDexelField,
    nominal: &TriMesh,
    stock: Option<&TriMesh>,
) -> DeviationField {
    let bvh = Bvh::build(nominal);
    let mut samples: Vec<Deviation> = Vec::new();
    // Two buffers, reused across every sample. One query per span endpoint means
    // hundreds of thousands of them on a real part, and a `Vec` allocated and
    // freed inside each would be pure overhead.
    let mut stack: Vec<u32> = Vec::new();
    let mut hits = Vec::new();

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
                        signed_deviation(nominal, &bvh, at, normal, &mut stack, &mut hits)
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
fn signed_deviation(
    nominal: &TriMesh,
    bvh: &Bvh,
    at: Vec3,
    normal: Vec3,
    stack: &mut Vec<u32>,
    hits: &mut Vec<crate::mesh::bvh::Hit>,
) -> Option<(f64, f64)> {
    // Far enough to cross any plausible part, short enough that a stray hit on
    // the far side of the model is not mistaken for a local deviation.
    const REACH: f64 = 200.0;

    let (nearest, _) = bvh.closest_point_into(nominal, at, stack)?;
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
    let sign = if contains(nominal, bvh, at, normal, hits) {
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
fn contains(
    nominal: &TriMesh,
    bvh: &Bvh,
    at: Vec3,
    normal: Vec3,
    hits: &mut Vec<crate::mesh::bvh::Hit>,
) -> bool {
    let query = Ray {
        origin: at,
        direction: normal,
    };
    if bvh.intersect_ray_all_into(nominal, &query, hits).is_err() {
        return false;
    }
    // Strictly ahead: a hit at the origin means the point is ON the surface, not
    // through it, and counting it would flip every on-surface sample.
    hits.iter().filter(|h| h.t > crate::eps::EPS_LENGTH).count() % 2 == 1
}
