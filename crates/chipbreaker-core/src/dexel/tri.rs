// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Three orthogonal bundles, and the guarantee that makes them worth having.
//!
//! # This is not "three bundles so the volume is more accurate"
//!
//! It is **three bundles so that every surface, whatever its orientation, is
//! well sampled by at least one of them**. Unit 5 established that volume is the
//! wrong thing to judge this on, twice over: it is non-monotone, because the
//! Gauss circle term oscillates and boundary errors cancel with signs inside a
//! global integral; and it floors out against tessellation below about
//! `h/R = 1/40`, so most of any improvement is invisible behind the mesher.
//!
//! See [ADR 0005](../../../docs/adr/0005-deviation-not-volume.md). The rule is:
//! **volume is a construction-time diagnostic, deviation is the assertion
//! metric.**
//!
//! # The theorem
//!
//! How well a bundle samples a surface depends on the angle between the surface
//! normal `n` and the bundle direction. Sampling is exact when the ray is normal
//! to the surface and degenerates as the surface turns parallel to the ray.
//!
//! For any unit normal `n` and the three coordinate axes:
//!
//! ```text
//! max( |n.x|, |n.y|, |n.z| )  >=  1 / sqrt(3)  ~=  0.5773503
//! ```
//!
//! **Proof.** Suppose all three were strictly below `1/sqrt(3)`. Then
//! `n.x^2 + n.y^2 + n.z^2 < 3 * (1/3) = 1`, contradicting `|n| = 1`. Equality
//! needs all three components equal in magnitude, which is exactly a body
//! diagonal such as `(1,1,1)/sqrt(3)`.
//!
//! So the worst surface in a tri-dexel field is met at `54.7356` degrees by all
//! three bundles, and never worse. [`WORST_CASE_COSINE`] is that bound and
//! `dexel coverage` measures against it.
//!
//! # Two different constants, and conflating them would misreport gouges
//!
//! "Deviation" can mean two things here and they differ by up to `3/sqrt(2)`.
//! Both are derived below; both are exported; the harness asserts on the first
//! and U9 will need the second.
//!
//! Throughout, `t` is the transverse distance from a surface point to the
//! nearest ray of a bundle. The transverse cells tile the plane, so
//! `t <= h/sqrt(2)` -- **half the cell DIAGONAL**, not half a cell.
//!
//! ## 1. Sample distance: how far is the nearest place the field sampled?
//!
//! This is what [`super::deviation`] measures, and it is the sampling-adequacy
//! question the `1/sqrt(3)` bound is about.
//!
//! A span endpoint `E` is an exact ray-surface intersection, so for a planar
//! patch both `E` and the surface point `P` lie on the same plane. Their
//! transverse separation is `t`; moving `t` transversely along a plane moves
//! `t * tan(theta)` along the ray axis, so
//!
//! ```text
//! |PE|^2 = t^2 + (t tan(theta))^2 = t^2 / cos^2(theta)
//! |PE|   = t / cos(theta)   <=   (h / sqrt(2)) / cos(theta)
//! ```
//!
//! Best of three axes gives `cos(theta) >= 1/sqrt(3)`, so
//!
//! ```text
//! sample distance  <=  h * sqrt(3/2)  ~=  1.224745 * h
//! ```
//!
//! That is [`SAMPLE_DISTANCE_CONSTANT`], and it is **tight**: a dense sweep of
//! an octahedron -- every face normal is a body diagonal -- attains
//! `1.224745 * h` exactly, at a vertex. An axis-aligned box, where the best
//! bundle has `cos(theta) = 1`, attains exactly `h/sqrt(2) = 0.707107 * h`.
//! Both measured to nine digits.
//!
//! **This distance is almost entirely lateral -- along the surface, not through
//! it.** For a plane the component along the surface normal is exactly zero,
//! because both ends of the displacement lie on the surface.
//!
//! ## 2. Perpendicular deviation: how far is a reconstruction from the truth?
//!
//! A different quantity, and the one that resembles gouge depth. It is **not a
//! property of the field at all** -- the field's samples sit exactly on the
//! surface -- but of whatever rule fills the space *between* samples.
//!
//! For the simplest rule, a flat top per cell at its ray's height, the error at
//! transverse offset `t` is `t * sin(theta)`, so
//!
//! ```text
//! perpendicular  <=  (h / sqrt(2)) * sin(theta)
//! best of three  <=  (h / sqrt(2)) * sqrt(2/3)  =  h / sqrt(3)  ~=  0.577350 * h
//! ```
//!
//! That is [`PERPENDICULAR_CONSTANT`]. Its value coincides numerically with
//! [`WORST_CASE_COSINE`] -- both reduce to `1/sqrt(3)` -- which is arithmetic
//! rather than a duplicated line, and is noted because it looks like one.
//!
//! ## Do not substitute one for the other
//!
//! The ratio of the two bounds is `3/sqrt(2) ~= 2.12`. Pointwise it is
//! `1/(sin(theta) cos(theta))`, which is **unbounded**: a face exactly normal to
//! a bundle has zero perpendicular error and a sample distance of up to
//! `h/sqrt(2)`. Unit 12 reports gouge depth, a perpendicular quantity; quoting
//! the sample-distance figure there would overstate it without limit on exactly
//! the surfaces that are best sampled.
//!
//! ## Neither bounds Unit 9
//!
//! [`PERPENDICULAR_CONSTANT`] is the bound for *nearest-neighbour*
//! reconstruction. A rule that interpolates between adjacent ray endpoints
//! should do better on smooth surfaces and worse across a sharp edge. Unit 9
//! must measure its own output; neither constant here is a substitute.
//!
//! # Cutting does not accumulate error across operations
//!
//! Stated here because Unit 7 depends on it and it is not obvious.
//!
//! A cut is **exact along each ray**: subtracting the swept tool from a ray's
//! spans is interval arithmetic on exact intersection parameters, not a
//! resampling. So after a thousand cuts, bundle X's field is still exactly the
//! true remaining solid sampled on X's lattice. The only error is the fixed
//! transverse sampling, set by `h` and unchanged by how many operations
//! preceded it.
//!
//! Two consequences:
//!
//! - The three bundles stay **independently correct** rather than drifting
//!   apart. Unit 7 should subtract per bundle and never compare them;
//!   reconciling them is Unit 9's job, and doing it earlier would mean
//!   reconstructing a surface two units ahead of schedule.
//! - Unit 15's "a thousand chained cuts equal one monolithic cut" test is
//!   achievable rather than aspirational, because there is no accumulation term
//!   for it to fight.
//!
//! # Registration
//!
//! The three lattices are **not** co-registered, and deliberately so.
//! Registration buys nothing here and would constrain Unit 10's adaptive
//! subdivision. Each bundle records its own origin, spacing and counts in the
//! format, so Unit 9 can reason about their relationship rather than assume one.
//!
//! # The half-cell offset is per bundle
//!
//! Each bundle applies it in **its own** transverse plane. The invariant is not
//! global, and Unit 5's corner-versus-centre table (247 and 857 coplanar
//! rejections against zero) is why it cannot be relaxed on any of the three.

use crate::budget::Spacing;
use crate::golden::{CanonicalHash, Digest, Hashable};
use crate::math::{Aabb3, Axis, Mat4, Vec3};
use crate::mesh::TriMesh;

use super::field::{BuildError, BuildOptions, BuildStats, DexelField};
use super::tessellation::{self, TessellationEstimate};

/// The bundle axes, in the order they are stored, hashed and serialized.
pub const AXES: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

/// `1 / sqrt(3)`: the worst-case best-of-three sampling cosine.
///
/// Attained exactly on a body diagonal. See the module header for the proof.
/// Written as a division rather than a decimal so it is exactly the `f64`
/// nearest the real value on every target.
pub const WORST_CASE_COSINE: f64 = 0.577_350_269_189_625_7;

/// `sqrt(3/2)`: the bound on **sample distance**, in units of `h`.
///
/// The metric [`super::deviation`] measures and asserts on. Tight: attained
/// exactly at an octahedron's vertices, where every face normal is a body
/// diagonal. See the module header for the derivation.
///
/// Do **not** use this for gouge depth or any other perpendicular quantity --
/// see [`PERPENDICULAR_CONSTANT`].
pub const SAMPLE_DISTANCE_CONSTANT: f64 = 1.224_744_871_391_589;

/// `1/sqrt(2)`: sample distance where the best bundle is normal to the surface.
///
/// The `cos(theta) = 1` case of [`SAMPLE_DISTANCE_CONSTANT`]. Attained exactly
/// at an axis-aligned box's vertices, so it is the floor for the metric rather
/// than a convenience.
pub const AXIS_ALIGNED_SAMPLE_CONSTANT: f64 = core::f64::consts::FRAC_1_SQRT_2;

/// `1/sqrt(3)`: the bound on **perpendicular** deviation, in units of `h`.
///
/// For a nearest-neighbour reconstruction -- a flat top per cell. Not a
/// property of the field, which samples the surface exactly; a property of the
/// rule that fills the gaps. This is the shape of the number Unit 12's gouge
/// depth needs, though Unit 9 must re-derive it for its own reconstruction.
///
/// Numerically equal to [`WORST_CASE_COSINE`]. Arithmetic, not a duplicated
/// line: both reduce to `1/sqrt(3)`.
pub const PERPENDICULAR_CONSTANT: f64 = 0.577_350_269_189_625_7;

/// Where a field came from, and how good its input was.
///
/// Carried so that a `.tdx` file can answer "should I trust this to 0.05 mm?"
/// without the mesh that produced it.
#[derive(Debug, Clone, PartialEq)]
pub struct Provenance {
    /// Canonical digest of the source mesh.
    pub source_digest: Digest,
    /// Triangles in the source mesh.
    pub source_triangles: u32,
    /// Cell size requested, in millimetres.
    pub requested_spacing_mm: f64,
    /// What the source mesh's own fidelity looks like.
    ///
    /// Recorded per §1c so that a field built from a coarse mesh carries the
    /// evidence with it. A number that only ever appeared in a warning on
    /// somebody's terminal is a number nobody has by the time it matters.
    pub tessellation: TessellationEstimate,
}

impl Hashable for Provenance {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Provenance");
        h.bytes(self.source_digest.as_bytes());
        h.u64(u64::from(self.source_triangles));
        h.f64(self.requested_spacing_mm);
        h.add(&self.tessellation);
        h.end();
    }
}

/// Which bundles to build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisSet(u8);

impl AxisSet {
    /// All three. The only set that carries the `1/sqrt(3)` guarantee.
    pub const XYZ: Self = Self(0b111);

    /// A set from a string like `"xyz"` or `"xz"`.
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        let mut bits = 0u8;
        for c in s.chars() {
            bits |= match c.to_ascii_lowercase() {
                'x' => 1,
                'y' => 2,
                'z' => 4,
                _ => return None,
            };
        }
        if bits == 0 { None } else { Some(Self(bits)) }
    }

    /// True if this set includes `axis`.
    #[must_use]
    pub const fn contains(self, axis: Axis) -> bool {
        self.0 & (1 << axis.index()) != 0
    }

    /// How many bundles.
    #[must_use]
    pub const fn len(self) -> u32 {
        self.0.count_ones()
    }

    /// True if empty, which [`Self::parse`] never produces.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Lowercase name, e.g. `"xyz"`.
    #[must_use]
    pub fn as_str(self) -> String {
        AXES.iter()
            .filter(|a| self.contains(**a))
            .map(|a| a.as_str())
            .collect()
    }
}

impl Default for AxisSet {
    fn default() -> Self {
        Self::XYZ
    }
}

/// How to build a tri-dexel field.
#[derive(Debug, Clone, PartialEq)]
pub struct TriBuildOptions {
    /// Cell size, in millimetres. The isotropic shorthand.
    ///
    /// Ignored when [`Self::spacing_xyz`] is set.
    pub spacing: f64,
    /// Independent cell size per world axis.
    ///
    /// Registration-safe by construction: each axis still draws its ordinates
    /// from one shared set, the sets simply have different steps. Every bundle
    /// takes the two that are transverse to it, so the three still agree about
    /// where a corner is.
    pub spacing_xyz: Option<Spacing>,
    /// Which bundles to build.
    pub axes: AxisSet,
    /// Where the stock sits in machine coordinates.
    pub placement: Mat4,
    /// Extra room around the stock bounds, in millimetres.
    pub margin: f64,
}

impl Default for TriBuildOptions {
    fn default() -> Self {
        Self {
            spacing: 0.5,
            spacing_xyz: None,
            axes: AxisSet::XYZ,
            placement: Mat4::IDENTITY,
            margin: 0.0,
        }
    }
}

/// What building three bundles cost.
#[derive(Debug, Clone, PartialEq)]
pub struct TriBuildStats {
    /// Per bundle, in [`AXES`] order. `None` for an axis that was not built.
    pub per_axis: [Option<BuildStats>; 3],
    /// Rays across every bundle.
    pub rays: u64,
    /// Spans across every bundle.
    pub spans: u64,
    /// Coplanar rejections. Always zero: construction aborts on the first.
    pub coplanar_rejected: u64,
    /// Degenerate triangles dropped before casting.
    pub degenerate_triangles: u32,
}

/// Three orthogonal dexel bundles over the same stock.
#[derive(Debug, Clone, PartialEq)]
pub struct TriDexelField {
    /// In [`AXES`] order. `None` for an axis that was not built.
    bundles: [Option<DexelField>; 3],
    provenance: Provenance,
}

impl TriDexelField {
    /// Builds every requested bundle from a closed stock mesh.
    ///
    /// # Errors
    /// See [`BuildError`]. A coplanar rejection or an odd crossing count on
    /// **any** bundle aborts the whole build, because a field with one unusable
    /// bundle is not a tri-dexel field.
    pub fn build(
        mesh: &TriMesh,
        options: &TriBuildOptions,
    ) -> Result<(Self, TriBuildStats), BuildError> {
        // Estimated once, from the source mesh, before any placement: the
        // tessellation is a property of the mesh, not of where it sits.
        let tessellation = tessellation::estimate(mesh);
        let mut digest = CanonicalHash::new();
        digest.add(mesh);
        let provenance = Provenance {
            source_digest: digest.finish(),
            source_triangles: mesh.triangle_count(),
            requested_spacing_mm: options.spacing,
            tessellation,
        };

        let mut bundles: [Option<DexelField>; 3] = [None, None, None];
        let mut per_axis: [Option<BuildStats>; 3] = [None, None, None];
        let mut totals = TriBuildStats {
            per_axis: [None, None, None],
            rays: 0,
            spans: 0,
            coplanar_rejected: 0,
            degenerate_triangles: 0,
        };

        // Ascending axis index, and the order is contract for the same reason
        // ray order is: the hash walks the bundles in this order.
        for axis in AXES {
            if !options.axes.contains(axis) {
                continue;
            }
            let (field, stats) = DexelField::build(
                mesh,
                &BuildOptions {
                    spacing: options.spacing,
                    spacing_xyz: options.spacing_xyz,
                    axis,
                    placement: options.placement,
                    margin: options.margin,
                },
            )?;
            totals.rays += stats.rays;
            totals.spans += stats.spans;
            totals.coplanar_rejected += stats.predicates.coplanar_rejected;
            totals.degenerate_triangles = stats.degenerate_triangles;
            per_axis[axis.index()] = Some(stats);
            bundles[axis.index()] = Some(field);
        }
        totals.per_axis = per_axis;

        let field = Self {
            bundles,
            provenance,
        };
        field.check_registration()?;
        Ok((field, totals))
    }

    /// Corner coordinates along one world axis, from whichever bundle sees it.
    ///
    /// Returns `None` when no built bundle has a transverse coordinate along
    /// `world` — with a single bundle, two of the three axes are like that.
    #[must_use]
    pub fn corner_coordinates(&self, world: Axis) -> Option<Vec<f64>> {
        for bundle_axis in AXES {
            let Some(bundle) = self.bundle(bundle_axis) else {
                continue;
            };
            let lattice = bundle.lattice();
            let [u, v, _] = bundle_axis.cyclic();
            let which = if u == world.index() {
                0
            } else if v == world.index() {
                1
            } else {
                continue;
            };
            let n = lattice.counts()[which];
            return Some(
                (0..n)
                    .map(|k| {
                        let (i, j) = if which == 0 { (k, 0) } else { (0, k) };
                        lattice.origin_of(i, j).to_array()[world.index()]
                    })
                    .collect(),
            );
        }
        None
    }

    /// Verifies that the three bundles share one corner lattice.
    ///
    /// # Why this is an invariant and not a coincidence
    ///
    /// Unit 6 recorded that the three bundles need not be co-registered. That
    /// was wrong, and Unit 9 is where it comes due: dual contouring needs a
    /// single grid whose **corners** are ray positions, because the three
    /// bundles *are* the three edge directions of that grid. An X-directed edge
    /// from `(x_i, y_j, z_k)` to `(x_{i+1}, y_j, z_k)` has to be a sub-segment
    /// of the X-bundle ray at transverse `(y_j, z_k)`, which requires all three
    /// bundles to draw their transverse coordinates from one common set.
    ///
    /// The Unit 6 centring already delivers this, and to the bit: `pad` depends
    /// only on the axis extent and the spacing, both shared, so the Y ordinates
    /// of the X-bundle's rays and of the Z-bundle's rays are computed from
    /// identical inputs by identical arithmetic. Measured at 0 ULP across five
    /// stock sizes including deliberately awkward spacings.
    ///
    /// It is checked anyway, because it is now load-bearing and was previously
    /// free to change. Unit 10's adaptive subdivision is the thing most likely
    /// to break it.
    ///
    /// **Note on the half-cell offset.** This puts the DC cell grid half a cell
    /// away from the dexel cell grid: dexel cell centres are the DC grid's
    /// corners. That is a relabelling, not a violation of Unit 5's rule that
    /// ray origins avoid the integer lattice — the rays are exactly where they
    /// always were, and only the name of the grid they define has changed.
    ///
    /// # Errors
    /// [`BuildError::Registration`] if two bundles disagree about the position
    /// or the count of the corners along a shared axis.
    pub fn check_registration(&self) -> Result<(), BuildError> {
        for world in AXES {
            let mut reference: Option<(Axis, Vec<f64>)> = None;
            for bundle_axis in AXES {
                let Some(bundle) = self.bundle(bundle_axis) else {
                    continue;
                };
                let lattice = bundle.lattice();
                let [u, v, _] = bundle_axis.cyclic();
                let which = if u == world.index() {
                    0
                } else if v == world.index() {
                    1
                } else {
                    continue;
                };
                let n = lattice.counts()[which];
                let coords: Vec<f64> = (0..n)
                    .map(|k| {
                        let (i, j) = if which == 0 { (k, 0) } else { (0, k) };
                        lattice.origin_of(i, j).to_array()[world.index()]
                    })
                    .collect();
                match &reference {
                    None => reference = Some((bundle_axis, coords)),
                    Some((first_axis, first)) => {
                        if first.len() != coords.len() {
                            return Err(BuildError::Registration {
                                axis: world.as_str(),
                                detail: format!(
                                    "bundle {} has {} corners along {} but bundle {} has {}",
                                    first_axis.as_str(),
                                    first.len(),
                                    world.as_str(),
                                    bundle_axis.as_str(),
                                    coords.len()
                                ),
                            });
                        }
                        // Bit equality, not a tolerance. These are the same
                        // arithmetic on the same inputs; anything else means the
                        // lattices were derived differently, and a tolerance
                        // would let that through until it mattered.
                        for (k, (a, b)) in first.iter().zip(coords.iter()).enumerate() {
                            if a.to_bits() != b.to_bits() {
                                return Err(BuildError::Registration {
                                    axis: world.as_str(),
                                    detail: format!(
                                        "corner {k} along {} is {a} in bundle {} and {b} in \
                                         bundle {}",
                                        world.as_str(),
                                        first_axis.as_str(),
                                        bundle_axis.as_str()
                                    ),
                                });
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Reassembles from parts, for the `.tdx` reader.
    #[must_use]
    pub const fn from_parts(bundles: [Option<DexelField>; 3], provenance: Provenance) -> Self {
        Self {
            bundles,
            provenance,
        }
    }

    /// One bundle, if it was built.
    #[must_use]
    pub const fn bundle(&self, axis: Axis) -> Option<&DexelField> {
        self.bundles[axis.index()].as_ref()
    }

    /// One bundle mutably, if it was built.
    ///
    /// The only way Unit 7 reaches a field's contents, and deliberately one
    /// bundle at a time: the cut contract is that a bundle is subtracted from
    /// independently and never compared with another.
    pub const fn bundle_mut(&mut self, axis: Axis) -> Option<&mut DexelField> {
        self.bundles[axis.index()].as_mut()
    }

    /// Every bundle that was built, in [`AXES`] order.
    pub fn bundles(&self) -> impl Iterator<Item = (Axis, &DexelField)> {
        AXES.into_iter()
            .filter_map(|a| self.bundles[a.index()].as_ref().map(|f| (a, f)))
    }

    /// Which axes this field carries.
    #[must_use]
    pub fn axes(&self) -> AxisSet {
        let mut bits = 0u8;
        for (axis, _) in self.bundles() {
            bits |= 1 << axis.index();
        }
        AxisSet(bits)
    }

    /// Where it came from and how good the input was.
    #[must_use]
    pub const fn provenance(&self) -> &Provenance {
        &self.provenance
    }

    /// True if all three bundles are present, and the `1/sqrt(3)` guarantee
    /// therefore holds.
    ///
    /// A two-bundle field has no such bound: a surface normal perpendicular to
    /// both remaining axes is sampled by neither.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.axes() == AxisSet::XYZ
    }

    /// Per-bundle volume, in [`AXES`] order.
    ///
    /// **A diagnostic, not a metric.** The three will disagree at `O(h^2)` with
    /// independent signs, and demanding they agree tightly would be demanding
    /// that three independent errors coincide. ADR 0005 has the argument.
    #[must_use]
    pub fn volumes(&self) -> [Option<f64>; 3] {
        [
            self.bundles[0].as_ref().map(DexelField::volume),
            self.bundles[1].as_ref().map(DexelField::volume),
            self.bundles[2].as_ref().map(DexelField::volume),
        ]
    }

    /// Mean of the per-bundle volumes.
    ///
    /// Averaged in ascending axis order, because floating-point addition is not
    /// associative and a different order is a different number. Still a
    /// diagnostic; see [`Self::volumes`].
    #[must_use]
    pub fn volume(&self) -> f64 {
        let mut total = 0.0;
        let mut count = 0u32;
        for (_, field) in self.bundles() {
            total += field.volume();
            count += 1;
        }
        if count == 0 {
            0.0
        } else {
            total / f64::from(count)
        }
    }

    /// Total spans across every bundle.
    #[must_use]
    pub fn total_spans(&self) -> usize {
        self.bundles().map(|(_, f)| f.total_spans()).sum()
    }

    /// Total rays across every bundle.
    #[must_use]
    pub fn rays(&self) -> usize {
        self.bundles().map(|(_, f)| f.arena().rays()).sum()
    }

    /// Bytes of span storage across every bundle.
    ///
    /// The number Unit 10 has to beat. Note it scales with **half the
    /// bounding-box surface area**, not with three times one face: the bundles
    /// cover `(WD + DH + HW) / h^2` rays between them. For a cube that is 3x a
    /// single bundle; for a 100x100x10 plate it is 1.2x, and for a 100x100x200
    /// bar it is 5x.
    #[must_use]
    pub fn bytes(&self) -> usize {
        self.bundles().map(|(_, f)| f.arena().bytes()).sum()
    }

    /// The union of the bundles' material bounds.
    #[must_use]
    pub fn material_bounds(&self) -> Aabb3 {
        self.bundles()
            .map(|(_, f)| f.material_bounds())
            .fold(Aabb3::EMPTY, |acc, b| acc.union(&b))
    }
}

impl Hashable for TriDexelField {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("TriDexelField");
        h.add(&self.provenance);
        // Every slot, present or not, so a two-bundle field cannot hash the
        // same as a three-bundle one that happens to share its contents.
        for axis in AXES {
            h.begin(axis.as_str());
            match &self.bundles[axis.index()] {
                Some(field) => {
                    h.bool(true);
                    h.add(field);
                }
                None => {
                    h.bool(false);
                }
            }
            h.end();
        }
        h.end();
    }
}

/// The best sampling cosine any of `axes` achieves against a unit normal.
///
/// `max(|n.x|, |n.y|, |n.z|)` when all three are present, which the module
/// header proves is never below [`WORST_CASE_COSINE`].
#[must_use]
pub fn best_cosine(normal: Vec3, axes: AxisSet) -> f64 {
    let n = normal.to_array();
    let mut best = 0.0f64;
    for axis in AXES {
        if axes.contains(axis) {
            best = best.max(n[axis.index()].abs());
        }
    }
    best
}
