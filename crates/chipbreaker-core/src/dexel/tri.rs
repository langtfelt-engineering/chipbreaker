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
//! # The corollary the bound implies, which is the one U12 will quote
//!
//! Take a plane with normal `n`, sampled by a bundle along `d`, cells of size
//! `h`. Expressed as a height over the transverse plane the surface has gradient
//! `tan(theta)`, `theta` being the angle between `n` and `d`. Over one cell the
//! height moves by `h * tan(theta)`, and the **perpendicular** distance from the
//! true plane to the sampled one is that times `cos(theta)`:
//!
//! ```text
//! deviation  ~  (h / 2) * sin(theta)
//! ```
//!
//! Zero when the ray is normal to the surface, worst when it is parallel. Taking
//! the best of three axes, `sin(theta)` is at most `sqrt(1 - 1/3)`, so
//!
//! ```text
//! best-of-three deviation  <=  (h / 2) * sqrt(2/3)  ~=  0.408 * h
//! ```
//!
//! That is [`DEVIATION_CONSTANT`]. It converts "bounded by a constant times `h`"
//! into a specific constant, and it has no cancellation and no oscillation in
//! it — which is the whole reason deviation replaced volume as the metric.
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

/// `sqrt(2/3) / 2`: the constant in `deviation <= C * h` for a planar surface.
///
/// See the corollary in the module header.
pub const DEVIATION_CONSTANT: f64 = 0.408_248_290_463_863;

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
    /// Cell size, in millimetres. Shared by every bundle at Unit 6.
    pub spacing: f64,
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

        Ok((
            Self {
                bundles,
                provenance,
            },
            totals,
        ))
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
