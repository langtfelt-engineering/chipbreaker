// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Building a field from a closed mesh.
//!
//! One bundle of parallel rays, each cast through Unit 2's BVH, the crossings
//! paired into spans, the spans stored in the arena. That is the whole of it.
//! **Nothing here cuts** — subtraction is Unit 7.
//!
//! # Two conditions are hard errors, not statistics
//!
//! Construction refuses to produce a field when either of these occurs, and the
//! choice is deliberate enough to be worth the paragraph.
//!
//! **A coplanar rejection.** Unit 2's caster discards a triangle that lies in
//! the ray's own plane, because such a triangle has no well-defined crossing:
//! the ray is *in* the surface rather than passing through it. Counting them and
//! carrying on would mean building a field with a hole whose size nobody knows.
//! ADR 0001 Part 2 requires this to abort, and the cell-centre ray offset is
//! what makes it unreachable for axis-aligned stock — which is most stock. If
//! this fires, the geometry is unusual and the right response is to look at it,
//! not to average over it.
//!
//! **An odd number of crossings.** A ray entering a closed solid must leave it.
//! An odd count means the mesh is not closed along that ray, or a crossing was
//! missed. Either way the material extends to infinity as far as this ray knows,
//! and there is no honest span to record. Unit 2 already validates closedness at
//! load, so reaching this means something disagrees, and a silent repair would
//! hide the disagreement.
//!
//! Both carry the ray index and its origin, because the first question anyone
//! asks is "where".
//!
//! # Order is contract
//!
//! Rays are cast in ascending ray index and the volume sum accumulates in the
//! same order. Floating-point addition is not associative, so a different
//! traversal is a different number. This is why Unit 11 cannot simply wrap the
//! loop in a parallel iterator and why there is no parallelism before it.

use std::borrow::Cow;

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Axis, Mat4, Ray, Vec3};
use crate::mesh::TriMesh;
use crate::mesh::bvh::{Bvh, RayError, RayStats};
use crate::spans::Span;

use super::arena::Arena;
use super::lattice::{Lattice, LatticeError};

/// Why a field could not be built.
#[derive(Debug, Clone, PartialEq)]
pub enum BuildError {
    /// The lattice itself was rejected.
    Lattice(LatticeError),
    /// A ray query failed.
    Ray {
        /// Which ray.
        ray: u32,
        /// Where it started.
        origin: [f64; 3],
        /// What went wrong.
        source: RayError,
    },
    /// A triangle lay in a ray's own plane.
    ///
    /// A hard error by ADR 0001 Part 2. See the module header.
    Coplanar {
        /// Which ray.
        ray: u32,
        /// Where it started.
        origin: [f64; 3],
        /// How many triangles were rejected on that ray.
        rejected: u64,
    },
    /// A ray crossed the surface an odd number of times.
    OddCrossings {
        /// Which ray.
        ray: u32,
        /// Where it started.
        origin: [f64; 3],
        /// How many crossings it found.
        crossings: usize,
    },
    /// The placement transform is singular or not finite.
    ///
    /// A degenerate placement collapses the stock into a plane, which is not a
    /// solid and has no interior for rays to find.
    BadPlacement {
        /// The determinant, which is zero or not finite.
        determinant: f64,
    },
    /// The stock mesh has no triangles.
    EmptyMesh,
}

impl core::fmt::Display for BuildError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Lattice(e) => write!(f, "{e}"),
            Self::Ray {
                ray,
                origin,
                source,
            } => write!(f, "ray {ray} from {origin:?} could not be cast: {source}"),
            Self::Coplanar {
                ray,
                origin,
                rejected,
            } => write!(
                f,
                "ray {ray} from {origin:?} met {rejected} triangle(s) lying in its own plane. \
                 A coplanar triangle has no well-defined crossing, so the field would have a \
                 hole of unknown size. This should be unreachable for axis-aligned stock \
                 because ray origins sit at cell centres (ADR 0001 Part 2); reaching it means \
                 the geometry is genuinely unusual and wants looking at"
            ),
            Self::OddCrossings {
                ray,
                origin,
                crossings,
            } => write!(
                f,
                "ray {ray} from {origin:?} crossed the surface {crossings} times, an odd \
                 number. A ray entering a closed solid must leave it, so the mesh is not \
                 closed along this ray or a crossing was missed"
            ),
            Self::BadPlacement { determinant } => write!(
                f,
                "the stock placement transform has determinant {determinant}, so it collapses \
                 the stock rather than placing it"
            ),
            Self::EmptyMesh => write!(f, "the stock mesh has no triangles"),
        }
    }
}

impl core::error::Error for BuildError {}

impl From<LatticeError> for BuildError {
    fn from(e: LatticeError) -> Self {
        Self::Lattice(e)
    }
}

/// How to build a field.
#[derive(Debug, Clone, PartialEq)]
pub struct BuildOptions {
    /// Cell size, in millimetres.
    pub spacing: f64,
    /// Which axis the rays run along. Unit 5 only exercises `Z`.
    pub axis: Axis,
    /// Where the stock sits in machine coordinates.
    ///
    /// Applied to the **mesh**, once, rather than to the rays. Transforming the
    /// rays instead would leave them non-axis-aligned in mesh space, which
    /// destroys the traversal coherence the BVH depends on and makes the span
    /// parameter a length only under a uniform scale. One pass over the vertices
    /// is cheaper and keeps everything downstream axis-aligned.
    pub placement: Mat4,
    /// Extra room around the stock bounds, in millimetres.
    ///
    /// The lattice covers the placed stock plus this margin, so that later units
    /// have somewhere to put material the tool has not reached yet.
    pub margin: f64,
}

impl Default for BuildOptions {
    fn default() -> Self {
        Self {
            spacing: 0.5,
            axis: Axis::Z,
            placement: Mat4::IDENTITY,
            margin: 0.0,
        }
    }
}

/// What building cost, and what it found.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BuildStats {
    /// Rays cast.
    pub rays: u64,
    /// Rays that found no material.
    pub empty_rays: u64,
    /// Spans recorded.
    pub spans: u64,
    /// Rays that outgrew the arena's inline capacity.
    pub spilled_rays: u64,
    /// Predicate counters, summed across every ray.
    pub predicates: RayStats,
}

/// A single-axis dexel field.
///
/// The lattice says where the rays are, the arena says what is on them. Both are
/// needed to say anything about volume, so they travel together.
#[derive(Debug, Clone, PartialEq)]
pub struct DexelField {
    lattice: Lattice,
    arena: Arena,
    placement: Mat4,
}

impl DexelField {
    /// An empty field on a given lattice: every ray carrying no material.
    #[must_use]
    pub fn empty(lattice: Lattice) -> Self {
        let arena = Arena::new(lattice.ray_count());
        Self {
            lattice,
            arena,
            placement: Mat4::IDENTITY,
        }
    }

    /// Reassembles a field from its parts, for the `.dexel` reader.
    ///
    /// Not a general constructor: it trusts that the arena has one entry per
    /// lattice ray, which the reader checks before calling.
    #[must_use]
    pub const fn from_parts(lattice: Lattice, arena: Arena, placement: Mat4) -> Self {
        Self {
            lattice,
            arena,
            placement,
        }
    }

    /// Builds a field from a closed stock mesh.
    ///
    /// # Errors
    /// See [`BuildError`]. A coplanar rejection or an odd crossing count aborts
    /// the build rather than being reported; the module header says why.
    pub fn build(mesh: &TriMesh, options: &BuildOptions) -> Result<(Self, BuildStats), BuildError> {
        if mesh.triangle_count() == 0 {
            return Err(BuildError::EmptyMesh);
        }
        let determinant = options.placement.determinant();
        if !determinant.is_finite() || determinant == 0.0 || !options.placement.is_finite() {
            return Err(BuildError::BadPlacement { determinant });
        }

        let placed = place(mesh, &options.placement);
        let bounds = grow(placed.bounds(), options.margin);
        let lattice = Lattice::new(bounds, options.spacing, options.axis)?;
        let bvh = Bvh::build(placed.as_ref());

        let mut arena = Arena::new(lattice.ray_count());
        let mut stats = BuildStats::default();
        let mut hits = Vec::new();
        let mut spans: Vec<Span> = Vec::new();

        let ray_count = u32::try_from(lattice.ray_count()).unwrap_or(u32::MAX);
        // Ascending ray index, and this is contract rather than convenience.
        // See the module header.
        for ray_index in 0..ray_count {
            let ray = lattice.ray_at(ray_index);
            let query = Ray {
                origin: ray.origin,
                direction: ray.direction,
            };
            let ray_stats = bvh
                .intersect_ray_all_into(placed.as_ref(), &query, &mut hits)
                .map_err(|source| BuildError::Ray {
                    ray: ray_index,
                    origin: ray.origin.to_array(),
                    source,
                })?;

            if ray_stats.coplanar_rejected > 0 {
                return Err(BuildError::Coplanar {
                    ray: ray_index,
                    origin: ray.origin.to_array(),
                    rejected: ray_stats.coplanar_rejected,
                });
            }
            if !hits.len().is_multiple_of(2) {
                return Err(BuildError::OddCrossings {
                    ray: ray_index,
                    origin: ray.origin.to_array(),
                    crossings: hits.len(),
                });
            }
            stats.predicates.merge(&ray_stats);
            stats.rays += 1;

            // Parity pairing. The hits arrive sorted by `t` with ties broken by
            // triangle index, so consecutive pairs bound material: in, out, in,
            // out. Touching the surface tangentially contributes a zero-length
            // span, which `push_merge` folds away.
            spans.clear();
            for pair in hits.chunks_exact(2) {
                let span = Span::ordered(pair[0].t, pair[1].t);
                match spans.last_mut() {
                    // Adjacent or overlapping spans are merged as they are built,
                    // which keeps the arena's contents normalised without a
                    // second pass. The comparison is `>=` on the raw values
                    // rather than a tolerance: two spans that share an endpoint
                    // exactly are one span, and two that merely come close are
                    // two, because inventing a tolerance here would quietly
                    // erase thin walls.
                    Some(previous) if span.t0 <= previous.t1 => {
                        previous.t1 = previous.t1.max(span.t1);
                    }
                    _ => spans.push(span),
                }
            }

            if spans.is_empty() {
                stats.empty_rays += 1;
            } else {
                arena.set(ray_index, &spans);
                stats.spans += spans.len() as u64;
            }
        }
        stats.spilled_rays = arena.spilled_rays() as u64;

        Ok((
            Self {
                lattice,
                arena,
                placement: options.placement,
            },
            stats,
        ))
    }

    /// Where the rays are.
    #[inline]
    #[must_use]
    pub const fn lattice(&self) -> &Lattice {
        &self.lattice
    }

    /// What is on them.
    #[inline]
    #[must_use]
    pub const fn arena(&self) -> &Arena {
        &self.arena
    }

    /// Mutable access, for Unit 7's subtraction.
    #[inline]
    pub const fn arena_mut(&mut self) -> &mut Arena {
        &mut self.arena
    }

    /// The placement the stock was built under.
    #[inline]
    #[must_use]
    pub const fn placement(&self) -> Mat4 {
        self.placement
    }

    /// Material volume, in cubic millimetres.
    ///
    /// Each ray contributes the total length of its spans times the cell area.
    /// **Summed in ascending ray index**, because floating-point addition is not
    /// associative and a different order is a different answer.
    ///
    /// This is a Riemann sum over the transverse plane: exact along the ray,
    /// first-order in the cell size across it. The measured convergence is
    /// superlinear in `h/R` for every solid tested — see the convergence table.
    #[must_use]
    pub fn volume(&self) -> f64 {
        let mut total = 0.0;
        let rays = u32::try_from(self.arena.rays()).unwrap_or(u32::MAX);
        for ray in 0..rays {
            for span in self.arena.get(ray) {
                total += span.length();
            }
        }
        total * self.lattice.cell_area()
    }

    /// Rays carrying material.
    #[must_use]
    pub fn filled_rays(&self) -> usize {
        self.arena.filled_rays()
    }

    /// Total spans.
    #[must_use]
    pub fn total_spans(&self) -> usize {
        self.arena.total_spans()
    }

    /// The material's bounding box, or [`Aabb3::EMPTY`] if there is none.
    ///
    /// Derived from the spans rather than from the lattice, so it shrinks as
    /// material is removed.
    #[must_use]
    pub fn material_bounds(&self) -> Aabb3 {
        let [u, v, w] = self.lattice.axis().cyclic();
        let mut bounds = Aabb3::EMPTY;
        let rays = u32::try_from(self.arena.rays()).unwrap_or(u32::MAX);
        for ray in 0..rays {
            let spans = self.arena.get(ray);
            let (Some(first), Some(last)) = (spans.first(), spans.last()) else {
                continue;
            };
            let (i, j) = self.lattice.coords(ray);
            let origin = self.lattice.origin_of(i, j).to_array();
            let half = 0.5 * self.lattice.spacing();
            let mut lo = [0.0; 3];
            let mut hi = [0.0; 3];
            lo[u] = origin[u] - half;
            hi[u] = origin[u] + half;
            lo[v] = origin[v] - half;
            hi[v] = origin[v] + half;
            lo[w] = origin[w] + first.t0;
            hi[w] = origin[w] + last.t1;
            bounds = bounds.union(&Aabb3::from_min_max(
                Vec3::from_array(lo),
                Vec3::from_array(hi),
            ));
        }
        bounds
    }
}

impl Hashable for DexelField {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("DexelField");
        h.add(&self.lattice);
        for row in &self.placement.m {
            h.f64_slice(row);
        }
        h.add(&self.arena);
        h.end();
    }
}

/// Applies the placement transform to every vertex.
///
/// The triangle indices are untouched. A transform with negative determinant
/// mirrors the stock, which would invert every face's orientation, so the winding
/// is reversed to keep outward normals outward. Without that, every ray would
/// report leaving before entering and the field would be the complement of the
/// stock.
fn place<'a>(mesh: &'a TriMesh, transform: &Mat4) -> Cow<'a, TriMesh> {
    // Borrowed on the identity, which is the common case: a full mesh copy for a
    // transform that changes nothing would be the largest allocation in the
    // build for no effect.
    if *transform == Mat4::IDENTITY {
        return Cow::Borrowed(mesh);
    }
    let vertices: Vec<Vec3> = mesh
        .vertices()
        .iter()
        .map(|v| transform.transform_point(*v))
        .collect();
    let triangles: Vec<[u32; 3]> = if transform.determinant() < 0.0 {
        mesh.triangles()
            .iter()
            .map(|t| [t[0], t[2], t[1]])
            .collect()
    } else {
        mesh.triangles().to_vec()
    };
    Cow::Owned(
        TriMesh::new(vertices, triangles, mesh.meta().clone())
            .unwrap_or_else(|_| unreachable!("a transform changes no index")),
    )
}

/// Grows a box by `margin` on every side.
fn grow(bounds: Aabb3, margin: f64) -> Aabb3 {
    if margin <= 0.0 || bounds.is_empty() {
        return bounds;
    }
    let d = Vec3::new(margin, margin, margin);
    Aabb3::from_min_max(bounds.min - d, bounds.max + d)
}
