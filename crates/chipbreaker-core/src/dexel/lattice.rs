// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Where the rays are.
//!
//! # Ray origins sit at cell centres. This is not a tuning parameter.
//!
//! From ADR 0001 Part 2, and one decision buys both halves:
//!
//! **Performance.** 4,096 rays against a 5,292-triangle lattice block: 2.52 ms
//! with origins offset to cell centres, 39.83 ms with them on the integer
//! lattice. Nearly 16x, on the innermost loop of the entire product.
//!
//! **Correctness.** A ray whose origin shares a coordinate with a mesh vertex is
//! coplanar with every edge at that vertex, which drives Unit 2's caster onto
//! the Simulation-of-Simplicity cascade — and, where a whole *face* aligns with
//! the ray, into a coplanar rejection that construction treats as a hard error.
//! The half-cell offset makes that unreachable for axis-aligned stock, which is
//! most stock.
//!
//! `origins_are_never_on_the_integer_lattice` enforces it. Somebody will
//! eventually simplify `min + (i + 0.5) * spacing` to `min + i * spacing`,
//! because the second is obviously tidier and the reason the first exists is two
//! documents away. The test is what stops them.
//!
//! # Anisotropy, and why Unit 6 exists
//!
//! A single-axis field is anisotropic **by construction**, and this is worth
//! stating here rather than discovering at U6.
//!
//! *Along* the ray, intersections come from Unit 2's exact-predicate caster and
//! are analytic: a Z-bundle captures a horizontal surface to machine precision,
//! however coarse the spacing. *Transverse* to the ray the shape is sampled on
//! this lattice, so a vertical wall is captured only to within a cell.
//!
//! The consequence is that accuracy depends on the **ratio of feature size to
//! cell size**, not on cell size alone. Measured: a sphere at `h/R = 1/100` has
//! a relative volume error of 0.00236% at R = 2.5 mm, at R = 5 mm, at R = 10 mm
//! and at R = 20 mm — identical to five significant figures.
//!
//! And the fix for a poorly captured vertical wall is **not finer spacing**. It
//! is another bundle along another axis, which is the whole of Unit 6.

use crate::golden::{CanonicalHash, Hashable};
use crate::math::{Aabb3, Axis, Vec3};

/// Why a lattice could not be built.
#[derive(Debug, Clone, PartialEq)]
pub enum LatticeError {
    /// The spacing must be a positive, finite length.
    BadSpacing {
        /// What was supplied.
        found: f64,
    },
    /// The bounds are empty or not finite.
    BadBounds {
        /// What was supplied.
        found: Aabb3,
    },
    /// The lattice would hold more rays than a `u32` index can address.
    TooManyRays {
        /// How many it would have needed.
        wanted: u64,
        /// Counts along each lattice axis.
        counts: [u64; 2],
    },
}

impl core::fmt::Display for LatticeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BadSpacing { found } => {
                write!(f, "spacing must be a positive finite length, got {found}")
            }
            Self::BadBounds { found } => write!(
                f,
                "the workspace bounds are empty or not finite: {:?} .. {:?}",
                found.min.to_array(),
                found.max.to_array()
            ),
            Self::TooManyRays { wanted, counts } => write!(
                f,
                "a {} x {} lattice needs {wanted} rays, more than a u32 index can \
                 address. Use a coarser spacing, or a smaller workspace",
                counts[0], counts[1]
            ),
        }
    }
}

impl core::error::Error for LatticeError {}

/// A bundle of parallel rays on a regular grid.
#[derive(Debug, Clone, PartialEq)]
pub struct Lattice {
    axis: Axis,
    /// Lower corner of the workspace the lattice covers.
    origin: Vec3,
    spacing: f64,
    /// Ray counts along the two lattice axes, in `axis.cyclic()` order.
    counts: [u32; 2],
    /// Extent along the ray axis, so a ray knows where to start and stop.
    length: f64,
}

impl Lattice {
    /// Builds a lattice covering `bounds` at `spacing`.
    ///
    /// The counts are chosen so the cell centres span the bounds: a workspace
    /// 100 mm wide at 0.5 mm spacing gets 200 rays, the first at 0.25 and the
    /// last at 99.75.
    ///
    /// # Errors
    /// See [`LatticeError`].
    pub fn new(bounds: Aabb3, spacing: f64, axis: Axis) -> Result<Self, LatticeError> {
        if !spacing.is_finite() || spacing <= 0.0 {
            return Err(LatticeError::BadSpacing { found: spacing });
        }
        if bounds.is_empty() || !bounds.is_finite() {
            return Err(LatticeError::BadBounds { found: bounds });
        }

        let extent = bounds.extent().to_array();
        let [u, v, w] = axis.cyclic();
        let count = |e: f64| -> u64 {
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "the value is finite and positive; the range is checked by the caller"
            )]
            let n = (e / spacing).ceil() as u64;
            n.max(1)
        };
        let counts = [count(extent[u]), count(extent[v])];
        let wanted = counts[0].saturating_mul(counts[1]);
        if wanted > u64::from(u32::MAX) {
            return Err(LatticeError::TooManyRays { wanted, counts });
        }

        Ok(Self {
            axis,
            origin: bounds.min,
            spacing,
            counts: [
                u32::try_from(counts[0]).unwrap_or(u32::MAX),
                u32::try_from(counts[1]).unwrap_or(u32::MAX),
            ],
            length: extent[w],
        })
    }

    /// Which axis the rays run along.
    #[inline]
    #[must_use]
    pub const fn axis(&self) -> Axis {
        self.axis
    }

    /// Lower corner of the workspace.
    #[inline]
    #[must_use]
    pub const fn origin(&self) -> Vec3 {
        self.origin
    }

    /// Cell size.
    #[inline]
    #[must_use]
    pub const fn spacing(&self) -> f64 {
        self.spacing
    }

    /// Ray counts along the two lattice axes.
    #[inline]
    #[must_use]
    pub const fn counts(&self) -> [u32; 2] {
        self.counts
    }

    /// Total rays.
    #[inline]
    #[must_use]
    pub fn ray_count(&self) -> usize {
        self.counts[0] as usize * self.counts[1] as usize
    }

    /// Cross-sectional area one ray represents, in square millimetres.
    ///
    /// The weight in the volume sum: each ray contributes its span measure times
    /// this.
    #[inline]
    #[must_use]
    pub fn cell_area(&self) -> f64 {
        self.spacing * self.spacing
    }

    /// Ray index from lattice coordinates, row-major in the first axis.
    ///
    /// The traversal order is part of the contract, because the volume sum runs
    /// in ascending ray index and floating-point addition is not associative.
    #[inline]
    #[must_use]
    pub const fn index(&self, i: u32, j: u32) -> u32 {
        i * self.counts[1] + j
    }

    /// Lattice coordinates from a ray index.
    #[inline]
    #[must_use]
    pub const fn coords(&self, ray: u32) -> (u32, u32) {
        (ray / self.counts[1], ray % self.counts[1])
    }

    /// Where ray `(i, j)` starts.
    ///
    /// **`+ 0.5` is load-bearing.** See the module header, and
    /// `origins_are_never_on_the_integer_lattice`, which fails if it is removed.
    #[must_use]
    pub fn origin_of(&self, i: u32, j: u32) -> Vec3 {
        let [u, v, w] = self.axis.cyclic();
        let mut point = self.origin.to_array();
        point[u] += (f64::from(i) + 0.5) * self.spacing;
        point[v] += (f64::from(j) + 0.5) * self.spacing;
        // Start behind the workspace, so a surface exactly on the lower bound is
        // still crossed rather than begun upon.
        point[w] -= self.spacing;
        Vec3::from_array(point)
    }

    /// How far a ray must travel to clear the workspace.
    #[inline]
    #[must_use]
    pub fn ray_length(&self) -> f64 {
        self.length + 2.0 * self.spacing
    }

    /// The ray at a given index.
    #[must_use]
    pub fn ray_at(&self, ray: u32) -> crate::math::Ray {
        let (i, j) = self.coords(ray);
        crate::math::Ray {
            origin: self.origin_of(i, j),
            direction: self.axis.direction(),
        }
    }

    /// The box the lattice's cell centres cover.
    #[must_use]
    pub fn covered_bounds(&self) -> Aabb3 {
        let [u, v, w] = self.axis.cyclic();
        let mut lo = self.origin.to_array();
        let mut hi = self.origin.to_array();
        lo[u] += 0.5 * self.spacing;
        lo[v] += 0.5 * self.spacing;
        hi[u] += (f64::from(self.counts[0]) - 0.5) * self.spacing;
        hi[v] += (f64::from(self.counts[1]) - 0.5) * self.spacing;
        hi[w] += self.length;
        Aabb3::from_min_max(Vec3::from_array(lo), Vec3::from_array(hi))
    }
}

impl Hashable for Lattice {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Lattice");
        h.add(&self.axis);
        h.f64_slice(&self.origin.to_array());
        h.f64(self.spacing);
        h.u64(u64::from(self.counts[0]));
        h.u64(u64::from(self.counts[1]));
        h.f64(self.length);
        h.end();
    }
}
