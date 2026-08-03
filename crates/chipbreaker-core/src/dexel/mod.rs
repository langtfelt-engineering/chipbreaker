// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The dexel field: one bundle of parallel rays, each carrying the intervals of
//! material along it.
//!
//! This is where the four foundation units compose. Unit 1's interval algebra
//! stores what is on a ray, Unit 2's leak-free caster finds where the boundary
//! is, Unit 3's tool solids will subtract from it at U7, and Unit 4's motion
//! stream says where the tool goes.
//!
//! **Unit 5 builds and stores a field. It does not cut.** Material removal is
//! U7, and a second and third bundle are U6.
//!
//! # Anisotropy
//!
//! Stated in [`lattice`] and worth the pointer from here, because it is the
//! single most important property of the structure: a one-axis field is exact
//! *along* the ray and sampled *transverse* to it, so accuracy depends on the
//! ratio of feature size to cell size rather than on cell size alone — and the
//! fix for a poorly captured vertical wall is another bundle, not finer
//! spacing.

pub mod arena;
pub mod convergence;
pub mod deviation;
pub mod field;
pub mod io;
pub mod lattice;
pub mod tessellation;
pub mod tri;

pub use arena::{Arena, INLINE_CAPACITY};
pub use deviation::{DeviationReport, SurfacePoint};
pub use field::{BuildError, BuildOptions, BuildStats, DexelField};
pub use io::{FORMAT_VERSION, FieldFormat, FormatError, TDX_FORMAT_VERSION};
pub use lattice::{Lattice, LatticeError};
pub use tessellation::TessellationEstimate;
pub use tri::{AxisSet, Provenance, TriBuildOptions, TriBuildStats, TriDexelField};
