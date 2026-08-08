// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The dexel field: one bundle of parallel rays, each carrying the intervals of
//! material along it.
//!
//! This is where the four foundations compose. The interval algebra
//! stores what is on a ray, the leak-free caster finds where the boundary
//! is, the tool solids subtract from it during a sweep, and the motion
//! stream says where the tool goes.
//!
//! **This module builds and stores a field. It does not cut.** Material removal
//! lives in `sweep`, and the second and third bundles in `dexel::tri`.
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
