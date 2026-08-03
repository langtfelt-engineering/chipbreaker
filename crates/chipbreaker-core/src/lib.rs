// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

#![forbid(unsafe_code)]
#![doc = include_str!("../README.md")]

pub mod eps;
pub mod golden;
pub mod math;
pub mod mesh;
pub mod predicates;
pub mod roots;
pub mod selftest;
pub mod spans;
pub mod tool;
pub mod toolpath;
pub mod transcendental;

pub use math::{Aabb3, Mat3, Mat4, Ray, Vec2, Vec3};
pub use predicates::{Orientation, Predicates};
pub use spans::{Span, Spans};

/// Version of this crate, as reported by `chipbreaker version`.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Version of the canonical hash encoding.
///
/// Any change to how a primitive is fed into [`golden::CanonicalHash`] — a new
/// type tag, a different length prefix, a different NaN canonicalization —
/// changes every golden hash in the repository. Bumping this constant is the
/// signal that a golden-file churn is intentional rather than a regression.
pub const CANONICAL_ENCODING_VERSION: u32 = 1;
