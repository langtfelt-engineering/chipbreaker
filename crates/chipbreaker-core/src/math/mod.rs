// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! `f64` linear algebra for Chipbreaker.
//!
//! # Why not `nalgebra` or `glam`?
//!
//! Both are excellent crates, and neither makes the guarantee we need. `glam`
//! is `f32`-first and selects SIMD paths by target feature, so the same source
//! produces different roundings on different machines. `nalgebra` is `f64`-clean
//! but its evaluation order is an implementation detail we do not control, and a
//! reassociation in a point release would silently invalidate every golden hash
//! in this repository. These types are a few hundred lines; owning them is
//! cheaper than auditing someone else's.
//!
//! # Determinism rules obeyed here
//!
//! - `f64` throughout, never `f32`.
//! - No `mul_add`. Every dot product and matrix multiply is written as explicit
//!   `a * b + c * d` so that native and WASM contract identically (which is to
//!   say, not at all).
//! - Fixed summation order. Dot products accumulate in ascending component
//!   order; determinants expand along a fixed row. Floating-point addition is
//!   not associative, so "the obvious order" has to be *the documented order*.
//! - `sqrt` is used freely: IEEE-754 requires it to be correctly rounded, so it
//!   is bit-identical everywhere. **Transcendental functions are not**, and this
//!   module deliberately exposes none. See `CONTRIBUTING.md`.
//!
//! # Layout
//!
//! Every type is `#[repr(C)]` and `Copy`, so it can cross the C ABI boundary
//! unchanged if 5-axis work ever adds one.

mod aabb3;
mod mat3;
mod mat4;
mod oct;
mod ray;
mod vec2;
mod vec3;

pub use aabb3::{Aabb3, Axis};
pub use mat3::Mat3;
pub use mat4::Mat4;
pub use oct::OctNormal;
pub use ray::Ray;
pub use vec2::Vec2;
pub use vec3::Vec3;
