// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Arc geometry in the IR, resolved to one unambiguous form.
//!
//! # One representation, whatever the program said
//!
//! G-code writes an arc two ways. `I`/`J`/`K` give the centre; `R` gives a
//! radius and leaves the centre to be derived, with the *sign* of `R` choosing
//! between the minor and major arc. Both are erased here: the IR stores a
//! centre, a plane, and a signed sweep, and the two input forms must produce
//! byte-identical output for the same arc. That equivalence is a required test,
//! because it is the cheapest way to catch a sign error in either path.
//!
//! # Why a signed sweep rather than a direction flag
//!
//! A sweep of `-PI/2` says everything a `Clockwise` flag plus a magnitude would,
//! and it composes: helices carry turns in the same number, and reversing an arc
//! is a negation rather than a match. The sweep integrates along it.

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec3;

/// The plane an arc is swept in, and therefore which axis pair it uses.
///
/// The plane also decides which words carry the centre — `G17` takes `I`,`J`;
/// `G18` takes `I`,`K`; `G19` takes `J`,`K` — and, notoriously, the sense of
/// `G2` versus `G3`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum ArcPlane {
    /// `G17`, the XY plane. Normal is `+Z`.
    #[default]
    Xy,
    /// `G18`, the ZX plane. **Normal is `+Y`, and the axis order is Z then X.**
    ///
    /// This is the one that catches people. The plane is conventionally named
    /// "XZ" but RS-274 orders it Z,X precisely so that the right-handed normal
    /// comes out as `+Y`; reading it as X,Z gives a normal of `-Y` and every
    /// `G2` becomes a `G3`. A corpus case pins the handedness against a
    /// known-good reference rather than against an argument.
    Zx,
    /// `G19`, the YZ plane. Normal is `+X`.
    Yz,
}

impl ArcPlane {
    /// Unit normal, right-handed, about which a positive sweep turns.
    #[must_use]
    pub const fn normal(self) -> Vec3 {
        match self {
            Self::Xy => Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            Self::Zx => Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Self::Yz => Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }

    /// The two in-plane axes, in the order RS-274 defines them, then the axis
    /// normal to the plane.
    ///
    /// `0` is X, `1` is Y, `2` is Z. The `Zx` row is `[2, 0, 1]` and not
    /// `[0, 2, 1]`; see the variant's own documentation.
    #[must_use]
    pub const fn axes(self) -> [usize; 3] {
        match self {
            Self::Xy => [0, 1, 2],
            Self::Zx => [2, 0, 1],
            Self::Yz => [1, 2, 0],
        }
    }

    /// The G-code that selects this plane.
    #[must_use]
    pub const fn as_gcode(self) -> &'static str {
        match self {
            Self::Xy => "G17",
            Self::Zx => "G18",
            Self::Yz => "G19",
        }
    }

    /// Name used in reports and in the IR.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Xy => "xy",
            Self::Zx => "zx",
            Self::Yz => "yz",
        }
    }
}

impl Hashable for ArcPlane {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// How the arc was written in the program, kept for provenance.
///
/// Not used geometrically — the two forms resolve to the same arc — but a
/// diagnostic that can say "the `R` form of this arc is ill-conditioned near
/// 180 degrees" is worth much more than one that cannot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ArcForm {
    /// Centre given by `I`/`J`/`K` offsets.
    CentreOffsets,
    /// Radius given by `R`, positive for the minor arc and negative for the
    /// major one.
    Radius,
}

impl ArcForm {
    /// Name used in reports and in the IR.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::CentreOffsets => "ijk",
            Self::Radius => "r",
        }
    }
}

impl Hashable for ArcForm {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// A resolved arc or helix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcData {
    /// Centre of curvature, in machine coordinates, millimetres.
    pub center: Vec3,
    /// Which plane the arc turns in.
    pub plane: ArcPlane,
    /// Signed sweep in radians, positive about [`ArcPlane::normal`].
    ///
    /// Carries the whole of the direction and the extent, including multiple
    /// turns of a helix: a two-turn clockwise helix has a sweep of `-4 PI`.
    pub sweep: f64,
    /// Radius, in millimetres. Derived, and stored because every consumer wants
    /// it and recomputing it invites two consumers to derive it differently.
    pub radius: f64,
    /// How the arc was written.
    pub form: ArcForm,
    /// Difference between the distance from the centre to the start and to the
    /// end, in millimetres, before recentring.
    ///
    /// CAM rounds coordinates, so this is rarely zero and is not an error below
    /// the tolerance. It is recorded because attributing a surface deviation
    /// needs to know whether the arc it came from was exact.
    pub radius_residual: f64,
}

impl ArcData {
    /// True if every field is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.center.is_finite()
            && self.sweep.is_finite()
            && self.radius.is_finite()
            && self.radius_residual.is_finite()
    }

    /// Number of complete turns, signed. Zero for an ordinary arc.
    #[must_use]
    pub fn turns(&self) -> i32 {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "a sweep large enough to overflow i32 turns is rejected upstream"
        )]
        {
            (self.sweep / core::f64::consts::TAU) as i32
        }
    }

    /// Arc length in the plane, ignoring any helical rise.
    #[must_use]
    pub fn planar_length(&self) -> f64 {
        self.radius * self.sweep.abs()
    }
}

impl Hashable for ArcData {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("ArcData");
        h.f64_slice(&self.center.to_array());
        h.add(&self.plane);
        h.f64(self.sweep);
        h.f64(self.radius);
        h.add(&self.form);
        h.f64(self.radius_residual);
        h.end();
    }
}
