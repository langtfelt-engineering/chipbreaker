// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Collisions: the tool's non-cutting geometry driving into something solid.
//!
//! # Why this is not a [`Finding`](super::Finding)
//!
//! A finding and a collision are both "something is wrong here", and that is
//! where the resemblance ends.
//!
//! A finding's severity is a **depth into the nominal surface**, with an area
//! and a volume measured over that surface. A collision's is **penetration of a
//! non-cutting element into an obstacle**: there is no nominal surface involved,
//! area and volume over one do not exist, and `worst_depth_mm` would be the same
//! field name carrying a different physical quantity. The stability contract
//! forbids exactly that, so collisions live in their own array with their own
//! shape.
//!
//! The two also come from different places. A finding is derived from the
//! deviation field and is governed by its detection floor; a collision is a
//! property of the **trajectory**, computed as the program is replayed, and the
//! deviation field never enters into it.
//!
//! # Penetration and clearance are separate quantities
//!
//! A collision reports how far in; a near miss reports how far off. They are not
//! two signs of one number, and [`Contact`] keeps them apart for the same reason
//! a finding refuses to blend depth with area: one number cannot say which of
//! two different situations this is, and a reader cannot recover what was lost.
//!
//! # What still transfers from a finding
//!
//! Identity is content-derived and quantised, so a collision that deepens keeps
//! its name and a diff can say "worse" rather than "different". Attribution
//! reuses the same machinery: a collision is caused by a segment, and naming the
//! NC line is the same problem solved the same way.

use crate::golden::CanonicalHash;
use crate::math::{Aabb3, Vec3};
use crate::tool::profile::ElementRole;
use crate::toolpath::MotionKind;

use super::Attribution;

/// What the tool hit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Obstacle {
    /// Material not yet removed, in the stock field as it stood at that moment.
    Stock,
    /// A static obstacle: a clamp, a vise, a tombstone, the table.
    Fixture {
        /// Which fixture, by load order, so the report is stable if two share a
        /// name.
        index: u32,
        /// What it was called on the command line, for the reader.
        name: String,
    },
}

impl Obstacle {
    /// The short name used in the report.
    #[must_use]
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Stock => "stock",
            Self::Fixture { .. } => "fixture",
        }
    }

    /// Sort key: stock first, then fixtures in load order.
    #[must_use]
    pub const fn order(&self) -> (u8, u32) {
        match self {
            Self::Stock => (0, 0),
            Self::Fixture { index, .. } => (1, *index),
        }
    }
}

/// Contact, or the absence of it by less than the configured margin.
///
/// Two variants rather than one signed number. A `-0.2` that means "cleared by
/// 0.2 mm" and a `0.2` that means "buried by 0.2 mm" are different enough that a
/// consumer sorting by magnitude would rank a safe pass alongside a crash.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Contact {
    /// The element entered the obstacle. **Always a defect.**
    Collision {
        /// How far in, at the deepest point. Strictly positive.
        penetration_mm: f64,
    },
    /// A **cutting** element entered a fixture. **Always a defect.**
    ///
    /// Kept distinct rather than folded into [`Self::Collision`], because the
    /// two are different mistakes with different fixes. A shank rubbing a wall
    /// means the tool is too short for the pocket; a flute in a clamp means the
    /// program is machining the wrong thing entirely, and the toolpath is wrong
    /// rather than the tooling.
    ///
    /// Widening "non-cutting contact" to cover it would have made that field
    /// mean "anything hit anything", and a reader could no longer tell which of
    /// the two they had without going back to the element role.
    CutterIntoFixture {
        /// How far in, at the deepest point. Strictly positive.
        penetration_mm: f64,
    },
    /// The element passed within the clearance threshold without touching.
    ///
    /// Not a defect, and reported anyway: it names the thing that will collide
    /// after a small edit, which is often more useful than the crash itself.
    NearMiss {
        /// Closest approach. Non-negative, and below the configured threshold.
        clearance_mm: f64,
    },
}

impl Contact {
    /// Whether this condemns the program.
    #[must_use]
    pub const fn is_collision(self) -> bool {
        matches!(
            self,
            Self::Collision { .. } | Self::CutterIntoFixture { .. }
        )
    }

    /// The name used in the report.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Collision { .. } => "collision",
            Self::CutterIntoFixture { .. } => "cutter-into-fixture",
            Self::NearMiss { .. } => "near-miss",
        }
    }

    /// The magnitude, whichever quantity this is. For sorting only — never
    /// report it without the variant beside it.
    #[must_use]
    pub const fn magnitude(self) -> f64 {
        match self {
            Self::Collision { penetration_mm } | Self::CutterIntoFixture { penetration_mm } => {
                penetration_mm
            }
            Self::NearMiss { clearance_mm } => clearance_mm,
        }
    }
}

/// One collision or near miss.
#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    /// Content-derived identity, sixteen hex characters. See [`collision_id`].
    pub id: String,
    /// Whether contact happened, and by how much.
    pub contact: Contact,
    /// Which part of the tool stack: the role, and the element's index in the
    /// profile. The role alone is not enough to find it in a two-stage holder.
    pub role: ElementRole,
    /// Index of the profile element, from the tip upward.
    pub element_index: u32,
    /// What it hit.
    pub obstacle: Obstacle,
    /// Where the worst point of the contact sits, in machine coordinates.
    pub at: Vec3,
    /// Bounds of the contact region.
    pub bounds: Aabb3,
    /// Whether the tool was rapiding.
    ///
    /// A rapid collision is categorically worse than a feed collision — full
    /// traverse rate into a clamp is what breaks spindles — and a consumer will
    /// want to sort by it.
    pub motion: MotionKind,
    /// Which NC lines could have caused it. Reuses a finding's machinery; expect
    /// `ambiguous` to be false, because a collision has one position on one
    /// segment.
    pub attribution: Attribution,
}

impl Collision {
    /// Whether this condemns the program. Near misses do not.
    #[must_use]
    pub const fn is_defect(&self) -> bool {
        self.contact.is_collision()
    }
}

/// The identity of a collision: what hit what, and roughly where.
///
/// Quantised like a finding's, and for the same reason — a collision that gets
/// deeper between two runs is the same collision, and a diff should say so.
/// **Penetration is deliberately not hashed**, exactly as severity is not hashed
/// for a finding.
///
/// The motion kind *is* hashed, because a rapid collision and a feed collision
/// at the same place are genuinely different problems with different fixes.
#[must_use]
pub fn collision_id(
    role: ElementRole,
    obstacle: &Obstacle,
    motion: MotionKind,
    at: Vec3,
    grid_mm: f64,
    disambiguator: u32,
) -> String {
    let q = |v: f64| {
        let c = (v / grid_mm).floor();
        #[allow(
            clippy::cast_possible_truncation,
            reason = "saturating, and a workspace is far inside i64"
        )]
        {
            c as i64
        }
    };
    let mut h = CanonicalHash::new();
    h.begin("Collision");
    h.str(role.as_str());
    h.str(obstacle.kind());
    let (class, index) = obstacle.order();
    h.u64(u64::from(class));
    h.u64(u64::from(index));
    h.str(motion.as_str());
    h.u64(q(at.x) as u64);
    h.u64(q(at.y) as u64);
    h.u64(q(at.z) as u64);
    h.u64(u64::from(disambiguator));
    h.end();
    h.finish().to_hex()[..16].to_owned()
}

/// Canonical order: worst first, then by a total order on content.
///
/// Collisions before near misses, rapids before feeds, deeper before shallower,
/// then position and identity. Every tie is broken by something derived from the
/// collision itself, never by the order they were found in — the same guarantee
/// a finding's clusters carry, and for the same reason.
pub fn sort_canonically(out: &mut [Collision]) {
    out.sort_by(|a, b| {
        b.contact
            .is_collision()
            .cmp(&a.contact.is_collision())
            .then_with(|| {
                // Rapid first among equals.
                a.motion
                    .is_cutting()
                    .cmp(&b.motion.is_cutting())
                    .then_with(|| b.contact.magnitude().total_cmp(&a.contact.magnitude()))
                    .then_with(|| a.obstacle.order().cmp(&b.obstacle.order()))
                    .then_with(|| a.at.x.total_cmp(&b.at.x))
                    .then_with(|| a.at.y.total_cmp(&b.at.y))
                    .then_with(|| a.at.z.total_cmp(&b.at.z))
                    .then_with(|| a.role.severity().cmp(&b.role.severity()))
                    .then_with(|| a.element_index.cmp(&b.element_index))
                    .then_with(|| a.id.cmp(&b.id))
            })
    });
}

/// How many of these are real collisions rather than near misses.
#[must_use]
pub fn collision_count(all: &[Collision]) -> usize {
    all.iter().filter(|c| c.is_defect()).count()
}
