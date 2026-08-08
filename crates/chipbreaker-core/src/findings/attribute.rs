// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Which line of the NC program caused this finding.
//!
//! # The feature that turns a number into an action
//!
//! "There is a 1.4 mm gouge at (21.0, 26.0, 25.0)" tells a machinist that
//! something is wrong. "Line 47, the plunge of the `G81` on that line, cut
//! 1.4 mm too deep" tells them what to change. The second is the product; the
//! first is a measurement that has not finished its job.
//!
//! # Why this is not a lookup
//!
//! **The field records the final state, not the history.** A point on the cut
//! surface was last touched by some segment, but nothing in the field remembers
//! which — spans carry positions and normals, not authorship. Several segments
//! may have passed through the same place, and the surface that survives belongs
//! to whichever removed the most.
//!
//! Two ways to recover it, and the choice is a real one:
//!
//! **Store it.** Four bytes of segment index per span endpoint, written during
//! the cut. Exact, no reconstruction, and the answer is simply read off. It
//! costs eight bytes per span on top of the twenty-four a span already occupies
//! — a third more memory for every field the engine ever builds, spent entirely
//! on the rare regions that turn into findings.
//!
//! **Recompute it.** For each finding, and only for findings, ask every motion
//! whether its swept volume contains that point. The last one that does is the
//! author. Costs nothing in steady state, and findings are rare.
//!
//! The second is implemented, and the arithmetic behind the choice is in
//! `examples/attribution_cost.rs` rather than asserted here.
//!
//! # Ambiguity is reported, never resolved by guessing
//!
//! A point on a surface cut by a finishing pass may lie on the boundary of the
//! roughing pass that preceded it, to within any tolerance worth using. Both
//! motions genuinely reach it.
//!
//! When that happens the finding names **both**. A confident wrong attribution
//! is worse than an honest set: it sends somebody to edit a line that was not
//! the problem, and when the edit does not help, it costs them their trust in
//! every other line the tool has named.

use crate::math::{Ray, Vec3};
use crate::sweep::Motion;
use crate::sweep::cut::{CutScratch, SweepMethod};
use crate::tool::Profile;
use crate::toolpath::Provenance;

/// How close to a swept surface a point may sit and still count as on it.
///
/// A finding's position is a span endpoint, which is an exact root of the swept
/// surface it lies on — so the tolerance is not there to absorb error in the
/// point. It is there because the *same* point often lies within rounding of a
/// second motion's surface as well, and the honest answer then is both.
const ON_SURFACE_MM: f64 = 1.0e-6;

/// Which segments could have produced a finding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Attribution {
    /// Indices into the motion list, ascending. Empty when nothing claimed the
    /// point.
    pub segments: Vec<u32>,
    /// The provenance of each, in the same order.
    pub provenance: Vec<Provenance>,
    /// Which setup the naming segments belong to.
    ///
    /// **A line number alone is ambiguous across a job.** Two setups have two
    /// programs, each numbering its own lines from one, so "line 47" names two
    /// different moves unless the setup is beside it.
    ///
    /// Zero for a single-setup run, which is what keeps those reports the shape
    /// they have always been: the field is present but says the only thing it
    /// could say.
    pub setup: u32,
}

impl Attribution {
    /// Nothing claimed this point.
    #[must_use]
    pub fn none() -> Self {
        Self {
            segments: Vec::new(),
            provenance: Vec::new(),
            setup: 0,
        }
    }

    /// True when more than one segment could have caused this.
    #[must_use]
    pub fn is_ambiguous(&self) -> bool {
        self.segments.len() > 1
    }

    /// True when nothing could be attributed at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }
}

/// Whether a motion's swept volume reaches `p`.
///
/// Answered by casting a ray through the point and asking the sweep for its
/// spans — the same dispatcher a cut uses, so a motion is judged by exactly the
/// code that performed it. A separate containment test would be a second
/// implementation of the swept volume, and the two would eventually disagree.
#[must_use]
pub fn motion_reaches(
    profile: &Profile,
    motion: &Motion,
    method: SweepMethod,
    scratch: &mut CutScratch,
    p: Vec3,
) -> bool {
    // Along +Z, so the parameter is the height and the arithmetic is a
    // subtraction. The axis is arbitrary; any would do.
    let origin = Vec3::new(p.x, p.y, p.z - 1.0);
    let ray = Ray {
        origin,
        direction: Vec3::new(0.0, 0.0, 1.0),
    };
    let Some(spans) = crate::sweep::cut::swept_spans_for(profile, motion, method, scratch, &ray)
    else {
        return false;
    };
    let t = 1.0;
    spans
        .iter()
        .any(|s| t >= s.t0 - ON_SURFACE_MM && t <= s.t1 + ON_SURFACE_MM)
}

/// Attributes a whole finding, by probing several of its points and unioning.
///
/// A finding covers a region and different parts of that region can lie on
/// different segments' surfaces, so one point is not enough — see
/// [`crate::findings::Cluster::probes`] for the case that showed it.
#[must_use]
pub fn attribute_finding(
    profile: &Profile,
    motions: &[Motion],
    bounds: &[crate::math::Aabb3],
    provenance: &[Provenance],
    method: SweepMethod,
    scratch: &mut CutScratch,
    probes: &[Vec3],
) -> Attribution {
    let mut segments: Vec<u32> = Vec::new();
    for p in probes {
        let a = attribute_point(profile, motions, bounds, provenance, method, scratch, *p);
        for s in a.segments {
            if !segments.contains(&s) {
                segments.push(s);
            }
        }
    }
    // Ascending, so the set is a property of the finding rather than of the
    // order the probes happened to be visited in.
    segments.sort_unstable();
    let prov = segments
        .iter()
        .map(|&i| {
            provenance
                .get(i as usize)
                .copied()
                .unwrap_or_else(|| Provenance::new(0, 0, 0))
        })
        .collect();
    Attribution {
        segments,
        provenance: prov,
        setup: 0,
    }
}

/// Attributes one point to the segments whose swept volumes reach it.
///
/// `bounds` is the swept bounding box of each motion, precomputed by the caller
/// so that the box rejection — which discards the overwhelming majority of a
/// real program's segments — costs one comparison rather than a sweep.
#[must_use]
pub fn attribute_point(
    profile: &Profile,
    motions: &[Motion],
    bounds: &[crate::math::Aabb3],
    provenance: &[Provenance],
    method: SweepMethod,
    scratch: &mut CutScratch,
    p: Vec3,
) -> Attribution {
    let mut segments = Vec::new();
    for (i, motion) in motions.iter().enumerate() {
        // The box is padded because a point *on* the swept surface is on the
        // boundary of the box too, and a strict test would reject exactly the
        // points this function exists to attribute.
        if !bounds[i].expand(ON_SURFACE_MM).contains(p) {
            continue;
        }
        if motion_reaches(profile, motion, method, scratch, p) {
            segments.push(u32::try_from(i).unwrap_or(u32::MAX));
        }
    }
    let prov = segments
        .iter()
        .map(|&i| {
            provenance
                .get(i as usize)
                .copied()
                .unwrap_or_else(|| Provenance::new(0, 0, 0))
        })
        .collect();
    Attribution {
        segments,
        provenance: prov,
        setup: 0,
    }
}
