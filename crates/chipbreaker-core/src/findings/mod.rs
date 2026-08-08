// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Turning a field of deviations into a list somebody can act on.
//!
//! # What changes at this layer
//!
//! Everything below this module computes quantities with right answers. A span
//! endpoint is where it is; a deviation is a distance; both can be checked
//! against arithmetic, and the whole engine is built so that they can be.
//!
//! **A finding is a judgement.** Whether forty adjacent samples are one problem
//! or forty, whether 0.03 mm is worth a machinist's afternoon, whether excess
//! stock on a roughing pass is a defect — none of these has an oracle. The
//! injected-defect corpus constrains *detection*, and says nothing about
//! presentation.
//!
//! So the standard shifts, and it is worth being explicit about what replaces
//! correctness:
//!
//! - **Reproducibility.** The same field produces the same findings, in the same
//!   order, with the same identities, on every target.
//! - **Declared rules.** The radius and tolerance that produced a grouping are
//!   part of the report, because a cluster without them is not interpretable.
//! - **Separated quantities.** Depth and area are reported apart, and the two
//!   signs are never blended, because collapsing either loses information the
//!   reader needs and cannot recover.
//!
//! # Identity, and why it is derived from the finding rather than counted
//!
//! A finding's `id` is a hash of what it *is*: its class and where it sits,
//! quantised. Never a counter.
//!
//! Counters make reports undiffable. Insert one new finding near the top of a
//! part and every later finding renumbers, so a diff of two reports shows
//! everything changed and the reader learns nothing. Content-derived identity
//! makes the diff exact: a finding that did not move keeps its name.
//!
//! **Severity is deliberately not an input.** If depth were hashed, a gouge that
//! deepened from 1.0 to 1.2 mm would appear as one finding disappearing and a
//! different one arriving, when what happened is that a known problem got worse.
//! Excluding it is what lets a diff say "changed severity" at all.
//!
//! The cost is a boundary case, and it is stated rather than hidden: a finding
//! whose centroid drifts across a quantisation boundary takes a new identity,
//! and a diff will show it as one finding gone and another arrived. The
//! quantisation is coarse — the cluster radius — so this needs real movement
//! rather than rounding, but it is a limitation and not a subtlety.

pub mod attribute;
pub mod cluster;
pub mod diff;
pub mod report;

pub use attribute::{Attribution, attribute_point, motion_reaches};
pub use cluster::{Classification, Cluster, ClusterParams, cluster, unsampled};
pub use diff::{Change, Diff};
pub use report::{Manifest, NumericalSemantics, Report};

use crate::golden::CanonicalHash;
use crate::math::{Aabb3, Vec3};

/// One reported problem, with its identity, its size and its cause.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// Content-derived identity: sixteen hex characters, stable across runs and
    /// across unrelated changes to other findings. See the module header.
    pub id: String,
    /// What kind of problem this is.
    pub class: Classification,
    /// **Exact.** Deepest departure from the nominal, as a positive depth.
    pub worst_depth_mm: f64,
    /// **Exact.** Mean depth over the finding's samples.
    pub mean_depth_mm: f64,
    /// **Estimated.** Surface area affected. See [`cluster`] for the estimator.
    pub area_mm2: f64,
    /// **Estimated.** Volume of material involved, area weighted by depth.
    pub volume_mm3: f64,
    /// **Exact.** How many samples the finding was built from. Zero for classes
    /// derived from the nominal rather than from the deviation field.
    pub sample_count: usize,
    /// Centroid of the finding.
    pub at: Vec3,
    /// Axis-aligned bounds.
    pub bounds: Aabb3,
    /// Which segments could have caused it.
    pub attribution: Attribution,
}

impl Finding {
    /// Whether this finding is a defect on its own, without further context.
    #[must_use]
    pub const fn is_defect(&self) -> bool {
        self.class.is_defect()
    }
}

/// The identity of a finding at a place, of a class.
///
/// Quantised to `grid_mm` so that a finding which merely grows or deepens keeps
/// its name, and only one that genuinely moves gets a new one. `disambiguator`
/// separates two findings of the same class that quantise to the same cell,
/// and is the finding's index within that cell in canonical order — a property
/// of the sorted list rather than of the traversal that built it.
#[must_use]
pub fn finding_id(class: Classification, at: Vec3, grid_mm: f64, disambiguator: u32) -> String {
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
    h.begin("Finding");
    h.str(class.as_str());
    // The cell, as integers. Hashing the raw centroid would make the identity
    // move whenever any sample did, which is the behaviour this exists to avoid.
    h.u64(q(at.x) as u64);
    h.u64(q(at.y) as u64);
    h.u64(q(at.z) as u64);
    h.u64(u64::from(disambiguator));
    h.end();
    h.finish().to_hex()[..16].to_owned()
}

/// Assembles findings from clusters, giving each an identity.
///
/// The clusters must already be in canonical order; [`cluster::sort_canonically`]
/// is what puts them there, and the disambiguator depends on it.
#[must_use]
pub fn identify(clusters: Vec<Cluster>, grid_mm: f64) -> Vec<Finding> {
    use std::collections::BTreeMap;
    let mut seen: BTreeMap<(Classification, i64, i64, i64), u32> = BTreeMap::new();
    let mut out = Vec::with_capacity(clusters.len());
    for c in clusters {
        let q = |v: f64| {
            let f = (v / grid_mm).floor();
            #[allow(clippy::cast_possible_truncation, reason = "saturating")]
            {
                f as i64
            }
        };
        let key = (c.class, q(c.at.x), q(c.at.y), q(c.at.z));
        let slot = seen.entry(key).or_insert(0);
        let id = finding_id(c.class, c.at, grid_mm, *slot);
        *slot += 1;
        out.push(Finding {
            id,
            class: c.class,
            worst_depth_mm: c.worst_depth_mm,
            mean_depth_mm: c.mean_depth_mm,
            area_mm2: c.area_mm2,
            volume_mm3: c.volume_mm3,
            sample_count: c.samples.len(),
            at: c.at,
            bounds: c.bounds,
            attribution: Attribution::none(),
        });
    }
    out
}

/// Counts by class, for a report's summary.
///
/// Returned as a fixed-length array in [`Classification::all`] order rather than
/// a map, so that a class with no findings appears as a zero rather than as an
/// absent key. A reader scanning for "how many gouges" should find the answer in
/// both cases.
#[must_use]
pub fn counts(findings: &[Finding]) -> [usize; 4] {
    let mut out = [0usize; 4];
    for f in findings {
        for (i, c) in Classification::all().iter().enumerate() {
            if f.class == *c {
                out[i] += 1;
            }
        }
    }
    out
}
