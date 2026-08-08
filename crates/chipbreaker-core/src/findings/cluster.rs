// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Grouping a field of deviations into discrete findings.
//!
//! # Why this is the hard part of the unit
//!
//! Everything before this had a right answer. A span endpoint is where it is; a
//! deviation is a distance; both can be checked against arithmetic. **A finding
//! is a judgement.** Whether forty adjacent samples are one problem or forty is
//! not a fact about the part, it is a decision about how to present the part,
//! and no oracle settles it.
//!
//! So the discipline here is different. It is not "is this correct" but "is this
//! reproducible, and is the rule that produced it written down where the reader
//! can see it". Both are achievable; correctness is not on offer.
//!
//! # Order independence, and why union-find rather than accretion
//!
//! The obvious clustering is greedy: walk the samples, and for each, either join
//! a nearby cluster or start a new one. It is also wrong, and wrong in a way that
//! is hard to see. Consider three samples in a line, each within the radius of
//! its neighbour but not of the far one. Walking left to right gives one cluster
//! of three; walking from the middle outward gives one cluster of three; walking
//! the ends first gives **two** clusters that never merge. The answer depends on
//! the traversal.
//!
//! Union-find has no such freedom. The partition it produces is the set of
//! connected components of the adjacency relation, and connected components are
//! a property of the graph rather than of the walk. Any union order gives the
//! same partition. That is the whole reason to reach for it here.
//!
//! What order *can* still reach is anything derived from a cluster's
//! **representative** rather than from its members, so nothing here uses a
//! representative: every reported quantity is a fold over the whole set, taken
//! in a canonical order.
//!
//! # The grid, and why `BTreeMap`
//!
//! Adjacency by brute force is quadratic, and a real field has tens of thousands
//! of samples above threshold. The samples are bucketed into a uniform grid of
//! cells one radius across, so a sample need only be compared against the
//! twenty-seven cells around it.
//!
//! `BTreeMap` rather than `HashMap`, per the determinism rules — the iteration
//! order of the buckets reaches the union order, and while union order cannot
//! change the partition, it can change which of two equal candidates is examined
//! first, and this project does not leave that to a hasher's seed.

use std::collections::BTreeMap;

use crate::deviation::Deviation;
use crate::math::{Aabb3, Axis, Vec3};

/// How the samples of a finding were grouped, reported alongside the findings
/// because a cluster is only meaningful with the rule that made it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterParams {
    /// Two samples of the same class join a cluster when they are no further
    /// apart than this, in millimetres.
    pub radius_mm: f64,
    /// Samples closer to the nominal than this are not deviations at all.
    pub tolerance_mm: f64,
}

impl ClusterParams {
    /// Parameters scaled to a lattice.
    ///
    /// The radius is **two cells**, which is the smallest value that can join
    /// samples from different bundles: three bundles at the same spacing put
    /// their samples on three interleaved lattices, and a radius under one cell
    /// would split a single physical gouge into one finding per bundle. Two
    /// cells joins them without reaching across a gap a machinist would call
    /// two separate marks.
    #[must_use]
    pub fn for_spacing(spacing_mm: f64, tolerance_mm: f64) -> Self {
        Self {
            radius_mm: 2.0 * spacing_mm,
            tolerance_mm,
        }
    }
}

/// What kind of problem a finding is.
///
/// The four are **not** four severities of one thing. A gouge and excess stock
/// have opposite signs and opposite consequences, and an unreachable region is
/// not a deviation at all — it is an absence of evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Classification {
    /// Material removed that should have stayed. Unambiguous, and nothing
    /// downstream can put it back.
    Gouge,
    /// Material left standing. **Expected by default** — it is what a roughing
    /// pass is for — and a defect only when this was the last operation on that
    /// surface, which a single comparison cannot know.
    ExcessStock,
    /// Nominal surface facing away from the tool's approach direction, so no
    /// 3-axis tool could reach it at this setup.
    Undercut,
    /// Nominal surface no ray sampled, for a reason not identified.
    Unreachable,
}

impl Classification {
    /// Short stable name, used in reports and as a hash input.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Gouge => "gouge",
            Self::ExcessStock => "excess-stock",
            Self::Undercut => "undercut",
            Self::Unreachable => "unreachable",
        }
    }

    /// Whether this class is a defect on its own, without further context.
    ///
    /// **Only a gouge is.** Excess stock is what a roughing pass leaves; an
    /// undercut is a property of the part and the setup rather than of the
    /// program; an unreachable region is missing evidence. Each is worth
    /// reporting and none is worth failing a run over, which is why this is a
    /// method rather than a comment.
    #[must_use]
    pub const fn is_defect(self) -> bool {
        matches!(self, Self::Gouge)
    }

    /// Every class, in a fixed order.
    #[must_use]
    pub const fn all() -> [Self; 4] {
        [
            Self::Gouge,
            Self::ExcessStock,
            Self::Undercut,
            Self::Unreachable,
        ]
    }
}

/// One cluster of samples, before it is given an identity or an attribution.
#[derive(Debug, Clone)]
pub struct Cluster {
    /// What kind of problem this is.
    pub class: Classification,
    /// Indices into the deviation field's samples, ascending.
    pub samples: Vec<u32>,
    /// Deepest departure from the nominal in the cluster, as a positive depth.
    pub worst_depth_mm: f64,
    /// Mean depth over the cluster's samples, in canonical order.
    pub mean_depth_mm: f64,
    /// Estimated surface area affected. See [`area_and_volume`] for the
    /// estimator and what bounds its error.
    pub area_mm2: f64,
    /// Estimated volume of material involved: the area, weighted by depth.
    pub volume_mm3: f64,
    /// Centroid of the cluster's sample positions.
    ///
    /// Good for saying *where* a finding is, and **useless for asking what
    /// caused it**: a centroid is an average, and the average of points on a
    /// curved or multi-faced surface lies off that surface entirely. Attribution
    /// uses [`Self::worst_at`].
    pub at: Vec3,
    /// Position of the deepest sample in the cluster.
    ///
    /// An actual span endpoint, so it lies exactly on a swept surface by
    /// construction — which is what makes it answerable when a motion is asked
    /// whether it reached this point. It is also the point a user cares about:
    /// "which line cut the deepest part of this gouge" is the question, and the
    /// centroid cannot answer it.
    pub worst_at: Vec3,
    /// A spread of sample positions across the finding, for attribution.
    ///
    /// **One point is not enough to attribute a finding.** A finding covers a
    /// region, and different parts of that region can lie on different segments'
    /// swept surfaces — a channel cut too shallow leaves excess whose deepest
    /// sample may sit where the *plunge* reached rather than where the pass did.
    /// Attributing the deepest point alone then names a real segment that is not
    /// the one a user needs to edit.
    ///
    /// So attribution probes several points and unions the answers. These are
    /// chosen deterministically — the deepest, then evenly spaced through the
    /// members in ascending index order — so the set is a property of the
    /// finding rather than of the traversal.
    pub probes: Vec<Vec3>,
    /// Axis-aligned bounds of the cluster.
    pub bounds: Aabb3,
}

/// A disjoint-set forest over sample indices.
struct Union {
    parent: Vec<u32>,
    rank: Vec<u8>,
}

impl Union {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..u32::try_from(n).unwrap_or(u32::MAX)).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            // Path halving: same asymptotics as full compression, one pass.
            let grand = self.parent[self.parent[x as usize] as usize];
            self.parent[x as usize] = grand;
            x = grand;
        }
        x
    }

    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        // Union by rank. Which of two equal-rank roots wins is decided by index,
        // not by argument order, so the forest's shape does not depend on the
        // order edges arrive in -- and neither, therefore, does anything a
        // profiler would show.
        let (lo, hi) = match self.rank[ra as usize].cmp(&self.rank[rb as usize]) {
            core::cmp::Ordering::Less => (ra, rb),
            core::cmp::Ordering::Greater => (rb, ra),
            core::cmp::Ordering::Equal => {
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                self.rank[hi as usize] += 1;
                (lo, hi)
            }
        };
        self.parent[lo as usize] = hi;
    }
}

/// The cell a point falls in, at the given cell size.
fn cell_of(p: Vec3, size: f64) -> (i64, i64, i64) {
    let q = |v: f64| {
        let c = (v / size).floor();
        // `as` on a non-finite or huge float saturates rather than wrapping,
        // which is the behaviour wanted here: a sample at infinity lands in a
        // corner bucket and is compared against nothing.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "saturating, and the lattice is far inside i64"
        )]
        {
            c as i64
        }
    };
    (q(p.x), q(p.y), q(p.z))
}

/// Which class a sample belongs to, or `None` if it is inside tolerance.
fn classify(d: &Deviation, tolerance_mm: f64) -> Option<Classification> {
    if d.signed_mm.abs() <= tolerance_mm {
        return None;
    }
    Some(if d.signed_mm < 0.0 {
        Classification::Gouge
    } else {
        Classification::ExcessStock
    })
}

/// Estimated area and volume for a set of samples.
///
/// # The estimator, and why it is not just "samples times cell area"
///
/// Each ray of a bundle covers one cell of *projected* area in the plane
/// perpendicular to its axis. A surface patch with unit normal `n` cut by that
/// ray therefore has true area `h^2 / |n . a|`, which runs away as the surface
/// turns to graze the bundle — and every surface grazes some bundle.
///
/// Two problems follow, and one choice solves both. A patch visible to all three
/// bundles is sampled three times, so summing naively triples it; and the
/// grazing bundles are exactly the ones whose `1 / |n . a|` is enormous.
///
/// So a sample counts **only when its own bundle is the best-aligned of the three
/// for its normal**. Each patch is then counted once, by the bundle that sees it
/// most squarely, and `|n . a|` for that bundle is at least `1/sqrt(3)` — the
/// body-diagonal worst case — so the weight is bounded by `sqrt(3)` rather than
/// unbounded.
///
/// It remains an estimate. It is reported as one, beside the sample count and
/// the worst depth, which are exact.
fn area_and_volume(samples: &[u32], all: &[Deviation], cell_mm: f64) -> (f64, f64) {
    let cell_area = cell_mm * cell_mm;
    let mut area = 0.0f64;
    let mut volume = 0.0f64;
    for &i in samples {
        let d = &all[i as usize];
        let n = d.normal;
        let dots = [n.x.abs(), n.y.abs(), n.z.abs()];
        // The best-aligned axis, ties to the lowest index so the choice is a
        // property of the normal rather than of the comparison order.
        let mut best = 0usize;
        for k in 1..3 {
            if dots[k] > dots[best] {
                best = k;
            }
        }
        if d.axis != best {
            continue;
        }
        let w = dots[best].max(1.0 / 3.0f64.sqrt());
        let patch = cell_area / w;
        area += patch;
        volume += patch * d.signed_mm.abs();
    }
    (area, volume)
}

/// Groups a deviation field's samples into findings.
///
/// Samples inside tolerance are dropped; the rest are joined when they are of
/// the same class and within `radius_mm` of each other. The result is sorted
/// into a canonical order — class, then worst depth descending, then position —
/// so that two runs of the same input produce the same list in the same order.
#[must_use]
pub fn cluster(samples: &[Deviation], params: &ClusterParams, cell_mm: f64) -> Vec<Cluster> {
    // Only the samples that are findings at all, in the order the field
    // produced them, which is bundle then ray then span and already canonical.
    let mut kept: Vec<u32> = Vec::new();
    let mut class_of: Vec<Classification> = Vec::new();
    for (i, d) in samples.iter().enumerate() {
        if let Some(c) = classify(d, params.tolerance_mm) {
            kept.push(u32::try_from(i).unwrap_or(u32::MAX));
            class_of.push(c);
        }
    }
    if kept.is_empty() {
        return Vec::new();
    }

    // Bucket by cell. The key is the cell, so iteration order is the lattice's
    // order rather than a hasher's.
    let mut grid: BTreeMap<(i64, i64, i64), Vec<u32>> = BTreeMap::new();
    for (slot, &i) in kept.iter().enumerate() {
        let key = cell_of(samples[i as usize].at, params.radius_mm);
        grid.entry(key)
            .or_default()
            .push(u32::try_from(slot).unwrap_or(u32::MAX));
    }

    let mut uf = Union::new(kept.len());
    let r2 = params.radius_mm * params.radius_mm;
    for (&(cx, cy, cz), here) in &grid {
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(there) = grid.get(&(cx + dx, cy + dy, cz + dz)) else {
                        continue;
                    };
                    for &a in here {
                        for &b in there {
                            if a >= b || class_of[a as usize] != class_of[b as usize] {
                                continue;
                            }
                            let pa = samples[kept[a as usize] as usize].at;
                            let pb = samples[kept[b as usize] as usize].at;
                            if pa.distance_squared(pb) <= r2 {
                                uf.union(a, b);
                            }
                        }
                    }
                }
            }
        }
    }

    // Gather members by root. Again a BTreeMap: the roots are indices, so the
    // order is numeric and reproducible.
    let mut groups: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for slot in 0..u32::try_from(kept.len()).unwrap_or(u32::MAX) {
        groups.entry(uf.find(slot)).or_default().push(slot);
    }

    let mut out: Vec<Cluster> = Vec::with_capacity(groups.len());
    for (_, slots) in groups {
        let class = class_of[slots[0] as usize];
        let members: Vec<u32> = slots.iter().map(|&s| kept[s as usize]).collect();

        let mut worst = 0.0f64;
        let mut worst_at = samples[members[0] as usize].at;
        let mut sum = 0.0f64;
        let mut centroid = Vec3::new(0.0, 0.0, 0.0);
        let mut bounds = Aabb3::EMPTY;
        for &i in &members {
            let d = &samples[i as usize];
            let depth = d.signed_mm.abs();
            if depth > worst {
                worst = depth;
                worst_at = d.at;
            }
            sum += depth;
            centroid = Vec3::new(
                centroid.x + d.at.x,
                centroid.y + d.at.y,
                centroid.z + d.at.z,
            );
            bounds = bounds.union_point(d.at);
        }
        #[allow(clippy::cast_precision_loss, reason = "a sample count")]
        let n = members.len() as f64;
        let (area_mm2, volume_mm3) = area_and_volume(&members, samples, cell_mm);
        let probes = probe_points(&members, samples, worst_at);
        out.push(Cluster {
            class,
            samples: members,
            worst_depth_mm: worst,
            mean_depth_mm: sum / n,
            area_mm2,
            volume_mm3,
            at: Vec3::new(centroid.x / n, centroid.y / n, centroid.z / n),
            worst_at,
            probes,
            bounds,
        });
    }

    sort_canonically(&mut out);
    out
}

/// How many points of a finding attribution asks about.
///
/// Eight is enough to reach both ends of a long channel and each face of a
/// corner, and small enough that attribution stays a rounding error beside the
/// comparison that produced the finding.
const PROBES: usize = 8;

/// A deterministic spread of positions across a cluster.
fn probe_points(members: &[u32], samples: &[Deviation], worst_at: Vec3) -> Vec<Vec3> {
    let mut out = vec![worst_at];
    if members.len() > 1 {
        let step = members.len().div_ceil(PROBES.saturating_sub(1)).max(1);
        for &i in members.iter().step_by(step) {
            let p = samples[i as usize].at;
            if !out.iter().any(|q| q.distance_squared(p) == 0.0) {
                out.push(p);
            }
        }
    }
    out.truncate(PROBES);
    out
}

/// Puts clusters in the order a report lists them.
///
/// Worst first within a class, because that is the order somebody reads them in.
/// Position breaks a tie, and `total_cmp` rather than `partial_cmp` so the order
/// is total over every `f64` and needs no unwrap.
pub fn sort_canonically(clusters: &mut [Cluster]) {
    clusters.sort_by(|a, b| {
        a.class
            .cmp(&b.class)
            .then(b.worst_depth_mm.total_cmp(&a.worst_depth_mm))
            .then(a.at.x.total_cmp(&b.at.x))
            .then(a.at.y.total_cmp(&b.at.y))
            .then(a.at.z.total_cmp(&b.at.z))
    });
}

/// The unsampled and unreachable parts of the nominal, as clusters.
///
/// # A different question from the one above
///
/// Everything else in this module groups places the *result* disagrees with the
/// nominal. This groups places the result says **nothing at all** about: nominal
/// surface no ray of any bundle landed near.
///
/// Two causes, and they are worth separating because one is the customer's
/// problem and one is ours:
///
/// - **Undercut.** The face points away from the tool's approach. No 3-axis tool
///   reaches it at this setup, at any resolution, and the fix is another setup
///   rather than a finer lattice. Detected from the nominal's own geometry: an
///   outward normal with a negative `Z` component faces down, and the tool comes
///   from above.
/// - **Unreachable.** Everything else — most often a pocket narrower than the
///   cutter, or a region the lattice simply missed. The honest report is that
///   there is no evidence here, not that the surface is good.
///
/// Reporting "no deviation" for either would be the worst available answer: it
/// reads as a pass and means nothing was looked at.
#[must_use]
pub fn unsampled(
    nominal: &crate::mesh::TriMesh,
    samples: &[Deviation],
    params: &ClusterParams,
) -> Vec<Cluster> {
    // Bucketed by **where on the nominal each sample was measured to**, not by
    // where the sample sits.
    //
    // The difference decides whether this function works at all. A gouge moves
    // the result surface away from the nominal by the gouge depth, so bucketing
    // by sample position leaves the nominal floor of a 1 mm gouge with no sample
    // within a 0.8 mm radius -- and the region gets reported as *unreachable*
    // when it is in fact the best-evidenced part of the part, with a gouge
    // finding sitting directly beneath it.
    //
    // That is exactly what the first version did, and the report said so:
    // two `unreachable` findings on the floor of a channel the engine had just
    // measured to a thousandth of a millimetre.
    let mut grid: BTreeMap<(i64, i64, i64), Vec<u32>> = BTreeMap::new();
    for (i, d) in samples.iter().enumerate() {
        grid.entry(cell_of(d.nearest_on_nominal, params.radius_mm))
            .or_default()
            .push(u32::try_from(i).unwrap_or(u32::MAX));
    }
    let covered = |p: Vec3| -> bool {
        let (cx, cy, cz) = cell_of(p, params.radius_mm);
        let r2 = params.radius_mm * params.radius_mm;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(bucket) = grid.get(&(cx + dx, cy + dy, cz + dz)) else {
                        continue;
                    };
                    if bucket
                        .iter()
                        .any(|&i| samples[i as usize].nearest_on_nominal.distance_squared(p) <= r2)
                    {
                        return true;
                    }
                }
            }
        }
        false
    };

    // One probe per nominal triangle, at its centroid. Coarser than the lattice
    // on a fine mesh and finer on a coarse one, which is the right way round: a
    // coarse mesh is where a whole facet can slip between rays.
    let mut probes: Vec<(Vec3, Classification, f64)> = Vec::new();
    for t in 0..nominal.triangle_count() {
        let c = nominal.centroid(t);
        if covered(c) {
            continue;
        }
        let Some(n) = nominal.face_normal(t) else {
            continue;
        };
        let class = if n.z < -1.0e-9 {
            Classification::Undercut
        } else {
            Classification::Unreachable
        };
        probes.push((c, class, nominal.double_area(t) / 2.0));
    }
    if probes.is_empty() {
        return Vec::new();
    }

    // The same union-find, over probes this time.
    let mut probe_grid: BTreeMap<(i64, i64, i64), Vec<u32>> = BTreeMap::new();
    for (i, (p, _, _)) in probes.iter().enumerate() {
        probe_grid
            .entry(cell_of(*p, params.radius_mm))
            .or_default()
            .push(u32::try_from(i).unwrap_or(u32::MAX));
    }
    let mut uf = Union::new(probes.len());
    let r2 = params.radius_mm * params.radius_mm;
    for (&(cx, cy, cz), here) in &probe_grid {
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(there) = probe_grid.get(&(cx + dx, cy + dy, cz + dz)) else {
                        continue;
                    };
                    for &a in here {
                        for &b in there {
                            if a >= b || probes[a as usize].1 != probes[b as usize].1 {
                                continue;
                            }
                            if probes[a as usize].0.distance_squared(probes[b as usize].0) <= r2 {
                                uf.union(a, b);
                            }
                        }
                    }
                }
            }
        }
    }

    let mut groups: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for i in 0..u32::try_from(probes.len()).unwrap_or(u32::MAX) {
        groups.entry(uf.find(i)).or_default().push(i);
    }

    let mut out = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let class = probes[members[0] as usize].1;
        let mut centroid = Vec3::new(0.0, 0.0, 0.0);
        let mut bounds = Aabb3::EMPTY;
        let mut area = 0.0f64;
        for &i in &members {
            let (p, _, a) = probes[i as usize];
            centroid = Vec3::new(centroid.x + p.x, centroid.y + p.y, centroid.z + p.z);
            bounds = bounds.union_point(p);
            area += a;
        }
        #[allow(clippy::cast_precision_loss, reason = "a probe count")]
        let n = members.len() as f64;
        out.push(Cluster {
            class,
            // Deliberately empty: these clusters index the *nominal*, not the
            // deviation field, and putting nominal triangle indices in a field
            // documented as deviation samples would be a trap for the next
            // reader.
            samples: Vec::new(),
            // There is no depth to report. A region nothing sampled has no
            // measured deviation, and inventing one -- zero, or the tolerance --
            // would put a number in a report that no measurement produced.
            worst_depth_mm: 0.0,
            mean_depth_mm: 0.0,
            area_mm2: area,
            volume_mm3: 0.0,
            at: Vec3::new(centroid.x / n, centroid.y / n, centroid.z / n),
            // No samples, so no deepest one. The centroid stands in, and nothing
            // attributes these anyway: no segment caused an undercut.
            worst_at: Vec3::new(centroid.x / n, centroid.y / n, centroid.z / n),
            probes: Vec::new(),
            bounds,
        });
    }
    sort_canonically(&mut out);
    out
}

/// The axis a bundle index names, for callers assembling reports.
#[must_use]
pub const fn axis_of(index: usize) -> Axis {
    match index {
        0 => Axis::X,
        1 => Axis::Y,
        _ => Axis::Z,
    }
}
