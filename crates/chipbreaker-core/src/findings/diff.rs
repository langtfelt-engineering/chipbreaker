// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Comparing two reports.
//!
//! # What converts a claim into a benefit somebody buys
//!
//! "Our engine is deterministic" is a property. **"Here is the diff between last
//! week's report and this week's, and only the intended thing changed"** is an
//! operation a customer can put in their CI, and it is the same property doing
//! visible work.
//!
//! The whole thing rests on identities being derived from content. With a
//! counter, inserting one finding renumbers every later one and the diff reports
//! that everything changed — technically true, useless. With content-derived
//! identities the diff is exact: a finding that did not move keeps its name, so
//! appearing, disappearing and changing severity are three distinguishable
//! events rather than one undifferentiated churn.
//!
//! # Manifest differences are reported first, and that ordering is deliberate
//!
//! If the resolution changed, every finding may have changed, and reading the
//! finding list first would send somebody hunting for a program bug that is
//! actually a settings change. So the manifest diff comes first and is labelled
//! as the thing that may explain everything below it.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::findings::report::Report;
use crate::findings::{Collision, Finding};

/// How a finding changed between two reports.
///
/// `Changed` carries two findings against the others' one, which makes the
/// variants different sizes. Boxed rather than padded: a diff of a large report
/// holds one of these per change, and most changes are appearances.
#[derive(Debug, Clone, PartialEq)]
pub enum Change {
    /// Present in the new report, absent from the old.
    Appeared(Finding),
    /// Present in the old report, absent from the new.
    Disappeared(Finding),
    /// Present in both, with a different depth or extent.
    Changed {
        /// As it was.
        before: Box<Finding>,
        /// As it is.
        after: Box<Finding>,
    },
}

impl Change {
    /// The finding's identity, whichever side it came from.
    #[must_use]
    pub fn id(&self) -> &str {
        match self {
            Self::Appeared(f) | Self::Disappeared(f) => &f.id,
            Self::Changed { after, .. } => &after.id,
        }
    }
}

/// How a collision changed between two reports.
///
/// Separate from [`Change`] for the same reason collisions are a separate array:
/// the severity that changed is a penetration, not a depth into a nominal
/// surface, and a single `worst_depth_mm` covering both would be one field name
/// carrying two quantities.
#[derive(Debug, Clone, PartialEq)]
pub enum CollisionChange {
    /// Present in the new report, absent from the old.
    Appeared(Collision),
    /// Present in the old report, absent from the new.
    Disappeared(Collision),
    /// Present in both, at a different penetration or clearance.
    Changed {
        /// As it was.
        before: Box<Collision>,
        /// As it is.
        after: Box<Collision>,
    },
}

impl CollisionChange {
    /// The collision's identity, whichever side it came from.
    #[must_use]
    pub fn id(&self) -> &str {
        match self {
            Self::Appeared(c) | Self::Disappeared(c) => &c.id,
            Self::Changed { after, .. } => &after.id,
        }
    }

    /// Whether this change involves a real collision on either side.
    #[must_use]
    pub fn touches_a_collision(&self) -> bool {
        match self {
            Self::Appeared(c) | Self::Disappeared(c) => c.is_defect(),
            Self::Changed { before, after } => before.is_defect() || after.is_defect(),
        }
    }
}

/// What differs between two reports.
#[derive(Debug, Clone, PartialEq)]
pub struct Diff {
    /// Settings and inputs that differ, as `name: (old, new)`.
    ///
    /// **Read this first.** A resolution change can move every finding, and a
    /// reader who starts with the finding list will look for a program bug that
    /// is not there.
    pub manifest: Vec<(String, String, String)>,
    /// Gates whose state changed, as `gate: (old, new)`.
    ///
    /// Reported alongside the manifest rather than buried with the findings: a
    /// gate that went from `pass` to `unchecked` has no finding attached to it
    /// at all, and a diff that only listed findings would show that change as
    /// nothing whatsoever.
    pub gates: Vec<(String, String, String)>,
    /// Findings that appeared, disappeared or changed, in canonical order.
    pub changes: Vec<Change>,
    /// Collisions that appeared, disappeared or changed, in canonical order.
    pub collisions: Vec<CollisionChange>,
}

impl Diff {
    /// True when the two reports say the same thing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.manifest.is_empty()
            && self.gates.is_empty()
            && self.changes.is_empty()
            && self.collisions.is_empty()
    }

    /// How many of each kind of change.
    #[must_use]
    pub fn tally(&self) -> (usize, usize, usize) {
        let mut t = (0, 0, 0);
        for c in &self.changes {
            match c {
                Change::Appeared(_) => t.0 += 1,
                Change::Disappeared(_) => t.1 += 1,
                Change::Changed { .. } => t.2 += 1,
            }
        }
        t
    }
}

/// Below this, two depths are the same depth.
///
/// Not zero, because a report is JSON and a float that survives a round trip
/// through decimal is not always the float that went in. A nanometre is far
/// below anything this engine claims to resolve and far above the round trip's
/// error.
const SAME_DEPTH_MM: f64 = 1.0e-9;

/// Compares two reports.
#[must_use]
pub fn diff(old: &Report, new: &Report) -> Diff {
    let mut manifest = Vec::new();
    let mut note = |name: &str, a: String, b: String| {
        if a != b {
            manifest.push((name.to_owned(), a, b));
        }
    };
    note(
        "spacing_mm",
        format!("{:?}", old.manifest.spacing_mm),
        format!("{:?}", new.manifest.spacing_mm),
    );
    note(
        "tolerance_mm",
        old.manifest.tolerance_mm.to_string(),
        new.manifest.tolerance_mm.to_string(),
    );
    note(
        "cluster_radius_mm",
        old.manifest.cluster_radius_mm.to_string(),
        new.manifest.cluster_radius_mm.to_string(),
    );
    note(
        "engine_version",
        old.manifest.engine_version.clone(),
        new.manifest.engine_version.clone(),
    );
    note(
        "engine_selftest",
        old.manifest.engine_selftest.clone(),
        new.manifest.engine_selftest.clone(),
    );
    // Inputs by role, so a changed stock mesh is named as such rather than as a
    // positional difference in a list.
    let by_role = |r: &Report| -> BTreeMap<String, String> {
        r.manifest
            .inputs
            .iter()
            .map(|i| (i.role.clone(), i.digest.clone()))
            .collect()
    };
    let (ao, an) = (by_role(old), by_role(new));
    let mut roles: Vec<&String> = ao.keys().chain(an.keys()).collect();
    roles.sort_unstable();
    roles.dedup();
    for role in roles {
        let a = ao.get(role).cloned().unwrap_or_else(|| "absent".to_owned());
        let b = an.get(role).cloned().unwrap_or_else(|| "absent".to_owned());
        note(&format!("input.{role}"), a, b);
    }

    let index = |r: &Report| -> BTreeMap<String, Finding> {
        r.findings
            .iter()
            .map(|f| (f.id.clone(), f.clone()))
            .collect()
    };
    let (fo, fnew) = (index(old), index(new));

    let mut changes = Vec::new();
    for (id, before) in &fo {
        match fnew.get(id) {
            None => changes.push(Change::Disappeared(before.clone())),
            Some(after) => {
                let moved = (before.worst_depth_mm - after.worst_depth_mm).abs() > SAME_DEPTH_MM
                    || (before.mean_depth_mm - after.mean_depth_mm).abs() > SAME_DEPTH_MM
                    || before.sample_count != after.sample_count
                    || before.attribution.segments != after.attribution.segments;
                if moved {
                    changes.push(Change::Changed {
                        before: Box::new(before.clone()),
                        after: Box::new(after.clone()),
                    });
                }
            }
        }
    }
    for (id, after) in &fnew {
        if !fo.contains_key(id) {
            changes.push(Change::Appeared(after.clone()));
        }
    }

    // Every gate on either side, so one that vanished is reported rather than
    // silently dropped.
    let mut gates = Vec::new();
    let mut names: Vec<&String> = old
        .verdict
        .gates()
        .keys()
        .chain(new.verdict.gates().keys())
        .collect();
    names.sort_unstable();
    names.dedup();
    for name in names {
        let show = |r: &Report| {
            r.verdict
                .gate(name)
                .map_or_else(|| "absent".to_owned(), |g| g.state.as_str().to_owned())
        };
        let (a, b) = (show(old), show(new));
        if a != b {
            gates.push((name.clone(), a, b));
        }
    }

    let by_id = |r: &Report| -> BTreeMap<String, Collision> {
        r.collisions
            .iter()
            .map(|c| (c.id.clone(), c.clone()))
            .collect()
    };
    let (co, cn) = (by_id(old), by_id(new));
    let mut collisions = Vec::new();
    for (id, before) in &co {
        match cn.get(id) {
            None => collisions.push(CollisionChange::Disappeared(before.clone())),
            Some(after) => {
                // A penetration that became a clearance is a change even at the
                // same magnitude: the program went from crashing to not.
                let moved = before.contact.is_collision() != after.contact.is_collision()
                    || (before.contact.magnitude() - after.contact.magnitude()).abs()
                        > SAME_DEPTH_MM
                    || before.attribution.segments != after.attribution.segments;
                if moved {
                    collisions.push(CollisionChange::Changed {
                        before: Box::new(before.clone()),
                        after: Box::new(after.clone()),
                    });
                }
            }
        }
    }
    for (id, after) in &cn {
        if !co.contains_key(id) {
            collisions.push(CollisionChange::Appeared(after.clone()));
        }
    }

    // Canonical order, so a diff of a diff is meaningful and so the exit code
    // and the text agree about what came first.
    changes.sort_by(|a, b| a.id().cmp(b.id()));
    collisions.sort_by(|a, b| a.id().cmp(b.id()));
    Diff {
        manifest,
        gates,
        changes,
        collisions,
    }
}

/// The diff as JSON.
#[must_use]
pub fn to_json(d: &Diff) -> Value {
    let manifest: Vec<Value> = d
        .manifest
        .iter()
        .map(|(k, a, b)| json!({ "field": k, "old": a, "new": b }))
        .collect();
    let changes: Vec<Value> = d
        .changes
        .iter()
        .map(|c| match c {
            Change::Appeared(f) => json!({
                "change": "appeared",
                "id": f.id,
                "class": f.class.as_str(),
                "worst_depth_mm": f.worst_depth_mm,
            }),
            Change::Disappeared(f) => json!({
                "change": "disappeared",
                "id": f.id,
                "class": f.class.as_str(),
                "worst_depth_mm": f.worst_depth_mm,
            }),
            Change::Changed { before, after } => json!({
                "change": "changed",
                "id": after.id,
                "class": after.class.as_str(),
                "worst_depth_mm": { "old": before.worst_depth_mm, "new": after.worst_depth_mm },
                "sample_count": { "old": before.sample_count, "new": after.sample_count },
            }),
        })
        .collect();
    let sev = |c: &Collision| match c.contact {
        crate::findings::Contact::Collision { penetration_mm }
        | crate::findings::Contact::CutterIntoFixture { penetration_mm } => {
            json!({ "penetration_mm": penetration_mm })
        }
        crate::findings::Contact::NearMiss { clearance_mm } => {
            json!({ "clearance_mm": clearance_mm })
        }
    };
    let collisions: Vec<Value> = d
        .collisions
        .iter()
        .map(|c| match c {
            CollisionChange::Appeared(x) => json!({
                "change": "appeared",
                "id": x.id,
                "contact": x.contact.as_str(),
                "severity": sev(x),
            }),
            CollisionChange::Disappeared(x) => json!({
                "change": "disappeared",
                "id": x.id,
                "contact": x.contact.as_str(),
                "severity": sev(x),
            }),
            CollisionChange::Changed { before, after } => json!({
                "change": "changed",
                "id": after.id,
                "contact": { "old": before.contact.as_str(), "new": after.contact.as_str() },
                "severity": { "old": sev(before), "new": sev(after) },
            }),
        })
        .collect();
    let gates: Vec<Value> = d
        .gates
        .iter()
        .map(|(k, a, b)| json!({ "gate": k, "old": a, "new": b }))
        .collect();
    let (appeared, disappeared, changed) = d.tally();
    json!({
        "identical": d.is_empty(),
        "manifest_differences": manifest,
        "gate_differences": gates,
        "summary": {
            "appeared": appeared,
            "disappeared": disappeared,
            "changed": changed,
            "collision_changes": d.collisions.len(),
        },
        "changes": changes,
        "collision_changes": collisions,
    })
}
