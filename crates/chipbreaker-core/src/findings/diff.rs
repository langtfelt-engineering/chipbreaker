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

use crate::findings::Finding;
use crate::findings::report::Report;

/// How a finding changed between two reports.
#[derive(Debug, Clone, PartialEq)]
pub enum Change {
    /// Present in the new report, absent from the old.
    Appeared(Finding),
    /// Present in the old report, absent from the new.
    Disappeared(Finding),
    /// Present in both, with a different depth or extent.
    Changed {
        /// As it was.
        before: Finding,
        /// As it is.
        after: Finding,
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

/// What differs between two reports.
#[derive(Debug, Clone, PartialEq)]
pub struct Diff {
    /// Settings and inputs that differ, as `name: (old, new)`.
    ///
    /// **Read this first.** A resolution change can move every finding, and a
    /// reader who starts with the finding list will look for a program bug that
    /// is not there.
    pub manifest: Vec<(String, String, String)>,
    /// Findings that appeared, disappeared or changed, in canonical order.
    pub changes: Vec<Change>,
}

impl Diff {
    /// True when the two reports say the same thing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.manifest.is_empty() && self.changes.is_empty()
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
                        before: before.clone(),
                        after: after.clone(),
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

    // Canonical order, so a diff of a diff is meaningful and so the exit code
    // and the text agree about what came first.
    changes.sort_by(|a, b| a.id().cmp(b.id()));
    Diff { manifest, changes }
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
    let (appeared, disappeared, changed) = d.tally();
    json!({
        "identical": d.is_empty(),
        "manifest_differences": manifest,
        "summary": {
            "appeared": appeared,
            "disappeared": disappeared,
            "changed": changed,
        },
        "changes": changes,
    })
}
