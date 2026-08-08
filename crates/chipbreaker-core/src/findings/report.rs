// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The verification report: findings, and everything needed to trust them.
//!
//! # The artifact is the product
//!
//! Everything else in this engine computes. This module is what a customer
//! actually receives, and it is the only part a quality engineer will read
//! during an audit. Two things follow.
//!
//! **A finding without its error budget is not evidence.** "There is a 1.4 mm
//! gouge" is a claim; "there is a 1.4 mm gouge, measured on a 0.4 mm lattice,
//! against a nominal whose own facets are 0.06 mm, with 62% of ray-cuts computed
//! in closed form and the rest bounded at 0.04 mm" is evidence. The second can
//! be checked, argued with, and relied on. The first has to be taken on trust,
//! and a verification tool that asks for trust has misunderstood its job.
//!
//! **A report has to be readable by somebody who did not run it.** Six months
//! later, from the file alone, it must be possible to say which inputs produced
//! it and whether they are the inputs in front of you now. That is what the
//! manifest is for, and why it is content-addressed rather than a list of
//! filenames — a path is not an input, it is a place an input used to be.
//!
//! # What is deliberately in the unhashed section
//!
//! Timestamp and host. Both are useful to a reader and neither may reach the
//! hash: two runs of the same inputs on two machines an hour apart must produce
//! the same report identity, or the identity is measuring the clock rather than
//! the work.
//!
//! This is the same split the self-test report has used since the beginning,
//! for the same reason, and it is worth keeping the two consistent.
//!
//! # Schema stability
//!
//! The JSON here is a **public interface**. Integrators build against it,
//! and later work extends it rather than reshaping it. Fields are sorted, the
//! version is explicit, and a new field is an addition — never a rename, never a
//! change of meaning under an existing name.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::deviation::DeviationField;
use crate::findings::{Classification, Finding};
use crate::golden::{CanonicalHash, Hashable};

/// The report schema version.
///
/// Bumped only for a **breaking** change to an existing field's meaning. Adding
/// a field does not bump it, because a consumer that ignores unknown keys is
/// unaffected — and a consumer that does not ignore them was going to break on
/// anything.
pub const SCHEMA_VERSION: u32 = 1;

/// The schema's stable name, so a consumer can tell a Chipbreaker report from
/// any other JSON it may be handed.
pub const SCHEMA: &str = "chipbreaker.verification-report";

/// One named input, by content rather than by path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputHash {
    /// What the input is for: `stock`, `nominal`, `program`, `tools`.
    pub role: String,
    /// Where it came from, for a human. **Not** part of the identity.
    pub path: String,
    /// Digest of the bytes.
    pub digest: String,
}

/// Everything needed to say which inputs produced which findings.
///
/// The manifest's own hash is the report's identity, and two runs sharing it
/// must produce byte-identical findings. That is a test rather than an
/// aspiration.
#[derive(Debug, Clone, PartialEq)]
pub struct Manifest {
    /// Every input, by content, sorted by role.
    pub inputs: Vec<InputHash>,
    /// Cell size per axis, in millimetres.
    pub spacing_mm: [f64; 3],
    /// The tolerance findings were judged against.
    pub tolerance_mm: f64,
    /// The radius that grouped samples into findings.
    pub cluster_radius_mm: f64,
    /// Engine version.
    pub engine_version: String,
    /// The engine's own self-test digest, which is identical on all four
    /// targets and therefore identifies the *build's behaviour* rather than the
    /// build.
    pub engine_selftest: String,
}

impl Hashable for Manifest {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Manifest");
        for i in &self.inputs {
            // The path is deliberately absent. Two runs of the same bytes from
            // different directories are the same run, and a manifest that
            // disagreed would make the identity a property of somebody's
            // filesystem layout.
            h.str(&i.role);
            h.str(&i.digest);
        }
        h.f64_slice(&self.spacing_mm);
        h.f64(self.tolerance_mm);
        h.f64(self.cluster_radius_mm);
        h.str(&self.engine_version);
        h.str(&self.engine_selftest);
        h.end();
    }
}

impl Manifest {
    /// The report's identity.
    #[must_use]
    pub fn digest(&self) -> String {
        let mut h = CanonicalHash::new();
        self.hash_canonical(&mut h);
        h.finish().to_hex()
    }
}

/// What the numbers in this report can and cannot support.
///
/// **This is the section the unit exists for.** Everything above it is a
/// measurement; this is the statement of what the measurement is worth.
#[derive(Debug, Clone, PartialEq)]
pub struct NumericalSemantics {
    /// Cell size per axis, repeated here so the section stands alone.
    pub spacing_mm: [f64; 3],
    /// The tolerance applied.
    pub tolerance_mm: f64,
    /// How the swept volumes were computed, when the run that produced the field
    /// reported it.
    ///
    /// **`None` is not zero, and the difference matters.** `verify` reads a
    /// field; the split belongs to the run that cut it, and a field does not
    /// carry it. Emitting zeros would put two numbers in an audited artifact
    /// that no measurement produced — and "no ray-cut was bounded" is a strong
    /// claim to make by accident. Absent says absent, and says how to get it.
    pub sweep: Option<SweptSplit>,
    /// Estimated chord error of the stock mesh.
    pub stock_facet_mm: f64,
    /// Estimated chord error of the nominal mesh.
    pub nominal_facet_mm: f64,
    /// The coarsest of the inputs, below which a tolerance describes the inputs
    /// rather than the part.
    pub tolerance_floor_mm: f64,
    /// Whether the requested tolerance is below that floor.
    pub below_floor: bool,
    /// How far the perpendicular reading overstated the metric at worst.
    pub worst_projection_gap_mm: f64,
}

/// How a run's swept volumes were computed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweptSplit {
    /// Ray-cuts whose swept volume was computed in closed form.
    pub ray_cuts_exact: u64,
    /// Ray-cuts that were sub-stepped.
    pub ray_cuts_bounded: u64,
    /// Worst deviation bound among the **bounded** ray-cuts only.
    ///
    /// Reported beside the split rather than as a single figure for the run: a
    /// mixed program's worst bound belongs to the segments that earned it, and
    /// quoting it for the whole job would be a claim about work that carried no
    /// sweep error at all.
    pub worst_bound_mm: f64,
}

/// What a deviation bound does **not** cover.
///
/// In the artifact, not only in the documentation. A customer reading a report
/// six months from now will not have read this project's README, and the single
/// most consequential misunderstanding available with this tool is to believe it
/// verified the part rather than the program.
pub const EXCLUSIONS: [&str; 7] = [
    "tool wear",
    "deflection under cutting load",
    "thermal growth",
    "spindle runout",
    "backlash",
    "controller interpolation between programmed points",
    "workholding deflection and fixture error",
];

/// The sentence that accompanies them.
pub const SCOPE_STATEMENT: &str = "This report compares the computed stock against the ideal \
     geometric cutting model. A part can match it exactly and still be out of tolerance for any \
     of the excluded reasons. Chipbreaker verifies the program, not the machine and not the part.";

/// A complete verification report.
#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    /// Which inputs, at what settings.
    pub manifest: Manifest,
    /// What the numbers are worth.
    pub semantics: NumericalSemantics,
    /// The findings, in canonical order.
    pub findings: Vec<Finding>,
    /// Whether the run is acceptable: no gouge above tolerance.
    pub accepted: bool,
}

impl Report {
    /// Counts by class, in [`Classification::all`] order.
    #[must_use]
    pub fn counts(&self) -> [usize; 4] {
        super::counts(&self.findings)
    }

    /// The verdict, and the one rule behind it.
    ///
    /// **Only a gouge fails a run.** Excess stock is what a roughing pass is
    /// supposed to leave; an undercut is a property of the part and the setup;
    /// an unreachable region is missing evidence. A tool that failed a correct
    /// roughing pass would be switched off within a day, and one that failed a
    /// part for having an undercut would be blaming the program for the
    /// geometry.
    #[must_use]
    pub fn decide(findings: &[Finding]) -> bool {
        !findings.iter().any(Finding::is_defect)
    }

    /// The report as JSON, with keys sorted and floats at full precision.
    #[must_use]
    pub fn to_json(&self) -> Value {
        let findings: Vec<Value> = self.findings.iter().map(finding_json).collect();
        let inputs: Vec<Value> = self
            .manifest
            .inputs
            .iter()
            .map(|i| {
                json!({
                    "digest": i.digest,
                    "path": i.path,
                    "role": i.role,
                })
            })
            .collect();
        let c = self.counts();
        let mut by_class = serde_json::Map::new();
        for (i, class) in Classification::all().iter().enumerate() {
            by_class.insert((*class.as_str()).to_owned(), json!(c[i]));
        }

        json!({
            "schema": SCHEMA,
            "schema_version": SCHEMA_VERSION,
            "accepted": self.accepted,
            "verdict_rule": "a run is accepted when no finding is a gouge above tolerance; \
                             excess stock, undercuts and unreachable regions are reported but \
                             do not decide it",
            "manifest": {
                "digest": self.manifest.digest(),
                "inputs": inputs,
                "spacing_mm": self.manifest.spacing_mm,
                "tolerance_mm": self.manifest.tolerance_mm,
                "cluster_radius_mm": self.manifest.cluster_radius_mm,
                "engine_version": self.manifest.engine_version,
                "engine_selftest": self.manifest.engine_selftest,
            },
            "numerical_semantics": {
                "spacing_mm": self.semantics.spacing_mm,
                "tolerance_mm": self.semantics.tolerance_mm,
                "swept_volumes": self.semantics.sweep.map_or_else(
                    || json!({
                        "available": false,
                        "why": "a field does not carry the statistics of the run that cut it; \
                                pass --run-report from `chipbreaker run --json` to include them",
                    }),
                    |s| json!({
                        "available": true,
                        "ray_cuts_exact": s.ray_cuts_exact,
                        "ray_cuts_bounded": s.ray_cuts_bounded,
                        "worst_bound_mm": s.worst_bound_mm,
                        "worst_bound_applies_to": "the sub-stepped ray-cuts only, never the \
                                                   whole run",
                    }),
                ),
                "stock_facet_mm": self.semantics.stock_facet_mm,
                "nominal_facet_mm": self.semantics.nominal_facet_mm,
                "tolerance_floor_mm": self.semantics.tolerance_floor_mm,
                "below_floor": self.semantics.below_floor,
                "worst_projection_gap_mm": self.semantics.worst_projection_gap_mm,
                "detection_floor": {
                    "note": "recall measured against 295 injected defects at 0.4 mm; \
                             100% at and above half a cell, 80% below it, and no gouges \
                             invented on a correctly machined part",
                    "reference": "tests/corpus/defect/expectations.json",
                },
            },
            "exclusions": EXCLUSIONS,
            "scope": SCOPE_STATEMENT,
            "summary": {
                "total": self.findings.len(),
                "by_class": Value::Object(by_class),
                "worst_gouge_mm": self.findings.iter()
                    .filter(|f| f.class == Classification::Gouge)
                    .map(|f| f.worst_depth_mm)
                    .fold(0.0f64, f64::max),
                "worst_excess_mm": self.findings.iter()
                    .filter(|f| f.class == Classification::ExcessStock)
                    .map(|f| f.worst_depth_mm)
                    .fold(0.0f64, f64::max),
            },
            "findings": findings,
        })
    }

    /// The digest of everything in the report that is not the clock or the host.
    ///
    /// Two runs of the same inputs must agree on this, and a golden test pins
    /// it.
    #[must_use]
    pub fn digest(&self) -> String {
        let mut h = CanonicalHash::new();
        h.begin("Report");
        self.manifest.hash_canonical(&mut h);
        h.bool(self.accepted);
        for f in &self.findings {
            h.str(&f.id);
            h.str(f.class.as_str());
            h.f64(f.worst_depth_mm);
            h.f64(f.mean_depth_mm);
            h.f64(f.area_mm2);
            h.f64(f.volume_mm3);
            h.u64(f.sample_count as u64);
            h.f64_slice(&f.at.to_array());
            h.f64_slice(&f.worst_at.to_array());
            for s in &f.attribution.segments {
                h.u64(u64::from(*s));
            }
        }
        h.end();
        h.finish().to_hex()
    }
}

fn finding_json(f: &Finding) -> Value {
    let segments: Vec<Value> = f
        .attribution
        .segments
        .iter()
        .zip(&f.attribution.provenance)
        .map(|(seg, p)| {
            let mut o = serde_json::Map::new();
            o.insert("segment".to_owned(), json!(seg));
            o.insert("file".to_owned(), json!(p.file));
            o.insert("line".to_owned(), json!(p.line));
            o.insert("block".to_owned(), json!(p.block));
            // The cycle step earns itself here: one `G81` becomes rapid, plunge
            // and retract, and a report naming line 42 three times without
            // saying which of the three makes the reader do the work.
            if p.cycle_step != crate::toolpath::NOT_A_CYCLE_STEP {
                o.insert("cycle_step".to_owned(), json!(p.cycle_step));
            }
            Value::Object(o)
        })
        .collect();

    json!({
        "id": f.id,
        "class": f.class.as_str(),
        "is_defect": f.is_defect(),
        "severity": {
            "worst_depth_mm": f.worst_depth_mm,
            "mean_depth_mm": f.mean_depth_mm,
            "area_mm2": f.area_mm2,
            "volume_mm3": f.volume_mm3,
            "note": "depth and area are reported separately and deliberately not \
                     combined: a deep narrow gouge and a shallow broad one are \
                     different problems, and one number cannot say which this is. \
                     Depth and sample count are exact; area and volume are estimates.",
        },
        "sample_count": f.sample_count,
        "at": [f.at.x, f.at.y, f.at.z],
        "worst_at": [f.worst_at.x, f.worst_at.y, f.worst_at.z],
        "bounds": {
            "min": [f.bounds.min.x, f.bounds.min.y, f.bounds.min.z],
            "max": [f.bounds.max.x, f.bounds.max.y, f.bounds.max.z],
        },
        "attribution": {
            "ambiguous": f.attribution.is_ambiguous(),
            "segments": segments,
        },
    })
}

/// Builds the numerical-semantics section from what the run and the comparison
/// already measured.
#[must_use]
pub fn semantics_from(
    d: &DeviationField,
    spacing_mm: [f64; 3],
    tolerance_mm: f64,
    sweep: Option<SweptSplit>,
) -> NumericalSemantics {
    NumericalSemantics {
        spacing_mm,
        tolerance_mm,
        sweep,
        stock_facet_mm: d.stock_facet_mm,
        nominal_facet_mm: d.nominal_facet_mm,
        tolerance_floor_mm: d.tolerance_floor_mm(),
        below_floor: d.below_floor(tolerance_mm),
        worst_projection_gap_mm: d.worst_projection_gap_mm,
    }
}

/// Digest of a file's bytes, for the manifest.
#[must_use]
pub fn digest_bytes(bytes: &[u8]) -> String {
    let mut h = CanonicalHash::new();
    h.begin("Input");
    h.bytes(bytes);
    h.end();
    h.finish().to_hex()
}

/// The unhashed section: useful to a reader, never part of the identity.
#[must_use]
pub fn environment(host: &str, elapsed_ms: f64) -> Value {
    let mut m = BTreeMap::new();
    m.insert("host", json!(host));
    m.insert("elapsed_ms", json!(elapsed_ms));
    m.insert("target", json!(std::env::consts::ARCH));
    m.insert("os", json!(std::env::consts::OS));
    m.insert(
        "note",
        json!(
            "excluded from every digest in this report: two runs of the same inputs on two \
             machines must agree, or the identity measures the clock rather than the work"
        ),
    );
    json!(m)
}
