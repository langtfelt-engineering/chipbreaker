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
//!
//! # The version 2 break
//!
//! Version 2 **removes `accepted`** and replaces it with [`verdict`](super::Verdict),
//! an object with one entry per gate. This is the only breaking change the
//! schema has made, and it was made deliberately.
//!
//! Collision checking gave `accepted` a meaning it could not carry. The field
//! meant "no gouge above tolerance", and a consumer reading it — the obvious
//! thing to read — would have passed a program that drives a holder into a
//! fixture. Two repairs were available and both were worse:
//!
//! - **Leaving `accepted` alone and adding a second flag** permits `accepted:
//!   true` beside a spindle crash. "They should have checked the other field" is
//!   not a defence that survives the incident report.
//! - **Widening `accepted` in place** changes an existing field's meaning under
//!   its own name, breaking every version-1 consumer silently — the one thing
//!   the contract above promises never happens.
//!
//! Renaming breaks them **loudly**, at the moment it can still be fixed. That is
//! the whole value of the change, and it is why the cost was worth paying now:
//! the schema froze three units ago and has no installed base, so the price of a
//! break is currently zero and will never be this low again.
//!
//! A schema page that explains a break honestly argues for the contract's
//! seriousness; one that quietly widens a field argues against it.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::deviation::DeviationField;
use crate::findings::verdict::{self, GateOutcome, Verdict};
use crate::findings::{Classification, Collision, Contact, Finding, Obstacle, collide};
use crate::golden::{CanonicalHash, Hashable};
use crate::toolpath::RapidPath;

/// The report schema version.
///
/// Bumped only for a **breaking** change to an existing field's meaning. Adding
/// a field does not bump it, because a consumer that ignores unknown keys is
/// unaffected — and a consumer that does not ignore them was going to break on
/// anything.
///
/// `2` removed `accepted` in favour of `verdict.gates`; see the module header
/// for why that was the safest of the three options rather than the tidiest.
pub const SCHEMA_VERSION: u32 = 2;

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
    /// One entry per setup boundary crossed, in order.
    ///
    /// **Empty for a single-setup job**, which is what keeps the common case's
    /// report the same shape it has always been: a reader who never re-fixtures
    /// sees nothing new, and the section costs them nothing.
    ///
    /// Each boundary states how it was crossed and what that cost, because a
    /// job with two sampling regimes under one manifest is only honest if the
    /// manifest says so. Everything else in this engine is exact by
    /// construction; this is the one place a transform can lose anything, and
    /// folding it into a single figure would hide which setup paid it.
    pub boundaries: Vec<Boundary>,
}

/// One setup boundary, and what crossing it cost.
#[derive(Debug, Clone, PartialEq)]
pub struct Boundary {
    /// Which setup this boundary leads into, counting from zero.
    pub into_setup: u32,
    /// `exact` or `resampled`.
    pub regime: String,
    /// The bound this crossing contributes, in millimetres. Zero when exact.
    pub bound_mm: f64,
}

impl Hashable for Boundary {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Boundary");
        h.u64(u64::from(self.into_setup));
        h.str(&self.regime);
        h.f64(self.bound_mm);
        h.end();
    }
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
        // Part of the identity: two jobs that reached the same geometry by
        // different re-fixturing did not do the same thing, and a digest that
        // could not tell them apart would be claiming they had.
        h.usize(self.boundaries.len());
        for b in &self.boundaries {
            h.add(b);
        }
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
    /// Collisions and near misses, in canonical order.
    ///
    /// A separate array rather than another [`Classification`]: a collision's
    /// severity is penetration into an obstacle, not depth into a nominal
    /// surface, and putting the two under one field name would give
    /// `worst_depth_mm` two different physical meanings.
    pub collisions: Vec<Collision>,
    /// Every gate, and the conjunction over them.
    pub verdict: Verdict,
    /// How rapids were represented when the program was replayed.
    ///
    /// In the report because a dogleg rapid can collide where a linear one does
    /// not, so a collision result is only as trustworthy as the path policy it
    /// was computed against. `None` when no program was replayed.
    pub rapid_path: Option<RapidPath>,
}

impl Report {
    /// Counts by class, in [`Classification::all`] order.
    #[must_use]
    pub fn counts(&self) -> [usize; 4] {
        super::counts(&self.findings)
    }

    /// The gouge gate.
    ///
    /// **Only a gouge fails it.** Excess stock is what a roughing pass is
    /// supposed to leave; an undercut is a property of the part and the setup;
    /// an unreachable region is missing evidence. A tool that failed a correct
    /// roughing pass would be switched off within a day, and one that failed a
    /// part for having an undercut would be blaming the program for the
    /// geometry.
    #[must_use]
    pub fn gouge_gate(findings: &[Finding]) -> GateOutcome {
        let n = findings.iter().filter(|f| f.is_defect()).count();
        if n == 0 {
            GateOutcome::pass()
        } else {
            GateOutcome::fail(format!(
                "{n} gouge{} above tolerance",
                if n == 1 { "" } else { "s" }
            ))
        }
    }

    /// The collision gate, given collisions that were actually looked for.
    ///
    /// Near misses do not fail it: passing 0.2 mm from a clamp is a warning
    /// about the next edit, not a crash. They are reported all the same.
    #[must_use]
    pub fn collision_gate(collisions: &[Collision]) -> GateOutcome {
        let n = collide::collision_count(collisions);
        if n == 0 {
            GateOutcome::pass()
        } else {
            GateOutcome::fail(format!("{n} collision{}", if n == 1 { "" } else { "s" }))
        }
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
        let mut gates = serde_json::Map::new();
        for (name, outcome) in self.verdict.gates() {
            gates.insert(
                name.clone(),
                match &outcome.why {
                    Some(w) => json!({"state": outcome.state.as_str(), "why": w}),
                    None => json!({"state": outcome.state.as_str()}),
                },
            );
        }
        let collisions: Vec<Value> = self.collisions.iter().map(collision_json).collect();

        json!({
            "schema": SCHEMA,
            "schema_version": SCHEMA_VERSION,
            "verdict": {
                "pass": self.verdict.pass(),
                "gates": Value::Object(gates),
            },
            "verdict_rule": verdict::VERDICT_RULE,
            "manifest": {
                "digest": self.manifest.digest(),
                "inputs": inputs,
                "spacing_mm": self.manifest.spacing_mm,
                "tolerance_mm": self.manifest.tolerance_mm,
                "cluster_radius_mm": self.manifest.cluster_radius_mm,
                "engine_version": self.manifest.engine_version,
                "engine_selftest": self.manifest.engine_selftest,
                // Absent, not empty, for a single-setup job: a reader who never
                // re-fixtures should see the report they have always seen.
                "boundaries": if self.manifest.boundaries.is_empty() {
                    Value::Null
                } else {
                    Value::Array(self.manifest.boundaries.iter().map(|b| json!({
                        "into_setup": b.into_setup,
                        "regime": b.regime,
                        "bound_mm": b.bound_mm,
                    })).collect())
                },
                "accumulated_transform_bound_mm": self.manifest.boundaries
                    .iter().map(|b| b.bound_mm).sum::<f64>(),
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
            "rapid_path": self.rapid_path.map_or_else(
                || json!({
                    "available": false,
                    "why": "no program was replayed, so no rapid policy applied",
                }),
                |p| json!({"available": true, "policy": p.as_str()}),
            ),
            "summary": {
                "total": self.findings.len(),
                "by_class": Value::Object(by_class),
                "collisions": collide::collision_count(&self.collisions),
                "near_misses": self.collisions.len()
                    - collide::collision_count(&self.collisions),
                "worst_penetration_mm": self.collisions.iter()
                    .filter_map(|c| match c.contact {
                        Contact::Collision { penetration_mm }
                        | Contact::CutterIntoFixture { penetration_mm } => Some(penetration_mm),
                        Contact::NearMiss { .. } => None,
                    })
                    .fold(0.0f64, f64::max),
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
            "collisions": collisions,
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
        self.verdict.hash_canonical(&mut h);
        h.str(self.rapid_path.map_or("", RapidPath::as_str));
        h.usize(self.collisions.len());
        for c in &self.collisions {
            h.str(&c.id);
            h.str(c.contact.as_str());
            h.f64(c.contact.magnitude());
            h.str(c.role.as_str());
            h.u64(u64::from(c.element_index));
            h.str(c.obstacle.kind());
            let (class, index) = c.obstacle.order();
            h.u64(u64::from(class));
            h.u64(u64::from(index));
            h.str(c.motion.as_str());
            h.f64_slice(&c.at.to_array());
            for s in &c.attribution.segments {
                h.u64(u64::from(*s));
            }
        }
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
            // A line number alone is ambiguous across a job: two setups number
            // their own lines from one.
            "setup": f.attribution.setup,
            "segments": segments,
        },
    })
}

/// One collision as JSON.
///
/// Shares `id`, `at`, `bounds` and `attribution` with a finding, and shares
/// nothing else — the severity block is a different quantity under a different
/// name, which is the entire reason collisions are not a [`Classification`].
fn collision_json(c: &Collision) -> Value {
    let segments: Vec<Value> = c
        .attribution
        .segments
        .iter()
        .zip(&c.attribution.provenance)
        .map(|(seg, p)| {
            let mut o = serde_json::Map::new();
            o.insert("segment".to_owned(), json!(seg));
            o.insert("file".to_owned(), json!(p.file));
            o.insert("line".to_owned(), json!(p.line));
            o.insert("block".to_owned(), json!(p.block));
            if p.cycle_step != crate::toolpath::NOT_A_CYCLE_STEP {
                o.insert("cycle_step".to_owned(), json!(p.cycle_step));
            }
            Value::Object(o)
        })
        .collect();

    let mut obstacle = serde_json::Map::new();
    obstacle.insert("kind".to_owned(), json!(c.obstacle.kind()));
    if let Obstacle::Fixture { index, name } = &c.obstacle {
        obstacle.insert("index".to_owned(), json!(index));
        obstacle.insert("name".to_owned(), json!(name));
    }

    // Penetration and clearance never share a key. A consumer that sorted on one
    // number would rank a safe pass beside a crash.
    let mut severity = serde_json::Map::new();
    match c.contact {
        Contact::Collision { penetration_mm } | Contact::CutterIntoFixture { penetration_mm } => {
            severity.insert("penetration_mm".to_owned(), json!(penetration_mm));
        }
        Contact::NearMiss { clearance_mm } => {
            severity.insert("clearance_mm".to_owned(), json!(clearance_mm));
        }
    }
    severity.insert(
        "note".to_owned(),
        json!(
            "penetration is how far the element entered the obstacle; clearance is how \
             close it came without touching. They are separate keys because they are \
             separate quantities, and neither is an area or a volume over a nominal \
             surface -- a collision has no position on the nominal part."
        ),
    );

    json!({
        "id": c.id,
        "contact": c.contact.as_str(),
        "is_defect": c.is_defect(),
        "severity": Value::Object(severity),
        "element": {
            "role": c.role.as_str(),
            "index": c.element_index,
        },
        "obstacle": Value::Object(obstacle),
        "motion": c.motion.as_str(),
        "at": [c.at.x, c.at.y, c.at.z],
        "bounds": {
            "min": [c.bounds.min.x, c.bounds.min.y, c.bounds.min.z],
            "max": [c.bounds.max.x, c.bounds.max.y, c.bounds.max.z],
        },
        "attribution": {
            "ambiguous": c.attribution.is_ambiguous(),
            "setup": c.attribution.setup,
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
