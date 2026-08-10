// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! One program, one stock, one report.
//!
//! # Why this is shared rather than written twice
//!
//! Three entry points assemble the same job: the browser build, the C ABI, and
//! through it the Python bindings. Written separately they would agree on the
//! day they were written and drift afterwards — one would gain the collision
//! check, another would default a tolerance differently, and the two would
//! return different verdicts for identical inputs. Nobody would notice, because
//! nothing compares them.
//!
//! That is not a hypothetical risk for this engine specifically. Its whole
//! claim is that the same inputs produce the same answer everywhere, and a
//! second assembly of the same steps is the easiest possible way to break that
//! claim while every determinism test still passes: the tests hash the
//! *engine*, and this would be a difference in what the engine was asked.
//!
//! So there is one assembly, and the callers differ only in their limits and in
//! how bytes reach them.
//!
//! # Why it lives here
//!
//! It needs the parser and the engine, and this is the crate that already has
//! both. The alternative — a fifth crate holding twelve functions — buys
//! tidiness at the cost of another licence exception, another dependency entry,
//! and another thing to explain.

use chipbreaker_core::budget::{Budget, Spacing};
use chipbreaker_core::deviation::compare;
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::cluster::{ClusterParams, cluster};
use chipbreaker_core::findings::detect::{CollideParams, collide_with_stock};
use chipbreaker_core::findings::identify;
use chipbreaker_core::findings::report::{
    InputHash, Manifest, Report, SweptSplit, digest_bytes, semantics_from, semantics_uncompared,
};
use chipbreaker_core::findings::verdict::{self, GateOutcome, Verdict};
use chipbreaker_core::mesh::TriMesh;
use chipbreaker_core::mesh::units::Unit;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::tool::ToolLibrary;

/// What a caller asks for, and the limits it imposes.
#[derive(Debug, Clone)]
pub struct JobRequest<'a> {
    /// The NC program.
    pub program: &'a str,
    /// The tool library, as JSON.
    pub tools: &'a str,
    /// The stock mesh, as STL bytes. Binary or ASCII.
    pub stock_stl: &'a [u8],
    /// The nominal part, when there is one.
    ///
    /// Absent is not a defect: the gouge gate reports `unchecked` and the
    /// report says why, which is a different thing from passing.
    pub nominal_stl: Option<&'a [u8]>,
    /// Which tool from the library. `None` takes the program's first `T` word.
    pub tool_id: Option<&'a str>,
    /// Where these bytes came from, for a human. Never part of the identity.
    pub source: Option<&'a str>,
    /// Dexel spacing, in millimetres.
    pub resolution_mm: f64,
    /// The tolerance findings are judged against.
    pub tolerance_mm: f64,
    /// Below this, a pass is reported as a near miss.
    pub clearance_mm: f64,
    /// A memory ceiling checked **before anything is allocated**, so that
    /// exceeding it is a sentence naming a resolution that would fit rather
    /// than an allocation failure.
    pub memory_ceiling_bytes: Option<u64>,
    /// The largest program this caller will replay. `None` for no limit.
    ///
    /// A property of where the engine is running rather than of the engine: a
    /// browser tab has one, a workstation does not.
    pub segment_cap: Option<usize>,
}

/// Runs one job, or returns the sentence explaining why it will not.
///
/// # Errors
///
/// Every error is a **refusal carrying a reason a person can act on**, not a
/// code: an unsupported construct named, a foreign dialect identified, a
/// resolution that will not fit together with one that would. Callers pass it
/// through intact.
#[allow(clippy::too_many_lines, reason = "one linear assembly of one job")]
pub fn run(req: &JobRequest<'_>) -> Result<Report, String> {
    if !req.resolution_mm.is_finite() || req.resolution_mm <= 0.0 {
        return Err(format!(
            "resolution must be a positive length, got {}",
            req.resolution_mm
        ));
    }

    let stock =
        read_stl(req.stock_stl).map_err(|e| format!("the stock mesh could not be read: {e}"))?;

    let library = ToolLibrary::from_json(req.tools)
        .map_err(|e| format!("the tool library could not be read: {e}"))?;

    let options = crate::resolve::ParseOptions::default();
    let rapid_path = options.rapid_path;
    let (toolpath, _, _) =
        crate::resolve::parse(req.program, "program", &options, None).map_err(|e| e.to_string())?;

    if let Some(cap) = req.segment_cap
        && toolpath.segments.len() > cap
    {
        return Err(format!(
            "this program has {} segments and this build is capped at {cap}. The cap is a \
             property of where the engine is running rather than of the engine itself: the \
             native build has no such limit and is several times faster.",
            toolpath.segments.len()
        ));
    }

    // The ceiling is checked **after parsing and before the field is built**.
    //
    // Both halves of that matter. Before the field, because the field is the
    // large allocation and the failure this prevents is a tab that disappears
    // or a host that dies without a message. After parsing, because the budget
    // needs to know how many segments there are -- and an earlier version of
    // this code, having no count yet, passed the segment *cap* instead. With no
    // cap set that was `usize::MAX`, and the first C host to run a job was told
    // its 6-line program needed sixteen exabytes of toolpath IR.
    //
    // The parse is bounded by the size of the file the caller already holds in
    // memory, so doing it first costs nothing worth protecting against.
    if let Some(bytes) = req.memory_ceiling_bytes {
        let extents = stock.bounds().extent().to_array();
        Budget::bytes(bytes)
            .check(
                extents,
                Spacing::uniform(req.resolution_mm),
                u64::try_from(toolpath.segments.len()).unwrap_or(u64::MAX),
                false,
            )
            .map_err(|e| e.to_string())?;
    }

    let profile = match req.tool_id {
        Some(id) => library
            .get(id)
            .ok_or_else(|| format!("no tool with id {id:?} in the library"))?
            .profile()
            .clone(),
        None => {
            let first = toolpath.segments.first().map_or(0, |s| s.tool);
            library
                .get_by_number(first)
                .ok_or_else(|| {
                    format!(
                        "the program's first motion uses tool number {first} and the library has \
                         no such tool; either add it or name a tool explicitly"
                    )
                })?
                .profile()
                .clone()
        }
    };

    let (mut field, _) = TriDexelField::build(
        &stock,
        &TriBuildOptions {
            spacing: req.resolution_mm,
            ..TriBuildOptions::default()
        },
    )
    .map_err(|e| format!("the stock field could not be built: {e}"))?;

    // Motions carry the provenance that lets a finding name an NC line. A
    // report that says "there is a collision" without saying which line caused
    // it has kept the least useful half of the answer.
    let mut motions = Vec::new();
    let mut kinds = Vec::new();
    let mut provenance = Vec::new();
    for seg in &toolpath.segments {
        if let Some(m) = chipbreaker_core::toolpath::segment_motion(seg) {
            motions.push(m);
            kinds.push(seg.kind);
            provenance.push(seg.source);
        }
    }

    let method = SweepMethod::Analytic {
        tolerance: req.resolution_mm / 10.0,
    };
    let mut scratch = CutScratch::new(&profile);

    // Collision checking **replaces** the cut rather than following it. The
    // check is interleaved with the cutting because a collision is a property
    // of the trajectory, and neither the untouched stock nor the finished field
    // can answer for the middle of the run.
    //
    // A tool with no holder geometry has nothing above the flutes that could
    // collide. The engine says so by name and the gate reports `unchecked`
    // carrying that sentence — which does not pass, so a caller sees what was
    // established rather than a clear.
    let (collisions, collision_gate, stats) = match collide_with_stock(
        &mut field,
        &profile,
        &motions,
        &kinds,
        &provenance,
        0,
        &[],
        &CollideParams {
            clearance_mm: req.clearance_mm,
            grid_mm: 2.0 * req.resolution_mm,
            method,
        },
        &mut scratch,
    ) {
        Ok((found, stats)) => {
            let gate = Report::collision_gate(&found);
            (found, gate, stats)
        }
        Err(unchecked) => {
            // Declined before cutting anything, so the material still has to be
            // removed for the comparison below to mean anything.
            let stats = cut_all(
                &mut field,
                &profile,
                &motions,
                method,
                &mut scratch,
                DEFAULT_BATCH,
            );
            (
                Vec::new(),
                GateOutcome::unchecked(unchecked.to_string()),
                stats,
            )
        }
    };

    // The split comes from the cut that actually happened, so it describes this
    // run and no other.
    let sweep = Some(SweptSplit {
        ray_cuts_exact: stats.rays_exact,
        ray_cuts_bounded: stats.rays_substepped,
        worst_bound_mm: stats.worst_bound_mm,
    });

    let nominal = match req.nominal_stl {
        Some(bytes) => {
            Some(read_stl(bytes).map_err(|e| format!("the nominal mesh could not be read: {e}"))?)
        }
        None => None,
    };

    let params = ClusterParams::for_spacing(req.resolution_mm, req.tolerance_mm);
    let spacing_mm = [req.resolution_mm; 3];

    let (findings, semantics, gouge_gate) = match &nominal {
        Some(n) => {
            let d = compare(&field, n, Some(&stock));
            let found = identify(
                cluster(&d.samples, &params, req.resolution_mm),
                params.radius_mm,
            );
            let gate = Report::gouge_gate(&found);
            (
                found,
                semantics_from(&d, spacing_mm, req.tolerance_mm, sweep),
                gate,
            )
        }
        None => (
            Vec::new(),
            semantics_uncompared(spacing_mm, req.tolerance_mm, sweep),
            GateOutcome::unchecked(
                "no nominal part was supplied, so the cut stock was compared against nothing; \
                 a gate that did not run has not passed",
            ),
        ),
    };

    // Inputs by content, never by path: two runs of the same bytes from
    // different places are the same run, and a manifest that disagreed would
    // make the identity a property of somebody's filesystem.
    let where_from = req.source.unwrap_or("(supplied by the caller)");
    let mut inputs = vec![
        InputHash {
            role: "program".to_owned(),
            path: where_from.to_owned(),
            digest: digest_bytes(req.program.as_bytes()),
        },
        InputHash {
            role: "stock".to_owned(),
            path: where_from.to_owned(),
            digest: digest_bytes(req.stock_stl),
        },
        InputHash {
            role: "tools".to_owned(),
            path: where_from.to_owned(),
            digest: digest_bytes(req.tools.as_bytes()),
        },
    ];
    if let Some(bytes) = req.nominal_stl {
        inputs.push(InputHash {
            role: "nominal".to_owned(),
            path: where_from.to_owned(),
            digest: digest_bytes(bytes),
        });
    }
    inputs.sort_by(|a, b| a.role.cmp(&b.role));

    Ok(Report {
        manifest: Manifest {
            inputs,
            spacing_mm,
            tolerance_mm: req.tolerance_mm,
            cluster_radius_mm: params.radius_mm,
            engine_version: env!("CARGO_PKG_VERSION").to_owned(),
            engine_selftest: selftest_digest(),
            // One setup, so no boundary is crossed and none is claimed.
            boundaries: Vec::new(),
        },
        semantics,
        findings,
        collisions,
        // Both gates stated, never silently omitted: a verdict with a gate
        // missing reads as a clear.
        verdict: Verdict::new()
            .with(verdict::GATE_GOUGE, gouge_gate)
            .with(verdict::GATE_COLLISION, collision_gate),
        // The policy the replay actually used. A collision result is only as
        // trustworthy as the rapid path it was computed against, because a
        // dogleg rapid can hit what a linear one misses.
        rapid_path: Some(rapid_path),
    })
}

/// The self-test digest, computed once.
fn selftest_digest() -> String {
    use std::sync::OnceLock;
    static DIGEST: OnceLock<String> = OnceLock::new();
    DIGEST
        .get_or_init(|| {
            chipbreaker_core::selftest::run_with(crate::selftest::suites())
                .digest
                .to_hex()
        })
        .clone()
}

/// Reads an STL, binary or ASCII, in millimetres.
///
/// Which one a customer's CAM system wrote is not a question worth asking a
/// caller about their own file.
///
/// # Errors
/// If the bytes are neither a readable binary STL nor valid UTF-8 ASCII STL.
pub fn read_stl(bytes: &[u8]) -> Result<TriMesh, String> {
    use chipbreaker_core::mesh::io::stl;
    if stl::looks_binary(bytes) {
        stl::read_binary(bytes, Unit::Millimetre).map_err(|e| e.to_string())
    } else {
        let text = core::str::from_utf8(bytes).map_err(|e| format!("not UTF-8: {e}"))?;
        stl::read_ascii(text, Unit::Millimetre).map_err(|e| e.to_string())
    }
}
