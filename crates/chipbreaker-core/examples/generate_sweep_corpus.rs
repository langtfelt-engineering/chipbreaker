// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Regenerates `tests/corpus/sweep/expectations.json`.
//!
//! Goldens are **digests, not files**, as everywhere else: a cut `.tdx` field
//! is a binary blob that cannot be diffed.
//!
//! Recorded alongside each digest, so a change can be attributed rather than
//! merely noticed:
//!
//! - **Removed volume per bundle**, at full precision. The three will not agree
//!   and are not meant to; each must reproduce its own number exactly.
//! - **Sub-steps and the deviation bound**, so a case that silently stopped
//!   taking its closed form shows up as a step count appearing from nowhere.
//! - **The span distribution and spill**, because cutting is what makes rays
//!   split and the arena's spill path was rebuilt on that evidence.
//!
//! A digest that moves while the volumes hold still is a serialization change;
//! one where the volumes move too is a geometry change; one where the sub-step
//! count moves is a dispatch change. Those want different investigations, which
//! is the whole reason to record more than the digest.

#![allow(missing_docs, reason = "an example binary, not API")]

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::golden::CanonicalHash;
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::arc::ArcMove;
use chipbreaker_core::sweep::cut::{
    CutScratch, SweepMethod, cut_tri_motion, distribution, spilled,
};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, bull_end_mill, drill, flat_end_mill};
use chipbreaker_core::toolpath::ArcPlane;

/// Cell size for every corpus case.
///
/// Divides the stock extents exactly, so `Lattice::pad` is zero and the cases
/// measure sweeping rather than ADR 0005's quantisation bias.
const SPACING: f64 = 0.5;

/// A full turn.
const TAU: f64 = 2.0 * PI;

/// Half a turn.
const PI: f64 = core::f64::consts::PI;

struct Case {
    id: &'static str,
    note: &'static str,
    tool: fn() -> Profile,
    moves: Vec<Motion>,
}

fn flat() -> Profile {
    flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn ball() -> Profile {
    ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn bull() -> Profile {
    bull_end_mill(10.0, 2.0, 16.0, &Shank::plain(8.0, 60.0)).expect("valid")
}

fn twist() -> Profile {
    drill(6.0, 118.0, 25.0, &Shank::plain(6.0, 50.0)).expect("valid")
}

fn stock() -> TriDexelField {
    TriDexelField::build(
        &shapes::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 10.0)),
        &TriBuildOptions {
            spacing_xyz: None,
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn horizontal(y: f64, z: f64) -> Motion {
    line(-5.0, y, z, 45.0, y, z)
}

fn line(sx: f64, sy: f64, sz: f64, ex: f64, ey: f64, ez: f64) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(sx, sy, sz),
        end: Vec3::new(ex, ey, ez),
    })
}

/// A `G17` arc or helix about `(cx, cy)`.
fn arc(cx: f64, cy: f64, radius: f64, from: f64, sweep: f64, z: f64, rise: f64) -> Motion {
    Motion::Arc(ArcMove {
        center: Vec3::new(cx, cy, 0.0),
        radius,
        start_angle: from,
        sweep,
        z,
        plane: ArcPlane::Xy,
        rise,
    })
}

fn cases() -> Vec<Case> {
    vec![
        Case {
            id: "case-a-slot-flat",
            note: "Case A closed form: the three-piece decomposition on a through slot",
            tool: flat,
            moves: vec![horizontal(15.0, -1.0)],
        },
        Case {
            id: "case-a-diagonal-ball",
            note: "Case A on an awkward bearing, where the prism's frame is not axis \
                   aligned and a sign error in the perpendicular would show",
            tool: ball,
            moves: vec![line(4.0, 5.0, 4.0, 36.0, 26.0, 4.0)],
        },
        Case {
            id: "case-a-along-x-ray-degenerate",
            note: "a move along X, so every X-bundle ray hits the degenerate path where \
                   the cross-section image collapses to a point",
            tool: flat,
            moves: vec![horizontal(15.0, 5.0)],
        },
        Case {
            id: "case-b-plunge-drill",
            note: "Case B closed form: a drill point, whose profile is not a flat bottom",
            tool: twist,
            moves: vec![line(20.0, 15.0, 12.0, 20.0, 15.0, 2.0)],
        },
        Case {
            id: "case-b-plunge-necked-shank",
            note: "bull-10-r2 necks from 5 mm to 4 mm, so the swept envelope is a moving \
                   MAXIMUM and a chain translation would under-report the neck",
            tool: bull,
            moves: vec![line(20.0, 15.0, 14.0, 20.0, 15.0, 1.0)],
        },
        Case {
            id: "case-b-retract",
            note: "an upward plunge, where the window runs the other way",
            tool: flat,
            moves: vec![
                line(20.0, 15.0, 12.0, 20.0, 15.0, 3.0),
                line(20.0, 15.0, 3.0, 20.0, 15.0, 12.0),
            ],
        },
        Case {
            id: "case-c-ramp-entry",
            note: "Case C: bounded sub-stepping, so this pins the step count AND the \
                   deviation bound it achieved",
            tool: flat,
            moves: vec![line(6.0, 15.0, 10.0, 30.0, 15.0, 5.0)],
        },
        Case {
            id: "case-c-ramp-diagonal",
            note: "a ramp with horizontal motion on both axes, the fully general case",
            tool: ball,
            moves: vec![line(5.0, 6.0, 11.0, 34.0, 25.0, 4.0)],
        },
        Case {
            id: "mixed-pocket-rib",
            note: "two slots leaving a rib, which is the geometry that made the Y bundle \
                   spill every ray and caused the arena to be rebuilt",
            tool: flat,
            moves: vec![horizontal(10.0, -1.0), horizontal(20.0, -1.0)],
        },
        Case {
            id: "arc-a-prime-full-circle",
            note: "Case A' closed form, full turn: the wedge covers everything and the \
                   swept solid is a plain annulus. Pins the annulus path in the X and Y \
                   bundles and the vertical cast at |d-R| in the Z bundle",
            tool: flat,
            moves: vec![arc(20.0, 15.0, 9.0, 0.0, TAU, 6.0, 0.0)],
        },
        Case {
            id: "arc-a-prime-quarter-with-caps",
            note: "a quarter turn, so both end caps are present and separate. A \
                   decomposition that dropped the endpoint tools would lose pi r^2 of \
                   cross-section here and nothing at all on the full circle",
            tool: ball,
            moves: vec![arc(20.0, 15.0, 8.0, 0.37, PI / 2.0, 5.0, 0.0)],
        },
        Case {
            id: "arc-a-prime-clockwise-across-zero",
            note: "a clockwise sweep whose wedge straddles the branch cut of the bearing. \
                   The half-plane pair is the union rather than the intersection here, \
                   which is the sign that is easy to get backwards",
            tool: flat,
            moves: vec![arc(18.0, 14.0, 10.0, 0.4, -2.6, 4.0, 0.0)],
        },
        Case {
            id: "arc-b-prime-helical-bore",
            note: "Case B': the angular and axial terms couple, so this sub-steps. Pins \
                   the step count and the bound, which is how a helix that silently \
                   started taking a closed form it has no right to would be caught",
            tool: flat,
            moves: vec![arc(20.0, 15.0, 5.0, 0.0, TAU, 10.0, -5.0)],
        },
        Case {
            id: "arc-g18-vertical-plane",
            note: "a G18 arc, turning about +Y. The tool's axis is not the arc's, so the \
                   collapse does not apply and this MUST sub-step. It is here to pin \
                   that refusal, not the accuracy",
            tool: flat,
            moves: vec![Motion::Arc(ArcMove {
                center: Vec3::new(20.0, 15.0, 0.0),
                radius: 6.0,
                start_angle: 0.0,
                sweep: PI / 2.0,
                z: 15.0,
                plane: ArcPlane::Zx,
                rise: 0.0,
            })],
        },
        Case {
            id: "mixed-arc-with-lead-in-and-out",
            note: "the shape a contouring pass actually has: tangential lead-in, arc, \
                   lead-out. Exercises the join between an exact linear case and an \
                   exact arc case, where a gap would show as a rib",
            tool: flat,
            moves: vec![
                line(5.0, 15.0, 5.0, 11.0, 15.0, 5.0),
                arc(20.0, 15.0, 9.0, PI, -PI, 5.0, 0.0),
                line(29.0, 15.0, 5.0, 36.0, 15.0, 5.0),
            ],
        },
        Case {
            id: "mixed-raster-with-plunges",
            note: "the mix real finishing work has: plunge, cut, retract, reposition",
            tool: flat,
            moves: vec![
                line(8.0, 8.0, 12.0, 8.0, 8.0, 6.0),
                line(8.0, 8.0, 6.0, 32.0, 8.0, 6.0),
                line(32.0, 8.0, 6.0, 32.0, 16.0, 6.0),
                line(32.0, 16.0, 6.0, 8.0, 16.0, 6.0),
                line(8.0, 16.0, 6.0, 8.0, 16.0, 12.0),
            ],
        },
    ]
}

fn main() {
    println!("{{");
    println!("  \"_note\": \"Generated by examples/generate_sweep_corpus.rs. Do not hand-edit.\",");
    println!(
        "  \"_metric\": \"Removed volume is a conservation check on the subtraction, not an \
         accuracy claim (ADR 0005). The three bundles are NOT expected to agree.\","
    );
    println!("  \"spacing_mm\": {SPACING},");
    println!("  \"cases\": [");

    let all = cases();
    for (index, case) in all.iter().enumerate() {
        let profile = (case.tool)();
        let mut field = stock();
        let mut scratch = CutScratch::new(&profile);
        let before = field.volumes();

        let mut substeps = 0u64;
        let mut worst_bound = 0.0f64;
        let mut rays_tested = 0u64;
        let mut rays_rejected = 0u64;
        let mut case_kinds: Vec<&str> = Vec::new();

        for motion in &case.moves {
            case_kinds.push(motion.case().as_str());
            let stats = cut_tri_motion(
                &mut field,
                &profile,
                motion,
                SweepMethod::Analytic {
                    tolerance: SPACING / 10.0,
                },
                &mut scratch,
            );
            substeps += stats.substeps;
            worst_bound = worst_bound.max(stats.worst_bound_mm);
            rays_tested += stats.rays_tested;
            rays_rejected += stats.rays_rejected;
        }

        let after = field.volumes();
        let removed: Vec<String> = AXES
            .iter()
            .map(|a| {
                let i = a.index();
                match (before[i], after[i]) {
                    (Some(b), Some(a)) => format!("{:.17e}", b - a),
                    _ => "null".to_owned(),
                }
            })
            .collect();

        let mut h = CanonicalHash::new();
        h.add(&field);
        let digest = h.finish().to_hex();

        let dist: Vec<String> = distribution(&field)
            .into_iter()
            .map(|(spans, rays)| format!("[{spans}, {rays}]"))
            .collect();

        println!("    {{");
        println!("      \"id\": \"{}\",", case.id);
        println!("      \"note\": \"{}\",", case.note);
        println!("      \"moves\": {},", case.moves.len());
        println!("      \"cases\": [\"{}\"],", case_kinds.join("\", \""));
        println!("      \"removed_mm3\": [{}],", removed.join(", "));
        println!("      \"substeps\": {substeps},");
        println!("      \"worst_bound_mm\": {worst_bound:.17e},");
        println!("      \"rays_tested\": {rays_tested},");
        println!("      \"rays_rejected\": {rays_rejected},");
        println!("      \"span_distribution\": [{}],", dist.join(", "));
        println!("      \"spilled_rays\": {},", spilled(&field));
        println!("      \"digest\": \"{digest}\"");
        print!("    }}");
        println!("{}", if index + 1 == all.len() { "" } else { "," });
    }
    println!("  ]");
    println!("}}");
}
