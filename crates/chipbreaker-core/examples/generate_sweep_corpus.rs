// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Regenerates `tests/corpus/sweep/expectations.json`.
//!
//! Goldens are **digests, not files**, as at Units 5 and 6: a cut `.tdx` field
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
//!   split and the arena's spill path was rebuilt at Unit 7 on that evidence.
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
use chipbreaker_core::sweep::LinearMove;
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod, cut_tri, distribution, spilled};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill, bull_end_mill, drill, flat_end_mill};

/// Cell size for every corpus case.
///
/// Divides the stock extents exactly, so `Lattice::pad` is zero and the cases
/// measure sweeping rather than ADR 0005's quantisation bias.
const SPACING: f64 = 0.5;

struct Case {
    id: &'static str,
    note: &'static str,
    tool: fn() -> Profile,
    moves: Vec<LinearMove>,
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
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn horizontal(y: f64, z: f64) -> LinearMove {
    LinearMove {
        start: Vec3::new(-5.0, y, z),
        end: Vec3::new(45.0, y, z),
    }
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
            moves: vec![LinearMove {
                start: Vec3::new(4.0, 5.0, 4.0),
                end: Vec3::new(36.0, 26.0, 4.0),
            }],
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
            moves: vec![LinearMove {
                start: Vec3::new(20.0, 15.0, 12.0),
                end: Vec3::new(20.0, 15.0, 2.0),
            }],
        },
        Case {
            id: "case-b-plunge-necked-shank",
            note: "bull-10-r2 necks from 5 mm to 4 mm, so the swept envelope is a moving \
                   MAXIMUM and a chain translation would under-report the neck",
            tool: bull,
            moves: vec![LinearMove {
                start: Vec3::new(20.0, 15.0, 14.0),
                end: Vec3::new(20.0, 15.0, 1.0),
            }],
        },
        Case {
            id: "case-b-retract",
            note: "an upward plunge, where the window runs the other way",
            tool: flat,
            moves: vec![
                LinearMove {
                    start: Vec3::new(20.0, 15.0, 12.0),
                    end: Vec3::new(20.0, 15.0, 3.0),
                },
                LinearMove {
                    start: Vec3::new(20.0, 15.0, 3.0),
                    end: Vec3::new(20.0, 15.0, 12.0),
                },
            ],
        },
        Case {
            id: "case-c-ramp-entry",
            note: "Case C: bounded sub-stepping, so this pins the step count AND the \
                   deviation bound it achieved",
            tool: flat,
            moves: vec![LinearMove {
                start: Vec3::new(6.0, 15.0, 10.0),
                end: Vec3::new(30.0, 15.0, 5.0),
            }],
        },
        Case {
            id: "case-c-ramp-diagonal",
            note: "a ramp with horizontal motion on both axes, the fully general case",
            tool: ball,
            moves: vec![LinearMove {
                start: Vec3::new(5.0, 6.0, 11.0),
                end: Vec3::new(34.0, 25.0, 4.0),
            }],
        },
        Case {
            id: "mixed-pocket-rib",
            note: "two slots leaving a rib, which is the geometry that made the Y bundle \
                   spill every ray and rebuilt the arena at Unit 7",
            tool: flat,
            moves: vec![horizontal(10.0, -1.0), horizontal(20.0, -1.0)],
        },
        Case {
            id: "mixed-raster-with-plunges",
            note: "the mix real finishing work has: plunge, cut, retract, reposition",
            tool: flat,
            moves: vec![
                LinearMove {
                    start: Vec3::new(8.0, 8.0, 12.0),
                    end: Vec3::new(8.0, 8.0, 6.0),
                },
                LinearMove {
                    start: Vec3::new(8.0, 8.0, 6.0),
                    end: Vec3::new(32.0, 8.0, 6.0),
                },
                LinearMove {
                    start: Vec3::new(32.0, 8.0, 6.0),
                    end: Vec3::new(32.0, 16.0, 6.0),
                },
                LinearMove {
                    start: Vec3::new(32.0, 16.0, 6.0),
                    end: Vec3::new(8.0, 16.0, 6.0),
                },
                LinearMove {
                    start: Vec3::new(8.0, 16.0, 6.0),
                    end: Vec3::new(8.0, 16.0, 12.0),
                },
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
            let stats = cut_tri(
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
