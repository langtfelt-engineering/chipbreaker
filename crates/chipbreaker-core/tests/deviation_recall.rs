// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Does the deviation field find the defects that were put there?
//!
//! The corpus is the oracle: every case perturbs exactly one segment by a known
//! depth at a known place, so recall and localisation are measurable rather than
//! argued. Recall is reported **as a curve against depth**, because a single
//! percentage hides the only part anyone asks about — the shape near the floor,
//! which is the honest answer to "what is the smallest gouge you can find".
//!
//! The false-positive floor is the test a customer effectively runs first: give
//! it a program that machines the part correctly and see whether it invents
//! anything.

use chipbreaker_core::defect::{DefectCase, STOCK, corpus};
use chipbreaker_core::deviation::{DeviationField, compare};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::Motion;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

const SPACING: f64 = 0.4;
const METHOD: SweepMethod = SweepMethod::Analytic {
    tolerance: SPACING / 10.0,
};

fn stock_mesh() -> TriMesh {
    shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(STOCK[0], STOCK[1], STOCK[2]),
    )
}

fn field_from(mesh: &TriMesh) -> TriDexelField {
    TriDexelField::build(
        mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0
}

fn mill(diameter: f64) -> Profile {
    flat_end_mill(diameter, 30.0, &Shank::plain(diameter, 60.0)).expect("valid")
}

/// Cuts a program into fresh stock.
fn cut(motions: &[Motion], profile: &Profile) -> TriDexelField {
    let mut field = field_from(&stock_mesh());
    let mut scratch = CutScratch::new(profile);
    cut_all(
        &mut field,
        profile,
        motions,
        METHOD,
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// The nominal part: what the clean program produces, extracted to a mesh.
///
/// Built by simulating the *good* path and contouring it, rather than by
/// modelling the intended shape independently. That is deliberate: it removes
/// the engine's own sampling error from both sides of the comparison, so what
/// remains is the injected defect and nothing else. A separately modelled
/// nominal would fold Unit 9's reconstruction error into every case and make the
/// detection floor a measurement of the contourer instead.
fn nominal_for(case: &DefectCase, profile: &Profile) -> TriMesh {
    use chipbreaker_core::contour::{ContourOptions, extract};
    let field = cut(&case.clean, profile);
    extract(&field, &ContourOptions::default()).expect("extracts").0
}

/// Runs one case and returns its deviation field.
fn run(case: &DefectCase) -> DeviationField {
    // A tool defect perturbs the cutter, not the path.
    let clean_tool = mill(6.0);
    let dirty_tool = mill(6.0 + case.tool_diameter_delta_mm);
    let nominal = nominal_for(case, &clean_tool);
    let result = if case.tool_length_delta_mm > 0.0 {
        // A longer tool reaches deeper for the same commanded Z. Modelled by
        // lowering the path, which is what a length offset error does.
        let lowered: Vec<Motion> = case
            .motions
            .iter()
            .map(|m| match m {
                Motion::Linear(l) => {
                    let d = case.tool_length_delta_mm;
                    Motion::Linear(chipbreaker_core::sweep::LinearMove {
                        start: Vec3::new(l.start.x, l.start.y, l.start.z - d),
                        end: Vec3::new(l.end.x, l.end.y, l.end.z - d),
                    })
                }
                other => *other,
            })
            .collect();
        cut(&lowered, &clean_tool)
    } else {
        cut(&case.motions, &dirty_tool)
    };
    compare(&result, &nominal, Some(&stock_mesh()))
}

/// Detected if any sample exceeds the tolerance.
fn detected(field: &DeviationField, tolerance: f64) -> bool {
    field.findings(tolerance) > 0
}

#[test]
fn a_perfect_part_reports_nothing() {
    // **The false-positive floor**, and the test a customer effectively runs
    // first. The same program produces both sides, so every sample should be
    // zero to within the engine's own noise.
    let profile = mill(6.0);
    let case = &corpus()[0];
    let nominal = nominal_for(case, &profile);
    let result = cut(&case.clean, &profile);
    let field = compare(&result, &nominal, Some(&stock_mesh()));

    let tolerance = SPACING;
    let spurious = field.findings(tolerance);
    #[allow(clippy::cast_precision_loss, reason = "a sample count")]
    let rate = spurious as f64 / field.samples.len().max(1) as f64;
    println!(
        "perfect part: {} samples, {spurious} above {tolerance} mm ({:.4}%), \
         worst gouge {:.4} mm, worst excess {:.4} mm, rms {:.4} mm",
        field.samples.len(),
        rate * 100.0,
        field.worst_gouge_mm,
        field.worst_excess_mm,
        field.rms_mm
    );
    assert!(
        rate < 0.01,
        "a correctly machined part produced findings on {:.2}% of samples at a \
         one-cell tolerance. That is the false-positive floor, and it is what a \
         customer sees before they see anything else.",
        rate * 100.0
    );
}

#[test]
fn the_sign_convention_is_gouges_negative_and_excess_positive() {
    // Tested from both ends. A sign error here inverts every finding in the
    // product: "metal left on the face" becomes "you have cut into it", which is
    // the difference between another pass and scrap.
    let cases = corpus();
    let deep_gouge = cases
        .iter()
        .find(|c| c.kind.is_gouge() && c.depth_mm.abs() >= 2.0)
        .expect("the corpus has a deep gouge");
    let deep_excess = cases
        .iter()
        .find(|c| !c.kind.is_gouge() && c.depth_mm.abs() >= 2.0)
        .expect("the corpus has thick excess stock");

    let g = run(deep_gouge);
    assert!(
        g.worst_gouge_mm > g.worst_excess_mm,
        "{}: a gouge case produced more excess ({:.4}) than gouge ({:.4}); the \
         sign convention is inverted",
        deep_gouge.id,
        g.worst_excess_mm,
        g.worst_gouge_mm
    );

    let e = run(deep_excess);
    assert!(
        e.worst_excess_mm > e.worst_gouge_mm,
        "{}: an excess-stock case produced more gouge ({:.4}) than excess \
         ({:.4}); the sign convention is inverted",
        deep_excess.id,
        e.worst_gouge_mm,
        e.worst_excess_mm
    );
}

#[test]
fn recall_against_depth() {
    // The headline. Reported as a curve because the shape near the floor is the
    // answer to "what is the smallest gouge you can find", and a single
    // percentage would hide exactly that.
    //
    // A subset of the corpus, stratified by depth: the full 295 cases each build
    // two fields and contour one, which is minutes rather than seconds and does
    // not belong in the fast suite. The bands are what matter, not the count.
    let cases = corpus();
    let tolerance = SPACING / 2.0;

    // Depth bands, in cells.
    let bands: [(f64, f64); 6] = [
        (0.0, 0.5),
        (0.5, 1.0),
        (1.0, 1.5),
        (1.5, 2.0),
        (2.0, 4.0),
        (4.0, 100.0),
    ];
    let mut hit = [0usize; 6];
    let mut total = [0usize; 6];

    for (index, case) in cases.iter().enumerate() {
        // Every seventh case: enough per band to be a rate, few enough to run.
        if index % 7 != 0 {
            continue;
        }
        let cells = case.cells(SPACING);
        let Some(band) = bands.iter().position(|(lo, hi)| cells >= *lo && cells < *hi) else {
            continue;
        };
        total[band] += 1;
        if detected(&run(case), tolerance) {
            hit[band] += 1;
        }
    }

    println!("\nrecall against depth, tolerance {tolerance} mm, cell {SPACING} mm:");
    println!("{:>14}{:>8}{:>8}{:>10}", "depth (cells)", "cases", "found", "recall");
    for (index, (lo, hi)) in bands.iter().enumerate() {
        if total[index] == 0 {
            continue;
        }
        #[allow(clippy::cast_precision_loss, reason = "small counts")]
        let rate = hit[index] as f64 / total[index] as f64;
        let label = if *hi >= 100.0 {
            format!("{lo:.1}+")
        } else {
            format!("{lo:.1}-{hi:.1}")
        };
        println!(
            "{label:>14}{:>8}{:>8}{:>9.1}%",
            total[index],
            hit[index],
            rate * 100.0
        );
    }

    // The contract: everything at or above two cells is found.
    let deep_total: usize = total[4] + total[5];
    let deep_hit: usize = hit[4] + hit[5];
    assert!(deep_total > 0, "no cases at or above two cells were sampled");
    assert_eq!(
        deep_hit, deep_total,
        "recall above 2x cell size must be 100%, got {deep_hit}/{deep_total}. \
         That number is what the product may claim, so it is not a threshold to \
         relax."
    );
}

#[test]
fn localisation_recovers_the_place_and_the_depth() {
    // Within one cell, and within 10% on depth. Only well-resolved cases: near
    // the floor the question is detection, not measurement, and demanding 10%
    // on a defect a fifth of a cell deep would be asking the lattice for
    // something it does not carry.
    let cases = corpus();
    let mut checked = 0usize;
    for case in cases.iter().filter(|c| c.cells(SPACING) >= 3.0).take(6) {
        let field = run(case);
        let worst = field
            .samples
            .iter()
            .max_by(|a, b| {
                a.signed_mm
                    .abs()
                    .partial_cmp(&b.signed_mm.abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .expect("samples");

        let recovered = worst.signed_mm.abs();
        let expected = case.depth_mm.abs();
        let error = (recovered - expected).abs() / expected;
        println!(
            "{}: depth {expected:.3} -> {recovered:.3} mm ({:.1}% out)",
            case.id,
            error * 100.0
        );
        assert!(
            error < 0.10,
            "{}: recovered {recovered:.4} mm against an injected {expected:.4} mm, \
             {:.1}% out",
            case.id,
            error * 100.0
        );
        checked += 1;
    }
    assert!(checked >= 4, "too few well-resolved cases checked: {checked}");
}

#[test]
fn a_coarse_nominal_is_detected_as_a_floor() {
    // The tessellation floor, on the nominal rather than the stock. A customer
    // feeding a 1 mm-faceted STL and asking for 0.01 mm findings is making ADR
    // 0005's mistake in a different costume, and the field must say so.
    let profile = mill(6.0);
    let case = &corpus()[0];
    let result = cut(&case.clean, &profile);

    // A deliberately coarse nominal: an eight-triangle box has facets metres
    // across in the sense that matters.
    let coarse = stock_mesh();
    let field = compare(&result, &coarse, Some(&stock_mesh()));
    println!(
        "coarse nominal: stock facets {:.3} mm, nominal facets {:.3} mm, floor {:.3} mm",
        field.stock_facet_mm,
        field.nominal_facet_mm,
        field.tolerance_floor_mm()
    );
    assert!(
        field.nominal_facet_mm > SPACING,
        "an eight-triangle box should read as coarser than a {SPACING} mm lattice"
    );
    assert!(
        field.below_floor(0.01),
        "a 0.01 mm tolerance against this nominal must be reported as below the \
         floor its inputs support"
    );
    assert!(
        !field.below_floor(100.0),
        "a tolerance far above the floor must not be flagged"
    );
}
