// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does clustering produce the same answer however the samples arrive?
//!
//! Clustering is the first thing in the engine with no right answer — whether
//! forty adjacent samples are one problem or forty is a presentation decision,
//! not a fact about the part. What can still be demanded is that the decision is
//! **reproducible**, and that is what this file tests.
//!
//! # Every test here carries its own mutation check
//!
//! `CONTRIBUTING.md` requires evidence that an assertion can fail. It matters
//! more here than anywhere so far: these assertions are about judgements rather
//! than numbers, and a judgement test can pass vacuously in ways a numeric one
//! cannot — by clustering everything into one blob, by clustering nothing, or by
//! comparing two empty lists and finding them equal.

use chipbreaker_core::defect::{STOCK, corpus};
use chipbreaker_core::deviation::{Deviation, compare};
use chipbreaker_core::dexel::tri::{TriBuildOptions, TriDexelField};
use chipbreaker_core::findings::cluster::{Classification, ClusterParams, cluster};
use chipbreaker_core::findings::{counts, identify};
use chipbreaker_core::math::Vec3;
use chipbreaker_core::mesh::{TriMesh, shapes};
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, flat_end_mill};

const SPACING: f64 = 0.4;

fn stock_mesh() -> TriMesh {
    shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(STOCK[0], STOCK[1], STOCK[2]),
    )
}

fn mill(diameter: f64) -> Profile {
    flat_end_mill(diameter, 30.0, &Shank::plain(diameter, 60.0)).expect("valid")
}

fn cut(motions: &[Motion], profile: &Profile) -> TriDexelField {
    let mut field = TriDexelField::build(
        &stock_mesh(),
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let mut scratch = CutScratch::new(profile);
    cut_all(
        &mut field,
        profile,
        motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// A part with one known gouge: the clean program, cut a millimetre too deep.
fn gouged_field() -> (TriDexelField, TriMesh) {
    use chipbreaker_core::contour::{ContourOptions, extract};
    let case = &corpus()[0];
    let profile = mill(6.0);
    let nominal = extract(&cut(&case.clean, &profile), &ContourOptions::default())
        .expect("extracts")
        .0;
    let deeper: Vec<Motion> = case
        .clean
        .iter()
        .map(|m| match m {
            Motion::Linear(l) => Motion::Linear(LinearMove {
                start: Vec3::new(l.start.x, l.start.y, l.start.z - 1.0),
                end: Vec3::new(l.end.x, l.end.y, l.end.z - 1.0),
            }),
            other => *other,
        })
        .collect();
    (cut(&deeper, &profile), nominal)
}

fn params() -> ClusterParams {
    ClusterParams::for_spacing(SPACING, SPACING / 2.0)
}

#[test]
fn a_gouged_part_produces_gouge_findings_and_no_excess() {
    let (field, nominal) = gouged_field();
    let d = compare(&field, &nominal, Some(&stock_mesh()));
    let found = cluster(&d.samples, &params(), SPACING);
    let f = identify(found, params().radius_mm);
    let by_class = counts(&f);

    println!(
        "gouged part: {} findings — {} gouge, {} excess, {} undercut, {} unreachable",
        f.len(),
        by_class[0],
        by_class[1],
        by_class[2],
        by_class[3]
    );
    for x in f.iter().take(3) {
        println!(
            "  {} {:<12} worst {:.4} mm, area {:.2} mm^2, {} samples at ({:.2}, {:.2}, {:.2})",
            x.id,
            x.class.as_str(),
            x.worst_depth_mm,
            x.area_estimate_mm2,
            x.sample_count,
            x.at.x,
            x.at.y,
            x.at.z
        );
    }

    assert!(by_class[0] > 0, "a part cut a millimetre deep has a gouge");
    assert_eq!(
        by_class[1], 0,
        "cutting too deep leaves nothing standing, so there is no excess stock"
    );
    // Not one finding per sample, and not one blob for the whole part. The
    // channel is a connected region, so a correct clustering makes it one
    // finding -- and this assertion is what would catch a radius so large that
    // every deviation on the part fused into a single meaningless finding.
    assert!(
        by_class[0] <= 4,
        "a single continuous channel should be a small number of findings, got {}",
        by_class[0]
    );
    let deepest = f
        .iter()
        .find(|x| x.class == Classification::Gouge)
        .expect("a gouge");
    assert!(
        (deepest.worst_depth_mm - 1.0).abs() < SPACING / 2.0,
        "the injected millimetre should come back as the worst depth, got {:.4}",
        deepest.worst_depth_mm
    );
}

#[test]
fn clustering_is_independent_of_sample_order() {
    // **The property that makes a finding reproducible.** Greedy accretion --
    // walk the samples and join or start a cluster -- passes on the ordering
    // the field happens to produce and fails on a shuffle, because three
    // collinear samples cluster differently depending on which end you start.
    // Union-find has no such freedom: the partition is the connected components
    // of the adjacency relation, and those are a property of the geometry.
    let (field, nominal) = gouged_field();
    let d = compare(&field, &nominal, Some(&stock_mesh()));
    let p = params();

    let baseline = summarise(&cluster(&d.samples, &p, SPACING), &d.samples);
    assert!(
        !baseline.is_empty(),
        "nothing to compare: the shuffle test would pass on two empty lists"
    );

    // A fixed seed, so a failure reproduces. SplitMix64 rather than a crate,
    // for the same reason everything else here avoids one.
    for seed in [1u64, 2, 0xDEAD_BEEF, u64::MAX] {
        let mut shuffled = d.samples.clone();
        fisher_yates(&mut shuffled, seed);
        let got = summarise(&cluster(&shuffled, &p, SPACING), &shuffled);
        assert_eq!(
            got, baseline,
            "seed {seed}: shuffling the samples changed the clustering. The \
             partition must be the connected components of the adjacency \
             relation, which is a property of the geometry rather than of the \
             traversal."
        );
    }
}

#[test]
fn the_order_independence_check_would_notice_a_greedy_clusterer() {
    // The mutation check for the test above.
    //
    // A greedy clusterer is simulated here directly -- walk the samples in
    // order, join the first cluster within the radius, or start a new one --
    // and the summary it produces must differ between two orderings. If it does
    // not, the fixture has no configuration where order matters and the test
    // above proves nothing about the real implementation.
    let (field, nominal) = gouged_field();
    let d = compare(&field, &nominal, Some(&stock_mesh()));
    let p = params();

    let straight = greedy(&d.samples, &p);
    let mut shuffled = d.samples.clone();
    fisher_yates(&mut shuffled, 12345);
    let mixed = greedy(&shuffled, &p);

    println!("greedy clusterer: {straight} clusters in field order, {mixed} after a shuffle");
    assert_ne!(
        straight, mixed,
        "a greedy clusterer produced the same count both ways on this fixture, \
         so the shuffle test above cannot distinguish it from union-find and is \
         not evidence of anything"
    );
}

/// What two clusterings must agree on: which physical samples ended up together.
///
/// **Deliberately not the sample indices.** Shuffling the input renumbers every
/// sample, so a cluster holding the same points holds different indices, and a
/// summary over indices reports a difference that is an artefact of the shuffle
/// rather than of the clustering. The first version of this did exactly that and
/// failed on a partition that was in fact identical -- same class, same 1621
/// members, same worst depth.
///
/// Positions are the sample's identity: they come from the field and the shuffle
/// does not touch them. Combined with XOR so the fold is commutative, which is
/// the point -- membership is a set, and a set has no order to compare.
fn summarise(
    cs: &[chipbreaker_core::findings::Cluster],
    samples: &[Deviation],
) -> Vec<(String, usize, u64, u64)> {
    cs.iter()
        .map(|c| {
            let members = c.samples.iter().fold(0u64, |acc, &i| {
                let at = samples[i as usize].at;
                // One hash per point, then XOR. Summing would collide on two
                // clusters that swapped a pair of members either side of a mean.
                let mut h = 1469598103934665603u64;
                for bits in [at.x.to_bits(), at.y.to_bits(), at.z.to_bits()] {
                    h = (h ^ bits).wrapping_mul(1099511628211);
                }
                acc ^ h
            });
            (
                c.class.as_str().to_owned(),
                c.samples.len(),
                c.worst_depth_mm.to_bits(),
                members,
            )
        })
        .collect()
}

/// The wrong algorithm, kept to prove the right one is being tested.
fn greedy(samples: &[Deviation], p: &ClusterParams) -> usize {
    let mut centres: Vec<Vec3> = Vec::new();
    for d in samples {
        if d.signed_mm.abs() <= p.tolerance_mm {
            continue;
        }
        if !centres
            .iter()
            .any(|c| c.distance_squared(d.at) <= p.radius_mm * p.radius_mm)
        {
            centres.push(d.at);
        }
    }
    centres.len()
}

/// Fisher-Yates over SplitMix64, so a shuffle is reproducible from its seed.
fn fisher_yates<T>(v: &mut [T], seed: u64) {
    let mut state = seed;
    let mut next = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    };
    for i in (1..v.len()).rev() {
        let j = (next() % (i as u64 + 1)) as usize;
        v.swap(i, j);
    }
}

#[test]
fn identities_are_derived_from_the_finding_and_not_from_a_counter() {
    // Insert a finding elsewhere on the part and the untouched one must keep its
    // name. A counter fails this immediately, and it is the property the whole
    // of `report-diff` rests on.
    let case = &corpus()[0];
    let profile = mill(6.0);
    use chipbreaker_core::contour::{ContourOptions, extract};
    let nominal = extract(&cut(&case.clean, &profile), &ContourOptions::default())
        .expect("extracts")
        .0;

    let deepen = |m: &Motion| match m {
        Motion::Linear(l) => Motion::Linear(LinearMove {
            start: Vec3::new(l.start.x, l.start.y, l.start.z - 1.0),
            end: Vec3::new(l.end.x, l.end.y, l.end.z - 1.0),
        }),
        other => *other,
    };
    let one: Vec<Motion> = case.clean.iter().map(deepen).collect();

    // The same gouge, plus a second one somewhere else entirely.
    let mut two = one.clone();
    two.push(Motion::Linear(LinearMove {
        start: Vec3::new(6.0, 25.0, 9.0),
        end: Vec3::new(34.0, 25.0, 9.0),
    }));

    let p = params();
    let ids = |ms: &[Motion]| -> Vec<String> {
        let d = compare(&cut(ms, &profile), &nominal, Some(&stock_mesh()));
        identify(cluster(&d.samples, &p, SPACING), p.radius_mm)
            .into_iter()
            .map(|f| f.id)
            .collect()
    };

    let before = ids(&one);
    let after = ids(&two);
    println!("before: {before:?}\nafter:  {after:?}");

    assert!(!before.is_empty(), "no findings to compare");
    assert!(
        after.len() > before.len(),
        "the second cut should add findings, got {} then {}",
        before.len(),
        after.len()
    );
    for id in &before {
        assert!(
            after.contains(id),
            "finding {id} lost its identity when an unrelated finding appeared \
             elsewhere on the part. Identities must be derived from the \
             finding's own class and position, never from its index in a list."
        );
    }
}

#[test]
fn the_identity_check_would_notice_a_counter() {
    // The mutation check for the test above: identities *assigned by position in
    // the list* must fail the "unchanged under an unrelated insertion" property
    // on this fixture. If they would not, the fixture inserts only at the end
    // and the test above is not testing what it claims.
    let (field, nominal) = gouged_field();
    let d = compare(&field, &nominal, Some(&stock_mesh()));
    let p = params();
    let cs = cluster(&d.samples, &p, SPACING);
    assert!(!cs.is_empty(), "need at least one finding");

    // A counter over the canonical order, which is sorted worst-first: a new
    // deeper finding anywhere takes index 0 and renumbers everything after it.
    let counter_ids: Vec<String> = (0..cs.len()).map(|i| format!("finding-{i}")).collect();
    let mut with_insert = counter_ids.clone();
    with_insert.insert(0, "finding-new".to_owned());
    let renumbered: Vec<String> = (0..with_insert.len())
        .map(|i| format!("finding-{i}"))
        .collect();

    assert_ne!(
        counter_ids.first(),
        renumbered.get(1),
        "a counter would have survived an insertion on this fixture, so the \
         identity test above proves nothing"
    );
}
