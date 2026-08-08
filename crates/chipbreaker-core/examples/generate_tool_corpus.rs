// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Regenerates `tests/corpus/tool/standard-library.json`.
//!
//! The library is a *fixture*, so generating it from the catalogue is
//! legitimate: it is the input the tests read, not the answer they check. The
//! expectations it is checked against are hand computed from closed forms in
//! `expectations.json`, because a test whose expected values come from the code
//! under test proves only that the code agrees with itself.
//!
//! Run with:
//!
//! ```text
//! cargo run -p chipbreaker-core --example generate_tool_corpus \
//!   > tests/corpus/tool/standard-library.json
//! ```

use chipbreaker_core::tool::catalog::{
    HolderStage, Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill,
    flat_end_mill, tapered_end_mill,
};
use chipbreaker_core::tool::io::ToolLibrary;
use chipbreaker_core::tool::profile::Profile;
use chipbreaker_core::tool::{Tool, ToolId};

fn main() {
    let plain = Shank::plain(6.0, 50.0);
    let held = Shank::with_holder(
        12.0,
        60.0,
        [
            HolderStage::cylinder(32.0, 28.0),
            HolderStage::taper(32.0, 50.0, 22.0),
        ],
    );

    // Real collet-chuck dimensions, because collision work is only worth doing
    // against geometry somebody could actually be holding. An ER16 nut is 28 mm
    // across and the chuck body behind it 34; an ER32 nut is 50 and its body 63.
    //
    // One diameter in each is an **inch size converted to millimetres**, which is
    // where a seventeen-significant-digit dimension in tool data actually comes
    // from: 1 1/16 in is 26.987499999999997 mm and 2 7/16 in is
    // 61.912499999999994 mm, and neither survives a parser that truncates, a
    // formatter that prints six digits, or a round trip through `f32`.
    //
    // A corpus built only from catalogue-round numbers cannot fail that way, so
    // it cannot detect it either. These are not invented noise -- they are what
    // an imperial chuck listed in millimetres genuinely measures.
    let er16 = |shank: f64, length: f64| {
        Shank::with_holder(
            shank,
            length,
            [
                // 1 1/16 in nut, 1 3/8 in body.
                HolderStage::cylinder(26.987499999999997, 21.0),
                HolderStage::cylinder(34.925, 41.0),
            ],
        )
    };
    let er32 = |shank: f64, length: f64| {
        Shank::with_holder(
            shank,
            length,
            [
                // 2 in nut, 2 7/16 in body.
                HolderStage::cylinder(50.8, 28.0),
                HolderStage::cylinder(61.912499999999994, 50.0),
            ],
        )
    };

    let entries: Vec<(&str, &str, Profile, f64)> = vec![
        (
            "flat-6",
            "6 mm flat end mill, 20 mm flute, plain 6 mm shank",
            flat_end_mill(6.0, 20.0, &plain).expect("valid"),
            80.0,
        ),
        (
            "ball-6",
            "6 mm ball nose, 20 mm flute",
            ball_end_mill(6.0, 20.0, &plain).expect("valid"),
            80.0,
        ),
        (
            "bull-10-r2",
            "10 mm bull nose, 2 mm corner, on an 8 mm shank",
            bull_end_mill(10.0, 2.0, 30.0, &Shank::plain(8.0, 60.0)).expect("valid"),
            95.0,
        ),
        (
            "chamfer-8-90",
            "8 mm chamfer mill, 1 mm flat tip, 90 degree included",
            chamfer_mill(8.0, 1.0, 90.0, 20.0, &Shank::plain(8.0, 55.0)).expect("valid"),
            85.0,
        ),
        (
            "vbit-8-60",
            "8 mm V-bit, 60 degree included, pointed",
            chamfer_mill(8.0, 0.0, 60.0, 20.0, &Shank::plain(8.0, 55.0)).expect("valid"),
            85.0,
        ),
        (
            "taper-3deg",
            "2 mm tip, 6 degree included taper, 20 mm flute",
            tapered_end_mill(2.0, 6.0, 20.0, &Shank::plain(8.0, 55.0)).expect("valid"),
            85.0,
        ),
        (
            "drill-6-118",
            "6 mm twist drill, 118 degree point",
            drill(6.0, 118.0, 30.0, &plain).expect("valid"),
            90.0,
        ),
        (
            "barrel-12-r200",
            "12 mm barrel cutter on a 200 mm arc",
            barrel_end_mill(12.0, 200.0, 60.0, &Shank::plain(12.0, 90.0)).expect("valid"),
            120.0,
        ),
        (
            "held-12",
            "12 mm flat in a shrink holder, for holder-collision work",
            flat_end_mill(12.0, 30.0, &held).expect("valid"),
            160.0,
        ),
        // The three cases below exist for collision checking, and each is a
        // shape where collisions actually happen rather than a tidy example.
        (
            "er16-flat-6",
            "6 mm flat, 20 mm flute, in an ER16 collet chuck",
            flat_end_mill(6.0, 20.0, &er16(6.0, 50.0)).expect("valid"),
            140.0,
        ),
        (
            "er32-stub-6",
            "6 mm flat, 10 mm flute, stub, in a bulky ER32 chuck",
            // The awkward one. A 6 mm cutter with only 28 mm of shank under a
            // holder 63 mm across: anything deeper than 28 mm and the chuck is
            // in the pocket. This is the geometry that crashes machines, and a
            // corpus of well-proportioned tools would never contain it.
            flat_end_mill(6.0, 10.0, &er32(6.0, 28.0)).expect("valid"),
            130.0,
        ),
        (
            "long-reach-6",
            "6 mm flat, 20 mm flute, 95 mm reach, in an ER16 chuck",
            // The other side of the same test: identical cutter, identical
            // holder, 67 mm more shank. It clears what the stub cannot, so a
            // collision reported for both is a collision the checker invented.
            flat_end_mill(6.0, 20.0, &er16(6.0, 95.0)).expect("valid"),
            185.0,
        ),
    ];

    let tools: Vec<Tool> = entries
        .into_iter()
        .enumerate()
        .map(|(index, (id, description, profile, gauge))| {
            // T1 upward, in listing order. The number is the primary key a
            // program resolves against; the name is metadata for reports.
            #[allow(clippy::cast_possible_truncation, reason = "a handful of tools")]
            let number = index as u32 + 1;
            Tool::new(
                number,
                ToolId::new(id).expect("valid identifier"),
                description,
                profile,
                gauge,
            )
            .expect("valid tool")
        })
        .collect();

    let library = ToolLibrary::from_tools(tools).expect("distinct identifiers");
    print!("{}", library.to_json());
}
