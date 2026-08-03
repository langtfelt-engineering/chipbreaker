// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

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
    ];

    let tools: Vec<Tool> = entries
        .into_iter()
        .map(|(id, description, profile, gauge)| {
            Tool::new(
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
