// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Known defects, injected into known-good toolpaths.
//!
//! # This is Phase D's dense reference, and it exists first for that reason
//!
//! Every unit up to eleven had a ground truth: an analytic volume, a dense
//! sub-stepping reference, an exact Sturm oracle, a longhand equivalent. When
//! something was wrong, a test could say so.
//!
//! Unit 12 and after produce **judgements**. Is this a gouge or excess stock? Is
//! 0.03 mm severe? Are forty adjacent deviations one finding or forty? There is
//! no oracle for "useful", and the discipline that carried eleven units does not
//! reach that question on its own.
//!
//! So the corpus is the oracle. Take a toolpath that produces the nominal part,
//! perturb exactly one segment by a known amount at a known place, and require
//! the deviation field to recover both. It is built **before** the deviation
//! field for the same reason the sub-stepping reference preceded Case A at Unit
//! 7: build the thing you are testing first and you end up validating it against
//! itself.
//!
//! # What a case carries
//!
//! Ground truth is the injection, not the measurement: the depth asked for, the
//! place it was asked for, and the kind of mistake it represents. A case is a
//! *question with a known answer*, and `_why` says what the question is for.
//!
//! # What this cannot tell you
//!
//! A recovered depth is the distance between the computed stock and the ideal
//! geometric cutting model. It is not the depth of a gouge in metal. Nothing
//! here models wear, deflection, thermal growth, runout, backlash or how a
//! controller interpolates between the points it is given.

use crate::math::Vec3;
use crate::sweep::{LinearMove, Motion};

/// What kind of mistake a case represents.
///
/// Named for the operator error rather than for the geometry, because that is
/// how a finding will eventually have to be explained.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DefectKind {
    /// A plunge that went further than it should have.
    PlungeTooDeep,
    /// A horizontal pass that cut into a wall it should have stopped short of.
    HorizontalOvercut,
    /// A rapid that clipped stock instead of clearing it.
    RapidClipsStock,
    /// The tool in the machine was larger than the tool in the program.
    ToolTooLarge,
    /// The tool was set too long, so every cut went deep.
    ToolTooLong,
    /// A retract that never happened, dragging the tool through material.
    MissingRetract,
    /// An arc whose centre was wrong, so it swept the wrong locus.
    ArcWrongCentre,
    /// Material left behind: the pass stopped short.
    ExcessStock,
}

impl DefectKind {
    /// Short stable name, used in the corpus and in reports.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::PlungeTooDeep => "plunge-too-deep",
            Self::HorizontalOvercut => "horizontal-overcut",
            Self::RapidClipsStock => "rapid-clips-stock",
            Self::ToolTooLarge => "tool-too-large",
            Self::ToolTooLong => "tool-too-long",
            Self::MissingRetract => "missing-retract",
            Self::ArcWrongCentre => "arc-wrong-centre",
            Self::ExcessStock => "excess-stock",
        }
    }

    /// Every kind, in a fixed order.
    #[must_use]
    pub const fn all() -> [Self; 8] {
        [
            Self::PlungeTooDeep,
            Self::HorizontalOvercut,
            Self::RapidClipsStock,
            Self::ToolTooLarge,
            Self::ToolTooLong,
            Self::MissingRetract,
            Self::ArcWrongCentre,
            Self::ExcessStock,
        ]
    }

    /// True if this kind removes material that should have stayed.
    ///
    /// A gouge is negative in the deviation field's convention; excess stock is
    /// positive. Getting this backwards inverts every finding in the product,
    /// which is why the sign lives in one place and is tested from both ends.
    #[must_use]
    pub const fn is_gouge(self) -> bool {
        !matches!(self, Self::ExcessStock)
    }
}

/// Where on the part a defect was injected.
///
/// The classification that matters is not the coordinate but the **kind of
/// geometry** it sits on: Unit 9 measured dual contouring reconstructing a flat
/// face exactly and an edge to within a cell, so a deviation of the same depth is
/// not equally easy to attribute in both places.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Locale {
    /// The middle of a large flat face.
    MidFace,
    /// Where two faces meet at a sharp angle.
    SharpEdge,
    /// Inside a concave blend.
    Fillet,
    /// The floor of a pocket, surrounded by walls.
    PocketFloor,
    /// A wall about one cell thick.
    ThinWall,
    /// Beside a hole that goes all the way through.
    NearThroughHole,
    /// A surface whose normal is a body diagonal -- the `1/sqrt(3)` worst case
    /// from Unit 6, where no bundle sees the surface better than 54.74 degrees.
    BodyDiagonal,
}

impl Locale {
    /// Short stable name.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::MidFace => "mid-face",
            Self::SharpEdge => "sharp-edge",
            Self::Fillet => "fillet",
            Self::PocketFloor => "pocket-floor",
            Self::ThinWall => "thin-wall",
            Self::NearThroughHole => "near-through-hole",
            Self::BodyDiagonal => "body-diagonal",
        }
    }

    /// Every locale, in a fixed order.
    #[must_use]
    pub const fn all() -> [Self; 7] {
        [
            Self::MidFace,
            Self::SharpEdge,
            Self::Fillet,
            Self::PocketFloor,
            Self::ThinWall,
            Self::NearThroughHole,
            Self::BodyDiagonal,
        ]
    }
}

/// Which bundle sees the perturbed surface best.
///
/// Recorded because the sampling guarantee is per bundle, and a defect on a
/// surface facing the coarse axis of an anisotropic field is a different
/// question from one facing the fine axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Facing {
    /// Normal along X.
    X,
    /// Normal along Y.
    Y,
    /// Normal along Z.
    Z,
    /// Normal along a body diagonal: no bundle sees it well.
    Diagonal,
}

impl Facing {
    /// Short stable name.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::X => "x",
            Self::Y => "y",
            Self::Z => "z",
            Self::Diagonal => "diagonal",
        }
    }
}

/// One injected defect, with the answer it is asking about.
#[derive(Debug, Clone)]
pub struct DefectCase {
    /// Stable identifier, used as the corpus key.
    pub id: String,
    /// What this case is for, in a sentence.
    pub why: String,
    /// The mistake being represented.
    pub kind: DefectKind,
    /// The geometry it sits on.
    pub locale: Locale,
    /// Which bundle sees it best.
    pub facing: Facing,
    /// **Ground truth.** How deep the perturbation is, in millimetres. Positive
    /// for excess stock, negative for a gouge, matching the deviation field.
    pub depth_mm: f64,
    /// **Ground truth.** Where the perturbation is centred, in machine
    /// coordinates.
    pub at: Vec3,
    /// Index of the perturbed segment in [`Self::motions`].
    ///
    /// `None` for a case whose perturbation is in the **tool** rather than the
    /// path -- a cutter larger or longer than the program believed. Those leave
    /// `motions` equal to `clean` and carry a delta below instead.
    pub segment: Option<usize>,
    /// Diameter the tool was actually larger by, in millimetres.
    ///
    /// A wrong-diameter cutter is not a wrong path, and modelling it as one
    /// would make `tool-too-large` and `horizontal-overcut` the same case with
    /// two names -- which the first version of this corpus did. It over-cuts on
    /// **both** sides of every pass, which is a different signature from a path
    /// that wandered one way.
    pub tool_diameter_delta_mm: f64,
    /// Length the tool was actually longer by, in millimetres.
    ///
    /// Distinct from a plunge that went too deep for the same reason: it deepens
    /// *every* cut in the program, not one segment.
    pub tool_length_delta_mm: f64,
    /// The good path.
    pub clean: Vec<Motion>,
    /// The same path with exactly one segment perturbed.
    pub motions: Vec<Motion>,
}

impl DefectCase {
    /// Cell sizes this defect's depth is, given a spacing.
    ///
    /// The number a recall curve is plotted against: absolute depth means
    /// nothing without the lattice it was found on.
    #[must_use]
    pub fn cells(&self, spacing: f64) -> f64 {
        if spacing > 0.0 {
            self.depth_mm.abs() / spacing
        } else {
            0.0
        }
    }
}

/// A straight move.
fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

/// The stock every case is cut from: 40 x 30 x 12 mm at the origin.
pub const STOCK: [f64; 3] = [40.0, 30.0, 12.0];

/// How far inside the stock a pass must begin and end, in millimetres.
///
/// Half the tool diameter, so a plunge at either end of a pass lands with the
/// whole cutter over material. Without it an anchor near an edge puts an end of
/// its own pass in mid-air, and a defect injected there is injected into
/// nothing.
const MARGIN: f64 = 3.0;

/// The depth ladder, in millimetres.
///
/// Deliberately straddling a 0.4 mm cell from a fifth of one to eight of them.
/// **The sub-cell end is the point**: it establishes the detection floor, which
/// is the number a customer asks for first and the one that decides what the
/// product may claim.
pub const DEPTHS: [f64; 10] = [0.08, 0.15, 0.25, 0.4, 0.6, 0.9, 1.4, 2.0, 2.8, 3.2];

/// Builds the whole corpus, in a fixed order.
///
/// Programmatic rather than hand-written: 200 cases by hand would be 200
/// opportunities for a transcription error in the ground truth, which is the one
/// thing here that must not be wrong.
#[must_use]
pub fn corpus() -> Vec<DefectCase> {
    let mut out = Vec::new();
    for (locale, facing) in [
        (Locale::MidFace, Facing::Z),
        (Locale::PocketFloor, Facing::Z),
        (Locale::SharpEdge, Facing::X),
        (Locale::Fillet, Facing::Y),
        (Locale::ThinWall, Facing::Y),
        (Locale::NearThroughHole, Facing::X),
        (Locale::BodyDiagonal, Facing::Diagonal),
    ] {
        for kind in DefectKind::all() {
            for (index, depth) in DEPTHS.iter().enumerate() {
                // Not every kind is meaningful in every locale, and a corpus
                // padded with cases that cannot happen would inflate the recall
                // denominator with questions nobody asked.
                if !plausible(kind, locale) {
                    continue;
                }
                // Ten depths x eight kinds x seven locales is 560, and the
                // spread matters more than the count. Gouges are thinned;
                // **excess stock keeps the whole ladder**, because there is only
                // one excess kind against seven gouge kinds and an unthinned
                // ladder is what keeps the sign balance from collapsing to a
                // tenth. The sign is not a detail: the failure modes differ, and
                // a corpus that is 87% gouges measures gouge recall and calls it
                // recall.
                if kind.is_gouge() && (index + kind as usize + locale as usize) % 2 == 1 {
                    continue;
                }
                out.push(build(kind, locale, facing, *depth));
            }
        }
    }
    out
}

/// Whether a kind can occur in a locale at all.
const fn plausible(kind: DefectKind, locale: Locale) -> bool {
    match (kind, locale) {
        // A plunge needs somewhere to plunge into.
        (DefectKind::PlungeTooDeep, Locale::ThinWall | Locale::SharpEdge) => false,
        // A through hole is where a rapid can clip, not a fillet.
        (DefectKind::RapidClipsStock, Locale::Fillet) => false,
        // An arc needs room to turn.
        (DefectKind::ArcWrongCentre, Locale::ThinWall) => false,
        _ => true,
    }
}

/// Builds one case.
fn build(kind: DefectKind, locale: Locale, facing: Facing, depth: f64) -> DefectCase {
    let at = anchor(locale);
    // Sign: gouges cut deeper than nominal and read negative; excess stock is
    // material left behind and reads positive.
    let signed = if kind.is_gouge() { -depth } else { depth };

    let (clean, motions, segment) = program(kind, locale, at, depth);
    // Tool perturbations leave the path alone. A cutter is not a coordinate.
    let (diameter_delta, length_delta) = match kind {
        DefectKind::ToolTooLarge => (2.0 * depth, 0.0),
        DefectKind::ToolTooLong => (0.0, depth),
        _ => (0.0, 0.0),
    };
    let id = format!(
        "{}-{}-{}-{:.2}",
        kind.as_str(),
        locale.as_str(),
        facing.as_str(),
        depth
    );
    let why = format!(
        "{} at {} facing {}, {:.2} mm deep: {}",
        kind.as_str(),
        locale.as_str(),
        facing.as_str(),
        depth,
        rationale(kind, locale)
    );
    DefectCase {
        id,
        why,
        kind,
        locale,
        facing,
        depth_mm: signed,
        at,
        segment,
        tool_diameter_delta_mm: diameter_delta,
        tool_length_delta_mm: length_delta,
        clean,
        motions,
    }
}

/// Why this combination is worth a case.
const fn rationale(kind: DefectKind, locale: Locale) -> &'static str {
    match locale {
        Locale::BodyDiagonal => {
            "no bundle sees a body-diagonal surface better than 54.74 degrees, so this is \
             the 1/sqrt(3) worst case from Unit 6"
        }
        Locale::SharpEdge => {
            "Unit 9 reconstructs a flat exactly and an edge to within a cell, so attribution \
             is hardest here"
        }
        Locale::Fillet => "a concave blend, where a gouge and the intended radius look alike",
        Locale::ThinWall => {
            "a wall about a cell thick, where a defect can remove the feature rather than \
             dent it"
        }
        Locale::NearThroughHole => {
            "beside a hole, where the surface the deviation is measured against ends"
        }
        Locale::PocketFloor => "surrounded by walls, so the defect is not on the outer surface",
        Locale::MidFace => match kind {
            DefectKind::ExcessStock => "the simplest excess-stock case, and the control",
            _ => "the simplest case, and the control the others are compared against",
        },
    }
}

/// A `y` offset by `distance` toward the middle of the stock.
///
/// Which side depends on where the anchor already sits, so that a case near the
/// far edge does not send its return leg off the part -- which is the same
/// mistake, in `y`, that clipping the pass extent fixes in `x`.
fn away_from(y: f64, distance: f64) -> f64 {
    if y <= 0.5 * STOCK[1] {
        y + distance
    } else {
        y - distance
    }
}

/// Where each locale sits on the part.
///
/// **Every anchor is below the stock top**, so the clean program cuts a real
/// slot and a perturbation has something to differ from. The first version put
/// `mid-face` at `z = 12`, the stock surface: the clean pass removed nothing,
/// and any perturbation that stayed at or above it removed nothing either, so
/// several cases were in the corpus with no defect in them. They counted in the
/// recall denominator and could not be found, because there was nothing there.
fn anchor(locale: Locale) -> Vec3 {
    match locale {
        Locale::MidFace => Vec3::new(20.0, 15.0, 7.0),
        Locale::PocketFloor => Vec3::new(20.0, 15.0, 6.0),
        Locale::SharpEdge => Vec3::new(8.0, 15.0, 7.5),
        Locale::Fillet => Vec3::new(20.0, 8.0, 8.0),
        Locale::ThinWall => Vec3::new(20.0, 22.0, 8.0),
        Locale::NearThroughHole => Vec3::new(30.0, 15.0, 6.0),
        Locale::BodyDiagonal => Vec3::new(12.0, 10.0, 8.0),
    }
}

/// The clean program and its perturbed twin.
///
/// Returns `(clean, perturbed, index of the changed segment)`. Exactly one
/// segment differs, which is what makes the ground truth unambiguous.
fn program(
    kind: DefectKind,
    locale: Locale,
    at: Vec3,
    depth: f64,
) -> (Vec<Motion>, Vec<Motion>, Option<usize>) {
    // A short, ordinary program: approach, a cut across the anchor, retract.
    //
    // The pass is **clipped to the stock**, with a margin, rather than running a
    // fixed 14 mm either side of the anchor. Two anchors sit close enough to an
    // edge that the unclipped form put an end of the pass in mid-air:
    // `near-through-hole` at `x = 30` ran to `x = 44` against a 40 mm stock, so
    // its plunge and its retract both happened outside the part. Every
    // `missing-retract` case there dragged the tool out through empty space and
    // injected nothing at all, while sitting in the recall denominator.
    let z = at.z;
    let x0 = (at.x - 14.0).max(MARGIN);
    let x1 = (at.x + 14.0).min(STOCK[0] - MARGIN);
    //
    // Four segments, not three. The fourth is the traverse home at clearance,
    // which cuts nothing at 16 mm over a 12 mm stock and exists so that the two
    // kinds whose mistake *is* a traverse have a level segment to spoil. Without
    // it they had to be modelled as a ramp out of the slot, and a ramp's deepest
    // point is wherever it happens to leave the slot rather than the depth the
    // case claims -- `rapid-clips-stock` at a claimed 1.4 mm injected 3.4.
    let back = away_from(at.y, 8.0);
    let clean = vec![
        line([x0, at.y, 16.0], [x0, at.y, z]),
        line([x0, at.y, z], [x1, at.y, z]),
        line([x1, at.y, z], [x1, at.y, 16.0]),
        line([x1, at.y, 16.0], [x0, back, 16.0]),
    ];
    let mut motions = clean.clone();

    let segment = match kind {
        // The plunge goes deeper than it should. One segment, one mistake.
        DefectKind::PlungeTooDeep => {
            motions[0] = line([x0, at.y, 16.0], [x0, at.y, z - depth]);
            motions[1] = line([x0, at.y, z - depth], [x1, at.y, z - depth]);
            motions[2] = line([x1, at.y, z - depth], [x1, at.y, 16.0]);
            Some(1)
        }
        // The cut wanders sideways into a wall.
        DefectKind::HorizontalOvercut => {
            motions[1] = line([x0, at.y - depth, z], [x1, at.y - depth, z]);
            Some(1)
        }
        // The path is correct; the cutter is not. Handled by the deltas on the
        // case rather than here, so the motions stay equal to `clean`.
        DefectKind::ToolTooLarge | DefectKind::ToolTooLong => None,
        // A rapid at a clearance height that is not clear.
        //
        // Three things have to be true at once, and this case was got wrong
        // three times, once for each of them.
        //
        // 1. The height is measured from the **stock top**, not from the retract
        //    height. The first version cleared to `16 - depth`, still above a
        //    12 mm stock at every depth in the ladder, and clipped nothing.
        // 2. The return leg has to cross **uncut** ground. The second travelled
        //    back along `at.y`, the line the pass had just cut a slot down, and
        //    ran along the inside of it removing nothing.
        // 3. The damaging segment has to be **level**. The third ramped out of
        //    the slot to the clearance height, so its deepest bite was wherever
        //    it crossed the slot edge rather than the depth claimed: 3.4 mm
        //    against a stated 1.4.
        //
        // The retract is lowered with it. It rises inside the slot the pass has
        // just cut, so it removes nothing and the whole of the injected defect
        // is the traverse.
        DefectKind::RapidClipsStock => {
            let clip = STOCK[2] - depth;
            motions[2] = line([x1, at.y, z], [x1, at.y, clip]);
            motions[3] = line([x1, at.y, clip], [x0, back, clip]);
            Some(3)
        }
        // The retract stops short, so the next move drags through material.
        //
        // Level at the same shortfall, for reason 3 above, and sideways rather
        // than back along the pass, for reason 2. It differs from the rapid in
        // extent rather than depth: a short move across the end of the pass
        // instead of a scar the width of the part, which is what a localisation
        // test has to be able to tell apart.
        DefectKind::MissingRetract => {
            let clip = STOCK[2] - depth;
            let out = away_from(at.y, 6.0);
            motions[2] = line([x1, at.y, z], [x1, at.y, clip]);
            motions[3] = line([x1, at.y, clip], [x1, out, clip]);
            Some(3)
        }
        // The arc turns about the wrong centre.
        DefectKind::ArcWrongCentre => {
            motions[1] = line([x0, at.y, z], [x1, at.y + depth, z]);
            Some(1)
        }
        // The pass stops short, leaving material.
        DefectKind::ExcessStock => {
            motions[1] = line([x0, at.y, z + depth], [x1, at.y, z + depth]);
            Some(1)
        }
    };
    let _ = locale;
    (clean, motions, segment)
}
