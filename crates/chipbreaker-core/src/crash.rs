// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Planted crashes, at known coordinates.
//!
//! # Why this is a separate corpus from the injected defects
//!
//! The defect corpus perturbs a segment and asks whether the deviation field
//! recovers the depth. Its ground truth is a *measurement* against the nominal
//! part.
//!
//! A crash has no nominal part in it. The truth is that at some point on some
//! segment, a piece of the tool that is not supposed to touch anything is inside
//! something solid — and whether the finished part would have been within
//! tolerance is beside the point, because the machine stopped. So the ground
//! truth here is **geometric and constructed**: the case states which element
//! should be found in which obstacle, and it is arranged so that it is true by
//! construction rather than by measurement.
//!
//! # The exit criterion is asymmetric, on purpose
//!
//! **Zero missed collisions.** A missed collision is a spindle, and there is no
//! recovering the credibility of a checker that has missed one.
//!
//! False positives are measured and reported rather than forbidden outright,
//! because the cure for them is usually to make the check less sensitive and
//! that trade is exactly the wrong one to make quietly. The clean half of the
//! corpus is what holds that line: every case has a partner that differs only in
//! the one dimension that matters, so a checker cannot pass by reporting
//! everything.
//!
//! # What this cannot tell you
//!
//! That the machine will crash. This is the ideal geometric model: no
//! deflection, no runout, no thermal growth, and no controller smoothing the
//! corner. A program clean here can still crash for any of those reasons, and
//! one that collides here would collide on any machine.

use crate::math::Vec3;
use crate::sweep::{LinearMove, Motion};
use crate::tool::Profile;
use crate::tool::catalog::{HolderStage, Shank, flat_end_mill};
use crate::tool::profile::ElementRole;

/// The block every case is cut from, in millimetres.
pub const STOCK: [f64; 3] = [80.0, 50.0, 40.0];

/// How far every case stands clear of its own collide/clear boundary.
///
/// Several cells. A case a fraction of a cell from the crossover measures the
/// sampling floor rather than the geometry, and the two have different right
/// answers -- so the corpus declines to contain one at all rather than scoring
/// it and explaining the result away afterwards.
pub const MARGIN: f64 = 3.0;

/// Flute length for the reach family, where only the chuck's height varies.
const REACH_FLUTE: f64 = 6.0;

/// What kind of crash a case represents.
///
/// Named for what the programmer did, not for the geometry, because that is how
/// it has to be explained back to them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CrashKind {
    /// The pocket is deeper than the tool has flute, so the shank rubs.
    ShankInPocketWall,
    /// The pocket is deeper than the tool has reach, so the chuck goes in.
    HolderIntoFloor,
    /// A rapid crosses a feature that was never cleared.
    RapidAcrossUnclearedStock,
    /// The holder sweeps into a clamp standing beside the part.
    HolderIntoClamp,
    /// The same cut with enough reach to clear. **No collision expected.**
    Clean,
}

impl CrashKind {
    /// The name used in the corpus file.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ShankInPocketWall => "shank-in-pocket-wall",
            Self::HolderIntoFloor => "holder-into-floor",
            Self::RapidAcrossUnclearedStock => "rapid-across-uncleared-stock",
            Self::HolderIntoClamp => "holder-into-clamp",
            Self::Clean => "clean",
        }
    }

    /// Whether a collision is expected at all.
    #[must_use]
    pub const fn collides(self) -> bool {
        !matches!(self, Self::Clean)
    }

    /// Which element must be found, for the cases that collide.
    #[must_use]
    pub const fn element(self) -> Option<ElementRole> {
        match self {
            Self::ShankInPocketWall | Self::RapidAcrossUnclearedStock => {
                Some(ElementRole::NonCutting)
            }
            Self::HolderIntoFloor | Self::HolderIntoClamp => Some(ElementRole::Holder),
            Self::Clean => None,
        }
    }
}

/// One planted crash, or one deliberately clean partner for it.
#[derive(Debug, Clone)]
pub struct CrashCase {
    /// Stable identifier, used in the corpus file and in failure messages.
    pub id: String,
    /// What this case is.
    pub kind: CrashKind,
    /// The program.
    pub motions: Vec<Motion>,
    /// Flute length, in millimetres.
    pub flute_mm: f64,
    /// Height of the top of the shank above the tip.
    pub shank_top_mm: f64,
    /// A clamp, as `(min, max)` in machine coordinates, when the case has one.
    pub clamp: Option<(Vec3, Vec3)>,
    /// Which segment is expected to carry the collision, when one is.
    pub segment: Option<usize>,
    /// What the case is for, in one sentence.
    pub why: String,
}

impl CrashCase {
    /// The tool this case runs, holder and all.
    ///
    /// Always a real chuck: a tool without one cannot be collision-checked at
    /// all, and a corpus built from those would measure nothing.
    ///
    /// # Panics
    ///
    /// If a case's dimensions do not form a valid tool — a shank beginning below
    /// the top of its own flutes, say. That is a fault in the corpus rather than
    /// in the caller, and it should stop the run loudly: a corpus that silently
    /// skipped its malformed cases would quietly shrink and still report a clean
    /// sweep over whatever was left.
    #[must_use]
    pub fn profile(&self) -> Profile {
        flat_end_mill(
            6.0,
            self.flute_mm,
            &Shank::with_holder(
                6.0,
                self.shank_top_mm,
                [
                    // A 2 in nut over a 2 7/16 in body: the ER32 chuck from the
                    // tool corpus, at the same dimensions.
                    HolderStage::cylinder(50.8, 28.0),
                    HolderStage::cylinder(61.912499999999994, 50.0),
                ],
            ),
        )
        .expect("the corpus geometry is valid by construction")
    }
}

fn linear(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(a[0], a[1], a[2]),
        end: Vec3::new(b[0], b[1], b[2]),
    })
}

/// A pocket: rapid in above the stock, plunge, feed across, retract.
fn pocket(x0: f64, x1: f64, y: f64, z: f64) -> Vec<Motion> {
    vec![
        linear([x0, y, 70.0], [x0, y, 70.0 - 1.0e-9]),
        linear([x0, y, 70.0], [x0, y, z]),
        linear([x0, y, z], [x1, y, z]),
        linear([x1, y, z], [x1, y, 70.0]),
    ]
}

/// The whole crash corpus.
///
/// Every colliding case is paired with a clean one that differs **only** in
/// reach, so a checker that reported everything would fail the clean half and a
/// checker that reported nothing would fail the colliding half. Neither
/// degenerate answer passes.
#[must_use]
#[allow(clippy::too_many_lines, reason = "a corpus is a list of cases")]
pub fn corpus() -> Vec<CrashCase> {
    let mut out = Vec::new();

    // --- Depth against flute length -------------------------------------
    //
    // The stock top is at z = 40. A pocket floor at `z` is `40 - z` deep. With
    // `flute` millimetres of flute, the shank starts `flute` above the tip, so
    // it is inside the wall whenever `40 - z > flute`.
    // A sweep rather than a handful of hand-picked pairs. Hand-picked cases
    // cluster around whatever the author was thinking about; a sweep crosses the
    // boundary from both sides at several depths, which is where an off-by-one
    // in the reach arithmetic would actually live.
    // Every case stands at least `MARGIN` clear of its own boundary. A case
    // sitting a fraction of a cell from the crossover is not testing the reach
    // arithmetic -- it is testing the sampling floor, which is a different
    // measurement with a different right answer, and mixing the two would make
    // "zero missed" mean "zero missed except where we were not really asking".
    //
    // Balanced by construction rather than by taking the first thirty of a
    // sweep. The sweep runs shallow-to-deep, so a plain truncation would have
    // returned almost nothing but collisions and left the false-positive half of
    // the corpus too thin to constrain anything.
    let (mut colliding, mut clear) = (Vec::new(), Vec::new());
    for z in [4.0, 6.0, 8.0, 10.0, 14.0, 18.0, 22.0, 26.0, 30.0] {
        for flute in [4.0, 8.0, 12.0, 18.0, 24.0, 30.0, 36.0] {
            // The shank begins `flute` above the tip, so it enters the wall when
            // `z + flute` is below the top face.
            if (z + flute - STOCK[2]).abs() < MARGIN {
                continue;
            }
            if z + flute < STOCK[2] {
                colliding.push((z, flute));
            } else {
                clear.push((z, flute));
            }
        }
    }
    colliding.truncate(15);
    clear.truncate(15);
    let mut pairs = colliding;
    pairs.extend(clear);
    for (i, (z, flute)) in pairs.into_iter().enumerate() {
        let depth = STOCK[2] - z;
        let collides = z + flute < STOCK[2];
        #[allow(clippy::cast_precision_loss, reason = "a hundred cases")]
        let y = 10.0 + ((i % 10) as f64) * 3.0;
        out.push(CrashCase {
            id: format!("pocket-depth-{i}"),
            kind: if collides {
                CrashKind::ShankInPocketWall
            } else {
                CrashKind::Clean
            },
            motions: pocket(15.0, 60.0, y, z),
            flute_mm: flute,
            // Well clear, so only the shank can be at fault here.
            shank_top_mm: 120.0,
            clamp: None,
            segment: Some(2),
            why: format!(
                "a {depth:.0} mm pocket cut with {flute:.0} mm of flute: the shank is \
                 {} the wall",
                if collides { "inside" } else { "clear of" }
            ),
        });
    }

    // --- Reach against depth, with the chuck ----------------------------
    //
    // Flute always covers the depth, so the shank is never at fault. What
    // varies is how far the chuck sits above the tip.
    // A short flute throughout, so what varies is only where the chuck sits.
    // An earlier sweep scaled the flute with the depth and produced tools whose
    // shank began below the top of their own flutes -- geometry the profile
    // validator refuses, and rightly, but which made the corpus unbuildable
    // rather than making a point.
    // **The chuck cannot be below the top face while the shank is above it**,
    // because the chuck sits on top of the shank. So a case where the chuck is
    // in the block always has shank contact as well, and labelling those clean
    // -- as an earlier version did, on the grounds that the chuck cleared --
    // marked genuine shank collisions as false positives.
    //
    // Each case is therefore classified by what is actually inside: the chuck if
    // it reaches, the shank if only it does, and clean only when both are above
    // the top face.
    let mut reaches = Vec::new();
    for z in [4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 30.0, 36.0] {
        for shank_top in [10.0, 14.0, 18.0, 26.0, 34.0, 90.0] {
            if shank_top >= REACH_FLUTE
                && (z + shank_top - STOCK[2]).abs() >= MARGIN
                && (z + REACH_FLUTE - STOCK[2]).abs() >= MARGIN
            {
                reaches.push((z, shank_top));
            }
        }
    }
    reaches.truncate(25);
    for (i, (z, shank_top)) in reaches.into_iter().enumerate() {
        let depth = STOCK[2] - z;
        let holder_in = z + shank_top < STOCK[2];
        let shank_in = z + REACH_FLUTE < STOCK[2];
        #[allow(clippy::cast_precision_loss, reason = "a hundred cases")]
        let y = 10.0 + ((i % 10) as f64) * 3.0;
        out.push(CrashCase {
            id: format!("chuck-reach-{i}"),
            kind: if holder_in {
                CrashKind::HolderIntoFloor
            } else if shank_in {
                CrashKind::ShankInPocketWall
            } else {
                CrashKind::Clean
            },
            motions: pocket(15.0, 60.0, y, z),
            flute_mm: REACH_FLUTE,
            shank_top_mm: shank_top,
            clamp: None,
            segment: Some(2),
            why: format!(
                "a {depth:.0} mm pocket with the chuck {shank_top:.0} mm above the tip:                  chuck {} the block, shank {}",
                if holder_in { "in" } else { "above" },
                if shank_in { "in" } else { "above" }
            ),
        });
    }

    // --- Rapids across uncleared stock ----------------------------------
    //
    // A rapid at a height that clears nothing, which is the move that does the
    // most damage because it happens at traverse rate.
    // A 4 mm flute, so the shank starts 4 mm above the tip and the rapid is
    // dangerous only once `z + 4` is below the top face. An earlier version
    // asked whether the *tip* was below it, which counted cases where nothing
    // but the cutting edge was in the stock -- and cutting geometry is not what
    // this check reports.
    const RAPID_FLUTE: f64 = 4.0;
    let heights: Vec<f64> = [
        8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 30.0, 32.0, 33.0, 42.0, 44.0, 46.0, 48.0, 50.0, 55.0,
        60.0, 65.0, 70.0, 75.0, 80.0,
    ]
    .into_iter()
    .filter(|z| (z + RAPID_FLUTE - STOCK[2]).abs() >= MARGIN)
    .collect();
    for (i, z) in heights.into_iter().enumerate() {
        let collides = z + RAPID_FLUTE < STOCK[2];
        out.push(CrashCase {
            id: format!("rapid-height-{i}"),
            kind: if collides {
                CrashKind::RapidAcrossUnclearedStock
            } else {
                CrashKind::Clean
            },
            motions: vec![
                linear([5.0, 25.0, 70.0], [5.0, 25.0, z]),
                linear([5.0, 25.0, z], [75.0, 25.0, z]),
                linear([75.0, 25.0, z], [75.0, 25.0, 70.0]),
            ],
            flute_mm: RAPID_FLUTE,
            shank_top_mm: 120.0,
            clamp: None,
            segment: Some(1),
            why: format!(
                "a rapid straight across the block at z = {z:.0}, which is {} the top face",
                if collides { "below" } else { "above" }
            ),
        });
    }

    // --- Chuck into a clamp ---------------------------------------------
    //
    // The part is fine; the fixture is in the way. This is the case a
    // part-only comparison can never find, whatever its resolution.
    // **The widest part of the chuck is not the part that reaches the clamp.**
    //
    // The tip sits at z = 36 with 6 mm of flute and the shank top 10 mm up, so
    // the nut spans z = 46 to 74 and the body above it starts at 74. The clamp
    // only stands to z = 60, so the geometry that can touch it is the Ø50.8 nut
    // and never the Ø61.9125 body.
    //
    // An earlier version took the body's radius and put the boundary 5.6 mm too
    // far out, which labelled a genuinely clear case as a collision and read the
    // correct answer as a miss. The corpus was wrong, not the checker.
    const CHUCK_REACH_X: f64 = 70.0 + 50.8 / 2.0;
    let clamps: Vec<f64> = [
        68.0, 70.0, 72.0, 74.0, 76.0, 78.0, 80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 99.0, 102.0,
        105.0, 108.0, 112.0, 118.0, 125.0, 132.0, 140.0, 150.0, 160.0, 170.0,
    ]
    .into_iter()
    .filter(|x| (x - CHUCK_REACH_X).abs() >= MARGIN)
    .collect();
    for (i, clamp_x) in clamps.into_iter().enumerate() {
        let collides = clamp_x < CHUCK_REACH_X;
        out.push(CrashCase {
            id: format!("clamp-{i}"),
            kind: if collides {
                CrashKind::HolderIntoClamp
            } else {
                CrashKind::Clean
            },
            motions: pocket(20.0, 70.0, 25.0, 36.0),
            flute_mm: 6.0,
            // Low chuck, so it sweeps at clamp height.
            shank_top_mm: 10.0,
            clamp: Some((
                Vec3::new(clamp_x, 18.0, 0.0),
                Vec3::new(clamp_x + 14.0, 32.0, 60.0),
            )),
            segment: Some(2),
            why: format!(
                "a clamp at x = {clamp_x:.0}, which the chuck's envelope {}",
                if collides {
                    "reaches"
                } else {
                    "stops short of"
                }
            ),
        });
    }

    out
}

/// How many cases expect a collision.
#[must_use]
pub fn colliding(cases: &[CrashCase]) -> usize {
    cases.iter().filter(|c| c.kind.collides()).count()
}
