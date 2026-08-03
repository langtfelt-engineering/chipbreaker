// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Canned cycle expansion.
//!
//! # The contract is the longhand
//!
//! A cycle expands to exactly the motion a programmer would have written out by
//! hand. That is not a description of the implementation, it is the test:
//! `tests/cycles.rs` writes each cycle both ways and asserts the resulting
//! segments are geometrically identical. Longhand has nowhere to hide a dialect
//! assumption, so anywhere the two disagree, one of them is wrong and it is
//! usually the cycle.
//!
//! # The shape of every cycle
//!
//! ```text
//! 1. rapid to (x, y) at whatever Z the tool is at
//! 2. rapid down to the R plane
//! 3. feed to the bottom      <- the only part that differs between cycles
//! 4. retract                 <- to the initial Z (G98) or the R plane (G99)
//! ```
//!
//! Step 4 is the one that changes every intermediate retract in a multi-hole
//! pattern, and therefore whether the tool clears a clamp between holes.
//!
//! # Two machine parameters this deliberately does not invent
//!
//! **`G73`'s chip-break retract.** A real control retracts a small distance
//! between pecks, set by a machine parameter that is not in the NC file. It is
//! not modelled, and `G73` expands as a straight feed to depth. That is exact
//! for material removal — the retract goes back into space already cut and
//! returns, removing nothing — and the difference is visible only to timing.
//! Inventing a clearance would put a rapid in the IR that the machine may not
//! make.
//!
//! **`G83`'s re-entry clearance.** A real control rapids back down to just above
//! the previous depth. "Just above" is again a parameter, so the rapid returns
//! to exactly the previous depth. The space is already cut, so the move is
//! through air either way, and choosing the parameter-free value keeps the
//! expansion reproducible.
//!
//! `G83`'s full retract to the R plane *is* modelled, because that one is
//! unambiguous, it is a real move through real space, and it is exactly the move
//! that hits a clamp.

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{MotionKind, Provenance};

use crate::diag::Site;

/// Most pecks one cycle firing may produce.
///
/// A `G83` with `Q0.0001` over a 100 mm depth asks for a million pecks and three
/// million segments, from one line of NC. That is not a program anybody wrote on
/// purpose -- it is a decimal point in the wrong place -- and producing the IR
/// for it would exhaust memory before anything could report the mistake.
///
/// Chosen well above any real drilling operation: a 300 mm hole pecked 0.1 mm at
/// a time is 3000, and both of those numbers are already implausible.
pub const MAX_PECKS: usize = 10_000;

/// Which cycle, and what its bottom motion looks like.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CycleKind {
    /// `G81`: feed to depth, retract.
    Drill,
    /// `G82`: feed to depth, dwell, retract.
    DrillDwell,
    /// `G83`: peck with a full retract to the R plane between pecks.
    PeckFullRetract,
    /// `G73`: peck with a chip-break retract, which is not modelled; see the
    /// module header.
    PeckChipBreak,
    /// `G85`: feed to depth, **feed** back out.
    Bore,
    /// `G86`: feed to depth, spindle stops, rapid out.
    BoreSpindleStop,
    /// `G84`/`G74`: tapping. Geometrically a bore — feed in, feed out — because
    /// the spindle synchronisation removes no extra material.
    Tap,
}

impl CycleKind {
    /// From a motion-group code key.
    #[must_use]
    pub const fn from_key(key: u32) -> Option<Self> {
        Some(match key {
            730 => Self::PeckChipBreak,
            740 | 840 => Self::Tap,
            810 => Self::Drill,
            820 => Self::DrillDwell,
            830 => Self::PeckFullRetract,
            850 => Self::Bore,
            860 => Self::BoreSpindleStop,
            870 | 880 | 890 => Self::Bore,
            _ => return None,
        })
    }

    /// True if the retract from the bottom is at feed rate rather than rapid.
    #[must_use]
    pub const fn retracts_at_feed(self) -> bool {
        matches!(self, Self::Bore | Self::Tap)
    }

    /// True if the cycle pecks.
    #[must_use]
    pub const fn pecks(self) -> bool {
        matches!(self, Self::PeckFullRetract | Self::PeckChipBreak)
    }
}

/// One motion of an expanded cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CycleMove {
    /// Rapid or feed.
    pub kind: MotionKind,
    /// Where it ends, in machine coordinates.
    pub to: Vec3,
}

/// Everything an expansion needs.
#[derive(Debug, Clone, Copy)]
pub struct CycleRequest {
    /// Which cycle.
    pub kind: CycleKind,
    /// Where the tool is now, in machine coordinates.
    pub from: Vec3,
    /// The hole's position, in machine coordinates. Its Z is ignored.
    pub hole: Vec3,
    /// Bottom of the hole, machine Z.
    pub bottom: f64,
    /// Retract plane, machine Z.
    pub r_plane: f64,
    /// Z the cycle started from, machine Z, for `G98`.
    pub initial_z: f64,
    /// Retract to the initial Z rather than the R plane.
    pub return_to_initial: bool,
    /// Peck depth in millimetres, for `G73`/`G83`.
    pub peck: Option<f64>,
    /// Where the block is.
    pub site: Site,
}

/// Why a cycle could not be expanded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CycleError {
    /// The peck depth would produce more than [`MAX_PECKS`] pecks.
    TooManyPecks {
        /// How many it asked for.
        wanted: usize,
    },
}

/// Expands one firing of a cycle into its longhand motion.
///
/// The result is exactly what a programmer would have written; see the module
/// header, and `tests/cycles.rs` which asserts it.
///
/// # Errors
/// [`CycleError::TooManyPecks`] for a peck depth so small that the expansion
/// would be unbounded; see [`MAX_PECKS`].
pub fn expand(request: &CycleRequest) -> Result<Vec<CycleMove>, CycleError> {
    if request.kind.pecks()
        && let Some(depth) = request.peck
        && depth > 0.0
    {
        let travel = (request.r_plane - request.bottom).abs();
        let wanted = (travel / depth).ceil();
        if wanted > MAX_PECKS as f64 {
            return Err(CycleError::TooManyPecks {
                wanted: wanted as usize,
            });
        }
    }
    Ok(expand_unchecked(request))
}

#[must_use]
fn expand_unchecked(request: &CycleRequest) -> Vec<CycleMove> {
    let mut moves = Vec::with_capacity(6);
    let axis = |z: f64| Vec3::new(request.hole.x, request.hole.y, z);

    // 1. Position over the hole, at whatever height the tool is already at.
    moves.push(CycleMove {
        kind: MotionKind::Rapid,
        to: Vec3::new(request.hole.x, request.hole.y, request.from.z),
    });

    // 2. Down to the R plane.
    moves.push(CycleMove {
        kind: MotionKind::Rapid,
        to: axis(request.r_plane),
    });

    // 3. The bottom motion, which is the only part that differs between cycles.
    match (request.kind.pecks(), request.peck) {
        (true, Some(depth)) if depth > 0.0 => {
            let mut current = request.r_plane;
            while current > request.bottom {
                let next = (current - depth).max(request.bottom);
                moves.push(CycleMove {
                    kind: MotionKind::Linear,
                    to: axis(next),
                });
                if next > request.bottom && request.kind == CycleKind::PeckFullRetract {
                    // Full retract to clear the swarf, then back down to where
                    // the peck ended. See the module header for why there is no
                    // re-entry clearance.
                    moves.push(CycleMove {
                        kind: MotionKind::Rapid,
                        to: axis(request.r_plane),
                    });
                    moves.push(CycleMove {
                        kind: MotionKind::Rapid,
                        to: axis(next),
                    });
                }
                current = next;
            }
        }
        _ => {
            moves.push(CycleMove {
                kind: MotionKind::Linear,
                to: axis(request.bottom),
            });
        }
    }

    // 4. Retract.
    let retract_to = if request.return_to_initial {
        request.initial_z
    } else {
        request.r_plane
    };
    moves.push(CycleMove {
        kind: if request.kind.retracts_at_feed() {
            MotionKind::Linear
        } else {
            MotionKind::Rapid
        },
        to: axis(retract_to),
    });

    moves
}

/// Tags each move of an expansion with its step, so a report can say which of
/// the three motions on line 42 was at fault.
#[must_use]
pub fn provenance_for(base: Provenance, step: usize) -> Provenance {
    base.in_cycle(u32::try_from(step).unwrap_or(u32::MAX - 1))
}
