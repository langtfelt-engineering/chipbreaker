// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Stage two: words into a block, with modal groups enforced.
//!
//! # Modal groups are the architecture, not a detail
//!
//! Nearly everything in RS-274 is modal. `G1` stays in force until another
//! motion code replaces it; so does the plane, the units, the distance mode, the
//! active work offset, the feed mode, and the canned cycle. A bare `X10.` line
//! inherits all of it. That is why the parser is a state machine rather than a
//! translator, and why the state is one explicit struct snapshotted per block
//! rather than a scattering of mutable variables.
//!
//! Codes are partitioned into **groups**, and at most one code from a group may
//! appear in a block. `G0 G1 X10.` is not "the last one wins" — it is a
//! programming error, and which motion a real control performs depends on the
//! control. Encoding the groups explicitly is what turns that from a silent
//! divergence into a named error.
//!
//! # This stage still does not resolve anything
//!
//! It sorts the words of one line into slots and checks the group rule. It does
//! not apply modality, does not know where the tool is, and does not produce
//! motion. Keeping it to that is what makes it testable on a line at a time.

use crate::diag::{GcodeError, Site};
use crate::lex::{RawBlock, Word};

/// A modal group: a set of codes of which at most one may appear per block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ModalGroup {
    /// `G0 G1 G2 G3 G33 G38.x G73 G76 G80..G89` — motion, including cycles.
    Motion,
    /// `G17 G18 G19` — plane selection.
    Plane,
    /// `G90 G91` — absolute or incremental distance.
    Distance,
    /// `G90.1 G91.1` — how `I`/`J`/`K` are read.
    ArcDistance,
    /// `G93 G94 G95` — feed rate mode.
    FeedMode,
    /// `G20 G21` — units.
    Units,
    /// `G40 G41 G42` — cutter radius compensation.
    CutterComp,
    /// `G43 G44 G49` — tool length offset.
    ToolLength,
    /// `G98 G99` — canned cycle return level.
    CycleReturn,
    /// `G54..G59.3` — work offset.
    WorkOffset,
    /// `G61 G61.1 G64` — path control mode.
    PathControl,
    /// Non-modal codes: `G4 G10 G28 G30 G53 G92 G92.1 G92.2 G92.3`.
    NonModal,
    /// `M0 M1 M2 M30 M60` — program flow.
    Stopping,
    /// `M3 M4 M5` — spindle.
    Spindle,
    /// `M6` — tool change.
    ToolChange,
    /// `M7 M8 M9` — coolant.
    Coolant,
}

impl ModalGroup {
    /// Name used in error messages.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Motion => "motion codes",
            Self::Plane => "plane selections",
            Self::Distance => "distance modes",
            Self::ArcDistance => "arc centre modes",
            Self::FeedMode => "feed rate modes",
            Self::Units => "unit selections",
            Self::CutterComp => "cutter compensation modes",
            Self::ToolLength => "tool length offset modes",
            Self::CycleReturn => "canned cycle return modes",
            Self::WorkOffset => "work offsets",
            Self::PathControl => "path control modes",
            Self::NonModal => "non-modal codes",
            Self::Stopping => "program flow codes",
            Self::Spindle => "spindle codes",
            Self::ToolChange => "tool change codes",
            Self::Coolant => "coolant codes",
        }
    }
}

/// Which group a `G` code belongs to, keyed by code times ten.
///
/// `None` for a code this dialect does not implement, which the caller turns
/// into [`GcodeError::UnsupportedCode`] rather than ignoring.
#[must_use]
pub fn g_group(key: u32) -> Option<ModalGroup> {
    Some(match key {
        // Motion. G73/G74/G76 and G81..G89 are canned cycles, which are in the
        // motion group precisely because they *are* motion once expanded.
        0 | 10 | 20 | 30 | 330 | 331 | 382 | 383 | 384 | 385 | 730 | 740 | 760 | 800..=890 => {
            ModalGroup::Motion
        }
        170 | 180 | 190 => ModalGroup::Plane,
        900 | 910 => ModalGroup::Distance,
        901 | 911 => ModalGroup::ArcDistance,
        930 | 940 | 950 => ModalGroup::FeedMode,
        200 | 210 => ModalGroup::Units,
        400 | 410 | 420 => ModalGroup::CutterComp,
        430 | 431 | 440 | 490 => ModalGroup::ToolLength,
        980 | 990 => ModalGroup::CycleReturn,
        // Exactly the standard nine, and no range shorthand: `540..=593` would
        // also swallow G54.1, which is Fanuc's *extended* offset set addressed
        // by a P word. Accepting it here would silently treat it as G54.
        540 | 550 | 560 | 570 | 580 | 590 | 591 | 592 | 593 => ModalGroup::WorkOffset,
        610 | 611 | 640 => ModalGroup::PathControl,
        40 | 100 | 280 | 281 | 300 | 301 | 530 | 920 | 921 | 922 | 923 => ModalGroup::NonModal,
        _ => return None,
    })
}

/// Which group an `M` code belongs to.
///
/// Unrecognised M-codes are *not* an error: shops use them for machine-specific
/// functions and a verification tool that refused a program for using M55 would
/// be refusing most real programs. They are recorded as warnings instead.
#[must_use]
pub fn m_group(key: u32) -> Option<ModalGroup> {
    Some(match key {
        0 | 10 | 20 | 300 | 600 => ModalGroup::Stopping,
        30 | 40 | 50 => ModalGroup::Spindle,
        60 => ModalGroup::ToolChange,
        70 | 80 | 90 => ModalGroup::Coolant,
        _ => return None,
    })
}

/// The axis letters this dialect resolves into coordinates.
///
/// `U`, `V` and `W` are incremental synonyms on lathes and are deliberately
/// absent: this is a milling parser, and accepting them silently would be
/// accepting a lathe program as though it were a mill program.
pub const AXIS_LETTERS: [char; 6] = ['X', 'Y', 'Z', 'A', 'B', 'C'];

/// One line, sorted into slots.
///
/// Everything is optional because everything is modal: an empty slot means
/// "unchanged", not "zero".
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Block {
    /// `G` codes present, as code keys, in the order written.
    pub g_codes: Vec<u32>,
    /// `M` codes present, as code keys, in the order written.
    pub m_codes: Vec<u32>,
    /// Axis words: X, Y, Z, A, B, C.
    pub axes: [Option<Word>; 6],
    /// Arc centre offsets, I, J, K.
    pub ijk: [Option<Word>; 3],
    /// `R`: arc radius, or canned cycle retract plane. Which one depends on the
    /// motion code, which is why the block keeps the word rather than a meaning.
    pub r: Option<Word>,
    /// `F`: feed rate.
    pub f: Option<Word>,
    /// `S`: spindle speed.
    pub s: Option<Word>,
    /// `T`: tool select.
    pub t: Option<Word>,
    /// `H`: tool length offset number.
    pub h: Option<Word>,
    /// `D`: cutter compensation number.
    pub d: Option<Word>,
    /// `P`: dwell time, subprogram number, `G10` offset number, `G64` tolerance.
    pub p: Option<Word>,
    /// `Q`: peck depth.
    pub q: Option<Word>,
    /// `L`: repeat count, or `G10` mode.
    pub l: Option<Word>,
    /// `N`: line number. A label, never an ordering.
    pub n: Option<Word>,
    /// `O`: program or subprogram number.
    pub o: Option<Word>,
    /// True if the line began with `/`.
    pub block_skip: bool,
    /// Where the line is.
    pub site: Site,
    /// Comments on the line.
    pub comments: Vec<String>,
}

impl Block {
    /// True if the block carries any axis word.
    #[must_use]
    pub fn has_axis_words(&self) -> bool {
        self.axes.iter().any(Option::is_some)
    }

    /// True if the block carries any word at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.g_codes.is_empty()
            && self.m_codes.is_empty()
            && !self.has_axis_words()
            && self.ijk.iter().all(Option::is_none)
            && self.r.is_none()
            && self.f.is_none()
            && self.s.is_none()
            && self.t.is_none()
            && self.h.is_none()
            && self.d.is_none()
            && self.p.is_none()
            && self.q.is_none()
            && self.l.is_none()
            && self.o.is_none()
    }

    /// The `G` code from `group`, if the block has one.
    #[must_use]
    pub fn g_in(&self, group: ModalGroup) -> Option<u32> {
        self.g_codes
            .iter()
            .copied()
            .find(|k| g_group(*k) == Some(group))
    }

    /// True if the block contains this exact `G` code key.
    #[must_use]
    pub fn has_g(&self, key: u32) -> bool {
        self.g_codes.contains(&key)
    }

    /// True if the block contains this exact `M` code key.
    #[must_use]
    pub fn has_m(&self, key: u32) -> bool {
        self.m_codes.contains(&key)
    }
}

/// Renders a code key the way it was written: `10` becomes `G1`, `591` becomes
/// `G59.1`.
#[must_use]
pub fn render_code(letter: char, key: u32) -> String {
    if key.is_multiple_of(10) {
        format!("{letter}{}", key / 10)
    } else {
        format!("{letter}{}.{}", key / 10, key % 10)
    }
}

/// Assembles one lexed line into a block, enforcing the modal group rule.
///
/// # Errors
///
/// [`GcodeError::ModalGroupConflict`] when two codes from one group appear,
/// [`GcodeError::UnsupportedCode`] for a `G` code this dialect does not
/// implement, and [`GcodeError::CutterCompensation`] for `G41`/`G42`.
///
/// The cutter-compensation refusal is here rather than later because it must
/// happen even if the block does nothing else: `G41` on its own arms the control
/// for every move that follows.
pub fn assemble(raw: &RawBlock) -> Result<Block, GcodeError> {
    let mut block = Block {
        block_skip: raw.block_skip,
        site: Site::line_only(raw.file, raw.line),
        comments: raw.comments.clone(),
        ..Block::default()
    };

    // Group -> the codes from it already seen on this line.
    let mut seen: Vec<(ModalGroup, Vec<String>)> = Vec::new();
    let note = |group: ModalGroup, text: String, seen: &mut Vec<(ModalGroup, Vec<String>)>| {
        if let Some(entry) = seen.iter_mut().find(|(g, _)| *g == group) {
            entry.1.push(text);
        } else {
            seen.push((group, vec![text]));
        }
    };

    for word in &raw.words {
        match word.letter {
            'G' => {
                let key = word.code_key();
                if key == 410 || key == 420 {
                    return Err(GcodeError::CutterCompensation {
                        site: word.site,
                        code: key / 10,
                    });
                }
                let Some(group) = g_group(key) else {
                    return Err(GcodeError::UnsupportedCode {
                        site: word.site,
                        code: render_code('G', key),
                    });
                };
                // Non-modal codes may legitimately share a block with each
                // other -- `G53 G0 X0` is one code from Motion and one from
                // NonModal -- but two from the same group still conflict.
                note(group, render_code('G', key), &mut seen);
                block.g_codes.push(key);
            }
            'M' => {
                let key = word.code_key();
                if let Some(group) = m_group(key) {
                    note(group, render_code('M', key), &mut seen);
                }
                block.m_codes.push(key);
            }
            letter => {
                let slot = match letter {
                    'X' => Some(&mut block.axes[0]),
                    'Y' => Some(&mut block.axes[1]),
                    'Z' => Some(&mut block.axes[2]),
                    'A' => Some(&mut block.axes[3]),
                    'B' => Some(&mut block.axes[4]),
                    'C' => Some(&mut block.axes[5]),
                    'I' => Some(&mut block.ijk[0]),
                    'J' => Some(&mut block.ijk[1]),
                    'K' => Some(&mut block.ijk[2]),
                    'R' => Some(&mut block.r),
                    'F' => Some(&mut block.f),
                    'S' => Some(&mut block.s),
                    'T' => Some(&mut block.t),
                    'H' => Some(&mut block.h),
                    'D' => Some(&mut block.d),
                    'P' => Some(&mut block.p),
                    'Q' => Some(&mut block.q),
                    'L' => Some(&mut block.l),
                    'N' => Some(&mut block.n),
                    'O' => Some(&mut block.o),
                    _ => None,
                };
                match slot {
                    // A repeated word is last-one-wins on real controls, and
                    // unlike a modal group conflict it is unambiguous.
                    Some(slot) => *slot = Some(word.clone()),
                    None => {
                        return Err(GcodeError::UnknownWord {
                            site: word.site,
                            letter,
                        });
                    }
                }
            }
        }
    }

    for (group, codes) in seen {
        if codes.len() > 1 {
            return Err(GcodeError::ModalGroupConflict {
                site: block.site,
                group: group.name(),
                codes,
            });
        }
    }

    Ok(block)
}
