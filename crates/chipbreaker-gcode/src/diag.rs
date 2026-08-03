// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Errors and warnings, all of which name a place in a file.
//!
//! # Two severities, and why the line matters more than either
//!
//! An **error** stops the parse: the program cannot be turned into a toolpath
//! without guessing, and guessing is the failure mode this project has already
//! decided never to accept.
//!
//! A **warning** is something the parse survived but a careful reader should
//! know about — an arc whose radius mismatch was inside tolerance, a comment
//! that never closed, an M-code we do not model. Warnings accumulate rather than
//! abort, and `--strict` promotes them.
//!
//! Every one of them carries a file, a line and usually a column. That is not
//! politeness. A user cannot act on "unsupported construct"; they can act on
//! "line 4192, column 7: G41 cutter compensation". The same reasoning that puts
//! [`chipbreaker_core::toolpath::Provenance`] on every segment applies here.

use core::fmt;

/// A position in a source file.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Site {
    /// Index into the parse's file table.
    pub file: u32,
    /// One-based line.
    pub line: u32,
    /// One-based column, or zero when the whole line is meant.
    pub column: u32,
}

impl Site {
    /// A position.
    #[must_use]
    pub const fn new(file: u32, line: u32, column: u32) -> Self {
        Self { file, line, column }
    }

    /// A whole line.
    #[must_use]
    pub const fn line_only(file: u32, line: u32) -> Self {
        Self::new(file, line, 0)
    }
}

impl fmt::Display for Site {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.column == 0 {
            write!(f, "line {}", self.line)
        } else {
            write!(f, "line {}, column {}", self.line, self.column)
        }
    }
}

/// A language we recognise well enough to refuse by name.
///
/// Refusing by name rather than as a syntax error is the whole point: somebody
/// who feeds a Siemens program to a Fanuc parser has made a category error, and
/// "unexpected character at line 3" will not tell them so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForeignDialect {
    /// Siemens 840D: `CYCLE81`, R-parameters, `DEF`.
    Siemens840d,
    /// Heidenhain Klartext: `BEGIN PGM`, `L X.. Y.. R0 F..`, `TOOL CALL`.
    HeidenhainKlartext,
}

impl ForeignDialect {
    /// The name to put in front of a user.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Siemens840d => "Siemens 840D",
            Self::HeidenhainKlartext => "Heidenhain Klartext",
        }
    }
}

/// Why a program could not be turned into a toolpath.
#[derive(Debug, Clone, PartialEq)]
pub enum GcodeError {
    /// The file is written in a different language, not a dialect of this one.
    ForeignLanguage {
        /// Where the giveaway was.
        site: Site,
        /// Which language.
        dialect: ForeignDialect,
        /// The text that gave it away.
        evidence: String,
    },
    /// Macro or parametric programming: `#` variables, `IF`, `WHILE`, `GOTO`.
    ///
    /// Common in shop-written programs, absent from CAM output. Approximating it
    /// would mean evaluating expressions we do not implement, and a program
    /// whose coordinates come out of arithmetic we guessed at is a program whose
    /// verification means nothing.
    MacroProgramming {
        /// Where.
        site: Site,
        /// Which construct.
        construct: String,
    },
    /// LinuxCNC `o`-word subprograms and flow control.
    OWord {
        /// Where.
        site: Site,
        /// The word as written.
        word: String,
    },
    /// `G41`/`G42` cutter radius compensation.
    ///
    /// The control offsets the programmed path by the tool radius, inserting
    /// lead-in, lead-out and corner geometry whose details differ between
    /// controls. Simulating the uncompensated path would produce a part wrong by
    /// the tool radius everywhere, and it would look plausible.
    CutterCompensation {
        /// Where.
        site: Site,
        /// 41 or 42.
        code: u32,
    },
    /// An axis word with no decimal point.
    ///
    /// On legacy Fanuc controls `X10` means ten least-input-increments — 0.010
    /// mm — and not ten millimetres. A factor of a thousand, and it parses
    /// perfectly. See `--legacy-increment`.
    MissingDecimalPoint {
        /// Where.
        site: Site,
        /// The word as written.
        word: String,
    },
    /// Two codes from the same modal group in one block.
    ///
    /// Not last-one-wins. `G0 G1 X10` is a programming error, and which motion
    /// the machine performs depends on the control.
    ModalGroupConflict {
        /// Where.
        site: Site,
        /// The group's name.
        group: &'static str,
        /// The codes that clashed, as written.
        codes: Vec<String>,
    },
    /// A number that is not finite, or not a number at all.
    NotANumber {
        /// Where.
        site: Site,
        /// The text.
        text: String,
    },
    /// A word whose letter means nothing in this dialect.
    UnknownWord {
        /// Where.
        site: Site,
        /// The letter.
        letter: char,
    },
    /// A `G` or `M` code we do not implement.
    UnsupportedCode {
        /// Where.
        site: Site,
        /// The code as written, such as `G33.1`.
        code: String,
    },
    /// `G43 H…` referenced a tool length offset the library does not define.
    ///
    /// The offset lives in the machine's tool table, which is not in the NC
    /// file. Assuming zero would put the tool tip in the wrong place by the
    /// whole length of the tool.
    UnknownToolOffset {
        /// Where.
        site: Site,
        /// The `H` number.
        h: u32,
    },
    /// A tool number with no entry in the library.
    UnknownTool {
        /// Where.
        site: Site,
        /// The `T` number.
        tool: u32,
    },
    /// An arc whose given centre is inconsistent with its endpoints beyond
    /// tolerance.
    ArcRadiusMismatch {
        /// Where.
        site: Site,
        /// Distance from the centre to the start.
        start_radius: f64,
        /// Distance from the centre to the end.
        end_radius: f64,
        /// The tolerance in force.
        tolerance: f64,
    },
    /// An `R`-form arc whose sweep is close enough to 180 degrees that the
    /// centre is not meaningfully determined.
    ///
    /// The centre of an `R` arc sits on the perpendicular bisector of the chord
    /// at a distance `sqrt(R^2 - (chord/2)^2)`. As the chord approaches `2R`
    /// that square root approaches zero and its derivative approaches infinity,
    /// so a rounding in the endpoints moves the centre arbitrarily far.
    ArcIllConditioned {
        /// Where.
        site: Site,
        /// Half the chord.
        half_chord: f64,
        /// The programmed radius.
        radius: f64,
    },
    /// An `R`-form arc whose endpoints coincide.
    ///
    /// With `I`/`J`/`K` that means a full circle. With `R` it means nothing at
    /// all: every circle of that radius through the point qualifies.
    FullCircleWithRadiusWord {
        /// Where.
        site: Site,
    },
    /// An arc whose radius is too small to reach between its endpoints.
    ArcRadiusTooSmall {
        /// Where.
        site: Site,
        /// Half the chord.
        half_chord: f64,
        /// The programmed radius.
        radius: f64,
    },
    /// Subprogram nesting exceeded the cap.
    SubprogramTooDeep {
        /// Where.
        site: Site,
        /// The cap.
        limit: u32,
    },
    /// `M98 P…` named a subprogram that does not exist.
    UnknownSubprogram {
        /// Where.
        site: Site,
        /// The `P` number.
        number: u32,
    },
    /// A canned cycle whose parameters cannot be expanded.
    BadCycle {
        /// Where.
        site: Site,
        /// What was wrong.
        detail: String,
    },
    /// A motion block that needs a feed rate but has never been given one.
    NoFeedRate {
        /// Where.
        site: Site,
    },
    /// The IR rejected what the resolver built. A bug here, not in the program.
    Ir {
        /// What the IR said.
        detail: String,
    },
    /// A warning promoted by `--strict`.
    Strict {
        /// The warning.
        warning: Box<GcodeWarning>,
    },
}

impl fmt::Display for GcodeError {
    #[allow(clippy::too_many_lines, reason = "one arm per variant, each short")]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ForeignLanguage {
                site,
                dialect,
                evidence,
            } => write!(
                f,
                "{site}: this looks like {}, which is a different language rather than \
                 a dialect of RS-274 (found {evidence:?})",
                dialect.name()
            ),
            Self::MacroProgramming { site, construct } => write!(
                f,
                "{site}: macro programming ({construct}) is not supported. Coordinates \
                 that come out of arithmetic this parser guessed at would make the \
                 verification meaningless"
            ),
            Self::OWord { site, word } => write!(
                f,
                "{site}: o-word subprograms ({word}) are LinuxCNC's procedural \
                 extension rather than Fanuc RS-274. M98/M99 subprograms are supported"
            ),
            Self::CutterCompensation { site, code } => write!(
                f,
                "{site}: G{code} cutter radius compensation is not supported. The \
                 control, not the program, decides the offset path, including lead-in \
                 and corner geometry that differs between controls; simulating the \
                 uncompensated path would be wrong by the tool radius everywhere. \
                 Post-process with compensation applied and G40 active"
            ),
            Self::MissingDecimalPoint { site, word } => write!(
                f,
                "{site}: {word} has no decimal point. On legacy controls that means \
                 least-input-increments, so {word} could be a thousandth of what it \
                 looks like. Write it with a decimal point, or pass \
                 --legacy-increment to say which the file means"
            ),
            Self::ModalGroupConflict { site, group, codes } => write!(
                f,
                "{site}: {} in one block, and they are all {group}. This is a \
                 programming error rather than a last-one-wins",
                codes.join(" and ")
            ),
            Self::NotANumber { site, text } => {
                write!(f, "{site}: {text:?} is not a finite number")
            }
            Self::UnknownWord { site, letter } => {
                write!(f, "{site}: {letter:?} is not a word letter in this dialect")
            }
            Self::UnsupportedCode { site, code } => {
                write!(f, "{site}: {code} is not supported")
            }
            Self::UnknownToolOffset { site, h } => write!(
                f,
                "{site}: G43 H{h} refers to a tool length offset the library does not \
                 define. The offset lives in the machine's tool table, not in the NC \
                 file; assuming zero would misplace the tool tip by its whole length"
            ),
            Self::UnknownTool { site, tool } => {
                write!(f, "{site}: T{tool} is not in the tool library")
            }
            Self::ArcRadiusMismatch {
                site,
                start_radius,
                end_radius,
                tolerance,
            } => write!(
                f,
                "{site}: the arc centre is {start_radius} from its start and \
                 {end_radius} from its end, a mismatch of {} against a tolerance of \
                 {tolerance}",
                (start_radius - end_radius).abs()
            ),
            Self::ArcIllConditioned {
                site,
                half_chord,
                radius,
            } => write!(
                f,
                "{site}: this R-form arc sweeps almost exactly 180 degrees (half-chord \
                 {half_chord} against radius {radius}), where the centre is not \
                 meaningfully determined by the endpoints. Write it with I/J/K"
            ),
            Self::FullCircleWithRadiusWord { site } => write!(
                f,
                "{site}: an arc whose endpoints coincide is a full circle with I/J/K, \
                 but with an R word it names no particular circle at all"
            ),
            Self::ArcRadiusTooSmall {
                site,
                half_chord,
                radius,
            } => write!(
                f,
                "{site}: radius {radius} cannot reach between endpoints {} apart",
                half_chord * 2.0
            ),
            Self::SubprogramTooDeep { site, limit } => {
                write!(f, "{site}: subprograms nested deeper than {limit}")
            }
            Self::UnknownSubprogram { site, number } => {
                write!(f, "{site}: no subprogram O{number} in this file")
            }
            Self::BadCycle { site, detail } => write!(f, "{site}: {detail}"),
            Self::NoFeedRate { site } => {
                write!(f, "{site}: a feed move with no feed rate ever commanded")
            }
            Self::Ir { detail } => write!(f, "the resolved toolpath is invalid: {detail}"),
            Self::Strict { warning } => write!(f, "{warning} (promoted by --strict)"),
        }
    }
}

impl core::error::Error for GcodeError {}

impl GcodeError {
    /// Short stable identifier, for reports and for corpus expectations.
    #[must_use]
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::ForeignLanguage { .. } => "foreign-language",
            Self::MacroProgramming { .. } => "macro-programming",
            Self::OWord { .. } => "o-word",
            Self::CutterCompensation { .. } => "cutter-compensation",
            Self::MissingDecimalPoint { .. } => "missing-decimal-point",
            Self::ModalGroupConflict { .. } => "modal-group-conflict",
            Self::NotANumber { .. } => "not-a-number",
            Self::UnknownWord { .. } => "unknown-word",
            Self::UnsupportedCode { .. } => "unsupported-code",
            Self::UnknownToolOffset { .. } => "unknown-tool-offset",
            Self::UnknownTool { .. } => "unknown-tool",
            Self::ArcRadiusMismatch { .. } => "arc-radius-mismatch",
            Self::ArcIllConditioned { .. } => "arc-ill-conditioned",
            Self::FullCircleWithRadiusWord { .. } => "full-circle-with-radius-word",
            Self::ArcRadiusTooSmall { .. } => "arc-radius-too-small",
            Self::SubprogramTooDeep { .. } => "subprogram-too-deep",
            Self::UnknownSubprogram { .. } => "unknown-subprogram",
            Self::BadCycle { .. } => "bad-cycle",
            Self::NoFeedRate { .. } => "no-feed-rate",
            Self::Ir { .. } => "invalid-ir",
            Self::Strict { .. } => "strict",
        }
    }

    /// Where it happened, if it happened somewhere.
    #[must_use]
    pub const fn site(&self) -> Option<Site> {
        match self {
            Self::ForeignLanguage { site, .. }
            | Self::MacroProgramming { site, .. }
            | Self::OWord { site, .. }
            | Self::CutterCompensation { site, .. }
            | Self::MissingDecimalPoint { site, .. }
            | Self::ModalGroupConflict { site, .. }
            | Self::NotANumber { site, .. }
            | Self::UnknownWord { site, .. }
            | Self::UnsupportedCode { site, .. }
            | Self::UnknownToolOffset { site, .. }
            | Self::UnknownTool { site, .. }
            | Self::ArcRadiusMismatch { site, .. }
            | Self::ArcIllConditioned { site, .. }
            | Self::FullCircleWithRadiusWord { site }
            | Self::ArcRadiusTooSmall { site, .. }
            | Self::BadCycle { site, .. }
            | Self::SubprogramTooDeep { site, .. }
            | Self::UnknownSubprogram { site, .. }
            | Self::NoFeedRate { site } => Some(*site),
            Self::Ir { .. } | Self::Strict { .. } => None,
        }
    }
}

/// Something survivable that a careful reader should still know about.
#[derive(Debug, Clone, PartialEq)]
pub enum GcodeWarning {
    /// An arc's radius mismatch was inside tolerance and the centre was moved.
    ArcRecentred {
        /// Where.
        site: Site,
        /// The mismatch before recentring.
        residual: f64,
    },
    /// A comment that opened and never closed.
    UnbalancedComment {
        /// Where it opened.
        site: Site,
    },
    /// A nested `(` inside a comment. Illegal, and common.
    NestedComment {
        /// Where.
        site: Site,
    },
    /// An M-code recognised as valid but for which we model no behaviour.
    UnmodelledMCode {
        /// Where.
        site: Site,
        /// The code.
        code: u32,
    },
    /// A block was skipped, or executed, because of a leading `/`.
    BlockSkip {
        /// Where.
        site: Site,
        /// Whether it was executed.
        executed: bool,
    },
    /// A zero-length move was dropped.
    ZeroLengthMove {
        /// Where.
        site: Site,
    },
    /// `G64 P…` set a path tolerance; we simulate the commanded path.
    PathToleranceIgnored {
        /// Where.
        site: Site,
        /// The tolerance the program asked for.
        tolerance: f64,
    },
    /// A `G73` was expanded without its chip-break retract.
    UnmodelledRetract {
        /// Where.
        site: Site,
    },
    /// Units changed mid-program.
    UnitsChanged {
        /// Where.
        site: Site,
        /// The units now in force.
        to: &'static str,
    },
}

impl GcodeWarning {
    /// Short stable identifier.
    #[must_use]
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::ArcRecentred { .. } => "arc-recentred",
            Self::UnbalancedComment { .. } => "unbalanced-comment",
            Self::NestedComment { .. } => "nested-comment",
            Self::UnmodelledMCode { .. } => "unmodelled-m-code",
            Self::BlockSkip { .. } => "block-skip",
            Self::ZeroLengthMove { .. } => "zero-length-move",
            Self::PathToleranceIgnored { .. } => "path-tolerance-ignored",
            Self::UnmodelledRetract { .. } => "unmodelled-retract",
            Self::UnitsChanged { .. } => "units-changed",
        }
    }

    /// Where it happened.
    #[must_use]
    pub const fn site(&self) -> Site {
        match self {
            Self::ArcRecentred { site, .. }
            | Self::UnbalancedComment { site }
            | Self::NestedComment { site }
            | Self::UnmodelledMCode { site, .. }
            | Self::BlockSkip { site, .. }
            | Self::ZeroLengthMove { site }
            | Self::PathToleranceIgnored { site, .. }
            | Self::UnmodelledRetract { site }
            | Self::UnitsChanged { site, .. } => *site,
        }
    }
}

impl fmt::Display for GcodeWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ArcRecentred { site, residual } => write!(
                f,
                "{site}: arc centre moved to split a radius mismatch of {residual}"
            ),
            Self::UnbalancedComment { site } => {
                write!(f, "{site}: a comment opened here and never closed")
            }
            Self::NestedComment { site } => {
                write!(f, "{site}: '(' inside a comment, which is not legal")
            }
            Self::UnmodelledMCode { site, code } => {
                write!(f, "{site}: M{code} is recorded but not modelled")
            }
            Self::BlockSkip { site, executed } => write!(
                f,
                "{site}: block-skip '/' block was {}",
                if *executed { "executed" } else { "skipped" }
            ),
            Self::ZeroLengthMove { site } => {
                write!(f, "{site}: a move of zero length was dropped")
            }
            Self::PathToleranceIgnored { site, tolerance } => write!(
                f,
                "{site}: G64 P{tolerance} lets the control round corners by up to \
                 {tolerance}; the commanded path is simulated instead"
            ),
            Self::UnmodelledRetract { site } => write!(
                f,
                "{site}: G73 retracts between pecks by a machine parameter that is not \
                 in this file, so that motion is missing from the toolpath. Harmless for \
                 material removal, but a collision check cannot see it. Pass \
                 --chip-break-clearance to supply it"
            ),
            Self::UnitsChanged { site, to } => {
                write!(f, "{site}: units changed to {to} mid-program")
            }
        }
    }
}

/// Warnings gathered during a parse, in the order they arose.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Diagnostics {
    warnings: Vec<GcodeWarning>,
}

impl Diagnostics {
    /// Empty.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Records a warning.
    pub fn warn(&mut self, warning: GcodeWarning) {
        self.warnings.push(warning);
    }

    /// The warnings, in order.
    #[must_use]
    pub fn warnings(&self) -> &[GcodeWarning] {
        &self.warnings
    }

    /// How many.
    #[must_use]
    pub fn len(&self) -> usize {
        self.warnings.len()
    }

    /// True if nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.warnings.is_empty()
    }

    /// How many warnings of a given kind.
    #[must_use]
    pub fn count_of(&self, kind: &str) -> usize {
        self.warnings.iter().filter(|w| w.kind() == kind).count()
    }

    /// The first warning, promoted to an error, if `strict` and any exist.
    ///
    /// Returning the *first* rather than a set: a strict run stops at the first
    /// thing it disagrees with, which is what strict means.
    #[must_use]
    pub fn first_as_error(&self, strict: bool) -> Option<GcodeError> {
        if !strict {
            return None;
        }
        self.warnings.first().map(|w| GcodeError::Strict {
            warning: Box::new(w.clone()),
        })
    }
}
