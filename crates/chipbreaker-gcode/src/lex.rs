// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Stage one: text into words.
//!
//! A word is a letter and a number — `X10.5`, `G01`, `M6`. That is nearly the
//! whole of RS-274's lexical structure, and keeping this stage to exactly that
//! is what makes the next three testable in isolation.
//!
//! # What this stage decides, and what it refuses to
//!
//! It decides where words are, what their letters and numbers are, what is a
//! comment, and whether a line was marked for block skip. It does **not** know
//! what `G1` means, that `X` is an axis, or that two motion codes in a block
//! conflict. Those are the block assembler's and the modal machine's problems.
//!
//! It does refuse three things, because they are lexical and because refusing
//! early gives a much better message than letting them fall through:
//!
//! * **Foreign languages.** Siemens and Heidenhain are not dialects, and their
//!   giveaways are visible at the character level.
//! * **Macro programming.** `#100`, `IF`, `WHILE`, `GOTO`.
//! * **`o`-words.** LinuxCNC's procedural extension.
//!
//! # The decimal point
//!
//! On legacy Fanuc controls an axis word without a decimal point is read in the
//! machine's least input increment: `X10` is 0.010 mm, not 10 mm. A factor of a
//! thousand, and it parses perfectly either way. The lexer records whether each
//! word had a decimal point and lets the caller decide; the default policy
//! rejects, because there is no reading of the file that is safe to guess.
//!
//! `X0` is exempt. Zero is zero in any increment, so nothing is ambiguous, and
//! rejecting it would reject a construct that appears in almost every program.

use chipbreaker_core::toolpath::Provenance;

use crate::diag::{ForeignDialect, GcodeError, GcodeWarning, Site};

/// One `letter number` pair.
#[derive(Debug, Clone, PartialEq)]
pub struct Word {
    /// The letter, upper-cased.
    pub letter: char,
    /// The number.
    pub value: f64,
    /// Whether the number was written with a decimal point.
    ///
    /// Kept because the *meaning* of an axis word depends on it; see the module
    /// header.
    pub had_decimal: bool,
    /// The word exactly as written, for error messages.
    pub raw: String,
    /// Where it starts.
    pub site: Site,
}

impl Word {
    /// The value as a `G`/`M` code key: the number times ten, rounded.
    ///
    /// `G59.1` becomes 591 and `G1` becomes 10. Codes are compared as integers
    /// because comparing them as floats invites `G59.1 == 59.1` to be false on
    /// some path, and because one tenth is the finest subdivision RS-274 uses.
    #[must_use]
    pub fn code_key(&self) -> u32 {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "codes are small non-negative numbers; the range is checked"
        )]
        {
            if !(0.0..=1000.0).contains(&self.value) {
                return u32::MAX;
            }
            (self.value * 10.0).round() as u32
        }
    }

    /// The integer part, for words like `T`, `H`, `L` and `P` that are counts.
    #[must_use]
    pub fn as_u32(&self) -> Option<u32> {
        if self.value < 0.0 || self.value > f64::from(u32::MAX) {
            return None;
        }
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "range checked immediately above"
        )]
        Some(self.value.round() as u32)
    }

    /// Rendered the way it would be written, for messages.
    #[must_use]
    pub fn code_text(&self) -> String {
        self.raw.clone()
    }
}

/// One source line, lexed.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RawBlock {
    /// The words on it, in order.
    pub words: Vec<Word>,
    /// Comments found on the line, in order, with the parentheses stripped.
    pub comments: Vec<String>,
    /// True if the line began with `/`.
    pub block_skip: bool,
    /// One-based line number.
    pub line: u32,
    /// Which file.
    pub file: u32,
}

impl RawBlock {
    /// True if the line carried no words at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.words.is_empty()
    }

    /// Provenance for anything this block produces.
    #[must_use]
    pub const fn provenance(&self, block_index: u32) -> Provenance {
        Provenance::new(self.file, self.line, block_index)
    }

    /// The first word with the given letter.
    #[must_use]
    pub fn word(&self, letter: char) -> Option<&Word> {
        self.words.iter().find(|w| w.letter == letter)
    }

    /// Every word with the given letter.
    pub fn words_with(&self, letter: char) -> impl Iterator<Item = &Word> {
        self.words.iter().filter(move |w| w.letter == letter)
    }
}

/// Letters this dialect gives meaning to.
///
/// `O` is here so that `O1000` program numbers lex, and so that LinuxCNC's
/// lower-case `o` words can be told apart from them and refused by name.
const KNOWN_LETTERS: &str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/// Substrings that give away a program written in another language.
const FOREIGN_MARKERS: &[(&str, ForeignDialect)] = &[
    ("CYCLE8", ForeignDialect::Siemens840d),
    ("CYCLE7", ForeignDialect::Siemens840d),
    ("MSG(", ForeignDialect::Siemens840d),
    ("DEF REAL", ForeignDialect::Siemens840d),
    ("DEF INT", ForeignDialect::Siemens840d),
    ("G17 G90 G54 SOFT", ForeignDialect::Siemens840d),
    ("BEGIN PGM", ForeignDialect::HeidenhainKlartext),
    ("END PGM", ForeignDialect::HeidenhainKlartext),
    ("TOOL CALL", ForeignDialect::HeidenhainKlartext),
    ("CYCL DEF", ForeignDialect::HeidenhainKlartext),
];

/// Keywords that mean macro or parametric programming.
const MACRO_KEYWORDS: &[&str] = &["IF", "WHILE", "GOTO", "THEN", "ENDIF", "ENDW", "DO"];

/// Lexes one file into blocks.
///
/// # Errors
///
/// Returns the first [`GcodeError`], which for this stage means a foreign
/// language, a macro construct, an `o`-word, an unknown letter, or a number that
/// is not finite.
pub fn lex(
    text: &str,
    file: u32,
    diagnostics: &mut crate::diag::Diagnostics,
) -> Result<Vec<RawBlock>, GcodeError> {
    let mut blocks = Vec::new();
    // A comment opened with `(` may, in badly-written files, run past the end of
    // its line. Tracking it across lines is what lets that be a warning rather
    // than a cascade of syntax errors.
    let mut open_comment: Option<Site> = None;

    for (index, raw_line) in text.lines().enumerate() {
        let line = u32::try_from(index + 1).unwrap_or(u32::MAX);
        let mut block = RawBlock {
            line,
            file,
            ..RawBlock::default()
        };

        detect_foreign(raw_line, Site::line_only(file, line))?;

        let chars: Vec<char> = raw_line.chars().collect();
        let mut i = 0usize;
        let mut seen_non_space = false;

        while i < chars.len() {
            let column = u32::try_from(i + 1).unwrap_or(u32::MAX);
            let site = Site::new(file, line, column);
            let c = chars[i];

            // Inside a comment that began on an earlier line.
            if let Some(opened) = open_comment {
                if c == ')' {
                    open_comment = None;
                } else if c == '(' {
                    diagnostics.warn(GcodeWarning::NestedComment { site });
                }
                let _ = opened;
                i += 1;
                continue;
            }

            match c {
                ' ' | '\t' | '\r' => {
                    i += 1;
                }
                '/' if !seen_non_space => {
                    block.block_skip = true;
                    seen_non_space = true;
                    i += 1;
                }
                '%' => {
                    // Tape start/end marker. A whole-line token, not a word.
                    i = chars.len();
                }
                ';' => {
                    block
                        .comments
                        .push(chars[i + 1..].iter().collect::<String>().trim().to_owned());
                    i = chars.len();
                }
                '(' => {
                    let mut text = String::new();
                    let mut j = i + 1;
                    let mut closed = false;
                    while j < chars.len() {
                        if chars[j] == ')' {
                            closed = true;
                            j += 1;
                            break;
                        }
                        if chars[j] == '(' {
                            diagnostics.warn(GcodeWarning::NestedComment {
                                site: Site::new(file, line, u32::try_from(j + 1).unwrap_or(0)),
                            });
                        }
                        text.push(chars[j]);
                        j += 1;
                    }
                    if !closed {
                        // Illegal, and common. Treat the rest of the file's line
                        // as comment and carry the state forward.
                        diagnostics.warn(GcodeWarning::UnbalancedComment { site });
                        open_comment = Some(site);
                    }
                    block.comments.push(text.trim().to_owned());
                    i = j;
                    seen_non_space = true;
                }
                ')' => {
                    // A close with nothing open. Comments do not nest, so
                    // `(outer (inner) )` closes at the first `)` and leaves this
                    // one stray. Illegal, and it appears in the wild for exactly
                    // that reason, so it is a warning and a skip rather than a
                    // refusal -- the file runs on the machine.
                    diagnostics.warn(GcodeWarning::UnbalancedComment { site });
                    i += 1;
                }
                '#' => {
                    return Err(GcodeError::MacroProgramming {
                        site,
                        construct: "# variable".to_owned(),
                    });
                }
                'o' => {
                    // Lower-case `o` at the start of a word is LinuxCNC. An
                    // upper-case `O` is a Fanuc program number and is fine.
                    let word: String = chars[i..]
                        .iter()
                        .take_while(|c| !c.is_whitespace())
                        .collect();
                    return Err(GcodeError::OWord { site, word });
                }
                c if c.is_ascii_alphabetic() => {
                    let upper = c.to_ascii_uppercase();

                    // A keyword rather than a word letter?
                    let rest: String = chars[i..]
                        .iter()
                        .take_while(|c| c.is_ascii_alphabetic())
                        .collect::<String>()
                        .to_ascii_uppercase();
                    if MACRO_KEYWORDS.contains(&rest.as_str()) {
                        return Err(GcodeError::MacroProgramming {
                            site,
                            construct: rest,
                        });
                    }
                    if !KNOWN_LETTERS.contains(upper) {
                        return Err(GcodeError::UnknownWord {
                            site,
                            letter: upper,
                        });
                    }

                    let (word, next) = read_number(&chars, i, upper, site)?;
                    block.words.push(word);
                    i = next;
                    seen_non_space = true;
                }
                _ => {
                    return Err(GcodeError::UnknownWord { site, letter: c });
                }
            }
        }

        blocks.push(block);
    }

    Ok(blocks)
}

/// Reads the number following a letter at `start`.
fn read_number(
    chars: &[char],
    start: usize,
    letter: char,
    site: Site,
) -> Result<(Word, usize), GcodeError> {
    let mut j = start + 1;
    // Space between the letter and its number is legal and appears in the wild.
    while j < chars.len() && (chars[j] == ' ' || chars[j] == '\t') {
        j += 1;
    }
    let number_start = j;
    if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
        j += 1;
    }
    let mut had_decimal = false;
    while j < chars.len() {
        match chars[j] {
            '0'..='9' => j += 1,
            '.' if !had_decimal => {
                had_decimal = true;
                j += 1;
            }
            _ => break,
        }
    }

    let text: String = chars[number_start..j].iter().collect();
    let raw = format!("{letter}{text}");
    if text.is_empty() || text == "+" || text == "-" || text == "." {
        return Err(GcodeError::NotANumber { site, text: raw });
    }
    // `str::parse` rather than anything of our own: it is correctly rounded,
    // which a hand-rolled decimal reader would not be.
    let value: f64 = text.parse().map_err(|_| GcodeError::NotANumber {
        site,
        text: raw.clone(),
    })?;
    if !value.is_finite() {
        return Err(GcodeError::NotANumber { site, text: raw });
    }

    Ok((
        Word {
            letter,
            value,
            had_decimal,
            raw,
            site,
        },
        j,
    ))
}

/// Refuses a program written in a different language, by name.
fn detect_foreign(line: &str, site: Site) -> Result<(), GcodeError> {
    let upper = line.to_ascii_uppercase();
    for (marker, dialect) in FOREIGN_MARKERS {
        if upper.contains(marker) {
            return Err(GcodeError::ForeignLanguage {
                site,
                dialect: *dialect,
                evidence: (*marker).to_owned(),
            });
        }
    }
    // Siemens R-parameters: `R1=` or `R10 =`. A bare `R10` is an arc radius in
    // RS-274, so the `=` is what distinguishes them.
    let bytes = upper.as_bytes();
    for (i, w) in bytes.windows(2).enumerate() {
        if w[0] == b'R' && w[1].is_ascii_digit() {
            let rest = &upper[i + 1..];
            let digits = rest.len() - rest.trim_start_matches(|c: char| c.is_ascii_digit()).len();
            if upper[i + 1 + digits..].trim_start().starts_with('=') {
                return Err(GcodeError::ForeignLanguage {
                    site,
                    dialect: ForeignDialect::Siemens840d,
                    evidence: upper[i..i + 1 + digits].to_owned(),
                });
            }
        }
    }
    Ok(())
}
