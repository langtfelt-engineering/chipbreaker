// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

#![forbid(unsafe_code)]

//! RS-274 G-code, into [`chipbreaker_core::toolpath::Toolpath`].
//!
//! **This crate is the only place in Chipbreaker that reads G-code text.**
//! Everything downstream consumes the IR. That is why it is a separate crate
//! rather than a module of the core: U5 cannot accidentally reach for a lexer
//! that is not in its dependency graph.
//!
//! # Four stages, kept apart on purpose
//!
//! 1. [`lex`] — text into words. Knows nothing about what a word means.
//! 2. [`block`] — words into blocks, with modal groups checked.
//! 3. [`modal`] — the state machine every block is interpreted against.
//! 4. [`resolve`] — programmed points into machine coordinates, and motion.
//!
//! A monolithic parser would be miserable to debug against a real file, where
//! the question is nearly always "which stage got this wrong" and the answer is
//! only available if the stages can be run separately. Each has its own tests
//! and its own errors.
//!
//! # What is refused, and why refusing beats approximating
//!
//! Siemens and Heidenhain programs, macro and parametric programming, `o`-word
//! subprograms, and `G41`/`G42` cutter radius compensation. Each is detected and
//! named rather than approximated.
//!
//! The precedent is Unit 2's 3MF component transforms: producing
//! plausible-but-wrong geometry is worse than producing none, because a
//! verification tool that is quietly wrong is worse than one that admits it
//! cannot answer. `G41` is the sharpest case — simulating the uncompensated path
//! yields a part wrong by the tool radius *everywhere*, and it looks entirely
//! reasonable.

pub mod block;
pub mod diag;
pub mod lex;

pub use diag::{Diagnostics, ForeignDialect, GcodeError, GcodeWarning, Site};

/// Version of this crate.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
