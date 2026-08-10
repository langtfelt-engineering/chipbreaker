// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! A declined job, as a document a caller can parse.
//!
//! # Why a refusal is not a verification report
//!
//! It was tempting to make it one — a single document type is easier to consume,
//! and every gate being `unchecked` already says "nothing was verified". The
//! reason not to is the same principle the rest of the report is built on.
//!
//! A verification report **asserts things about inputs**: which ones, by
//! content, at what spacing, against what tolerance, with what facet error. A
//! refusal is precisely the case where those were never established. A refusal
//! wearing a report's shape would have to carry a manifest digest computed over
//! inputs that were never read, and a `spacing_mm` for a field that was never
//! built. That is the mistake [`NumericalSemantics::sweep`] exists to avoid,
//! made larger: numbers in an audited artifact that no measurement produced.
//!
//! So there are two documents, and they are told apart by `schema`.
//!
//! # What they share, so a caller need not branch to find the answer
//!
//! Both carry `schema`, `schema_version`, `verdict` and `verdict_rule` at the
//! top level, with the same meanings. A consumer that only wants the gate reads
//! `verdict.pass` from either without knowing which it has, and gets `false`
//! from a refusal — because [`Gate::Unchecked`] does not pass, which is a rule
//! that was already load-bearing before this type existed.
//!
//! A consumer that wants to *explain* the outcome branches on `schema` and
//! reads `message` here. That is one branch, in the place where the two
//! outcomes genuinely differ.
//!
//! # The message is the product
//!
//! `G41`, a foreign dialect, macro programming, an oblique re-fixture, a
//! resolution that will not fit in the memory available — every one of those is
//! a sentence written for a person to read, usually naming what to do instead.
//! Flattening it to an error code loses the only part of a refusal anybody
//! wanted. It travels intact through this type, through the C ABI, and into a
//! Python exception.
//!
//! [`NumericalSemantics::sweep`]: super::report::NumericalSemantics::sweep
//! [`Gate::Unchecked`]: super::verdict::Gate::Unchecked

use serde_json::{Value, json};

use super::verdict::{self, GATE_COLLISION, GATE_GOUGE, GateOutcome, Verdict};

/// The refusal document's schema name.
pub const SCHEMA: &str = "chipbreaker.refusal";

/// The refusal document's version.
///
/// Versioned separately from the verification report, and deliberately: the two
/// documents change for different reasons, and tying them together would force a
/// break in one to look like a break in the other.
pub const SCHEMA_VERSION: u32 = 1;

/// A job the engine declined, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal {
    /// What to tell the person who ran it. A sentence, not a code.
    pub message: String,
}

impl Refusal {
    /// A refusal carrying `message`.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// The verdict a refusal carries: every gate unchecked, for this reason.
    ///
    /// Not an empty verdict. An empty one would also fail — [`Verdict::pass`]
    /// requires at least one gate — but it would fail silently, and a reader
    /// asking "why did the gouge gate not pass?" deserves the answer rather
    /// than an absence.
    #[must_use]
    pub fn verdict(&self) -> Verdict {
        Verdict::new()
            .with(GATE_GOUGE, GateOutcome::unchecked(self.message.clone()))
            .with(GATE_COLLISION, GateOutcome::unchecked(self.message.clone()))
    }

    /// The document.
    #[must_use]
    pub fn to_json(&self) -> Value {
        let v = self.verdict();
        let mut gates = serde_json::Map::new();
        for (name, outcome) in v.gates() {
            gates.insert(
                name.clone(),
                match &outcome.why {
                    Some(w) => json!({"state": outcome.state.as_str(), "why": w}),
                    None => json!({"state": outcome.state.as_str()}),
                },
            );
        }
        json!({
            "schema": SCHEMA,
            "schema_version": SCHEMA_VERSION,
            "refused": true,
            "message": self.message,
            "verdict": {
                "pass": v.pass(),
                "gates": Value::Object(gates),
            },
            "verdict_rule": verdict::VERDICT_RULE,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refusal_does_not_pass() {
        let r = Refusal::new("G41 arms cutter radius compensation");
        assert!(!r.verdict().pass());
        assert_eq!(r.to_json()["verdict"]["pass"], json!(false));
    }

    #[test]
    fn the_sentence_survives_into_the_document() {
        // The whole point of the type. A refusal that reaches a caller as
        // "failed" has thrown away the part that was worth having.
        let sentence = "this program has 40000 segments and the browser build is capped at 20000";
        let doc = Refusal::new(sentence).to_json();
        assert_eq!(doc["message"], json!(sentence));
        assert_eq!(doc["verdict"]["gates"]["gouge"]["why"], json!(sentence));
    }

    #[test]
    fn every_gate_is_unchecked_rather_than_failed() {
        // `fail` would say the engine checked and found a defect. It did not
        // check, and the difference is the whole of the gate design.
        let doc = Refusal::new("no").to_json();
        for gate in [GATE_GOUGE, GATE_COLLISION] {
            assert_eq!(doc["verdict"]["gates"][gate]["state"], json!("unchecked"));
        }
    }

    #[test]
    fn a_caller_can_read_the_verdict_without_knowing_which_document_it_has() {
        // Both documents carry `schema`, `schema_version`, `verdict` and
        // `verdict_rule` at the top level. This is the contract that lets a
        // consumer read the gate before it branches.
        let doc = Refusal::new("no").to_json();
        for key in ["schema", "schema_version", "verdict", "verdict_rule"] {
            assert!(doc.get(key).is_some(), "{key} must be present");
        }
        assert_eq!(doc["schema"], json!(SCHEMA));
    }
}
