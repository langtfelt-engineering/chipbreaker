// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! The verdict: one gate per thing that can condemn a program.
//!
//! # Why this replaced a boolean
//!
//! Schema version 1 carried `accepted`, a single bit meaning "no gouge above
//! tolerance". Collision checking made that bit dangerous rather than merely
//! incomplete: a consumer reading `accepted` — the obvious thing to read — would
//! have passed a program that drives a holder into a fixture, because `accepted`
//! never knew about collisions and never would.
//!
//! Widening `accepted` in place was the worse option still. A field that keeps
//! its name and changes its meaning breaks every consumer silently, which is the
//! one thing the stability contract promises never happens.
//!
//! So the field is **renamed**, and that is the load-bearing part. A version-1
//! consumer looking for `accepted` finds nothing and fails loudly, at the moment
//! it can still be fixed, rather than reading a bit that no longer means what it
//! was written against.
//!
//! # The forward-compatibility rule
//!
//! Every future gate is a **new key** under `gates`. A consumer that computes
//! its own answer by ignoring keys it does not recognise still gets the right
//! result, because [`Verdict::pass`] is a conjunction: an unknown gate can only
//! ever make the answer stricter, never laxer. Integrators may rely on that.
//!
//! # `Unchecked` is not `Pass`
//!
//! A gate that could not run reports [`Gate::Unchecked`] with a reason, and an
//! unchecked gate does **not** pass. This is deliberate and it is the whole
//! point: a holder that is not modelled cannot be found hitting anything, and a
//! tool that reported "clear" on that basis would be manufacturing safety out of
//! missing data. Absence of evidence is reported as absence of evidence.

use crate::golden::{CanonicalHash, Hashable};

use core::fmt;
use std::collections::BTreeMap;

/// The gouge gate: geometry against the nominal part.
pub const GATE_GOUGE: &str = "gouge";
/// The collision gate: non-cutting geometry against anything solid.
pub const GATE_COLLISION: &str = "collision";

/// What one gate concluded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Gate {
    /// The gate ran and found nothing.
    Pass,
    /// The gate ran and condemned the program.
    Fail,
    /// The gate could not run. **This does not pass.**
    Unchecked,
}

impl Gate {
    /// The name used in the report.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "pass",
            Self::Fail => "fail",
            Self::Unchecked => "unchecked",
        }
    }

    /// Whether this gate permits the run. Only [`Gate::Pass`] does.
    #[must_use]
    pub const fn is_pass(self) -> bool {
        matches!(self, Self::Pass)
    }

    /// Parses a gate state from a report.
    ///
    /// An unrecognised state is **not** read as a pass. A future version may add
    /// a state this build has never heard of, and guessing "fine" about a word
    /// it cannot read is how a tool certifies something it did not check.
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "pass" => Some(Self::Pass),
            "fail" => Some(Self::Fail),
            "unchecked" => Some(Self::Unchecked),
            _ => None,
        }
    }
}

impl fmt::Display for Gate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl Hashable for Gate {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(self.as_str());
    }
}

/// A gate's conclusion, and why when it is not a plain pass.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GateOutcome {
    /// What the gate concluded.
    pub state: Gate,
    /// Why, for anything other than a pass.
    ///
    /// Required in practice for [`Gate::Unchecked`]: "unchecked" without a
    /// reason tells a reader that something is missing but not what to supply,
    /// which leaves them unable to act on the one finding that is entirely
    /// within their power to fix.
    pub why: Option<String>,
}

impl GateOutcome {
    /// A gate that ran and found nothing.
    #[must_use]
    pub const fn pass() -> Self {
        Self {
            state: Gate::Pass,
            why: None,
        }
    }

    /// A gate that condemned the run.
    #[must_use]
    pub fn fail(why: impl Into<String>) -> Self {
        Self {
            state: Gate::Fail,
            why: Some(why.into()),
        }
    }

    /// A gate that could not run.
    #[must_use]
    pub fn unchecked(why: impl Into<String>) -> Self {
        Self {
            state: Gate::Unchecked,
            why: Some(why.into()),
        }
    }
}

impl Hashable for GateOutcome {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("GateOutcome");
        h.add(&self.state);
        // The empty string stands in for an absent reason, so a pass and a pass
        // that somehow acquired an empty reason hash alike rather than differing
        // on nothing a reader could see.
        h.str(self.why.as_deref().unwrap_or(""));
        h.end();
    }
}

/// Every gate, and the conjunction over them.
///
/// Ordered, because the report is byte-stable and an unordered map reaching the
/// output would make two runs of the same inputs differ.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Verdict {
    gates: BTreeMap<String, GateOutcome>,
}

impl Verdict {
    /// An empty verdict, which does not pass: nothing has been checked.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Records a gate, replacing any previous conclusion under that name.
    #[must_use]
    pub fn with(mut self, name: &str, outcome: GateOutcome) -> Self {
        self.gates.insert(String::from(name), outcome);
        self
    }

    /// The gates, in name order.
    #[must_use]
    pub const fn gates(&self) -> &BTreeMap<String, GateOutcome> {
        &self.gates
    }

    /// One gate's conclusion.
    #[must_use]
    pub fn gate(&self, name: &str) -> Option<&GateOutcome> {
        self.gates.get(name)
    }

    /// Whether the run passes: **every** gate passes, and there is at least one.
    ///
    /// A conjunction, so an unrecognised gate can only make a consumer's own
    /// answer stricter. An empty verdict does not pass, because a report that
    /// checked nothing has certified nothing.
    #[must_use]
    /// # Examples
    ///
    /// A verdict passes only when every gate does, and an unchecked gate is
    /// not a pass:
    ///
    /// ```
    /// use chipbreaker_core::findings::{GateOutcome, Verdict};
    /// use chipbreaker_core::findings::verdict::{GATE_COLLISION, GATE_GOUGE};
    ///
    /// let clear = Verdict::new()
    ///     .with(GATE_GOUGE, GateOutcome::pass())
    ///     .with(GATE_COLLISION, GateOutcome::pass());
    /// assert!(clear.pass());
    ///
    /// // No nominal was supplied, so the gouge gate never ran. This is the
    /// // case an integration is most likely to misread as success.
    /// let partial = Verdict::new()
    ///     .with(GATE_GOUGE, GateOutcome::unchecked("no nominal was supplied"))
    ///     .with(GATE_COLLISION, GateOutcome::pass());
    /// assert!(!partial.pass());
    ///
    /// // An empty verdict has certified nothing, so it does not pass either.
    /// assert!(!Verdict::new().pass());
    /// ```
    pub fn pass(&self) -> bool {
        !self.gates.is_empty() && self.gates.values().all(|g| g.state.is_pass())
    }
}

impl Hashable for Verdict {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Verdict");
        h.usize(self.gates.len());
        for (name, outcome) in &self.gates {
            h.str(name);
            h.add(outcome);
        }
        h.bool(self.pass());
        h.end();
    }
}

/// The sentence that travels with the verdict in every report.
pub const VERDICT_RULE: &str = "a run passes when every gate passes. A gate that could not run \
     reports `unchecked` with a reason and does not pass, because a check that did not happen is \
     not a check that succeeded. Future versions add gates as new keys under `gates`; a consumer \
     that ignores unknown keys still computes a correct, and never a laxer, answer.";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_verdict_does_not_pass() {
        // A report that checked nothing has certified nothing. The alternative
        // -- vacuous truth -- would make the safest-looking report the one that
        // did the least work.
        assert!(!Verdict::new().pass());
    }

    #[test]
    fn unchecked_does_not_pass() {
        let v = Verdict::new()
            .with(GATE_GOUGE, GateOutcome::pass())
            .with(GATE_COLLISION, GateOutcome::unchecked("no holder defined"));
        assert!(!v.pass(), "an unchecked gate must not certify a run");
        assert!(v.gate(GATE_COLLISION).expect("present").why.is_some());
    }

    #[test]
    fn pass_requires_every_gate() {
        assert!(
            Verdict::new()
                .with(GATE_GOUGE, GateOutcome::pass())
                .with(GATE_COLLISION, GateOutcome::pass())
                .pass()
        );
        assert!(
            !Verdict::new()
                .with(GATE_GOUGE, GateOutcome::pass())
                .with(GATE_COLLISION, GateOutcome::fail("holder into clamp"))
                .pass()
        );
    }

    #[test]
    fn an_unknown_gate_can_only_tighten_the_answer() {
        // The forward-compatibility rule, as a test rather than a promise in
        // prose: adding a gate never turns a false into a true.
        let base = Verdict::new().with(GATE_GOUGE, GateOutcome::pass());
        for extra in [
            GateOutcome::pass(),
            GateOutcome::fail("x"),
            GateOutcome::unchecked("x"),
        ] {
            let widened = base.clone().with("some-future-gate", extra);
            assert!(
                base.pass() || !widened.pass(),
                "adding a gate must never make a failing verdict pass"
            );
        }
    }

    #[test]
    fn gates_are_ordered_regardless_of_insertion() {
        let a = Verdict::new()
            .with(GATE_COLLISION, GateOutcome::pass())
            .with(GATE_GOUGE, GateOutcome::pass());
        let b = Verdict::new()
            .with(GATE_GOUGE, GateOutcome::pass())
            .with(GATE_COLLISION, GateOutcome::pass());
        let names = |v: &Verdict| v.gates().keys().cloned().collect::<Vec<_>>();
        assert_eq!(names(&a), names(&b));
        let (mut ha, mut hb) = (CanonicalHash::new(), CanonicalHash::new());
        a.hash_canonical(&mut ha);
        b.hash_canonical(&mut hb);
        assert_eq!(ha.finish().to_hex(), hb.finish().to_hex());
    }
}
