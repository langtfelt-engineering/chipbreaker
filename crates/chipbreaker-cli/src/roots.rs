// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! The `chipbreaker roots` subcommands.
//!
//! # Why the root solver has a command of its own
//!
//! It is not a user-facing feature. It is here because it is the component most
//! likely to be blamed when a ray gives a surprising answer, and a bug report
//! that says "the tool leaked at this position" is far cheaper to act on if the
//! reporter can also paste the polynomial and what came back. The residual and
//! the multiplicity printed here are usually enough to tell a solver bug from a
//! conditioning limit — and at a double root, `f64` gives eight digits rather
//! than sixteen, so a root that looks wrong in the ninth is not a bug.

use chipbreaker_core::eps::SQRT_F64_EPSILON;
use chipbreaker_core::roots::{self, RootSet};
use clap::Subcommand;
use serde_json::{Value, json};

/// `chipbreaker roots ...`
#[derive(Debug, Subcommand)]
pub enum RootsCommand {
    /// Solve a polynomial of degree one to four for its real roots.
    ///
    /// `allow_negative_numbers` is set because half of all polynomials have a
    /// negative coefficient somewhere, and without it clap reads `-4` as an
    /// unknown flag. A root solver that cannot be handed `x^2 - 4` is not one.
    #[command(allow_negative_numbers = true)]
    Solve {
        /// Coefficients in **descending** degree: `a b c` means `a x^2 + b x + c`.
        #[arg(value_name = "COEFFICIENT", required = true, num_args = 2..=5)]
        coefficients: Vec<f64>,
        /// Emit JSON instead of text.
        #[arg(long)]
        json: bool,
    },
}

impl RootsCommand {
    /// Whether this invocation asked for JSON.
    #[must_use]
    pub const fn json(&self) -> bool {
        match self {
            Self::Solve { json, .. } => *json,
        }
    }
}

fn solve(coefficients: &[f64]) -> Result<RootSet, String> {
    match coefficients {
        [a, b] => Ok(roots::solve_linear(*a, *b)),
        [a, b, c] => Ok(roots::solve_quadratic(*a, *b, *c)),
        [a, b, c, d] => Ok(roots::solve_cubic(*a, *b, *c, *d)),
        [a, b, c, d, e] => Ok(roots::solve_quartic(*a, *b, *c, *d, *e)),
        _ => Err(format!(
            "give 2 to 5 coefficients, in descending degree; got {}",
            coefficients.len()
        )),
    }
}

/// Renders a polynomial the way it would be written by hand.
fn render_polynomial(coefficients: &[f64]) -> String {
    let degree = coefficients.len() - 1;
    let mut out = String::new();
    for (i, c) in coefficients.iter().enumerate() {
        if *c == 0.0 {
            continue;
        }
        let power = degree - i;
        if !out.is_empty() {
            out.push_str(if *c < 0.0 { " - " } else { " + " });
        } else if *c < 0.0 {
            out.push('-');
        }
        let magnitude = c.abs();
        if magnitude != 1.0 || power == 0 {
            out.push_str(&format!("{magnitude}"));
            if power > 0 {
                out.push('*');
            }
        }
        match power {
            0 => {}
            1 => out.push('x'),
            _ => out.push_str(&format!("x^{power}")),
        }
    }
    if out.is_empty() {
        out.push('0');
    }
    out
}

/// Runs a `roots` subcommand.
///
/// # Errors
/// Returns a message suitable for stderr.
pub fn run(command: &RootsCommand) -> Result<(Value, String, bool), String> {
    let RootsCommand::Solve { coefficients, .. } = command;
    for c in coefficients {
        if !c.is_finite() {
            return Err(format!("`{c}` is not a coefficient"));
        }
    }
    let found = solve(coefficients)?;

    let entries: Vec<Value> = found
        .iter()
        .map(|(value, multiplicity)| {
            let residual = roots::eval(coefficients, value);
            json!({
                "multiplicity": multiplicity,
                "residual": residual,
                "root": value,
                // At a root of multiplicity m the attainable relative accuracy
                // is eps^(1/m); reporting it stops a correct answer at a double
                // root from looking like a wrong one.
                "relative_accuracy_floor": if multiplicity > 1 {
                    SQRT_F64_EPSILON
                } else {
                    f64::EPSILON
                },
            })
        })
        .collect();

    let requested = coefficients.len() - 1;
    let solved = usize::from(found.solved_degree());
    let mut text = format!(
        "{}\n\ndegree {requested}{}\n{} distinct real root(s), {} counted with multiplicity\n",
        render_polynomial(coefficients),
        if solved == requested {
            String::new()
        } else {
            format!(
                " (solved as {solved}: the leading coefficient is negligible \
                 against the rest, so the root it would contribute is far outside \
                 any region of interest)"
            )
        },
        found.len(),
        found.total_multiplicity(),
    );
    for (value, multiplicity) in found.iter() {
        let residual = roots::eval(coefficients, value);
        text.push_str(&format!(
            "  x = {value:<24}  multiplicity {multiplicity}   residual {residual:e}{}\n",
            if multiplicity > 1 {
                "   (a repeated root is resolvable to about eight digits, not sixteen)"
            } else {
                ""
            }
        ));
    }

    let results = json!({
        "coefficients": coefficients,
        "command": "solve",
        "degree_requested": requested,
        "degree_solved": solved,
        "distinct_roots": found.len(),
        "roots": entries,
        "total_multiplicity": found.total_multiplicity(),
    });
    Ok((results, text, true))
}
