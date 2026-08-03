// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Tool and holder geometry: solids of revolution about the `+Z` axis.
//!
//! # The coordinate convention
//!
//! Every tool in Chipbreaker is expressed the same way, and the convention is
//! part of the public contract rather than an implementation detail:
//!
//! * **The axis is `+Z`.** A tool is a solid of revolution about it.
//! * **The tip is at the origin.** Not the gauge line, not the shank end — the
//!   point that touches the work first.
//! * **The tool occupies `z >= 0`**, running upward from the tip toward the
//!   spindle.
//! * **The generating profile lives in `(r, z)` with `r >= 0`**, begins at
//!   `(0, 0)`, and is an open chain. The solid is closed by the axis and by a
//!   single disc cap at the top.
//!
//! Putting the origin at the tip is the choice that matters. Toolpaths are
//! programmed to the tip, gauge lengths are measured to the tip, and tool
//! changes swap one tip for another; anchoring anywhere else would put a
//! subtraction into every one of those, and a subtraction is a place to get a
//! sign wrong. [`Tool::gauge_length`] carries the spindle-relative measurement
//! for the cases that genuinely need it.
//!
//! # What is deliberately not modelled
//!
//! Flutes, helix angle, rake, relief, and the number of teeth are out of scope
//! for the project, not merely for this unit. Material removal is determined by
//! the tool's swept envelope, which is a surface of revolution: a two-flute and
//! a four-flute cutter of the same diameter and corner radius remove exactly the
//! same material. Modelling flutes would add no accuracy to the simulation and
//! would turn every ray intersection from a quartic into something with no
//! closed form at all.

pub mod catalog;
pub mod profile;

pub use catalog::{CatalogError, HolderStage, Shank};
pub use profile::{ArcDirection, ElementRole, Profile, ProfileElement, ProfileError, RoledElement};

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec3;

use core::fmt;

/// The tool axis. Every tool is a solid of revolution about this direction,
/// through the origin.
pub const TOOL_AXIS: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

/// A stable identifier for a tool within a library.
///
/// Deliberately a name rather than an index. Indices renumber when a library is
/// edited, and a renumbered index in a saved simulation silently points at a
/// different tool; a name that no longer resolves is a loud error instead.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ToolId(String);

impl ToolId {
    /// Builds an identifier.
    ///
    /// # Errors
    ///
    /// Rejects an empty name, leading or trailing whitespace, control
    /// characters, and any non-ASCII character. The restriction is not
    /// squeamishness about Unicode: tool identifiers appear in file names, in
    /// golden-hash inputs, and in report keys, and a name that normalises
    /// differently on two platforms would break bit-identical output.
    pub fn new(name: impl Into<String>) -> Result<Self, ToolError> {
        let name = name.into();
        if name.is_empty() {
            return Err(ToolError::EmptyId);
        }
        if name.trim() != name {
            return Err(ToolError::PaddedId { found: name });
        }
        if !name.is_ascii() || name.chars().any(char::is_control) {
            return Err(ToolError::UnprintableId { found: name });
        }
        Ok(Self(name))
    }

    /// The identifier as a string.
    #[inline]
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for ToolId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl Hashable for ToolId {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.str(&self.0);
    }
}

/// Why a tool was rejected.
#[derive(Debug, Clone, PartialEq)]
pub enum ToolError {
    /// A tool identifier cannot be empty.
    EmptyId,
    /// A tool identifier cannot begin or end with whitespace.
    PaddedId {
        /// The offending name.
        found: String,
    },
    /// A tool identifier must be printable ASCII.
    UnprintableId {
        /// The offending name.
        found: String,
    },
    /// The gauge length must be finite and at least the tool's own length: the
    /// gauge line is above the tip by definition, and never inside the tool.
    BadGaugeLength {
        /// The value supplied.
        found: f64,
        /// The distance from the tip to the top of the profile.
        minimum: f64,
    },
    /// The profile itself did not validate.
    Profile(ProfileError),
}

impl fmt::Display for ToolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyId => write!(f, "a tool identifier cannot be empty"),
            Self::PaddedId { found } => {
                write!(f, "tool identifier {found:?} has leading or trailing space")
            }
            Self::UnprintableId { found } => {
                write!(f, "tool identifier {found:?} is not printable ASCII")
            }
            Self::BadGaugeLength { found, minimum } => write!(
                f,
                "gauge length {found} is not a finite value of at least {minimum}, \
                 the distance from the tip to the top of the tool"
            ),
            Self::Profile(e) => write!(f, "{e}"),
        }
    }
}

impl core::error::Error for ToolError {}

impl From<ProfileError> for ToolError {
    fn from(e: ProfileError) -> Self {
        Self::Profile(e)
    }
}

/// A tool: an identifier, a generating profile, and how far the tip stands out
/// of the spindle.
#[derive(Debug, Clone, PartialEq)]
pub struct Tool {
    id: ToolId,
    description: String,
    profile: Profile,
    gauge_length: f64,
}

impl Tool {
    /// Builds a tool.
    ///
    /// # Errors
    ///
    /// Returns [`ToolError::BadGaugeLength`] if the gauge line would fall inside
    /// the tool or is not finite.
    pub fn new(
        id: ToolId,
        description: impl Into<String>,
        profile: Profile,
        gauge_length: f64,
    ) -> Result<Self, ToolError> {
        let minimum = profile.total_length();
        if !gauge_length.is_finite() || gauge_length < minimum {
            return Err(ToolError::BadGaugeLength {
                found: gauge_length,
                minimum,
            });
        }
        Ok(Self {
            id,
            description: description.into(),
            profile,
            gauge_length,
        })
    }

    /// The tool's identifier.
    #[inline]
    #[must_use]
    pub const fn id(&self) -> &ToolId {
        &self.id
    }

    /// Free-text description, for reports. Never parsed.
    #[inline]
    #[must_use]
    pub fn description(&self) -> &str {
        &self.description
    }

    /// The generating profile.
    #[inline]
    #[must_use]
    pub const fn profile(&self) -> &Profile {
        &self.profile
    }

    /// Distance from the tip to the gauge line, along `+Z`.
    ///
    /// The gauge line is the spindle's reference face. This is the number a
    /// machine's tool table holds, and the only reason the tool needs to know
    /// anything about the spindle: everything else in the simulation is measured
    /// from the tip.
    #[inline]
    #[must_use]
    pub const fn gauge_length(&self) -> f64 {
        self.gauge_length
    }

    /// Largest radius anywhere on the tool.
    #[must_use]
    pub fn max_radius(&self) -> f64 {
        self.profile.max_radius()
    }

    /// Diameter at the widest point.
    #[must_use]
    pub fn diameter(&self) -> f64 {
        2.0 * self.max_radius()
    }

    /// Distance from the tip to the top of the tool.
    #[must_use]
    pub fn total_length(&self) -> f64 {
        self.profile.total_length()
    }

    /// Distance from the tip to the top of the cutting geometry: the deepest cut
    /// the tool can take without rubbing.
    #[must_use]
    pub fn flute_length(&self) -> Option<f64> {
        self.profile.top_of_role(ElementRole::Cutting)
    }

    /// True if any element is tagged [`ElementRole::Holder`].
    #[must_use]
    pub fn has_holder(&self) -> bool {
        self.profile
            .elements()
            .iter()
            .any(|e| e.role == ElementRole::Holder)
    }
}

impl Hashable for Tool {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Tool");
        h.add(&self.id);
        h.str(&self.description);
        h.add(&self.profile);
        h.f64(self.gauge_length);
        h.end();
    }
}
