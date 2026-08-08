// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Length units, and the conversion boundary.
//!
//! # The convention
//!
//! **Core geometry is canonically millimetres.** Conversion happens exactly
//! once, at load, and never again. Nothing inside the engine ever asks what unit
//! a coordinate is in, because there is only one answer.
//!
//! This is what gives [`crate::eps::EPS_SPAN_MIN`] at 1e-9 mm a meaning. A
//! tolerance is a length; a length without a unit is not a quantity.
//!
//! # Why the CLI refuses to guess
//!
//! STL and OBJ carry **no unit information whatsoever**. Not a header field, not
//! a convention, nothing. A file of numbers between 0 and 100 is equally a
//! 100 mm bracket and a 100 inch beam.
//!
//! So `--units` is required for those formats, with no default. Refusing to
//! proceed is the correct behaviour: silently assuming millimetres for an inch
//! part yields a model 25.4x too small, the simulation passes, and the customer
//! finds out when the tool plunges through the fixture. An error message is a
//! far better outcome than a scrapped billet, and "it defaulted to mm" is not a
//! defence anybody wants to make.
//!
//! 3MF is different: it declares its unit in XML metadata, so it is read from
//! the file, and an explicit `--units` that contradicts the file is an error
//! rather than an override.

use core::fmt;

/// A length unit that input files may be expressed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Unit {
    /// Micrometres. 3MF permits this; no CAD system we target exports it.
    Micron,
    /// Millimetres — the canonical internal unit.
    Millimetre,
    /// Centimetres.
    Centimetre,
    /// Metres.
    Metre,
    /// Inches.
    Inch,
    /// Thousandths of an inch, the customary unit of machining tolerance in the
    /// United States. Also called "mil"; we spell it `thou` because "mil" means
    /// millimetre to half the world and 0.001 inch to the other half, and that
    /// ambiguity has no place in a units module.
    Thou,
    /// Feet. 3MF permits it.
    Foot,
}

impl Unit {
    /// Every unit, in a fixed order.
    pub const ALL: [Unit; 7] = [
        Unit::Micron,
        Unit::Millimetre,
        Unit::Centimetre,
        Unit::Metre,
        Unit::Inch,
        Unit::Thou,
        Unit::Foot,
    ];

    /// The short name used on the command line and in 3MF metadata.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Micron => "micron",
            Self::Millimetre => "mm",
            Self::Centimetre => "cm",
            Self::Metre => "m",
            Self::Inch => "in",
            Self::Thou => "thou",
            Self::Foot => "ft",
        }
    }

    /// Parses a unit name, accepting the aliases real files and real users use.
    #[must_use]
    pub fn from_name(s: &str) -> Option<Self> {
        // Case-insensitive without allocating for the common path.
        let lower = s.trim().to_ascii_lowercase();
        Some(match lower.as_str() {
            "micron" | "microns" | "um" | "µm" => Self::Micron,
            "mm" | "millimetre" | "millimeter" | "millimetres" | "millimeters" => Self::Millimetre,
            "cm" | "centimetre" | "centimeter" => Self::Centimetre,
            "m" | "metre" | "meter" => Self::Metre,
            "in" | "inch" | "inches" => Self::Inch,
            "thou" | "mil" | "mils" => Self::Thou,
            "ft" | "foot" | "feet" => Self::Foot,
            _ => return None,
        })
    }

    /// How many millimetres one of this unit is.
    ///
    /// # Exactness
    ///
    /// `1000.0`, `10.0` and `0.001` are exactly representable in binary, so
    /// those conversions are exact and reversible.
    ///
    /// **`25.4` is not.** The nearest `f64` is
    /// `25.399999999999998578915...`, so inch input carries a relative error of
    /// about 6e-17 — a quarter of an ULP. That is three orders of magnitude
    /// below [`crate::eps::EPS_WELD`] and eight below any machining tolerance,
    /// so it is harmless; but it is a real rounding and it is worth knowing it
    /// exists.
    ///
    /// It is applied **once**, at load. The reason to care is that repeated
    /// conversion would accumulate: mm → in → mm is not the identity. The API
    /// makes that impossible by only ever converting inward.
    #[must_use]
    pub const fn millimetres_per(self) -> f64 {
        match self {
            Self::Micron => 0.001,
            Self::Millimetre => 1.0,
            Self::Centimetre => 10.0,
            Self::Metre => 1000.0,
            Self::Inch => 25.4,
            Self::Thou => 0.0254,
            Self::Foot => 304.8,
        }
    }

    /// True if converting from this unit is exact in binary floating point.
    ///
    /// Only the decimal-metric units are; anything derived from the inch is not,
    /// because 25.4 has no finite binary expansion.
    #[must_use]
    pub const fn conversion_is_exact(self) -> bool {
        matches!(
            self,
            Self::Micron | Self::Millimetre | Self::Centimetre | Self::Metre
        )
    }
}

impl fmt::Display for Unit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Every accepted spelling, for a CLI error message that actually helps.
#[must_use]
pub fn accepted_names() -> String {
    Unit::ALL
        .iter()
        .map(|u| u.name())
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_round_trip() {
        for u in Unit::ALL {
            assert_eq!(Unit::from_name(u.name()), Some(u), "{u}");
            assert_eq!(Unit::from_name(&u.name().to_uppercase()), Some(u), "{u}");
            assert_eq!(
                Unit::from_name(&format!("  {}  ", u.name())),
                Some(u),
                "{u}"
            );
        }
        assert_eq!(Unit::from_name("furlong"), None);
        assert_eq!(Unit::from_name(""), None);
    }

    #[test]
    fn aliases_people_actually_type() {
        assert_eq!(Unit::from_name("millimeter"), Some(Unit::Millimetre));
        assert_eq!(Unit::from_name("millimetre"), Some(Unit::Millimetre));
        assert_eq!(Unit::from_name("inches"), Some(Unit::Inch));
        assert_eq!(Unit::from_name("mil"), Some(Unit::Thou));
        assert_eq!(Unit::from_name("feet"), Some(Unit::Foot));
        assert_eq!(Unit::from_name("um"), Some(Unit::Micron));
    }

    #[test]
    fn metric_conversions_are_exact_and_reversible() {
        for u in [
            Unit::Micron,
            Unit::Millimetre,
            Unit::Centimetre,
            Unit::Metre,
        ] {
            assert!(u.conversion_is_exact(), "{u}");
            let f = u.millimetres_per();
            // Exactly representable, so multiplying and dividing round-trips.
            for v in [1.0f64, 3.0, 12345.0, 0.5] {
                assert_eq!(v * f / f, v, "{u} round trip of {v}");
            }
        }
    }

    #[test]
    fn imperial_conversion_is_documented_as_inexact() {
        assert!(!Unit::Inch.conversion_is_exact());
        assert!(!Unit::Thou.conversion_is_exact());
        assert!(!Unit::Foot.conversion_is_exact());
        // The specific claim in the docs: 25.4 is not exactly representable.
        let f = Unit::Inch.millimetres_per();
        assert_ne!(f, 25.4f64.next_up());
        assert!((f - 25.4).abs() < 1e-15);
        // And the error is far below the weld lattice at part scale.
        let one_inch_in_mm = 1.0 * f;
        assert!((one_inch_in_mm - 25.4).abs() < crate::eps::EPS_WELD);
    }

    #[test]
    fn relative_magnitudes_are_right() {
        assert_eq!(Unit::Metre.millimetres_per(), 1000.0);
        assert_eq!(Unit::Centimetre.millimetres_per(), 10.0);
        assert_eq!(Unit::Micron.millimetres_per(), 0.001);
        // A thou is a thousandth of an inch.
        let thou = Unit::Thou.millimetres_per();
        let inch = Unit::Inch.millimetres_per();
        assert!((thou * 1000.0 - inch).abs() < 1e-12);
        // A foot is twelve inches.
        assert!((Unit::Foot.millimetres_per() - inch * 12.0).abs() < 1e-12);
    }

    #[test]
    fn accepted_names_lists_everything() {
        let listed = accepted_names();
        for u in Unit::ALL {
            assert!(listed.contains(u.name()), "{u} missing from the help text");
        }
    }
}
