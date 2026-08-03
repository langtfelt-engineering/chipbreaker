// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Constructors for the standard tool forms, and for holder stacks.
//!
//! # Why these are constructors and not data
//!
//! Every form here reduces to a chain of segments and arcs — nothing in the rest
//! of the engine knows that a ball nose is a ball nose. What the constructors buy
//! is that the chain is *built correctly*: tangent where it should be tangent,
//! closed where it should be closed, and rejected when the dimensions do not
//! describe a tool. A corner radius larger than the tool radius, a flute length
//! shorter than the ball it sits on, a point angle of zero — each is a plausible
//! typo in a tool library, and each produces a profile that validates as
//! geometry while being nonsense as a cutter.
//!
//! # Angles are in degrees
//!
//! Every angle in this module is in degrees, because every drawing, catalogue,
//! and machine tool table gives them in degrees, and a unit conversion buried in
//! the caller is a unit conversion that will eventually be forgotten. The
//! conversion happens once, here.

use crate::eps::EPS_LENGTH;
use crate::transcendental as t;

use super::profile::{
    ArcDirection, ElementRole, Profile, ProfileElement, ProfileError, RoledElement,
};
use crate::math::Vec2;

use core::f64::consts::PI;
use core::fmt;

/// Why a set of tool dimensions was rejected.
#[derive(Debug, Clone, PartialEq)]
pub enum CatalogError {
    /// A dimension that must be strictly positive was not.
    NotPositive {
        /// Which parameter.
        parameter: &'static str,
        /// What was supplied.
        value: f64,
    },
    /// A dimension that must be non-negative and finite was not.
    NotFinite {
        /// Which parameter.
        parameter: &'static str,
        /// What was supplied.
        value: f64,
    },
    /// A dimension fell outside the range in which the form makes sense.
    OutOfRange {
        /// Which parameter.
        parameter: &'static str,
        /// What was supplied.
        value: f64,
        /// Lowest acceptable value.
        low: f64,
        /// Highest acceptable value.
        high: f64,
    },
    /// A length was shorter than the geometry it has to contain.
    TooShort {
        /// Which parameter.
        parameter: &'static str,
        /// What was supplied.
        value: f64,
        /// The shortest value the rest of the dimensions allow.
        minimum: f64,
        /// What forces that minimum.
        because: &'static str,
    },
    /// A holder stack with no stages.
    EmptyHolder,
    /// The assembled chain failed profile validation. Should not happen for
    /// dimensions that pass the checks above; if it does, it is a bug here
    /// rather than in the caller's data.
    Profile(ProfileError),
}

impl fmt::Display for CatalogError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotPositive { parameter, value } => {
                write!(f, "{parameter} must be greater than zero, got {value}")
            }
            Self::NotFinite { parameter, value } => {
                write!(
                    f,
                    "{parameter} must be finite and non-negative, got {value}"
                )
            }
            Self::OutOfRange {
                parameter,
                value,
                low,
                high,
            } => write!(f, "{parameter} must lie in [{low}, {high}], got {value}"),
            Self::TooShort {
                parameter,
                value,
                minimum,
                because,
            } => write!(
                f,
                "{parameter} is {value} but must be at least {minimum}, because {because}"
            ),
            Self::EmptyHolder => write!(f, "a holder stack needs at least one stage"),
            Self::Profile(e) => write!(f, "{e}"),
        }
    }
}

impl core::error::Error for CatalogError {}

impl From<ProfileError> for CatalogError {
    fn from(e: ProfileError) -> Self {
        Self::Profile(e)
    }
}

fn positive(parameter: &'static str, value: f64) -> Result<f64, CatalogError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(CatalogError::NotPositive { parameter, value })
    }
}

fn non_negative(parameter: &'static str, value: f64) -> Result<f64, CatalogError> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(CatalogError::NotFinite { parameter, value })
    }
}

fn in_range(parameter: &'static str, value: f64, low: f64, high: f64) -> Result<f64, CatalogError> {
    if value.is_finite() && value >= low && value <= high {
        Ok(value)
    } else {
        Err(CatalogError::OutOfRange {
            parameter,
            value,
            low,
            high,
        })
    }
}

fn at_least(
    parameter: &'static str,
    value: f64,
    minimum: f64,
    because: &'static str,
) -> Result<f64, CatalogError> {
    if value.is_finite() && value >= minimum {
        Ok(value)
    } else {
        Err(CatalogError::TooShort {
            parameter,
            value,
            minimum,
            because,
        })
    }
}

/// Accumulates a profile chain, dropping the degenerate pieces that standard
/// dimensions routinely produce.
///
/// A bull nose whose corner radius equals its tool radius *is* a ball nose, and
/// its flat bottom is a zero-length segment; a cutter whose shank matches its
/// cutting diameter has no step. Emitting those would fail validation for what
/// is a perfectly ordinary tool, so the builder drops anything shorter than
/// [`EPS_LENGTH`] rather than making every constructor special-case it.
struct Chain {
    elements: Vec<RoledElement>,
    cursor: Vec2,
}

impl Chain {
    fn new() -> Self {
        Self {
            elements: Vec::new(),
            cursor: Vec2::new(0.0, 0.0),
        }
    }

    /// Starts a chain from elements the caller built, taken verbatim.
    ///
    /// Nothing is dropped or adjusted here, so a chain that does not begin at
    /// the tip or does not join up reaches [`Profile::new`] intact and is
    /// reported against the caller's own element indices.
    fn from_elements(elements: Vec<RoledElement>) -> Self {
        let cursor = elements
            .last()
            .map_or(Vec2::new(0.0, 0.0), |e| e.element.end());
        Self { elements, cursor }
    }

    /// Extends the chain to `(r, z)` with a straight segment.
    fn line_to(&mut self, r: f64, z: f64, role: ElementRole) {
        let end = Vec2::new(r, z);
        if (end - self.cursor).length() > EPS_LENGTH {
            self.elements.push(RoledElement {
                element: ProfileElement::Segment {
                    start: self.cursor,
                    end,
                },
                role,
            });
            self.cursor = end;
        }
    }

    /// Extends the chain to `(r, z)` with a circular arc about `center`.
    fn arc_to(&mut self, r: f64, z: f64, center: Vec2, direction: ArcDirection, role: ElementRole) {
        let end = Vec2::new(r, z);
        if (end - self.cursor).length() > EPS_LENGTH {
            self.elements.push(RoledElement {
                element: ProfileElement::Arc {
                    start: self.cursor,
                    end,
                    center,
                    direction,
                },
                role,
            });
            self.cursor = end;
        }
    }

    fn build(self) -> Result<Profile, CatalogError> {
        Ok(Profile::new(self.elements)?)
    }
}

/// One stage of a holder: a cylinder, or a frustum when the two diameters
/// differ.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HolderStage {
    /// Diameter at the bottom of the stage, nearest the tool.
    pub bottom_diameter: f64,
    /// Diameter at the top. Equal to `bottom_diameter` for a plain cylinder.
    pub top_diameter: f64,
    /// Height of the stage along the axis.
    pub length: f64,
}

impl HolderStage {
    /// A plain cylinder.
    #[must_use]
    pub const fn cylinder(diameter: f64, length: f64) -> Self {
        Self {
            bottom_diameter: diameter,
            top_diameter: diameter,
            length,
        }
    }

    /// A frustum, wider or narrower at the top.
    #[must_use]
    pub const fn taper(bottom_diameter: f64, top_diameter: f64, length: f64) -> Self {
        Self {
            bottom_diameter,
            top_diameter,
            length,
        }
    }
}

/// Everything above the cutting geometry: the shank, and optionally a holder.
#[derive(Debug, Clone, PartialEq)]
pub struct Shank {
    /// Diameter of the shank.
    pub diameter: f64,
    /// Distance from the tip to the top of the shank.
    pub overall_length: f64,
    /// Holder stages stacked above the shank, from the bottom up.
    pub holder: Vec<HolderStage>,
}

impl Shank {
    /// A shank with no holder.
    #[must_use]
    pub fn plain(diameter: f64, overall_length: f64) -> Self {
        Self {
            diameter,
            overall_length,
            holder: Vec::new(),
        }
    }

    /// A shank with a holder stack above it.
    #[must_use]
    pub fn with_holder(
        diameter: f64,
        overall_length: f64,
        holder: impl IntoIterator<Item = HolderStage>,
    ) -> Self {
        Self {
            diameter,
            overall_length,
            holder: holder.into_iter().collect(),
        }
    }

    /// Appends the shank and holder to a chain that has reached the top of the
    /// cutting geometry at `(flute_radius, flute_top)`.
    fn append(&self, chain: &mut Chain, flute_top: f64) -> Result<(), CatalogError> {
        positive("shank diameter", self.diameter)?;
        at_least(
            "overall length",
            self.overall_length,
            flute_top,
            "the shank cannot begin below the top of the flutes",
        )?;

        // The step from the cutting diameter to the shank diameter. Horizontal,
        // so it revolves to an annular disc — which is exactly what a neck
        // relief or an overhanging cutter head is.
        chain.line_to(0.5 * self.diameter, flute_top, ElementRole::NonCutting);
        chain.line_to(
            0.5 * self.diameter,
            self.overall_length,
            ElementRole::NonCutting,
        );

        let mut z = self.overall_length;
        for stage in &self.holder {
            positive("holder stage bottom diameter", stage.bottom_diameter)?;
            positive("holder stage top diameter", stage.top_diameter)?;
            positive("holder stage length", stage.length)?;
            chain.line_to(0.5 * stage.bottom_diameter, z, ElementRole::Holder);
            z += stage.length;
            chain.line_to(0.5 * stage.top_diameter, z, ElementRole::Holder);
        }
        Ok(())
    }
}

/// Half of an included angle, in radians, guarded against the degenerate ends.
fn half_angle(parameter: &'static str, degrees: f64) -> Result<f64, CatalogError> {
    // Strictly inside (0, 180): zero is a cylinder and 180 is a flat disc, and
    // both have a dedicated constructor.
    in_range(parameter, degrees, 1.0e-6, 180.0 - 1.0e-6)?;
    Ok(0.5 * degrees * PI / 180.0)
}

/// A flat end mill: a cylinder with a square end.
///
/// # Errors
/// See [`CatalogError`].
pub fn flat_end_mill(
    diameter: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let l = positive("flute length", flute_length)?;

    let mut chain = Chain::new();
    chain.line_to(0.5 * d, 0.0, ElementRole::Cutting);
    chain.line_to(0.5 * d, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A ball-nose end mill: a hemisphere of radius `diameter / 2` on a cylinder.
///
/// # Errors
/// See [`CatalogError`]. The flute length must reach at least the top of the
/// ball, since below that the tool is not yet at full diameter.
pub fn ball_end_mill(
    diameter: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let radius = 0.5 * d;
    let l = at_least(
        "flute length",
        flute_length,
        radius,
        "the tool is not at full diameter until the top of the ball",
    )?;

    let mut chain = Chain::new();
    chain.arc_to(
        radius,
        radius,
        Vec2::new(0.0, radius),
        ArcDirection::CounterClockwise,
        ElementRole::Cutting,
    );
    chain.line_to(radius, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A bull-nose (toroidal) end mill: a flat end with a radiused corner.
///
/// A corner radius equal to the tool radius gives a ball nose, and the flat
/// bottom vanishes rather than becoming a degenerate element.
///
/// # Errors
/// See [`CatalogError`].
pub fn bull_end_mill(
    diameter: f64,
    corner_radius: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let radius = 0.5 * d;
    let corner = in_range("corner radius", corner_radius, 0.0, radius)?;
    let l = at_least(
        "flute length",
        flute_length,
        corner,
        "the tool is not at full diameter until the top of the corner radius",
    )?;

    let mut chain = Chain::new();
    chain.line_to(radius - corner, 0.0, ElementRole::Cutting);
    chain.arc_to(
        radius,
        corner,
        Vec2::new(radius - corner, corner),
        ArcDirection::CounterClockwise,
        ElementRole::Cutting,
    );
    chain.line_to(radius, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A chamfer mill or V-bit: a cone rising from a flat tip to full diameter.
///
/// `tip_diameter` of zero gives a true V-bit with a point. `included_angle` is
/// the full angle at the point, measured across the axis, as engraving-tool
/// catalogues give it.
///
/// # Errors
/// See [`CatalogError`].
pub fn chamfer_mill(
    diameter: f64,
    tip_diameter: f64,
    included_angle_degrees: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let tip = in_range("tip diameter", tip_diameter, 0.0, d)?;
    let half = half_angle("included angle", included_angle_degrees)?;

    // Rise from the tip diameter to full diameter at the cone's half-angle,
    // which is measured from the axis.
    let cone_height = (0.5 * d - 0.5 * tip) / t::tan(half);
    let l = at_least(
        "flute length",
        flute_length,
        cone_height,
        "the cone has not reached full diameter below that height",
    )?;

    let mut chain = Chain::new();
    chain.line_to(0.5 * tip, 0.0, ElementRole::Cutting);
    chain.line_to(0.5 * d, cone_height, ElementRole::Cutting);
    chain.line_to(0.5 * d, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A tapered end mill: a cone that is still widening where the flutes end.
///
/// Unlike [`chamfer_mill`], the diameter is set by the flute length rather than
/// given, because that is how tapered cutters are specified: a tip diameter, a
/// per-side taper, and a depth.
///
/// `included_angle` is the full angle across the axis, so a "3 degree per side"
/// taper is 6 degrees here.
///
/// # Errors
/// See [`CatalogError`].
pub fn tapered_end_mill(
    tip_diameter: f64,
    included_angle_degrees: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let tip = non_negative("tip diameter", tip_diameter)?;
    let half = half_angle("included angle", included_angle_degrees)?;
    let l = positive("flute length", flute_length)?;

    let top_radius = 0.5 * tip + l * t::tan(half);

    let mut chain = Chain::new();
    chain.line_to(0.5 * tip, 0.0, ElementRole::Cutting);
    chain.line_to(top_radius, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A twist drill: a conical point on a cylinder.
///
/// `point_angle` is the full included angle at the point — 118 degrees for
/// general-purpose drills, 135 for split points.
///
/// The point is modelled as a plain cone. A real drill point has a chisel edge
/// and relief that a cone does not, but neither changes the swept envelope,
/// which is what removes material.
///
/// # Errors
/// See [`CatalogError`].
pub fn drill(
    diameter: f64,
    point_angle_degrees: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let half = half_angle("point angle", point_angle_degrees)?;
    let point_height = 0.5 * d / t::tan(half);
    let l = at_least(
        "flute length",
        flute_length,
        point_height,
        "the drill is not at full diameter below the top of its point",
    )?;

    let mut chain = Chain::new();
    chain.line_to(0.5 * d, point_height, ElementRole::Cutting);
    chain.line_to(0.5 * d, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// A barrel (circle-segment) cutter: one large-radius arc from the tip to the
/// widest point.
///
/// `barrel_radius` is the radius of that arc. It must be at least the tool
/// radius; equal to it, the arc *is* a hemisphere and the result is exactly the
/// ball nose that [`ball_end_mill`] produces. Larger values flatten the form,
/// which is the whole point of a barrel cutter: a 200 mm arc on a 12 mm tool
/// leaves a scallop a fortieth the height of a ball nose at the same stepover.
///
/// # Errors
/// See [`CatalogError`].
pub fn barrel_end_mill(
    diameter: f64,
    barrel_radius: f64,
    flute_length: f64,
    shank: &Shank,
) -> Result<Profile, CatalogError> {
    let d = positive("diameter", diameter)?;
    let radius = 0.5 * d;
    let barrel = at_least(
        "barrel radius",
        barrel_radius,
        radius,
        "a barrel narrower than a ball is not a barrel",
    )?;

    // Centre the arc so that it passes through the tip and reaches r = radius.
    // Its centre sits at r = radius - barrel, which is at or left of the axis,
    // and at the height where the tool is widest:
    //   (0 - centre_r)^2 + (0 - centre_z)^2 = barrel^2
    let centre_r = radius - barrel;
    let centre_z = (barrel * barrel - centre_r * centre_r).sqrt();
    let l = at_least(
        "flute length",
        flute_length,
        centre_z,
        "the tool is not at full diameter until the widest point of the barrel",
    )?;

    let mut chain = Chain::new();
    chain.arc_to(
        radius,
        centre_z,
        Vec2::new(centre_r, centre_z),
        ArcDirection::CounterClockwise,
        ElementRole::Cutting,
    );
    chain.line_to(radius, l, ElementRole::Cutting);
    shank.append(&mut chain, l)?;
    chain.build()
}

/// An arbitrary cutting profile with a standard shank and holder above it.
///
/// The caller supplies the cutting geometry as a chain from the tip; this
/// validates it, then appends the shank and holder in the same way every other
/// constructor does. For form tools and ground profiles that match no catalogue
/// entry.
///
/// # Errors
/// See [`CatalogError`]. The cutting chain must begin at the tip and be
/// continuous, as for any profile.
pub fn form_tool(cutting: &[ProfileElement], shank: &Shank) -> Result<Profile, CatalogError> {
    let mut chain =
        Chain::from_elements(cutting.iter().copied().map(RoledElement::cutting).collect());
    let flute_top = chain.cursor.y;
    shank.append(&mut chain, flute_top)?;
    chain.build()
}

/// A holder stack on its own, with no tool below it.
///
/// Used to check a holder against fixtures independently of what is gripped in
/// it. The chain still starts at the origin, so the first stage's bottom face is
/// the datum.
///
/// # Errors
/// See [`CatalogError`].
pub fn holder_stack(stages: &[HolderStage]) -> Result<Profile, CatalogError> {
    if stages.is_empty() {
        return Err(CatalogError::EmptyHolder);
    }
    let mut chain = Chain::new();
    let mut z = 0.0;
    for stage in stages {
        positive("holder stage bottom diameter", stage.bottom_diameter)?;
        positive("holder stage top diameter", stage.top_diameter)?;
        positive("holder stage length", stage.length)?;
        chain.line_to(0.5 * stage.bottom_diameter, z, ElementRole::Holder);
        z += stage.length;
        chain.line_to(0.5 * stage.top_diameter, z, ElementRole::Holder);
    }
    chain.build()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shank() -> Shank {
        Shank::plain(6.0, 60.0)
    }

    fn roles(p: &Profile) -> Vec<ElementRole> {
        p.elements().iter().map(|e| e.role).collect()
    }

    #[test]
    fn a_flat_end_mill_is_a_bottom_face_a_side_and_a_shank() {
        let p = flat_end_mill(6.0, 20.0, &shank()).expect("valid dimensions");
        // Bottom face, side, shank. The step is dropped: the shank is the same
        // diameter as the cutter.
        assert_eq!(
            roles(&p),
            vec![
                ElementRole::Cutting,
                ElementRole::Cutting,
                ElementRole::NonCutting
            ]
        );
        assert!((p.max_radius() - 3.0).abs() < 1e-12);
        assert!((p.total_length() - 60.0).abs() < 1e-12);
        assert_eq!(p.top_of_role(ElementRole::Cutting), Some(20.0));
    }

    #[test]
    fn a_bull_nose_with_full_corner_radius_is_exactly_a_ball_nose() {
        let ball = ball_end_mill(8.0, 25.0, &shank()).expect("valid");
        let bull = bull_end_mill(8.0, 4.0, 25.0, &shank()).expect("valid");
        assert_eq!(
            ball, bull,
            "a bull nose radiused to the tool radius is a ball nose, \
             and its flat bottom must vanish rather than become a degenerate element"
        );
    }

    #[test]
    fn a_barrel_of_ball_radius_is_exactly_a_ball_nose() {
        let ball = ball_end_mill(8.0, 25.0, &shank()).expect("valid");
        let barrel = barrel_end_mill(8.0, 4.0, 25.0, &shank()).expect("valid");
        assert_eq!(ball, barrel, "the limiting case of a barrel is a ball");
    }

    #[test]
    fn a_barrel_reaches_full_diameter_higher_up_than_a_ball_does() {
        let barrel = barrel_end_mill(12.0, 200.0, 60.0, &Shank::plain(12.0, 100.0)).expect("valid");
        assert!((barrel.max_radius() - 6.0).abs() < 1e-9);
        // A 200 mm arc through the tip of a 12 mm tool reaches full width at
        // sqrt(200^2 - 194^2) = 48.6 mm, where a ball nose would take 6 mm.
        let top_of_arc = barrel.elements()[0].element.end().y;
        let expected = (200.0f64 * 200.0 - 194.0 * 194.0).sqrt();
        assert!(
            (top_of_arc - expected).abs() < 1e-9,
            "{top_of_arc} vs {expected}"
        );
    }

    #[test]
    fn a_barrel_flatter_than_a_ball_is_rejected() {
        let err = barrel_end_mill(8.0, 3.0, 25.0, &shank()).expect_err("3 < 4");
        assert!(matches!(err, CatalogError::TooShort { .. }), "{err:?}");
    }

    #[test]
    fn a_ball_nose_shorter_than_its_own_ball_is_rejected() {
        let err = ball_end_mill(8.0, 3.0, &shank()).expect_err("the ball alone is 4 mm tall");
        match err {
            CatalogError::TooShort { value, minimum, .. } => {
                assert!((value - 3.0).abs() < 1e-12);
                assert!((minimum - 4.0).abs() < 1e-12);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_corner_radius_larger_than_the_tool_radius_is_rejected() {
        let err = bull_end_mill(8.0, 5.0, 25.0, &shank()).expect_err("5 > 4");
        assert!(matches!(err, CatalogError::OutOfRange { .. }), "{err:?}");
    }

    #[test]
    fn a_ninety_degree_v_bit_rises_at_forty_five_degrees() {
        let p = chamfer_mill(10.0, 0.0, 90.0, 20.0, &shank()).expect("valid");
        // Half-angle 45 degrees, so the cone reaches r = 5 at z = 5.
        let cone = p.elements()[0].element;
        assert!((cone.start().x).abs() < 1e-12);
        assert!((cone.end().x - 5.0).abs() < 1e-12);
        assert!((cone.end().y - 5.0).abs() < 1e-9, "{:?}", cone.end());
    }

    #[test]
    fn a_chamfer_mill_with_a_flat_tip_starts_with_that_flat() {
        let p = chamfer_mill(10.0, 2.0, 90.0, 20.0, &shank()).expect("valid");
        let flat = p.elements()[0].element;
        assert!(
            (flat.end().x - 1.0).abs() < 1e-12,
            "half of the tip diameter"
        );
        assert!(flat.end().y.abs() < 1e-12, "and still on the bottom face");
    }

    #[test]
    fn a_118_degree_drill_point_has_the_height_trigonometry_says() {
        let p = drill(10.0, 118.0, 40.0, &shank()).expect("valid");
        let point = p.elements()[0].element;
        let expected = 5.0 / t::tan(0.5 * 118.0 * PI / 180.0);
        assert!(
            (point.end().y - expected).abs() < 1e-12,
            "{} vs {expected}",
            point.end().y
        );
        assert!((point.end().x - 5.0).abs() < 1e-12);
    }

    #[test]
    fn a_tapered_mill_widens_by_the_taper_over_the_flute_length() {
        // 6 degrees included is 3 degrees per side.
        let p = tapered_end_mill(2.0, 6.0, 20.0, &Shank::plain(10.0, 60.0)).expect("valid");
        let expected = 1.0 + 20.0 * t::tan(3.0 * PI / 180.0);
        assert!(
            (p.max_radius() - 5.0).abs() < 1e-12,
            "the shank is the widest part here"
        );
        let cone = p.elements()[1].element;
        assert!(
            (cone.end().x - expected).abs() < 1e-12,
            "{} vs {expected}",
            cone.end().x
        );
    }

    #[test]
    fn a_holder_stack_stacks_upward_and_is_tagged_holder() {
        let p = flat_end_mill(
            6.0,
            20.0,
            &Shank::with_holder(
                6.0,
                40.0,
                [
                    HolderStage::cylinder(25.0, 30.0),
                    HolderStage::taper(25.0, 45.0, 20.0),
                ],
            ),
        )
        .expect("valid");
        assert!((p.total_length() - 90.0).abs() < 1e-12, "40 + 30 + 20");
        assert!((p.max_radius() - 22.5).abs() < 1e-12);
        assert_eq!(p.top_of_role(ElementRole::Cutting), Some(20.0));
        assert!(
            p.top_of_role(ElementRole::Holder).is_some(),
            "the holder must be tagged, or U8 cannot tell a crash from a cut"
        );
    }

    #[test]
    fn a_holder_stack_on_its_own_is_a_valid_profile() {
        let p = holder_stack(&[
            HolderStage::cylinder(30.0, 40.0),
            HolderStage::taper(30.0, 50.0, 25.0),
        ])
        .expect("valid");
        assert!(p.elements().iter().all(|e| e.role == ElementRole::Holder));
        assert!((p.total_length() - 65.0).abs() < 1e-12);
        assert!((p.max_radius() - 25.0).abs() < 1e-12);
    }

    #[test]
    fn an_empty_holder_stack_is_rejected() {
        assert_eq!(holder_stack(&[]), Err(CatalogError::EmptyHolder));
    }

    #[test]
    fn a_shank_that_would_begin_below_the_flutes_is_rejected() {
        let err = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 10.0))
            .expect_err("overall length 10 is below the 20 mm flute");
        assert!(matches!(err, CatalogError::TooShort { .. }), "{err:?}");
    }

    #[test]
    fn degenerate_angles_are_rejected_rather_than_producing_infinities() {
        assert!(
            drill(6.0, 0.0, 20.0, &shank()).is_err(),
            "a zero point angle"
        );
        assert!(
            drill(6.0, 180.0, 20.0, &shank()).is_err(),
            "a flat-bottomed drill is a flat end mill, not a drill"
        );
        assert!(chamfer_mill(10.0, 0.0, 0.0, 20.0, &shank()).is_err());
        assert!(tapered_end_mill(2.0, 180.0, 20.0, &shank()).is_err());
    }

    #[test]
    fn non_positive_dimensions_are_rejected() {
        assert!(flat_end_mill(0.0, 20.0, &shank()).is_err());
        assert!(flat_end_mill(-6.0, 20.0, &shank()).is_err());
        assert!(flat_end_mill(6.0, 0.0, &shank()).is_err());
        assert!(flat_end_mill(f64::NAN, 20.0, &shank()).is_err());
        assert!(flat_end_mill(6.0, 20.0, &Shank::plain(0.0, 60.0)).is_err());
    }

    #[test]
    fn a_form_tool_takes_the_callers_chain_verbatim() {
        let cutting = [
            ProfileElement::Segment {
                start: Vec2::new(0.0, 0.0),
                end: Vec2::new(2.0, 0.0),
            },
            ProfileElement::Arc {
                start: Vec2::new(2.0, 0.0),
                end: Vec2::new(4.0, 2.0),
                center: Vec2::new(2.0, 2.0),
                direction: ArcDirection::CounterClockwise,
            },
        ];
        let p = form_tool(&cutting, &Shank::plain(8.0, 50.0)).expect("valid");
        assert_eq!(p.elements()[0].element, cutting[0]);
        assert_eq!(p.elements()[1].element, cutting[1]);
        assert_eq!(p.top_of_role(ElementRole::Cutting), Some(2.0));
    }

    #[test]
    fn a_form_tool_chain_that_does_not_start_at_the_tip_is_reported_not_repaired() {
        let cutting = [ProfileElement::Segment {
            start: Vec2::new(1.0, 0.0),
            end: Vec2::new(3.0, 0.0),
        }];
        let err = form_tool(&cutting, &Shank::plain(8.0, 50.0))
            .expect_err("the caller's chain is taken as given");
        assert!(
            matches!(
                err,
                CatalogError::Profile(ProfileError::TipNotAtOrigin { .. })
            ),
            "{err:?}"
        );
    }
}
