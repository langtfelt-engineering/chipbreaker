// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Turning `I`/`J`/`K` or `R` into one unambiguous arc.
//!
//! # The two forms must agree
//!
//! The same arc can be written either way, and both must resolve to
//! byte-identical [`ArcData`]. That equivalence is the cheapest available test
//! for a sign error in either path, and it is required by the definition of
//! done.
//!
//! # Why `R` is the dangerous one
//!
//! The centre of an `R`-form arc sits on the perpendicular bisector of the
//! chord, at a distance `sqrt(R^2 - h^2)` from its midpoint where `h` is half
//! the chord. As the chord approaches `2R` — a sweep approaching 180 degrees —
//! that square root approaches zero and **its derivative approaches infinity**.
//! A rounding of one micron in an endpoint then moves the centre by millimetres.
//!
//! There is no tolerance that rescues this; the information is not in the file.
//! So the conditioning is computed, reported, and beyond a threshold the arc is
//! refused with an error that says to write it with `I`/`J`/`K` instead.
//!
//! The sign of `R` chooses which arc: positive takes the minor one (sweep at
//! most 180 degrees), negative the major one.
//!
//! # Radius mismatch
//!
//! CAM posts rounded coordinates, so the distance from the given centre to the
//! start almost never equals the distance to the end exactly. Real controls
//! disagree about this: LinuxCNC rejects beyond a tolerance, Fanuc silently
//! adjusts. The policy here is to accept within a tolerance, move the centre to
//! the point that splits the difference, and **record the residual** so that U13
//! can tell a surface deviation caused by geometry from one caused by rounding.

use chipbreaker_core::math::Vec3;
use chipbreaker_core::toolpath::{ArcData, ArcForm, ArcPlane};
use chipbreaker_core::transcendental as t;

use crate::diag::{Diagnostics, GcodeError, GcodeWarning, Site};

/// Default arc radius mismatch tolerance, in millimetres.
///
/// LinuxCNC's default, and below any cut this engine would call a defect. Ten
/// times tighter would reject files that real controls run correctly, which for
/// a verification tool means refusing to check programs that are fine.
pub const DEFAULT_ARC_TOLERANCE: f64 = 0.01;

/// How close to a half-turn an `R`-form arc may come before its centre stops
/// being determined by its endpoints.
///
/// Expressed as a fraction: the arc is refused when `half_chord > radius *
/// (1 - RADIUS_FORM_MARGIN)`. At this margin the centre's sensitivity to an
/// endpoint is about `1/sqrt(2 * 0.0005)` — some thirty times — which is the
/// point at which a micron of rounding becomes tens of microns of centre error.
pub const RADIUS_FORM_MARGIN: f64 = 5.0e-4;

/// Which way an arc turns, from the `G` code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Turn {
    /// `G2`.
    Clockwise,
    /// `G3`.
    CounterClockwise,
}

impl Turn {
    /// Sign of the sweep about the plane's right-handed normal.
    ///
    /// `G2` is clockwise *looking along the negative normal* — that is, looking
    /// down at the XY plane from `+Z` — which makes it a negative rotation about
    /// the normal itself.
    #[must_use]
    pub const fn sign(self) -> f64 {
        match self {
            Self::Clockwise => -1.0,
            Self::CounterClockwise => 1.0,
        }
    }
}

/// A point in the arc's own plane, in the plane's axis order.
///
/// A named type rather than loose pairs of floats: every routine below takes a
/// start, an end and a centre, and six positional `f64` arguments is six chances
/// to transpose two of them in a way that still compiles.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Planar {
    u: f64,
    v: f64,
}

impl Planar {
    const fn new(u: f64, v: f64) -> Self {
        Self { u, v }
    }

    fn distance_to(self, other: Self) -> f64 {
        t::hypot(self.u - other.u, self.v - other.v)
    }

    fn midpoint(self, other: Self) -> Self {
        Self::new(0.5 * (self.u + other.u), 0.5 * (self.v + other.v))
    }
}

/// Projects a point onto a plane's two in-plane axes.
fn in_plane(p: Vec3, plane: ArcPlane) -> Planar {
    let [u, v, _] = plane.axes();
    let a = p.to_array();
    Planar::new(a[u], a[v])
}

/// The component of a point along the plane's normal axis.
fn out_of_plane(p: Vec3, plane: ArcPlane) -> f64 {
    let [_, _, w] = plane.axes();
    p.to_array()[w]
}

/// Builds a point from in-plane and out-of-plane components.
fn from_plane(u_value: f64, v_value: f64, w_value: f64, plane: ArcPlane) -> Vec3 {
    let [u, v, w] = plane.axes();
    let mut out = [0.0f64; 3];
    out[u] = u_value;
    out[v] = v_value;
    out[w] = w_value;
    Vec3::from_array(out)
}

/// Everything the resolver knows about an arc before it is worked out.
#[derive(Debug, Clone, Copy)]
pub struct ArcRequest {
    /// Start, in machine coordinates.
    pub start: Vec3,
    /// End, in machine coordinates.
    pub end: Vec3,
    /// Which plane.
    pub plane: ArcPlane,
    /// Which way.
    pub turn: Turn,
    /// Centre offsets or absolute centre, already converted to millimetres and
    /// already made absolute in machine coordinates by the caller. `None` where
    /// the word was absent.
    pub centre: Option<Vec3>,
    /// `R` in millimetres, signed. `None` when the centre form was used.
    pub radius_word: Option<f64>,
    /// Extra whole turns from a `P` word, for multi-turn helices.
    pub extra_turns: u32,
    /// Mismatch tolerance in millimetres.
    pub tolerance: f64,
    /// Where the block is.
    pub site: Site,
}

/// Resolves an arc into its canonical form.
///
/// # Errors
///
/// See [`GcodeError::ArcRadiusMismatch`], [`GcodeError::ArcIllConditioned`],
/// [`GcodeError::FullCircleWithRadiusWord`] and
/// [`GcodeError::ArcRadiusTooSmall`].
pub fn resolve(request: &ArcRequest, diagnostics: &mut Diagnostics) -> Result<ArcData, GcodeError> {
    let start = in_plane(request.start, request.plane);
    let end = in_plane(request.end, request.plane);

    let (centre, form, residual) = match (request.centre, request.radius_word) {
        (Some(given), _) => {
            let (centre, residual) = reconcile(
                start,
                end,
                in_plane(given, request.plane),
                request.tolerance,
                request.site,
            )?;
            if residual != 0.0 {
                diagnostics.warn(GcodeWarning::ArcRecentred {
                    site: request.site,
                    residual,
                });
            }
            (centre, ArcForm::CentreOffsets, residual)
        }
        (None, Some(radius)) => {
            let centre = from_radius(start, end, radius, request.turn, request.site)?;
            // The R form defines the centre exactly from the radius, so there is
            // no mismatch to record: any inconsistency was already resolved by
            // choosing where to put the centre.
            (centre, ArcForm::Radius, 0.0)
        }
        (None, None) => {
            // The caller only builds an ArcRequest for a G2/G3 block, and such a
            // block must carry one form or the other.
            return Err(GcodeError::ArcRadiusTooSmall {
                site: request.site,
                half_chord: 0.0,
                radius: 0.0,
            });
        }
    };

    let radius = start.distance_to(centre);
    let sweep = sweep_of(start, end, centre, request.turn)
        + request.turn.sign() * f64::from(request.extra_turns) * core::f64::consts::TAU;

    let centre = from_plane(
        centre.u,
        centre.v,
        out_of_plane(request.start, request.plane),
        request.plane,
    );

    Ok(ArcData {
        center: centre,
        plane: request.plane,
        sweep,
        radius,
        form,
        radius_residual: residual,
    })
}

/// Moves a given centre so that it is equidistant from both endpoints.
///
/// Returns the adjusted centre and the mismatch that was split.
fn reconcile(
    start: Planar,
    end: Planar,
    centre: Planar,
    tolerance: f64,
    site: Site,
) -> Result<(Planar, f64), GcodeError> {
    let from_start = start.distance_to(centre);
    let from_end = end.distance_to(centre);
    let residual = from_start - from_end;

    if residual.abs() > tolerance {
        return Err(GcodeError::ArcRadiusMismatch {
            site,
            start_radius: from_start,
            end_radius: from_end,
            tolerance,
        });
    }
    if residual == 0.0 {
        return Ok((centre, 0.0));
    }

    // Move the centre along the perpendicular bisector of the chord, which is
    // the locus of points equidistant from both endpoints. Projecting the given
    // centre onto it is the smallest correction that makes the arc consistent.
    let mid = start.midpoint(end);
    let chord_u = end.u - start.u;
    let chord_v = end.v - start.v;
    let chord_len = t::hypot(chord_u, chord_v);
    if chord_len == 0.0 {
        // A full circle: both endpoints coincide, so any centre is equidistant
        // and there is nothing to reconcile.
        return Ok((centre, 0.0));
    }
    let dir_u = -chord_v / chord_len;
    let dir_v = chord_u / chord_len;
    let along = (centre.u - mid.u) * dir_u + (centre.v - mid.v) * dir_v;
    Ok((
        Planar::new(mid.u + dir_u * along, mid.v + dir_v * along),
        residual,
    ))
}

/// Derives a centre from a signed `R` word.
fn from_radius(
    start: Planar,
    end: Planar,
    radius: f64,
    turn: Turn,
    site: Site,
) -> Result<Planar, GcodeError> {
    let chord_u = end.u - start.u;
    let chord_v = end.v - start.v;
    let chord = t::hypot(chord_u, chord_v);

    if chord == 0.0 {
        // With I/J/K this is a full circle. With R it names no circle at all:
        // every circle of that radius through the point qualifies.
        return Err(GcodeError::FullCircleWithRadiusWord { site });
    }

    let half_chord = 0.5 * chord;
    let magnitude = radius.abs();
    if half_chord > magnitude {
        return Err(GcodeError::ArcRadiusTooSmall {
            site,
            half_chord,
            radius: magnitude,
        });
    }
    // Near a half-turn the centre is not determined by the endpoints; see the
    // module header. Refuse rather than produce a plausible wrong answer.
    if half_chord > magnitude * (1.0 - RADIUS_FORM_MARGIN) {
        return Err(GcodeError::ArcIllConditioned {
            site,
            half_chord,
            radius: magnitude,
        });
    }

    let height = (magnitude * magnitude - half_chord * half_chord).sqrt();
    let mid = start.midpoint(end);
    // Perpendicular to the chord, rotated +90 degrees about the plane normal.
    let dir_u = -chord_v / chord;
    let dir_v = chord_u / chord;

    // Of the two candidate centres, which one gives the arc the sign of R asks
    // for? Positive R is the minor arc (sweep <= 180), negative R the major one.
    // For a counter-clockwise turn the minor arc has its centre on the left of
    // the chord; clockwise reverses it, and a negative R reverses it again.
    let side = turn.sign() * if radius < 0.0 { -1.0 } else { 1.0 };
    Ok(Planar::new(
        mid.u + dir_u * height * side,
        mid.v + dir_v * height * side,
    ))
}

/// Signed sweep from start to end about the centre, in the direction `turn`.
fn sweep_of(start: Planar, end: Planar, centre: Planar, turn: Turn) -> f64 {
    let a0 = t::atan2(start.v - centre.v, start.u - centre.u);
    let a1 = t::atan2(end.v - centre.v, end.u - centre.u);
    let mut delta = (a1 - a0) * turn.sign();
    // atan2 gives (-PI, PI], so the raw difference is in (-TAU, TAU). Exactly
    // one representative lies in (0, TAU] — and zero means a full circle rather
    // than no motion, because a G2/G3 block whose endpoints coincide is a
    // complete turn.
    while delta <= 0.0 {
        delta += core::f64::consts::TAU;
    }
    delta * turn.sign()
}
