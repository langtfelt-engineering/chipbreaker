// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Unit normals in four bytes, octahedron-mapped.
//!
//! # Why store a normal at all
//!
//! Dual contouring places a vertex by minimising a quadratic error function over
//! the *planes* through its edge crossings. A crossing without a normal has no
//! plane, only a point, and the minimiser degenerates to the centroid — which is
//! plain surface nets: manifold, smooth, and with every sharp edge rounded off.
//! A machined part is mostly flats and fillets meeting at edges, so rounding
//! them off is not a cosmetic loss.
//!
//! The normal is free at both sites where an endpoint is born: the triangle
//! normal during field construction, and the analytic tool surface normal during
//! a cut. So this is a storage decision, not a computation one.
//!
//! # The encoding
//!
//! Project the unit sphere onto the octahedron `|x| + |y| + |z| = 1` along the
//! ray from the origin, then unfold that octahedron into the square
//! `[-1, 1]^2`: the upper hemisphere maps directly, and the lower hemisphere
//! folds outward into the four corner triangles. Quantise the square to two
//! `u16`s.
//!
//! Four bytes, and no worse than about 0.1° anywhere on the sphere — three
//! orders of magnitude below the angular error a dexel lattice already carries,
//! so it is not the limiting term in anything.
//!
//! Two properties matter more than the compactness:
//!
//! - **No transcendentals.** Encoding is division, addition and a round;
//!   decoding adds one `sqrt`. All are IEEE-754 exact operations, so the
//!   round-trip is bit-identical on every target by construction rather than by
//!   libm agreeing with itself. Spherical coordinates would have needed `atan2`
//!   and `asin` on the hot path and put four platforms' worth of trust in them.
//! - **No reserved bit patterns.** Every one of the 2^32 codes decodes to some
//!   unit vector, so there is no sentinel to accidentally collide with. See
//!   [`OctNormal::PLACEHOLDER`] for how "unknown" is handled instead.
//!
//! The map is onto but not one-to-one, and the exception is worth knowing: the
//! four corners `(+/-1, +/-1)` all unfold to the south pole `(0, 0, -1)`. So two
//! codes can name the same direction, and code equality is a stronger statement
//! than direction equality. Compare decoded vectors, never codes, when what you
//! mean is "the same way up".
//!
//! # The sign convention, which is the part that bites
//!
//! **A stored normal points out of the remaining material**, away from the solid
//! and into the air. Not out of the cutter, and not along the ray.
//!
//! This matters most where it is easiest to get backwards. When a cut removes
//! the middle of a span, the new endpoints lie on the *tool's* surface, and the
//! tool's own outward normal there points into the workpiece. The stored value
//! is its negation. Get that wrong and every cut face is inverted while every
//! cast face is correct, which produces a mesh that passes a manifold check, is
//! watertight, and is inside out in exactly the regions the customer cares
//! about. `spans::subtract` owns that negation and `sweep` tests it.

use crate::golden::{CanonicalHash, Hashable};
use crate::math::Vec3;

/// A unit normal quantised to four bytes.
///
/// See the module header for the encoding and, more importantly, for the sign
/// convention.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct OctNormal {
    /// First octahedral coordinate.
    pub u: u16,
    /// Second octahedral coordinate.
    pub v: u16,
}

/// Quantisation denominator, and the largest magnitude a code may take.
///
/// **Signed, not offset.** The obvious mapping — `(x + 1) / 2` scaled by
/// `u16::MAX` — is not symmetric about zero: it sends `0` to `32767.5`, which
/// rounds to `32768` and leaves the encoding with a half-step bias. Negation
/// then fails to be an involution, and `(0, 0, -1)` and `(0, 0, 1)` sit at
/// unequal distances from their shared axis.
///
/// Mapping `[-1, 1]` onto `[-32767, 32767]` and storing the two's-complement
/// bits instead makes `0` exact, makes `+/-1` exact, and makes the whole
/// encoding odd-symmetric — which is what lets [`OctNormal::negated`] be exact
/// integer arithmetic rather than a decode-negate-re-encode round trip.
const SCALE: f64 = 32767.0;

impl OctNormal {
    /// The value an endpoint carries when no normal was recorded.
    ///
    /// Decodes to `+Z`, and is `(0, 0)` — which falls out of the symmetric
    /// quantisation rather than being chosen, since `+Z` is the pole the
    /// octahedral map sends to the origin of the square.
    ///
    /// It is a *default*, not a sentinel: it is indistinguishable from a
    /// genuine `+Z`, because reserving a bit pattern would mean one real
    /// direction could not be represented, and a silently wrong normal is worse
    /// than an unavailable one.
    ///
    /// Where "unknown" has to be known — reading a version 2 `.tdx`, which
    /// predates normals, or `extract --no-normals` — the caller knows it from
    /// context and the extractor degrades to surface nets deliberately. Nothing
    /// infers it from the value.
    pub const PLACEHOLDER: Self = Self { u: 0, v: 0 };

    /// Encodes a direction. The input need not be normalised; zero yields
    /// [`Self::PLACEHOLDER`].
    #[must_use]
    pub fn encode(n: Vec3) -> Self {
        let l1 = n.x.abs() + n.y.abs() + n.z.abs();
        // `is_finite` first, so the NaN case is handled by a total predicate and
        // the comparison below only ever sees an ordered value.
        if !l1.is_finite() || l1 <= 0.0 {
            // Zero, or a NaN that arithmetic below would propagate into a
            // meaningless code. A degenerate triangle can produce either.
            return Self::PLACEHOLDER;
        }
        let (mut px, mut py) = (n.x / l1, n.y / l1);
        if n.z < 0.0 {
            // Fold the lower hemisphere out into the corner triangles.
            let (ax, ay) = (px.abs(), py.abs());
            let (sx, sy) = (sign_nonzero(px), sign_nonzero(py));
            px = (1.0 - ay) * sx;
            py = (1.0 - ax) * sy;
        }
        Self {
            u: quantise(px),
            v: quantise(py),
        }
    }

    /// Decodes back to a unit vector.
    #[must_use]
    pub fn decode(self) -> Vec3 {
        let px = dequantise(self.u);
        let py = dequantise(self.v);
        let pz = 1.0 - px.abs() - py.abs();
        let (x, y) = if pz < 0.0 {
            // Unfold the corner triangles back onto the lower hemisphere.
            (
                (1.0 - py.abs()) * sign_nonzero(px),
                (1.0 - px.abs()) * sign_nonzero(py),
            )
        } else {
            (px, py)
        };
        let v = Vec3::new(x, y, pz);
        let len = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        if len > 0.0 {
            Vec3::new(v.x / len, v.y / len, v.z / len)
        } else {
            // Unreachable for any code: the octahedron does not pass through the
            // origin. Kept total rather than panicking in the inner loop.
            Vec3::new(0.0, 0.0, 1.0)
        }
    }

    /// The opposite direction, exactly.
    ///
    /// Done in code space, in integers, because this runs on every cut face and
    /// because the identity is clean. Writing `p` for the square coordinate of a
    /// direction in the upper hemisphere, the antipode's coordinate is
    /// `-fold(p)` where `fold` is the same corner reflection the encoder uses:
    ///
    /// ```text
    /// fold(u, v) = ((1 - |v|) * sign(u),  (1 - |u|) * sign(v))
    /// ```
    ///
    /// Applying it twice returns the original *direction* exactly. It returns
    /// the original *code* too, except at the south pole, where the four corners
    /// of the square are aliases for one direction and the round trip may land
    /// on a different corner than it started from. Either way nothing drifts: a
    /// decode-negate-encode round trip would lose a quantisation step each time,
    /// and this loses none.
    #[must_use]
    pub fn negated(self) -> Self {
        let (u, v) = (i32::from(self.u as i16), i32::from(self.v as i16));
        let max = SCALE as i32;
        let fu = (max - v.abs()) * sign_i(u);
        let fv = (max - u.abs()) * sign_i(v);
        #[allow(
            clippy::cast_possible_truncation,
            reason = "both terms are bounded by SCALE, which fits i16"
        )]
        Self {
            u: (-fu) as i16 as u16,
            v: (-fv) as i16 as u16,
        }
    }
}

/// `+1` for a positive or zero input, `-1` otherwise, on integers.
///
/// Matches [`sign_nonzero`] so the integer fold in [`OctNormal::negated`] agrees
/// with the float fold in [`OctNormal::encode`] on the seams.
#[inline]
const fn sign_i(x: i32) -> i32 {
    if x >= 0 { 1 } else { -1 }
}

/// `+1` for a positive or zero input, `-1` otherwise.
///
/// Zero must map to `+1` rather than to `0`, or the fold collapses a whole edge
/// of the octahedron onto the origin and `(0, 0, -1)` becomes indistinguishable
/// from `(0, 0, +1)`.
#[inline]
fn sign_nonzero(x: f64) -> f64 {
    if x >= 0.0 { 1.0 } else { -1.0 }
}

/// `[-1, 1]` to `[-32767, 32767]`, round to nearest, stored as two's complement.
#[inline]
#[allow(
    clippy::cast_possible_truncation,
    reason = "clamped into i16 range immediately before the cast"
)]
fn quantise(x: f64) -> u16 {
    let q = (x.clamp(-1.0, 1.0) * SCALE).round();
    (q as i16) as u16
}

/// Two's complement back to `[-1, 1]`.
#[inline]
fn dequantise(q: u16) -> f64 {
    f64::from(q as i16) / SCALE
}

impl Hashable for OctNormal {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        // The code, not the decoded vector: the code is what is stored, and
        // hashing the decode would hide a quantisation change behind a rounding.
        h.begin("OctNormal")
            .u64(u64::from(self.u))
            .u64(u64::from(self.v))
            .end();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn angle_between(a: Vec3, b: Vec3) -> f64 {
        let dot = (a.x * b.x + a.y * b.y + a.z * b.z).clamp(-1.0, 1.0);
        crate::transcendental::acos(dot)
    }

    /// A deterministic spread over the sphere, without any random source.
    fn directions() -> Vec<Vec3> {
        let mut out = Vec::new();
        // The axes and the diagonals, which are where the fold's seams lie.
        for v in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
        ] {
            out.push(v);
        }
        let s = 1.0 / 3.0f64.sqrt();
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    out.push(Vec3::new(sx * s, sy * s, sz * s));
                }
            }
        }
        // A lattice over the sphere, skipping the degenerate zero vector.
        for i in -8..=8 {
            for j in -8..=8 {
                for k in -8..=8 {
                    let v = Vec3::new(f64::from(i), f64::from(j), f64::from(k));
                    let len = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
                    if len > 0.0 {
                        out.push(Vec3::new(v.x / len, v.y / len, v.z / len));
                    }
                }
            }
        }
        out
    }

    #[test]
    fn the_round_trip_is_accurate_everywhere_on_the_sphere() {
        let mut worst = 0.0f64;
        let mut worst_at = Vec3::new(0.0, 0.0, 1.0);
        for n in directions() {
            let back = OctNormal::encode(n).decode();
            let a = angle_between(n, back);
            if a > worst {
                worst = a;
                worst_at = n;
            }
        }
        let degrees = worst.to_degrees();
        assert!(
            degrees < 0.2,
            "worst round-trip error {degrees:.4} deg at {worst_at:?}; the encoding \
             claims about 0.1 deg and the QEF is built on these normals"
        );
    }

    #[test]
    fn the_lower_hemisphere_is_not_confused_with_the_upper() {
        // The fold's failure mode: `sign(0) = 0` collapses an octahedron edge
        // and makes -Z decode as +Z. That would flip every downward-facing
        // surface in the field.
        for (a, b) in [
            (Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0)),
            (Vec3::new(0.0, 0.6, 0.8), Vec3::new(0.0, 0.6, -0.8)),
            (Vec3::new(0.6, 0.0, 0.8), Vec3::new(0.6, 0.0, -0.8)),
        ] {
            let (ea, eb) = (OctNormal::encode(a), OctNormal::encode(b));
            assert_ne!(ea, eb, "{a:?} and {b:?} encode identically");
            assert!(
                ea.decode().z > 0.0 && eb.decode().z < 0.0,
                "hemisphere lost: {a:?} -> {:?}, {b:?} -> {:?}",
                ea.decode(),
                eb.decode()
            );
        }
    }

    #[test]
    fn decoding_always_yields_a_unit_vector() {
        // Every one of the 2^32 codes must decode to something usable, because
        // there is no reserved pattern to reject. Swept coarsely over the grid.
        let mut worst = 0.0f64;
        for u in (0..=u16::MAX).step_by(257) {
            for v in (0..=u16::MAX).step_by(257) {
                let n = OctNormal { u, v }.decode();
                let len = (n.x * n.x + n.y * n.y + n.z * n.z).sqrt();
                worst = worst.max((len - 1.0).abs());
            }
        }
        assert!(
            worst < 1.0e-12,
            "worst deviation from unit length {worst:.3e}"
        );
    }

    #[test]
    fn negation_is_an_involution_and_really_reverses() {
        for n in directions() {
            let e = OctNormal::encode(n);
            let flipped = e.negated();
            let d = flipped.decode();
            let original = e.decode();
            let dot = d.x * original.x + d.y * original.y + d.z * original.z;
            assert!(
                dot < -0.999,
                "negation of {n:?} gave {d:?}, dot {dot:.6} with {original:?}"
            );

            // Twice back to where it started -- so a cut face negated on write
            // and again by a reader does not drift.
            //
            // Compared as a **direction**, not as a code. At the south pole all
            // four corners of the square are aliases for `(0, 0, -1)`, so the
            // round trip can legitimately return a different corner than it
            // started from. Asserting code equality here failed on exactly that
            // case and would have been asserting something untrue about the
            // encoding rather than something true about the code.
            let back = flipped.negated().decode();
            let same = back.x * original.x + back.y * original.y + back.z * original.z;
            assert!(
                same > 0.999_999,
                "negating twice moved {n:?}: {original:?} -> {back:?}"
            );
        }
    }

    #[test]
    fn the_placeholder_decodes_to_plus_z() {
        let n = OctNormal::PLACEHOLDER.decode();
        assert!(
            n.z > 0.999,
            "the placeholder must decode to +Z, got {n:?}; a surprising default \
             would show up as a tilted surface rather than as a missing one"
        );
    }

    #[test]
    fn encoding_is_stable_under_the_scale_of_its_input() {
        // The field feeds unnormalised triangle normals in, so a large triangle
        // and a small one of the same orientation must not encode differently in
        // any way that matters.
        //
        // Tolerance of one step per component rather than bit equality:
        // `n / l1` genuinely rounds differently when the inputs are scaled, and
        // demanding an identical code would be demanding that division be exact.
        // One step is 0.003 deg, which is nothing. What must NOT happen is a
        // scale-dependent jump.
        for n in directions() {
            let base = OctNormal::encode(n);
            for k in [1.0e-6, 0.5, 1.0, 3.0, 1.0e6] {
                let scaled = OctNormal::encode(Vec3::new(n.x * k, n.y * k, n.z * k));
                let du = i32::from(scaled.u as i16) - i32::from(base.u as i16);
                let dv = i32::from(scaled.v as i16) - i32::from(base.v as i16);
                assert!(
                    du.abs() <= 1 && dv.abs() <= 1,
                    "scaling {n:?} by {k} moved the code by ({du}, {dv}) steps"
                );
            }
        }
    }

    #[test]
    fn degenerate_input_falls_back_rather_than_producing_nonsense() {
        for bad in [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(f64::NAN, 0.0, 1.0),
            Vec3::new(f64::INFINITY, 0.0, 0.0),
        ] {
            assert_eq!(
                OctNormal::encode(bad),
                OctNormal::PLACEHOLDER,
                "{bad:?} should fall back to the placeholder"
            );
        }
    }
}
