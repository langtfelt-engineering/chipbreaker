// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Bit-reproducible transcendental functions.
//!
//! # Why this module exists
//!
//! `std`'s `f64::sin` and friends lower to the platform's libm. MSVC's, glibc's
//! and wasi-libc's implementations are different code, and they genuinely differ
//! in the last bit on some inputs. None of them is wrong — IEEE-754 does not
//! require transcendentals to be correctly rounded, so each is free to be
//! off by an ULP wherever it likes.
//!
//! That freedom is fatal to us. The moment a tessellated tool profile computed
//! with `sin` reaches a golden hash, the native and WASM builds disagree and the
//! determinism guarantee is gone.
//!
//! **We do not need correctly rounded trig. We need reproducible trig**, which
//! is a far lower bar: one implementation, in pure Rust, compiled from the same
//! source for every target. That is the [`libm`] crate.
//!
//! # Rule
//!
//! **Never call a `std` transcendental.** `clippy.toml` denies them by path, so
//! this is enforced rather than merely requested. `f64::sqrt` remains allowed:
//! IEEE-754 *does* require it to be correctly rounded, so every target already
//! agrees on it.
//!
//! # Is `libm` actually bit-identical across targets?
//!
//! Yes, and the reasoning is worth recording because "it's pure Rust" is not by
//! itself sufficient.
//!
//! `libm` has two features that could in principle introduce target-dependent
//! behaviour, and neither does:
//!
//! - **`unstable-intrinsics`** (off by default, and requires nightly) would route
//!   `fma` through a compiler intrinsic. We do not enable it, so `fma` is the
//!   software implementation everywhere.
//! - **`arch`** (on by default) substitutes hardware instructions for a specific
//!   short list: `sqrt`, `fabs`, `ceil`, `floor`, `trunc`, `rint`, and on
//!   AArch64 `fma`. Every one of those is an operation IEEE-754 specifies
//!   *exactly*, so the hardware instruction and the software fallback produce
//!   identical bits by definition. No transcendental is arch-substituted; `sin`,
//!   `cos`, `exp`, `pow` and the rest are the same Rust source on every target.
//!
//! This analysis is the argument. The proof is
//! [`crate::selftest`]'s `transcendental` suite, which evaluates several thousand
//! seeded inputs and folds the results into the canonical hash that CI compares
//! between native and `wasm32-wasip1`. If the reasoning above is ever wrong, that
//! job fails.
//!
//! # If it does fail
//!
//! The escape hatch is `libm`'s `force-soft-floats` feature, which disables the
//! `arch` substitutions as well as intrinsics. It costs performance and should
//! not be needed; it is documented here so the next person does not have to
//! rediscover it under time pressure.
//!
//! # Accuracy
//!
//! `libm` is a port of MUSL's libm, typically within 1 ULP. That is comparable to
//! the platform libms it replaces, and far tighter than any tolerance in
//! [`crate::eps`]. Reproducibility is what we bought; accuracy was not sacrificed
//! to get it.

/// Sine of `x` radians.
#[inline]
#[must_use]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// Cosine of `x` radians.
#[inline]
#[must_use]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// Sine and cosine together, which is cheaper than computing both separately and
/// is what every tessellation loop actually wants.
#[inline]
#[must_use]
pub fn sin_cos(x: f64) -> (f64, f64) {
    libm::sincos(x)
}

/// Tangent of `x` radians.
#[inline]
#[must_use]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// Arcsine, in radians. NaN outside `[-1, 1]`.
#[inline]
#[must_use]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// Arccosine, in radians. NaN outside `[-1, 1]`.
#[inline]
#[must_use]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

/// Arctangent, in radians.
#[inline]
#[must_use]
pub fn atan(x: f64) -> f64 {
    libm::atan(x)
}

/// Four-quadrant arctangent of `y / x`, in radians.
#[inline]
#[must_use]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// `e^x`.
#[inline]
#[must_use]
pub fn exp(x: f64) -> f64 {
    libm::exp(x)
}

/// Natural logarithm.
#[inline]
#[must_use]
pub fn ln(x: f64) -> f64 {
    libm::log(x)
}

/// Base-10 logarithm.
#[inline]
#[must_use]
pub fn log10(x: f64) -> f64 {
    libm::log10(x)
}

/// `x^y` for real `y`.
#[inline]
#[must_use]
pub fn powf(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

/// `sqrt(x^2 + y^2)`, without intermediate overflow.
#[inline]
#[must_use]
pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

/// Real cube root, defined for negative `x`.
#[inline]
#[must_use]
pub fn cbrt(x: f64) -> f64 {
    libm::cbrt(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    /// Difference in ULPs, for finite same-signed values.
    fn ulp_diff(a: f64, b: f64) -> i64 {
        if a == b {
            return 0;
        }
        let ai = a.to_bits() as i64;
        let bi = b.to_bits() as i64;
        (ai - bi).abs()
    }

    #[test]
    fn exact_values_at_the_obvious_points() {
        assert_eq!(sin(0.0), 0.0);
        assert_eq!(cos(0.0), 1.0);
        assert_eq!(tan(0.0), 0.0);
        assert_eq!(atan(0.0), 0.0);
        assert_eq!(exp(0.0), 1.0);
        assert_eq!(ln(1.0), 0.0);
        assert_eq!(log10(1.0), 0.0);
        assert_eq!(log10(1000.0), 3.0);
        assert_eq!(powf(2.0, 10.0), 1024.0);
        assert_eq!(powf(9.0, 0.5), 3.0);
        assert_eq!(hypot(3.0, 4.0), 5.0);
        assert_eq!(cbrt(-27.0), -3.0);
        assert_eq!(asin(0.0), 0.0);
        assert_eq!(acos(1.0), 0.0);
        assert_eq!(atan2(0.0, 1.0), 0.0);
    }

    #[test]
    fn identities_hold_to_within_a_few_ulp() {
        // Not a re-test of libm's correctness; a smoke test that the wrappers
        // are wired to the functions their names claim.
        for i in -100..=100 {
            let x = f64::from(i) * 0.05;
            let (s, c) = sin_cos(x);
            assert_eq!(s, sin(x), "sin_cos disagrees with sin at {x}");
            assert_eq!(c, cos(x), "sin_cos disagrees with cos at {x}");
            assert!((s * s + c * c - 1.0).abs() < 1e-15, "sin^2 + cos^2 at {x}");
            if c.abs() > 1e-3 {
                assert!((tan(x) - s / c).abs() < 1e-12, "tan at {x}");
            }
            assert!((ln(exp(x)) - x).abs() < 1e-13, "ln(exp) at {x}");
            let cube = cbrt(x);
            assert!((cube * cube * cube - x).abs() < 1e-13, "cbrt at {x}");
        }
        assert!((sin(FRAC_PI_2) - 1.0).abs() < 1e-15);
        assert!(cos(FRAC_PI_2).abs() < 1e-15);
        assert!((tan(FRAC_PI_4) - 1.0).abs() < 1e-15);
        assert!((atan2(1.0, 1.0) - FRAC_PI_4).abs() < 1e-15);
        assert!((atan2(0.0, -1.0) - PI).abs() < 1e-15);
        assert!((asin(1.0) - FRAC_PI_2).abs() < 1e-15);
        assert!((acos(0.0) - FRAC_PI_2).abs() < 1e-15);
    }

    #[test]
    fn agrees_closely_with_the_platform_libm() {
        // Documents the accuracy claim: swapping std for libm changes results by
        // at most a couple of ULP, which is far below every tolerance in `eps`.
        // It is emphatically *not* a determinism check — std is the thing we are
        // replacing precisely because it varies by platform.
        let mut worst = 0i64;
        for i in -500..=500 {
            let x = f64::from(i) * 0.017;
            #[expect(
                clippy::disallowed_methods,
                reason = "comparing against the std implementation is the point of this test"
            )]
            let reference = [
                (x.sin(), sin(x)),
                (x.cos(), cos(x)),
                (x.exp(), exp(x)),
                (x.atan(), atan(x)),
            ];
            for (std_value, ours) in reference {
                assert_eq!(std_value.is_nan(), ours.is_nan());
                if std_value.is_finite() && ours.is_finite() {
                    worst = worst.max(ulp_diff(std_value, ours));
                }
            }
        }
        assert!(
            worst <= 4,
            "libm and the platform libm differ by {worst} ULP"
        );
    }

    #[test]
    fn edge_cases_do_not_panic_and_propagate_nan() {
        assert!(asin(2.0).is_nan());
        assert!(acos(-2.0).is_nan());
        assert!(ln(-1.0).is_nan());
        assert!(sin(f64::NAN).is_nan());
        assert_eq!(ln(0.0), f64::NEG_INFINITY);
        assert_eq!(exp(f64::NEG_INFINITY), 0.0);
        assert_eq!(exp(1000.0), f64::INFINITY);
        assert_eq!(hypot(f64::INFINITY, f64::NAN), f64::INFINITY);
        assert_eq!(powf(1.0, f64::NAN), 1.0, "1^anything is 1 per IEEE-754");
    }
}
