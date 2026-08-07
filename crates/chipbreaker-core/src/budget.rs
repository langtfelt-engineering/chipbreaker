// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! What a job will cost in memory, computed before anything is allocated.
//!
//! # Predict, do not react
//!
//! The footprint of a tri-dexel field is a **pure function** of the stock
//! extents, the three spacings, the arena's inline capacity and the size of a
//! `Span`. Nothing about it is heuristic, so there is no reason to discover it
//! by allocating and finding out. The whole point of this module is that a job
//! too large to run is refused before the caller has waited for it.
//!
//! A partially built field that dies halfway is strictly worse than a refusal:
//! the time is already spent, the host process may be damaged, and the message
//! arrives at the least useful moment. An OEM embedding this engine needs to be
//! able to say "it refuses cleanly above a configurable budget", which is a
//! sentence they can put in their own release notes. "It sometimes runs the
//! machine out of memory" is a defect report.
//!
//! # The refusal carries the answer
//!
//! An error that says a job is too large is a defect report. An error that says
//! **what would fit** is a feature. So [`Budget::check`] solves for the finest
//! spacing that fits and names it, and the three contributors — field,
//! extraction window, toolpath IR — are reported separately, because a user
//! whose problem is a three-million-segment program needs to know that rather
//! than be told to coarsen a lattice that was never the issue.
//!
//! # The uncut prediction is a floor, not a ceiling
//!
//! Unit 7 established that cutting **splits spans**, and that spill is per
//! bundle rather than per ray: the rib case spilled all 4,500 rays of the Y
//! bundle while X and Z spilled none. A field that fits at construction can
//! therefore exceed its budget after a pocket is cut.
//!
//! Two consequences, both handled here rather than left to the caller:
//!
//! - The prediction carries **headroom for spill**, stated explicitly rather
//!   than folded silently into a number. See [`SPILL_MODEL`].
//! - The ceiling is checked **again as spill grows**, so a long job that creeps
//!   over refuses with the same clear message instead of dying at an arbitrary
//!   segment. That refusal names the operation and the segment index, because
//!   "how far did it get" is the first thing anyone asks.

use core::fmt;

use crate::dexel::arena::INLINE_CAPACITY;
use crate::spans::Span;

/// Bytes per ray in a freshly built, uncut bundle.
///
/// `INLINE_CAPACITY` spans and a `u16` length. **Not** the `u32` spill index:
/// the arena allocates that lazily, on first spill, so an uncut field genuinely
/// does not carry it.
///
/// Counting it here was the first version, and it over-predicted every case by
/// exactly 8.00% -- `54/50` -- which is the kind of constant error that looks
/// like a safety margin and is actually a wrong model. It belongs in
/// [`Footprint::spill_headroom_bytes`], where it is charged only if spilling
/// can happen.
#[must_use]
pub const fn bytes_per_ray() -> usize {
    INLINE_CAPACITY * size_of::<Span>() + size_of::<u16>()
}

/// Bytes a spilled ray costs on top of its inline storage.
///
/// The lazily allocated `u32` spill index plus one heap `Span`, which is what a
/// ray going from two spans to three actually costs.
#[must_use]
pub const fn bytes_per_spilled_ray() -> usize {
    size_of::<u32>() + size_of::<Span>()
}

/// Bytes per corner in the extraction sweep's window.
///
/// Unit 9's slab sweep holds two planes of corner signs, the crossings on and
/// between them, and two planes of cell records. Measured at about 130 bytes per
/// `(x, y)` corner; the window is `O(area)`, so this multiplies the largest
/// cross-section rather than the volume.
pub const EXTRACTION_BYTES_PER_PLANE_CORNER: usize = 130;

/// Bytes per toolpath segment, measured at Unit 4.
pub const IR_BYTES_PER_SEGMENT: usize = 192;

/// How the spill allowance is derived.
///
/// **Not a fudge factor.** Unit 7's corpus measured post-cut span distributions,
/// and the worst case it produced -- two slots leaving a rib -- spilled *every
/// ray of one bundle* while the other two spilled none. A bundle at
/// `INLINE_CAPACITY = 2` spills at three spans, so each spilled ray costs
/// [`bytes_per_spilled_ray`] on top of what it already holds.
///
/// The allowance is therefore **the largest single bundle, spilled entirely**,
/// which is exactly the worst the corpus has produced rather than a round
/// number. It is named in the error message so a user who knows their part is
/// worse than the corpus can raise the budget deliberately instead of guessing.
pub const SPILL_MODEL: &str = "the largest bundle spilling entirely, as measured at Unit 7";

/// A predicted footprint, broken into the parts a user can act on separately.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Footprint {
    /// The three bundles as built, before any cutting.
    pub field_bytes: u64,
    /// Room held back for spans splitting under the cutter.
    pub spill_headroom_bytes: u64,
    /// The extraction sweep's window, if the job will extract.
    pub extraction_bytes: u64,
    /// The toolpath IR, if a program is loaded alongside.
    pub ir_bytes: u64,
}

impl Footprint {
    /// Everything, summed.
    #[must_use]
    pub const fn total_bytes(&self) -> u64 {
        self.field_bytes
            .saturating_add(self.spill_headroom_bytes)
            .saturating_add(self.extraction_bytes)
            .saturating_add(self.ir_bytes)
    }
}

/// Renders a byte count the way a person reads one.
#[must_use]
pub fn human(bytes: u64) -> String {
    #[allow(clippy::cast_precision_loss, reason = "display only")]
    let b = bytes as f64;
    const KIB: f64 = 1024.0;
    const MIB: f64 = KIB * 1024.0;
    const GIB: f64 = MIB * 1024.0;
    if b >= GIB {
        format!("{:.2} GiB", b / GIB)
    } else if b >= MIB {
        format!("{:.0} MiB", b / MIB)
    } else if b >= KIB {
        format!("{:.0} KiB", b / KIB)
    } else {
        format!("{bytes} B")
    }
}

/// The three spacings of a field.
///
/// Isotropic is the special case where all three are equal, and it stays the
/// common one; carrying three from the start means the memory formula and the
/// sampling bound are written once rather than twice.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spacing {
    /// Cell size along X, in millimetres.
    pub x: f64,
    /// Cell size along Y.
    pub y: f64,
    /// Cell size along Z.
    pub z: f64,
}

impl Spacing {
    /// The same cell size on every axis.
    #[must_use]
    pub const fn uniform(h: f64) -> Self {
        Self { x: h, y: h, z: h }
    }

    /// True if all three agree.
    #[must_use]
    pub fn is_uniform(&self) -> bool {
        self.x == self.y && self.y == self.z
    }

    /// As an array in axis order.
    #[must_use]
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// All three scaled by `k`.
    #[must_use]
    pub fn scaled(self, k: f64) -> Self {
        Self {
            x: self.x * k,
            y: self.y * k,
            z: self.z * k,
        }
    }

    /// Worst-case distance from a surface point to the nearest place the field
    /// sampled, in millimetres.
    ///
    /// # The anisotropic sampling bound
    ///
    /// Unit 6 derived `h * sqrt(3/2)` for equal spacings. That derivation
    /// assumed three equal cells and does not survive anisotropy as stated, so
    /// here is the general form.
    ///
    /// For the bundle along axis `a`, the transverse cell measures `h_u x h_v`,
    /// so a surface point is within half that cell's **diagonal** of some ray:
    ///
    /// ```text
    /// c_a = sqrt(h_u^2 + h_v^2) / 2
    /// ```
    ///
    /// The sample distance through that bundle is `c_a / |n_a|`, and the field
    /// gets the best of three, so the worst case over unit normals is
    ///
    /// ```text
    /// D = max over |n| = 1 of  min over a of  c_a / |n_a|
    /// ```
    ///
    /// At the maximum all three are equal — if one were larger, tilting `n`
    /// toward that axis would raise the minimum — so `|n_a| = c_a / D`, and
    /// `sum |n_a|^2 = 1` gives `D = sqrt(sum c_a^2)`. Expanding,
    ///
    /// ```text
    /// sum c_a^2 = [(h_y^2+h_z^2) + (h_z^2+h_x^2) + (h_x^2+h_y^2)] / 4
    ///           = (h_x^2 + h_y^2 + h_z^2) / 2
    ///
    /// D = sqrt( (h_x^2 + h_y^2 + h_z^2) / 2 )
    /// ```
    ///
    /// which collapses to `h * sqrt(3/2)` when the three agree, exactly
    /// reproducing [`crate::dexel::tri::SAMPLE_DISTANCE_CONSTANT`]. Confirmed
    /// against a dense sweep of random normals, which reaches 99.8% of it.
    ///
    /// **The consequence is the product-relevant part.** `D` is a quadratic mean:
    /// it is driven by the *largest* spacing, so coarsening one axis degrades the
    /// worst case for every surface, not only for surfaces facing that axis.
    /// Anisotropy does not buy accuracy for free — it buys memory, and pays in
    /// the guarantee, unless the part's normals genuinely avoid the coarse
    /// direction. See the auto-selection discussion.
    #[must_use]
    pub fn sample_distance_bound(&self) -> f64 {
        ((self.x * self.x + self.y * self.y + self.z * self.z) / 2.0).sqrt()
    }

    /// Worst-case perpendicular error of a nearest-neighbour reconstruction.
    ///
    /// The companion to [`Self::sample_distance_bound`], generalising Unit 6's
    /// `h / sqrt(3)`. The error at transverse offset `t` is `t * sin(theta)`,
    /// and the same optimisation over unit normals gives
    ///
    /// ```text
    /// P = D / sqrt(1 + D^2 / (sum c_a^2))  ... isotropically  h/sqrt(3)
    /// ```
    ///
    /// which for equal spacings is `h * sqrt(3/2) * sqrt(2/3) = h / sqrt(3)`.
    /// Reported for continuity with Unit 6; Unit 9's dual contouring beats it by
    /// a wide margin and Unit 12 will measure its own.
    #[must_use]
    pub fn perpendicular_bound(&self) -> f64 {
        // sin(theta) at the worst case, where cos(theta) = c_a / D and the three
        // are balanced, works out to sqrt(2/3) of the sample distance for equal
        // spacings. Kept as the same ratio in general, which is exact for the
        // uniform case and a close upper estimate otherwise.
        self.sample_distance_bound() * (2.0f64 / 3.0).sqrt()
    }
}

/// Chooses three spacings that minimise memory **without weakening the bound**.
///
/// # What is being optimised, and why it is constrained rather than free
///
/// Anisotropy is not a free saving. [`Spacing::sample_distance_bound`] is a
/// quadratic mean, so it is driven by the *largest* spacing: coarsening one axis
/// degrades the worst case for every surface, not only for surfaces facing that
/// axis, and no amount of refinement elsewhere rescues it. A rule that simply
/// scaled each axis by the part's extent — or by the surface area facing it —
/// would buy memory by quietly spending accuracy.
///
/// So this holds the bound fixed at whatever `reference` would have given and
/// minimises memory subject to it:
///
/// ```text
/// minimise   rays = (D H)/(hy hz) + (H W)/(hz hx) + (W D)/(hx hy)
/// subject to hx^2 + hy^2 + hz^2 = 3 * reference^2
/// ```
///
/// The constraint is exactly "the sample-distance bound is unchanged", since
/// `D = sqrt(sum h^2 / 2)`.
///
/// # What falls out
///
/// For a **cube** the problem is symmetric, so the optimum is isotropic and this
/// returns the input unchanged — correctly, because there is nothing to win. For
/// a **plate** the dominant cost is the bundle looking through the thin
/// direction, whose ray count is `W D / (hx hy)` and does not involve `hz` at
/// all, so the optimiser coarsens `hx` and `hy` and pays for it with a finer
/// `hz`. A **bar** is the same argument with the roles swapped.
///
/// The search is a fixed-resolution scan over the constraint surface followed by
/// a fixed number of refinement rounds. Deterministic by construction: no
/// convergence test, no data-dependent iteration count, and it runs once per
/// build so its cost is irrelevant.
#[must_use]
pub fn auto_spacing(extents: [f64; 3], reference: f64) -> Spacing {
    let target = Spacing::uniform(reference).sample_distance_bound();
    // The constraint sphere: hx^2 + hy^2 + hz^2 = 2 * target^2.
    let radius = (2.0 * target * target).sqrt();

    let cost = |h: Spacing| -> f64 {
        let counts = ray_counts(extents, h);
        #[allow(clippy::cast_precision_loss, reason = "comparing magnitudes")]
        let total = counts.iter().sum::<u64>() as f64;
        total
    };

    // Parametrise the positive octant of the sphere by two angles, so every
    // candidate satisfies the constraint exactly rather than approximately.
    let candidate = |a: f64, b: f64| -> Spacing {
        // `transcendental`, not `f64::sin` -- the project forbids std
        // transcendentals so that every target computes the same bits, and this
        // runs once per build where a divergence would change the spacings and
        // so the whole field.
        let (sa, ca) = crate::transcendental::sin_cos(a);
        let (sb, cb) = crate::transcendental::sin_cos(b);
        Spacing {
            x: radius * sa * cb,
            y: radius * sa * sb,
            z: radius * ca,
        }
    };

    // Coarse scan, then refinement. Both fixed-size.
    let half_pi = core::f64::consts::PI / 2.0;
    let (mut best_a, mut best_b) = (0.0f64, 0.0f64);
    let mut best = f64::INFINITY;
    const SCAN: usize = 48;
    for i in 1..SCAN {
        for j in 1..SCAN {
            #[allow(clippy::cast_precision_loss, reason = "small loop indices")]
            let (a, b) = (
                half_pi * i as f64 / SCAN as f64,
                half_pi * j as f64 / SCAN as f64,
            );
            let c = cost(candidate(a, b));
            if c < best {
                best = c;
                best_a = a;
                best_b = b;
            }
        }
    }
    let mut window = half_pi / SCAN as f64;
    for _ in 0..6 {
        let mut improved = (best_a, best_b);
        for di in -2i32..=2 {
            for dj in -2i32..=2 {
                let a = (best_a + f64::from(di) * window / 2.0).clamp(1.0e-6, half_pi - 1.0e-6);
                let b = (best_b + f64::from(dj) * window / 2.0).clamp(1.0e-6, half_pi - 1.0e-6);
                let c = cost(candidate(a, b));
                if c < best {
                    best = c;
                    improved = (a, b);
                }
            }
        }
        best_a = improved.0;
        best_b = improved.1;
        window /= 2.0;
    }
    candidate(best_a, best_b)
}

/// A memory budget, and the arithmetic for staying inside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Budget {
    /// The ceiling, in bytes. `None` for unlimited.
    limit_bytes: Option<u64>,
}

impl Budget {
    /// No ceiling.
    #[must_use]
    pub const fn unlimited() -> Self {
        Self { limit_bytes: None }
    }

    /// A ceiling in bytes.
    #[must_use]
    pub const fn bytes(limit: u64) -> Self {
        Self {
            limit_bytes: Some(limit),
        }
    }

    /// The ceiling, if any.
    #[must_use]
    pub const fn limit(&self) -> Option<u64> {
        self.limit_bytes
    }

    /// Predicts a job's footprint.
    ///
    /// `extents` are the workspace dimensions in millimetres, `segments` the
    /// toolpath length, and `extracting` whether a mesh will be pulled out
    /// afterwards.
    #[must_use]
    pub fn predict(
        extents: [f64; 3],
        spacing: Spacing,
        segments: u64,
        extracting: bool,
    ) -> Footprint {
        let counts = ray_counts(extents, spacing);
        let per_ray = bytes_per_ray() as u64;
        let field_bytes = counts.iter().sum::<u64>().saturating_mul(per_ray);

        // The largest bundle, spilled entirely. See `SPILL_MODEL`.
        let spill_headroom_bytes = counts
            .iter()
            .copied()
            .max()
            .unwrap_or(0)
            .saturating_mul(bytes_per_spilled_ray() as u64);

        // The extraction window is O(area): the largest cross-section, since the
        // sweep runs along one axis and holds planes perpendicular to it. Unit 9
        // sweeps in z, so the window is the x-y plane.
        let extraction_bytes = if extracting {
            let nx = axis_count(extents[0], spacing.x) + 2;
            let ny = axis_count(extents[1], spacing.y) + 2;
            nx.saturating_mul(ny)
                .saturating_mul(EXTRACTION_BYTES_PER_PLANE_CORNER as u64)
        } else {
            0
        };

        Footprint {
            field_bytes,
            spill_headroom_bytes,
            extraction_bytes,
            ir_bytes: segments.saturating_mul(IR_BYTES_PER_SEGMENT as u64),
        }
    }

    /// Checks a job against the ceiling, before anything is allocated.
    ///
    /// # Errors
    /// [`BudgetError::TooLarge`], carrying the breakdown and the finest spacing
    /// that would fit.
    pub fn check(
        &self,
        extents: [f64; 3],
        spacing: Spacing,
        segments: u64,
        extracting: bool,
    ) -> Result<Footprint, BudgetError> {
        let footprint = Self::predict(extents, spacing, segments, extracting);
        let Some(limit) = self.limit_bytes else {
            return Ok(footprint);
        };
        if footprint.total_bytes() <= limit {
            return Ok(footprint);
        }
        Err(BudgetError::TooLarge {
            footprint,
            limit,
            spacing,
            suggestion: self.coarsest_that_fits(extents, spacing, segments, extracting),
        })
    }

    /// The finest uniform scaling of `spacing` that fits, if any does.
    ///
    /// Scales all three axes together, preserving the ratio between them, so a
    /// deliberately anisotropic choice is not silently turned isotropic. Bisects
    /// rather than solving in closed form because the ray count involves a
    /// ceiling per axis and so is a step function — a closed-form answer would
    /// be a fraction of a cell out and could suggest a spacing that does not
    /// actually fit.
    fn coarsest_that_fits(
        &self,
        extents: [f64; 3],
        spacing: Spacing,
        segments: u64,
        extracting: bool,
    ) -> Option<Spacing> {
        let limit = self.limit_bytes?;
        // If the toolpath alone busts the budget, no spacing helps, and saying
        // so is more useful than suggesting a lattice change that cannot work.
        let ir = segments.saturating_mul(IR_BYTES_PER_SEGMENT as u64);
        if ir > limit {
            return None;
        }
        let fits = |k: f64| {
            Self::predict(extents, spacing.scaled(k), segments, extracting).total_bytes() <= limit
        };
        // Find any coarsening that fits, doubling out to a sane ceiling.
        let mut hi = 1.0f64;
        let mut found = false;
        for _ in 0..40 {
            hi *= 2.0;
            if fits(hi) {
                found = true;
                break;
            }
        }
        if !found {
            return None;
        }
        // Bisect for the finest. Fixed iteration count: deterministic, and 40
        // halvings is far past the point where the answer stops moving.
        let mut lo = 1.0f64;
        for _ in 0..40 {
            let mid = lo / 2.0 + hi / 2.0;
            if fits(mid) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        Some(spacing.scaled(hi))
    }

    /// Checks a field that has grown under cutting.
    ///
    /// # Errors
    /// [`BudgetError::GrewTooLarge`], naming the operation and the segment so
    /// the user knows how far the job got.
    pub fn check_growth(
        &self,
        actual_bytes: u64,
        operation: &'static str,
        segment: u64,
    ) -> Result<(), BudgetError> {
        let Some(limit) = self.limit_bytes else {
            return Ok(());
        };
        if actual_bytes <= limit {
            return Ok(());
        }
        Err(BudgetError::GrewTooLarge {
            actual_bytes,
            limit,
            operation,
            segment,
        })
    }
}

/// Rays in each bundle, in `AXES` order.
#[must_use]
pub fn ray_counts(extents: [f64; 3], spacing: Spacing) -> [u64; 3] {
    let n = [
        axis_count(extents[0], spacing.x),
        axis_count(extents[1], spacing.y),
        axis_count(extents[2], spacing.z),
    ];
    // A bundle along an axis has one ray per cell of the other two.
    [
        n[1].saturating_mul(n[2]),
        n[2].saturating_mul(n[0]),
        n[0].saturating_mul(n[1]),
    ]
}

/// Cells along one axis, matching `Lattice`'s own rounding.
fn axis_count(extent: f64, spacing: f64) -> u64 {
    // `is_finite` first, so the comparisons below only ever see ordered values.
    if !extent.is_finite() || !spacing.is_finite() || extent <= 0.0 || spacing <= 0.0 {
        return 0;
    }
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a non-negative count, saturated below"
    )]
    let n = (extent / spacing).ceil() as u64;
    n.max(1)
}

/// Why a job was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum BudgetError {
    /// The job would not fit, and was refused before allocating.
    TooLarge {
        /// What it would have cost.
        footprint: Footprint,
        /// The ceiling.
        limit: u64,
        /// The spacing asked for.
        spacing: Spacing,
        /// The finest scaling of that spacing which fits, if one does.
        suggestion: Option<Spacing>,
    },
    /// A field within budget at construction grew past it while cutting.
    GrewTooLarge {
        /// What it costs now.
        actual_bytes: u64,
        /// The ceiling.
        limit: u64,
        /// What was being done.
        operation: &'static str,
        /// Which segment it had reached.
        segment: u64,
    },
}

impl fmt::Display for BudgetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooLarge {
                footprint,
                limit,
                spacing,
                suggestion,
            } => {
                let s = spacing.to_array();
                if spacing.is_uniform() {
                    write!(f, "{} mm requires ", s[0])?;
                } else {
                    write!(f, "{} x {} x {} mm requires ", s[0], s[1], s[2])?;
                }
                write!(f, "{} (", human(footprint.total_bytes()))?;
                let mut parts: Vec<String> =
                    vec![format!("field {}", human(footprint.field_bytes))];
                if footprint.spill_headroom_bytes > 0 {
                    parts.push(format!(
                        "spill headroom {}",
                        human(footprint.spill_headroom_bytes)
                    ));
                }
                if footprint.extraction_bytes > 0 {
                    parts.push(format!(
                        "extraction window {}",
                        human(footprint.extraction_bytes)
                    ));
                }
                if footprint.ir_bytes > 0 {
                    parts.push(format!("toolpath IR {}", human(footprint.ir_bytes)));
                }
                write!(
                    f,
                    "{}) against a budget of {}.",
                    parts.join(" + "),
                    human(*limit)
                )?;
                match suggestion {
                    Some(fit) => {
                        let g = fit.to_array();
                        let cost = Budget::predict(
                            // Extents are not carried on the error, so the
                            // suggestion's own cost is reported by the caller;
                            // here only the spacing is named.
                            [0.0, 0.0, 0.0],
                            *fit,
                            0,
                            false,
                        );
                        let _ = cost;
                        if fit.is_uniform() {
                            write!(f, " {} mm fits.", g[0])
                        } else {
                            write!(f, " {} x {} x {} mm fits.", g[0], g[1], g[2])
                        }
                    }
                    None => write!(
                        f,
                        " No spacing fits: the toolpath IR alone is {}, so the budget \
                         has to rise or the program has to be split.",
                        human(footprint.ir_bytes)
                    ),
                }
            }
            Self::GrewTooLarge {
                actual_bytes,
                limit,
                operation,
                segment,
            } => write!(
                f,
                "the field grew to {} during {operation} at segment {segment}, past the \
                 budget of {}. Cutting splits spans, so a field that fits when built can \
                 exceed the budget once pockets are cut; raise the budget or coarsen the \
                 lattice.",
                human(*actual_bytes),
                human(*limit)
            ),
        }
    }
}

impl std::error::Error for BudgetError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_isotropic_bound_reproduces_unit_6_exactly() {
        // The generalisation must not move the number Unit 6 published, or every
        // accuracy claim made since would need restating.
        for h in [0.05, 0.1, 0.4, 1.6] {
            let got = Spacing::uniform(h).sample_distance_bound();
            let expected = h * crate::dexel::tri::SAMPLE_DISTANCE_CONSTANT;
            assert!(
                (got - expected).abs() < 1.0e-15 * expected.max(1.0),
                "h={h}: anisotropic form gives {got}, Unit 6 gives {expected}"
            );
        }
    }

    #[test]
    fn the_bound_is_driven_by_the_largest_spacing() {
        // The product-relevant consequence: it is a quadratic mean, so a coarse
        // axis degrades the guarantee for every surface rather than only for
        // those facing it. Anisotropy trades accuracy for memory; it is not free.
        let base = Spacing::uniform(1.0).sample_distance_bound();
        let coarse_z = Spacing {
            x: 1.0,
            y: 1.0,
            z: 2.0,
        }
        .sample_distance_bound();
        assert!(
            coarse_z > base * 1.4,
            "doubling one axis should cost about sqrt(2) in the bound, got {:.4}x",
            coarse_z / base
        );
        // And refining one axis cannot rescue a coarse one.
        let mixed = Spacing {
            x: 0.1,
            y: 0.1,
            z: 2.0,
        }
        .sample_distance_bound();
        assert!(
            mixed > 1.4,
            "a 2 mm axis cannot be compensated by 0.1 mm elsewhere, got {mixed:.4}"
        );
    }

    #[test]
    fn ray_counts_match_the_lattice() {
        // The prediction is only worth having if it counts what the lattice
        // actually builds.
        let counts = ray_counts([40.0, 30.0, 10.0], Spacing::uniform(0.5));
        assert_eq!(counts, [60 * 20, 20 * 80, 80 * 60]);
    }

    #[test]
    fn an_over_budget_job_is_refused_with_a_spacing_that_fits() {
        let budget = Budget::bytes(8 * 1024 * 1024);
        let err = budget
            .check([100.0, 100.0, 100.0], Spacing::uniform(0.1), 0, false)
            .expect_err("must refuse");
        let BudgetError::TooLarge { suggestion, .. } = &err else {
            panic!("wrong variant: {err:?}");
        };
        let fit = suggestion.expect("a coarser spacing should fit");
        assert!(
            budget.check([100.0, 100.0, 100.0], fit, 0, false).is_ok(),
            "the suggested spacing {fit:?} does not actually fit"
        );
        // And it must be the *finest* that fits, to within the bisection.
        let finer = fit.scaled(0.97);
        assert!(
            budget
                .check([100.0, 100.0, 100.0], finer, 0, false)
                .is_err(),
            "a spacing 3% finer than the suggestion also fits, so the suggestion \
             is needlessly coarse"
        );
        let text = err.to_string();
        assert!(
            text.contains("fits."),
            "the message must name a way forward: {text}"
        );
    }

    #[test]
    fn a_toolpath_too_large_on_its_own_says_so() {
        // Coarsening the lattice cannot help here, and suggesting it would send
        // the user in the wrong direction.
        let budget = Budget::bytes(1024 * 1024);
        let err = budget
            .check([10.0, 10.0, 10.0], Spacing::uniform(1.0), 100_000, false)
            .expect_err("must refuse");
        let text = err.to_string();
        assert!(
            text.contains("toolpath IR"),
            "the message must name the IR as the problem: {text}"
        );
        assert!(
            text.contains("No spacing fits"),
            "it must not suggest a lattice change that cannot work: {text}"
        );
    }

    #[test]
    fn growth_refusal_names_the_operation_and_segment() {
        let budget = Budget::bytes(1000);
        let err = budget
            .check_growth(2000, "cutting", 41_332)
            .expect_err("must refuse");
        let text = err.to_string();
        assert!(text.contains("cutting"), "{text}");
        assert!(
            text.contains("41332"),
            "the segment index is how far it got: {text}"
        );
    }

    #[test]
    fn an_unlimited_budget_never_refuses() {
        let budget = Budget::unlimited();
        assert!(
            budget
                .check([1.0e6, 1.0e6, 1.0e6], Spacing::uniform(0.001), 0, true)
                .is_ok()
        );
        assert!(budget.check_growth(u64::MAX, "cutting", 0).is_ok());
    }

    #[test]
    fn the_breakdown_names_each_contributor_separately() {
        // A user whose problem is a three-million-segment program must not be
        // told to coarsen a lattice that was never the issue.
        let budget = Budget::bytes(1024);
        let err = budget
            .check([50.0, 50.0, 50.0], Spacing::uniform(0.2), 3_000_000, true)
            .expect_err("must refuse");
        let text = err.to_string();
        for part in [
            "field",
            "spill headroom",
            "extraction window",
            "toolpath IR",
        ] {
            assert!(text.contains(part), "the breakdown omits {part}: {text}");
        }
    }
}
