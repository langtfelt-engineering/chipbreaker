// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Finding collisions by replaying the program.
//!
//! # Why this cannot read a cut field
//!
//! A collision is a property of the **trajectory**, not of the end state, and
//! neither end of a run can stand in for the middle.
//!
//! At the moment motion *k* executes, the material present is the stock after
//! motions `0..k`. Testing against the **final** field tests every motion
//! against the least material the job ever contains, so every collision with
//! material that a later pass removes is missed — a systematic false negative,
//! and the one this unit exists to make impossible. Testing against the
//! **initial** stock invents collisions instead: a holder correctly following
//! its cutter down into a pocket the cutter just opened is reported as a crash.
//!
//! So detection is interleaved. Check, then cut, then step. The field is
//! consumed as the program consumes it, which is the only state in which the
//! question has a right answer.
//!
//! # How the non-cutting geometry is isolated
//!
//! By sweeping it directly, using the **same swept-volume code that performs the
//! cut**. That last part is what keeps the answer consistent with the cutter: a
//! second implementation written to answer this question separately would
//! eventually disagree about a grazing contact, and the disagreement would
//! surface as a collision reported against a move that never happened.
//!
//! A profile has to begin at the tip and a shank does not, so the first attempt
//! swept the whole tool, swept the cutter alone, and subtracted. Correct, and it
//! computed a swept volume twice on every ray — about half the cost of the whole
//! check. [`non_cutting_only`] builds a legal profile for the shank and holder
//! instead, by running up the axis at zero radius to the top of the flutes; the
//! stub revolves to nothing, so the solid is exactly the non-cutting part.
//!
//! # What penetration means here
//!
//! The overlap of two span sets along a ray, measured **along that ray**. Exact
//! along the ray and sampled across the bundle, which is the same property every
//! other measurement in this engine has, and it is reported under a name that
//! says so rather than as an unqualified depth.

use std::collections::BTreeMap;

use crate::dexel::tri::TriDexelField;
use crate::math::{Aabb3, Axis, Ray, Vec2, Vec3};
use crate::spans::Spans;
use crate::sweep::Motion;
use crate::sweep::cut::{
    CutScratch, SweepMethod, cut_tri_motion, swept_spans_for, transverse_overlaps,
};
use crate::tool::Profile;
use crate::tool::profile::{ElementRole, ProfileElement, RoledElement};
use crate::toolpath::{MotionKind, Provenance};

use super::attribute::Attribution;
use super::collide::{Collision, Contact, Obstacle, collision_id, sort_canonically};

/// How collision checking was configured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollideParams {
    /// Report a near miss when the gap is below this. Zero disables it.
    pub clearance_mm: f64,
    /// Quantisation for content-derived identity, as for findings.
    pub grid_mm: f64,
    /// How swept volumes are computed.
    pub method: SweepMethod,
}

/// Why collision checking could not run.
///
/// Returned rather than swallowed, because every one of these has to become an
/// `unchecked` gate with a reason attached. A checker that quietly reported no
/// collisions in these cases would be at its most dangerous exactly when it knew
/// least.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Unchecked {
    /// The tool carries no holder geometry.
    NoHolder,
    /// The program was expanded without motion the machine will make.
    UnmodelledRetracts(u32),
}

impl core::fmt::Display for Unchecked {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoHolder => write!(
                f,
                "the tool defines no holder geometry, so nothing above the shank \
                 could be found hitting anything; add a holder to the tool library"
            ),
            Self::UnmodelledRetracts(n) => write!(
                f,
                "{n} retract(s) in this program were not modelled, so the machine \
                 makes motion this replay does not contain and a clean result would \
                 not cover it"
            ),
        }
    }
}

/// The cutting geometry alone, as a profile in its own right.
///
/// Valid because a profile is closed by the axis and one top cap: truncating the
/// chain at the top of the flutes leaves a well-defined solid, which is exactly
/// the cutter without its shank.
///
/// `None` when the tool is entirely cutting geometry, which has no non-cutting
/// part to collide with anything.
#[must_use]
pub fn cutting_only(profile: &Profile) -> Option<Profile> {
    let cutting: Vec<_> = profile
        .elements()
        .iter()
        .filter(|e| e.role == ElementRole::Cutting)
        .copied()
        .collect();
    if cutting.len() == profile.elements().len() {
        return None;
    }
    Profile::new(cutting).ok()
}

/// Whether a tool can be collision-checked at all.
#[must_use]
pub fn holder_present(profile: &Profile) -> bool {
    profile.top_of_role(ElementRole::Holder).is_some()
}

/// The non-cutting geometry alone, as a profile in its own right.
///
/// # Why this is worth the trouble
///
/// The obvious way to isolate the shank and holder is to sweep the whole tool,
/// sweep the cutter alone, and subtract. That is correct, and it computes a
/// swept volume **twice per ray** — which turned out to be about half the cost
/// of collision checking. Sweeping the non-cutting part directly does one.
///
/// # How a profile that does not start at the tip is made legal
///
/// A profile must begin at `(0, 0)` and its solid is bounded by the chain, the
/// axis, and one top cap. So the chain runs **up the axis at zero radius** to
/// the top of the flutes, steps out to the shank, and continues. The zero-radius
/// stub revolves to nothing, so the solid is exactly the shank and holder and
/// contains no part of the cutter.
#[must_use]
pub fn non_cutting_only(profile: &Profile) -> Option<Profile> {
    let first = profile
        .elements()
        .iter()
        .position(|e| e.role != ElementRole::Cutting)?;
    let start = profile.elements()[first].element.start();
    let mut chain = Vec::with_capacity(profile.elements().len() - first + 2);
    // Up the axis to the height where the non-cutting geometry begins.
    if start.y > 0.0 {
        chain.push(RoledElement::non_cutting(ProfileElement::Segment {
            start: Vec2::new(0.0, 0.0),
            end: Vec2::new(0.0, start.y),
        }));
    }
    // Out to where it actually starts. A horizontal step, so it revolves to an
    // annular disc of zero thickness and adds no volume.
    if start.x > 0.0 {
        chain.push(RoledElement::non_cutting(ProfileElement::Segment {
            start: Vec2::new(0.0, start.y),
            end: start,
        }));
    }
    chain.extend(profile.elements()[first..].iter().copied());
    Profile::new(chain).ok()
}

/// The grouping key: motion, role severity, element, contact kind, obstacle.
type GroupKey = (usize, u8, u32, bool, u8, u32);

/// One overlap or near miss found on one ray.
struct Hit {
    at: Vec3,
    length_mm: f64,
    role: ElementRole,
    element_index: u32,
    obstacle: Obstacle,
    contact: Contact,
}

/// The smallest gap between two span sets, measured **along the ray**.
///
/// `None` when either set is empty.
///
/// This is a one-dimensional gap and therefore an **upper bound** on the true
/// distance: a ray running nearly parallel to both surfaces crosses a long
/// stretch of air between two points that are in fact close together. The bound
/// falls the right way — the reported gap is never smaller than the real one, so
/// a near miss is never invented, though a shallow-angle one can be missed.
///
/// That asymmetry is deliberate. A near miss is a warning about the next edit,
/// and a warning that fires on geometry which is not actually close would be
/// switched off within a week, taking the real ones with it.
fn closest_gap(a: &Spans, b: &Spans) -> Option<f64> {
    if a.is_empty() || b.is_empty() {
        return None;
    }
    let mut best = f64::INFINITY;
    for x in a.iter() {
        for y in b.iter() {
            // Zero when they touch or overlap; otherwise the distance between
            // the facing endpoints.
            let gap = if x.t1 < y.t0 {
                y.t0 - x.t1
            } else if y.t1 < x.t0 {
                x.t0 - y.t1
            } else {
                0.0
            };
            best = best.min(gap);
        }
    }
    best.is_finite().then_some(best)
}

/// The role and profile index of whatever non-cutting element sits at height
/// `z` above the tip.
fn element_at(profile: &Profile, z: f64) -> (ElementRole, u32) {
    let mut best = (ElementRole::NonCutting, 0u32);
    for (i, e) in profile.elements().iter().enumerate() {
        if e.role == ElementRole::Cutting {
            continue;
        }
        let (lo, hi) = e.element.z_range();
        if z >= lo - 1.0e-9 && z <= hi + 1.0e-9 {
            #[allow(clippy::cast_possible_truncation, reason = "a handful of elements")]
            {
                best = (e.role, i as u32);
            }
            // Keep scanning: the holder is above the shank, and where two
            // elements share a z the more severe role is the one to report.
            if e.role == ElementRole::Holder {
                break;
            }
        }
    }
    best
}

/// Replays a program, checking non-cutting geometry against the stock as it
/// stands at each step.
///
/// `stock` is **the field the program starts from** and is consumed in place.
/// Passing a cut field would answer a different and wrong question; see the
/// module header.
///
/// # Errors
///
/// Returns [`Unchecked`] when the inputs cannot support an answer.
#[allow(
    clippy::too_many_arguments,
    reason = "one replay loop; a struct here would exist only to be destructured"
)]
pub fn collide_with_stock(
    stock: &mut TriDexelField,
    profile: &Profile,
    motions: &[Motion],
    kinds: &[MotionKind],
    provenance: &[Provenance],
    unmodelled_retracts: u32,
    fixtures: &[(String, TriDexelField)],
    params: &CollideParams,
    scratch: &mut CutScratch,
) -> Result<Vec<Collision>, Unchecked> {
    if !holder_present(profile) {
        return Err(Unchecked::NoHolder);
    }
    if unmodelled_retracts > 0 {
        return Err(Unchecked::UnmodelledRetracts(unmodelled_retracts));
    }
    let Some(non_cutting) = non_cutting_only(profile) else {
        // Entirely cutting geometry: nothing above the flutes exists, so there
        // is nothing that could collide. An empty list is the right answer here
        // and is not the same as `unchecked`.
        return Ok(Vec::new());
    };

    let mut raw: Vec<(usize, Hit)> = Vec::new();
    for (k, motion) in motions.iter().enumerate() {
        for hit in hits_for_motion(
            stock,
            &Obstacle::Stock,
            profile,
            &non_cutting,
            motion,
            params,
            scratch,
        ) {
            raw.push((k, hit));
        }
        // Fixtures are checked against the same motion and never cut. A clamp
        // does not get out of the way, which is the entire reason it is
        // dangerous.
        for (index, (name, field)) in fixtures.iter().enumerate() {
            let obstacle = Obstacle::Fixture {
                index: u32::try_from(index).unwrap_or(u32::MAX),
                name: name.clone(),
            };
            for hit in hits_for_motion(
                field,
                &obstacle,
                profile,
                &non_cutting,
                motion,
                params,
                scratch,
            ) {
                raw.push((k, hit));
            }
        }
        // Then, and only then, remove what this motion cuts.
        cut_tri_motion(stock, profile, motion, params.method, scratch);
    }

    Ok(assemble(raw, kinds, provenance, params))
}

/// Every overlap this motion's non-cutting geometry has with one obstacle.
#[allow(
    clippy::too_many_arguments,
    reason = "the inner loop, shared by stock and fixtures"
)]
fn hits_for_motion(
    field: &TriDexelField,
    obstacle: &Obstacle,
    profile: &Profile,
    cutting: &Profile,
    motion: &Motion,
    params: &CollideParams,
    scratch: &mut CutScratch,
) -> Vec<Hit> {
    // `cutting` is the non-cutting sub-profile; the name is kept for the call
    // sites below, which read it as "the geometry being swept".
    let mut out = Vec::new();
    // Widened by the clearance when near misses are wanted, so a ray that passes
    // close without touching is still visited. Without this the rejection would
    // discard exactly the rays a near miss lives on.
    let bounds = motion.swept_bounds(profile).expand(params.clearance_mm);
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let Some(bundle) = field.bundle(axis) else {
            continue;
        };
        let lattice = bundle.lattice().clone();
        let rays = u32::try_from(bundle.arena().rays()).unwrap_or(u32::MAX);
        for ray_index in 0..rays {
            if bundle.arena().span_count(ray_index) == 0 {
                continue;
            }
            let (i, j) = lattice.coords(ray_index);
            let origin = lattice.origin_of(i, j);
            if !transverse_overlaps(&bounds, axis, origin, lattice.spacing_uv()) {
                continue;
            }
            let ray = Ray {
                origin,
                direction: axis.direction(),
            };

            // One sweep, of the shank and holder alone. See `non_cutting_only`
            // for why this is not the whole tool minus the cutter.
            let Some(non_cutting) = swept_spans_for(cutting, motion, params.method, scratch, &ray)
            else {
                continue;
            };
            let non_cutting = non_cutting.clone();
            if non_cutting.is_empty() {
                continue;
            }

            let mut material = Spans::with_capacity(4);
            bundle.arena().read_into(ray_index, &mut material);
            let overlap = non_cutting.intersect(&material);
            for s in overlap.iter() {
                let length = s.length();
                if length <= 0.0 {
                    continue;
                }
                let at = origin + axis.direction() * s.midpoint();
                let z = height_above_tip(motion, at);
                let (role, element_index) = element_at(profile, z);
                out.push(Hit {
                    at,
                    length_mm: length,
                    role,
                    element_index,
                    obstacle: obstacle.clone(),
                    contact: Contact::Collision {
                        penetration_mm: length,
                    },
                });
            }

            // A near miss, only where there was no contact at all on this ray:
            // a ray that already collides has nothing to warn about.
            if params.clearance_mm > 0.0
                && overlap.is_empty()
                && let Some(gap) = closest_gap(&non_cutting, &material)
                && gap < params.clearance_mm
            {
                {
                    let at = non_cutting
                        .hull()
                        .map_or(origin, |h| origin + axis.direction() * h.midpoint());
                    let z = height_above_tip(motion, at);
                    let (role, element_index) = element_at(profile, z);
                    out.push(Hit {
                        at,
                        length_mm: gap,
                        role,
                        element_index,
                        obstacle: obstacle.clone(),
                        contact: Contact::NearMiss { clearance_mm: gap },
                    });
                }
            }
        }
    }
    out
}

/// How far above the tool tip a point sits, at the nearest point of the motion.
///
/// The tool is a solid of revolution about `+Z`, so this is the point's height
/// over whichever tip position on the path is closest to it.
fn height_above_tip(motion: &Motion, at: Vec3) -> f64 {
    let tip = match motion {
        Motion::Linear(l) => {
            let d = l.end - l.start;
            let len2 = d.dot(d);
            if len2 <= 0.0 {
                l.start
            } else {
                let t = ((at - l.start).dot(d) / len2).clamp(0.0, 1.0);
                l.start + d * t
            }
        }
        // An arc's tip height is its own z, which the swept volume already
        // accounts for; the nearest-point refinement below the millimetre does
        // not change which element is named.
        other => other.at(0.0),
    };
    at.z - tip.z
}

/// Groups raw ray overlaps into collisions, one per element per motion per
/// obstacle, and gives each an identity.
fn assemble(
    raw: Vec<(usize, Hit)>,
    kinds: &[MotionKind],
    provenance: &[Provenance],
    params: &CollideParams,
) -> Vec<Collision> {
    // **One collision per element per motion**, not one per ray and not one per
    // grid cell.
    //
    // A holder ploughing through a block overlaps thousands of rays, and an
    // earlier version reported each cell separately: 1038 entries for a single
    // plunge. That is not a report anybody can act on, and it is not more
    // information — every one of those entries names the same element on the
    // same move, and the only number that differs is which ray happened to be
    // deepest.
    //
    // What a reader needs is "the holder hit the stock on line 5, worst
    // penetration 50.4 mm", which is one row. The bounds carry the extent, and
    // the worst point carries where to look.
    let q = |v: f64| {
        let c = (v / params.grid_mm).floor();
        #[allow(clippy::cast_possible_truncation, reason = "saturating")]
        {
            c as i64
        }
    };
    let mut groups: BTreeMap<GroupKey, Vec<Hit>> = BTreeMap::new();
    for (k, hit) in raw {
        let (class, index) = hit.obstacle.order();
        groups
            .entry((
                k,
                hit.role.severity(),
                hit.element_index,
                // Contact and near miss never merge: one is a crash and the
                // other is a warning, and a group containing both would have to
                // report one number for two different quantities.
                hit.contact.is_collision(),
                class,
                index,
            ))
            .or_default()
            .push(hit);
    }

    let mut out = Vec::new();
    let mut seen: BTreeMap<(i64, i64, i64), u32> = BTreeMap::new();
    for ((k, _, element_index, is_collision, _, _), hits) in groups {
        // Worst means deepest for a collision and **closest** for a near miss.
        // Taking the maximum of both would report the least alarming near miss
        // in the group, which is the one nobody needs to know about.
        let worst = if is_collision {
            hits.iter()
                .max_by(|a, b| a.length_mm.total_cmp(&b.length_mm))
        } else {
            hits.iter()
                .min_by(|a, b| a.length_mm.total_cmp(&b.length_mm))
        }
        .expect("a group is never empty");
        let motion = kinds.get(k).copied().unwrap_or(MotionKind::Linear);
        let mut bounds = Aabb3::EMPTY;
        for h in &hits {
            bounds = bounds.union_point(h.at);
        }
        let cell = (q(worst.at.x), q(worst.at.y), q(worst.at.z));
        let slot = seen.entry(cell).or_insert(0);
        let id = collision_id(
            worst.role,
            &worst.obstacle,
            motion,
            worst.at,
            params.grid_mm,
            *slot,
        );
        *slot += 1;
        let attribution = provenance
            .get(k)
            .map_or_else(Attribution::none, |p| Attribution {
                segments: vec![u32::try_from(k).unwrap_or(u32::MAX)],
                provenance: vec![*p],
            });
        out.push(Collision {
            id,
            contact: worst.contact,
            role: worst.role,
            element_index,
            obstacle: worst.obstacle.clone(),
            at: worst.at,
            bounds,
            motion,
            attribution,
        });
    }
    sort_canonically(&mut out);
    out
}
