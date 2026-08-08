// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Does a cut face carry the normal of the surface that cut it?
//!
//! For five units it did not. Every swept span was built with
//! [`chipbreaker_core::spans::Span::ordered`], which leaves the placeholder,
//! which decodes to `+Z`, which the subtraction negates to `(0, 0, -1)`. A
//! slot's two end walls face opposite directions and shared one normal, and
//! nothing noticed, because dual contouring averages several crossings per
//! vertex and the sharp-feature tests all used uncut boxes.
//!
//! # The oracle is not the engine
//!
//! Comparing the stored normal against
//! [`chipbreaker_core::tool::surface_normal`] would test the sweep against the
//! very function the sweep calls, which is a spelling check. So the oracle here
//! is geometric and independent:
//!
//! **A ball-nose end mill is a Minkowski sum.** A ball mill of radius `r` is
//! exactly the set of points within `r` of the segment running up its axis from
//! `z = r` to the top of the flutes — a capsule. Sweeping it along a straight
//! motion sums that segment with the motion, giving a **parallelogram**, and the
//! swept solid is every point within `r` of it. So for any point `p` on the swept
//! surface, the direction out of the *cutter* is
//!
//! ```text
//! normalize( p - nearest point of the parallelogram )
//! ```
//!
//! with no reference to profiles, elements, arcs, or spans. The nearest point of
//! a parallelogram is a two-variable clamped least-squares problem, solved here
//! exactly rather than sampled.
//!
//! # Which way is out
//!
//! A span endpoint stores the outward normal of the **workpiece**, and on a cut
//! face that is the reverse of the cutter's: the material the tool removed sat on
//! the tool's side, so what remains faces the other way. The subtraction in
//! `spans` is where the reversal happens, and it is the single sign the whole
//! convention rests on — get it backwards and the result is a watertight,
//! manifold, inside-out mesh.
//!
//! So the oracle below negates: it takes the direction **from the swept surface
//! back toward the tool's centre line**. Written the other way round, every case
//! here failed by exactly 180 degrees, which is a pleasant way to find out that
//! the directions were already right.
//!
//! # It cannot go vacuous
//!
//! The failure this file exists for makes every cut normal `(0, 0, -1)`, and a
//! cut with a level floor has genuine `(0, 0, +1)` faces of its own — so a test
//! that merely looked for variety, or counted placeholders, would pass while the
//! defect was present. Two guards:
//!
//! * Every case asserts a **minimum number of cut endpoints checked**, so a
//!   filter that quietly matched nothing fails rather than passes.
//! * [`the_check_rejects_the_defect_it_was_written_for`] runs the same comparison
//!   against the normals the engine used to produce and asserts it **fails**.
//!   That is the evidence CONTRIBUTING.md asks for, executed rather than
//!   described.

use chipbreaker_core::dexel::tri::{AXES, TriBuildOptions, TriDexelField};
use chipbreaker_core::math::{OctNormal, Vec3};
use chipbreaker_core::mesh::shapes;
use chipbreaker_core::sweep::batch::{DEFAULT_BATCH, cut_all};
use chipbreaker_core::sweep::cut::{CutScratch, SweepMethod};
use chipbreaker_core::sweep::{LinearMove, Motion};
use chipbreaker_core::tool::Profile;
use chipbreaker_core::tool::catalog::{Shank, ball_end_mill};

const SPACING: f64 = 0.35;
const STOCK: [f64; 3] = [30.0, 24.0, 12.0];

/// Ball radius used throughout, and the top of the flutes above the tip.
const RADIUS: f64 = 3.0;
const FLUTE: f64 = 25.0;

/// How far from the swept surface a point may lie and still count as being on
/// it.
///
/// Span endpoints are exact roots, so this is a guard against calling a *stock*
/// face a cut face, not a tolerance on the roots themselves.
const ON_SURFACE: f64 = 1.0e-7;

/// Angular agreement demanded, in degrees.
///
/// Sixteen bits per component spread over an octahedron give about `2^32`
/// distinct directions, so the quantisation floor is near
/// `sqrt(4 pi / 2^32) = 0.003` degrees — and that is what the level cases
/// measure, to two figures, which is a quiet confirmation that nothing else is
/// contributing.
///
/// The body-diagonal ramp reaches 0.015 degrees, five times the floor. That is
/// **not** encoding: a ramp is sub-stepped, so its swept surface is a fan of
/// chords rather than the smooth sweep the oracle computes, and the residual is
/// the linearisation bound. Both sit far below this threshold, which is
/// set to catch a wrong *face* rather than to pin a rounding.
const TOLERANCE_DEG: f64 = 0.2;

fn mill() -> Profile {
    ball_end_mill(2.0 * RADIUS, FLUTE, &Shank::plain(2.0 * RADIUS, 60.0)).expect("valid")
}

fn cut(motions: &[Motion]) -> TriDexelField {
    let mesh = shapes::box_solid(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(STOCK[0], STOCK[1], STOCK[2]),
    );
    let mut field = TriDexelField::build(
        &mesh,
        &TriBuildOptions {
            spacing: SPACING,
            ..TriBuildOptions::default()
        },
    )
    .expect("builds")
    .0;
    let profile = mill();
    let mut scratch = CutScratch::new(&profile);
    cut_all(
        &mut field,
        &profile,
        motions,
        SweepMethod::Analytic {
            tolerance: SPACING / 10.0,
        },
        &mut scratch,
        DEFAULT_BATCH,
    );
    field
}

/// The parallelogram of ball centres swept by one linear motion.
///
/// Corner `a`, and the two edge vectors spanning it: the motion, and the tool's
/// own axis from the ball centre to the top of the flutes.
struct Centres {
    a: Vec3,
    motion: Vec3,
    axis: Vec3,
}

impl Centres {
    fn of(m: &LinearMove) -> Self {
        Self {
            a: Vec3::new(m.start.x, m.start.y, m.start.z + RADIUS),
            motion: Vec3::new(
                m.end.x - m.start.x,
                m.end.y - m.start.y,
                m.end.z - m.start.z,
            ),
            axis: Vec3::new(0.0, 0.0, FLUTE - RADIUS),
        }
    }

    /// The point of the parallelogram nearest to `p`.
    ///
    /// The unconstrained minimum first, from the two-by-two normal equations; if
    /// it falls outside the unit square the minimum is on the boundary, and each
    /// of the four edges is a clamped projection onto a segment.
    fn nearest(&self, p: Vec3) -> Vec3 {
        let (u, v) = (self.motion, self.axis);
        let d = Vec3::new(p.x - self.a.x, p.y - self.a.y, p.z - self.a.z);
        let (uu, uv, vv) = (dot(u, u), dot(u, v), dot(v, v));
        let det = uu * vv - uv * uv;
        if det.abs() > 1.0e-12 {
            let s = (dot(d, u) * vv - dot(d, v) * uv) / det;
            let t = (dot(d, v) * uu - dot(d, u) * uv) / det;
            if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
                return add(self.a, add(scale(u, s), scale(v, t)));
            }
        }
        // On the boundary. A degenerate parallelogram -- a plunge, where the
        // motion is parallel to the tool axis -- reaches this too, and its edges
        // still describe the segment correctly.
        let mut best = self.a;
        let mut best_d2 = f64::INFINITY;
        for (from, along) in [
            (self.a, u),
            (self.a, v),
            (add(self.a, v), u),
            (add(self.a, u), v),
        ] {
            let point = clamped_projection(from, along, p);
            let d2 = dist2(point, p);
            if d2 < best_d2 {
                best_d2 = d2;
                best = point;
            }
        }
        best
    }
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

fn scale(a: Vec3, k: f64) -> Vec3 {
    Vec3::new(a.x * k, a.y * k, a.z * k)
}

fn dist2(a: Vec3, b: Vec3) -> f64 {
    let (dx, dy, dz) = (a.x - b.x, a.y - b.y, a.z - b.z);
    dx * dx + dy * dy + dz * dz
}

fn clamped_projection(from: Vec3, along: Vec3, p: Vec3) -> Vec3 {
    let len2 = dot(along, along);
    if len2 <= 0.0 {
        return from;
    }
    let d = Vec3::new(p.x - from.x, p.y - from.y, p.z - from.z);
    let s = (dot(d, along) / len2).clamp(0.0, 1.0);
    add(from, scale(along, s))
}

/// One cut endpoint: where it is, what the engine stored, and what the geometry
/// says.
struct Sample {
    at: Vec3,
    stored: Vec3,
    truth: Vec3,
}

/// Every span endpoint that lies on the surface swept by one of `motions`.
///
/// A point is on that surface when its distance to the nearest ball-centre
/// parallelogram equals the ball radius. Stock faces fail that test, which is how
/// the two are told apart without knowing which faces the stock had.
fn cut_endpoints(field: &TriDexelField, motions: &[Motion]) -> Vec<Sample> {
    let hulls: Vec<Centres> = motions
        .iter()
        .filter_map(|m| match m {
            Motion::Linear(l) => Some(Centres::of(l)),
            _ => None,
        })
        .collect();

    let mut out = Vec::new();
    for axis in AXES {
        let Some(bundle) = field.bundle(axis) else {
            continue;
        };
        let lattice = bundle.lattice().clone();
        let direction = axis.direction();
        let rays = u32::try_from(bundle.arena().rays()).expect("small");
        for ray in 0..rays {
            let (i, j) = lattice.coords(ray);
            let origin = lattice.origin_of(i, j);
            for span in bundle.arena().get(ray) {
                for (t, code) in [(span.t0, span.n0), (span.t1, span.n1)] {
                    let at = add(origin, scale(direction, t));
                    // The motion whose swept surface this point lies on, if any.
                    // Interior points of one sweep that fall on the surface of
                    // another are excluded: there the two solids meet and the
                    // boundary belongs to whichever reaches further.
                    let mut on: Option<Vec3> = None;
                    let mut inside_any = false;
                    for hull in &hulls {
                        let near = hull.nearest(at);
                        let d = dist2(near, at).sqrt();
                        if d < RADIUS - ON_SURFACE {
                            inside_any = true;
                        } else if (d - RADIUS).abs() <= ON_SURFACE {
                            // Toward the centre line, not away from it: see the
                            // module header on which way is out.
                            let n = Vec3::new(near.x - at.x, near.y - at.y, near.z - at.z);
                            on = n.normalize();
                        }
                    }
                    if inside_any {
                        continue;
                    }
                    if let Some(truth) = on {
                        out.push(Sample {
                            at,
                            stored: code.decode(),
                            truth,
                        });
                    }
                }
            }
        }
    }
    out
}

/// Angle between two unit vectors, in degrees.
fn angle_deg(a: Vec3, b: Vec3) -> f64 {
    let c = dot(a, b).clamp(-1.0, 1.0);
    chipbreaker_core::transcendental::acos(c).to_degrees()
}

/// Runs one case and reports the worst disagreement.
fn check(name: &str, motions: &[Motion], least: usize) -> f64 {
    let field = cut(motions);
    let samples = cut_endpoints(&field, motions);
    assert!(
        samples.len() >= least,
        "{name}: only {} cut endpoints were identified, against a floor of \
         {least}. The filter matched nothing, so a pass would have meant nothing.",
        samples.len()
    );

    let mut worst = 0.0f64;
    let mut worst_at = Vec3::new(0.0, 0.0, 0.0);
    let mut worst_pair = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    for s in &samples {
        let a = angle_deg(s.stored, s.truth);
        if a > worst {
            worst = a;
            worst_at = s.at;
            worst_pair = (s.stored, s.truth);
        }
    }
    println!(
        "{name}: {} cut endpoints, worst {worst:.4} deg at ({:.3}, {:.3}, {:.3}) \
         stored ({:.3}, {:.3}, {:.3}) truth ({:.3}, {:.3}, {:.3})",
        samples.len(),
        worst_at.x,
        worst_at.y,
        worst_at.z,
        worst_pair.0.x,
        worst_pair.0.y,
        worst_pair.0.z,
        worst_pair.1.x,
        worst_pair.1.y,
        worst_pair.1.z,
    );
    assert!(
        worst <= TOLERANCE_DEG,
        "{name}: a cut face is {worst:.4} degrees out at ({:.3}, {:.3}, {:.3}). \
         Stored ({:.3}, {:.3}, {:.3}), geometry says ({:.3}, {:.3}, {:.3}). The \
         oct encoding costs about 0.003 degrees and sub-stepping a little more; \
         anything near this threshold is the wrong surface, not a rounding.",
        worst_at.x,
        worst_at.y,
        worst_at.z,
        worst_pair.0.x,
        worst_pair.0.y,
        worst_pair.0.z,
        worst_pair.1.x,
        worst_pair.1.y,
        worst_pair.1.z,
    );
    worst
}

fn line(start: [f64; 3], end: [f64; 3]) -> Motion {
    Motion::Linear(LinearMove {
        start: Vec3::new(start[0], start[1], start[2]),
        end: Vec3::new(end[0], end[1], end[2]),
    })
}

#[test]
fn a_level_pass_along_x() {
    // Case A, the ordinary one: contouring, pocketing, facing.
    check("along x", &[line([5.0, 12.0, 8.0], [25.0, 12.0, 8.0])], 400);
}

#[test]
fn a_level_pass_along_y_and_across_the_bundles() {
    // The same motion turned ninety degrees. Case A maps the ray into the
    // motion's own frame and the normal has to come back out of it; an
    // unrotated normal passes the X case and fails this one.
    check("along y", &[line([15.0, 4.0, 8.0], [15.0, 20.0, 8.0])], 400);
}

#[test]
fn a_face_diagonal_pass() {
    // Neither the mapped frame nor the world frame is axis aligned, so a
    // rotation applied in the wrong sense shows up here and nowhere above.
    check(
        "face diagonal",
        &[line([6.0, 5.0, 8.0], [24.0, 19.0, 8.0])],
        400,
    );
}

#[test]
fn a_plunge() {
    // Case B, both of its ray orientations at once: the Z bundle runs along the
    // plunge and dilates its spans, the X and Y bundles run across it and take a
    // chord of the swept disc.
    check(
        "plunge",
        &[line([15.0, 12.0, 14.0], [15.0, 12.0, 5.0])],
        300,
    );
}

#[test]
fn a_body_diagonal_ramp() {
    // The case with no symmetry left to hide behind. A ramp along `(1, 1, 1)`
    // is neither level nor vertical, so it takes the sub-stepped path, and every
    // one of its sub-steps must place its own normals correctly for the union to
    // come out right.
    check(
        "body diagonal",
        &[line([7.0, 6.0, 11.0], [22.0, 18.0, 5.0])],
        300,
    );
}

#[test]
fn a_pass_that_doubles_back_over_itself() {
    // Two motions whose swept solids overlap. Where they do, the union has to
    // keep the normal of whichever surface is actually outermost, which is the
    // one property a per-motion answer cannot supply on its own.
    check(
        "doubled back",
        &[
            line([5.0, 10.0, 8.0], [25.0, 10.0, 8.0]),
            line([25.0, 13.0, 8.0], [5.0, 13.0, 8.0]),
        ],
        600,
    );
}

#[test]
fn the_check_rejects_the_defect_it_was_written_for() {
    // The evidence CONTRIBUTING.md asks for, run rather than asserted in prose.
    //
    // Before the fix every cut endpoint carried `PLACEHOLDER` negated by the
    // subtraction. Substituting that here must make the comparison fail; if it
    // does not, the comparison is not measuring what it claims to and every pass
    // above is worthless.
    let motions = [line([6.0, 5.0, 8.0], [24.0, 19.0, 8.0])];
    let field = cut(&motions);
    let samples = cut_endpoints(&field, &motions);
    assert!(!samples.is_empty(), "no cut endpoints to test against");

    let defect = OctNormal::PLACEHOLDER.negated().decode();
    assert!(
        (defect.z + 1.0).abs() < 1.0e-9,
        "the defect being reproduced is (0, 0, -1); the placeholder now negates \
         to ({:.3}, {:.3}, {:.3}), so this test no longer reproduces it",
        defect.x,
        defect.y,
        defect.z
    );

    let worst = samples
        .iter()
        .map(|s| angle_deg(defect, s.truth))
        .fold(0.0f64, f64::max);
    assert!(
        worst > TOLERANCE_DEG,
        "substituting the old placeholder normal on every cut face still passed \
         the {TOLERANCE_DEG} degree check, worst {worst:.4}. The check cannot \
         detect the defect it exists for."
    );

    // And it is not a near miss: a slot cut across the stock has walls facing
    // every which way, so a single normal must be wildly wrong somewhere.
    assert!(
        worst > 45.0,
        "the old normal was only {worst:.4} degrees out at worst, which would \
         mean the cut has no faces transverse to Z. Choose a case that does."
    );
}
