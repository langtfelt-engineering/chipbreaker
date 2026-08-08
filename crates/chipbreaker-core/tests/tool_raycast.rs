// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt

//! Ray-versus-tool intersection, checked against closed forms and against the
//! tool's own analytic volume.
//!
//! # The two things being established
//!
//! **No ray leaks.** Every span is bounded, lies inside the tool's bounding
//! cylinder, and has both endpoints on the surface. A leak is the failure that
//! matters: in a field an unbounded span is a column of stock removed from the middle
//! of a part, and it is silent.
//!
//! **A bundle of rays recovers the volume.** Summing span length times cell area
//! over a dense parallel bundle is a Riemann sum for the volume, and the volume
//! is known in closed form from section 6. This is a far stronger check than any
//! hand-written case: it exercises every surface type at every incidence angle
//! at once, and a systematically wrong span — one entering surface missed, one
//! tangency mishandled — moves the total. It is run along all three axes,
//! because a ray parallel to the tool axis never tests the cap and a ray
//! perpendicular to it never tests the cone the same way.

use chipbreaker_core::eps::EPS_LENGTH;
use chipbreaker_core::math::{Ray, Vec3};
use chipbreaker_core::tool::catalog::{
    HolderStage, Shank, ball_end_mill, barrel_end_mill, bull_end_mill, chamfer_mill, drill,
    flat_end_mill, tapered_end_mill,
};
use chipbreaker_core::tool::profile::Profile;
use chipbreaker_core::tool::raycast::{RaycastScratch, RaycastStats};

/// Every standard form, so that each test covers cylinders, cones, discs,
/// spheres, and tori.
fn corpus() -> Vec<(&'static str, Profile)> {
    let shank = Shank::plain(6.0, 50.0);
    let stepped = Shank::plain(8.0, 55.0);
    let held = Shank::with_holder(
        6.0,
        40.0,
        [
            HolderStage::cylinder(25.0, 20.0),
            HolderStage::taper(25.0, 40.0, 15.0),
        ],
    );
    vec![
        ("flat", flat_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "flat-stepped-shank",
            flat_end_mill(6.0, 20.0, &stepped).expect("valid"),
        ),
        (
            "flat-necked",
            flat_end_mill(10.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid"),
        ),
        ("ball", ball_end_mill(6.0, 20.0, &shank).expect("valid")),
        (
            "bull",
            bull_end_mill(6.0, 1.5, 20.0, &shank).expect("valid"),
        ),
        (
            "bull-small-corner",
            bull_end_mill(10.0, 0.5, 20.0, &Shank::plain(10.0, 50.0)).expect("valid"),
        ),
        (
            "chamfer",
            chamfer_mill(8.0, 1.0, 90.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "vbit",
            chamfer_mill(8.0, 0.0, 60.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        (
            "tapered",
            tapered_end_mill(2.0, 10.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid"),
        ),
        ("drill", drill(6.0, 118.0, 30.0, &shank).expect("valid")),
        (
            "barrel",
            barrel_end_mill(12.0, 60.0, 40.0, &Shank::plain(12.0, 70.0)).expect("valid"),
        ),
        ("held", flat_end_mill(6.0, 20.0, &held).expect("valid")),
    ]
}

/// Casts one bundle of parallel rays over the tool's bounding box and returns
/// the volume the spans imply, together with the statistics.
///
/// `axis` selects the direction: 0 for `+X`, 1 for `+Y`, 2 for `+Z`. Rays are
/// placed at cell *centres*, which for an even `n` keeps every ray off the tool
/// axis — a ray exactly on the axis meets every surface of revolution
/// tangentially, and sampling it would test the degenerate case while pretending
/// to measure a volume.
fn bundle_volume(profile: &Profile, axis: usize, n: usize) -> (f64, RaycastStats) {
    let cylinder = profile.bounding_cylinder();
    let radius = cylinder.radius * 1.25 + 1.0;
    let z_lo = cylinder.z_min - 1.0;
    let z_hi = cylinder.z_max + 1.0;

    // The two coordinates swept, and the span the ray travels along.
    let (u_lo, u_hi, v_lo, v_hi) = if axis == 2 {
        (-radius, radius, -radius, radius)
    } else {
        (-radius, radius, z_lo, z_hi)
    };
    let du = (u_hi - u_lo) / n as f64;
    let dv = (v_hi - v_lo) / n as f64;

    let mut scratch = RaycastScratch::with_capacity(profile.len());
    let mut spans = chipbreaker_core::spans::Spans::new();
    let mut stats = RaycastStats::default();
    let mut total = 0.0f64;

    for i in 0..n {
        let u = u_lo + (i as f64 + 0.5) * du;
        for j in 0..n {
            let v = v_lo + (j as f64 + 0.5) * dv;
            let origin = match axis {
                0 => Vec3::new(-radius - 1.0, u, v),
                1 => Vec3::new(u, -radius - 1.0, v),
                _ => Vec3::new(u, v, z_lo - 1.0),
            };
            let direction = match axis {
                0 => Vec3::new(1.0, 0.0, 0.0),
                1 => Vec3::new(0.0, 1.0, 0.0),
                _ => Vec3::new(0.0, 0.0, 1.0),
            };
            let ray = Ray::new_normalized(origin, direction).expect("a unit axis direction");
            profile.intersect_ray_into(&ray, &mut scratch, &mut spans, &mut stats);
            total += spans.measure();
        }
    }
    (total * du * dv, stats)
}

#[test]
fn a_ray_through_a_cylinder_gives_exactly_the_chord() {
    // A plain cylinder: a flat end mill whose shank matches its cutter.
    let profile = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    // Straight through the middle, along +X at z = 25.
    let ray =
        Ray::new_normalized(Vec3::new(-100.0, 0.5, 25.0), Vec3::new(1.0, 0.0, 0.0)).expect("unit");
    let spans = profile.intersect_ray(&ray);
    assert_eq!(spans.len(), 1, "one span through a convex solid: {spans}");

    // The chord of a circle of radius 3 at offset 0.5.
    let half = (9.0f64 - 0.25).sqrt();
    let s = spans.as_slice()[0];
    assert!((s.t0 - (100.0 - half)).abs() < 1e-9, "{s}");
    assert!((s.t1 - (100.0 + half)).abs() < 1e-9, "{s}");
    assert!((s.length() - 2.0 * half).abs() < 1e-9, "{s}");
}

#[test]
fn a_ray_along_the_axis_of_a_ball_nose_enters_at_the_tip() {
    let profile = ball_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    let ray =
        Ray::new_normalized(Vec3::new(0.0, 0.0, -10.0), Vec3::new(0.0, 0.0, 1.0)).expect("unit");
    let spans = profile.intersect_ray(&ray);
    assert_eq!(spans.len(), 1, "{spans}");
    let s = spans.as_slice()[0];
    assert!((s.t0 - 10.0).abs() < 1e-9, "enters at the tip: {s}");
    assert!((s.t1 - 60.0).abs() < 1e-9, "leaves through the cap: {s}");
}

#[test]
fn a_ray_that_misses_the_tool_returns_nothing() {
    for (name, profile) in corpus() {
        let radius = profile.bounding_cylinder().radius;
        let ray = Ray::new_normalized(
            Vec3::new(-100.0, radius * 2.0 + 5.0, 10.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .expect("unit");
        assert!(
            profile.intersect_ray(&ray).is_empty(),
            "{name} reported material well outside its own bounding cylinder"
        );
    }
}

#[test]
fn a_ray_through_the_corner_radius_of_a_bull_nose_crosses_the_torus() {
    // A 10 mm cutter with a 2 mm corner: the torus runs from z = 0 to z = 2 at
    // radii 3 to 5. A ray at z = 1 crosses it twice.
    let profile = bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(10.0, 50.0)).expect("valid");
    let ray =
        Ray::new_normalized(Vec3::new(-100.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).expect("unit");
    let spans = profile.intersect_ray(&ray);
    assert_eq!(spans.len(), 1, "{spans}");

    // At z = 1 the corner arc is at r = 3 + sqrt(2^2 - 1^2) = 3 + sqrt(3).
    let r = 3.0 + 3.0f64.sqrt();
    let s = spans.as_slice()[0];
    assert!(
        (s.length() - 2.0 * r).abs() < 1e-9,
        "{s}, expected {}",
        2.0 * r
    );
}

#[test]
fn every_span_endpoint_lies_on_the_surface() {
    for (name, profile) in corpus() {
        let cylinder = profile.bounding_cylinder();
        let radius = cylinder.radius * 1.3 + 1.0;
        for i in 0..24 {
            for j in 0..24 {
                let y = -radius + 2.0 * radius * (f64::from(i) + 0.5) / 24.0;
                let z = cylinder.z_min - 0.5
                    + (cylinder.z_max - cylinder.z_min + 1.0) * (f64::from(j) + 0.5) / 24.0;
                let ray =
                    Ray::new_normalized(Vec3::new(-radius - 1.0, y, z), Vec3::new(1.0, 0.0, 0.0))
                        .expect("unit");
                for span in profile.intersect_ray(&ray).iter() {
                    for endpoint in [span.t0, span.t1] {
                        let p = ray.at(endpoint);
                        let contact = profile.nearest_surface((p.x * p.x + p.y * p.y).sqrt(), p.z);
                        assert!(
                            contact.distance < 1e-6,
                            "{name}: span endpoint {endpoint} is {} from the surface",
                            contact.distance
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn spans_agree_with_the_containment_predicate_along_the_whole_ray() {
    for (name, profile) in corpus() {
        let cylinder = profile.bounding_cylinder();
        let radius = cylinder.radius * 1.3 + 1.0;
        let far = 2.0 * (radius + cylinder.z_max) + 4.0;
        for i in 0..16 {
            for j in 0..16 {
                let y = -radius + 2.0 * radius * (f64::from(i) + 0.5) / 16.0;
                let z = cylinder.z_min - 0.5
                    + (cylinder.z_max - cylinder.z_min + 1.0) * (f64::from(j) + 0.5) / 16.0;
                let ray =
                    Ray::new_normalized(Vec3::new(-radius - 1.0, y, z), Vec3::new(1.0, 0.0, 0.0))
                        .expect("unit");
                let spans = profile.intersect_ray(&ray);

                for k in 0..200 {
                    let t = far * f64::from(k) / 200.0;
                    let p = ray.at(t);
                    let inside = profile.contains_xyz(p);
                    if inside == spans.contains(t) {
                        continue;
                    }
                    // A disagreement is only meaningful away from a boundary:
                    // on it, containment is documented as unclassified.
                    let contact = profile.nearest_surface((p.x * p.x + p.y * p.y).sqrt(), p.z);
                    assert!(
                        contact.distance < 1e-6,
                        "{name}: at t = {t} the spans say {} and containment says {inside}, \
                         and the point is {} from any surface",
                        spans.contains(t),
                        contact.distance
                    );
                }
            }
        }
    }
}

#[test]
fn no_span_escapes_the_bounding_cylinder() {
    let mut total = RaycastStats::default();
    for (name, profile) in corpus() {
        let cylinder = profile.bounding_cylinder();
        for axis in 0..3 {
            let (_, stats) = bundle_volume(&profile, axis, 24);
            total.merge(&stats);
            // Re-cast to inspect the spans themselves.
            let radius = cylinder.radius * 1.25 + 1.0;
            let reach = 2.0 * (radius + cylinder.z_max + 2.0);
            for i in 0..24 {
                let u = -radius + 2.0 * radius * (f64::from(i) + 0.5) / 24.0;
                let (origin, direction) = match axis {
                    0 => (
                        Vec3::new(-radius - 1.0, u, 0.5 * cylinder.z_max),
                        Vec3::new(1.0, 0.0, 0.0),
                    ),
                    1 => (
                        Vec3::new(u, -radius - 1.0, 0.5 * cylinder.z_max),
                        Vec3::new(0.0, 1.0, 0.0),
                    ),
                    _ => (
                        Vec3::new(u, 0.25, cylinder.z_min - 1.0),
                        Vec3::new(0.0, 0.0, 1.0),
                    ),
                };
                let ray = Ray::new_normalized(origin, direction).expect("unit");
                for span in profile.intersect_ray(&ray).iter() {
                    assert!(
                        span.t0.is_finite() && span.t1.is_finite(),
                        "{name}: unbounded span {span}"
                    );
                    assert!(
                        span.t1 <= reach,
                        "{name}: span {span} runs past the bounding cylinder at {reach}"
                    );
                    assert!(span.length() > 0.0, "{name}: empty span {span}");
                }
            }
        }
    }
    assert!(total.rays > 0 && total.crossings > 0);
    eprintln!(
        "{} rays, {} crossings, {} collapsed, {} spans, {} grazes",
        total.rays, total.crossings, total.collapsed, total.spans, total.grazes
    );
}

#[test]
fn a_bundle_of_rays_recovers_the_analytic_volume() {
    // 96 x 96 rays along each of three axes, against twelve tools.
    const N: usize = 96;
    let mut worst = 0.0f64;
    let mut worst_case = String::new();

    for (name, profile) in corpus() {
        let exact = profile.volume();
        for axis in 0..3 {
            let (measured, _) = bundle_volume(&profile, axis, N);
            let error = (measured - exact).abs() / exact;
            if error > worst {
                worst = error;
                worst_case = format!("{name} along axis {axis}: {measured} vs {exact}");
            }
            assert!(
                error < 0.02,
                "{name} along axis {axis}: rays measured {measured}, closed form says {exact} \
                 ({:.3}% out)",
                error * 100.0
            );
        }
    }
    eprintln!(
        "worst bundle-versus-closed-form error: {:.4}% ({worst_case})",
        worst * 100.0
    );
}

#[test]
fn the_bundle_converges_as_the_rays_are_refined() {
    // A Riemann sum over a bundle of rays converges as the cells shrink. If it
    // does not, the spans are wrong in a way that no single-ray test would show.
    //
    // The bound asserted is the O(1/n) rate rather than "better than the
    // previous bundle". A Riemann sum over a curved silhouette is not monotone
    // in the refinement: where the cells happen to straddle the boundary
    // symmetrically the errors cancel and a coarse grid comes out luckily
    // accurate, then gets worse before it gets better. The rate is what is
    // guaranteed, so the rate is what is checked.
    let profile = bull_end_mill(10.0, 2.0, 20.0, &Shank::plain(8.0, 50.0)).expect("valid");
    let exact = profile.volume();
    let mut finest = f64::INFINITY;
    for n in [16usize, 32, 64, 128, 256] {
        let (measured, _) = bundle_volume(&profile, 0, n);
        let error = (measured - exact).abs() / exact;
        eprintln!(
            "n = {n:>4}: {measured:>12.4} vs {exact:>12.4}  ({:.4}% against a {:.4}% bound)",
            error * 100.0,
            200.0 / n as f64
        );
        assert!(
            error < 2.0 / n as f64,
            "at n = {n} the error is {error}, above the O(1/n) bound of {}",
            2.0 / n as f64
        );
        finest = error;
    }
    assert!(finest < 0.005, "the finest bundle is still {finest} out");
}

#[test]
fn a_tangent_ray_grazes_rather_than_removing_a_sliver() {
    // Exactly tangent to the side of a 6 mm cylinder.
    let profile = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    let ray =
        Ray::new_normalized(Vec3::new(-100.0, 3.0, 25.0), Vec3::new(1.0, 0.0, 0.0)).expect("unit");
    let mut scratch = RaycastScratch::new();
    let mut spans = chipbreaker_core::spans::Spans::new();
    let mut stats = RaycastStats::default();
    profile.intersect_ray_into(&ray, &mut scratch, &mut spans, &mut stats);
    assert!(
        spans.is_empty(),
        "a tangent ray removes no material, but reported {spans}"
    );
    assert!(
        stats.tangencies > 0,
        "the tangency must be visible in the statistics rather than silently \
         dropped: {stats:?}"
    );
}

#[test]
fn a_ray_starting_inside_the_tool_reports_the_span_from_where_it_starts() {
    let profile = flat_end_mill(6.0, 20.0, &Shank::plain(6.0, 50.0)).expect("valid");
    let ray =
        Ray::new_normalized(Vec3::new(0.0, 0.5, 25.0), Vec3::new(1.0, 0.0, 0.0)).expect("unit");
    let spans = profile.intersect_ray(&ray);
    assert_eq!(spans.len(), 1, "{spans}");
    let s = spans.as_slice()[0];
    assert!(
        s.t0.abs() < EPS_LENGTH,
        "the span must begin at the ray origin, not at the far wall: {s}"
    );
    assert!((s.t1 - (9.0f64 - 0.25).sqrt()).abs() < 1e-9, "{s}");
}
