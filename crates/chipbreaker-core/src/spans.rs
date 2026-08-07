// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Sorted, disjoint, normalized sets of half-open intervals on a line.
//!
//! This is the primitive the whole engine rests on. A dexel ray stores the
//! material along it as a [`Spans`]; a cut is [`Spans::subtract`]; a stock
//! intersection is [`Spans::intersect`]; a multi-setup union is
//! [`Spans::union`]. From U5 onward, essentially every operation Chipbreaker
//! performs bottoms out in this file, so it is worth reading carefully.
//!
//! # Half-open intervals
//!
//! Every span is `[t0, t1)`: `t0` is inside, `t1` is not. This makes abutting
//! spans tile the line without overlap — `[0, 1)` and `[1, 2)` cover `[0, 2)`
//! exactly once, with no shared endpoint to decide the ownership of. Closed
//! intervals would force an arbitrary tie-break at every boundary, and dexel
//! fields have a boundary at every ray crossing.
//!
//! # The invariant
//!
//! A [`Spans`] is always **structurally valid**:
//!
//! 1. Sorted ascending by `t0`.
//! 2. Every span has strictly positive, non-NaN length.
//! 3. Consecutive spans are separated by a gap strictly greater than
//!    [`EPS_SPAN_MERGE`] — so they are disjoint, and not merely touching.
//!
//! [`Spans::debug_check_invariant`] asserts this, and every public method that
//! returns or mutates a `Spans` calls it in debug builds.
//!
//! A **normalized** `Spans` additionally has no span shorter than
//! [`EPS_SPAN_MIN`]. Every set operation returns a normalized result;
//! [`Spans::push_merge`] does not, because a caller building a set incrementally
//! may legitimately push two sub-threshold pieces that together exceed it.
//! [`Spans::normalize`] restores the stronger property, and
//! [`Spans::is_normalized`] tests it.
//!
//! # Normalization order: merge first, then drop
//!
//! This order is forced, not arbitrary. Consider `[0, 10)`, `[10+g, 10+g+s)`,
//! `[10+g+s+g', ...)` where `g` and `g'` are below the merge threshold and `s`
//! is below the drop threshold. Dropping the sliver first leaves a combined gap
//! of `g + s + g'`, which may exceed the threshold, so the outer spans stay
//! separate. Merging first fuses all three. The two orders give different
//! answers, and only merge-first converges: after it, every gap already exceeds
//! the threshold, and removing a span only widens a gap, so a second pass
//! changes nothing. Normalization is therefore idempotent — a property the test
//! suite checks directly.
//!
//! # Tolerance and the algebraic laws
//!
//! The set-algebra identities (`(a - b) ∪ (a ∩ b) == a`, and friends) hold
//! **exactly** when every endpoint in play is separated from every other by more
//! than [`EPS_SPAN_MERGE`]. They do not hold in the sliver regime, and no
//! tolerance-based implementation can make them: an intersection thinner than
//! the drop threshold vanishes, and the difference then covers ground the
//! original did not. This is a deliberate, documented trade — the alternative is
//! accumulating sub-nanometre voids across millions of cuts. The property tests
//! generate endpoints on a grid far coarser than the tolerance so that they test
//! the algebra; the sliver behaviour is pinned separately by
//! `tolerance_behaviour_is_characterised`.

use core::fmt;

use crate::eps::{EPS_SPAN_MERGE, EPS_SPAN_MIN};
use crate::golden::{CanonicalHash, Hashable};
use crate::math::OctNormal;

/// A single half-open interval `[t0, t1)`, with the surface normal at each end.
///
/// # Why the normals live here and not beside here
///
/// They are attributes of the *endpoints*, and the endpoints are what the
/// boolean algebra creates and destroys. A cut is a subtraction, and subtraction
/// is precisely the operation that invents an endpoint that did not exist
/// before; only the merge-scan knows, at that moment, that the new endpoint lies
/// on the cutter rather than on the stock. A parallel array maintained outside
/// the scan would have to re-derive that afterwards by matching positions, which
/// is both slower and a guess.
///
/// So the scan carries them, and `Span` grows from 16 bytes to 24.
///
/// The normals take no part in ordering, validity, measure or merging. Two spans
/// with the same bounds and different normals are *not* `==`, which is
/// deliberate: it is what makes a golden hash notice a flipped cut face.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Span {
    /// Inclusive lower bound.
    pub t0: f64,
    /// Exclusive upper bound.
    pub t1: f64,
    /// Outward surface normal at `t0`. See [`OctNormal`] for the convention.
    pub n0: OctNormal,
    /// Outward surface normal at `t1`.
    pub n1: OctNormal,
}

impl Span {
    /// Constructs a span with no recorded normals.
    ///
    /// Both ends get [`OctNormal::PLACEHOLDER`]. Use this where the geometry is
    /// what matters and the orientation is genuinely unknown — interval algebra
    /// under test, clipping bounds, a legacy file. Use [`Span::with_normals`]
    /// wherever a real surface is being described, because a placeholder that
    /// reaches the extractor costs a sharp edge.
    ///
    /// Does not reorder: see [`Span::is_valid`].
    #[inline]
    #[must_use]
    pub const fn new(t0: f64, t1: f64) -> Self {
        Self {
            t0,
            t1,
            n0: OctNormal::PLACEHOLDER,
            n1: OctNormal::PLACEHOLDER,
        }
    }

    /// Constructs a span carrying the outward normal at each end.
    #[inline]
    #[must_use]
    pub const fn with_normals(t0: f64, t1: f64, n0: OctNormal, n1: OctNormal) -> Self {
        Self { t0, t1, n0, n1 }
    }

    /// The same interval with both normals replaced by the placeholder.
    ///
    /// What `extract --no-normals` uses to produce the surface-nets control.
    #[inline]
    #[must_use]
    pub const fn without_normals(self) -> Self {
        Self::new(self.t0, self.t1)
    }

    /// Constructs a span from two parameters in either order.
    #[inline]
    #[must_use]
    pub fn ordered(a: f64, b: f64) -> Self {
        if a <= b {
            Self::new(a, b)
        } else {
            Self::new(b, a)
        }
    }

    /// `t1 - t0`, which is negative for an invalid span.
    #[inline]
    #[must_use]
    pub fn length(self) -> f64 {
        self.t1 - self.t0
    }

    /// True if the span has strictly positive length and neither bound is NaN.
    ///
    /// `t1 > t0` is false whenever either bound is NaN, so this single
    /// comparison rejects both degeneracy and NaN.
    #[inline]
    #[must_use]
    pub fn is_valid(self) -> bool {
        self.t1 > self.t0
    }

    /// True if `t` lies in `[t0, t1)`.
    #[inline]
    #[must_use]
    pub fn contains(self, t: f64) -> bool {
        t >= self.t0 && t < self.t1
    }

    /// The midpoint, computed overflow-safely as `t0/2 + t1/2`.
    #[inline]
    #[must_use]
    pub fn midpoint(self) -> f64 {
        self.t0 / 2.0 + self.t1 / 2.0
    }

    /// Shifts both bounds by `d`.
    #[inline]
    #[must_use]
    pub fn translated(self, d: f64) -> Self {
        Self::with_normals(self.t0 + d, self.t1 + d, self.n0, self.n1)
    }
}

impl fmt::Display for Span {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}, {})", self.t0, self.t1)
    }
}

impl Hashable for Span {
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Span").f64(self.t0).f64(self.t1);
        // Hashed, so an inverted cut face moves the digest instead of producing
        // a mesh that validates and is inside out.
        self.n0.hash_canonical(h);
        self.n1.hash_canonical(h);
        h.end();
    }
}

// Boolean combination selectors for `boolean_scan`. A const generic rather than
// a closure or function pointer so that each operation monomorphizes into its
// own loop with the combination folded in — this is the hot loop of the product.
const OP_UNION: u8 = 0;
const OP_INTERSECT: u8 = 1;
const OP_DIFFERENCE: u8 = 2;

#[inline(always)]
fn combine<const OP: u8>(in_a: bool, in_b: bool) -> bool {
    match OP {
        OP_UNION => in_a || in_b,
        OP_INTERSECT => in_a && in_b,
        _ => in_a && !in_b,
    }
}

/// One merge-scan over two sorted, disjoint span lists.
///
/// Walks the two endpoint streams together, keeping the "inside a" / "inside b"
/// state, and emits a span for every maximal run where the combination is true.
/// One pass, no sorting, and one `push` per output span. `out` is cleared first
/// and is expected to be a caller-owned scratch buffer.
///
/// The output is sorted and non-overlapping but not yet normalized: a difference
/// can leave a gap or a span narrower than the tolerances. Callers run
/// [`normalize_in_place`] on the result.
fn boolean_scan<const OP: u8>(a: &[Span], b: &[Span], out: &mut Vec<Span>) {
    out.clear();

    let mut ia = 0usize;
    let mut ib = 0usize;
    let mut in_a = false;
    let mut in_b = false;
    let mut open_at = 0.0f64;
    let mut open_normal = OctNormal::PLACEHOLDER;
    let mut open = false;

    loop {
        // The next endpoint each stream will present. `None` means exhausted,
        // which can only happen while that stream's `in_` flag is false (the
        // index advances on exit, not on entry).
        let ea = if ia < a.len() {
            Some(if in_a { a[ia].t1 } else { a[ia].t0 })
        } else {
            None
        };
        let eb = if ib < b.len() {
            Some(if in_b { b[ib].t1 } else { b[ib].t0 })
        } else {
            None
        };

        // Early exit: an intersection cannot reopen once either stream is spent.
        if OP == OP_INTERSECT && (ea.is_none() || eb.is_none()) {
            debug_assert!(!open, "intersection open past the end of a stream");
            break;
        }

        let t = match (ea, eb) {
            (None, None) => break,
            (Some(x), None) => x,
            (None, Some(y)) => y,
            // Not `f64::min`: on a tie both streams must advance, and taking the
            // smaller of two equal values keeps both equality tests below true.
            (Some(x), Some(y)) => {
                if x <= y {
                    x
                } else {
                    y
                }
            }
        };

        // The normal to attach if this endpoint ends up on the output boundary.
        //
        // `a` wins a tie. `a` is the material and `b` the cutter, so when a cut
        // lands exactly on an existing face, the face that was already there is
        // the one that survives -- and, more to the point, the rule has to be
        // *some* fixed rule rather than whichever stream the loop happened to
        // examine first.
        //
        // A `b` endpoint is **negated**. Its stored normal points out of the
        // cutter, and the cutter's material sits on the side the workpiece's
        // does not, so out-of-the-remaining-material is the reverse. This one
        // sign is the whole of the convention in `OctNormal`, and getting it
        // backwards yields a watertight, manifold, inside-out mesh.
        let normal_here = if ea == Some(t) {
            Some(if in_a { a[ia].n1 } else { a[ia].n0 })
        } else if eb == Some(t) && OP == OP_DIFFERENCE {
            Some(if in_b { b[ib].n1 } else { b[ib].n0 }.negated())
        } else if eb == Some(t) {
            // Union and intersection do not reverse anything: both operands are
            // solids and their surfaces keep facing the way they faced.
            Some(if in_b { b[ib].n1 } else { b[ib].n0 })
        } else {
            None
        };

        if ea == Some(t) {
            if in_a {
                ia += 1;
            }
            in_a = !in_a;
        }
        if eb == Some(t) {
            if in_b {
                ib += 1;
            }
            in_b = !in_b;
        }

        let inside = combine::<OP>(in_a, in_b);
        if inside && !open {
            open_at = t;
            open_normal = normal_here.unwrap_or(OctNormal::PLACEHOLDER);
            open = true;
        } else if !inside && open {
            out.push(Span::with_normals(
                open_at,
                t,
                open_normal,
                normal_here.unwrap_or(OctNormal::PLACEHOLDER),
            ));
            open = false;
        }
    }

    debug_assert!(!open, "merge-scan finished with a span still open");
}

/// Sorts, merges, and drops slivers, restoring the full normalized form.
///
/// See the module documentation for why the order is merge-then-drop.
fn normalize_in_place(v: &mut Vec<Span>) {
    // Degenerate and NaN-bearing spans carry no information and would make the
    // sort order undefined.
    v.retain(|s| s.is_valid());
    if v.is_empty() {
        return;
    }

    // `total_cmp` gives a total order over all f64 including NaN, so the sort is
    // deterministic; `sort_by` is stable, so equal keys keep input order.
    v.sort_by(|x, y| x.t0.total_cmp(&y.t0).then_with(|| x.t1.total_cmp(&y.t1)));

    let mut write = 0usize;
    let mut current = v[0];
    for read in 1..v.len() {
        let next = v[read];
        if next.t0 - current.t1 <= EPS_SPAN_MERGE {
            // Overlapping or close enough to fuse. `max` because `next` may be
            // wholly contained in `current` -- in which case `current` keeps its
            // own far end, normal included.
            //
            // The normal must travel with the bound it belongs to. Moving `t1`
            // and leaving `n1` behind would leave the fused span describing its
            // far face with the normal of a face that is now in its interior,
            // which the extractor would then use to place a vertex.
            if next.t1 > current.t1 {
                current.t1 = next.t1;
                current.n1 = next.n1;
            }
        } else {
            // `current` is final. Keep it only if it survives the drop
            // threshold; discarding it only widens the gap around it, so the
            // invariant is preserved either way.
            if current.length() >= EPS_SPAN_MIN {
                v[write] = current;
                write += 1;
            }
            current = next;
        }
    }
    if current.length() >= EPS_SPAN_MIN {
        v[write] = current;
        write += 1;
    }
    v.truncate(write);
}

/// A sorted, disjoint, normalized set of half-open intervals on a line.
///
/// See the module documentation for the invariant and the tolerance policy.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Spans {
    /// Invariant: sorted by `t0`, every span valid, gaps > [`EPS_SPAN_MERGE`].
    spans: Vec<Span>,
}

impl Spans {
    /// The empty set.
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self { spans: Vec::new() }
    }

    /// The empty set, with room for `n` spans.
    #[inline]
    #[must_use]
    pub fn with_capacity(n: usize) -> Self {
        Self {
            spans: Vec::with_capacity(n),
        }
    }

    /// The set containing exactly `span`, or the empty set if `span` is invalid
    /// or shorter than [`EPS_SPAN_MIN`].
    #[must_use]
    pub fn from_span(span: Span) -> Self {
        if span.is_valid() && span.length() >= EPS_SPAN_MIN {
            Self { spans: vec![span] }
        } else {
            Self::new()
        }
    }

    /// Builds a set from arbitrary spans, in any order, possibly overlapping.
    #[must_use]
    pub fn from_unsorted(mut spans: Vec<Span>) -> Self {
        normalize_in_place(&mut spans);
        let out = Self { spans };
        out.debug_check_invariant();
        out
    }

    /// The spans, in ascending order.
    #[inline]
    #[must_use]
    pub fn as_slice(&self) -> &[Span] {
        &self.spans
    }

    /// Iterates the spans in ascending order.
    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, Span> {
        self.spans.iter()
    }

    /// The number of spans.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.spans.len()
    }

    /// True if the set contains no spans.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.spans.is_empty()
    }

    /// Removes every span, keeping the allocation for reuse.
    #[inline]
    pub fn clear(&mut self) {
        self.spans.clear();
    }

    /// The smallest span containing all of them, or `None` when empty.
    #[inline]
    #[must_use]
    pub fn hull(&self) -> Option<Span> {
        match (self.spans.first(), self.spans.last()) {
            (Some(f), Some(l)) => Some(Span::new(f.t0, l.t1)),
            _ => None,
        }
    }

    /// Total length of all spans.
    ///
    /// Summed in **ascending `t0` order**, which is storage order. The order is
    /// part of the contract: floating-point addition is not associative, so a
    /// different traversal gives a different last bit, and this value feeds the
    /// removed-material totals that U12 hashes.
    #[must_use]
    pub fn measure(&self) -> f64 {
        let mut total = 0.0;
        for s in &self.spans {
            total += s.length();
        }
        total
    }

    /// True if `t` lies inside any span.
    ///
    /// Binary search: `O(log n)`.
    #[must_use]
    pub fn contains(&self, t: f64) -> bool {
        // Index of the first span starting after `t`; the only candidate is the
        // one before it.
        let idx = self.spans.partition_point(|s| s.t0 <= t);
        idx > 0 && self.spans[idx - 1].contains(t)
    }

    /// Adds a span, merging it into the set.
    ///
    /// The **hot path** is appending at or after the current maximum `t0`, which
    /// is what a front-to-back sweep along a dexel ray does; that case is `O(1)`
    /// and allocation-free apart from the amortised push. Inserting before the
    /// end falls back to a full re-normalization, `O(n log n)`; if a caller
    /// finds itself doing that repeatedly it should collect into a `Vec<Span>`
    /// and use [`Spans::from_unsorted`] once.
    ///
    /// Invalid (non-positive-length or NaN) spans are ignored. Sub-threshold
    /// spans are **not** dropped here — see the module documentation on
    /// structural validity versus normalization.
    pub fn push_merge(&mut self, span: Span) {
        if !span.is_valid() {
            return;
        }
        match self.spans.last_mut() {
            None => self.spans.push(span),
            Some(last) if span.t0 >= last.t0 => {
                if span.t0 - last.t1 <= EPS_SPAN_MERGE {
                    // Fuse. `last` can only grow to the right, so its gap to the
                    // span before it is unchanged and the invariant holds.
                    // The normal travels with the bound; see
                    // `normalize_in_place`.
                    if span.t1 > last.t1 {
                        last.t1 = span.t1;
                        last.n1 = span.n1;
                    }
                } else {
                    self.spans.push(span);
                }
            }
            Some(_) => {
                // Out-of-order insert: the slow path.
                self.spans.push(span);
                normalize_in_place(&mut self.spans);
            }
        }
        self.debug_check_invariant();
    }

    /// Restores the fully normalized form: sorts, merges, and drops spans
    /// shorter than [`EPS_SPAN_MIN`].
    ///
    /// Idempotent — see the module documentation.
    pub fn normalize(&mut self) {
        normalize_in_place(&mut self.spans);
        self.debug_check_invariant();
    }

    /// True if no span is shorter than [`EPS_SPAN_MIN`], in addition to the
    /// structural invariant.
    #[must_use]
    pub fn is_normalized(&self) -> bool {
        self.check_invariant().is_ok() && self.spans.iter().all(|s| s.length() >= EPS_SPAN_MIN)
    }

    /// Checks the structural invariant, naming the first violation.
    ///
    /// # Errors
    /// Returns a human-readable description of the first span that breaks
    /// sortedness, validity, or disjointness.
    pub fn check_invariant(&self) -> Result<(), String> {
        for (i, s) in self.spans.iter().enumerate() {
            if !s.is_valid() {
                return Err(format!("span {i} is degenerate or NaN: {s}"));
            }
            if i > 0 {
                let prev = self.spans[i - 1];
                if s.t0 < prev.t0 {
                    return Err(format!("span {i} {s} starts before span {} {prev}", i - 1));
                }
                // Written as an explicit NaN test plus `<=` rather than
                // `!(gap > EPS)`: this is a checker, so it must cope with a
                // corrupt set in which both bounds are infinite and the
                // subtraction yields NaN.
                let gap = s.t0 - prev.t1;
                if gap.is_nan() || gap <= EPS_SPAN_MERGE {
                    return Err(format!(
                        "spans {} {prev} and {i} {s} are separated by {gap}, \
                         which is not greater than EPS_SPAN_MERGE ({EPS_SPAN_MERGE})",
                        i - 1
                    ));
                }
            }
        }
        Ok(())
    }

    /// Asserts the structural invariant in debug builds; a no-op in release.
    ///
    /// # Panics
    /// In debug builds, panics with the offending span if the invariant is
    /// broken. This is a bug in this module, never in the caller.
    #[inline]
    pub fn debug_check_invariant(&self) {
        // `cfg` rather than `debug_assert!`, so the check runs exactly once
        // rather than once to test and once to build the message.
        #[cfg(debug_assertions)]
        if let Err(e) = self.check_invariant() {
            panic!("Spans invariant violated: {e}\nset = {self}");
        }
    }

    /// Union: everything in either set.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        let mut out = Self::new();
        self.union_into(other, &mut out);
        out
    }

    /// Intersection: everything in both sets.
    #[must_use]
    pub fn intersect(&self, other: &Self) -> Self {
        let mut out = Self::new();
        self.intersect_into(other, &mut out);
        out
    }

    /// Difference: everything in `self` that is not in `other`.
    ///
    /// This is a cut. It is the single most-executed operation in Chipbreaker.
    #[must_use]
    pub fn subtract(&self, other: &Self) -> Self {
        let mut out = Self::new();
        self.subtract_into(other, &mut out);
        out
    }

    /// [`Spans::union`], reusing `out`'s allocation.
    pub fn union_into(&self, other: &Self, out: &mut Self) {
        self.op_into::<OP_UNION>(other, out);
    }

    /// [`Spans::intersect`], reusing `out`'s allocation.
    pub fn intersect_into(&self, other: &Self, out: &mut Self) {
        self.op_into::<OP_INTERSECT>(other, out);
    }

    /// [`Spans::subtract`], reusing `out`'s allocation.
    ///
    /// The hot path for material removal: a caller sweeping a toolpath keeps one
    /// scratch `Spans` per ray and never allocates after the first cut.
    pub fn subtract_into(&self, other: &Self, out: &mut Self) {
        self.op_into::<OP_DIFFERENCE>(other, out);
    }

    fn op_into<const OP: u8>(&self, other: &Self, out: &mut Self) {
        boolean_scan::<OP>(&self.spans, &other.spans, &mut out.spans);
        normalize_in_place(&mut out.spans);
        out.debug_check_invariant();
    }

    /// The part of `bounds` not covered by `self`.
    ///
    /// The complement is taken *within* an explicit window because the
    /// complement of a bounded set on the whole real line is unbounded, and an
    /// unbounded span has no useful measure. `bounds` is the stock envelope in
    /// U5 and the dexel extent thereafter.
    #[must_use]
    pub fn complement_within(&self, bounds: Span) -> Self {
        let mut out = Self::new();
        self.complement_within_into(bounds, &mut out);
        out
    }

    /// [`Spans::complement_within`], reusing `out`'s allocation.
    pub fn complement_within_into(&self, bounds: Span, out: &mut Self) {
        if !bounds.is_valid() {
            out.clear();
            return;
        }
        boolean_scan::<OP_DIFFERENCE>(&[bounds], &self.spans, &mut out.spans);
        normalize_in_place(&mut out.spans);
        out.debug_check_invariant();
    }

    /// The spans of `self` clipped to `bounds`.
    #[must_use]
    pub fn clipped_to(&self, bounds: Span) -> Self {
        self.intersect(&Self::from_span(bounds))
    }
}

impl FromIterator<Span> for Spans {
    /// Collects arbitrary spans, normalizing them.
    fn from_iter<I: IntoIterator<Item = Span>>(iter: I) -> Self {
        Self::from_unsorted(iter.into_iter().collect())
    }
}

impl<'a> IntoIterator for &'a Spans {
    type Item = &'a Span;
    type IntoIter = core::slice::Iter<'a, Span>;

    fn into_iter(self) -> Self::IntoIter {
        self.spans.iter()
    }
}

impl fmt::Display for Spans {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("{")?;
        for (i, s) in self.spans.iter().enumerate() {
            if i > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{s}")?;
        }
        f.write_str("}")
    }
}

impl Hashable for Spans {
    /// Hashes the spans in ascending order, length-prefixed.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("Spans");
        h.add_all(self.spans.iter());
        h.end();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    /// A span whose ends carry recognisable directions.
    fn sn(t0: f64, t1: f64, a: Vec3, b: Vec3) -> Span {
        Span::with_normals(t0, t1, OctNormal::encode(a), OctNormal::encode(b))
    }

    fn dir(n: OctNormal) -> Vec3 {
        n.decode()
    }

    /// Roughly equal directions, to within the encoding's own resolution.
    fn same_dir(a: Vec3, b: Vec3) -> bool {
        a.x * b.x + a.y * b.y + a.z * b.z > 0.999
    }

    #[test]
    fn subtracting_puts_the_cutter_normal_on_the_new_faces_reversed() {
        // **The sign convention, which is the one that produces an inside-out
        // mesh if it is wrong.**
        //
        // Material occupies [0, 10) along the ray, with its own outward normals
        // pointing away from it: -Z at the near end, +Z at the far end. A cutter
        // occupies [4, 6), with ITS outward normals pointing away from IT: -Z at
        // 4, +Z at 6.
        //
        // The result is [0, 4) and [6, 10). The face at 4 is new, lies on the
        // cutter, and must point +Z -- out of the remaining material at 4, which
        // is the OPPOSITE of the cutter's own -Z there. Likewise at 6.
        let material = sn(
            0.0,
            10.0,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        let cutter = sn(
            4.0,
            6.0,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        );

        let out = Spans::from_span(material).subtract(&Spans::from_span(cutter));
        assert_eq!(pairs(&out), [(0.0, 4.0), (6.0, 10.0)]);
        let v = out.as_slice();

        assert!(
            same_dir(dir(v[0].n0), Vec3::new(0.0, 0.0, -1.0)),
            "the untouched near face kept its own normal, got {:?}",
            dir(v[0].n0)
        );
        assert!(
            same_dir(dir(v[0].n1), Vec3::new(0.0, 0.0, 1.0)),
            "the cut face at 4 must point OUT of the material that remains below              it, i.e. +Z -- the reverse of the cutter's -Z there. Got {:?}. If              this is -Z the whole mesh will be inside out on cut faces only.",
            dir(v[0].n1)
        );
        assert!(
            same_dir(dir(v[1].n0), Vec3::new(0.0, 0.0, -1.0)),
            "the cut face at 6 must point -Z, got {:?}",
            dir(v[1].n0)
        );
        assert!(
            same_dir(dir(v[1].n1), Vec3::new(0.0, 0.0, 1.0)),
            "the untouched far face kept its own normal, got {:?}",
            dir(v[1].n1)
        );
    }

    #[test]
    fn an_untouched_span_keeps_its_normals_exactly() {
        // Not merely close: a cut elsewhere on the ray must not perturb a face
        // it never reached, or a thousand cuts would drift the normals the way
        // Unit 7 proved the positions do not.
        let a = sn(
            0.0,
            2.0,
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.3, 0.5, 0.8),
        );
        let far = Spans::from_span(Span::new(50.0, 60.0));
        let out = Spans::from_span(a).subtract(&far);
        assert_eq!(
            out.as_slice(),
            &[a],
            "an untouched span must survive bit for bit"
        );
    }

    #[test]
    fn fusing_two_spans_takes_the_far_normal_from_the_far_span() {
        // The merge path. Fusing [0,4) with [4,8) must describe the result's far
        // face with the SECOND span's normal, not the first's -- the first's
        // face is now interior and does not exist.
        let a = sn(
            0.0,
            4.0,
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        let b = sn(
            4.0,
            8.0,
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        let out = Spans::from_unsorted(vec![a, b]);
        assert_eq!(pairs(&out), [(0.0, 8.0)]);
        assert!(
            same_dir(dir(out.as_slice()[0].n1), Vec3::new(0.0, 1.0, 0.0)),
            "fused span kept the interior face's normal, got {:?}",
            dir(out.as_slice()[0].n1)
        );
        assert!(
            same_dir(dir(out.as_slice()[0].n0), Vec3::new(-1.0, 0.0, 0.0)),
            "fused span lost its near normal"
        );
    }

    #[test]
    fn normals_do_not_affect_the_geometry_of_any_operation() {
        // The guarantee that keeps the Unit 1 property tests meaningful: the
        // algebra must be blind to the attribute it carries.
        let bare = |x: &Spans| -> Vec<(f64, f64)> { pairs(x) };
        let a_plain = s(&[(0.0, 4.0), (6.0, 10.0)]);
        let b_plain = s(&[(2.0, 7.0)]);
        let a_norm = Spans::from_unsorted(vec![
            sn(
                0.0,
                4.0,
                Vec3::new(1.0, 2.0, 3.0),
                Vec3::new(-3.0, 1.0, 0.5),
            ),
            sn(
                6.0,
                10.0,
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.2, 0.2, -1.0),
            ),
        ]);
        let b_norm = Spans::from_unsorted(vec![sn(
            2.0,
            7.0,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]);
        assert_eq!(bare(&a_plain.union(&b_plain)), bare(&a_norm.union(&b_norm)));
        assert_eq!(
            bare(&a_plain.intersect(&b_plain)),
            bare(&a_norm.intersect(&b_norm))
        );
        assert_eq!(
            bare(&a_plain.subtract(&b_plain)),
            bare(&a_norm.subtract(&b_norm))
        );
        assert_eq!(a_plain.measure(), a_norm.measure());
    }

    #[test]
    fn a_span_differing_only_in_its_normals_is_not_equal() {
        // Deliberate: it is what lets the golden hash catch a flipped cut face.
        let up = sn(
            0.0,
            1.0,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        let down = sn(
            0.0,
            1.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
        );
        assert_ne!(up, down);
        let (mut h1, mut h2) = (CanonicalHash::new(), CanonicalHash::new());
        up.hash_canonical(&mut h1);
        down.hash_canonical(&mut h2);
        assert_ne!(
            h1.finish(),
            h2.finish(),
            "an inverted face must move the digest"
        );
    }

    /// Builds a `Spans` from `[t0, t1]` pairs, asserting it is already in
    /// canonical form so the test data itself cannot hide a normalization bug.
    fn s(pairs: &[(f64, f64)]) -> Spans {
        let v: Vec<Span> = pairs.iter().map(|&(a, b)| Span::new(a, b)).collect();
        let out = Spans::from_unsorted(v.clone());
        assert_eq!(
            out.as_slice(),
            v.as_slice(),
            "test literal is not already normalized"
        );
        out
    }

    fn pairs(x: &Spans) -> Vec<(f64, f64)> {
        x.iter().map(|s| (s.t0, s.t1)).collect()
    }

    #[test]
    fn span_basics() {
        let a = Span::new(1.0, 3.0);
        assert_eq!(a.length(), 2.0);
        assert!(a.is_valid());
        assert!(a.contains(1.0), "lower bound is inclusive");
        assert!(!a.contains(3.0), "upper bound is exclusive");
        assert!(!a.contains(0.9));
        assert_eq!(a.midpoint(), 2.0);
        assert_eq!(a.translated(1.0), Span::new(2.0, 4.0));
        assert_eq!(Span::ordered(3.0, 1.0), a);
        assert_eq!(format!("{a}"), "[1, 3)");

        assert!(!Span::new(1.0, 1.0).is_valid(), "zero length");
        assert!(!Span::new(3.0, 1.0).is_valid(), "inverted");
        assert!(!Span::new(f64::NAN, 1.0).is_valid(), "NaN");
        assert!(!Span::new(0.0, f64::NAN).is_valid(), "NaN");
    }

    #[test]
    fn midpoint_does_not_overflow() {
        let huge = Span::new(-f64::MAX, f64::MAX);
        assert_eq!(huge.midpoint(), 0.0);
    }

    #[test]
    fn construction_normalizes() {
        // Unsorted, overlapping, touching, degenerate and NaN input.
        let raw = vec![
            Span::new(5.0, 7.0),
            Span::new(0.0, 2.0),
            Span::new(1.0, 3.0),
            Span::new(3.0, 4.0),
            Span::new(9.0, 9.0),
            Span::new(f64::NAN, 1.0),
        ];
        let x = Spans::from_unsorted(raw);
        assert_eq!(pairs(&x), [(0.0, 4.0), (5.0, 7.0)]);
        assert!(x.is_normalized());

        assert!(Spans::new().is_empty());
        assert_eq!(Spans::from_span(Span::new(2.0, 1.0)), Spans::new());
        assert_eq!(pairs(&Spans::from_span(Span::new(1.0, 2.0))), [(1.0, 2.0)]);
    }

    #[test]
    fn union_merges_and_orders() {
        let a = s(&[(0.0, 1.0), (4.0, 5.0)]);
        let b = s(&[(0.5, 4.5)]);
        assert_eq!(pairs(&a.union(&b)), [(0.0, 5.0)]);

        let c = s(&[(10.0, 11.0)]);
        assert_eq!(pairs(&a.union(&c)), [(0.0, 1.0), (4.0, 5.0), (10.0, 11.0)]);
        assert_eq!(a.union(&Spans::new()), a);
        assert_eq!(Spans::new().union(&a), a);
        // Abutting spans fuse: half-open intervals tile without a seam.
        assert_eq!(
            pairs(&s(&[(0.0, 1.0)]).union(&s(&[(1.0, 2.0)]))),
            [(0.0, 2.0)]
        );
    }

    #[test]
    fn intersect_keeps_only_overlap() {
        let a = s(&[(0.0, 10.0)]);
        let b = s(&[(-5.0, 2.0), (4.0, 6.0), (8.0, 20.0)]);
        assert_eq!(
            pairs(&a.intersect(&b)),
            [(0.0, 2.0), (4.0, 6.0), (8.0, 10.0)]
        );
        assert!(a.intersect(&Spans::new()).is_empty());
        assert!(Spans::new().intersect(&a).is_empty());
        // Abutting sets share nothing: [0,1) and [1,2) are disjoint.
        assert!(s(&[(0.0, 1.0)]).intersect(&s(&[(1.0, 2.0)])).is_empty());
    }

    #[test]
    fn subtract_removes_material() {
        let stock = s(&[(0.0, 10.0)]);
        let cut = s(&[(2.0, 3.0), (7.0, 20.0)]);
        assert_eq!(pairs(&stock.subtract(&cut)), [(0.0, 2.0), (3.0, 7.0)]);
        assert_eq!(stock.subtract(&Spans::new()), stock);
        assert!(stock.subtract(&stock).is_empty());
        assert!(Spans::new().subtract(&stock).is_empty());
        // A cut strictly inside splits one span into two.
        assert_eq!(
            pairs(&s(&[(0.0, 10.0)]).subtract(&s(&[(4.0, 6.0)]))),
            [(0.0, 4.0), (6.0, 10.0)]
        );
    }

    #[test]
    fn complement_within_clips_to_bounds() {
        let a = s(&[(2.0, 4.0), (6.0, 8.0)]);
        assert_eq!(
            pairs(&a.complement_within(Span::new(0.0, 10.0))),
            [(0.0, 2.0), (4.0, 6.0), (8.0, 10.0)]
        );
        // Bounds narrower than the set.
        assert_eq!(
            pairs(&a.complement_within(Span::new(3.0, 7.0))),
            [(4.0, 6.0)]
        );
        // Empty set complements to the whole window.
        assert_eq!(
            pairs(&Spans::new().complement_within(Span::new(0.0, 1.0))),
            [(0.0, 1.0)]
        );
        // Invalid bounds give the empty set rather than a panic.
        assert!(a.complement_within(Span::new(5.0, 5.0)).is_empty());
        assert!(a.complement_within(Span::new(5.0, 1.0)).is_empty());
    }

    #[test]
    fn complement_within_is_an_involution_on_subsets() {
        let bounds = Span::new(0.0, 10.0);
        for a in [
            s(&[(2.0, 4.0), (6.0, 8.0)]),
            s(&[(0.0, 10.0)]),
            s(&[(0.0, 3.0)]),
            s(&[(7.0, 10.0)]),
            Spans::new(),
        ] {
            let round_tripped = a.complement_within(bounds).complement_within(bounds);
            assert_eq!(round_tripped, a, "double complement of {a}");
        }
    }

    #[test]
    fn measure_sums_in_ascending_order() {
        let a = s(&[(0.0, 1.0), (4.0, 5.5), (10.0, 10.25)]);
        assert_eq!(a.measure(), 2.75);
        assert_eq!(Spans::new().measure(), 0.0);
        // Inclusion-exclusion, on grid-aligned data where it holds exactly.
        let b = s(&[(0.5, 4.5)]);
        assert_eq!(
            a.union(&b).measure() + a.intersect(&b).measure(),
            a.measure() + b.measure()
        );
    }

    #[test]
    fn contains_respects_half_open_bounds() {
        let a = s(&[(0.0, 1.0), (4.0, 5.0)]);
        assert!(a.contains(0.0));
        assert!(a.contains(0.5));
        assert!(!a.contains(1.0), "upper bound excluded");
        assert!(!a.contains(2.0));
        assert!(a.contains(4.999));
        assert!(!a.contains(-1.0));
        assert!(!a.contains(f64::NAN));
        assert!(!Spans::new().contains(0.0));
    }

    #[test]
    fn push_merge_appends_in_the_hot_path() {
        let mut x = Spans::new();
        x.push_merge(Span::new(0.0, 1.0));
        x.push_merge(Span::new(2.0, 3.0));
        // Overlaps the tail: extends it.
        x.push_merge(Span::new(2.5, 4.0));
        // Abuts the tail: fuses.
        x.push_merge(Span::new(4.0, 5.0));
        // Wholly inside the tail: no change.
        x.push_merge(Span::new(4.2, 4.3));
        assert_eq!(pairs(&x), [(0.0, 1.0), (2.0, 5.0)]);
        // Invalid spans are ignored.
        x.push_merge(Span::new(9.0, 9.0));
        x.push_merge(Span::new(f64::NAN, 1.0));
        assert_eq!(pairs(&x), [(0.0, 1.0), (2.0, 5.0)]);
    }

    #[test]
    fn push_merge_handles_the_out_of_order_slow_path() {
        let mut x = Spans::new();
        x.push_merge(Span::new(5.0, 6.0));
        x.push_merge(Span::new(0.0, 1.0));
        x.push_merge(Span::new(0.5, 5.5));
        assert_eq!(pairs(&x), [(0.0, 6.0)]);
        assert!(x.check_invariant().is_ok());
    }

    #[test]
    fn normalize_is_idempotent() {
        // Two slivers separated by sub-threshold gaps: merge-first fuses all
        // three into something that survives; drop-first would not.
        let g = EPS_SPAN_MERGE / 2.0;
        let raw = vec![
            Span::new(0.0, 1.0),
            Span::new(1.0 + g, 1.0 + g + EPS_SPAN_MIN / 4.0),
            Span::new(1.0 + 2.0 * g + EPS_SPAN_MIN / 4.0, 3.0),
        ];
        let mut x = Spans::from_unsorted(raw);
        assert_eq!(x.len(), 1, "merge-first must fuse across the sliver");
        let once = x.clone();
        x.normalize();
        assert_eq!(x, once, "normalize must be idempotent");
        assert!(x.is_normalized());
    }

    #[test]
    fn tolerance_behaviour_is_characterised() {
        // These are the documented consequences of the epsilon policy, pinned so
        // that a change to the thresholds is a deliberate act with a visible
        // test diff rather than a silent behavioural shift.

        // A gap narrower than EPS_SPAN_MERGE is closed.
        let merged = Spans::from_unsorted(vec![
            Span::new(0.0, 1.0),
            Span::new(1.0 + EPS_SPAN_MERGE / 2.0, 2.0),
        ]);
        assert_eq!(pairs(&merged), [(0.0, 2.0)]);

        // A gap wider than EPS_SPAN_MERGE is kept.
        let kept = Spans::from_unsorted(vec![
            Span::new(0.0, 1.0),
            Span::new(1.0 + EPS_SPAN_MERGE * 10.0, 2.0),
        ]);
        assert_eq!(kept.len(), 2);

        // A span shorter than EPS_SPAN_MIN, isolated, is dropped entirely.
        let dropped = Spans::from_unsorted(vec![
            Span::new(0.0, 1.0),
            Span::new(5.0, 5.0 + EPS_SPAN_MIN / 2.0),
        ]);
        assert_eq!(pairs(&dropped), [(0.0, 1.0)]);

        // A sub-threshold cut is absorbed. There are two routes to that, and
        // both matter:
        //
        // 1. Through a normalized cut set, the sliver never survives
        //    construction, so the subtraction is a no-op on an empty set.
        let a = s(&[(0.0, 10.0)]);
        let normalized_sliver = Spans::from_span(Span::new(5.0, 5.0 + EPS_SPAN_MERGE / 2.0));
        assert!(normalized_sliver.is_empty(), "the cut set itself vanishes");
        assert_eq!(pairs(&a.subtract(&normalized_sliver)), [(0.0, 10.0)]);

        // 2. Through `push_merge`, which by design does not drop slivers, the
        //    cut does reach the merge-scan — and the hole it opens is narrower
        //    than EPS_SPAN_MERGE, so normalization closes it again.
        let mut raw_sliver = Spans::new();
        raw_sliver.push_merge(Span::new(5.0, 5.0 + EPS_SPAN_MERGE / 2.0));
        assert_eq!(raw_sliver.len(), 1, "push_merge keeps sub-threshold spans");
        assert!(!raw_sliver.is_normalized());
        assert_eq!(
            pairs(&a.subtract(&raw_sliver)),
            [(0.0, 10.0)],
            "sub-tolerance cuts are absorbed; this is the documented trade"
        );

        // The useful consequence of EPS_SPAN_MIN == EPS_SPAN_MERGE: an interior
        // cut either vanishes completely or splits the span cleanly. There is no
        // middle regime where a cut is large enough to survive normalization but
        // too small to open a surviving gap.
        let just_big_enough = Spans::from_span(Span::new(5.0, 5.0 + EPS_SPAN_MIN * 2.0));
        assert_eq!(just_big_enough.len(), 1);
        assert_eq!(a.subtract(&just_big_enough).len(), 2, "clean split");
    }

    #[test]
    fn invariant_checker_names_violations() {
        assert!(Spans::new().check_invariant().is_ok());
        // The checker inspects private state, so construct through the private
        // field, which this test module can reach.
        let bad_order = Spans {
            spans: vec![Span::new(5.0, 6.0), Span::new(0.0, 1.0)],
        };
        assert!(
            bad_order
                .check_invariant()
                .expect_err("out of order")
                .contains("starts before")
        );

        let touching = Spans {
            spans: vec![Span::new(0.0, 1.0), Span::new(1.0, 2.0)],
        };
        assert!(
            touching
                .check_invariant()
                .expect_err("zero gap")
                .contains("EPS_SPAN_MERGE")
        );

        let degenerate = Spans {
            spans: vec![Span::new(1.0, 1.0)],
        };
        assert!(
            degenerate
                .check_invariant()
                .expect_err("zero length")
                .contains("degenerate")
        );
    }

    #[test]
    fn scratch_buffer_variants_match_allocating_ones() {
        let a = s(&[(0.0, 3.0), (5.0, 9.0)]);
        let b = s(&[(2.0, 6.0)]);
        let mut out = Spans::with_capacity(8);

        a.union_into(&b, &mut out);
        assert_eq!(out, a.union(&b));
        a.intersect_into(&b, &mut out);
        assert_eq!(out, a.intersect(&b));
        a.subtract_into(&b, &mut out);
        assert_eq!(out, a.subtract(&b));
        a.complement_within_into(Span::new(0.0, 10.0), &mut out);
        assert_eq!(out, a.complement_within(Span::new(0.0, 10.0)));

        // Reuse must not leave stale spans behind.
        Spans::new().union_into(&Spans::new(), &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn helpers() {
        let a = s(&[(0.0, 1.0), (4.0, 5.0)]);
        assert_eq!(a.len(), 2);
        assert_eq!(a.hull(), Some(Span::new(0.0, 5.0)));
        assert_eq!(Spans::new().hull(), None);
        assert_eq!(
            pairs(&a.clipped_to(Span::new(0.5, 4.5))),
            [(0.5, 1.0), (4.0, 4.5)]
        );
        assert_eq!(a.iter().count(), 2);
        assert_eq!((&a).into_iter().count(), 2);
        assert_eq!(format!("{a}"), "{[0, 1), [4, 5)}");

        let collected: Spans = vec![Span::new(4.0, 5.0), Span::new(0.0, 1.0)]
            .into_iter()
            .collect();
        assert_eq!(collected, a);

        let mut c = a.clone();
        c.clear();
        assert!(c.is_empty());
    }

    #[test]
    fn hashing_reflects_content_not_history() {
        let built_at_once = s(&[(0.0, 2.0)]);
        let mut built_piecewise = Spans::new();
        built_piecewise.push_merge(Span::new(0.0, 1.0));
        built_piecewise.push_merge(Span::new(1.0, 2.0));
        assert_eq!(
            built_at_once.canonical_digest(),
            built_piecewise.canonical_digest(),
            "equal sets must hash equally regardless of construction order"
        );
        assert_ne!(
            built_at_once.canonical_digest(),
            s(&[(0.0, 1.0), (1.5, 2.0)]).canonical_digest()
        );
        assert_ne!(
            Spans::new().canonical_digest(),
            built_at_once.canonical_digest()
        );
    }

    #[test]
    fn empty_and_unbounded_edge_cases() {
        let e = Spans::new();
        assert_eq!(e.union(&e), e);
        assert_eq!(e.intersect(&e), e);
        assert_eq!(e.subtract(&e), e);
        assert_eq!(e.measure(), 0.0);
        assert_eq!(e.hull(), None);

        // Infinite bounds are legitimate: a complement within an unbounded
        // window is how U5 asks "where is there no material at all".
        let a = s(&[(0.0, 1.0)]);
        let all = Span::new(f64::NEG_INFINITY, f64::INFINITY);
        let comp = a.complement_within(all);
        assert_eq!(comp.len(), 2);
        assert!(comp.contains(-1e300));
        assert!(comp.contains(1e300));
        assert!(!comp.contains(0.5));
    }
}
