// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Span storage for a whole field: packed inline, with a spill path.
//!
//! # The measurement that chose this design
//!
//! Run `cargo run --release -p chipbreaker-core --example span_distribution`.
//! Measured before this file was written, because the arena's justification *is*
//! the shape of the distribution and guessing it would be guessing at the
//! central data structure of the product:
//!
//! | mesh | 0 spans | 1 span | 2 spans | max |
//! |---|---:|---:|---:|---:|
//! | box, stock at rest | — | 100% | — | 1 |
//! | sphere | 21.8% | 78.2% | — | 1 |
//! | torus, axis along the bundle | 44.3% | 55.7% | — | 1 |
//! | nested shells (a cavity) | 21.8% | 58.6% | 19.6% | 2 |
//!
//! The distribution is not merely skewed, it is nearly degenerate: **stock at
//! rest is exactly one span on every ray**, and the only case that reaches two
//! is a genuine internal cavity. So this is not a general-purpose allocator and
//! must not become one. It is a flat array with [`INLINE_CAPACITY`] slots per
//! ray and a map for the rays that outgrow them.
//!
//! One measurement surprised: a torus whose axis lies *along* the bundle does
//! not produce two-span rays. Its hole appears as 44% **empty** rays. A through
//! hole gives two spans only when it runs *transverse* to the bundle. Worth
//! knowing before sizing anything against "holes give two spans".
//!
//! # Why not `Vec<Spans>`
//!
//! Unit 1 measured it: 24 bytes of header per ray plus one allocation each. A
//! 2000x2000 lattice is 4M rays, so roughly 96 MB of headers before a single
//! interval exists, and four million allocations whose addresses depend on
//! allocator state. The field hash must not depend on allocation history.
//!
//! # What U7 needs from this
//!
//! U7 subtracts the tool from these spans millions of times, and subtraction
//! **splits**: cutting a slot through a solid ray turns one span into two.
//! Growth is therefore the common mutation, not a rare one, which is why the
//! inline capacity is two rather than one — one would spill on the first pocket
//! cut into a block. Beyond two, [`Arena::spill`] takes over, and its `BTreeMap`
//! keeps iteration order deterministic where a `HashMap` would not.

use std::collections::BTreeMap;

use crate::golden::{CanonicalHash, Hashable};
use crate::spans::{Span, Spans};

/// Spans stored inline per ray before spilling.
///
/// Two, from the measurement in the module header: one covers every ray of
/// ordinary stock at rest but only 80.4% of a mesh with a cavity, and U7's
/// subtraction splits spans, so one would spill on the first pocket. Four would
/// double the resting footprint to buy a case that has not been observed.
pub const INLINE_CAPACITY: usize = 2;

/// Per-ray span storage for a field.
///
/// Layout is a pure function of the ray count: no allocation depends on the
/// order rays were filled, so the field hash cannot depend on allocation
/// history.
#[derive(Debug, Clone, PartialEq)]
pub struct Arena {
    /// `rays * INLINE_CAPACITY` slots, used when `len[ray] <= INLINE_CAPACITY`.
    inline: Vec<Span>,
    /// How many spans each ray carries, whether inline or spilled.
    ///
    /// `u16` rather than `u8`: a ray through a honeycomb or a lattice-work part
    /// can genuinely carry hundreds, and a silent wrap at 256 would be a field
    /// that looks fine and is wrong.
    len: Vec<u16>,
    /// Rays that outgrew the inline slots, holding **all** of their spans.
    ///
    /// A `BTreeMap` because this is iterated when hashing, and unordered
    /// iteration reaching a float is what the determinism rules forbid.
    spill: BTreeMap<u32, Vec<Span>>,
}

impl Arena {
    /// An arena for `rays` rays, every one empty.
    ///
    /// # Panics
    /// Panics if `rays` exceeds what a `u32` index can address, which is the
    /// limit the hashing and serialization rules impose.
    #[must_use]
    pub fn new(rays: usize) -> Self {
        assert!(
            u32::try_from(rays).is_ok(),
            "a field of {rays} rays cannot be addressed by a u32 index"
        );
        Self {
            inline: vec![Span::new(0.0, 0.0); rays * INLINE_CAPACITY],
            len: vec![0; rays],
            spill: BTreeMap::new(),
        }
    }

    /// How many rays.
    #[inline]
    #[must_use]
    pub fn rays(&self) -> usize {
        self.len.len()
    }

    /// True if there are no rays at all.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len.is_empty()
    }

    /// The spans of one ray.
    ///
    /// # Panics
    /// Panics if `ray` is out of range.
    #[must_use]
    pub fn get(&self, ray: u32) -> &[Span] {
        let index = ray as usize;
        assert!(index < self.len.len(), "ray {ray} out of range");
        let count = self.len[index] as usize;
        if count > INLINE_CAPACITY {
            return self.spill.get(&ray).map_or(&[], Vec::as_slice);
        }
        let base = index * INLINE_CAPACITY;
        &self.inline[base..base + count]
    }

    /// How many spans one ray carries.
    ///
    /// # Panics
    /// Panics if `ray` is out of range.
    #[inline]
    #[must_use]
    pub fn span_count(&self, ray: u32) -> usize {
        let index = ray as usize;
        assert!(index < self.len.len(), "ray {ray} out of range");
        self.len[index] as usize
    }

    /// Replaces one ray's spans.
    ///
    /// The only mutation, and deliberately so: U7 will subtract into a scratch
    /// [`Spans`] and store the result, which keeps the growth decision in one
    /// place rather than spread across an insert/remove/split API that each
    /// caller could get subtly wrong.
    ///
    /// # Panics
    /// Panics if `ray` is out of range, or if `spans` is longer than a `u16`.
    pub fn set(&mut self, ray: u32, spans: &[Span]) {
        let index = ray as usize;
        assert!(index < self.len.len(), "ray {ray} out of range");
        let count = u16::try_from(spans.len())
            .unwrap_or_else(|_| panic!("ray {ray} has {} spans, more than u16", spans.len()));

        if spans.len() > INLINE_CAPACITY {
            // Spilled. The inline slots are left as they are rather than
            // cleared: `get` never reads them while `len` exceeds the capacity,
            // and clearing them would be work that changes nothing observable.
            self.spill.insert(ray, spans.to_vec());
        } else {
            // Back within the inline slots, so any previous spill must go --
            // otherwise a ray that shrank would keep stale storage alive and
            // the arena would only ever grow.
            self.spill.remove(&ray);
            let base = index * INLINE_CAPACITY;
            self.inline[base..base + spans.len()].copy_from_slice(spans);
        }
        self.len[index] = count;
    }

    /// Copies one ray's spans into `out`, which is cleared first.
    ///
    /// The `_into` shape U7 needs: one scratch buffer for a whole sweep, no
    /// allocation after the first ray.
    pub fn read_into(&self, ray: u32, out: &mut Spans) {
        out.clear();
        for span in self.get(ray) {
            out.push_merge(*span);
        }
    }

    /// Total spans across every ray, summed in **ascending ray index**.
    ///
    /// The order is part of the contract. Integer addition is associative so it
    /// could not matter here, but [`Self::measure`] sums floats in the same
    /// order and the two must agree about what "traversal order" means.
    #[must_use]
    pub fn total_spans(&self) -> usize {
        self.len.iter().map(|n| *n as usize).sum()
    }

    /// Rays carrying at least one span.
    #[must_use]
    pub fn filled_rays(&self) -> usize {
        self.len.iter().filter(|n| **n > 0).count()
    }

    /// How many rays spilled past the inline capacity.
    ///
    /// The number that says whether [`INLINE_CAPACITY`] is still the right
    /// choice. If this stops being near zero on real work, revisit it against a
    /// fresh measurement rather than by intuition.
    #[must_use]
    pub fn spilled_rays(&self) -> usize {
        self.spill.len()
    }

    /// Span counts by ray, as a histogram from span count to ray count.
    #[must_use]
    pub fn distribution(&self) -> BTreeMap<usize, usize> {
        let mut out = BTreeMap::new();
        for count in &self.len {
            *out.entry(*count as usize).or_default() += 1;
        }
        out
    }

    /// Bytes this arena occupies, excluding the `Vec` headers themselves.
    #[must_use]
    pub fn bytes(&self) -> usize {
        self.inline.len() * size_of::<Span>()
            + self.len.len() * size_of::<u16>()
            + self
                .spill
                .values()
                .map(|v| v.len() * size_of::<Span>() + size_of::<u32>())
                .sum::<usize>()
    }
}

impl Hashable for Arena {
    /// Hashes the spans a ray *has*, never the slots it was given.
    ///
    /// Unused inline slots hold whatever was last written there, so feeding the
    /// raw backing array to the hash would make the digest depend on a ray's
    /// history rather than its contents — two fields with identical geometry
    /// would disagree because one had been cut and restored.
    fn hash_canonical(&self, h: &mut CanonicalHash) {
        h.begin("DexelArena");
        h.usize(self.rays());
        for ray in 0..self.rays() {
            let index = u32::try_from(ray).unwrap_or(u32::MAX);
            let spans = self.get(index);
            h.usize(spans.len());
            for span in spans {
                h.add(span);
            }
        }
        h.end();
    }
}
