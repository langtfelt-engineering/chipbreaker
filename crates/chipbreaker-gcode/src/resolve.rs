// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Chipbreaker Contributors

//! Stage four: programmed words into machine coordinates, and motion.
//!
//! # The pipeline, in order
//!
//! Every coordinate in the IR comes out of this, and the order is the contract:
//!
//! ```text
//! programmed word
//!   -> units                  (G20/G21 factor, applied exactly once)
//!   -> G90/G91                (a position, or a delta from where we are)
//!   -> active work offset     (G54-G59.3)
//!   -> G92 shift, if active
//!   -> tool length offset     (G43/G44 H, against the U3 tool library)
//!   -> machine coordinates     <- what the IR stores
//! ```
//!
//! **Incremental mode skips most of it, and that is not a shortcut.** A delta is
//! a delta in every frame: rotating or translating a coordinate system does not
//! change a displacement. So `G91 X10.` adds ten millimetres to the current
//! machine position directly, with no offset chain at all. Only absolute
//! coordinates need to traverse it. That falls straight out of ADR 0003's choice
//! of the machine frame, and it is written down here because somebody will
//! otherwise reconstruct the offset chain for the incremental case and find that
//! it cancels.
//!
//! # Three decisions this module makes explicitly
//!
//! **`G53` keeps tool length compensation.** `G53` is non-modal and bypasses the
//! work offset and the `G92` shift; that much is settled and every control
//! agrees. Whether it also bypasses tool length compensation is *not* consistent
//! between controls, so it is a choice. It is kept here, because the IR stores a
//! **tool tip** position: a segment on which "tip" quietly meant "spindle gauge
//! point" would be a trap for U5, which has no way to know it should treat that
//! one segment differently. A corpus case pins it.
//!
//! **`G28` and `G30` are two moves, not one.** They travel to the reference
//! point *via* an intermediate point given by the block's axis words. Collapsing
//! them into one straight move to the reference point is how a simulation
//! reports clearance through a fixture that the real machine would have hit.
//!
//! **Tool length compensation contributes nothing when `H` matches `T`.** With
//! `G43 H<n>` a real control sets the spindle so that the *tip* lands on the
//! commanded point, so when the `H` offset is the tool's own length the
//! commanded position already is the tip and there is no correction to make.
//! What the lookup is for is the case where they differ: `H` naming a different
//! tool's length displaces the tip by the difference, which is a real and
//! expensive mistake, and it is reported. Under `G49` no compensation is active
//! and the commanded position is taken as the tip with a warning, because the
//! common reason to see `G49` is that the length is already baked into the work
//! offset — assuming otherwise would be wrong far more often than right.

use std::collections::BTreeMap;

use chipbreaker_core::math::Vec3;
use chipbreaker_core::tool::ToolLibrary;
use chipbreaker_core::toolpath::{
    ArcData, FeedMode, FeedSpec, MotionKind, MotionSegment, OffsetEpoch, PathEvent, PathEventKind,
    Provenance, RapidPath, TOOLPATH_SCHEMA_VERSION, Toolpath, ToolpathHeader, WorkOffsetId,
};

use crate::arcs::{self, ArcRequest, DEFAULT_ARC_TOLERANCE, Turn};
use crate::block::{Block, ModalGroup};
use crate::cycles::{self, CycleKind, CycleRequest};
use crate::diag::{Diagnostics, GcodeError, GcodeWarning, Site};
use crate::modal::{
    ArcCentreMode, CycleParams, CycleReturn, DistanceMode, ModalState, MotionMode, PathControl,
    ToolLength, Units,
};

/// How deep `M98` may nest before it is called a runaway.
pub const DEFAULT_SUBPROGRAM_DEPTH: u32 = 16;

/// Options a caller may set.
#[derive(Debug, Clone)]
pub struct ParseOptions {
    /// Units assumed before the program says otherwise.
    pub default_units: Units,
    /// Arc radius mismatch tolerance, in millimetres.
    pub arc_tolerance: f64,
    /// How rapids are represented.
    pub rapid_path: RapidPath,
    /// Value of one least input increment, in the program's units.
    ///
    /// `None` — the default — rejects an axis word with no decimal point rather
    /// than guessing whether `X10` means ten millimetres or ten thousandths.
    pub legacy_increment: Option<f64>,
    /// Whether blocks marked `/` are executed.
    pub execute_block_skip: bool,
    /// Promote warnings to errors.
    pub strict: bool,
    /// Subprogram nesting cap.
    pub max_subprogram_depth: u32,
    /// `G73`'s chip-break retract distance, in millimetres.
    ///
    /// No default, deliberately. It is a machine parameter absent from the NC
    /// file, and inventing one would put motion in the IR that the machine may
    /// not make. Absent, `G73` expands as a straight plunge and the omission is
    /// counted in [`chipbreaker_core::toolpath::ToolpathHeader::unmodelled_retracts`].
    pub chip_break_clearance: Option<f64>,
}

impl Default for ParseOptions {
    fn default() -> Self {
        Self {
            default_units: Units::Millimetres,
            arc_tolerance: DEFAULT_ARC_TOLERANCE,
            rapid_path: RapidPath::Linear,
            legacy_increment: None,
            execute_block_skip: true,
            strict: false,
            max_subprogram_depth: DEFAULT_SUBPROGRAM_DEPTH,
            chip_break_clearance: None,
        }
    }
}

/// What a parse produced besides the toolpath.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParseStats {
    /// Blocks read.
    pub blocks: u32,
    /// Segments emitted.
    pub segments: u32,
    /// Zero-length moves dropped.
    ///
    /// A full circle is **not** one of these: its chord is zero but its sweep is
    /// not, so it is very much motion.
    pub dropped_zero_length: u32,
    /// Blocks skipped because of a leading `/`.
    pub skipped_blocks: u32,
    /// Segments produced by expanding canned cycles.
    pub cycle_segments: u32,
    /// Subprogram calls executed.
    pub subprogram_calls: u32,
}

/// The machine, as the resolver models it.
pub struct Resolver<'a> {
    options: ParseOptions,
    tools: Option<&'a ToolLibrary>,
    state: ModalState,
    /// Where the tool tip is, in machine coordinates.
    position: Vec3,
    /// `G92` shift, in machine coordinates. Zero when inactive.
    g92: Vec3,
    /// Current value of each work offset.
    offsets: BTreeMap<WorkOffsetId, Vec3>,
    /// Every value each offset has held, and from which segment.
    epochs: BTreeMap<WorkOffsetId, Vec<OffsetEpoch>>,
    segments: Vec<MotionSegment>,
    events: Vec<PathEvent>,
    diagnostics: Diagnostics,
    stats: ParseStats,
    path_tolerance: Option<f64>,
    unmodelled_retracts: u32,
}

impl<'a> Resolver<'a> {
    /// A resolver at power-up state.
    #[must_use]
    pub fn new(options: ParseOptions, tools: Option<&'a ToolLibrary>) -> Self {
        let state = ModalState {
            units: options.default_units,
            ..ModalState::default()
        };
        let mut offsets = BTreeMap::new();
        let mut epochs = BTreeMap::new();
        // Every control has an offset active at power-up. Modelling its absence
        // would be a state that cannot arise.
        let g54 = WorkOffsetId::from_gcode(54, 0).unwrap_or_else(|| unreachable!());
        offsets.insert(g54, Vec3::ZERO);
        epochs.insert(
            g54,
            vec![OffsetEpoch {
                value: Vec3::ZERO,
                from_segment: 0,
            }],
        );
        Self {
            options,
            tools,
            state,
            position: Vec3::ZERO,
            g92: Vec3::ZERO,
            offsets,
            epochs,
            segments: Vec::new(),
            events: Vec::new(),
            diagnostics: Diagnostics::new(),
            stats: ParseStats::default(),
            path_tolerance: None,
            unmodelled_retracts: 0,
        }
    }

    /// Sets a work offset's value, as `G10 L2` would.
    pub fn set_offset(&mut self, id: WorkOffsetId, value: Vec3) {
        self.offsets.insert(id, value);
        let at = u32::try_from(self.segments.len()).unwrap_or(u32::MAX);
        self.epochs.entry(id).or_default().push(OffsetEpoch {
            value,
            from_segment: at,
        });
    }

    /// The diagnostics gathered so far.
    #[must_use]
    pub const fn diagnostics(&self) -> &Diagnostics {
        &self.diagnostics
    }

    /// The statistics gathered so far.
    #[must_use]
    pub const fn stats(&self) -> &ParseStats {
        &self.stats
    }

    /// The modal state, for tests and for the CLI's `lint`.
    #[must_use]
    pub const fn state(&self) -> &ModalState {
        &self.state
    }

    /// Where the tool tip is, in machine coordinates.
    #[must_use]
    pub const fn position(&self) -> Vec3 {
        self.position
    }

    /// Converts a programmed value to millimetres.
    fn to_mm(&self, value: f64) -> f64 {
        value * self.state.units.to_mm()
    }

    /// Reads an axis word, applying the decimal-point policy.
    fn axis_value(&self, word: &crate::lex::Word) -> Result<f64, GcodeError> {
        // Zero is exempt: zero is zero in any increment, so nothing is
        // ambiguous, and rejecting it would reject a construct in almost every
        // program ever written.
        if word.had_decimal || word.value == 0.0 {
            return Ok(self.to_mm(word.value));
        }
        match self.options.legacy_increment {
            Some(increment) => Ok(self.to_mm(word.value * increment)),
            None => Err(GcodeError::MissingDecimalPoint {
                site: word.site,
                word: word.raw.clone(),
            }),
        }
    }

    /// The machine-coordinate translation from the active workpiece frame.
    fn active_offset(&self) -> Vec3 {
        self.offsets
            .get(&self.state.work_offset)
            .copied()
            .unwrap_or(Vec3::ZERO)
    }

    /// The correction tool length compensation makes to the tip position.
    ///
    /// See the module header: zero when `H` names the tool that is actually in
    /// the spindle, which is the whole of the ordinary case.
    fn tool_length_correction(&mut self, site: Site) -> Result<Vec3, GcodeError> {
        let Some(h) = self.state.tool_length.h() else {
            return Ok(Vec3::ZERO);
        };
        let Some(library) = self.tools else {
            // No library given. The H cannot be checked, so say so rather than
            // pretending it was.
            return Ok(Vec3::ZERO);
        };
        let h_length =
            lookup_length(library, h).ok_or(GcodeError::UnknownToolOffset { site, h })?;
        let t_length = lookup_length(library, self.state.tool).unwrap_or(h_length);
        let sign = match self.state.tool_length {
            ToolLength::Negative { .. } => -1.0,
            ToolLength::None | ToolLength::Positive { .. } => 1.0,
        };
        Ok(Vec3::new(0.0, 0.0, sign * (h_length - t_length)))
    }

    /// Turns a block's axis words into a target position in machine coordinates.
    ///
    /// This is the pipeline in the module header, in that order.
    fn target(&mut self, block: &Block, ignore_offsets: bool) -> Result<Vec3, GcodeError> {
        let mut target = self.position;
        let offset = if ignore_offsets {
            Vec3::ZERO
        } else {
            self.active_offset() + self.g92
        };
        let correction = if ignore_offsets {
            // G53 keeps tool length compensation; see the module header.
            self.tool_length_correction(block.site)?
        } else {
            self.tool_length_correction(block.site)?
        };

        for (index, slot) in block.axes.iter().enumerate() {
            let Some(word) = slot else { continue };
            // A/B/C are rotary and have no place in a 3-axis IR yet. They are
            // parsed so that a 5-axis file is not a syntax error at U4, and
            // ignored so that U16 can add them without changing what U4 meant.
            if index >= 3 {
                continue;
            }
            let value = self.axis_value(word)?;
            let mut point = target.to_array();
            point[index] = match self.state.distance {
                // A delta is a delta in every frame; see the module header.
                DistanceMode::Incremental => point[index] + value,
                DistanceMode::Absolute => {
                    value + offset.to_array()[index] + correction.to_array()[index]
                }
            };
            target = Vec3::from_array(point);
        }
        Ok(target)
    }

    /// Records an event at the current end of the segment stream.
    fn push_event(&mut self, kind: PathEventKind, source: Provenance) {
        let at = u32::try_from(self.segments.len()).unwrap_or(u32::MAX);
        self.events.push(PathEvent {
            at_segment: at,
            kind,
            source,
        });
    }

    /// The feed specification in force.
    fn feed_spec(&self, kind: MotionKind) -> FeedSpec {
        if kind == MotionKind::Rapid {
            return FeedSpec::rapid();
        }
        FeedSpec {
            value: self.state.feed_mm().unwrap_or(0.0),
            mode: self.state.feed_mode,
            spindle_rpm: self.state.spindle,
        }
    }

    /// Emits a straight move to `to`, dropping it if it goes nowhere.
    pub fn emit_linear(&mut self, kind: MotionKind, to: Vec3, source: Provenance, site: Site) {
        if to == self.position {
            self.stats.dropped_zero_length += 1;
            self.diagnostics.warn(GcodeWarning::ZeroLengthMove { site });
            return;
        }
        let segment = MotionSegment {
            kind,
            start: self.position,
            end: to,
            arc: None,
            orientation: None,
            tool: self.state.tool,
            feed: self.feed_spec(kind),
            source,
        };
        self.position = to;
        self.segments.push(segment);
        self.stats.segments += 1;
        if source.is_from_cycle() {
            self.stats.cycle_segments += 1;
        }
    }

    /// Emits an arc or helix.
    fn emit_arc(&mut self, to: Vec3, arc: ArcData, source: Provenance) {
        let normal = arc.plane.normal();
        let rise = (to - self.position).dot(normal);
        let kind = if rise == 0.0 {
            MotionKind::Arc
        } else {
            MotionKind::Helix
        };
        let segment = MotionSegment {
            kind,
            start: self.position,
            end: to,
            arc: Some(arc),
            orientation: None,
            tool: self.state.tool,
            feed: self.feed_spec(kind),
            source,
        };
        self.position = to;
        self.segments.push(segment);
        self.stats.segments += 1;
        if source.is_from_cycle() {
            self.stats.cycle_segments += 1;
        }
    }

    /// Finishes, producing the toolpath.
    ///
    /// # Errors
    /// [`GcodeError::Ir`] if the assembled segments violate an IR invariant,
    /// which would be a bug here rather than in the program.
    pub fn finish(self, program: &str) -> Result<(Toolpath, Diagnostics, ParseStats), GcodeError> {
        let header = ToolpathHeader {
            schema_version: TOOLPATH_SCHEMA_VERSION,
            program: program.to_owned(),
            offsets: self.epochs,
            rapid_path: self.options.rapid_path,
            arc_tolerance: self.options.arc_tolerance,
            path_tolerance: self.path_tolerance,
            block_skip_executed: self.options.execute_block_skip,
            unmodelled_retracts: self.unmodelled_retracts,
        };
        let path =
            Toolpath::new(header, self.segments, self.events).map_err(|e| GcodeError::Ir {
                detail: e.to_string(),
            })?;
        Ok((path, self.diagnostics, self.stats))
    }
}

/// Resolves a `T` or `H` number against the tool library.
///
/// The convention: **a library entry whose identifier parses as a decimal
/// integer is that tool's number.** So `T4` and `H4` both resolve to the entry
/// named `"4"`. Entries with descriptive names are perfectly legal and simply
/// are not addressable from NC, which is what a library shared between a
/// simulator and a documentation generator wants.
fn lookup_length(library: &ToolLibrary, number: u32) -> Option<f64> {
    let name = number.to_string();
    library
        .get(&name)
        .or_else(|| library.get(&format!("T{number}")))
        .map(chipbreaker_core::tool::Tool::gauge_length)
}

/// Reads the codes in a block that change modal state, before any motion.
///
/// Separated from motion because the order matters: `G90 G1 X10.` must apply the
/// distance mode before reading `X`, and `G20 X1.` must change the unit first.
pub fn apply_modal(state: &mut ModalState, block: &Block) {
    if let Some(key) = block.g_in(ModalGroup::Plane) {
        state.plane = match key {
            180 => chipbreaker_core::toolpath::ArcPlane::Zx,
            190 => chipbreaker_core::toolpath::ArcPlane::Yz,
            _ => chipbreaker_core::toolpath::ArcPlane::Xy,
        };
    }
    if let Some(key) = block.g_in(ModalGroup::Distance) {
        state.distance = if key == 910 {
            DistanceMode::Incremental
        } else {
            DistanceMode::Absolute
        };
    }
    if let Some(key) = block.g_in(ModalGroup::ArcDistance) {
        state.arc_centre = if key == 901 {
            ArcCentreMode::Absolute
        } else {
            ArcCentreMode::Incremental
        };
    }
    if let Some(key) = block.g_in(ModalGroup::FeedMode) {
        state.feed_mode = match key {
            930 => FeedMode::InverseTime,
            950 => FeedMode::UnitsPerRevolution,
            _ => FeedMode::UnitsPerMinute,
        };
    }
    if let Some(key) = block.g_in(ModalGroup::Units) {
        state.units = if key == 200 {
            Units::Inches
        } else {
            Units::Millimetres
        };
    }
    if let Some(key) = block.g_in(ModalGroup::CycleReturn) {
        state.cycle_return = if key == 990 {
            CycleReturn::RPlane
        } else {
            CycleReturn::InitialZ
        };
    }
    if let Some(key) = block.g_in(ModalGroup::PathControl) {
        state.path_control = match key {
            611 => PathControl::ExactPath,
            640 => PathControl::Blended { tolerance: None },
            _ => PathControl::ExactStop,
        };
    }
    if let Some(key) = block.g_in(ModalGroup::WorkOffset)
        && let Some(id) = WorkOffsetId::from_gcode(key / 10, key % 10)
    {
        state.work_offset = id;
    }
}

/// Turn direction from a motion code.
#[must_use]
pub const fn turn_of(key: u32) -> Option<Turn> {
    match key {
        20 => Some(Turn::Clockwise),
        30 => Some(Turn::CounterClockwise),
        _ => None,
    }
}

/// Motion mode from a motion-group code.
#[must_use]
pub const fn motion_of(key: u32) -> MotionMode {
    match key {
        0 => MotionMode::Rapid,
        10 => MotionMode::Linear,
        20 => MotionMode::ArcClockwise,
        30 => MotionMode::ArcCounterClockwise,
        800 => MotionMode::None,
        other => MotionMode::Cycle(other),
    }
}

/// Everything a block needs from the resolver to build an arc.
pub(crate) fn arc_request(
    state: &ModalState,
    block: &Block,
    start: Vec3,
    end: Vec3,
    turn: Turn,
    tolerance: f64,
    to_mm: f64,
) -> ArcRequest {
    // Which words carry the centre depends on the plane: G17 uses I,J; G18 uses
    // I,K; G19 uses J,K. Reading them positionally rather than by plane is the
    // classic way to get an arc in the wrong place.
    let mut centre_components = [None, None, None];
    for (index, slot) in block.ijk.iter().enumerate() {
        if let Some(word) = slot {
            centre_components[index] = Some(word.value * to_mm);
        }
    }

    let centre = if centre_components.iter().any(Option::is_some) {
        let base = match state.arc_centre {
            // Fanuc's default: offsets from the arc's start point.
            ArcCentreMode::Incremental => start.to_array(),
            // Absolute positions, in machine coordinates.
            ArcCentreMode::Absolute => [0.0, 0.0, 0.0],
        };
        let mut point = base;
        for (axis, component) in centre_components.iter().enumerate() {
            if let Some(value) = component {
                point[axis] = base[axis] + value;
            }
        }
        Some(Vec3::from_array(point))
    } else {
        None
    };

    ArcRequest {
        start,
        end,
        plane: state.plane,
        turn,
        centre,
        radius_word: if centre.is_none() {
            block.r.as_ref().map(|w| w.value * to_mm)
        } else {
            None
        },
        extra_turns: block
            .p
            .as_ref()
            .and_then(crate::lex::Word::as_u32)
            .unwrap_or(0)
            .saturating_sub(1),
        tolerance,
        site: block.site,
    }
}

/// Resolves one arc block.
///
/// # Errors
/// See [`arcs::resolve`].
pub(crate) fn resolve_arc(
    request: &ArcRequest,
    diagnostics: &mut Diagnostics,
) -> Result<ArcData, GcodeError> {
    arcs::resolve(request, diagnostics)
}

/// Parses one program into a toolpath.
///
/// # Errors
///
/// The first [`GcodeError`] encountered. Warnings accumulate instead, and are
/// returned alongside the toolpath; `--strict` turns the first of them into an
/// error at the end.
pub fn parse(
    text: &str,
    program: &str,
    options: &ParseOptions,
    tools: Option<&ToolLibrary>,
) -> Result<(Toolpath, Diagnostics, ParseStats), GcodeError> {
    let mut resolver = Resolver::new(options.clone(), tools);
    let raw = crate::lex::lex(text, 0, &mut resolver.diagnostics)?;

    // Assemble every line once. Subprograms are re-executed rather than
    // re-parsed, so a call in a loop costs no lexing.
    let mut blocks = Vec::with_capacity(raw.len());
    for line in &raw {
        if line.is_empty() {
            continue;
        }
        blocks.push(crate::block::assemble(line)?);
    }

    let table = subprogram_table(&blocks);
    let mut cursor = Cursor {
        index: 0,
        block_number: 0,
    };
    resolver.run_range(&blocks, &table, 0, blocks.len(), 0, &mut cursor)?;

    if let Some(error) = resolver.diagnostics.first_as_error(options.strict) {
        return Err(error);
    }
    resolver.finish(program)
}

impl Resolver<'_> {
    /// Interprets one assembled block.
    ///
    /// # Errors
    /// Any [`GcodeError`] the block provokes.
    #[allow(
        clippy::too_many_lines,
        reason = "one block is one sequence of decisions; splitting it would \
                  hide the order, and the order is the semantics"
    )]
    pub fn execute(&mut self, block: &Block, block_index: u32) -> Result<(), GcodeError> {
        let source = Provenance::new(block.site.file, block.site.line, block_index);

        if block.block_skip && !self.options.execute_block_skip {
            self.stats.skipped_blocks += 1;
            self.diagnostics.warn(GcodeWarning::BlockSkip {
                site: block.site,
                executed: false,
            });
            return Ok(());
        }

        let previous_units = self.state.units;
        apply_modal(&mut self.state, block);
        if self.state.units != previous_units {
            self.diagnostics.warn(GcodeWarning::UnitsChanged {
                site: block.site,
                to: self.state.units.as_str(),
            });
        }
        if block.g_in(ModalGroup::WorkOffset).is_some() {
            self.push_event(
                PathEventKind::WorkOffsetChanged {
                    to: self.state.work_offset,
                },
                source,
            );
        }
        // G64's tolerance rides on a P word, and only means anything here.
        if block.has_g(640) {
            let tolerance = block.p.as_ref().map(|w| w.value * self.state.units.to_mm());
            self.state.path_control = PathControl::Blended { tolerance };
            if let Some(value) = tolerance {
                self.path_tolerance = Some(value);
                self.diagnostics.warn(GcodeWarning::PathToleranceIgnored {
                    site: block.site,
                    tolerance: value,
                });
            }
        }

        // Words that set state regardless of motion.
        if let Some(word) = &block.f {
            self.state.feed = Some(word.value);
        }
        if let Some(word) = &block.s {
            self.state.spindle = Some(word.value);
        }
        if let Some(word) = &block.t
            && let Some(number) = word.as_u32()
        {
            self.state.tool = number;
        }
        if let Some(key) = block.g_in(ModalGroup::ToolLength) {
            let h = block.h.as_ref().and_then(crate::lex::Word::as_u32);
            self.state.tool_length = match (key, h) {
                (430, Some(h)) => ToolLength::Positive { h },
                (440, Some(h)) => ToolLength::Negative { h },
                // G43.1 gives the length inline rather than by number; treated
                // as no table lookup, which is what it is.
                (431, _) => ToolLength::None,
                (490, _) => ToolLength::None,
                // G43 with no H uses the tool's own number, which is what every
                // control does.
                (430, None) => ToolLength::Positive { h: self.state.tool },
                (440, None) => ToolLength::Negative { h: self.state.tool },
                _ => self.state.tool_length,
            };
            // Resolving it now, rather than at the first move, so that an
            // unknown offset is reported on the line that asked for it.
            self.tool_length_correction(block.site)?;
            self.push_event(
                PathEventKind::ToolLengthOffset {
                    h: self.state.tool_length.h(),
                },
                source,
            );
        }

        // M codes.
        for &code in &block.m_codes {
            match code {
                0 => self.push_event(PathEventKind::Stop, source),
                10 => self.push_event(PathEventKind::OptionalStop, source),
                20 | 300 => self.push_event(PathEventKind::ProgramEnd, source),
                30 | 40 => {
                    let sign = if code == 40 { -1.0 } else { 1.0 };
                    let rpm = sign * self.state.spindle.unwrap_or(0.0).abs();
                    self.state.spindle = Some(rpm);
                    self.push_event(PathEventKind::Spindle { rpm }, source);
                }
                50 => {
                    self.state.spindle = Some(0.0);
                    self.push_event(PathEventKind::Spindle { rpm: 0.0 }, source);
                }
                60 => self.push_event(
                    PathEventKind::ToolChange {
                        tool: self.state.tool,
                    },
                    source,
                ),
                other => {
                    let number = other / 10;
                    self.diagnostics.warn(GcodeWarning::UnmodelledMCode {
                        site: block.site,
                        code: number,
                    });
                    self.push_event(PathEventKind::UnmodelledMCode { code: number }, source);
                }
            }
        }

        // Non-modal codes, before motion, because several of them change where
        // motion starts from.
        if block.has_g(40) {
            let seconds = block.p.as_ref().map_or(0.0, |w| w.value);
            self.push_event(PathEventKind::Dwell { seconds }, source);
        }
        // G10 and G92 blocks carry axis words that are *parameters*, not a
        // destination, and they command no motion. Returning here rather than
        // falling through is load-bearing: `G10 L2 P1 X-250.` would otherwise
        // set the offset and then rapid to X-250, which is a move across the
        // machine that the program never asked for. G92 happened to survive the
        // same bug only because its target worked out to where the tool already
        // was, and was silently dropped as zero-length.
        if block.has_g(100) {
            self.apply_g10(block, source)?;
            return Ok(());
        }
        for key in [920, 921, 922, 923] {
            if block.has_g(key) {
                self.apply_g92(key, block, source)?;
                return Ok(());
            }
        }

        // G28/G30 are two moves via an intermediate point, not one.
        if block.has_g(280) || block.has_g(300) {
            let intermediate = self.target(block, false)?;
            self.emit_linear(MotionKind::Rapid, intermediate, source, block.site);
            // The reference point. Machine zero, because the true value is a
            // machine parameter absent from the NC file, and zero is where a
            // machine's reference switches are by definition of the machine
            // frame.
            self.emit_linear(MotionKind::Rapid, Vec3::ZERO, source, block.site);
            return Ok(());
        }

        // Motion.
        let motion_key = block.g_in(ModalGroup::Motion);
        if let Some(key) = motion_key {
            self.state.motion = motion_of(key);
            if self.state.motion == MotionMode::None {
                self.state.cycle = None;
            }
        }
        if !block.has_axis_words() {
            return Ok(());
        }
        if self.state.motion == MotionMode::None {
            return Ok(());
        }

        if let MotionMode::Cycle(key) = self.state.motion {
            return self.fire_cycle(key, block, source);
        }

        // G53 is non-modal and applies to this block alone.
        let ignore_offsets = block.has_g(530);
        let target = self.target(block, ignore_offsets)?;

        match self.state.motion {
            MotionMode::Rapid => {
                self.emit_linear(MotionKind::Rapid, target, source, block.site);
            }
            MotionMode::Linear => {
                if self.state.feed.is_none() {
                    return Err(GcodeError::NoFeedRate { site: block.site });
                }
                self.emit_linear(MotionKind::Linear, target, source, block.site);
            }
            MotionMode::ArcClockwise | MotionMode::ArcCounterClockwise => {
                if self.state.feed.is_none() {
                    return Err(GcodeError::NoFeedRate { site: block.site });
                }
                let turn = if self.state.motion == MotionMode::ArcClockwise {
                    Turn::Clockwise
                } else {
                    Turn::CounterClockwise
                };
                let request = arc_request(
                    &self.state,
                    block,
                    self.position,
                    target,
                    turn,
                    self.options.arc_tolerance,
                    self.state.units.to_mm(),
                );
                let arc = resolve_arc(&request, &mut self.diagnostics)?;
                self.emit_arc(target, arc, source);
            }
            MotionMode::Cycle(_) | MotionMode::None => unreachable!("handled above"),
        }
        Ok(())
    }

    /// Reads or refreshes the parameters of the active canned cycle.
    ///
    /// They persist. Once a cycle is active, a block carrying only `X` fires it
    /// again at the new position with the same `Z`, `R` and `Q`, which is why
    /// they live on the modal state rather than being re-read per block.
    fn cycle_params(&mut self, block: &Block) -> Result<CycleParams, GcodeError> {
        let previous = self.state.cycle;
        let mut params = previous.unwrap_or(CycleParams {
            // The Z the cycle started from, for G98. Captured before any of this
            // block's motion, which is what makes a G98 retract go back to where
            // the tool was when the cycle began rather than to the last hole.
            initial_z: self.position.z,
            ..CycleParams::default()
        });
        if previous.is_none() {
            params.initial_z = self.position.z;
            // Without an R the cycle has no retract plane, and the sanest
            // reading of that is the height the tool is already at.
            params.r = self.position.z;
            params.z = self.position.z;
        }

        if let Some(word) = &block.r {
            let value = self.to_mm(word.value);
            params.r = match self.state.distance {
                // Under G91 the R word is measured from the initial Z, not from
                // the workpiece origin. Reading it as absolute puts the retract
                // plane somewhere else entirely.
                DistanceMode::Incremental => params.initial_z + value,
                DistanceMode::Absolute => value + self.frame_offset().z,
            };
        }
        // **R before Z, and the order is load-bearing.** Under G91 the Z word is
        // measured from the R plane, so reading Z first uses whatever R held
        // before this block -- which on the cycle's first firing is the height
        // the tool happened to be at. The differential test caught exactly this:
        // a bolt pattern drilled to Z+3 instead of Z-5, and every motion after
        // it was consistent with that wrong depth.
        if let Some(word) = &block.axes[2] {
            let value = self.axis_value(word)?;
            params.z = match self.state.distance {
                // Incremental Z is measured from the R plane downward, which is
                // what every control does and what makes a bolt pattern work.
                DistanceMode::Incremental => params.r + value,
                DistanceMode::Absolute => value + self.frame_offset().z,
            };
        }
        if let Some(word) = &block.q {
            params.q = Some(self.to_mm(word.value).abs());
        }
        if let Some(word) = &block.p {
            params.p = Some(word.value);
        }
        self.state.cycle = Some(params);
        Ok(params)
    }

    /// The translation from the active workpiece frame to machine coordinates.
    fn frame_offset(&self) -> Vec3 {
        self.active_offset() + self.g92
    }

    /// Fires the active canned cycle once per repeat.
    fn fire_cycle(
        &mut self,
        key: u32,
        block: &Block,
        source: Provenance,
    ) -> Result<(), GcodeError> {
        let Some(kind) = CycleKind::from_key(key) else {
            return Err(GcodeError::UnsupportedCode {
                site: block.site,
                code: crate::block::render_code('G', key),
            });
        };
        let params = self.cycle_params(block)?;

        // L or K is the repeat count. L0 means do not execute -- a real case,
        // and one that reads exactly like a typo.
        let repeats = block
            .l
            .as_ref()
            .and_then(crate::lex::Word::as_u32)
            .unwrap_or(1);
        if repeats == 0 {
            return Ok(());
        }

        for _ in 0..repeats {
            // Under G91 each repeat steps by the same X/Y increment, which is
            // how one line becomes a bolt pattern.
            let hole = self.target(block, false)?;
            let request = CycleRequest {
                kind,
                from: self.position,
                hole,
                bottom: params.z,
                r_plane: params.r,
                initial_z: params.initial_z,
                return_to_initial: self.state.cycle_return == CycleReturn::InitialZ,
                peck: params.q,
                chip_break: self.options.chip_break_clearance,
                site: block.site,
            };
            if kind == CycleKind::PeckChipBreak
                && params.q.is_some_and(|q| q > 0.0)
                && self.options.chip_break_clearance.is_none()
            {
                self.unmodelled_retracts += 1;
                self.diagnostics
                    .warn(GcodeWarning::UnmodelledRetract { site: block.site });
            }
            let moves = cycles::expand(&request).map_err(|e| match e {
                cycles::CycleError::TooManyPecks { wanted } => GcodeError::BadCycle {
                    site: block.site,
                    detail: format!(
                        "a peck depth of {} over {} mm asks for {wanted} pecks, above the {} this build will expand. A Q an order of magnitude too small is the usual cause",
                        params.q.unwrap_or(0.0),
                        (params.r - params.z).abs(),
                        cycles::MAX_PECKS
                    ),
                },
            })?;
            for (step, motion) in moves.into_iter().enumerate() {
                if motion.kind != MotionKind::Rapid && self.state.feed.is_none() {
                    return Err(GcodeError::NoFeedRate { site: block.site });
                }
                self.emit_linear(
                    motion.kind,
                    motion.to,
                    cycles::provenance_for(source, step),
                    block.site,
                );
            }
            if let Some(seconds) = params.p
                && kind == CycleKind::DrillDwell
            {
                self.push_event(PathEventKind::Dwell { seconds }, source);
            }
        }
        Ok(())
    }

    /// `G10 L2`/`L20`: the program rewrites a work offset.
    fn apply_g10(&mut self, block: &Block, source: Provenance) -> Result<(), GcodeError> {
        let l = block
            .l
            .as_ref()
            .and_then(crate::lex::Word::as_u32)
            .unwrap_or(2);
        let Some(p) = block.p.as_ref().and_then(crate::lex::Word::as_u32) else {
            return Ok(());
        };
        // P1 is G54, P2 is G55, and so on. The same off-by-53 the IR's newtype
        // exists to prevent, in a different disguise.
        let Some(id) = WorkOffsetId::from_gcode(53 + p, 0) else {
            return Ok(());
        };

        let mut value = self.offsets.get(&id).copied().unwrap_or(Vec3::ZERO);
        for (axis, slot) in block.axes.iter().enumerate().take(3) {
            let Some(word) = slot else { continue };
            let programmed = self.axis_value(word)?;
            let mut components = value.to_array();
            components[axis] = if l == 20 {
                // L20 sets the offset so that the *current position* reads as
                // the programmed value, rather than setting the offset itself.
                self.position.to_array()[axis] - programmed
            } else {
                programmed
            };
            value = Vec3::from_array(components);
        }
        self.set_offset(id, value);
        self.push_event(PathEventKind::WorkOffsetRedefined { offset: id }, source);
        Ok(())
    }

    /// `G92` and its cancellations.
    fn apply_g92(&mut self, key: u32, block: &Block, source: Provenance) -> Result<(), GcodeError> {
        match key {
            920 => {
                // G92 sets a shift such that the current position reads as the
                // programmed value. A persistent coordinate shift, not a move.
                let base = self.active_offset();
                for (axis, slot) in block.axes.iter().enumerate().take(3) {
                    let Some(word) = slot else { continue };
                    let programmed = self.axis_value(word)?;
                    let mut shift = self.g92.to_array();
                    shift[axis] =
                        self.position.to_array()[axis] - base.to_array()[axis] - programmed;
                    self.g92 = Vec3::from_array(shift);
                }
                self.push_event(PathEventKind::CoordinateShift { active: true }, source);
            }
            921 | 922 => {
                self.g92 = Vec3::ZERO;
                self.push_event(PathEventKind::CoordinateShift { active: false }, source);
            }
            // G92.3 restores a previously cancelled shift. We do not keep the
            // cancelled value, so this is a no-op and says so.
            _ => {}
        }
        Ok(())
    }
}

/// Where the driver is, threaded through nested calls so that block indices
/// stay unique across a subprogram executed more than once.
struct Cursor {
    /// The next block index to hand to [`Resolver::execute`].
    index: usize,
    /// Monotonic counter, so two executions of one subprogram body are
    /// distinguishable in provenance.
    block_number: u32,
}

/// Where each `O` number's body begins and ends.
///
/// A body runs from the block *after* its `O` label to the matching `M99`. A
/// label with no `M99` has no body and calling it is an error rather than a
/// silent fall-through to the end of the file.
fn subprogram_table(blocks: &[Block]) -> BTreeMap<u32, (usize, usize)> {
    let mut table = BTreeMap::new();
    let mut open: Option<(u32, usize)> = None;
    for (index, block) in blocks.iter().enumerate() {
        if let Some(word) = &block.o
            && let Some(number) = word.as_u32()
        {
            // A new label closes nothing: a body that never saw M99 is simply
            // not callable, which is what the missing entry means.
            open = Some((number, index + 1));
        }
        if block.has_m(990)
            && let Some((number, start)) = open.take()
        {
            table.insert(number, (start, index));
        }
    }
    table
}

impl Resolver<'_> {
    /// Executes `blocks[start..end]`, following `M98` calls.
    ///
    /// # Errors
    /// Any [`GcodeError`], including [`GcodeError::SubprogramTooDeep`] and
    /// [`GcodeError::UnknownSubprogram`].
    fn run_range(
        &mut self,
        blocks: &[Block],
        table: &BTreeMap<u32, (usize, usize)>,
        start: usize,
        end: usize,
        depth: u32,
        cursor: &mut Cursor,
    ) -> Result<(), GcodeError> {
        if depth > self.options.max_subprogram_depth {
            let site = blocks.get(start).map_or(Site::default(), |b| b.site);
            return Err(GcodeError::SubprogramTooDeep {
                site,
                limit: self.options.max_subprogram_depth,
            });
        }

        let mut index = start;
        while index < end {
            let block = &blocks[index];
            cursor.index = index;

            // A bare `O` label is a marker, not motion, and a body reached by
            // falling off the end of the main program is not executed: real
            // controls stop at M30, and so does this.
            if block.has_m(200) || block.has_m(300) {
                self.stats.blocks += 1;
                self.execute(block, cursor.block_number)?;
                cursor.block_number += 1;
                return Ok(());
            }
            if block.has_m(990) {
                return Ok(());
            }

            if block.has_m(980) {
                let Some(number) = block.p.as_ref().and_then(crate::lex::Word::as_u32) else {
                    index += 1;
                    continue;
                };
                let Some(&(body_start, body_end)) = table.get(&number) else {
                    return Err(GcodeError::UnknownSubprogram {
                        site: block.site,
                        number,
                    });
                };
                let repeats = block
                    .l
                    .as_ref()
                    .and_then(crate::lex::Word::as_u32)
                    .unwrap_or(1);
                for _ in 0..repeats {
                    self.stats.subprogram_calls += 1;
                    self.run_range(blocks, table, body_start, body_end, depth + 1, cursor)?;
                }
                index += 1;
                continue;
            }

            self.stats.blocks += 1;
            self.execute(block, cursor.block_number)?;
            cursor.block_number += 1;
            index += 1;
        }
        Ok(())
    }
}
