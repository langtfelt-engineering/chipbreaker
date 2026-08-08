"""Generate the mechanical half of the G-code corpus.

The landmine cases are hand-written and live in the repository already; this
writes the ones whose content is a permutation of something — the same arc in
three planes, the same cycle in eight forms — where writing them by hand would
be forty chances to make a typo that looks like a test.

Every generated file carries a full-precision coordinate somewhere, per the
fixture rule in CONTRIBUTING.md: round numbers survive any parser ever written,
which is exactly why they cannot detect one that is wrong.

Regenerate with:

    python scripts/gcode_corpus.py
"""

import io
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CORPUS = os.path.join(HERE, "..", "tests", "corpus", "gcode")

# A coordinate needing all 17 significant digits. Taken from real geometry -- a
# 3 degree taper over 20 mm -- rather than invented.
AWKWARD = 1.0 + 20.0 * math.tan(math.radians(3))
assert len(repr(AWKWARD).replace(".", "").lstrip("-")) >= 17, repr(AWKWARD)

PREAMBLE = "%\nO1000 (generated corpus entry)\nG21 G17 G90 G94 G54\nG0 X0. Y0. Z10.\nS8000 M3\n"
EPILOGUE = "M5\nG0 Z10.\nM30\n%\n"

entries = {}


def add(name, why, body, *, expect="parse", error=None, tail=EPILOGUE):
    """Writes one entry and records what it is for."""
    text = PREAMBLE + body + tail
    path = os.path.join(CORPUS, name + ".nc")
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    entries[name] = {"_why": why, "expect": expect, "error": error}


# --- arcs: the same arc, every way of writing it ---------------------------

add(
    "arc-quarter-ijk",
    "A quarter arc in centre-offset form. Paired with arc-quarter-r: the two "
    "must resolve to the same arc within 32 ULP, and their golden hashes are "
    "SUPPOSED to differ. See the corpus README.",
    f"G0 X10. Y0.\nG1 Z-1. F{500 + 0:.1f}\nG3 X0. Y10. I-10. J0.\nG0 Z10.\n",
)
add(
    "arc-quarter-r",
    "The same quarter arc in radius form. Positive R selects the minor arc.",
    "G0 X10. Y0.\nG1 Z-1. F500.\nG3 X0. Y10. R10.\nG0 Z10.\n",
)
add(
    "arc-major-r",
    "Negative R selects the major arc: the same endpoints, the long way round.",
    "G0 X10. Y0.\nG1 Z-1. F500.\nG3 X0. Y10. R-10.\nG0 Z10.\n",
)
add(
    "arc-full-circle-ijk",
    "Coincident endpoints with I/J/K is a full circle. Its chord is zero and it "
    "is emphatically not a zero-length move.",
    "G0 X10. Y0.\nG1 Z-1. F500.\nG2 X10. Y0. I-10. J0.\nG0 Z10.\n",
)
add(
    "arc-helix",
    "A helix: the third axis moves linearly across the arc, so the segment is "
    "longer than the planar arc by the rise.",
    "G0 X10. Y0. Z0.\nG1 Z-1. F500.\nG3 X10. Y0. Z-5. I-10. J0.\nG0 Z10.\n",
)
# Each plane's arc runs from its FIRST axis to its SECOND, in the order RS-274
# defines them: G17 is X,Y; G18 is **Z,X**; G19 is Y,Z. Writing G18 as X-to-Z
# instead of Z-to-X gives a 270 degree arc rather than a 90 degree one -- which
# is the correct answer to a different question, and exactly the trap the unit
# specification warns about. The first version of this generator had it that way
# round and the three-plane test caught it.
for plane, code, start, end, centre in [
    ("g17", "G17", "X10. Y0.", "X0. Y10.", "I-10. J0."),
    ("g18", "G18", "X0. Z10.", "X10. Z0.", "I0. K-10."),
    ("g19", "G19", "Y10. Z0.", "Y0. Z10.", "J-10. K0."),
]:
    add(
        f"arc-plane-{plane}",
        f"The same quarter arc in {code}. G18's handedness is the trap: RS-274 "
        "orders it Z then X so its normal is +Y, and reading it as X,Z turns "
        "every G2 into a G3.",
        f"{code}\nG0 {start}\nG1 F500.\nG3 {end} {centre}\nG17\n",
    )
add(
    "arc-residual-within-tolerance",
    "A centre 4 microns inconsistent between the endpoints, which is ordinary "
    "CAM rounding rather than an error. The centre is moved onto the chord's "
    "perpendicular bisector and the residual is recorded.",
    "G0 X10. Y0.\nG1 Z-1. F500.\nG3 X0. Y10.004 I-10. J0.\nG0 Z10.\n",
)

# --- canned cycles: every cycle, and every retract mode --------------------

CYCLES = [
    ("g81", "G81", "", "Drill: position, plunge to R, feed to depth, retract."),
    ("g82", "G82", " P0.5", "Drill with a dwell, which removes no material."),
    ("g83", "G83", " Q3.", "Peck with a full retract to the R plane between pecks."),
    ("g73", "G73", " Q3.", "Peck whose chip-break retract is a machine parameter absent from the file. Without --chip-break-clearance the retract is omitted and counted in the header, so a collision check can refuse to certify against it."),
    ("g85", "G85", "", "Bore: feed in, feed back out."),
    ("g86", "G86", "", "Bore with the spindle stopped: feed in, rapid out."),
    ("g84", "G84", "", "Tapping, which is geometrically a bore."),
]
for name, code, extra, why in CYCLES:
    for ret, ret_code in [("g98", "G98"), ("g99", "G99")]:
        add(
            f"cycle-{name}-{ret}",
            f"{why} {ret_code} retracts to "
            + ("the initial Z" if ret == "g98" else "the R plane")
            + ", which changes every intermediate retract in a pattern.",
            f"F250.\n{ret_code} {code} X20. Y30. Z-5. R2.{extra}\nX40.\nG80\n",
        )

add(
    "cycle-repeat-bolt-pattern",
    "One line becomes a bolt pattern: L3 under G91 fires the cycle three times, "
    "each stepping by the same increment.",
    "F250.\nG99 G91 G81 X10. Y0. Z-7. R-8. L3\nG80\nG90\n",
)
add(
    "cycle-l0-does-nothing",
    "L0 means do not execute. A real case, and one that reads exactly like a typo.",
    "F250.\nG99 G81 X10. Y0. Z-5. R2. L0\nG80\n",
)
add(
    "cycle-persists-until-g80",
    "A cycle fires again on any block carrying axis words, and a block with only "
    "an F word does not fire it.",
    "F250.\nG99 G81 X10. Y0. Z-5. R2.\nX20.\nF300.\nX30.\nG80\n",
)

# --- the coordinate pipeline ----------------------------------------------

add(
    "offset-g54-to-g55",
    "A mid-program work offset change. In machine coordinates the path stays "
    "continuous across it, which is the whole argument of ADR 0003.",
    f"G10 L2 P1 X-250. Y-100. Z-{AWKWARD!r}\nG10 L2 P2 X-150. Y-200. Z-300.\n"
    "G54 G0 X0. Y0.\nG1 Z-1. F400.\nG0 Z10.\nG55 G0 X0. Y0.\nG1 Z-1. F400.\n",
)
add(
    "offset-g10-mid-program",
    "G10 L2 rewrites an offset that earlier segments already used. Their "
    "geometry is unaffected, but the header must record both epochs or a report "
    "rendering into a workpiece frame places the early moves wrongly.",
    "G10 L2 P1 X-100. Y0. Z0.\nG54 G0 X10. Y0.\nG10 L2 P1 X-200. Y0. Z0.\nG0 X10.\n",
)
add(
    "offset-g92-shift",
    "G92 is a persistent coordinate shift, not a move. Nothing travels on the "
    "G92 line itself.",
    "G10 L2 P1 X-100. Y0. Z0.\nG54 G0 X10. Y0.\nG92 X0.\nG0 X5.\nG92.1\nG0 X5.\n",
)
add(
    "offset-g53-machine-coordinates",
    "G53 is non-modal and bypasses the work offset and the G92 shift for one "
    "block. It keeps tool length compensation, because the IR stores a tip "
    "position and a segment where 'tip' meant something else would trap a field.",
    "G10 L2 P1 X-250. Y-100. Z0.\nG54 G0 X0. Y0.\nG53 G0 X-10. Y-10.\nG0 X0. Y0.\n",
)
add(
    "motion-g28-two-moves",
    "G28 travels to the reference point VIA an intermediate point given by the "
    "block's axis words. One straight move to the reference point can pass "
    "through the fixture.",
    "G0 X50. Y50.\nG1 Z-20. F300.\nG28 Z0.\n",
)
add(
    "units-change-mid-program",
    "G20/G21 can change partway, and the change affects feed rates and offsets "
    "as well as coordinates.",
    "G21 G0 X10. Y10.\nG20 G1 X1. F10.\nG21 G1 X20. F250.\n",
)
add(
    "distance-incremental",
    "G91: a delta is a delta in every frame, so incremental motion needs no "
    "offset chain at all.",
    "G90 G0 X10. Y20.\nG91 G1 X5. Y-5. F300.\nG1 X5.\nG90\n",
)
add(
    "feed-inverse-time",
    "G93 inverse time: F is a reciprocal duration rather than a distance rate, "
    "so it must not be scaled by the unit factor. The norm in 5-axis output.",
    "G93 G1 X10. Y10. F4.\nG1 X20. F2.\nG94 G1 X30. F500.\n",
)
add(
    "feed-per-revolution",
    "G95 feed per revolution, which needs the spindle speed to become a duration.",
    "G95 G1 X10. F0.1\nG94\n",
)

# --- comments, block skip, subprograms -------------------------------------

add(
    "comment-forms",
    "Both comment forms in one file, including an inline comment that must not "
        "swallow the words following it on the same line.",
    "G0 X10. (rapid across) Y20.\nG1 Z-1. F300. ; plunge\n",
)
add(
    "comment-unbalanced",
    "An unbalanced parenthesis. Illegal, common in the wild, and a warning "
    "rather than a refusal because the file runs on the machine.",
    "G0 X10. (never closed\nG0 Y20.)\nG1 Z-1. F300.\n",
)
add(
    "block-skip",
    "A leading slash marks a block as optional, conditional on a control switch "
    "we do not have. The policy is a flag and is recorded in the IR header.",
    "G0 X10.\n/G0 X20.\nG0 X30.\n",
)
add(
    "subprogram-call",
    "M98 calls a body and M99 returns. Execution stops at M30 rather than "
    "falling into the bodies that follow it.",
    "F300.\nM98 P200 L2\nG0 X50.\n",
    tail="M30\nO200\nG91 G1 X10.\nG90\nM99\n%\n",
)

# --- files that must be rejected ------------------------------------------

REJECT = [
    (
        "reject-cutter-comp-g41",
        "cutter-compensation",
        "G41 asks the control to offset the path by the tool radius, including "
        "lead-in and corner geometry that differs between controls. Simulating "
        "the uncompensated path would be wrong by the tool radius everywhere.",
        "G41 D1 G1 X10. F300.\n",
    ),
    (
        "reject-macro-variable",
        "macro-programming",
        "Coordinates that come out of arithmetic the parser guessed at would "
        "make the verification meaningless.",
        "#100 = 5.0\nG1 X#100 F300.\n",
    ),
    (
        "reject-macro-if",
        "macro-programming",
        "Flow control is macro programming: common in shop-written programs and "
        "absent from CAM output, so refusing it costs little and guessing costs much.",
        "IF [#1 GT 5] GOTO 100\n",
    ),
    (
        "reject-o-word",
        "o-word",
        "LinuxCNC's procedural extension, not Fanuc RS-274. M98/M99 are supported.",
        "o100 sub\no100 endsub\n",
    ),
    (
        "reject-missing-decimal",
        "missing-decimal-point",
        "On a legacy control X10 means 0.010 mm. A factor of a thousand that "
        "parses perfectly either way.",
        "G0 X10\n",
    ),
    (
        "reject-modal-conflict",
        "modal-group-conflict",
        "Two motion codes in one block. Which one a real control performs "
        "depends on the control, so there is no safe reading.",
        "G0 G1 X10. F300.\n",
    ),
    (
        "reject-arc-r-full-circle",
        "full-circle-with-radius-word",
        "Coincident endpoints with an R word name no particular circle: every "
        "circle of that radius through the point qualifies.",
        "G0 X10. Y0.\nG1 F300.\nG2 X10. Y0. R10.\n",
    ),
    (
        "reject-arc-radius-mismatch",
        "arc-radius-mismatch",
        "A centre half a millimetre inconsistent between the endpoints, far "
        "beyond ordinary CAM rounding.",
        "G0 X10. Y0.\nG1 F300.\nG3 X0. Y10.5 I-10. J0.\n",
    ),
    (
        "reject-arc-ill-conditioned",
        "arc-ill-conditioned",
        "An R-form arc sweeping almost exactly 180 degrees, where the centre is "
        "not determined by the endpoints. No tolerance rescues it.",
        "G0 X-9.999 Y0.\nG1 F300.\nG3 X9.999 Y0. R10.\n",
    ),
    (
        "reject-no-feed-rate",
        "no-feed-rate",
        "A feed move before any F word has been commanded. Treating a missing feed "
        "as zero would divide by it later.",
        "G1 X10.\n",
    ),
    (
        "reject-unknown-subprogram",
        "unknown-subprogram",
        "M98 naming a body that does not exist. Falling through silently would "
        "run the wrong geometry rather than none.",
        "M98 P999\n",
    ),
    (
        "reject-subprogram-recursion",
        "subprogram-too-deep",
        "A subprogram that calls itself would recurse until the stack gave out, "
        "which is a crash rather than a diagnosis.",
        "M98 P300\n",
    ),
    (
        "reject-unsupported-code",
        "unsupported-code",
        "G65 is a macro call, which needs the parametric evaluation this parser "
        "refuses. Naming it beats ignoring it.",
        "G65 P1000\n",
    ),
    (
        "reject-g54-1",
        "unsupported-code",
        "Fanuc's extended offsets are addressed by a P word and are a different "
        "mechanism. A range like 540..=593 would silently treat this as G54.",
        "G54.1 P3 G0 X0.\n",
    ),
]
for name, error, why, body in REJECT:
    tail = "M30\n%\n"
    if name == "reject-subprogram-recursion":
        tail = "M30\nO300\nM98 P300\nM99\n%\n"
    add(name, why, body, expect="reject", error=error, tail=tail)

# Foreign languages need no preamble -- they are not this language at all.
for name, error, why, text in [
    (
        "reject-siemens-cycle",
        "foreign-language",
        "Siemens 840D is a different language, not a dialect. Refusing by name "
        "beats a syntax error at line 3.",
        "N10 G17 G90 G54\nN20 CYCLE81(10,0,2,-15,,)\nN30 M30\n",
    ),
    (
        "reject-siemens-rparam",
        "foreign-language",
        "An R-parameter assignment. The `=` is what tells it from an arc radius.",
        "N10 R1=45.0\nN20 G1 X=R1 F500\n",
    ),
    (
        "reject-heidenhain",
        "foreign-language",
        "Heidenhain Klartext, which is a different language rather than a "
        "dialect of RS-274 and is refused by name.",
        "0 BEGIN PGM TEST MM\n1 TOOL CALL 5 Z S2000\n2 END PGM TEST MM\n",
    ),
]:
    path = os.path.join(CORPUS, name + ".nc")
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    entries[name] = {"_why": why, "expect": "reject", "error": error}


doc = {
    "schema": "chipbreaker.gcode-corpus",
    "version": 1,
    "note": (
        "Generated by scripts/gcode_corpus.py. `expect` is parse or reject; for "
        "a rejection, `error` is the GcodeError::kind() that must be produced. "
        "See README.md for the arc form amendment."
    ),
    "entries": {k: entries[k] for k in sorted(entries)},
}
with io.open(
    os.path.join(CORPUS, "expectations.json"), "w", encoding="utf-8", newline="\n"
) as f:
    json.dump(doc, f, indent=2, sort_keys=True)
    f.write("\n")

print(f"{len(entries)} corpus entries written")
