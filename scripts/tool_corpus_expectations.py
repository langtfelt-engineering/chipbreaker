"""Compute tool-corpus expectations from textbook formulas.

Deliberately not written in Rust and deliberately not calling Chipbreaker: the
point of an expectation file is to be an *independent* statement of the answer,
and one produced by the code under test is only a statement that the code agrees
with itself.

Every volume is additionally cross-checked against numerical quadrature of
`pi * integral r(z)^2 dz`, to 1e-9 relative. Two things were needed to make that
check mean anything, and both were found by the check failing rather than by
foresight:

*   **Integrate piecewise.** Simpson converges as `h^4` on a smooth interval and
    barely at all across a jump. Integrating the tapered mill straight through
    the step up to its shank reported a 2e-6 disagreement that was entirely the
    quadrature's own error.

*   **Integrate an arc in its angle, not in z.** Where the profile is a torus,
    `r(z)^2` contains `sqrt(rho^2 - (z - cz)^2)`, whose derivative is infinite at
    the ends of the arc; Simpson is poor there and the bull nose came out 3e-7
    off. Substituting `z = cz + rho sin(theta)` makes the integrand a
    trigonometric polynomial and the error vanishes. A sphere needs no such
    treatment: with its centre on the axis, `r^2 = rho^2 - (z - cz)^2` is a
    polynomial already.

Regenerate with:

    python scripts/tool_corpus_expectations.py > tests/corpus/tool/expectations.json
"""

import json
import math
import sys

PI = math.pi


def simpson(f, a, b, n=200_000):
    """Composite Simpson over a single smooth interval."""
    if n % 2:
        n += 1
    h = (b - a) / n
    total = f(a) + f(b)
    for i in range(1, n):
        total += f(a + i * h) * (4 if i % 2 else 2)
    return total * h / 3


def piece_z(radius_of_z, z0, z1):
    """Volume of a piece over which `r(z)` is smooth."""
    return PI * simpson(lambda z: radius_of_z(z) ** 2, z0, z1)


def piece_arc(cr, rho, theta0, theta1):
    """Volume swept by a circular arc, integrated in its own angle.

    With `r = cr + rho cos t` and `dz = rho cos t dt` the integrand is a
    trigonometric polynomial, so Simpson is effectively exact.
    """
    return PI * simpson(
        lambda t: (cr + rho * math.cos(t)) ** 2 * rho * math.cos(t), theta0, theta1
    )


def check(name, closed, pieces, tol=1e-9):
    """Assert the closed form agrees with the sum of the numeric pieces."""
    numeric = sum(pieces)
    rel = abs(closed - numeric) / max(abs(closed), 1.0)
    assert rel < tol, f"{name}: closed {closed} vs numeric {numeric} (rel {rel:e})"
    return closed


def cyl(r, h):
    return PI * r * r * h


def frustum(r_bottom, r_top, h):
    return PI * h * (r_bottom**2 + r_bottom * r_top + r_top**2) / 3.0


def frustum_lateral(r_bottom, r_top, h):
    return PI * (r_bottom + r_top) * math.hypot(h, r_top - r_bottom)


tools = {}

# --- flat-6: a plain cylinder, r = 3, h = 50 --------------------------------
tools["flat-6"] = dict(
    volume=check("flat-6", cyl(3, 50), [piece_z(lambda z: 3.0, 0, 50)]),
    area=2 * PI * 3 * 50 + 2 * PI * 9,
    diameter=6.0,
    total_length=50.0,
    cutting_length=20.0,
    formula="cylinder r=3 h=50; area = side + two discs",
)

# --- ball-6: hemisphere r = 3, then a cylinder to z = 50 --------------------
tools["ball-6"] = dict(
    volume=check(
        "ball-6",
        (2 / 3) * PI * 27 + cyl(3, 47),
        [
            # Centre on the axis, so r^2 is a polynomial and z is a fine variable.
            piece_z(lambda z: math.sqrt(max(9 - (3 - z) ** 2, 0.0)), 0, 3),
            piece_z(lambda z: 3.0, 3, 50),
        ],
    ),
    area=2 * PI * 9 + 2 * PI * 3 * 47 + PI * 9,
    diameter=6.0,
    total_length=50.0,
    cutting_length=20.0,
    formula="hemisphere r=3 + cylinder r=3 h=47",
)

# --- bull-10-r2: flat to r=3, a 2 mm corner torus to r=5, then a necked shank
#
# Under the corner r(z) = 3 + sqrt(4 - (2 - z)^2). Substituting u = 2 - z and
# using integral of sqrt(a^2 - u^2) over [0, a] = pi a^2 / 4 gives the closed
# form below.
corner = PI * (26 + 6 * PI - 8 / 3)
tools["bull-10-r2"] = dict(
    volume=check(
        "bull-10-r2",
        corner + cyl(5, 28) + cyl(4, 30),
        [
            piece_arc(3.0, 2.0, -PI / 2, 0.0),
            piece_z(lambda z: 5.0, 2, 30),
            piece_z(lambda z: 4.0, 30, 60),
        ],
    ),
    # Bottom annulus r 0..3, torus band by Pappus, side, step annulus r 4..5,
    # shank, top disc.
    area=PI * 9
    + (6 * PI**2 + 8 * PI)
    + 2 * PI * 5 * 28
    + PI * 9
    + 2 * PI * 4 * 30
    + PI * 16,
    diameter=10.0,
    total_length=60.0,
    cutting_length=30.0,
    formula="flat r<=3, quarter torus R=3 rho=2, cylinder r=5 to z=30, shank r=4 to z=60",
)

# --- chamfer-8-90: 1 mm flat tip, 45 degree half angle, to r = 4 -----------
CH = 3.5  # (4 - 0.5) / tan(45)
tools["chamfer-8-90"] = dict(
    volume=check(
        "chamfer-8-90",
        frustum(0.5, 4.0, CH) + cyl(4, 55 - CH),
        [
            piece_z(lambda z: 0.5 + z, 0, CH),
            piece_z(lambda z: 4.0, CH, 55),
        ],
    ),
    area=PI * 0.25 + frustum_lateral(0.5, 4.0, CH) + 2 * PI * 4 * (55 - CH) + PI * 16,
    diameter=8.0,
    total_length=55.0,
    cutting_length=20.0,
    formula="disc r=0.5, frustum 0.5->4 over 3.5, cylinder r=4 to z=55",
)

# --- vbit-8-60: pointed, 30 degree half angle ------------------------------
VH = 4.0 / math.tan(math.radians(30))
tools["vbit-8-60"] = dict(
    volume=check(
        "vbit-8-60",
        cyl(4, VH) / 3 + cyl(4, 55 - VH),
        [
            piece_z(lambda z: (4.0 / VH) * z, 0, VH),
            piece_z(lambda z: 4.0, VH, 55),
        ],
    ),
    area=PI * 4 * math.hypot(4, VH) + 2 * PI * 4 * (55 - VH) + PI * 16,
    diameter=8.0,
    total_length=55.0,
    cutting_length=20.0,
    formula="cone r=4 h=4/tan(30), cylinder r=4 to z=55",
)

# --- taper-3deg: 1 mm tip radius, 3 degrees per side, 20 mm flute ----------
TAN3 = math.tan(math.radians(3))
TOP = 1.0 + 20.0 * TAN3
tools["taper-3deg"] = dict(
    volume=check(
        "taper-3deg",
        frustum(1.0, TOP, 20) + cyl(4, 35),
        [
            piece_z(lambda z: 1.0 + z * TAN3, 0, 20),
            piece_z(lambda z: 4.0, 20, 55),
        ],
    ),
    area=PI * 1.0
    + frustum_lateral(1.0, TOP, 20)
    + PI * (16 - TOP**2)
    + 2 * PI * 4 * 35
    + PI * 16,
    diameter=8.0,
    total_length=55.0,
    cutting_length=20.0,
    formula="disc r=1, frustum 1->1+20 tan3 over 20, step to r=4, cylinder to z=55",
)

# --- drill-6-118: 59 degree half angle point -------------------------------
DH = 3.0 / math.tan(math.radians(59))
tools["drill-6-118"] = dict(
    volume=check(
        "drill-6-118",
        cyl(3, DH) / 3 + cyl(3, 50 - DH),
        [
            piece_z(lambda z: (3.0 / DH) * z, 0, DH),
            piece_z(lambda z: 3.0, DH, 50),
        ],
    ),
    area=PI * 3 * math.hypot(3, DH) + 2 * PI * 3 * (50 - DH) + PI * 9,
    diameter=6.0,
    total_length=50.0,
    cutting_length=30.0,
    formula="cone r=3 h=3/tan(59), cylinder r=3 to z=50",
)

# --- barrel-12-r200: one 200 mm arc through the tip to r = 6 ---------------
#
# Its centre sits at r = 6 - 200 = -194, past the axis. That is the case which
# sent every barrel down the sphere branch of the ray caster until it was tested
# the magnitude of the major radius rather than its sign.
R_ARC, R_TOOL = 200.0, 6.0
CR = R_TOOL - R_ARC
CZ = math.sqrt(R_ARC**2 - CR**2)
THETA0 = -math.asin(CZ / R_ARC)


def arc_antiderivative(theta):
    """Antiderivative of `r^2 dz` along the circle, evaluated at `theta`."""
    return R_ARC * (
        CR**2 * math.sin(theta)
        + 2 * CR * R_ARC * (theta / 2 + math.sin(2 * theta) / 4)
        + R_ARC**2 * (math.sin(theta) - math.sin(theta) ** 3 / 3)
    )


barrel_nose = PI * (arc_antiderivative(0.0) - arc_antiderivative(THETA0))
tools["barrel-12-r200"] = dict(
    volume=check(
        "barrel-12-r200",
        barrel_nose + cyl(6, 90 - CZ),
        [
            piece_arc(CR, R_ARC, THETA0, 0.0),
            piece_z(lambda z: 6.0, CZ, 90),
        ],
    ),
    area=None,  # the arc's Pappus term is pinned by the unit tests instead
    diameter=12.0,
    total_length=90.0,
    cutting_length=60.0,
    formula="arc R=200 centred at r=-194, tip to widest point, then cylinder r=6",
)

# --- held-12: a 12 mm flat in a two-stage shrink holder --------------------
tools["held-12"] = dict(
    volume=check(
        "held-12",
        cyl(6, 60) + cyl(16, 28) + frustum(16, 25, 22),
        [
            piece_z(lambda z: 6.0, 0, 60),
            piece_z(lambda z: 16.0, 60, 88),
            piece_z(lambda z: 16.0 + 9.0 * (z - 88) / 22.0, 88, 110),
        ],
    ),
    area=None,
    diameter=50.0,  # the holder is the widest part, not the cutter
    total_length=110.0,
    cutting_length=30.0,
    formula="cylinder r=6 h=60, holder cylinder r=16 h=28, holder frustum 16->25 over 22",
)

# --- the collet-chuck tools -------------------------------------------------
#
# Stacks of plain cylinders, so the closed forms are elementary and the
# quadrature cross-check is exact rather than merely close. The interest is not
# in the arithmetic; it is that these carry real ER dimensions, and that one
# radius in each needs all seventeen significant digits to round-trip. A corpus
# built from catalogue-round numbers alone would pass through a parser that
# truncates and never notice.
ER16_NUT_R = 26.987499999999997 / 2  # 1 1/16 in
ER16_BODY_R = 34.925 / 2  # 1 3/8 in
ER32_NUT_R = 50.8 / 2  # 2 in
ER32_BODY_R = 61.912499999999994 / 2  # 2 7/16 in


def collet_tool(name, shank_r, shank_h, nut_r, nut_h, body_r, body_h, cutting):
    """A cutter and shank of one diameter under a two-stage chuck."""
    z1 = shank_h + nut_h
    z2 = z1 + body_h
    return dict(
        volume=check(
            name,
            cyl(shank_r, shank_h) + cyl(nut_r, nut_h) + cyl(body_r, body_h),
            [
                piece_z(lambda z: shank_r, 0, shank_h),
                piece_z(lambda z: nut_r, shank_h, z1),
                piece_z(lambda z: body_r, z1, z2),
            ],
        ),
        area=None,
        diameter=2 * max(shank_r, nut_r, body_r),
        total_length=z2,
        cutting_length=cutting,
        formula=(
            f"cylinder r={shank_r} h={shank_h}, nut r={nut_r} h={nut_h}, "
            f"body r={body_r} h={body_h}"
        ),
    )


tools["er16-flat-6"] = collet_tool(
    "er16-flat-6", 3.0, 50.0, ER16_NUT_R, 21.0, ER16_BODY_R, 41.0, 20.0
)
tools["er32-stub-6"] = collet_tool(
    "er32-stub-6", 3.0, 28.0, ER32_NUT_R, 28.0, ER32_BODY_R, 50.0, 10.0
)
tools["long-reach-6"] = collet_tool(
    "long-reach-6", 3.0, 95.0, ER16_NUT_R, 21.0, ER16_BODY_R, 41.0, 20.0
)

doc = {
    "schema": "chipbreaker.tool-corpus-expectations",
    "version": 1,
    "note": (
        "Computed from textbook formulas independently of Chipbreaker, and every "
        "volume cross-checked to 1e-9 against piecewise Simpson quadrature of "
        "pi * integral r(z)^2 dz, with arcs integrated in their own angle. "
        "A null field is one this file does not pin. "
        "Regenerate with scripts/tool_corpus_expectations.py."
    ),
    "tools": {k: tools[k] for k in sorted(tools)},
}
# Write LF explicitly. Python's `print` translates to CRLF on Windows, so
# without this the corpus file's line endings depend on who last regenerated it.
sys.stdout.reconfigure(newline="\n")
print(json.dumps(doc, indent=2, sort_keys=True))
