"""Nested DC-motor rig: every printed part, parametric in build123d.

Run from the repo root:
    python cad/rig.py                 # carrier = A, inner = B  (default)
    python cad/rig.py --carrier B     # role-swap configuration
    python cad/rig.py --carrier both  # both configurations
    python cad/rig.py --no-check      # skip the (slower) clearance checks

Each configuration is written to cad/exports/carrier_<X>/ as print-oriented
STLs plus assembly STEPs. cad/exports/rig_assembly.step is the colored
full-rig assembly, and docs/BOM.md is regenerated from the geometry.

Mechanical concept (details and reasoning in docs/DESIGN_NOTES.md):

    carrier motor ─ coupler hub ─ Oldham disc ─┐
                                                rear hub ── rear bearing (floating)
                                                slip ring rotor
                                                rear cap ═╗  (bolted)
                                                cradle body ╝ ── front bearing (locating)
                                                  └ inner motor, keyed by its flats
                                                       └ output shaft ─ encoder disc / dyno drum

The cradle is supported only by its two bearings. The carrier delivers torque
alone through the Oldham coupling, which tolerates the small misalignment
between the carrier's shaft and the cradle axis. So the axis is never held by
three bearings at once.
"""

import argparse
import copy
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

from build123d import Align, Box, Color, Compound, Cone, Cylinder, Pos, Rot, export_step, export_stl

import parameters as p

EXPORT_DIR = Path(__file__).parent / "exports"
BOM_PATH = p.ROOT / "docs" / "BOM.md"

ALIGN_MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)
ALIGN_MIN = (Align.MIN, Align.MIN, Align.MIN)


# ---------------------------------------------------------------------------
# Primitives. Rotating parts are built in a LOCAL frame whose +Z is the spin
# axis with z == rig x, then mapped onto the rig axis by `to_rig`.
# ---------------------------------------------------------------------------
def zcyl(r, z0, z1, arc=360):
    return Pos(0, 0, z0) * Cylinder(r, z1 - z0, arc_size=arc, align=ALIGN_MIN_Z)


def zcone(r0, r1, z0, z1):
    return Pos(0, 0, z0) * Cone(r0, r1, z1 - z0, align=ALIGN_MIN_Z)


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=ALIGN_MIN)


def zprofile(dia, flat, z0, z1):
    """Motor-can cross-section: a circle trimmed by two flats normal to local X
    (which maps to rig Z, so flats end up top and bottom)."""
    c = zcyl(dia / 2, z0, z1)
    if flat > 0:
        c = c & box(-flat / 2, flat / 2, -dia, dia, z0, z1)
    return c


def windows(ri, ro, z0, z1, count, width_deg):
    out = None
    for i in range(count):
        w = Rot(0, 0, i * 360 / count - width_deg / 2) * (zcyl(ro, z0, z1, width_deg) - zcyl(ri, z0, z1))
        out = w if out is None else out + w
    return out


def at_polar(r, deg, shape):
    a = math.radians(deg)
    return Pos(r * math.cos(a), r * math.sin(a), 0) * shape


def to_rig(local, H):
    return Pos(0, 0, H) * Rot(0, 90, 0) * local


def vcyl(r, x, y, z0, z1):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=ALIGN_MIN_Z)


def vslot(r, x, y, z0, z1, travel=0.0, along="x"):
    """Vertical stadium slot (or plain hole when travel == 0)."""
    if travel == 0:
        return vcyl(r, x, y, z0, z1)
    dx, dy = (travel, 0) if along == "x" else (0, travel)
    s = vcyl(r, x - dx, y - dy, z0, z1) + vcyl(r, x + dx, y + dy, z0, z1)
    if along == "x":
        return s + box(x - dx, x + dx, y - r, y + r, z0, z1)
    return s + box(x - r, x + r, y - dy, y + dy, z0, z1)


def ycyl(r, x, z, y0, y1):
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=ALIGN_MIN_Z)


def insert_hole_up(x, y, z_face):
    """Heat-set insert pilot entering a part's BOTTOM face (base-plate screws come from below)."""
    return vcyl(p.hole(p.M3_INSERT_HOLE_DIA) / 2, x, y, z_face, z_face + p.M3_INSERT_DEPTH)


def insert_hole_down(x, y, z_face):
    return vcyl(p.hole(p.M3_INSERT_HOLE_DIA) / 2, x, y, z_face - p.M3_INSERT_DEPTH, z_face)


def screw_from_top(x, y, z_bottom, z_top):
    """M3 clearance hole with a counterbore opening on the top face."""
    return vcyl(p.hole(p.M3_CLEARANCE_DIA) / 2, x, y, z_bottom - 0.1, z_top + 0.1) + vcyl(
        p.hole(p.M3_HEAD_DIA) / 2, x, y, z_top - p.M3_HEAD_HEIGHT - 0.3, z_top + 0.1
    )


SCREW_LENGTHS = [6, 8, 10, 12, 16, 20, 25, 30]


def screw_length(grip):
    """Shortest standard M3 that reaches at least 4 mm into an insert below a `grip`-thick clamped layer."""
    for length in SCREW_LENGTHS:
        if length >= grip + 4.0 and length <= grip + p.M3_INSERT_DEPTH - 0.3 + 1e-6:
            return length
    return next(l for l in SCREW_LENGTHS if l >= grip + 4.0)


# ---------------------------------------------------------------------------
# Layout: every axial station and the axis height, derived from the motors.
# ---------------------------------------------------------------------------
class DesignError(ValueError):
    pass


def make_layout(carrier: p.MotorDims, inner: p.MotorDims) -> SimpleNamespace:
    L = SimpleNamespace(carrier=carrier, inner=inner, fasteners={})

    # --- cradle body around the inner motor; x = 0 is the body / rear-cap joint
    L.front_bearing = p.pick_front_bearing(inner)
    L.dF, L.DF, L.BF = p.BEARINGS[L.front_bearing]
    L.rear_bearing = p.REAR_BEARING
    L.dR, L.DR, L.BR = p.BEARINGS[L.rear_bearing]

    L.pocket_d, L.pocket_flat = p.pocket_dia(inner), p.pocket_flat(inner)
    L.r_journal = L.dF / 2 + p.LOCATING_JOURNAL_FIT
    if L.r_journal - L.pocket_d / 2 < p.MIN_WALL - 1e-6:
        raise DesignError(f"Cradle wall under {p.MIN_WALL} mm with {L.front_bearing}; set FRONT_BEARING larger.")
    L.pocket_depth = inner.can_length - p.AXIAL_CLAMP_PRELOAD
    L.body_front = L.pocket_depth + p.CRADLE_LIP
    L.fb_x1 = L.body_front - 0.5
    L.fb_x0 = L.fb_x1 - L.BF
    L.r_shoulder = (L.dF + 0.25 * (L.DF - L.dF)) / 2
    L.r_bolt = L.pocket_d / 2 + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    L.r_flange = L.r_bolt + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    flange_cone_end = p.CRADLE_FLANGE_T + (L.r_flange - L.r_journal)
    if flange_cone_end > L.fb_x0 - 1.5 - 0.5:
        raise DesignError("Inner motor too short for flange + front bearing on the cradle body.")

    # --- rear cap, slip ring, rear bearing, Oldham, coupler (all at x < 0)
    flat_cap = (inner.flat_width - 2) / 2 if inner.flat_width > 0 else 1e9
    L.r_cavity = min(0.4 * inner.can_dia, flat_cap)
    L.cavity_depth = inner.rear_clearance + 0.5
    L.cap_t = L.cavity_depth + p.CAP_WALL
    L.r_seat = p.SLIP_RING_BORE / 2 + p.SLIP_RING_SEAT_FIT
    L.r_rear_journal = L.dR / 2 + p.FLOATING_JOURNAL_FIT
    if L.r_seat <= L.r_rear_journal + 0.1:
        raise DesignError("SLIP_RING_BORE must exceed the rear bearing bore so the ring can slide over the journal.")
    L.r_wire = L.r_seat + 1.1 + 0.6
    L.ring_ri, L.ring_ro = L.r_flange, L.r_flange + p.ENCODER_RING_WIDTH
    L.cradle_beam_r = (L.ring_ri + 3.5 + L.ring_ro - 1.0) / 2

    L.sr_x1 = -L.cap_t - p.SLIP_RING_WIRE_GAP
    L.sr_x0 = L.sr_x1 - p.SLIP_RING_LENGTH
    L.rp_x1 = L.sr_x0 - p.AXIAL_GAP
    L.rb_x1 = L.rp_x1 - p.PEDESTAL_LIP
    L.rb_x0 = L.rb_x1 - L.BR
    L.rp_x0 = L.rb_x0 - p.PEDESTAL_LIP
    L.hub_end = L.rp_x0 - p.AXIAL_GAP - p.OLDHAM_SLOT_DEPTH
    L.slot_w = p.hole(p.OLDHAM_TONGUE_W + 2 * p.OLDHAM_CLEARANCE)

    L.od_x1 = L.hub_end - 0.5  # Oldham web, +X face
    L.od_x0 = L.od_x1 - p.OLDHAM_WEB_T
    L.ch_x1 = L.od_x0 - 0.5    # coupler hub, +X end
    L.carrier_wall = max(2.5, carrier.boss_length + 0.8)
    L.engagement = carrier.boss_length + carrier.shaft_length - L.carrier_wall - p.AXIAL_GAP
    if L.engagement < p.MIN_SHAFT_ENGAGEMENT:
        raise DesignError(f"Carrier shaft only engages {L.engagement:.1f} mm in the coupler hub.")
    L.ch_x0 = L.ch_x1 - (L.engagement + p.OLDHAM_SLOT_DEPTH + 0.5)
    L.set_screw = L.engagement / 2 - 2.9 >= 0.1
    L.cw_x1 = L.ch_x0 - p.AXIAL_GAP      # carrier mount front wall, +X face
    L.xA = L.cw_x1 - L.carrier_wall       # carrier front face
    L.mount_len = max(12.0, carrier.can_length - 3.0)

    # --- output side: encoder disc / dyno drum on the inner motor's shaft
    L.fp_x0 = L.fb_x0 - p.PEDESTAL_LIP
    L.fp_x1 = L.fb_x1 + p.PEDESTAL_LIP
    L.out_hub_x0 = L.fp_x1 + p.AXIAL_GAP
    L.disc_x0 = L.fp_x1 + p.AXIAL_GAP + max(
        p.SENSOR_ARM_THICKNESS + (p.SENSOR_SLOT_WIDTH - p.ENCODER_RING_T) / 2,
        p.SENSOR_PCB_WIDTH / 2 - p.ENCODER_RING_T / 2,
        p.PEDESTAL_FOOT_EXT + BRACKET_HALF_W + 0.5 - p.AXIAL_GAP - p.ENCODER_RING_T / 2,  # bracket foot vs pedestal foot
    )
    L.disc_x1 = L.disc_x0 + p.ENCODER_RING_T
    L.out_hub_len = L.disc_x0 - L.out_hub_x0
    L.inner_shaft_x0 = L.pocket_depth + inner.boss_length
    L.inner_shaft_x1 = L.inner_shaft_x0 + inner.shaft_length
    out_engagement = min(L.inner_shaft_x1, L.disc_x1) - L.out_hub_x0
    if out_engagement < p.MIN_SHAFT_ENGAGEMENT:
        raise DesignError(f"Inner shaft only engages {out_engagement:.1f} mm in the output disc hub.")
    L.disc_r = p.OUTPUT_DISC_DIA / 2
    L.win_ri = p.DYNO_DRUM_DIA / 2 + p.SENSOR_ARM_PAST_BEAM + 1.0 + 1.0  # arm tips stay 1 mm off the drum
    L.out_beam_r = (L.win_ri + L.disc_r - 1.5) / 2
    if L.out_beam_r - L.win_ri < 1.0:
        raise DesignError("OUTPUT_DISC_DIA too small for windows outside the dyno drum.")

    # --- Phase 1: dyno drum on the carrier motor's own shaft (cradle removed)
    L.p1_hub_x0 = L.ch_x0
    L.p1_disc_x0 = L.p1_hub_x0 + L.out_hub_len
    L.p1_disc_c = L.p1_disc_x0 + p.ENCODER_RING_T / 2

    # --- axis height
    L.H = math.ceil(max(
        L.DF / 2 + p.M3_INSERT_DEPTH + 2,                   # front pedestal: room for a bottom insert
        L.DR / 2 + p.M3_INSERT_DEPTH + 2,
        L.ring_ro + p.RADIAL_GAP + 2,                       # cradle encoder ring over the base plate
        p.SLIP_RING_OD / 2 + p.RADIAL_GAP + 2,
        (L.pocket_flat or L.pocket_d) / 2 + p.M3_INSERT_DEPTH + 2,
        p.DYNO_DRUM_DIA / 2 + 1.5 + p.RADIAL_GAP + 2,
        L.disc_r + p.RADIAL_GAP + 2,
    ))
    return L


# ---------------------------------------------------------------------------
# Rotating parts (local frame, z == rig x)
# ---------------------------------------------------------------------------
def cradle_body(L):
    """Holds the inner motor: keyed pocket, centering lip, front-bearing journal, bolting flange."""
    body = zcyl(L.r_journal, 0, L.body_front)
    body += zcyl(L.r_flange, 0, p.CRADLE_FLANGE_T)
    # 45-degree cone under the flange: it prints front-face-down, so this is the overhang
    body += zcone(L.r_flange, L.r_journal, p.CRADLE_FLANGE_T, p.CRADLE_FLANGE_T + (L.r_flange - L.r_journal))
    body += zcyl(L.r_shoulder, L.fb_x0 - 1.5, L.fb_x0)  # flat shoulder the locating bearing seats against
    body -= zprofile(L.pocket_d, L.pocket_flat, -0.1, L.pocket_depth)
    boss_hole = p.hole(L.inner.boss_dia + 2 * p.BOSS_CENTERING_CLEARANCE)
    body -= zcyl(boss_hole / 2, L.pocket_depth - 0.1, L.body_front + 0.1)
    for a in (90, 210, 330):
        body -= at_polar(L.r_bolt, a, zcyl(p.hole(p.M3_INSERT_HOLE_DIA) / 2, -0.1, p.M3_INSERT_DEPTH))
    return body


def rear_cap(L):
    """Closes the cradle, clamps the inner motor, carries the slip-ring rotor,
    the floating rear bearing and the Oldham slot, plus the cradle encoder ring."""
    cap = zcyl(L.r_flange, -L.cap_t, 0)
    cap += zcyl(L.ring_ro, -p.ENCODER_RING_T, 0)
    cap -= windows(L.ring_ri + 3.5, L.ring_ro - 1.0, -p.ENCODER_RING_T - 0.1, 0.1, p.CRADLE_PPR, 25)
    cap += zcyl(L.r_seat, L.sr_x0, -L.cap_t + 0.01)
    cap += zcyl(L.r_rear_journal, L.hub_end, L.sr_x0 + 0.01)
    # terminal cavity, keyhole-extended (along local Y = round side of the can) to the two wire holes
    cap -= zcyl(L.r_cavity, -L.cavity_depth, 0.1)
    cap -= box(-1.5, 1.5, -(L.r_wire + 1.1), L.r_wire + 1.1, -L.cavity_depth, 0.1)
    for y in (-L.r_wire, L.r_wire):
        cap -= Pos(0, y, 0) * zcyl(1.1, -L.cap_t - 0.1, 0.1)
    for a in (90, 210, 330):
        cap -= at_polar(L.r_bolt, a, zcyl(p.hole(p.M3_CLEARANCE_DIA) / 2, -L.cap_t - 0.1, 0.1))
        cap -= at_polar(L.r_bolt, a, zcyl(p.hole(p.M3_HEAD_DIA) / 2, -L.cap_t - 0.1, -L.cap_t + p.M3_HEAD_HEIGHT + 0.3))
    r = L.r_rear_journal + 0.5
    cap -= box(-r, r, -L.slot_w / 2, L.slot_w / 2, L.hub_end - 0.1, L.hub_end + p.OLDHAM_SLOT_DEPTH)
    return cap


def oldham_disc(L):
    r = L.r_rear_journal
    disc = zcyl(r, L.od_x0, L.od_x1)
    disc += zcyl(r, L.od_x1, L.od_x1 + p.OLDHAM_TONGUE_DEPTH) & box(
        -r, r, -p.OLDHAM_TONGUE_W / 2, p.OLDHAM_TONGUE_W / 2, L.od_x1, L.od_x1 + p.OLDHAM_TONGUE_DEPTH)
    disc += zcyl(r, L.od_x0 - p.OLDHAM_TONGUE_DEPTH, L.od_x0) & box(
        -p.OLDHAM_TONGUE_W / 2, p.OLDHAM_TONGUE_W / 2, -r, r, L.od_x0 - p.OLDHAM_TONGUE_DEPTH, L.od_x0)
    return disc


def coupler_hub(L):
    """Clamps onto the carrier's shaft; its slot drives the Oldham disc."""
    r = p.HUB_OD / 2
    hub = zcyl(r, L.ch_x0, L.ch_x1)
    hub -= zcyl(L.carrier.shaft_dia / 2, L.ch_x0 - 0.1, L.ch_x1 - p.OLDHAM_SLOT_DEPTH - 0.5)  # drill to size
    hub -= box(-L.slot_w / 2, L.slot_w / 2, -r - 1, r + 1, L.ch_x1 - p.OLDHAM_SLOT_DEPTH, L.ch_x1 + 0.1)
    if L.set_screw:
        z = L.ch_x0 + L.engagement / 2
        nut = p.M3_SQUARE_NUT + 0.3
        y0 = L.carrier.shaft_dia / 2 + 1.0
        hub -= Pos(0, 0, z) * Rot(-90, 0, 0) * Cylinder(p.hole(p.M3_CLEARANCE_DIA) / 2, r + 1, align=ALIGN_MIN_Z)
        hub -= box(-nut / 2, r + 1, y0, y0 + p.M3_SQUARE_NUT_T + 0.3, z - nut / 2, z + nut / 2)
    return hub


def output_disc(L, shaft_dia, hub_x0, drum=False):
    """Encoder disc on a motor shaft. With drum=True a rope-brake drum is added
    beyond the disc — that is the dynamometer drum used in Phase 1 and the
    Phase 3 loaded sweep."""
    disc_x0 = hub_x0 + L.out_hub_len
    disc_x1 = disc_x0 + p.ENCODER_RING_T
    part = zcyl(shaft_dia / 2 + 2.5, hub_x0, disc_x0 + 0.01)
    part += zcyl(L.disc_r, disc_x0, disc_x1)
    part -= windows(L.win_ri, L.disc_r - 1.5, disc_x0 - 0.1, disc_x1 + 0.1, p.OUTPUT_PPR, 360 / p.OUTPUT_PPR / 2)
    end = disc_x1
    if drum:
        rd = p.DYNO_DRUM_DIA / 2
        part += zcyl(rd, disc_x1 - 0.01, disc_x1 + p.DYNO_DRUM_LENGTH)
        part += zcone(rd + 1.5, rd, disc_x1 + p.DYNO_DRUM_LENGTH, disc_x1 + p.DYNO_DRUM_LENGTH + 1.5)
        end = disc_x1 + p.DYNO_DRUM_LENGTH + 1.5
    part -= zcyl(shaft_dia / 2, hub_x0 - 0.1, end + 0.1)  # printed at nominal; drill to size
    return part


# ---------------------------------------------------------------------------
# Static parts (rig frame)
# ---------------------------------------------------------------------------
def split_bearing_block(L, xc, bearing_name, label):
    """Pedestal for one bearing, split at the axis so the bearing drops in.
    Returns (lower, cap, base_fastener_positions)."""
    d, D, B = p.BEARINGS[bearing_name]
    H, g = L.H, p.CLAMP_GAP
    w = B + 2 * p.PEDESTAL_LIP
    bolt_y = D / 2 + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    hy = bolt_y + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    top = H + D / 2 + 4.0

    lower = box(xc - w / 2, xc + w / 2, -hy, hy, 0, H - g / 2)
    lower += box(xc - w / 2 - p.PEDESTAL_FOOT_EXT, xc + w / 2 + p.PEDESTAL_FOOT_EXT, -hy, hy, 0, p.FOOT_HEIGHT)
    cap = box(xc - w / 2, xc + w / 2, -hy, hy, H + g / 2, top)

    seat = to_rig(zcyl(D / 2 + p.PRINTER_HOLE_UNDERSIZE + p.BEARING_SEAT_FIT, xc - B / 2, xc + B / 2), H)
    lip = to_rig(zcyl((d + 0.7 * (D - d)) / 2, xc - w, xc + w), H)
    lower, cap = lower - seat - lip, cap - seat - lip
    for y in (-bolt_y, bolt_y):
        lower -= insert_hole_down(xc, y, H - g / 2)
        cap -= screw_from_top(xc, y, H + g / 2, top)
    # base inserts sit directly under the cap-bolt inserts, far below them (H >> 2 x insert depth)
    feet = [(xc, -bolt_y), (xc, bolt_y)]
    for x, y in feet:
        lower -= insert_hole_up(x, y, 0)
    _count(L, f"M3 x {screw_length(top - (H + g / 2) - p.M3_HEAD_HEIGHT - 0.3)} SHCS (pedestal caps)", 2)
    _count(L, "M3 heat-set insert", 4)
    return lower, cap, [(x, y, "x", p.PEDESTAL_SLOT_TRAVEL) for x, y in feet]


def carrier_mount(L):
    """Split clamp around the carrier's can; the front wall centers its boss."""
    m, H, g = L.carrier, L.H, p.CLAMP_GAP
    pd, pf = p.pocket_dia(m), p.pocket_flat(m)
    half_h = (pf or pd) / 2
    bolt_y = pd / 2 + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    hy = bolt_y + p.M3_INSERT_HOLE_DIA / 2 + p.MIN_WALL
    x0, x1 = L.xA - L.mount_len, L.cw_x1
    top = H + half_h + 3.0

    lower = box(x0, x1, -hy, hy, 0, H - g / 2)
    cap = box(x0, x1, -hy, hy, H + g / 2, top)
    pocket = to_rig(zprofile(pd, pf, x0 - 0.1, L.xA), H)
    boss = to_rig(zcyl(p.hole(m.boss_dia + 2 * p.BOSS_CENTERING_CLEARANCE) / 2, L.xA - 0.1, x1 + 0.1), H)
    lower, cap = lower - pocket - boss, cap - pocket - boss
    bolt_x = [L.xA - 4.0, x0 + 4.0]
    for x in bolt_x:
        for y in (-bolt_y, bolt_y):
            lower -= insert_hole_down(x, y, H - g / 2)
            cap -= screw_from_top(x, y, H + g / 2, top)
    feet = [(x, 0) for x in bolt_x]
    for x, y in feet:
        lower -= insert_hole_up(x, y, 0)
    _count(L, f"M3 x {screw_length(top - (H + g / 2) - p.M3_HEAD_HEIGHT - 0.3)} SHCS (carrier mount cap)", 4)
    _count(L, "M3 heat-set insert", 6)
    return lower, cap, [(x, y, None, 0) for x, y in feet]


BRACKET_HALF_W = 6.0
BRACKET_PLATE_T = 4.0
BRACKET_FOOT_LEN = 18.0


def sensor_bracket(L, x_c, side, beam_r):
    """L-bracket holding a slot-sensor module so its beam crosses a disc at
    radius `beam_r`, at 3 o'clock (side=+1, +Y) or 9 o'clock (side=-1)."""
    H = L.H
    yf = beam_r + p.SENSOR_PCB_TO_BEAM + p.SENSOR_PCB_THICKNESS
    x0, x1 = x_c - BRACKET_HALF_W, x_c + BRACKET_HALF_W
    plate = box(x0, x1, yf, yf + BRACKET_PLATE_T, 0, H + p.SENSOR_SLOT_ADJUST + 6)
    foot = box(x0, x1, yf, yf + BRACKET_FOOT_LEN, 0, p.FOOT_HEIGHT)
    part = plate + foot
    r = p.hole(p.M3_CLEARANCE_DIA) / 2
    part -= ycyl(r, x_c, H - p.SENSOR_SLOT_ADJUST, yf - 1, yf + 5) + ycyl(r, x_c, H + p.SENSOR_SLOT_ADJUST, yf - 1, yf + 5)
    part -= box(x_c - r, x_c + r, yf - 1, yf + 5, H - p.SENSOR_SLOT_ADJUST, H + p.SENSOR_SLOT_ADJUST)
    fy = yf + BRACKET_PLATE_T + (BRACKET_FOOT_LEN - BRACKET_PLATE_T) / 2
    feet = [(x_c - 3.0, fy), (x_c + 3.0, fy)]
    for x, y in feet:
        part -= insert_hole_up(x, y, 0)
    module = box(x_c - p.SENSOR_PCB_WIDTH / 2, x_c + p.SENSOR_PCB_WIDTH / 2, yf - p.SENSOR_PCB_THICKNESS, yf, H - 6, H + 6)
    arm_y0 = beam_r - p.SENSOR_ARM_PAST_BEAM
    for sx in (-1, 1):
        a0 = x_c + sx * p.SENSOR_SLOT_WIDTH / 2
        a1 = a0 + sx * p.SENSOR_ARM_THICKNESS
        module += box(min(a0, a1), max(a0, a1), arm_y0, yf - p.SENSOR_PCB_THICKNESS + 0.01, H - 3, H + 3)
    if side < 0:
        mirror = Pos(x_c, 0, 0) * Rot(0, 0, 180) * Pos(-x_c, 0, 0)
        part, module = mirror * part, mirror * module
        feet = [(2 * x_c - x, -y) for x, y in feet]
    _count(L, "M3 heat-set insert", 2)
    return part, module, [(x, y, "y", p.BASE_SLOT_ADJUST) for x, y in feet]


def slip_ring_stop(L):
    """Post with a window that catches a pin on the slip-ring stator: blocks
    stator rotation only, so it cannot fight the bearings."""
    xc = (L.sr_x0 + L.sr_x1) / 2
    y1 = -(p.SLIP_RING_OD / 2 + p.RADIAL_GAP + 2.0)
    y0 = y1 - 8.0
    post = box(xc - 8, xc + 8, y0, y1, 0, L.H + 8)
    post -= box(xc - 2.2, xc + 2.2, y0 - 1, y1 + 1, L.H - 2.2, L.H + 2.2)
    feet = [(xc - 5.0, (y0 + y1) / 2), (xc + 5.0, (y0 + y1) / 2)]
    for x, y in feet:
        post -= insert_hole_up(x, y, 0)
    _count(L, "M3 heat-set insert", 2)
    return post, [(x, y, None, 0) for x, y in feet]


def base_plate(L, fasteners, footprints, cord_x):
    """Base plate: screws come up from below through counterbored holes or
    adjustment slots into inserts in each part's bottom face. `cord_x` lists
    drum stations that need rope-brake cord slots through the plate."""
    xs = [bb.min.X for bb in footprints] + [bb.max.X for bb in footprints]
    ys = [abs(bb.min.Y) for bb in footprints] + [abs(bb.max.Y) for bb in footprints]
    x0, x1 = min(xs) - p.BASE_MARGIN, max(xs) + p.BASE_MARGIN
    hy = max(ys) + p.BASE_MARGIN
    T = p.BASE_THICKNESS
    plate = box(x0, x1, -hy, hy, -T, 0)
    r, rh = p.hole(p.M3_CLEARANCE_DIA) / 2, p.hole(p.M3_HEAD_DIA) / 2
    for x, y, along, travel in fasteners:
        plate -= vslot(r, x, y, -T - 0.1, 0.1, travel, along or "x")
        plate -= vslot(rh, x, y, -T - 0.1, -T + p.M3_HEAD_HEIGHT + 0.3, travel, along or "x")
    rd = p.DYNO_DRUM_DIA / 2 + 1.0
    for xc in cord_x:
        for y in (-rd, rd):
            plate -= vslot(2.0, xc, y, -T - 0.1, 0.1, p.DYNO_DRUM_LENGTH / 2 - 2, "x")
    grip = T - p.M3_HEAD_HEIGHT - 0.3
    _count(L, f"M3 x {screw_length(grip)} SHCS (base plate, from below)", len(fasteners))
    return plate


def fit_coupon(L):
    """Small test pieces at the exact fit values, printed before the real parts:
    front/rear bearing seats, both journals, slip-ring seat, motor pocket slice."""
    pieces = []
    for i, (name, journal_r) in enumerate([(L.front_bearing, L.r_journal), (L.rear_bearing, L.r_rear_journal)]):
        d, D, B = p.BEARINGS[name]
        ring = zcyl(D / 2 + 4, 0, B + 1) - zcyl(D / 2 + p.PRINTER_HOLE_UNDERSIZE + p.BEARING_SEAT_FIT, 1, B + 1.1)
        ring -= zcyl((d + 0.7 * (D - d)) / 2, -0.1, 1.1)
        stub = zcyl(journal_r + 3, 0, 2) + zcyl(journal_r, 2, 2 + B + 1)
        pieces += [Pos(i * 50, 0, 0) * ring, Pos(i * 50 + 25, 0, 0) * stub]
    pieces.append(Pos(100, 0, 0) * (zcyl(L.r_seat + 3, 0, 2) + zcyl(L.r_seat, 2, 10)))
    slice_ = zcyl(L.r_journal, 0, 6) - zprofile(L.pocket_d, L.pocket_flat, p.CRADLE_LIP, 6.1)
    slice_ -= zcyl(p.hole(L.inner.boss_dia + 2 * p.BOSS_CENTERING_CLEARANCE) / 2, -0.1, 6.1)
    pieces.append(Pos(125, 0, 0) * slice_)
    return Compound(children=pieces)


def _count(L, item, qty):
    L.fasteners[item] = L.fasteners.get(item, 0) + qty


# ---------------------------------------------------------------------------
# Purchased hardware, modeled only for clearance checks
# ---------------------------------------------------------------------------
def motor_dummy(m, front_x, H):
    pd = m.can_dia
    can = to_rig(zprofile(pd, m.flat_width, front_x - m.can_length, front_x), H)
    can += to_rig(zcyl(m.boss_dia / 2, front_x - 0.01, front_x + m.boss_length), H)
    shaft = to_rig(zcyl(m.shaft_dia / 2, front_x + m.boss_length, front_x + m.boss_length + m.shaft_length), H)
    return can, shaft


def bearing_dummy(name, x0, H):
    d, D, B = p.BEARINGS[name]
    return to_rig(zcyl(D / 2, x0, x0 + B) - zcyl(d / 2, x0 - 0.1, x0 + B + 0.1), H)


# ---------------------------------------------------------------------------
# Assemblies
# ---------------------------------------------------------------------------
@dataclass
class Item:
    name: str
    shape: object
    group: str            # "static", "carrier" (spins with the cradle), "output" (output shaft speed)
    printed: bool = True


@dataclass
class Build:
    L: SimpleNamespace
    phase3: list = field(default_factory=list)
    phase3_loaded: list = field(default_factory=list)
    phase1: list = field(default_factory=list)
    exports: dict = field(default_factory=dict)  # file stem -> (shape, print rotation)


def build(carrier: p.MotorDims, inner: p.MotorDims) -> Build:
    L = make_layout(carrier, inner)
    H = L.H
    b = Build(L)

    body, cap = cradle_body(L), rear_cap(L)
    oldham, hub = oldham_disc(L), coupler_hub(L)
    enc = output_disc(L, inner.shaft_dia, L.out_hub_x0)
    drum_inner = output_disc(L, inner.shaft_dia, L.out_hub_x0, drum=True)
    drum_carrier = output_disc(L, carrier.shaft_dia, L.p1_hub_x0, drum=True)
    _count(L, f"M3 x {screw_length(L.cap_t - p.M3_HEAD_HEIGHT - 0.3)} SHCS (rear cap to cradle)", 3)
    _count(L, "M3 heat-set insert", 3)
    if L.set_screw:
        _count(L, "M3 x 6 set screw + M3 square nut (coupler hub)", 1)

    fp_lower, fp_cap, fp_feet = split_bearing_block(L, (L.fb_x0 + L.fb_x1) / 2, L.front_bearing, "front")
    rp_lower, rp_cap, rp_feet = split_bearing_block(L, (L.rb_x0 + L.rb_x1) / 2, L.rear_bearing, "rear")
    cm_lower, cm_cap, cm_feet = carrier_mount(L)
    stop, stop_feet = slip_ring_stop(L)
    cradle_x = -p.ENCODER_RING_T / 2
    out_x = L.disc_x0 + p.ENCODER_RING_T / 2
    sb_cradle, mod_cradle, sb1_feet = sensor_bracket(L, cradle_x, +1, L.cradle_beam_r)
    sb_out, mod_out, sb2_feet = sensor_bracket(L, out_x, +1, L.out_beam_r)
    sb_p1, mod_p1, sb3_feet = sensor_bracket(L, L.p1_disc_c, -1, L.out_beam_r)

    statics = [fp_lower, fp_cap, rp_lower, rp_cap, cm_lower, cm_cap, stop, sb_cradle, sb_out, sb_p1]
    feet = fp_feet + rp_feet + cm_feet + stop_feet + sb1_feet + sb2_feet + sb3_feet
    drum_len = p.ENCODER_RING_T + p.DYNO_DRUM_LENGTH / 2
    base = base_plate(L, feet, [s.bounding_box() for s in statics],
                      [L.disc_x0 + drum_len, L.p1_disc_x0 + drum_len])

    can_a, shaft_a = motor_dummy(carrier, L.xA, H)
    can_b, shaft_b = motor_dummy(inner, L.pocket_depth, H)
    sr = p.SLIP_RING_BORE / 2
    slip_ring = to_rig(zcyl(p.SLIP_RING_OD / 2, L.sr_x0, L.sr_x1) - zcyl(sr + 0.5, L.sr_x0 - 0.1, L.sr_x1 + 0.1), H)

    rig = lambda s: to_rig(s, H)
    pedestals = [
        Item("pedestal_front_lower", fp_lower, "static"),
        Item("pedestal_front_cap", fp_cap, "static"),
        Item("pedestal_rear_lower", rp_lower, "static"),
        Item("pedestal_rear_cap", rp_cap, "static"),
    ]
    common_static = [
        Item("base_plate", base, "static"),
        Item("carrier_mount_lower", cm_lower, "static"),
        Item("carrier_mount_cap", cm_cap, "static"),
        Item("motor_carrier_can", can_a, "static", printed=False),
        Item("motor_carrier_shaft", shaft_a, "carrier", printed=False),
    ]
    cradle_group = pedestals + [
        Item("cradle_body", rig(body), "carrier"),
        Item("rear_cap", rig(cap), "carrier"),
        Item("oldham_disc", rig(oldham), "carrier"),
        Item("coupler_hub", rig(hub), "carrier"),
        Item("motor_inner_can", can_b, "carrier", printed=False),
        Item("motor_inner_shaft", shaft_b, "output", printed=False),
        Item("slip_ring_stop", stop, "static"),
        Item("slip_ring", slip_ring, "static", printed=False),
        Item("bearing_front", bearing_dummy(L.front_bearing, L.fb_x0, H), "bearing", printed=False),
        Item("bearing_rear", bearing_dummy(L.rear_bearing, L.rb_x0, H), "bearing", printed=False),
        Item("sensor_bracket", sb_cradle, "static"),
        Item("sensor_cradle", mod_cradle, "static", printed=False),
        Item("sensor_bracket_output", sb_out, "static"),
        Item("sensor_output", mod_out, "static", printed=False),
    ]
    b.phase3 = common_static + cradle_group + [Item("encoder_disc", rig(enc), "output")]
    b.phase3_loaded = common_static + cradle_group + [Item("dyno_drum_inner", rig(drum_inner), "output")]
    b.phase1 = common_static + [
        Item("dyno_drum_carrier", rig(drum_carrier), "carrier"),
        Item("sensor_bracket_p1", sb_p1, "static"),
        Item("sensor_p1", mod_p1, "static", printed=False),
    ]

    # Print-oriented exports. Rotating parts are exported from their local
    # frame (axis = Z); flips put the face that must be flat on the bed.
    zflip = (180, 0, 0)
    b.exports = {
        "base_plate": (base, None),
        "pedestal_front_lower": (fp_lower, None),
        "pedestal_front_cap": (fp_cap, zflip),
        "pedestal_rear_lower": (rp_lower, None),
        "pedestal_rear_cap": (rp_cap, zflip),
        f"carrier_mount_lower_{carrier.name}": (cm_lower, None),
        f"carrier_mount_cap_{carrier.name}": (cm_cap, zflip),
        "coupler_hub": (hub, None),
        "oldham_disc": (oldham, None),
        f"cradle_body_{inner.name}": (body, zflip),
        f"rear_cap_{inner.name}": (cap, zflip),
        f"encoder_disc_{inner.name}": (enc, zflip),
        f"dyno_drum_{inner.name}": (drum_inner, None),
        f"dyno_drum_{carrier.name}": (drum_carrier, None),
        "slip_ring_stop": (stop, None),
        "sensor_bracket": (sb_cradle, None),
        "fit_coupon": (fit_coupon(L), None),
    }
    return b


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
MIN_RUNNING_CLEARANCE = 0.3  # mm between anything spinning and anything it must not touch

# Pairs that are meant to touch or overlap (press fits, clamps, a part and its own shaft)
ALLOWED_OVERLAP = {
    frozenset(("motor_inner_can", "rear_cap")),        # AXIAL_CLAMP_PRELOAD squeezes the motor
    frozenset(("motor_inner_shaft", "encoder_disc")),
    frozenset(("motor_inner_shaft", "dyno_drum_inner")),
    frozenset(("motor_carrier_shaft", "coupler_hub")),
    frozenset(("motor_carrier_shaft", "dyno_drum_carrier")),
    frozenset(("motor_carrier_can", "motor_carrier_shaft")),
    frozenset(("motor_inner_can", "motor_inner_shaft")),
    frozenset(("bearing_front", "cradle_body")),
    frozenset(("bearing_rear", "rear_cap")),
    frozenset(("cradle_body", "rear_cap")),
    frozenset(("oldham_disc", "rear_cap")),
    frozenset(("oldham_disc", "coupler_hub")),
    frozenset(("motor_carrier_can", "carrier_mount_lower")),
    frozenset(("motor_carrier_can", "carrier_mount_cap")),
}
# Same-group parts that legitimately sit close (sensor module on its own bracket, etc.)
SKIP_DISTANCE = ALLOWED_OVERLAP | {
    frozenset(("sensor_cradle", "sensor_bracket")),
    frozenset(("sensor_output", "sensor_bracket_output")),
    frozenset(("sensor_p1", "sensor_bracket_p1")),
}


def check(items, title):
    """Return a list of problems: any rotating/static pair closer than
    MIN_RUNNING_CLEARANCE, and any unintended solid overlap between parts."""
    problems = []
    for i, a in enumerate(items):
        for b_ in items[i + 1:]:
            pair = frozenset((a.name, b_.name))
            if "bearing" in (a.group, b_.group):
                continue
            moving = a.group != b_.group
            if moving and pair not in SKIP_DISTANCE:
                dist = a.shape.distance_to(b_.shape)
                if dist < MIN_RUNNING_CLEARANCE:
                    problems.append(f"{title}: {a.name} ({a.group}) is {dist:.2f} mm from {b_.name} ({b_.group})")
            elif pair not in ALLOWED_OVERLAP:
                if not a.shape.bounding_box().overlaps(b_.shape.bounding_box()):
                    continue
                vol = (a.shape & b_.shape).volume
                if vol > 0.5:
                    problems.append(f"{title}: {a.name} overlaps {b_.name} by {vol:.1f} mm^3")
    return problems


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
def to_bed(shape, rot):
    if rot:
        shape = Rot(*rot) * shape
    bb = shape.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * shape


# Assembly colors: printed parts by what they do, purchased hardware by what it is
GROUP_COLORS = {"static": (0.78, 0.80, 0.83), "carrier": (0.88, 0.48, 0.25), "output": (0.25, 0.56, 0.88)}
HARDWARE_COLORS = {
    "base_plate": (0.55, 0.58, 0.62),
    "motor_carrier_can": (0.30, 0.30, 0.32), "motor_inner_can": (0.30, 0.30, 0.32),
    "motor_carrier_shaft": (0.75, 0.75, 0.78), "motor_inner_shaft": (0.75, 0.75, 0.78),
    "bearing_front": (0.62, 0.64, 0.68), "bearing_rear": (0.62, 0.64, 0.68),
    "slip_ring": (0.80, 0.66, 0.23),
    "sensor_cradle": (0.18, 0.49, 0.20), "sensor_output": (0.18, 0.49, 0.20), "sensor_p1": (0.18, 0.49, 0.20),
}


def assembly_step(items, path: Path, label: str):
    """Write one STEP holding every part as a named, colored child. Parts are
    copied so the same shape can appear in several assemblies."""
    children = []
    for it in items:
        s = copy.copy(it.shape)
        s.label = it.name
        s.color = Color(*HARDWARE_COLORS.get(it.name, GROUP_COLORS.get(it.group, (0.6, 0.6, 0.6))))
        children.append(s)
    export_step(Compound(children=children, label=label), str(path))


def export(b: Build, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for stem, (shape, rot) in b.exports.items():
        export_stl(to_bed(shape, rot), str(out_dir / f"{stem}.stl"))
    for name, items in (("assembly_phase3", b.phase3), ("assembly_phase3_loaded", b.phase3_loaded),
                        ("assembly_phase1", b.phase1)):
        assembly_step(items, out_dir / f"{name}.step", name)


def summary(b: Build) -> str:
    L = b.L
    lines = [
        f"carrier = motor {L.carrier.name}, inner = motor {L.inner.name}",
        f"  axis height            {L.H:.1f} mm above the base plate",
        f"  front bearing (locating) {L.front_bearing} on a {2 * L.r_journal:.2f} mm journal",
        f"  rear bearing (floating)  {L.rear_bearing} on a {2 * L.r_rear_journal:.2f} mm journal",
        f"  cradle wall at journal  {L.r_journal - L.pocket_d / 2:.2f} mm",
        f"  carrier shaft engagement {L.engagement:.1f} mm (set screw: {'yes' if L.set_screw else 'no - press fit + CA'})",
        f"  cradle encoder beam radius {L.cradle_beam_r:.1f} mm, output disc beam radius {L.out_beam_r:.1f} mm",
        f"  axial stack: carrier face x={L.xA:.1f}, coupler {L.ch_x0:.1f}..{L.ch_x1:.1f}, "
        f"rear bearing {L.rb_x0:.1f}..{L.rb_x1:.1f}, slip ring {L.sr_x0:.1f}..{L.sr_x1:.1f}, "
        f"front bearing {L.fb_x0:.1f}..{L.fb_x1:.1f}, encoder disc {L.disc_x0:.1f}",
    ]
    for m in (L.carrier, L.inner):
        if not m.is_measured:
            lines.append(f"  WARNING: motor {m.name} dimensions are still ASSUMED - caliper it (data/motor_dims.csv)")
    return "\n".join(lines)


def write_bom(builds: list[Build]):
    L = builds[0].L
    printed = sorted({stem for b in builds for stem in b.exports if stem != "fit_coupon"})
    hardware = {}
    for b in builds:
        for k, v in b.L.fasteners.items():
            hardware[k] = max(hardware.get(k, 0), v)
    bearings = {}
    for b in builds:
        names = [b.L.front_bearing, b.L.rear_bearing]
        for n in set(names):
            bearings[n] = max(bearings.get(n, 0), names.count(n))

    material = {
        "base_plate": "PETG", "pedestal": "PETG", "cradle_body": "PETG, 100% infill (balance)",
        "rear_cap": "PETG, 100% infill (balance)", "coupler_hub": "PETG", "oldham_disc": "PETG, supports under web",
        "carrier_mount": "PETG", "encoder_disc": "PLA or PETG", "dyno_drum": "PETG, supports under disc",
        "slip_ring_stop": "PLA or PETG", "sensor_bracket": "PLA or PETG",
    }

    def mat(stem):
        return next((v for k, v in material.items() if stem.startswith(k)), "PETG")

    rows = [
        "# Bill of Materials",
        "",
        "_Generated by `python cad/rig.py` from the parametric model. Do not edit by hand; "
        "change cad/parameters.py or data/motor_dims.csv and re-run._",
        "",
        "## Purchased parts",
        "",
        "| Item | Qty | Spec / notes |",
        "| --- | --- | --- |",
        "| Brushed PM DC motor A | 1 | Exact model recorded in data/motor_dims.csv; characterized in Phase 1 |",
        "| Brushed PM DC motor B | 1 | Same |",
    ]
    for name, qty in sorted(bearings.items()):
        d, D, B = p.BEARINGS[name]
        rows.append(f"| Ball bearing {name} | {qty} | {d}x{D}x{B} mm, metal shields (ZZ), NOT rubber-sealed 2RS (seal drag) |")
    rows += [
        f"| Through-bore slip ring | 1 | >= 2 circuits, bore {p.SLIP_RING_BORE} mm, OD <= {p.SLIP_RING_OD} mm, "
        f"length <= {p.SLIP_RING_LENGTH} mm. **Choose the lowest rotating-torque spec you can find**: "
        "it must be well under the carrier's stall torque |",
        "| Slotted opto-interrupter module (LM393 type) | 3 | 5 mm slot; 2 on the rig + 1 for the Phase 1 dynamometer |",
        "| Arduino Uno / Nano (5 V) | 1 | Runs firmware/tachometer; timestamps pulses on D2 and D3 |",
        "| Bench power supply, current-limited | 2 | One per motor, independently adjustable; current readout used as data |",
        "| Multimeter | 1 | 4-wire resistance not needed; used for locked-rotor V/I and supply cross-checks |",
        "| Digital force gauge (0-5 N, 0.01 N) + hanging masses (10-200 g) | 1 set | Rope-brake dynamometer |",
        "| Braided cord, ~0.5 mm (fishing line / kevlar thread) | 1 m | Rope brake |",
        "| Steel fender washer (e.g. M8 x 24 mm) | 1 | Known-inertia flywheel for the spin-up test; weigh it |",
        "| Calipers | 1 | Motor dimensions and fit-coupon checks |",
        "| Drill bit = motor shaft diameter (e.g. 2.0 mm) | 1 | Ream every printed shaft bore to size |",
        "| Thin silicone hook-up wire (28-30 AWG) | 1 m | Inner motor leads through the rear cap to the slip ring |",
        "| M3 x 10 screw + nut | 1 | Anti-rotation pin on the slip-ring stator (rides in the stop's window) |",
        "| Clear polycarbonate sheet or printed guard | 1 | Shield around the spinning cradle (see Safety) |",
        "| CA glue | - | Optional: lock press-fit discs on shafts |",
    ]
    for item in sorted(hardware):
        rows.append(f"| {item} | {hardware[item]} | |")
    rows += [
        "",
        "## 3D-printed parts",
        "",
        "STLs are print-oriented in `cad/exports/carrier_<X>/`. Print `fit_coupon.stl` first and tune the "
        "fits in cad/parameters.py before printing the rest.",
        "",
        "| Part | Qty | Material / notes |",
        "| --- | --- | --- |",
    ]
    for stem in printed:
        qty = 3 if stem == "sensor_bracket" else 1
        rows.append(f"| {stem}.stl | {qty} | {mat(stem)} |")
    rows.append("")
    BOM_PATH.write_text("\n".join(rows), encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--carrier", choices=["A", "B", "both"], default="A")
    ap.add_argument("--no-check", action="store_true")
    args = ap.parse_args(argv)

    motors = p.load_motor_dims()
    roles = ["A", "B"] if args.carrier == "both" else [args.carrier]
    builds, failed = [], False
    for c in roles:
        inner = "B" if c == "A" else "A"
        b = build(motors[c], motors[inner])
        print(summary(b))
        if not args.no_check:
            problems = []
            for title, items in (("phase 3", b.phase3), ("phase 3 loaded", b.phase3_loaded), ("phase 1", b.phase1)):
                problems += check(items, title)
            if problems:
                failed = True
                print("  CLEARANCE PROBLEMS:")
                for msg in problems:
                    print("   - " + msg)
            else:
                print(f"  clearance check passed (>= {MIN_RUNNING_CLEARANCE} mm running clearance, no overlaps)")
        out = EXPORT_DIR / f"carrier_{c}"
        export(b, out)
        print(f"  exported {len(b.exports)} STLs + 3 assembly STEPs to {out.relative_to(p.ROOT)}")
        builds.append(b)
    # the one file to open for a 3D look at the whole running rig (first configuration built)
    assembly_step(builds[0].phase3, EXPORT_DIR / "rig_assembly.step", "nested_motor_rig")
    print(f"full colored assembly written to {(EXPORT_DIR / 'rig_assembly.step').relative_to(p.ROOT)}")
    write_bom(builds)
    print(f"BOM written to {BOM_PATH.relative_to(p.ROOT)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
