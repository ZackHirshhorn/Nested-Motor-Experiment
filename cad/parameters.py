"""Every number the rig geometry depends on, in one place.

Three categories, kept visibly separate because a bad fit means something
different in each:

  * MEASURED  — motor dimensions, read from data/motor_dims.csv (calipers).
                The rows shipped in that file are generic 130-size values and
                are marked ASSUMED in its `source` column until replaced.
  * CATALOG   — standard parts (bearings, fasteners) at their nominal sizes.
  * ASSUMED   — purchased parts whose exact size depends on the part you buy
                (slip ring, slot sensor module). Caliper them and update here.

Geometry is laid out in RIG coordinates: X is the spin axis (+X points from
the carrier toward the inner motor's output shaft), Y lateral, Z up. The base
plate's top face is z = 0 and the spin axis sits at height AXIS_HEIGHT.
"""

import csv
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MOTOR_DIMS_CSV = ROOT / "data" / "motor_dims.csv"

# ---------------------------------------------------------------------------
# Printer / process
# ---------------------------------------------------------------------------
PRINTER_HOLE_UNDERSIZE = 0.175  # mm, radial. Carried over from the Fermentation Cap
                                 # calibration cylinder (same printer assumed). Added to
                                 # every printed hole/pocket radius below.
MIN_WALL = 1.6                  # mm, thinnest load-bearing printed wall allowed (4 perimeters @ 0.4)

# ---------------------------------------------------------------------------
# Fits (radial, mm). Positive = clearance, negative = interference.
# Print fit_coupon.stl first and adjust these before printing the big parts.
# ---------------------------------------------------------------------------
MOTOR_POCKET_CLEARANCE = 0.10    # motor can in its cradle/mount pocket (snug; torque goes through the flats)
BOSS_CENTERING_CLEARANCE = 0.05  # motor front boss in its centering hole — this sets concentricity
BEARING_SEAT_FIT = -0.025        # bearing outer race in a printed pocket (light press)
LOCATING_JOURNAL_FIT = 0.0       # front bearing inner race on the cradle (locating: no slip)
FLOATING_JOURNAL_FIT = -0.05     # rear bearing inner race on the rear hub (floating: slides axially)
SLIP_RING_SEAT_FIT = -0.05       # slip-ring rotor sleeve on the rear hub (its own set screws grip it)
OLDHAM_CLEARANCE = 0.15          # each side of each Oldham tongue in its slot
AXIAL_CLAMP_PRELOAD = 0.10       # cradle pocket is this much shorter than the motor can, so the
                                 # rear cap clamps the motor axially against the front lip

# Small motor shafts: the bore is printed at nominal shaft diameter (which
# prints ~0.35 mm undersize) and then drilled to size with a shaft-diameter
# drill bit. More repeatable than trusting a printed 2 mm hole.

# ---------------------------------------------------------------------------
# CATALOG: thin-section deep-groove bearings, (bore d, outer D, width B) in mm.
# ZZ (metal shield) recommended over 2RS — rubber seals add drag that the
# carrier has to fight continuously.
# ---------------------------------------------------------------------------
BEARINGS = {
    "6800ZZ": (10, 19, 5),
    "6801ZZ": (12, 21, 5),
    "6802ZZ": (15, 24, 5),
    "6803ZZ": (17, 26, 5),
    "6804ZZ": (20, 32, 7),
    "6805ZZ": (25, 37, 7),
    "6806ZZ": (30, 42, 7),
    "6807ZZ": (35, 47, 7),
}
REAR_BEARING = "6801ZZ"   # floating bearing on the rear hub (carrier side)
FRONT_BEARING = "auto"    # locating bearing directly on the cradle body; "auto" picks the smallest
                          # one whose bore leaves MIN_WALL around the inner motor's pocket

# ---------------------------------------------------------------------------
# CATALOG: fasteners
# ---------------------------------------------------------------------------
M3_CLEARANCE_DIA = 3.4
M3_HEAD_DIA = 6.0         # socket head cap screw, counterbore diameter (before undersize)
M3_HEAD_HEIGHT = 3.2
M3_INSERT_HOLE_DIA = 4.0  # heat-set insert pilot (M3 x 5.7 "short" style)
M3_INSERT_DEPTH = 6.0
M3_SQUARE_NUT = 5.5       # DIN 562 square nut, across flats
M3_SQUARE_NUT_T = 1.8

# ---------------------------------------------------------------------------
# ASSUMED: through-bore slip ring (2+ circuits). Sizes vary a lot between
# vendors — caliper yours. Check its datasheet "rotating torque" against the
# carrier's stall torque from Phase 1 before buying (see docs/DESIGN_NOTES.md).
# ---------------------------------------------------------------------------
SLIP_RING_BORE = 12.7     # mm, rotor sleeve bore (must exceed the rear bearing bore so it can pass over)
SLIP_RING_OD = 40.0       # mm, stator outer diameter
SLIP_RING_LENGTH = 25.0   # mm, overall axial length
SLIP_RING_WIRE_GAP = 6.0  # mm between the rear cap and the slip ring, room to splice the inner motor's leads

# ---------------------------------------------------------------------------
# ASSUMED: slotted opto-interrupter module (generic LM393 "speed sensor" board)
# ---------------------------------------------------------------------------
SENSOR_SLOT_WIDTH = 5.0       # mm, gap between the fork arms
SENSOR_PCB_TO_BEAM = 9.0      # mm, from the PCB's front face to the optical beam
SENSOR_ARM_PAST_BEAM = 2.5    # mm, how far the fork arms reach beyond the beam
SENSOR_ARM_THICKNESS = 3.0    # mm, each fork arm, measured along the beam
SENSOR_PCB_THICKNESS = 1.6
SENSOR_PCB_WIDTH = 14.0       # mm, module board width measured along the beam

# ---------------------------------------------------------------------------
# Rig design choices
# ---------------------------------------------------------------------------
BASE_THICKNESS = 8.0
BASE_MARGIN = 10.0            # mm of plate beyond the outermost footprint
PEDESTAL_LIP = 1.5            # axial lip either side of each bearing pocket
PEDESTAL_WALL = 6.0           # material around the bearing pocket / insert bosses
PEDESTAL_FOOT_EXT = 3.0       # foot flange beyond the pedestal upright, each side along X
PEDESTAL_SLOT_TRAVEL = 3.0    # +/- mm of axial adjustment in the pedestal feet
CLAMP_GAP = 0.4               # gap left between split halves so the screws actually clamp

CRADLE_LIP = 1.6              # front lip the inner motor seats against (its boss passes through)
CRADLE_FLANGE_T = 7.0         # body flange that the rear cap bolts to (holds M3 heat-set inserts)
CAP_WALL = 2.5                # rear cap material behind the terminal cavity
ENCODER_RING_WIDTH = 9.0      # radial width of the cradle encoder ring beyond the flange
ENCODER_RING_T = 1.6          # thickness of every encoder ring/disc (must be < SENSOR_SLOT_WIDTH)
CRADLE_PPR = 2                # windows in the cradle encoder ring (pulses per rev)
OUTPUT_PPR = 4                # windows in the output encoder disc
OUTPUT_DISC_DIA = 36.0
DYNO_DRUM_DIA = 16.0          # rope-brake drum (Phase 1 dynamometer and Phase 3 loaded sweep)
DYNO_DRUM_LENGTH = 10.0
HUB_OD = 14.0                 # set-screw hubs on motor shafts (room for an M3 square nut)
OLDHAM_TONGUE_W = 4.0         # Oldham coupling tongue width
OLDHAM_TONGUE_DEPTH = 2.5     # how far each tongue projects from the Oldham web
OLDHAM_WEB_T = 2.0
OLDHAM_SLOT_DEPTH = 3.0       # slots are deeper than the tongues so the coupling never bottoms out
MIN_SHAFT_ENGAGEMENT = 4.0    # mm of motor shaft that must sit inside a printed hub
FOOT_HEIGHT = 8.0             # pedestal/bracket feet: thick enough for a bottom heat-set insert
SENSOR_SLOT_ADJUST = 10.0     # +/- mm height adjustment slot in the sensor bracket upright
BASE_SLOT_ADJUST = 3.0        # +/- mm adjustment slots in the base plate for sensor brackets

AXIAL_GAP = 1.0               # minimum axial gap between a rotating and a static part
RADIAL_GAP = 1.0              # minimum radial gap between a rotating and a static part

MAX_CRADLE_RPM = 4000         # recommended operating limit for a printed cradle; see DESIGN_NOTES.md


# ---------------------------------------------------------------------------
# MEASURED: motor dimensions
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MotorDims:
    name: str
    can_dia: float          # across the round part of the can
    flat_width: float       # across the flats (0 = fully round can)
    can_length: float       # front face to rear face of the end bell (the clamped length)
    boss_dia: float         # front bearing boss
    boss_length: float
    shaft_dia: float
    shaft_length: float     # shaft beyond the boss face
    rear_clearance: float   # terminals / end-bell bumps behind the can
    source: str

    @property
    def radius(self) -> float:
        return self.can_dia / 2

    @property
    def is_measured(self) -> bool:
        return not self.source.upper().startswith("ASSUMED")


def load_motor_dims(path: Path = MOTOR_DIMS_CSV) -> dict[str, MotorDims]:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    motors = {}
    for row in rows:
        motors[row["motor"]] = MotorDims(
            name=row["motor"],
            source=row["source"],
            **{k: float(v) for k, v in row.items() if k not in ("motor", "source")},
        )
    return motors


def hole(dia: float) -> float:
    """Model diameter for a printed hole that should come out at `dia`."""
    return dia + 2 * PRINTER_HOLE_UNDERSIZE


def pocket_dia(m: MotorDims) -> float:
    return hole(m.can_dia + 2 * MOTOR_POCKET_CLEARANCE)


def pocket_flat(m: MotorDims) -> float:
    if m.flat_width <= 0:
        return 0.0
    return hole(m.flat_width + 2 * MOTOR_POCKET_CLEARANCE)


def pick_front_bearing(inner: MotorDims) -> str:
    if FRONT_BEARING != "auto":
        return FRONT_BEARING
    need = pocket_dia(inner) + 2 * MIN_WALL
    for name, (d, _, _) in sorted(BEARINGS.items(), key=lambda kv: kv[1][0]):
        if d >= need:
            return name
    raise ValueError(
        f"No bearing in BEARINGS has a bore >= {need:.2f} mm for motor {inner.name}; add a larger one."
    )
