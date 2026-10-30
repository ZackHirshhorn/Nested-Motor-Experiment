# Design notes: engineering review of the plan and the decisions it drove

This file records *why* the rig looks the way it does, and where it departs from
[the original plan](Nested%20DC-Motor%20Rotation%20Experiment.md). Read it before
changing the CAD or the procedure.

## 1. What the experiment actually tests

ω_out = ω_cradle + ω_rel is a rigid-body kinematic identity. It is always true,
because ω_rel is *defined* as the rotor's speed relative to the cradle. The rig
cannot falsify it. What it can test is **how each motor behaves when nested**:

1. Does the inner motor's relative speed depend only on its own voltage and load,
   regardless of the carrier's rotation? Its back-EMF depends on relative speed
   only, so the motor model says yes.
2. How much does the carrier slow down, and is that predictable?
3. Does role order matter, and by how much?

For that reason the rig measures **both** ω_cradle (encoder ring on the rear cap)
and ω_out (disc on the output shaft) at the same time. The plan only had the
output encoder, so the carrier's speed during a powered run had to be inferred
from a separate baseline. With both measured, ω_rel = ω_out − ω_cradle is observed
directly, and each motor can be checked against its own bench model *inside* the rig.

## 2. Torque balance: the two motors are in series

The plan says the carrier must "continuously cancel the inner motor's own reaction
torque". That is only true in transients. Take moments about the spin axis for two
bodies: the inner rotor (r), and the cradle + inner stator (C).

```
rotor:   τ_em − τ_f − τ_out             = J_r · α_r
cradle:  τ_A − τ_em + τ_f − τ_par       = J_C · α_C
```

Here τ_em is the electromagnetic torque between the inner rotor and stator, τ_f is
the inner motor's internal friction (brushes and bushings, acting between rotor
and stator), τ_out is the external load on the output shaft, τ_par is the parasitic
drag on the cradle (bearings, slip ring, windage), and τ_A is the carrier's torque.

In steady state, τ_em − τ_f = τ_out, so

> **τ_A = τ_out + τ_par**

The inner motor's friction is internal to the nested assembly and cancels. The
only torque the inner motor feeds back into the carrier is the external load.
At no output load the carrier fights only τ_par. Under load, **both motors carry
the same τ_out and their speeds add**, exactly like two motors in series:

```
ω_out(τ_out) = ω_A(τ_out + τ_par) + ω_B(τ_out)
```

Consequences:

- **Free speed** of the combination is the sum of the free speeds, minus the
  carrier's sag under τ_par.
- **The combined torque–speed line's slope** is the sum of both motors' slopes.
  Its stall torque falls between the two motors' stall torques. Past the weaker
  motor's stall point, that motor is back-driven.
- **The role-swap effect is small and specific**: only τ_par moves from one motor
  to the other. The predicted difference in ω_out is
  τ_par × (ω_free/τ_stall of motor B − the same for motor A), and it is the same
  at every output load. The plan's expectation of a "measurably larger deviation
  or outright stalling" happens only if τ_par is a large fraction of one motor's
  stall torque. In practice that means a high-drag slip ring.
- **Transients are different.** While the inner motor accelerates, its stator
  reaction (up to the inner motor's *stall torque*) kicks the cradle backward, and
  the carrier must absorb it. Start the carrier first, then bring the inner motor
  up on a current-limited supply.

`analysis/motor_model.py` implements this model. `analysis/compare_runs.py` checks
it two ways. First, the carrier's extra current when the output is loaded should
correspond to exactly τ_out ("inner reaction" column). Second, ω_out − ω_cradle
should match the inner motor's bench line at τ_out ("rel" column).

## 3. The real stall risk is the slip ring, not the inner motor

A 130-size motor has a stall torque of a few mN·m. Small slip rings often quote a
"rotating torque" of 1–10 mN·m, and rubber-sealed (2RS) bearings add drag of the
same order. Since τ_par is everything the carrier fights at no load, **the
slip-ring choice decides whether the rig starts at all.** Hence:

- The BOM specifies **ZZ (metal-shield) bearings, not 2RS**.
- Phase 1 adds a **slip-ring drag measurement**: thread wound on the rotor and
  pulled with the force gauge. `analysis/select_carrier.py` uses it, and rejects
  any assignment where the carrier's stall torque is less than 3× τ_par, to leave
  room for breakaway friction.
- **Check the slip ring's maximum speed rating.** Many low-cost slip rings are
  rated for only a few hundred rpm. At thousands of rpm, brush bounce and wear
  set in. If your ring is rated low, run the carrier at reduced voltage, use a
  geared motor as the carrier, or build a high-speed ring (graphite brushes on a
  brass sleeve).
- If no slip ring is low enough in drag, a fallback is a small battery pack riding
  on the cradle. It has zero drag but no current readback for the inner motor.

## 4. Dynamometer: rope brake, plus a spin-up method

The plan describes a cord *wound* on a drum and running to a hanging mass. At
thousands of rpm that just winds the mass up in a fraction of a second; it cannot
hold a steady operating point. The standard fix, used here, is a **rope (band)
brake**. The cord wraps once around the drum. One end carries a hanging mass
(T_tight), the other is anchored to the force gauge (T_slack), and the drum slips
inside the cord:

```
τ = (T_tight − T_slack) × r_eff,     r_eff = drum radius + half the cord diameter
```

Raising the mass raises the load. The drum is 16 mm in diameter (8.25 mm effective
radius with a 0.5 mm cord). A 5 mN·m stall torque then needs only about 0.6 N of
tension difference, well within a 0.01 N-resolution gauge. Friction heat in a
printed drum limits each point to a few seconds, so read quickly.

Because rope-brake points on tiny motors are noisy, there are two independent
cross-checks:

- **Electrical method** (primary). R from locked-rotor tests at several rotor
  positions. kE from no-load runs at three voltages, by fitting V − IR = kE·ω.
  Friction is modelled as a no-load current I0(ω) = a + b·ω. Torque at any point is
  then kT·(I − I0(ω)), with kT = kE in SI units. The rope-brake points are used to
  fit kT *independently*; agreement with kE confirms the data are self-consistent.
- **Spin-up method**. Log one pulse timestamp per encoder window while the motor
  accelerates from rest. J·dω/dt = τ(ω) is linear in ω, so a fit gives the
  free speed and τ_stall/J. A second run with a weighed steel washer (known J)
  added gives J itself, and therefore the stall torque, with no force measurement
  at all. The firmware's raw mode and `log_serial.py --raw` exist for this.

`analysis/selftest.py` checks the fitting code end-to-end on synthetic data with
realistic noise. It recovers kE, R, the free speed, stall torque, inertia and
τ_par to within about 1%.

## 5. Mechanical architecture

**Kinematic support.** The cradle runs on its own two bearings. The plan's "rigid
flange" coupling to the carrier's shaft would have added a third support (the
carrier's own bushings) on the same axis. Any printed misalignment would then
load all three. The rig uses:

- **An Oldham coupling** (coupler hub, floating cross-tongued disc, slot in the
  rear hub). It transmits torque only, tolerates parallel and slight angular
  misalignment, and puts no radial load on the carrier's shaft.
- **A locating/floating bearing pair.** The front bearing (6805ZZ, 25 mm bore)
  sits directly on the cradle body with a light press fit against a shoulder, and
  its outer race is captured between the split pedestal's lips. That bearing
  fixes the cradle axially. The rear bearing (6801ZZ) slides on its journal, so
  thermal growth and print error cannot preload the pair.
- **A rotation-only stop for the slip ring.** The slip ring's stator is not
  mounted. A pin on it rides in a window in a post, which blocks rotation only.
  Bolting the stator down would have been a fourth support.

**Bearing sizes are derived, not chosen.** `FRONT_BEARING = "auto"` picks the
smallest thin-section bearing whose bore leaves at least 1.6 mm of wall around the
inner motor's pocket. For a 130-size motor that is a 6805. A larger motor (for
example a 280-size) moves up automatically.

**Concentricity.** Each motor is centered by its front bearing boss in a close
hole (0.05 mm radial clearance), not by its can, which has looser tolerance. The
inner motor's flats key it against rotation, so torque never relies on friction.
Its axial position is clamped by the rear cap with a 0.1 mm preload.

**Split pedestals.** The bearings drop into half-pockets and are clamped by caps.
The cradle assembly is then lowered in complete, with bearings already on its
journals. The base plate has ±3 mm slots under the pedestals to remove axial
preload.

**Balance.** The rotating parts are symmetric: 3 bolts at 120°, 2 wire holes at
180°, encoder windows evenly spaced. Print the cradle body and rear cap at 100%
infill so the mass is uniform. Hoop stress in the printed cradle is negligible
(under 1 MPa at 8000 rpm), but a 1 g·mm imbalance gives about 0.7 N of rotating
force at 8000 rpm. Start below 4000 rpm (`MAX_CRADLE_RPM`) and work upward while
watching for vibration.

**Every clearance is checked by the script.** `cad/rig.py` places dummy motors,
bearings, the slip ring and the sensor forks. It then verifies at least 0.3 mm of
running clearance between every rotating and static part, and no unintended
overlap between parts, in three assemblies: Phase 3, Phase 3 with the dyno drum,
and the Phase 1 dynamometer. It exits non-zero if anything fails. Several
collisions in early iterations were found this way.

## 6. Second-order effects worth watching

- **Brushes under centrifugal load.** The inner motor's brush springs are a few
  mm off-axis. At 8000 rpm they see about 300 g of radial acceleration, which can
  change contact pressure and therefore R and friction. Watch for this: I_inner
  at zero output load should match the bench no-load current at the same relative
  speed.
- **Slip-ring resistance** reduces the inner motor's effective voltage. Measure it
  with the multimeter (both circuits in series, turning slowly) and pass it as
  `compare_runs.py --r-slip`.
- **Direction.** The single-channel tachometers measure speed, not direction.
  Set polarities with tape flags at low voltage. Then confirm that ω_out ≈
  ω_cradle + ω_rel, not the difference.
- **Heating.** kE drops a few percent as the magnets warm, and R rises about 0.4%/K.
  Characterize and run at similar temperatures, and keep locked-rotor tests to a
  few seconds.

## 7. Assumed values to replace before printing

| What | Where | Why it matters |
|---|---|---|
| Motor dimensions (both rows are generic 130-size) | data/motor_dims.csv | Pocket, boss and shaft fits; whole axial stack |
| Slip-ring bore, OD, length | cad/parameters.py | Rear hub, axis height, stop position |
| Slot-sensor geometry (gap, PCB to beam) | cad/parameters.py | Bracket position (the brackets also have adjustment slots) |
| Printer hole undersize 0.175 mm | cad/parameters.py | Carried over from the Fermentation Cap calibration; same printer assumed |

`cad/rig.py` prints a warning for every motor row still marked `ASSUMED`.
