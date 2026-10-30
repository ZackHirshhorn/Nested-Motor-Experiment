# Nested DC-Motor Rotation Experiment

Oct 30, 2026 · @Zack

## Overview & Objectives

This experiment builds and tests a rig where one DC motor (the **carrier**, Motor A) spins the entire body of a second DC motor (the **inner motor**, Motor B) in place, while Motor B's own rotor keeps turning inside that spinning body. The hypothesis under test: in the ground frame, Motor B's output shaft turns at

```latex
\omega_{out} = \omega_{carrier} + \omega_{B,\,rel}
```

where ω\_carrier is the cradle's rotation rate (driven by Motor A) and ω\_B,rel is Motor B's own rotor speed relative to its casing — i.e. roughly the sum of the two motors' free-running speeds, provided the rotation is coaxial, same-direction, and neither motor is starved of torque by the extra mechanical load of the setup.

A Claude Code session will run this as a small multidisciplinary project directly from this repository, in four linked phases:

1. **Characterize** both motors independently (kV, winding resistance, stall current, stall torque, torque-speed curve) via small test scripts and a simple dynamometer rig — this also decides which motor should be the carrier.
2. **Design** the 3D-printed fixtures parametrically in Python (consistent with this project's existing Python-CAD workflow — build123d or CadQuery), driven by the measured motor dimensions and torque margins, and export STL files.
3. **Document** the build as a bill of materials and assembly instructions generated alongside the CAD source.
4. **Test** the assembled rig, log rpm/current data, and compare measured vs predicted composite speed — including a repeat run with the carrier/inner roles swapped.

Suggested repo layout: `/cad` (parametric part scripts + exported STL), `/data` (motor characterization and test-rig CSV logs), `/analysis` (scripts/notebooks computing kV, kT, stall torque, and predicted vs measured rpm), `/docs` (this plan, BOM, assembly instructions).

## Physics Recap

For coaxial, same-direction rotation, angular velocities add in the ground frame:

```latex
\omega_{out} = \omega_{carrier} + \omega_{rel}
```

The carrier's job is not symmetric with the inner motor's. The carrier must:

- accelerate the inner motor's full housing inertia I\_B during spin-up,
- continuously cancel the inner motor's own electromagnetic reaction torque — the Newton's-third-law pair to the torque the inner motor's rotor produces on its own stator, which would otherwise spin the housing backward, and
- overcome friction in the ground-fixed bearing that carries the housing's weight.

The inner motor, by contrast, only has to spin its own rotor against its own internal friction. This asymmetry is why the choice of which motor plays which role (addressed in Phase 1 and Mechanical Design) matters as much as the nominal free speeds.

Terms used throughout this plan:

- **kV** — speed constant, rpm per volt: ω\_free = kV × V
- **kT** — torque constant, N·m per amp; for a PM brushed DC motor kT = 1/kV in consistent SI units (V·s/rad = N·m/A)
- **R** — winding resistance, from a brief locked-rotor test: R = V\_applied / I\_stall
- **Stall torque**: τ\_stall = kT × I\_stall
- **Torque–speed line** (linear approximation for a PM DC motor): τ(ω) = τ\_stall × (1 − ω/ω\_free)

Undersizing the carrier's torque margin over the inner motor's reaction torque, housing inertia, and bearing friction is the main failure mode of this rig — Phase 1 exists to measure that margin before any part is printed.

## Phase 1 — Motor Characterization & Torque Rating

**Purpose:** get kV, R, stall current, stall torque, and a torque–speed curve for Motor A and Motor B independently, before either is coupled to the other. This is also what decides which motor becomes the carrier.

**Test rig:** a simple lever-arm (Prony-brake style) dynamometer — a small 3D-printed drum keyed to the motor's own shaft, a cord wound on the drum running over a light pulley to either a hanging mass or a digital force gauge. Torque at any load point is τ = F × r (force × drum radius). Speed at each load point is read from a slotted encoder disc on the same shaft via an IR photo-interrupter (or a smartphone strobe app as a cross-check).

**Procedure, run once per motor:**

1. **No-load speed test** — apply rated voltage with nothing attached to the shaft, record steady-state rpm → ω\_free, compute kV = ω\_free / V.
2. **Locked-rotor test** — briefly (a few seconds only, to avoid overheating the winding) apply a known low voltage with the shaft clamped stationary, record current → R = V / I\_stall.
3. **Loaded sweep** — at constant rated voltage, step through increasing brake loads (weights or force-gauge settings), recording steady-state rpm and current at each point. This traces the torque–speed line directly, and its zero-speed intercept cross-checks τ\_stall = kT × I\_stall from steps 1–2.
4. Repeat steps 1–3 at one or two other voltages if time allows, to confirm kV and kT hold constant across voltage (expected for PM brushed DC motors).

**Output of this phase:** a short table — kV, R, I\_stall, τ\_stall, rated free speed — for Motor A and Motor B, logged to `/data/motor_A.csv` and `/data/motor_B.csv` and summarized in the Data Analysis section below. Whichever motor shows the larger torque margin over the other's housing inertia + reaction torque + bearing friction (estimated from these numbers) becomes the carrier in Phase 2 — per the Physics Recap above, pairing a low-torque motor as carrier for a higher-torque inner motor is the likeliest way to stall the rig.

## Phase 2 — Mechanical Design

Seven 3D-printed parts, all parametric in a single Python CAD script:

1. **Base plate** — rigid platform holding the carrier motor and the bearing block in fixed alignment, coaxial with the spin axis.
2. **Carrier motor mount** — clamps/screws the carrier motor's body to the base plate.
3. **Shaft coupler** — connects the carrier's output shaft to the cradle; a shaft-size bore (press-fit or set-screw) on one end, a mating flange on the other.
4. **Cradle** — the part that actually does the "carrying": a sleeve/clamp that rigidly holds the inner motor's entire body, with a flange that mates to the coupler on one end and a bearing seat on the other (or both ends), so the cradle+inner-motor assembly spins on its own ground-fixed bearing, not on the carrier.
5. **Bearing block** — houses one or two small radial ball bearings, bolted to the base plate; this is what actually carries the inner motor's weight and radial/axial loads, so the carrier only has to supply rotational torque, not structural support.
6. **Slip-ring mount** — bracket holding a small 2-wire slip ring coaxial with the spin axis, carrying power from the fixed supply into the spinning cradle and on to the inner motor's leads.
7. **Output encoder disc** — a slotted disc on the inner motor's own output shaft, read by a fixed IR photo-interrupter to measure the ground-frame output rpm.

**CAD approach:** write each part as a parametric function in one Python CAD script (build123d or CadQuery, matching this project's existing Python-CAD workflow), with motor OD/length, shaft diameter, and bearing bore/OD as named parameters read from the Phase 1 measurement CSVs — so re-measuring a motor and re-running the script regenerates correctly-fitting parts automatically. Export each part to STL from the script; keep the script itself, not just the STL, as the source of truth, version-controlled alongside the data it consumes.

&#91;embedded content: assembly stack — carrier, coupler, cradle, bearing block, slip ring, output encoder\]

Only the carrier and the bearing block attach to the base plate; the cradle (and the inner motor inside it) is free to spin.

## Bill of Materials

| Item | Qty | Spec / notes |
| --- | --- | --- |
| DC motor (carrier, Motor A) | 1 | brushed PM DC, voltage per Phase 1 result; record exact model/kV |
| DC motor (inner, Motor B) | 1 | brushed PM DC, voltage per Phase 1 result; record exact model/kV |
| Radial ball bearing | 2 | small flanged bearing sized to the cradle's bearing seat OD (e.g. 8–12 mm bore) |
| Miniature slip ring | 1 | 2-wire, low-current (hobby motor current rating), through-bore sized to the spin axis |
| IR photo-interrupter / reflective sensor | 2 | one for Phase 1 dynamometer rpm, one for the combined rig's output encoder |
| Microcontroller (e.g. Arduino) | 1 | timestamps photo-interrupter pulses for rpm; optional but recommended over a phone strobe app |
| Digital force gauge or calibrated hanging weights | 1 set | for the Phase 1 lever-arm torque measurement |
| Bench power supply (or batteries) | 2 | matched to each motor's rated voltage; independent supply for Motor A and Motor B |
| Multimeter | 1 | voltage/current readback during characterization and combined tests |
| M3 screws, nuts, heat-set inserts | \~20 | fastening base plate, motor mounts, bearing block, slip-ring bracket |
| Thin flexible hook-up wire | as needed | motor leads through the slip ring |
| 3D-printed: base plate | 1 | PETG recommended for stiffness |
| 3D-printed: carrier motor mount | 1 | PETG or PLA |
| 3D-printed: shaft coupler | 1 | PETG recommended (cyclic torque load) |
| 3D-printed: cradle | 1 | PETG recommended (cyclic torque load, carries Motor B) |
| 3D-printed: bearing block | 1 | PETG recommended (bearing press-fit) |
| 3D-printed: slip-ring mount | 1 | PLA or PETG |
| 3D-printed: output encoder disc | 1 | PLA, slotted per encoder resolution needed |
| Calipers | 1 | measuring motor OD/length and shaft diameter before CAD scripting |

## Assembly Instructions

1. Print all seven parts (Mechanical Design above); let PETG parts cool fully before removing supports — the coupler, cradle, and bearing block all see cyclic load.
2. Press the two radial bearings into the bearing block; confirm the inner races spin freely by hand with no wobble.
3. Bolt the carrier motor mount and the bearing block to the base plate, both coaxial with the intended spin axis (check alignment with a straightedge or dial indicator before fully tightening).
4. Mount the carrier motor (Motor A, or whichever Phase 1 selected) in its mount; do not connect its shaft yet.
5. Press-fit the inner motor (Motor B) into the cradle; confirm it is rigidly seated with no play — any looseness here shows up as vibration once spinning.
6. Seat the cradle's bearing stub(s) into the bearing block's bearings. Confirm the cradle spins freely by hand before attaching anything else.
7. Attach the shaft coupler to the carrier motor's output shaft, then join the coupler to the cradle's input flange. Recheck that the full carrier→coupler→cradle chain still turns freely by hand.
8. Mount the slip-ring bracket coaxial with the spin axis; connect the slip ring's rotating side to the inner motor's leads (through the cradle), and its stationary side to the bench supply via the flexible hook-up wire.
9. Fit the output encoder disc onto the inner motor's own output shaft (the true output of the assembly) and position the fixed IR photo-interrupter so the disc's slots pass through its gap without touching it.
10. Wire both motors to their own supply/multimeter channels, independently switchable, so each can be powered and measured separately before the combined run.
11. With everything de-energized, spin the whole assembly by hand once more end-to-end — confirm nothing binds, rubs, or contacts the frame — before applying any power (Phase 3).

## Phase 3 — Combined Test Procedure

1. **Baseline check.** With the rig assembled but only the carrier powered (inner motor leads disconnected), run the carrier up to steady state and log its loaded rpm — this is ω\_carrier under the real load of spinning the cradle, which will be lower than its Phase 1 free speed.
2. **Combined run.** Power the inner motor through the slip ring as well. Once both reach steady state, read the output encoder disc's rpm (ground frame) and compare to the predicted ω\_out = ω\_carrier + ω\_B,rel from steps above and Phase 1 data.
3. **Loaded sweep.** Apply the Phase 1 dynamometer (lever arm + weights/force gauge) to the output shaft itself and repeat step 2 at several load points, tracing the combined system's own torque–speed line.
4. **Role-swap trial.** Rebuild with roles reversed — the former inner motor now as carrier, and vice versa — and repeat steps 1–3. This directly tests the "order" effect: whichever arrangement keeps the carrier closer to its own free speed under load should show a composite rpm closer to the ideal sum.
5. Log every run (voltage, current, both motors' individual free/loaded rpm, combined output rpm, applied brake load) to `/data/combined_runs.csv`.

## Data Analysis & Expected Results

For each run, compute:

- **Predicted composite speed:** ω\_out,pred = ω\_carrier,loaded + ω\_B,free, using the carrier's *loaded* steady-state rpm from the baseline check (step 1 of Phase 3), not its unloaded Phase 1 free speed — the carrier never reaches its free speed once it is tasked with spinning the cradle.
- **Measured composite speed:** ω\_out,meas, read directly off the output encoder.
- **Deviation:** (ω\_out,meas − ω\_out,pred) / ω\_out,pred, as a percentage.

Expected sources of deviation, roughly in order of likely size: the carrier's loaded speed sagging further under the inner motor's reaction torque than the no-load baseline predicted; bearing friction in the cradle's support bearings; any residual imbalance in the cradle/inner-motor assembly causing vibration losses; and slip-ring contact resistance adding a small additional electrical load on the inner motor's supply.

The role-swap trial (Phase 3, step 4) should show a measurably larger deviation — or outright stalling — when the lower-torque-margin motor is forced into the carrier role, confirming the asymmetry discussed in the Physics Recap. Summarize all runs in one comparison table (carrier role, ω\_out,pred, ω\_out,meas, deviation %) at the top of `/analysis`.

## Safety Notes & Appendix

**Safety:**

- Keep the locked-rotor test (Phase 1, step 2) brief — a few seconds — stall current can exceed rated current several-fold and overheat the winding.
- Tie back hair/clothing and keep fingers clear of the cradle and output shaft once either motor is powered; a spinning cradle with an off-axis motor inside it can throw noticeable vibration.
- Confirm free rotation by hand (Assembly step 11) before every power-on, every time the rig is reassembled.
- Use a current-limited bench supply where possible, and don't leave an unattended motor energized against a stall.

**Reference formulas:**

```latex
\omega_{free} = k_V \times V
```

```latex
R = \dfrac{V_{applied}}{I_{stall}}
```

```latex
\tau_{stall} = k_T \times I_{stall}, \quad k_T = \dfrac{1}{k_V}\ \text{(SI units)}
```

```latex
\tau(\omega) = \tau_{stall}\left(1 - \dfrac{\omega}{\omega_{free}}\right)
```

```latex
P_{mech} = \tau \times \omega
```
