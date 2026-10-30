# Test procedure

Step-by-step version of the plan's Phases 1 and 3, with the corrections from
[DESIGN_NOTES.md](DESIGN_NOTES.md) applied. Every measurement goes into a CSV in
`/data`; `data/example/` holds filled-in examples of every file.

**Equipment:** two current-limited bench supplies, multimeter, Arduino running
`firmware/tachometer` (upload it with the Arduino IDE), `python analysis/log_serial.py <port>`,
force gauge plus hanging masses, about 1 m of 0.5 mm braided cord, calipers, and one
steel fender washer.

## Phase 0: measure, then print

1. Caliper both motors and fill in `data/motor_dims.csv`. Can diameter (round part),
   width across flats (0 if round), can length (front face to rear face of the end
   bell), boss diameter and length, shaft diameter and length beyond the boss, and
   how far the terminals stick out behind the end bell. Replace `ASSUMED` in `source`
   with the motor's model number.
2. Caliper the slip ring and the slot-sensor module, and update the `ASSUMED` block
   in `cad/parameters.py`.
3. `python cad/rig.py --carrier both`. It must print `clearance check passed` for
   both configurations.
4. Print `fit_coupon.stl`. Check that each bearing presses into its seat and onto its
   journal, that the rear bearing slides on its journal, that the slip ring slides
   onto its seat, and that the motor drops into the pocket slice with its boss
   centered. Adjust the fits in `parameters.py`, re-run, and repeat until all fit.
5. For Phase 1 print only `base_plate`, `carrier_mount_*` for the motor under test,
   `dyno_drum_<motor>`, and one `sensor_bracket`.

## Phase 1: characterize each motor (run once for A, once for B)

Mount the motor in its carrier mount on the base plate. Ream the dyno drum's bore
with a shaft-size drill and press the drum onto the shaft. Mount the sensor bracket
on the −Y side so the disc runs through the fork. Connect the sensor to D2 and run
`log_serial.py <port> --ppr0 4`.

Record every reading as a row in `data/motor_<X>.csv`:

| column | meaning |
|---|---|
| `test` | `locked`, `noload` or `brake` |
| `voltage_V` | supply voltage, measured at the motor terminals |
| `current_A` | supply current |
| `rpm` | steady mean from log_serial (0 for locked) |
| `T_tight_N`, `T_slack_N` | brake rows: hanging-mass weight (m·g) and force-gauge reading |
| `drum_radius_mm` | brake rows: drum radius + half the cord diameter (8.25 for a 0.5 mm cord) |
| `notes` | rotor position, temperature, anything odd |

1. **Locked rotor (R).** Remove the drum and hold the shaft with pliers. Set the
   supply to about 1 V with the current limit just above the expected stall
   current. Apply power for 2–3 s and record V and I. Repeat at **at least three
   rotor positions** (turn the shaft about 1/3 turn each time), because brush contact
   resistance varies with commutator angle. The script averages them and warns
   if they spread more than 15%.
2. **No-load (kE, friction).** With the drum on but no cord, record steady V, I and
   rpm at **three voltages**, for example 50%, 100% and 150% of the voltage you plan
   to run. Wait about 10 s at each for the reading to settle.
3. **Rope brake (validation).** At the run voltage, wrap the cord once around the
   drum. Hang a mass on the tight side and hook the slack side to the force gauge,
   anchored below. Both cord ends go down through the base-plate slots, so set the
   rig at the bench edge. Step the mass from about 10% to 60% of the expected stall
   torque, and record V, I, rpm, mass weight and gauge reading at each step. Keep
   each point short, because the drum heats.
4. **Spin-up (inertia; optional but recommended).** Remove the cord. Run
   `log_serial.py <port> --raw 0 --ppr0 4 --seconds 2 --out data/spinup_<X>_base.csv --meta V=<run V>`,
   then switch the supply on with the current limit set above stall current. Glue
   or tape the weighed washer concentric on the drum face, compute
   J = m·(r_o² + r_i²)/2, and repeat with `--out data/spinup_<X>_flywheel.csv --meta V=<run V> J_added=<J in kg·m²>`.
5. **Slip-ring drag (τ_par estimate).** Put the slip ring on any 12 mm rod, wind
   thread around its rotor sleeve, hold the stator, and pull the thread with the
   force gauge. Torque = F × sleeve radius. Record the breakaway and steady pull
   values. Add about 0.2 mN·m for the two ZZ bearings.

Then:

```
python analysis/characterize.py                       # -> analysis/motor_summary.md, analysis/params/
python analysis/select_carrier.py --vc <V> --vi <V> --parasitic-mNm <step 5 value>
```

`select_carrier.py` writes `analysis/carrier_selection.md` with the recommended
carrier. Set `--carrier` accordingly when printing.

## Phase 2: build

Print the rest of the parts and assemble per [ASSEMBLY.md](ASSEMBLY.md).

## Phase 3: combined runs

Run `log_serial.py <port>`: channel 0 is the cradle ring (2 ppr) and channel 1 is
the output disc (4 ppr). Log every run as a row in `data/combined_runs.csv`:

| column | meaning |
|---|---|
| `run_id` | any unique label |
| `carrier` | `A` or `B` (which motor is carrying) |
| `V_carrier`, `I_carrier` | carrier supply readback |
| `V_inner`, `I_inner` | inner-motor supply readback (on the fixed side of the slip ring) |
| `inner_powered` | 0 = inner leads open (baseline), 1 = powered |
| `T_tight_N`, `T_slack_N`, `drum_radius_mm` | rope brake on the output drum; leave blank for no load |
| `rpm_cradle`, `rpm_out` | channel 0 and channel 1 means |

1. **Polarity.** At about 1 V, check with tape flags that the carrier and inner motor
   turn the same way seen from the output end. Swap leads if they don't.
2. **Baseline.** Power the carrier only, with the inner leads open, at the run
   voltage. Record its current and the cradle rpm. This gives ω_carrier,loaded for
   the plan's formula *and* τ_par from the carrier's current.
3. **Combined, free.** With the carrier running, bring the inner motor up to the run
   voltage on a current-limited supply. Wait for steady state and record. Expect
   rpm_out ≈ rpm_cradle + the inner motor's free speed.
4. **Loaded sweep.** Swap the encoder disc for `dyno_drum_<inner>`. The output
   sensor bracket stays where it is, because the disc sits at the same place on both
   parts. Repeat step 3 at 4–5 brake loads, up to about 60% of the *smaller* motor's
   stall torque.
5. **Role swap.** Rebuild with `cad/exports/carrier_<other>/` parts. If both motors
   have the same dimensions the parts are identical: just swap the motors. Repeat
   steps 1–4.
6. Analyze:

```
python analysis/compare_runs.py --r-slip <measured slip-ring loop resistance, Ω>
```

This writes `analysis/RESULTS.md`, with the predicted-vs-measured table at the top
(plan formula and series model side by side) and the torque-balance checks.

## Safety

- Keep locked-rotor tests to a few seconds and always use a current limit.
- Fit the clear guard over the cradle before powering. Tie back hair and loose
  sleeves. Never touch the cradle or output shaft while either motor is powered.
- Before every power-on, turn the whole drive train by hand and confirm nothing
  binds or rubs.
- Start below 4000 rpm cradle speed and increase gradually. Stop at the first sign
  of vibration and re-check balance and seating. Don't exceed the slip ring's
  rated speed.
- Start the carrier before the inner motor and stop the inner motor first. The
  inner motor's start-up reaction kicks the cradle backward.
- Never leave a motor energized against a stall unattended.
