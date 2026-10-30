# Assembly

Parts come from `cad/exports/carrier_<X>/`, where X is the motor
`select_carrier.py` chose as the carrier. Hardware is listed in [BOM.md](BOM.md).
View `assembly_phase3.step` in any CAD viewer alongside these steps.

Axis convention: +X runs from the carrier toward the output shaft. The cradle
sensor and output sensor sit on the +Y side; the slip-ring stop and the Phase 1
sensor sit on the −Y side.

## 1. Prepare the printed parts

1. Print `fit_coupon.stl` first and tune the fits (see PROCEDURE.md, Phase 0).
2. Print the cradle body and rear cap in PETG at **100% infill**, so their mass is
   uniform and they balance. Print the coupler, pedestals, base and carrier mount
   in PETG. The Oldham disc and dyno drums need supports. Every STL is already
   oriented for printing.
3. Fit the heat-set inserts:
   - pedestal lowers: 2 in the top split face, 2 in the bottom
   - carrier-mount lower: 4 in the top split face, 2 in the bottom
   - cradle body: 3 in the rear flange face
   - each sensor bracket: 2 in the bottom of the foot
   - slip-ring stop: 2 in the bottom
4. **Ream every shaft bore** (coupler hub, encoder disc, dyno drums) with a drill bit
   of exactly the motor shaft diameter, turned by hand in a pin vise. The bores are
   printed at nominal size, which comes out slightly under.

## 2. Cradle sub-assembly (inner motor)

5. Press the **6805ZZ** bearing onto the cradle body's front journal until it seats
   on the shoulder. **Push on the inner race only**, for example with a socket that
   fits the inner race. This is the locating bearing, so it must not slide.
6. Solder about 80 mm of thin silicone wire to each inner-motor terminal. Thread
   the leads through the two holes in the rear cap, from the cavity side out.
7. Slide the inner motor into the cradle body from the rear, flats aligned with the
   pocket flats, until its boss sits in the front lip hole.
8. Fit the rear cap and tighten its 3 screws evenly, a quarter turn at a time. The
   pocket is 0.1 mm short, so the cap clamps the motor axially. Shake the
   sub-assembly: nothing may rattle.
9. Slide the slip ring over the rear hub's narrow end and onto its seat. Keep the
   gap to the rear cap's back face at about 6 mm for wiring, and tighten the ring's
   own grub screws. Splice the motor leads to the ring's rotor leads in the gap,
   cover with heat-shrink, and tie the splices down **opposite each other** to keep
   the balance.
10. Slide the **6801ZZ** bearing onto the rear journal. It is a sliding fit (the
    floating bearing).
11. Drop the Oldham disc into the slot in the end of the rear hub.

## 3. Base and static parts

12. Screw the following up through the base plate from below, leaving the slotted
    ones loose for now: both pedestal lowers, the carrier-mount lower, the
    slip-ring stop, and the two Phase 3 sensor brackets.
13. Seat the carrier motor in its mount, with its boss in the front-wall hole, and
    fit the cap (4 screws). Push the coupler hub onto the carrier shaft until it
    stops about 1 mm short of the front wall. Insert the square nut and tighten the
    set screw onto the shaft.

## 4. Install the cradle

14. Lower the whole cradle sub-assembly into the pedestals: front bearing into the
    front pedestal's half-pocket, rear bearing into the rear one. Turn it so the
    Oldham disc's free tongue drops into the coupler hub's slot.
15. Fit both pedestal caps. Slide the front pedestal along its slots until the
    Oldham disc has a little end play (about 0.5 mm) between the hubs, then tighten
    its base screws and its cap screws. The rear pedestal goes wherever its bearing
    sits naturally on the journal; then tighten it. It must not push the cradle
    axially.
16. Fit an M3 screw radially into the slip-ring stator (a flange hole, or glued on)
    as an anti-rotation pin. Turn the stator so the pin enters the window in the
    stop post. The pin must not touch the window's edges once the stator settles.
17. Press the encoder disc onto the inner motor's output shaft, with its hub toward
    the motor, until the disc sits centered in the output sensor's fork.

## 5. Sensors and wiring

18. Screw each slot-sensor module to its bracket upright. Slide it up or down in
    the upright slot, and the bracket in or out on its base slots, until the
    encoder ring or disc runs through the middle of the fork gap with the windows
    crossing the beam.
19. Wire the cradle sensor to D2 and the output sensor to D3, both with 5 V and
    GND. Wire the carrier to supply 1. Wire the inner motor to supply 2 through the
    slip ring's stator leads.
20. **Final hand check:** turn the coupler by hand through several turns. The
    whole train must turn freely, with nothing rubbing on the sensors, pedestals or
    base. Then hold the cradle and turn the output disc: it must also turn freely.
21. Fit the clear guard over the cradle before applying power.

## Role swap

Run `python cad/rig.py --carrier <other>`. If both motors measured the same, the
parts are identical: just exchange the two motors (steps 6–8 and 13). Otherwise
print the full `carrier_<other>` set. The script lays out the base plate, axis
height and bearing choice again for the swapped motors.
