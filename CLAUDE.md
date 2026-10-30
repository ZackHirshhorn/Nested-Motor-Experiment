# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A four-phase experiment: characterize two DC motors, design 3D-printed fixtures, build, and test. One motor (the carrier) spins the whole body of the other (the inner motor). The plan is `docs/Nested DC-Motor Rotation Experiment.md`. `docs/DESIGN_NOTES.md` records where and why the implementation departs from that plan. Read it before changing the mechanics or the analysis model.

## Commands

```
python cad/rig.py --carrier both   # build all parts, run clearance checks, export STL/STEP, regenerate docs/BOM.md
python cad/rig.py --no-check       # same without the clearance checks (~2x faster)
python cad/render.py               # regenerate docs/rig_section.png
python analysis/selftest.py        # synthetic end-to-end check of the whole analysis pipeline
python analysis/characterize.py    # Phase 1 fits from data/motor_*.csv
python analysis/select_carrier.py --parasitic-mNm <x>
python analysis/compare_runs.py --r-slip <ohm>
```

Run from the repo root with the system Python (build123d, numpy, scipy and matplotlib are installed globally). `cad/rig.py` exiting 0 with `clearance check passed` is the CAD's validation. `analysis/selftest.py` printing `SELFTEST PASSED` is the analysis's validation.

## Architecture

- `cad/parameters.py` holds every number. Motor dimensions come from `data/motor_dims.csv`; rows whose `source` starts with `ASSUMED` are placeholders and trigger warnings. Keep the MEASURED / CATALOG / ASSUMED distinction in comments when editing.
- `cad/rig.py`:
  - `make_layout()` derives the whole axial stack and the axis height from the two motors. Every station is computed from its neighbour plus a named gap; nothing is hard-coded.
  - Rotating parts are built in a local frame whose +Z is the spin axis with z == rig x, and are mapped by `to_rig()`. Static parts are built directly in rig coordinates (X = spin axis, base top at z = 0).
  - `check()` places dummy hardware (motors, bearings, slip ring, sensor forks). It requires 0.3 mm between parts in different rotation groups, and no solid overlap between parts in the same group except the pairs listed in `ALLOWED_OVERLAP`. If you add a part, give it the right group. Add to `ALLOWED_OVERLAP` only for intended fits.
  - Exports go to `cad/exports/carrier_<X>/` and are committed so the printable files are available without running the script. Re-run `python cad/rig.py --carrier both` and commit the exports together with any CAD or parameter change, so they never go stale. `cad/exports/rig_assembly.step` is the colored full-rig assembly to open for a 3D look; `assembly_step()` sets its colors. `docs/BOM.md` is generated, so do not hand-edit it.
- `analysis/motor_model.py`: the motor model V = IR + kE·ω, τ = kT·(I − I0(ω)), with I0 affine in ω; the fitting code; and `nested_prediction()`, the series model (carrier torque = τ_out + τ_par, inner torque = τ_out). Every script imports from here.
- `firmware/tachometer/tachometer.ino`: an Arduino tachometer on D2/D3 at 250000 baud. Its summary and raw output formats are parsed by `analysis/log_serial.py` and `motor_model.read_spinup()`; change them together.

## Data conventions

CSV columns are documented in `docs/PROCEDURE.md`, and `data/example/` contains synthetic samples of every file (written by selftest). Rope-brake torque is always (T_tight − T_slack) × drum_radius_mm, where the radius is measured to the cord centerline.
