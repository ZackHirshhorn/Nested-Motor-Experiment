"""Validate the analysis pipeline on synthetic data with a known answer.

    python analysis/selftest.py

Simulates two small motors with known constants, the Phase 1 bench tests
(no-load, locked rotor, rope brake, two spin-ups), and the Phase 3 combined
runs (series physics with a parasitic cradle torque and a resistive slip ring),
with realistic measurement noise. Writes them in the real file formats to
data/example/, runs characterize -> select_carrier -> compare_runs on them
(results in analysis/example/), and checks that the known values come back.
The example files double as templates for the real data.
"""

import csv
import math
from pathlib import Path

import numpy as np

import characterize
import compare_runs
import select_carrier
from motor_model import RPM, MotorModel, nested_prediction

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data" / "example"
OUT = ROOT / "analysis" / "example"
rng = np.random.default_rng(7)

TRUE = {
    "A": MotorModel("A", kE=0.0030, R=1.6, i0_a=0.08, i0_b=8e-5, J=1.0e-7),
    "B": MotorModel("B", kE=0.0026, R=2.2, i0_a=0.06, i0_b=6e-5, J=0.8e-7),
}
TAU_PAR = 0.8e-3      # N m, cradle bearings + slip ring
R_SLIP = 0.15         # ohm
DRUM_R = 8.25e-3      # m, drum radius + half cord
J_DRUM = 3.9e-7       # kg m^2, printed dyno drum
J_WASHER = 3.7e-7     # kg m^2, steel washer
V_TEST = 3.0


def noisy_rpm(w):
    return w / RPM * (1 + rng.normal(0, 0.003))


def noisy_i(i):
    return i * (1 + rng.normal(0, 0.01)) + rng.normal(0, 0.003)


def write(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def bench(m: MotorModel):
    rows = []
    for k in range(3):  # locked rotor at three rotor positions, brush resistance scatter
        V = 1.0
        rows.append(["locked", V, round(V / (m.R * (1 + rng.normal(0, 0.03))), 4), 0, "", "", "", f"position {k + 1}"])
    for V in (1.5, 3.0, 4.5):
        w = m.free_speed(V)
        rows.append(["noload", V, round(noisy_i(m.current(V, w)), 4), round(noisy_rpm(w)), "", "", "", ""])
    for frac in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6):
        tau = frac * m.stall_torque(V_TEST)
        w = m.speed(V_TEST, tau)
        slack = rng.uniform(0.05, 0.2)
        tight = slack + tau / DRUM_R + rng.normal(0, 0.01)
        rows.append(["brake", V_TEST, round(noisy_i(m.current(V_TEST, w)), 4), round(noisy_rpm(w)),
                     round(tight, 3), round(slack, 3), DRUM_R * 1000, ""])
    write(DATA / f"motor_{m.name}.csv",
          ["test", "voltage_V", "current_A", "rpm", "T_tight_N", "T_slack_N", "drum_radius_mm", "notes"], rows)


def spinup_file(m: MotorModel, J_added, tag, ppr=4):
    J = m.J + J_DRUM + J_added
    c0 = m.stall_torque(V_TEST) / J
    wf = m.free_speed(V_TEST)
    c1 = c0 / wf
    t = np.linspace(0, 6 / c1, 200000)
    theta = wf * (t - (1 - np.exp(-c1 * t)) / c1)
    slot_err = np.radians(rng.normal(0, 1.0, ppr))   # printed slot spacing error
    n = int(theta[-1] / (2 * np.pi / ppr))
    targets = np.arange(1, n) * 2 * np.pi / ppr + slot_err[np.arange(1, n) % ppr]
    tp = np.interp(targets, theta, t)
    t_us = np.round(tp * 1e6 / 4) * 4 + 12_345_678   # micros() has 4 us resolution
    lines = [f"# mode=raw ch=0 ppr={ppr} V={V_TEST}" + (f" J_added={J_added}" if J_added else ""), "t_us"]
    lines += [str(int(x)) for x in t_us]
    (DATA / f"spinup_{m.name}_{tag}.csv").write_text("\n".join(lines) + "\n")


def combined():
    rows, k = [], 0
    for c, i in (("A", "B"), ("B", "A")):
        car, inn = TRUE[c], TRUE[i]
        # baseline: inner unpowered, its rotor just rides along
        w = car.speed(V_TEST, TAU_PAR)
        k += 1
        rows.append([k, c, V_TEST, round(noisy_i(car.current(V_TEST, w)), 4), 0, 0, 0, "", "", "",
                     round(noisy_rpm(w)), round(noisy_rpm(w)), "baseline"])
        for tau in (0.0, 0.4e-3, 0.8e-3, 1.2e-3, 1.6e-3):
            pr = nested_prediction(car, inn, V_TEST, V_TEST, tau, TAU_PAR, R_SLIP)
            slack = rng.uniform(0.05, 0.2) if tau else 0
            tight = slack + tau / DRUM_R + (rng.normal(0, 0.01) if tau else 0)
            w_c = noisy_rpm(pr["w_cradle"])
            k += 1
            rows.append([k, c, V_TEST, round(noisy_i(pr["I_carrier"]), 4), V_TEST, round(noisy_i(pr["I_inner"]), 4), 1,
                         round(tight, 3) if tau else "", round(slack, 3) if tau else "", DRUM_R * 1000 if tau else "",
                         round(w_c), round(w_c + noisy_rpm(pr["w_rel"])), ""])
    write(DATA / "combined_runs.csv",
          ["run_id", "carrier", "V_carrier", "I_carrier", "V_inner", "I_inner", "inner_powered", "T_tight_N", "T_slack_N",
           "drum_radius_mm", "rpm_cradle", "rpm_out", "notes"], rows)


def check(label, got, want, tol):
    err = abs(got - want) / abs(want)
    status = "ok " if err <= tol else "FAIL"
    print(f"  [{status}] {label:38s} got {got:.4g}  true {want:.4g}  ({100 * err:.1f}% , tol {100 * tol:.0f}%)")
    return err <= tol


def main():
    DATA.mkdir(parents=True, exist_ok=True)
    for m in TRUE.values():
        bench(m)
        spinup_file(m, 0.0, "base")
        spinup_file(m, J_WASHER, "flywheel")
    combined()

    print("--- characterize")
    characterize.main(["--data-dir", str(DATA), "--out-dir", str(OUT)])
    print("--- select_carrier")
    select_carrier.main(["--params-dir", str(OUT / "params"), "--parasitic-mNm", str(TAU_PAR * 1e3)])
    print("--- compare_runs")
    compare_runs.main(["--data-dir", str(DATA), "--out-dir", str(OUT), "--r-slip", str(R_SLIP)])

    print("--- recovered vs true")
    ok = True
    for name, t in TRUE.items():
        f = MotorModel.load(OUT / "params" / f"motor_{name}.json")
        ok &= check(f"{name} kE", f.kE, t.kE, 0.03)
        ok &= check(f"{name} R", f.R, t.R, 0.06)
        ok &= check(f"{name} free speed @3V", f.free_speed(V_TEST), t.free_speed(V_TEST), 0.02)
        ok &= check(f"{name} stall torque @3V", f.stall_torque(V_TEST), t.stall_torque(V_TEST), 0.08)
        ok &= check(f"{name} J (rotor + drum, spin-up)", f.J, t.J + J_DRUM, 0.10)
    results = (OUT / "RESULTS.md").read_text(encoding="utf-8")
    for c in ("A", "B"):
        tag = f"Carrier {c}: baseline"
        line = next(l for l in results.splitlines() if tag in l)
        tp = float(line.split("**")[1].split()[0]) * 1e-3
        ok &= check(f"carrier {c} parasitic torque from current", tp, TAU_PAR, 0.25)
    print("SELFTEST " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
