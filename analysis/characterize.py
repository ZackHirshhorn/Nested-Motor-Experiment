"""Phase 1: fit each motor's constants from its bench data.

    python analysis/characterize.py                     # reads data/, writes analysis/
    python analysis/characterize.py --data-dir data/example --out-dir analysis/example

Inputs (per motor X):
    data/motor_X.csv              rows of test = noload | locked | brake (columns in docs/PROCEDURE.md)
    data/spinup_X_base.csv        optional raw-mode logger capture, dyno drum only
    data/spinup_X_flywheel.csv    optional, same with the steel washer added (header: # J_added=<kg m^2>)

Outputs:
    <out>/params/motor_X.json     fitted MotorModel, consumed by select_carrier.py and compare_runs.py
    <out>/motor_summary.md        the Phase 1 table
    <out>/plots/torque_speed_X.png
"""

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from motor_model import RPM, MotorModel, fit_motor, inertia_from_two_spinups, read_rows, read_spinup, spinup_line, fnum

ROOT = Path(__file__).resolve().parent.parent


def spinup(data_dir: Path, name: str):
    base, fly = data_dir / f"spinup_{name}_base.csv", data_dir / f"spinup_{name}_flywheel.csv"
    if not (base.exists() and fly.exists()):
        return None
    mb, tb = read_spinup(base)
    mf, tf = read_spinup(fly)
    c0b, c1b = spinup_line(tb, int(mb.get("ppr", 4)))
    c0f, c1f = spinup_line(tf, int(mf.get("ppr", 4)))
    J = inertia_from_two_spinups(c0b, c0f, float(mf["J_added"]))
    return {"J": J, "tau_stall": J * c0b, "w_free": c0b / c1b, "V": float(mb.get("V", "nan"))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data-dir", type=Path, default=ROOT / "data")
    ap.add_argument("--out-dir", type=Path, default=ROOT / "analysis")
    ap.add_argument("--motors", nargs="+", default=["A", "B"])
    args = ap.parse_args(argv)
    (args.out_dir / "params").mkdir(parents=True, exist_ok=True)
    (args.out_dir / "plots").mkdir(parents=True, exist_ok=True)

    lines = [
        "# Phase 1 — motor characterization",
        "",
        "Values at each motor's highest no-load test voltage. *Apparent kV* is the plan's "
        "definition (free rpm / V, includes friction and IR drop); *back-EMF kV* is 1/kE, the "
        "true motor constant. kT is fitted independently from the rope-brake points: it should "
        "match kE (same number in SI) if the data are self-consistent.",
        "",
        "| Motor | V test | free rpm | apparent kV (rpm/V) | back-EMF kV (rpm/V) | kE = kT (mN·m/A) | kT from brake (mN·m/A) "
        "| R (Ω) | I0 @ free (A) | I stall (A) | τ stall (mN·m) | brake fit RMS (rpm) | J spin-up (g·cm²) | τ stall spin-up (mN·m) |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name in args.motors:
        path = args.data_dir / f"motor_{name}.csv"
        rows = read_rows(path)
        if not rows:
            print(f"{path}: no data yet, skipping")
            continue
        m, rep = fit_motor(name, rows)
        V = max(fnum(r, "voltage_V") for r in rows if r["test"] == "noload")
        sp = spinup(args.data_dir, name)
        if sp:
            m.J = sp["J"]
        m.save(args.out_dir / "params" / f"motor_{name}.json")
        wf = m.free_speed(V)
        cell = lambda v, f: f.format(v) if v is not None and not (isinstance(v, float) and math.isnan(v)) else "–"
        lines.append(
            f"| {name} | {V:.2f} | {wf / RPM:.0f} | {m.apparent_kv(V):.0f} | {m.kv_rpm_per_volt:.0f} | {m.kE * 1e3:.3f} "
            f"| {cell(rep.get('kT_brake', float('nan')) * 1e3, '{:.3f}')} | {m.R:.3f} | {m.i0(wf):.3f} "
            f"| {m.stall_current(V):.2f} | {m.stall_torque(V) * 1e3:.2f} | {cell(rep.get('brake_speed_rms_rpm'), '{:.0f}')} "
            f"| {cell(sp and sp['J'] * 1e7, '{:.2f}')} | {cell(sp and sp['tau_stall'] * 1e3, '{:.2f}')} |"
        )
        if rep["R_spread_pct"] > 15:
            lines.append(f"\n> Motor {name}: locked-rotor R varies {rep['R_spread_pct']:.0f}% across rotor positions — "
                         "take more positions or lower the test voltage.\n")

        fig, ax = plt.subplots(figsize=(6, 4))
        tau = np.linspace(0, m.stall_torque(V), 50)
        ax.plot(tau * 1e3, m.speed(V, tau) / RPM, label=f"model @ {V:g} V")
        if "brake_points" in rep:
            pts = [(t, w) for v, t, w in rep["brake_points"] if abs(v - V) < 1e-6]
            if pts:
                ax.plot([t * 1e3 for t, _ in pts], [w for _, w in pts], "o", label="rope brake")
        ax.set_xlabel("shaft torque (mN·m)")
        ax.set_ylabel("speed (rpm)")
        ax.set_title(f"Motor {name}")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.out_dir / "plots" / f"torque_speed_{name}.png", dpi=110)
        plt.close(fig)
        print(f"motor {name}: kE={m.kE:.5f} V·s/rad, R={m.R:.3f} Ω, I0={m.i0_a:.3f}+{m.i0_b:.2e}·ω A, "
              f"free {wf / RPM:.0f} rpm, stall {m.stall_torque(V) * 1e3:.2f} mN·m @ {V:g} V")
    (args.out_dir / "motor_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
