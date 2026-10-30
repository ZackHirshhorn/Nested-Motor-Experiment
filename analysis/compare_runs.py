"""Phase 3: predicted vs measured composite speed for every combined run.

    python analysis/compare_runs.py
    python analysis/compare_runs.py --r-slip 0.12          # measured slip-ring resistance, ohm
    python analysis/compare_runs.py --data-dir data/example --out-dir analysis/example

Reads data/combined_runs.csv and the Phase 1 fits in analysis/params/. Writes
analysis/RESULTS.md with the comparison table first, then plots.

Two predictions are reported side by side:
  * plan   — the plan's formula: omega_carrier from the baseline run (inner
             unpowered) + the inner motor's bench free speed. Only valid at zero
             output load.
  * series — the series model in motor_model.py, using the parasitic torque
             measured from the baseline run's carrier current. It covers loaded
             runs as well.

Two checks test the physics directly instead of the speed sum:
  * inner reaction: the carrier's extra torque (from its current) when the
    inner motor is powered should equal tau_out. Its internal friction should
    not appear in it.
  * inner relative speed: rpm_out - rpm_cradle should match the inner motor's
    own model at torque tau_out. The carrier's rotation should not affect it.
"""

import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from motor_model import RPM, MotorModel, brake_torque, fnum, nested_prediction, read_rows

ROOT = Path(__file__).resolve().parent.parent


def pct(meas, pred):
    return 100 * (meas - pred) / pred


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data-dir", type=Path, default=ROOT / "data")
    ap.add_argument("--out-dir", type=Path, default=ROOT / "analysis")
    ap.add_argument("--r-slip", type=float, default=0.0, help="slip-ring circuit resistance, both brushes (ohm)")
    args = ap.parse_args(argv)
    rows = read_rows(args.data_dir / "combined_runs.csv")
    if not rows:
        print("combined_runs.csv has no data yet")
        return
    models = {n: MotorModel.load(args.out_dir / "params" / f"motor_{n}.json") for n in ("A", "B")}

    by_config = defaultdict(list)
    for r in rows:
        by_config[r["carrier"]].append(r)

    table = [
        "| Run | Carrier | V_c | V_i | τ_out (mN·m) | ω_out meas (rpm) | plan pred | plan dev % | series pred | series dev % "
        "| cradle meas / pred (rpm) | rel meas / pred (rpm) | inner reaction vs τ_out (mN·m) |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    notes, plot_data = [], {}
    for c, runs in sorted(by_config.items()):
        i = "B" if c == "A" else "A"
        car, inn = models[c], models[i]
        base = [r for r in runs if fnum(r, "inner_powered") == 0 and brake_torque(r) == 0]
        if not base:
            notes.append(f"Carrier {c}: no baseline run (inner unpowered, no brake) — cannot compute predictions.")
            continue
        b = base[-1]
        w_base = fnum(b, "rpm_cradle") * RPM
        tau_par = car.torque_from_current(fnum(b, "I_carrier"), w_base)
        notes.append(
            f"Carrier {c}: baseline cradle speed {w_base / RPM:.0f} rpm at {fnum(b, 'V_carrier'):g} V "
            f"(bench free speed {car.free_speed(fnum(b, 'V_carrier')) / RPM:.0f} rpm); parasitic torque from the "
            f"carrier current: **{tau_par * 1e3:.2f} mN·m** (= {100 * tau_par / car.stall_torque(fnum(b, 'V_carrier')):.0f}% "
            "of its stall torque)."
        )
        pts = []
        for r in runs:
            if fnum(r, "inner_powered") == 0:
                continue
            Vc, Vi = fnum(r, "V_carrier"), fnum(r, "V_inner")
            tau = brake_torque(r)
            pred = nested_prediction(car, inn, Vc, Vi, tau, tau_par, args.r_slip)
            plan = w_base + inn.free_speed(Vi)
            w_out, w_cr = fnum(r, "rpm_out") * RPM, fnum(r, "rpm_cradle") * RPM
            reaction = car.torque_from_current(fnum(r, "I_carrier"), w_cr) - tau_par
            table.append(
                f"| {r.get('run_id', '')} | {c} | {Vc:g} | {Vi:g} | {tau * 1e3:.2f} | {w_out / RPM:.0f} | {plan / RPM:.0f} "
                f"| {pct(w_out, plan):+.1f} | {pred['w_out'] / RPM:.0f} | {pct(w_out, pred['w_out']):+.1f} "
                f"| {w_cr / RPM:.0f} / {pred['w_cradle'] / RPM:.0f} | {(w_out - w_cr) / RPM:.0f} / {pred['w_rel'] / RPM:.0f} "
                f"| {reaction * 1e3:.2f} vs {tau * 1e3:.2f} |"
            )
            pts.append((tau, w_out, Vc, Vi))
        if pts:
            Vc, Vi = pts[-1][2], pts[-1][3]
            tmax = max(p[0] for p in pts) * 1.2 or 1e-3
            t = np.linspace(0, tmax, 50)
            line = [nested_prediction(car, inn, Vc, Vi, x, tau_par, args.r_slip)["w_out"] for x in t]
            plot_data[c] = (t, np.array(line), pts)

    fig, ax = plt.subplots(figsize=(6.5, 4))
    for c, (t, line, pts) in plot_data.items():
        h = ax.plot(t * 1e3, line / RPM, label=f"series model, carrier {c}")
        ax.plot([p[0] * 1e3 for p in pts], [p[1] / RPM for p in pts], "o", color=h[0].get_color(), label=f"measured, carrier {c}")
    ax.set_xlabel("output torque τ_out (mN·m)")
    ax.set_ylabel("ω_out (rpm)")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("Combined rig: measured vs predicted")
    fig.tight_layout()
    (args.out_dir / "plots").mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out_dir / "plots" / "combined_runs.png", dpi=110)

    doc = [
        "# Results — predicted vs measured composite speed",
        "",
        "_Generated by `python analysis/compare_runs.py`; re-run after adding rows to combined_runs.csv._",
        "",
        *table,
        "",
        "Deviation = (measured − predicted) / predicted. *plan* = baseline cradle speed + inner bench free speed; "
        "*series* = series model with measured parasitic torque.",
        "",
        "## Notes",
        "",
        *[f"- {n}" for n in notes],
        "",
        "![combined runs](plots/combined_runs.png)",
        "",
        "Phase 1 fits: [motor_summary.md](motor_summary.md) · carrier choice: [carrier_selection.md](carrier_selection.md)",
        "",
    ]
    (args.out_dir / "RESULTS.md").write_text("\n".join(doc), encoding="utf-8")
    print("\n".join(table))
    print("\n".join(notes))


if __name__ == "__main__":
    main()
