"""End of Phase 1: decide which motor is the carrier, before anything is printed.

    python analysis/select_carrier.py --parasitic-mNm 1.5
    python analysis/select_carrier.py --vc 3 --vi 3 --parasitic-mNm 1.5 --params-dir analysis/example/params

The carrier alone fights the cradle's parasitic torque (bearings + slip ring
+ windage). Get that number from the slip ring's datasheet ("rotating torque")
plus about 0.2 mN·m per ZZ bearing, or, once the cradle is built, measure it
from the baseline run (compare_runs.py prints it). Both motors carry the
output load equally. Two criteria follow:

  1. Start-up margin: the carrier's stall torque must clear tau_par with room
     for breakaway friction, which is typically 2-3x running friction. Required:
     margin >= MIN_MARGIN.
  2. Among the assignments that pass, the better one keeps more speed. Moving
     tau_par onto a motor costs tau_par x (omega_free / tau_stall) of that motor,
     so the carrier should be the motor with the stiffer torque-speed line.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from motor_model import RPM, MotorModel, combined_stall_torque, nested_prediction

ROOT = Path(__file__).resolve().parent.parent
MIN_MARGIN = 3.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--params-dir", type=Path, default=ROOT / "analysis" / "params")
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--vc", type=float, default=3.0, help="carrier supply voltage")
    ap.add_argument("--vi", type=float, default=3.0, help="inner supply voltage")
    ap.add_argument("--parasitic-mNm", type=float, required=True, help="cradle parasitic torque estimate (mN·m)")
    args = ap.parse_args(argv)
    out = args.out_dir or args.params_dir.parent
    tau_par = args.parasitic_mNm * 1e-3
    m = {n: MotorModel.load(args.params_dir / f"motor_{n}.json") for n in ("A", "B")}

    lines = [
        "# Carrier selection",
        "",
        f"Carrier at {args.vc:g} V, inner at {args.vi:g} V, cradle parasitic torque {args.parasitic_mNm:g} mN·m.",
        "",
        "| Carrier | Inner | carrier stall / τ_par | cradle rpm | ω_out free (rpm) | ideal sum of free speeds (rpm) | combined stall torque (mN·m) | verdict |",
        "|---|---|---|---|---|---|---|---|",
    ]
    results = {}
    fig, ax = plt.subplots(figsize=(6.5, 4))
    for c, i in (("A", "B"), ("B", "A")):
        car, inn = m[c], m[i]
        margin = car.stall_torque(args.vc) / tau_par if tau_par > 0 else float("inf")
        pred = nested_prediction(car, inn, args.vc, args.vi, 0.0, tau_par)
        ideal = (car.free_speed(args.vc) + inn.free_speed(args.vi)) / RPM
        ts = combined_stall_torque(car, inn, args.vc, args.vi, tau_par)
        ok = margin >= MIN_MARGIN
        results[c] = (ok, pred["w_out"])
        lines.append(
            f"| {c} | {i} | {margin:.1f} | {pred['w_cradle'] / RPM:.0f} | {pred['w_out'] / RPM:.0f} | {ideal:.0f} "
            f"| {ts * 1e3:.2f} | {'OK' if ok else f'REJECT: margin < {MIN_MARGIN:g}'} |"
        )
        tau = np.linspace(0, ts, 60)
        ax.plot(tau * 1e3, [nested_prediction(car, inn, args.vc, args.vi, t, tau_par)["w_out"] / RPM for t in tau],
                label=f"carrier {c} / inner {i}")
    passing = [c for c, (ok, _) in results.items() if ok]
    lines.append("")
    if not passing:
        lines.append("**Neither motor has the start-up margin.** Reduce the parasitic torque (lower-drag slip ring, "
                     "ZZ instead of 2RS bearings) or use a stronger or geared carrier.")
    else:
        best = max(passing, key=lambda c: results[c][1])
        lines.append(f"**Recommended carrier: motor {best}.**")
    d = (results["A"][1] - results["B"][1]) / RPM
    lines.append(f"\nPredicted role-swap effect on free-running ω_out: {abs(d):.0f} rpm "
                 f"({'A' if d > 0 else 'B'} as carrier is faster). Under output load both motors carry the same torque, "
                 "so this difference stays constant along the combined torque-speed line.")
    ax.set_xlabel("output torque (mN·m)")
    ax.set_ylabel("ω_out (rpm)")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("Predicted combined torque-speed line")
    fig.tight_layout()
    (out / "plots").mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "plots" / "carrier_selection.png", dpi=110)
    (out / "carrier_selection.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
