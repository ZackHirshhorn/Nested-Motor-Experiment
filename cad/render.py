"""Draw a labelled cross-section of the rig through the spin axis (XZ plane).

    python cad/render.py                 # -> docs/rig_section.png (carrier = A)
    python cad/render.py --carrier B
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import parameters as p
import rig

COLORS = {"static": "#a7b1bc", "carrier": "#e07a3f", "output": "#3f8fe0", "bearing": "#4a4a4a"}
HARDWARE = {"slip_ring": "#c9a93b", "motor_carrier_can": "#707070", "motor_inner_can": "#707070",
            "motor_carrier_shaft": "#303030", "motor_inner_shaft": "#303030",
            "sensor_cradle": "#2e7d32", "sensor_output": "#2e7d32"}
LABELS = {
    "carrier_mount_lower": "carrier mount", "motor_carrier_can": "carrier motor", "coupler_hub": "coupler hub",
    "oldham_disc": "Oldham disc", "pedestal_rear_lower": "rear pedestal\n(floating brg)",
    "slip_ring": "slip ring", "rear_cap": "rear cap +\ncradle encoder ring", "motor_inner_can": "inner motor",
    "cradle_body": "cradle body", "pedestal_front_lower": "front pedestal\n(locating brg)",
    "encoder_disc": "output\nencoder disc", "base_plate": "base plate",
}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--carrier", choices=["A", "B"], default="A")
    args = ap.parse_args(argv)
    motors = p.load_motor_dims()
    b = rig.build(motors[args.carrier], motors["B" if args.carrier == "A" else "A"])
    slab = rig.box(-500, 500, -0.05, 0.05, -50, 200)

    fig, ax = plt.subplots(figsize=(13, 5.5))
    labels = []
    for it in b.phase3:
        if it.name.startswith("sensor_bracket"):
            continue
        cut = it.shape & slab
        if cut.volume < 1e-6:
            continue
        verts, tris = cut.tessellate(0.05, 0.3)
        V = np.array([(v.X, v.Z) for v in verts])
        color = HARDWARE.get(it.name, COLORS[it.group])
        for t in tris:
            ax.fill(*V[list(t)].T, color=color, linewidth=0)
        if it.name in LABELS:
            bb = cut.bounding_box()
            labels.append(((bb.min.X + bb.max.X) / 2, bb.max.Z, it.name))
    # stagger labels over three rows so neighbours never collide
    rows = [b.L.H + 30, b.L.H + 42, b.L.H + 54]
    for k, (x, z, name) in enumerate(sorted(l for l in labels if l[2] != "base_plate")):
        ax.annotate(LABELS[name], (x, z), xytext=(x, rows[k % 3]), ha="center", fontsize=8,
                    arrowprops=dict(arrowstyle="-", color="#888", lw=0.6))
    x, z, name = next(l for l in labels if l[2] == "base_plate")
    ax.annotate(LABELS[name], (x, -8), xytext=(x, -18), ha="center", fontsize=8)
    ax.axhline(b.L.H, color="#555", lw=0.5, ls="-.")
    ax.set_aspect("equal")
    ax.set_xlabel("x along spin axis (mm)")
    ax.set_ylabel("z (mm)")
    ax.set_ylim(-24, b.L.H + 64)
    ax.set_title(f"Section through the spin axis — carrier {args.carrier}   "
                 "(orange: turns with the cradle, blue: output speed, grey: static)", fontsize=10)
    fig.tight_layout()
    out = p.ROOT / "docs" / "rig_section.png"
    fig.savefig(out, dpi=120)
    print(f"wrote {out.relative_to(p.ROOT)}")


if __name__ == "__main__":
    main()
