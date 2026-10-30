"""PM brushed DC motor model, parameter fitting, and the nested-rig prediction.

Motor model (SI units, omega in rad/s):

    V = I R + kE omega                 electrical
    tau_shaft = kT (I - I0(omega))     mechanical, kT = kE for a PM DC motor
    I0(omega) = i0_a + i0_b omega      no-load (friction + iron loss) current

At a fixed voltage this gives the linear torque-speed line the plan assumes,
but with friction kept separate from the electromagnetic constants, so kE
fitted from no-load runs at several voltages is a true motor constant.

Nested rig (derivation in docs/DESIGN_NOTES.md): with the inner motor's
housing carried by the cradle, its internal friction acts on both its own
rotor and its own stator and cancels. The torque the inner motor puts back
into the cradle is exactly the external output load tau_out. So in steady
state:

    carrier torque  = tau_out + tau_par     (tau_par = cradle bearings + slip ring + windage)
    inner torque    = tau_out
    omega_out       = omega_carrier(tau_out + tau_par) + omega_inner_rel(tau_out)

The two motors are in series: same torque, speeds add.
"""

import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

# Results use Greek symbols; the Windows console defaults to cp1252.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

RPM = 2 * math.pi / 60  # rpm -> rad/s


@dataclass
class MotorModel:
    name: str
    kE: float      # V s/rad  (== kT in N m/A)
    R: float       # ohm, terminal resistance (winding + brushes)
    i0_a: float    # A
    i0_b: float    # A s/rad
    J: float = float("nan")  # kg m^2, rotor (+ dyno drum) inertia, from the spin-up test

    @property
    def kT(self):
        return self.kE

    @property
    def kv_rpm_per_volt(self):
        """Back-EMF speed constant (the friction-free kV)."""
        return 1 / self.kE / RPM

    def i0(self, w):
        return self.i0_a + self.i0_b * w

    def free_speed(self, V):
        return (V - self.R * self.i0_a) / (self.kE + self.R * self.i0_b)

    def stall_current(self, V):
        return V / self.R

    def stall_torque(self, V):
        return self.kT * (V / self.R - self.i0_a)

    def speed(self, V, tau):
        """Steady speed (rad/s) delivering shaft torque tau (N m) at voltage V.
        Valid past stall too (negative speed = being back-driven)."""
        return (V - self.R * self.i0_a - self.R * tau / self.kT) / (self.kE + self.R * self.i0_b)

    def current(self, V, w):
        return (V - self.kE * w) / self.R

    def torque_from_current(self, I, w):
        return self.kT * (I - self.i0(w))

    def apparent_kv(self, V):
        """kV as the plan defines it: free speed / voltage (includes friction and IR drop)."""
        return self.free_speed(V) / RPM / V

    def save(self, path: Path):
        path.write_text(json.dumps(asdict(self), indent=2))

    @classmethod
    def load(cls, path: Path):
        return cls(**json.loads(path.read_text()))


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def read_rows(path: Path):
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(line for line in f if not line.startswith("#"))]
    return rows


def fnum(row, key, default=float("nan")):
    v = (row.get(key) or "").strip()
    return float(v) if v else default


def brake_torque(row):
    """Rope brake: tau = (T_tight - T_slack) * r_eff, r_eff to the cord centerline."""
    t, s = fnum(row, "T_tight_N", 0.0), fnum(row, "T_slack_N", 0.0)
    r = fnum(row, "drum_radius_mm", 0.0) / 1000
    return (t - s) * r


# ---------------------------------------------------------------------------
# Fitting from Phase 1 data
# ---------------------------------------------------------------------------
def fit_motor(name: str, rows) -> tuple[MotorModel, dict]:
    locked = [r for r in rows if r["test"] == "locked"]
    noload = [r for r in rows if r["test"] == "noload"]
    brake = [r for r in rows if r["test"] == "brake"]
    if not locked or not noload:
        raise ValueError(f"motor {name}: need at least one 'locked' and one 'noload' row")

    # R: average over rotor positions (brush contact varies with commutator angle)
    r_each = np.array([fnum(r, "voltage_V") / fnum(r, "current_A") for r in locked])
    R = float(np.mean(r_each))

    V = np.array([fnum(r, "voltage_V") for r in noload])
    I = np.array([fnum(r, "current_A") for r in noload])
    w = np.array([fnum(r, "rpm") for r in noload]) * RPM
    # kE through the origin:  V - I R = kE w
    kE = float(np.sum((V - I * R) * w) / np.sum(w * w))
    if len(set(np.round(V, 3))) >= 2:
        i0_b, i0_a = np.polyfit(w, I, 1)
    else:
        i0_a, i0_b = float(np.mean(I)), 0.0
    m = MotorModel(name, kE, R, float(i0_a), float(i0_b))

    report = {"R_spread_pct": float(100 * np.std(r_each) / R) if len(r_each) > 1 else 0.0,
              "n_locked": len(locked), "n_noload": len(noload), "n_brake": len(brake)}
    if brake:
        Vb = np.array([fnum(r, "voltage_V") for r in brake])
        Ib = np.array([fnum(r, "current_A") for r in brake])
        wb = np.array([fnum(r, "rpm") for r in brake]) * RPM
        tau = np.array([brake_torque(r) for r in brake])
        x = Ib - m.i0(wb)
        # independent kT from the brake: if kT_brake ~= kE the model is self-consistent
        report["kT_brake"] = float(np.sum(tau * x) / np.sum(x * x))
        report["brake_speed_rms_rpm"] = float(np.sqrt(np.mean((m.speed(Vb, tau) - wb) ** 2)) / RPM)
        report["brake_points"] = [(float(a), float(b), float(c)) for a, b, c in zip(Vb, tau, wb / RPM)]
    return m, report


# ---------------------------------------------------------------------------
# Spin-up (inertia) method
# ---------------------------------------------------------------------------
def read_spinup(path: Path):
    """Raw-mode logger file: '# key=value' header lines, then a t_us column."""
    meta, t = {}, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                for tok in line[1:].split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        meta[k] = v
            elif line and line[0].isdigit():
                t.append(int(line))
    return meta, np.array(t, dtype=float) * 1e-6


def spinup_line(t, ppr, min_frac=0.3):
    """Fit domega/dt = c0 - c1 omega to a spin-up. Speeds come from one full
    revolution of pulses (ppr intervals), which cancels slot-spacing errors.
    Only points above min_frac of the top speed are used: at low speed a
    current-limited supply clips the torque, which would bias c0."""
    t = np.unwrap(t, period=2**32 * 1e-6)
    rev_t = t[ppr:] - t[:-ppr]
    w = 2 * math.pi / rev_t
    tc = (t[ppr:] + t[:-ppr]) / 2
    keep = w > min_frac * np.max(w)
    tc, w = tc[keep], w[keep]
    # local slope from a sliding linear fit
    k = max(5, len(w) // 40)
    dw = np.array([np.polyfit(tc[max(0, i - k):i + k + 1], w[max(0, i - k):i + k + 1], 1)[0] for i in range(len(w))])
    use = slice(k, -k) if len(w) > 4 * k else slice(None)
    slope, c0 = np.polyfit(w[use], dw[use], 1)
    return float(c0), float(-slope)


def inertia_from_two_spinups(c0_base, c0_fly, J_added):
    """Same motor, same voltage, so the same stall torque: J_base c0_base = (J_base + J_added) c0_fly."""
    return J_added * c0_fly / (c0_base - c0_fly)


# ---------------------------------------------------------------------------
# Nested-rig prediction
# ---------------------------------------------------------------------------
def nested_prediction(carrier: MotorModel, inner: MotorModel, V_c, V_i, tau_out, tau_par, R_slip=0.0):
    """Steady-state speeds (rad/s) of the cradle, the inner rotor relative to
    the cradle, and the output shaft, plus the currents the series model predicts.
    The slip ring's resistance R_slip is in series with the inner winding."""
    inner = replace(inner, R=inner.R + R_slip)
    w_c = carrier.speed(V_c, tau_out + tau_par)
    w_rel = inner.speed(V_i, tau_out)
    return {
        "w_cradle": w_c,
        "w_rel": w_rel,
        "w_out": w_c + w_rel,
        "I_carrier": carrier.current(V_c, w_c),
        "I_inner": inner.current(V_i, w_rel),
    }


def combined_stall_torque(carrier, inner, V_c, V_i, tau_par):
    """Output torque at which omega_out reaches zero (one motor is then being back-driven)."""
    # omega_out is linear in tau_out: find its root from two evaluations
    f0 = nested_prediction(carrier, inner, V_c, V_i, 0.0, tau_par)["w_out"]
    f1 = nested_prediction(carrier, inner, V_c, V_i, 1e-3, tau_par)["w_out"]
    return -f0 * 1e-3 / (f1 - f0)
