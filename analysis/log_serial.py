"""Read the Arduino tachometer (firmware/tachometer) over USB serial.

Steady-state reading (Phase 1 no-load/brake points, Phase 3 runs). Prints
the mean and spread of both channels over the last few seconds; copy the
means into the CSV row:
    python analysis/log_serial.py COM5 --ppr0 2 --ppr1 4
    python analysis/log_serial.py COM5 --ppr0 4            # Phase 1: dyno disc on channel 0

Spin-up capture (inertia method). Start this, then switch the motor on;
stops after --seconds:
    python analysis/log_serial.py COM5 --raw 0 --ppr0 4 --seconds 2 --out data/spinup_A_base.csv --meta V=3.0
    python analysis/log_serial.py COM5 --raw 0 --ppr0 4 --seconds 2 --out data/spinup_A_flywheel.csv --meta V=3.0 J_added=3.7e-7

Needs pyserial (pip install pyserial).
"""

import argparse
import statistics
import sys
import time
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed: pip install pyserial")

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("port")
    ap.add_argument("--ppr0", type=int, default=2, help="pulses per rev on D2 (cradle ring = 2, discs = 4)")
    ap.add_argument("--ppr1", type=int, default=4, help="pulses per rev on D3 (output disc = 4)")
    ap.add_argument("--window", type=float, default=3.0, help="seconds averaged in steady-state mode")
    ap.add_argument("--raw", type=int, choices=[0, 1], help="capture raw pulse timestamps on this channel")
    ap.add_argument("--seconds", type=float, default=2.0, help="raw capture length")
    ap.add_argument("--out", help="raw capture file")
    ap.add_argument("--meta", nargs="*", default=[], help="key=value pairs written to the capture header")
    args = ap.parse_args()

    with serial.Serial(args.port, 250000, timeout=0.5) as s:
        time.sleep(2.0)  # the board resets when the port opens
        s.reset_input_buffer()
        s.write(f"p 0 {args.ppr0}\np 1 {args.ppr1}\n".encode())

        if args.raw is not None:
            if not args.out:
                sys.exit("--raw needs --out")
            ppr = args.ppr0 if args.raw == 0 else args.ppr1
            s.write(f"r {args.raw}\n".encode())
            lines, t_end = [], None
            print("capturing — switch the motor on now")
            while t_end is None or time.time() < t_end:
                line = s.readline().decode(errors="replace").strip()
                if line.isdigit():
                    if t_end is None:
                        t_end = time.time() + args.seconds
                    lines.append(line)
            s.write(b"s\n")
            header = f"# mode=raw ch={args.raw} ppr={ppr} " + " ".join(args.meta)
            with open(args.out, "w") as f:
                f.write(header + "\nt_us\n" + "\n".join(lines) + "\n")
            print(f"{len(lines)} pulses written to {args.out}")
            return

        hist = deque()
        print("steady-state mode; Ctrl-C to stop")
        try:
            while True:
                parts = s.readline().decode(errors="replace").strip().split(",")
                if len(parts) != 5 or not parts[0].isdigit():
                    continue
                t = int(parts[0]) / 1000
                hist.append((t, float(parts[1]), float(parts[2])))
                while hist and hist[0][0] < t - args.window:
                    hist.popleft()
                if len(hist) >= 3:
                    r0 = [h[1] for h in hist]
                    r1 = [h[2] for h in hist]
                    print(f"\rch0 {statistics.mean(r0):8.0f} ± {statistics.stdev(r0):5.0f} rpm   "
                          f"ch1 {statistics.mean(r1):8.0f} ± {statistics.stdev(r1):5.0f} rpm   "
                          f"(last {args.window:g} s)   ", end="", flush=True)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
