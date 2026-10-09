"""Correct the IMU timestamps of a recording against its LiDAR timestamps (pure Python).

Usage:
  python tools/restamp_imu.py <imu_in.csv> <imu_out.csv> --offset-ms A [--drift-ms-per-s B]

The IMU stamps are late against the LiDAR's by d(t) = A + B * t [ms], t in s from the first IMU sample (measure A and B
with tools/imu_time_offset.py, per segment for B). Each sample's `timestamp` and `timestampUnix` [ns] are reduced by d(t);
samples that would fall before zero are dropped. All other columns are copied unchanged. The output must be a new file;
keep the original outside the recording folder, so it can't be processed by accident.

Recording 44 (dLidar, Unitree L2) was corrected on 2026-10-09 with --offset-ms 79.4 --drift-ms-per-s -0.0983.
"""
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--offset-ms", type=float, required=True, help="lag of the IMU behind the LiDAR at the first sample [ms]")
    ap.add_argument("--drift-ms-per-s", type=float, default=0.0, help="change of the lag per second of recording [ms/s]")
    args = ap.parse_args()
    if args.src == args.dst:
        raise SystemExit("the output must be a new file")

    a_ns = args.offset_ms * 1e6
    b = args.drift_ms_per_s * 1e-3  # ns of lag per ns of recording
    kept = dropped = 0
    prev = None
    t0 = None
    last_shift = 0.0
    with open(args.src) as f, open(args.dst, "w", newline="") as g:
        header = f.readline()
        cols = header.split()
        it, iu = cols.index("timestamp"), cols.index("timestampUnix")
        g.write(header)
        for line in f:
            v = line.split()
            if len(v) < len(cols):
                continue
            ts = int(v[it])
            if t0 is None:
                t0 = ts
            last_shift = a_ns + b * (ts - t0)
            new = int(round(ts - last_shift))
            if new < 0:
                dropped += 1
                continue
            if prev is not None and new <= prev:
                raise SystemExit(f"timestamps would not increase at {ts}: the drift is too large")
            prev = new
            v[it] = str(new)
            v[iu] = str(int(v[iu]) - (ts - new))
            g.write(" ".join(v) + "\n")
            kept += 1
    print(f"kept {kept}, dropped {dropped}; shift {args.offset_ms:.1f} ms at the first sample, {last_shift / 1e6:.1f} ms at the last")


if __name__ == "__main__":
    main()
