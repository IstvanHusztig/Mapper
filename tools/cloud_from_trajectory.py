"""A viewable point cloud for a trajectory that has no cloud of its own.

A loop closure replay (`dMapper --replay-loop-closure`) and an odometry snapshot (`odometry_snapshot.bin`) give only poses.
This tool carries the points of a result's merged cloud (`scan_lio_merged.laz`) from that result's trajectory to the other
trajectory: each point keeps its position relative to its own trajectory node (the last node at or before the point's time)
and is placed again with that node's pose in the other trajectory. Both trajectories must come from the same recording with
the same node times. The written cloud has the source cloud's header and attributes; only x, y, z change.

    python tools/cloud_from_trajectory.py <result folder> <trajectory> <out.laz>

<trajectory> is a result or replay folder (trajectory_lio_*.csv) or an odometry snapshot (.bin). Checked on a loop closure
stage against dMapper's own cloud of that stage: within 0.2 mm (the cloud file's resolution); about 25 s for 56 million
points. Prints the largest point move."""
import argparse
import sys
from pathlib import Path

import laspy
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_quality_lib import trajectory  # noqa: E402
import snapshot_trajectory  # noqa: E402


def load(path):
    """node times [s], rotations (n, 3, 3) and positions (n, 3) of a result/replay folder or an odometry snapshot"""
    p = Path(path)
    if p.suffix == ".bin":
        rows, poses = snapshot_trajectory.load_snapshot(p, with_poses=True)
        m = np.asarray(poses, float).reshape(-1, 4, 4).transpose(0, 2, 1)  # stored column-major
        return np.array([float(r[0]) for r in rows]) * 1e-9, m[:, :3, :3], m[:, :3, 3]
    t, r, pos = trajectory(str(p))
    return np.asarray(t), np.asarray(r), np.asarray(pos)


def carry(points, times, source, target):
    """points (n, 3) at times (n,) placed with `source` (t, R, p), returned placed with `target`"""
    T0, R0, P0 = source
    _, R1, P1 = target
    i = np.clip(np.searchsorted(T0, times, "left") - 1, 0, len(T0) - 1)
    local = np.einsum("nji,nj->ni", R0[i], points - P0[i])
    return np.einsum("nij,nj->ni", R1[i], local) + P1[i]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("result", help="result folder with scan_lio_merged.laz and its trajectory")
    ap.add_argument("trajectory", help="result or replay folder, or odometry_snapshot.bin, of the same recording")
    ap.add_argument("out", help="the cloud to write (.laz); must not exist")
    a = ap.parse_args(argv)
    src, out = Path(a.result), Path(a.out)
    if out.exists():
        print(f"{out} exists: choose a new name", file=sys.stderr)
        return 2
    source, target = load(src), load(a.trajectory)
    if len(source[0]) != len(target[0]) or np.max(np.abs(source[0] - target[0])) > 1e-6:
        print("the two trajectories have different node times (another recording or another cut)", file=sys.stderr)
        return 2
    moved = 0.0
    with laspy.open(src / "scan_lio_merged.laz") as rd:
        with laspy.open(out, mode="w", header=rd.header) as wr:
            for block in rd.chunk_iterator(2_000_000):
                x = np.column_stack([np.asarray(block.x), np.asarray(block.y), np.asarray(block.z)])
                y = carry(x, np.asarray(block.gps_time) * 1e-9, source, target)
                if len(x):
                    moved = max(moved, float(np.max(np.linalg.norm(y - x, axis=1))))
                block.x, block.y, block.z = y[:, 0], y[:, 1], y[:, 2]
                wr.write_points(block)
    print(f"wrote {out}; largest point move {moved:.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
