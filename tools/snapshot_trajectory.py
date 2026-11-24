"""Read world positions from dMapper little-endian snapshot v1, without changing it.

Source contract: src/infrastructure/odometry_snapshot_file.cpp. This reader does
not certify the stored settings hash against a TOML; run provenance does that.
Snapshot seconds are converted to decimal nanoseconds identically within this
stage. Never match them approximately to the rounded final CSV timestamps.
"""
from pathlib import Path
from decimal import Decimal
import struct
import math


def load_snapshot(path, with_metadata=False, with_poses=False):
    from trajectory_seams import validate
    data = Path(path).read_bytes()
    offset = 0

    def take(fmt):
        nonlocal offset
        size = struct.calcsize("<" + fmt)
        if offset + size > len(data):
            raise ValueError("truncated odometry snapshot")
        values = struct.unpack_from("<" + fmt, data, offset)
        offset += size
        return values

    def array(width):
        count, = take("Q")
        if count > 100_000_000 or count > (len(data) - offset) // (8 * width):
            raise ValueError("invalid snapshot list length")
        return [take(str(width) + "d") for _ in range(count)]

    magic, version, settings_hash, windows = take("8sIQQ")
    if magic != b"DMSNAP01" or version not in (1, 2):
        raise ValueError("unsupported odometry snapshot format")
    if windows > 1_000_000 or windows > len(data) // 256:
        raise ValueError("invalid snapshot window count")
    rows = []
    poses_out = []
    starts = []
    for _ in range(windows):
        poses, model, stamps, angles, accel = array(16), array(16), array(2), array(3), array(3)
        state = take("27d")
        if not poses or not (len(poses) == len(model) == len(stamps) == len(angles)) or len(accel) not in (0, len(poses)):
            raise ValueError("inconsistent snapshot window arrays")
        starts.append({"time_s": stamps[0][0], "position": poses[0][12:15]})
        for values in (*poses, *model, *stamps, *angles, *accel, state):
            if not all(math.isfinite(v) for v in values):
                raise ValueError("non-finite snapshot data")
        for p, t in zip(poses, stamps):
            if any(abs(p[i] - v) > 1e-12 for i, v in ((3, 0), (7, 0), (11, 0), (15, 1))):
                raise ValueError("invalid snapshot affine matrix")
            poses_out.append(p)
            rows.append((Decimal(str(t[0])) * Decimal(1_000_000_000), p[12], p[13], p[14]))
    tail = take("18d")
    if version >= 2:  # the windows' readings (odometry_snapshot_file.h): a count, then per window a flag and 65 bytes if set
        if take("Q")[0] != windows:
            raise ValueError("invalid snapshot readings count")
        for _ in range(windows):
            if take("B")[0]:
                offset += 65
    if offset != len(data) or not all(math.isfinite(v) for v in tail):
        raise ValueError("invalid snapshot tail or trailing bytes")
    validate(rows)
    if with_poses:  # the 16 numbers of each node's pose, column-major as stored (rotation: p[0:3], p[4:7], p[8:11] are its columns)
        return rows, poses_out
    return (rows, {"settings_hash": settings_hash, "windows": starts}) if with_metadata else rows
