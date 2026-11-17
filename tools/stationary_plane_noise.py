"""Characterize raw stationary planar returns without a mapper replay.

Automatic geometric planes are not surveyed walls. Fit on an early interval and
evaluate later returns in the SAME sensor frame; never align intervals to hide
motion. The fixed 50 mm selection band truncates tails; report that limitation.
"""
import argparse
import hashlib
import json
from pathlib import Path

import laspy
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def characterize(source, output):
    rng = np.random.default_rng(20261113)
    points, times = [], []
    # Recording verification already established monotonic LiDAR timestamps.
    with laspy.open(source) as reader:
        for block in reader.chunk_iterator(500000):
            t = np.asarray(block.gps_time)
            mask = (t >= 1) & (t < 15)
            points.append(np.column_stack((block.x, block.y, block.z))[mask][::10])
            times.append(t[mask][::10])
            if t[-1] >= 15:
                break
    p, t = np.concatenate(points), np.concatenate(times)
    valid = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) >= .5)
    p, t = p[valid], t[valid]
    available = t < 7
    planes = []
    for index in range(5):
        pool = p[available]
        if len(pool) < 1000:
            break
        probe = pool[rng.choice(len(pool), min(12000, len(pool)), replace=False)]
        best, score = None, -1
        for _ in range(160):
            a, b, c = probe[rng.choice(len(probe), 3, replace=False)]
            n = np.cross(b-a, c-a)
            if np.linalg.norm(n) < 1e-8:
                continue
            n /= np.linalg.norm(n)
            count = np.count_nonzero(np.abs((probe-a) @ n) < .03)
            if count > score:
                best, score = (n, a), count
        n, center = best
        for _ in range(3):
            fit = pool[np.abs((pool-center) @ n) < .03]
            center = fit.mean(axis=0)
            _, vectors = np.linalg.eigh(np.cov(fit.T))
            n = vectors[:, 0]
        # Freeze spatial footprint from fit support, with broad 50 mm normal band.
        axis = vectors[:, 1:]
        uv = (fit-center) @ axis
        lo, hi = np.quantile(uv, [.02, .98], axis=0)
        proj = (p-center) @ axis
        residual = (p-center) @ n
        selected = ((proj >= lo) & (proj <= hi)).all(axis=1) & (np.abs(residual) < .05)
        rows = []
        for start, end in ((1, 7), (7, 11), (11, 15)):
            m = selected & (t >= start) & (t < end)
            r, pp = residual[m], p[m]
            if len(r) < 100:
                continue
            rr = np.linalg.norm(pp, axis=1)
            incidence = np.degrees(np.arccos(np.clip(np.abs(pp @ n) / rr, 0, 1)))
            rows.append(dict(time_s=[start,end], points=int(m.sum()),
                median_m=float(np.median(r)), mad_sigma_m=float(1.4826*np.median(np.abs(r-np.median(r)))),
                abs_p90_m=float(np.quantile(np.abs(r), .9)),
                range_p10_p50_p90_m=np.quantile(rr,[.1,.5,.9]).tolist(),
                incidence_p10_p50_p90_deg=np.quantile(incidence,[.1,.5,.9]).tolist()))
        planes.append(dict(id=index, normal=n.tolist(), center_m=center.tolist(),
            footprint_min_m=lo.tolist(), footprint_max_m=hi.tolist(), intervals=rows))
        available &= np.abs(residual) >= .06
    report = dict(source=str(source), source_sha256=sha(source),
        script_sha256=sha(__file__), seed=20261113, sampling_stride=10,
        analyzed_points=len(p), interval_s=[1,15], normal_selection_halfwidth_m=.05,
        planes=planes, limitations=['Geometric patches not independently identified physical surfaces.',
            'Selection truncates large residual tails and may mix fixtures.',
            'No pose alignment, deskew or map fitting; scanner rested per acquisition notes.',
            'Near-range indoor repeatability is not total outdoor residual noise calibration.'])
    Path(output).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps([{'id':x['id'], 'intervals':x['intervals']} for x in planes]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    characterize(args.source, args.output)
