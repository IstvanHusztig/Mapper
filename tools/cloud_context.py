"""Inspect time-local scene context without rerunning SLAM.

Raw XYZ range statistics do not depend on estimated poses. Exported chunk clouds
are placed using session_poses.mrp and remain dependent on the final trajectory.
All times are seconds from the first exported trajectory node. Output files are
new diagnostic artifacts; this tool never edits input clouds or invokes dMapper.
Requires numpy, laspy/lazrs, matplotlib (already used by map analysis tools).
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import laspy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from trajectory_seams import chunk_poses


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('result', type=Path)
    ap.add_argument('raw_laz', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--interval', nargs=2, type=float, action='append', required=True)
    ap.add_argument('--panel', nargs=2, type=float, action='append',
                    help='Optional scene-panel intervals; defaults to three per extraction interval.')
    ap.add_argument('--mark-time', type=float, action='append', default=[],
                    help='Optional event markers, in seconds from first trajectory node.')
    ap.add_argument('--raw-time-scale', type=float, required=True,
                    help='Raw GPS time to seconds; verify recorder format first.')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise SystemExit('Output must be an empty diagnostic directory.')
    chunks = chunk_poses(args.result)
    trajectory = []
    metadata = []
    for name, matrix in chunks:
        a = np.loadtxt(args.result / name, skiprows=1, usecols=(0, 4, 8, 12), ndmin=2)
        m = np.asarray(matrix)
        trajectory.append(np.column_stack((a[:, 0]*1e-9, a[:, 1:] @ m[:3, :3].T + m[:3, 3])))
        metadata.append((name, m, a[0, 0]*1e-9, a[-1, 0]*1e-9))
    trajectory = np.concatenate(trajectory)
    trajectory = trajectory[np.argsort(trajectory[:, 0])]
    origin = float(trajectory[0, 0])
    trajectory[:, 0] -= origin
    assert np.all(np.diff(trajectory[:, 0]) > 0)
    periods = [tuple(v) for v in args.interval]
    assert all(0 <= lo < hi for lo, hi in periods)
    end = int(np.ceil(trajectory[-1, 0])) + 1
    counts = np.zeros((end, 6), dtype=np.int64)
    bins = np.arange(0, 200.25, .25)
    hist = np.zeros((end, len(bins)), dtype=np.int64)
    raw_views = []
    total = 0
    last_time = -np.inf
    reversals = 0
    with laspy.open(args.raw_laz) as reader:
        raw_count = reader.header.point_count
        for a in reader.chunk_iterator(1_000_000):
            t = np.asarray(a.gps_time) * args.raw_time_scale - origin
            reversals += int(np.sum(np.diff(t) < 0)) + int(t[0] < last_time)
            last_time = t[-1]
            p = np.column_stack((a.x, a.y, a.z))
            r = np.linalg.norm(p, axis=1)
            valid = np.isfinite(r) & np.isfinite(t) & (r > .05) & (t >= 0) & (t < end)
            sec = np.floor(t[valid]).astype(int)
            rv = r[valid]
            tests = [np.ones(len(rv), bool), rv < 3, rv < 5, rv < 10, rv > 15, rv > 30]
            for j, test in enumerate(tests):
                counts[:, j] += np.bincount(sec[test], minlength=end)
            rb = np.minimum((rv/.25).astype(int), len(bins)-1)
            hist += np.bincount(sec*len(bins)+rb, minlength=end*len(bins)).reshape(hist.shape)
            keep = np.zeros(len(t), bool)
            for lo, hi in periods:
                keep |= (t >= lo) & (t < hi)
            ids = np.flatnonzero(keep)[::20]
            raw_views.append(np.column_stack((t[ids], p[ids])))
            total += len(a)
            if total % 10_000_000 == 0:
                print(f'Raw returns inspected: {total}/{raw_count}', flush=True)
    assert total == raw_count
    raw_sample = np.concatenate(raw_views)
    np.save(args.output/'raw_sensor_frame_sample.npy', raw_sample)
    rows = []
    for second in range(end):
        n = int(counts[second, 0])
        if not n:
            continue
        cumulative = np.cumsum(hist[second])
        q = [float(bins[np.searchsorted(cumulative, n*f)]) for f in (.5, .9)]
        rows.append(dict(second=second, valid_returns=n, range_p50_lower_bin_m=q[0],
                         range_p90_lower_bin_m=q[1],
                         **{key: float(counts[second, k]/n) for k, key in enumerate(
                             ['unused', 'under3m_share', 'under5m_share', 'under10m_share',
                              'over15m_share', 'over30m_share']) if k}))
    with (args.output/'raw_ranges_by_second.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

    exported = []
    world_samples = []
    for lo, hi in periods:
        header = laspy.LasHeader(point_format=1, version='1.2')
        header.scales = [.0001]*3
        path = args.output/f'final_context_{lo:g}_{hi:g}s.laz'
        n = 0
        with laspy.open(path, mode='w', header=header) as writer:
            for name, m, first, last in metadata:
                if last-origin < lo or first-origin > hi:
                    continue
                laz = args.result / name.replace('trajectory_lio_', 'scan_lio_').replace('.csv', '.laz')
                with laspy.open(laz) as reader:
                    for a in reader.chunk_iterator(1_000_000):
                        t = np.asarray(a.gps_time)*1e-9-origin
                        keep = (t >= lo) & (t < hi)
                        if not keep.any():
                            continue
                        p = np.column_stack((a.x[keep], a.y[keep], a.z[keep])) @ m[:3, :3].T + m[:3, 3]
                        out = laspy.ScaleAwarePointRecord.zeros(len(p), header=header)
                        out.x, out.y, out.z = p.T
                        out.gps_time = np.asarray(a.gps_time)[keep]
                        out.intensity = np.asarray(a.intensity)[keep]
                        writer.write_points(out)
                        world_samples.append(np.column_stack((t[keep][::20], p[::20])))
                        n += len(p)
        exported.append(dict(path=str(path), points=n, from_s=lo, to_s=hi))
        print(f'Exported existing cloud context {lo:g}–{hi:g}s: {n} points', flush=True)
    world = np.concatenate(world_samples)
    np.save(args.output/'final_world_sample.npy', world)

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), layout='constrained', sharex=True)
    times = [r['second'] for r in rows]
    for key, label in [('range_p50_lower_bin_m','Median range'),('range_p90_lower_bin_m','90th percentile range')]:
        axes[0].plot(times,[r[key] for r in rows],label=label)
    for key, label in [('under5m_share','Returns within 5m'),('over15m_share','Returns beyond 15m')]:
        axes[1].plot(times,[100*r[key] for r in rows],label=label)
    axes[2].plot(times,[r['valid_returns'] for r in rows])
    for ax in axes:
        for mark in args.mark_time:
            ax.axvline(mark, color='red', ls='--', label=f'Event at {mark:g}s')
        ax.set_xlim(min(x[0] for x in periods),max(x[1] for x in periods)); ax.grid(alpha=.25)
    axes[0].set_ylabel('Raw XYZ range [m]'); axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Share of recorded returns [%]'); axes[1].legend(fontsize=8)
    axes[2].set_ylabel('Recorded valid returns / s'); axes[2].set_xlabel('Seconds from first trajectory node')
    fig.suptitle('Raw scene context: independent of estimated translation\nRange quantiles use 0.25m bins; return counts are not emitted-pulse coverage')
    fig.savefig(args.output/'raw-range-context.png',dpi=150); plt.close(fig)

    sections = args.panel or [(float(a),float(b)) for lo,hi in periods
                             for a,b in zip(np.linspace(lo,hi,4)[:-1],np.linspace(lo,hi,4)[1:])]
    sections = [(lo,hi) for lo,hi in sections if any(a<=lo and hi<=b for a,b in periods)]
    if not sections:
        raise ValueError('At least one panel must lie inside an extraction interval.')
    fig, axes = plt.subplots(2,len(sections),figsize=(4*len(sections),9),layout='constrained',squeeze=False)
    for col,(lo,hi) in enumerate(sections):
        a = world[(world[:,0]>=lo)&(world[:,0]<hi)]
        center=np.array([np.interp((lo+hi)/2,trajectory[:,0],trajectory[:,k]) for k in (1,2,3)])
        p=a[:,1:]-center
        tt=trajectory[(trajectory[:,0]>=lo)&(trajectory[:,0]<hi),1:]-center
        for row,(i,j,label) in enumerate([(0,1,'top XY'),(0,2,'side XZ')]):
            ax=axes[row,col]; ax.scatter(p[:,i],p[:,j],c=a[:,0],vmin=lo,vmax=hi,s=.2,cmap='turbo',rasterized=True)
            ax.plot(tt[:,i],tt[:,j],'k-',lw=1); ax.plot(0,0,'kx',ms=6)
            ax.set(xlim=(-20,20),ylim=(-20,20),title=f'{lo}–{hi}s: {label}',xlabel='Offset [m]',ylabel='Offset [m]'); ax.set_aspect('equal'); ax.grid(alpha=.2)
    fig.suptitle('Time-local exported cloud: sensor-centred views, same scale\nBlack path = estimated sensor motion. Geometry inherits final trajectory errors; colours progress through each interval.')
    fig.savefig(args.output/'local-cloud-panels.png',dpi=140); plt.close(fig)
    summary=dict(time_origin_sensor_seconds=origin,raw_time_scale=args.raw_time_scale,raw_returns_inspected=total,
                 raw_timestamp_reversals=reversals,raw_quantile_bin_width_m=.25,exports=exported,
                 limitations=['Raw ranges use XYZ Euclidean norm, not raw_range extra field; no factory recalibration.',
                              'Recorded-return count does not measure no-return pulses or matching support.',
                              'Exported cloud geometry already contains trajectory errors.',
                              'Deterministic visual sampling: every twentieth selected return per input block.',
                              'World views are centred using linearly interpolated final trajectory; no cloud pose interpolation.'])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    (args.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    def sha(path):
        h=hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(8*1024*1024),b''):
                h.update(block)
        return h.hexdigest()
    source_files=[args.raw_laz,args.result/'session_poses.mrp',Path(__file__),
                  Path(__file__).with_name('trajectory_seams.py')]
    for name, _, first, last in metadata:
        source_files.append(args.result/name)
        if any(last-origin>=lo and first-origin<=hi for lo,hi in periods):
            source_files.append(args.result/name.replace('trajectory_lio_','scan_lio_').replace('.csv','.laz'))
    manifest={'inputs':{str(p):sha(p) for p in source_files},
              'outputs':{p.name:sha(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
