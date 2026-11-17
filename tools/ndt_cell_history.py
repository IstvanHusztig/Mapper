"""Recover selected cell insertions from saved poses/caches; no optimizer or map update."""
import argparse
import json
from pathlib import Path
import struct

import laspy
import numpy as np

from run_livox4_gain_comparison import window_bytes, sha

DTYPE = np.dtype([('xyz', '<f8', (3,)), ('t', '<f8'), ('intensity', '<f4'),
                  ('node', '<i4'), ('id', 'u1'), ('index', '<i4')])


def keys(points, resolution):
    c = np.floor(points / resolution).astype(np.int64) & 65535
    return (c[:, 0] << 32) | (c[:, 1] << 16) | c[:, 2]


def analyse(snapshot, cache, raw, ranking, output):
    ranked = json.loads(ranking.read_text())['iterations'][0]['grids']
    targets = {}
    for grid in ('indoor', 'outdoor'):
        targets[grid] = {r['cell_key']: dict(r, history=[]) for r in ranked[grid]['cells_2_5m'][:10]}
    identities = {str(p): sha(p) for p in (snapshot, raw, ranking)}

    def collect(w, points, times, view):
        for grid, resolution in (('indoor', .15), ('outdoor', .4)):
            ids = keys(points, resolution)
            hit = np.isin(ids, list(targets[grid]))
            for k in np.flatnonzero(hit):
                targets[grid][int(ids[k])]['history'].append([w, int(k), float(times[k]), *points[k].tolist(), *view.tolist()])

    # Match the production seed selection; any ambiguity is caught by target first-point/count checks.
    with laspy.open(raw) as f:
        blocks = []
        count = 0
        for chunk in f.chunk_iterator(500000):
            xyz = np.column_stack([chunk.x, chunk.y, chunk.z])
            t = np.asarray(chunk.gps_time)
            xy = np.linalg.norm(xyz[:, :2], axis=1)
            keep = (t != 0) & (xy > .35) & (xy < 40.)
            blocks.append(np.column_stack([t[keep], xyz[keep]]))
            count += int(keep.sum())
            if count > 60001:
                break
    seed = np.concatenate(blocks)
    seed = seed[np.argsort(seed[:, 0], kind='stable')][:60001]
    tail = np.frombuffer(snapshot.read_bytes()[-144:], '<f8')
    mg = tail[:16].reshape(4, 4).T
    collect(0, seed[:, 1:] @ mg[:3, :3].T + mg[:3, 3], seed[:, 0], mg[:3, 3])
    for w, payload in enumerate(window_bytes(snapshot)[:297], 1):
        n, = struct.unpack_from('<Q', payload)
        poses = np.frombuffer(payload, '<f8', n*16, 8).reshape(n, 4, 4).transpose(0, 2, 1)
        path = cache / f'intermediate_points_{w-1}.cache'
        b = path.read_bytes()
        import hashlib
        identities[str(path)] = hashlib.sha256(b).hexdigest()
        points = np.frombuffer(b, DTYPE)
        pose = poses[points['node']]
        world = np.einsum('nij,nj->ni', pose[:, :3, :3], points['xyz']) + pose[:, :3, 3]
        collect(w, world, points['t'], poses[0, :3, 3])

    errors = []
    summary = {}
    for grid, cells in targets.items():
        summary[grid] = []
        for key, r in cells.items():
            a = np.asarray(r['history'])
            target, normal = np.array(r['target']), np.array(r['normal'])
            if len(a) != r['occupancy'] or not np.allclose(a[0, 3:6], target, rtol=0, atol=1e-12):
                errors.append(f'{grid}/{key}: history identity mismatch')
                continue
            accepted = (a[:, 6:9] - target) @ normal >= 0
            accepted[:3] = True  # first three insertions precede viewpoint filtering
            xyz = a[accepted, 3:6]
            centred = xyz - xyz.mean(axis=0)
            cov = centred.T @ centred / len(xyz)
            eig, vectors = np.linalg.eigh(cov)
            r['history_summary'] = {
                'count': len(a), 'accepted_covariance_updates_or_seed': int(accepted.sum()),
                'rejected_viewpoint': int((~accepted).sum()),
                'first_window': int(a[0, 0]), 'last_window': int(a[-1, 0]),
                'first_last_time_s': [float(a[0, 2]), float(a[-1, 2])],
                'windows': {str(int(w)): int((a[:, 0] == w).sum()) for w in np.unique(a[:, 0])},
                'centroid': xyz.mean(axis=0).tolist(),
                'first_point_to_centroid_m': float(np.linalg.norm(target - xyz.mean(axis=0))),
                'centred_cov_eigenvalues': eig.tolist(),
                'centred_min_axis': vectors[:, 0].tolist() if eig[1] > 1e-10 * max(eig[2], 1e-20) else None,
                'max_point_distance_m': float(np.linalg.norm(xyz-target, axis=1).max())}
            summary[grid].append({k: v for k, v in r.items() if k != 'history'})
    report = {'scope': 'Saved-pose insertion membership; first-point and occupancy checked; no changed trajectory.',
              'seed_limit': 'NumPy timestamp sort; selected-cell first-point and occupancy gates detect relevant mismatches.',
              'history_columns': ['window', 'source_index', 'time_s', 'x', 'y', 'z', 'view_x', 'view_y', 'view_z'],
              'errors': errors, 'input_hashes': identities, 'cells': targets, 'summary': summary}
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    for grid, rows in summary.items():
        for r in rows[:3]:
            print(grid, r['cell_key'], json.dumps(r['history_summary']))
    print('errors', errors)
    if errors:
        raise SystemExit(2)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot', 'cache', 'raw', 'ranking', 'output'):
        p.add_argument(name, type=Path)
    analyse(**vars(p.parse_args()))
