"""Inspect validated per-return NDT evidence; never estimates a new trajectory."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_csv(path):
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        names = [n for n in reader.fieldnames if n != 'grid']
        values = [[float(row[n]) for n in names] for row in reader]
    array = np.asarray(values)
    return {n: array[:, i] for i, n in enumerate(names)}


def analyse(folder, states, output):
    validation = json.loads((folder / 'validation.json').read_text())
    if validation['status'] != 'validated_and_exported':
        raise ValueError('Reconstruction did not pass identity gates')
    saved = json.loads(states.read_text())
    report = {'interpretation': 'RHS work along the recorded angular/full step, not per-cell endpoint-yaw response',
              'validation': str(folder / 'validation.json'), 'iterations': []}
    for state in saved['iterations']:
        it = state['iteration']
        steps = np.asarray(state['solver_update_before_bias_clamp'])[:6 * len(state['poses_before'])].reshape(-1, 6)
        item = {'iteration': it, 'grids': {}}
        for grid in ('indoor', 'outdoor'):
            d = read_csv(folder / f'iteration-{it}-{grid}.csv')
            yaw_push = d['b5'] * steps[d['pose_index'].astype(int), 5]
            groups = []
            selected = d['range_bin'] == 1
            for key in np.unique(d['cell_key'][selected]):
                m = selected & (d['cell_key'] == key)
                first = np.flatnonzero(m)[0]
                cov = np.array([[d[f'cov{r}{c}'][first] for c in range(3)] for r in range(3)])
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                normal = np.array([d[f'normal_{axis}'][first] for axis in 'xyz'])
                groups.append({'cell_key': int(key), 'matches_2_5m': int(m.sum()),
                               'occupancy': int(d['occupancy'][first]),
                               'yaw_push': float(yaw_push[m].sum()),
                               'angular_push': float(d['angular_push'][m].sum()),
                               'full_step_push': float(d['rhs_dot_step'][m].sum()),
                               'target': [float(d[f'target_{a}'][first]) for a in 'xyz'],
                               'normal': normal.tolist(), 'cov_eigenvalues': eigenvalues.tolist(),
                               'cov_min_axis': eigenvectors[:, 0].tolist(),
                               'normal_vs_cov_min_axis_deg': (float(np.rad2deg(np.arccos(np.clip(abs(normal @ eigenvectors[:, 0]), 0, 1))))
                                                              if np.linalg.norm(normal) > .5 else None),
                               'residual_mean': [float(d[f'residual_{a}'][m].mean()) for a in 'xyz'],
                               'residual_norm_mean': float(d['residual_norm'][m].mean()),
                               'huber_weight_mean': float(d['huber_weight'][m].mean()),
                               'raw_cost': float(d['weighted_cost'][m].sum()),
                               'robust_cost': float(d['robust_cost'][m].sum()),
                               'time_min_max': [float(d['time_s'][m].min()), float(d['time_s'][m].max())]})
            groups.sort(key=lambda r: r['yaw_push'], reverse=True)
            bins = []
            for b in range(4):
                m = d['range_bin'] == b
                bins.append({'bin': b, 'matches': int(m.sum()),
                             'yaw_push': float(yaw_push[m].sum()),
                             'angular_push': float(d['angular_push'][m].sum()),
                             'full_step_push': float(d['rhs_dot_step'][m].sum()),
                             'robust_cost': float(d['robust_cost'][m].sum())})
            item['grids'][grid] = {'bins': bins, 'cells_2_5m': groups}
        report['iterations'].append(item)
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    for grid, data in report['iterations'][0]['grids'].items():
        print(grid, json.dumps(data['bins']))
        for cell in data['cells_2_5m'][:5]:
            print(json.dumps(cell))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    p.add_argument('states', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    analyse(a.folder, a.states, a.output)
