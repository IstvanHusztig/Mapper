"""Reduce a Livox4 diagnostic log to iteration-level angular factor evidence; no SLAM."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
from run_livox4_gain_comparison import save, sha

HEADER = re.compile(r'LIVOX4_TRACE window=(\d+) iteration=(-?\d+) kind=(\w+)(?: (.*))?')
PAIRS = re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')


def groups(path):
    key = None
    records = {}
    with path.open(encoding='utf-8', errors='strict') as f:
        for line in f:
            m = HEADER.search(line)
            if not m:
                continue
            new = (int(m[1]), int(m[2]))
            if new != key:
                if key is not None:
                    yield key, records
                key, records = new, {}
            fields = dict(PAIRS.findall(m[4] or ''))
            if 'values' in fields:
                a = np.fromstring(fields['values'][1:-1], sep=',')
                a = a.reshape(int(fields['rows']), int(fields['cols']))
                if not np.isfinite(a).all():
                    raise ValueError('Nonfinite trace matrix')
                value = a
            else:
                value = fields
            records.setdefault(m[3], []).append(value)
    if key is not None:
        yield key, records


def yaw(poses):
    a = poses.reshape(-1, 3, 4)
    return np.arctan2(-a[:, 0, 1], a[:, 0, 0])


def angle_difference(a, b):
    return np.rad2deg(np.arctan2(np.sin(a-b), np.cos(a-b)))


def analyse(log, output):
    rows = []
    initial = {}
    errors = []
    for (w, it), r in groups(log):
        if it < 0:
            continue
        def one(name):
            values = r[name]
            if len(values) != 1:
                raise ValueError(f'Duplicate {w}/{it}/{name}')
            return values[0]
        poses_before, poses_after = one('poses_before'), one('poses_after')
        if it == 0:
            initial[w] = yaw(poses_before)
        n = len(poses_before)
        step = one('solver_update_before_bias_clamp').ravel()
        responses = {k.removeprefix('shared_H_response_'): one(k).ravel()
                     for k in r if k.startswith('shared_H_response_')}
        factors = [x['name'] for x in r['factor']]
        ndt_sum = sum(responses[f'ndt_range_{b}'] for b in range(4))
        motion_sum = responses['motion_translation_rows'] + responses['motion_gyro_rows']
        reconstruction = float(np.linalg.norm(sum(responses[f] for f in factors) - step))
        range_error = float(np.linalg.norm(ndt_sum - responses['ndt_projected']))
        motion_error = float(np.linalg.norm(motion_sum - responses['motion_edges_including_bias_coupling']))
        decision = one('decision')
        checks = one('response_check')
        if max(reconstruction, range_error, motion_error) > 1e-7 * max(1., np.linalg.norm(step)):
            errors.append(f'{w}/{it}: decomposition mismatch')
        if decision['applied'] != '1' or checks.get('dropped_writes') != '0':
            errors.append(f'{w}/{it}: rejected step or dropped matrix write')
        match_checks = [x for x in r['matches'] if x['stage'].endswith('before')]
        if any(x.get('assembly_identity_mismatches') != '0' for x in match_checks):
            errors.append(f'{w}/{it}: diagnostic/assembly match mismatch')
        nodes = [0, 30, 60, 90, 120, 150, 164, n-1]
        pre = angle_difference(yaw(poses_before), initial[w])
        post = angle_difference(yaw(poses_after), initial[w])
        factor_steps = {x['name']: {k: float(v) for k, v in x.items() if k != 'name'}
                        for x in r['factor_step']}
        row = {'window': w, 'iteration': it, 'nodes': nodes,
               'yaw_before_deg': pre[nodes].tolist(), 'yaw_after_deg': post[nodes].tolist(),
               'actual_yaw_step_deg': np.rad2deg(step[:n*6].reshape(n, 6)[nodes, 5]).tolist(),
               'factor_yaw_response_deg': {k: np.rad2deg(v[:n*6].reshape(n, 6)[nodes, 5]).tolist()
                                           for k, v in responses.items()},
               'factor_steps': factor_steps, 'factor_angular': r['factor_angular'],
               'frozen_ndt': r['frozen_ndt'], 'prior_costs_before': one('prior_costs'),
               'turnover': r['match_turnover'], 'range_support': r['range_support'],
               'huber_support': r.get('huber_support', []), 'range_step': r['range_step'],
               'projection': one('projection_matrix').tolist() if 'projection_matrix' in r else None,
               'decomposition_errors': {'total': reconstruction, 'range': range_error, 'motion': motion_error},
               'response_check': checks, 'decision': decision}
        rows.append(row)
    expected = {(w, i) for w in range(297, 301) for i in range(100)}
    actual = [(r['window'], r['iteration']) for r in rows]
    if set(actual) != expected or len(actual) != len(expected):
        errors.append('Expected exactly 400 unique iterations W297-300, 0-99')
    report = {'source': str(log), 'source_sha256': sha(log), 'tool_sha256': sha(Path(__file__)),
              'interpretation': 'Shared damped-H RHS responses, not factor removal or ground truth',
              'errors': errors, 'iterations': rows}
    save(output, report)
    for w in range(297, 301):
        group = [r for r in rows if r['window'] == w]
        if not group:
            continue
        first = next((r for r in group if abs(r['yaw_after_deg'][-2]) > 3), None)
        peak = max(group, key=lambda r: abs(r['actual_yaw_step_deg'][-2]))
        print(json.dumps({'window': w, 'first_gt3deg_iteration': first['iteration'] if first else None,
                          'peak_step_iteration': peak['iteration'], 'peak_step_node164_deg': peak['actual_yaw_step_deg'][-2],
                          'final_node164_deg': group[-1]['yaw_after_deg'][-2],
                          'peak_factor_node164_deg': {k: v[-2] for k, v in peak['factor_yaw_response_deg'].items()}}))
    print(json.dumps({'iterations': len(rows), 'errors': errors}))
    if errors:
        raise SystemExit(2)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    analyse(a.log, a.output)
