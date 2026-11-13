"""Compare two isolated-window optimizer traces, without running the mapper.

Reports estimated state changes and matched support, not accuracy or a complete
objective. Both traces must contain exactly the requested iteration sequence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

HEADER = re.compile(r'LIVOX2_TRACE window=(\d+) iteration=(-?\d+) kind=(\w+) (.*)')
PAIRS = re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')
KINDS = {'poses_before', 'poses_after', 'gravity', 'velocity_end_after',
         'offset_after', 'decision', 'matches', 'projection', 'factor_step',
         'range_support', 'experiment_weights'}


def read_trace(path, window, count):
    digest = hashlib.sha256()
    groups = {}
    with path.open('rb') as stream:
        for raw in stream:
            digest.update(raw)
            m = HEADER.fullmatch(raw.decode('utf-8').rstrip('\r\n'))
            if not m:
                raise ValueError(f'Malformed trace line in {path}')
            w, i = int(m[1]), int(m[2])
            if w != window or i < 0:
                continue
            if m[3] not in KINDS:
                continue
            fields = dict(PAIRS.findall(m[4]))
            if 'values' in fields:
                value = np.fromstring(fields['values'][1:-1], sep=',').reshape(
                    int(fields['rows']), int(fields['cols']))
                if not np.isfinite(value).all():
                    raise ValueError('Nonfinite matrix')
            else:
                value = fields
            groups.setdefault(i, {}).setdefault(m[3], []).append(value)
    if sorted(groups) != list(range(count)):
        raise ValueError(f'Expected {count} consecutive iterations in {path}')
    for d in groups.values():
        for k in KINDS - {'matches', 'projection', 'factor_step', 'range_support'}:
            if len(d.get(k, [])) != 1:
                raise ValueError(f'Missing or repeated {k}')
        stages = [v['stage'] for v in d.get('matches', [])]
        if sorted(stages) != ['indoor_after', 'indoor_before', 'outdoor_after', 'outdoor_before']:
            raise ValueError('Incomplete matched-support coverage')
    return groups, digest.hexdigest()


def summarize(groups):
    seed = groups[0]['poses_before'][0].reshape(-1, 3, 4)
    g = groups[0]['gravity'][0].ravel()
    g = g / np.linalg.norm(g)
    rows = []
    for i, d in groups.items():
        poses = d['poses_after'][0].reshape(-1, 3, 4)
        cos_tilt = (seed[-1, :, :3].T @ g) @ (poses[-1, :, :3].T @ g)
        matches = {v['stage']: v for v in d['matches']}
        support = {k: {'source': int(v['source_count']), 'matched': int(v['matched']),
                       'unmatched': int(v['unmatched']),
                       'partial_cost': float(v['unprojected_matched_only_cost'])}
                   for k, v in matches.items()}
        rows.append({'iteration': i,
                     'endpoint_tilt_change_deg': math.degrees(math.acos(np.clip(cos_tilt, -1, 1))),
                     'endpoint_speed_mps': float(np.linalg.norm(d['velocity_end_after'][0])),
                     'endpoint_translation_from_seed_m': (poses[-1, :, 3] - seed[-1, :, 3]).tolist(),
                     'acceleration_offset_mps2': d['offset_after'][0].ravel().tolist(),
                     'decision': d['decision'][0],
                     'projection_min_scale': min(float(v['scale']) for v in d['projection']),
                     'support': support,
                     'local_joint_step_quadratic': {
                         v['name']: float(v['step_H_step']) - 2 * float(v['rhs_dot_step'])
                         for v in d['factor_step'] if 'name' in v}})
    return {'iterations': rows,
            'accepted_iterations': sum(r['decision']['applied'] == '1' for r in rows),
            'final': rows[-1]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('control', type=Path)
    p.add_argument('intervention', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--window', type=int, required=True)
    p.add_argument('--iterations', type=int, default=100)
    args = p.parse_args()
    control, c_hash = read_trace(args.control, args.window, args.iterations)
    intervention, a_hash = read_trace(args.intervention, args.window, args.iterations)
    for kind in ('poses_before', 'gravity'):
        if not np.array_equal(control[0][kind][0], intervention[0][kind][0]):
            raise ValueError(f'Different initial {kind}')
    report = {'window': args.window,
              'inputs': [{'path': str(args.control.resolve()), 'sha256': c_hash},
                         {'path': str(args.intervention.resolve()), 'sha256': a_hash}],
              'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'control': summarize(control), 'intervention': summarize(intervention),
              'limitations': 'Estimated tilt relative to incoming gravity, not physical attitude error. '
              'Matched-only NDT costs omit unmatched support and other factors; both grids can use '
              'the same return. No full-route, cloud-quality or robustness verdict. Exact control '
              'reproduction and intervention isolation require the separate driver validation.'}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: report[k]['final'] for k in ('control', 'intervention')}, indent=2))


if __name__ == '__main__':
    main()
