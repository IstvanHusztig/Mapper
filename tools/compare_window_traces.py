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
         'range_support', 'range_step', 'experiment_weights'}


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
            if m[3] not in KINDS and m[3] != 'huber_support' and not m[3].startswith(
                    ('range_information_sum_', 'huber_weighted_information_')):
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
        for k in KINDS - {'matches', 'projection', 'factor_step', 'range_support', 'range_step'}:
            if len(d.get(k, [])) != 1:
                raise ValueError(f'Missing or repeated {k}')
        stages = [v['stage'] for v in d.get('matches', [])]
        if sorted(stages) != ['indoor_after', 'indoor_before', 'outdoor_after', 'outdoor_before']:
            raise ValueError('Incomplete matched-support coverage')
        for v in d['matches']:
            if 'assembly_identity_mismatches' in v and int(v['assembly_identity_mismatches']) != 0:
                raise ValueError('Assembly association mismatch')
    return groups, digest.hexdigest()


def summarize(groups, samples=None, acceleration_scale=None):
    seed = groups[0]['poses_before'][0].reshape(-1, 3, 4)
    g = groups[0]['gravity'][0].ravel()
    incoming_gravity = g.copy()
    g = g / np.linalg.norm(g)
    rows = []
    for i, d in groups.items():
        ranges = d['range_step']
        if len(ranges) != 4 or {v['bin'] for v in ranges} != {'0', '1', '2', '3'}:
            raise ValueError('Incomplete range-step coverage')
        ndt = [v for v in d['factor_step'] if v.get('name') == 'ndt_projected']
        if len(ndt) != 1:
            raise ValueError('Missing NDT step contribution')
        for metric in ('rhs_dot_step', 'step_H_step'):
            if not math.isclose(sum(float(v[metric]) for v in ranges), float(ndt[0][metric]),
                                rel_tol=1e-9, abs_tol=1e-8):
                raise ValueError(f'Range/NDT {metric} conservation failed')
        poses = d['poses_after'][0].reshape(-1, 3, 4)
        cos_tilt = (seed[-1, :, :3].T @ g) @ (poses[-1, :, :3].T @ g)
        matches = {v['stage']: v for v in d['matches']}
        support = {k: {'source': int(v['source_count']), 'matched': int(v['matched']),
                       'unmatched': int(v['unmatched']),
                       'partial_cost': float(v['unprojected_matched_only_cost'])}
                   for k, v in matches.items()}
        row = {'iteration': i,
                     'endpoint_tilt_change_deg': math.degrees(math.acos(np.clip(cos_tilt, -1, 1))),
                     'endpoint_speed_mps': float(np.linalg.norm(d['velocity_end_after'][0])),
                     'endpoint_translation_from_seed_m': (poses[-1, :, 3] - seed[-1, :, 3]).tolist(),
                     'acceleration_offset_mps2': d['offset_after'][0].ravel().tolist(),
                     'decision': d['decision'][0],
                     'projection_min_scale': min(float(v['scale']) for v in d['projection']),
                     'support': support,
                     'local_joint_step_quadratic': {
                         v['name']: float(v['step_H_step']) - 2 * float(v['rhs_dot_step'])
                         for v in d['factor_step'] if 'name' in v}}
        if samples is not None:
            if samples.shape != (len(poses), 7):
                raise ValueError('Checkpoint sample shape differs from poses')
            dt = np.diff(samples[:, 0])
            if not np.all(dt > 0):
                raise ValueError('Checkpoint timestamps must increase')
            acceleration = (np.einsum('nij,nj->ni', poses[:, :, :3], samples[:, 1:4])
                            - incoming_gravity)*acceleration_scale
            row['rotated_acceleration_sample_mean_mps2'] = acceleration.mean(axis=0).tolist()
            row['rotated_acceleration_left_time_mean_mps2'] = (
                np.sum(acceleration[:-1]*dt[:, None], axis=0)/dt.sum()).tolist()
        if 'huber_support' in d:
            groups = d['huber_support']
            expected = {(stage, str(bin_id)) for stage in matches for bin_id in range(4)}
            if len(groups) != 16 or {(v['stage'], v['bin']) for v in groups} != expected:
                raise ValueError('Incomplete Huber support coverage')
            alpha = float(d['experiment_weights'][0]['alpha'])
            huber = {}
            for stage in matches:
                bins = [v for v in groups if v['stage'] == stage]
                raw = sum(alpha*d[f'range_information_sum_{stage}_{k}'][0] for k in range(4))
                weighted = sum(d[f'huber_weighted_information_{stage}_{k}'][0] for k in range(4))
                total = sum(float(v['sum_weight']) for v in bins)
                matched = sum(int(v['matched']) for v in bins)
                if matched != support[stage]['matched']:
                    raise ValueError('Huber/match counts differ')
                if not math.isclose(sum(float(v['raw_cost']) for v in bins), support[stage]['partial_cost'],
                                    rel_tol=1e-9, abs_tol=1e-8):
                    raise ValueError('Huber raw/match cost conservation failed')
                huber[stage] = {
                    'matched': matched,
                    'downweighted': sum(int(v['downweighted']) for v in bins),
                    'sum_weight': total,
                    'mean_weight': total/matched if matched else None,
                    'robust_cost': sum(float(v['robust_cost']) for v in bins),
                    'raw_information_symmetric_eigenvalues': np.linalg.eigvalsh((raw+raw.T)/2).tolist(),
                    'weighted_information_symmetric_eigenvalues': np.linalg.eigvalsh((weighted+weighted.T)/2).tolist(),
                    'bins': bins}
            row['huber'] = huber
        rows.append(row)
    return {'iterations': rows,
            'validated_range_quadratic_checks': 2*len(rows),
            'accepted_iterations': sum(r['decision']['applied'] == '1' for r in rows),
            'final': rows[-1]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('control', type=Path)
    p.add_argument('intervention', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--window', type=int, required=True)
    p.add_argument('--iterations', type=int, default=100)
    p.add_argument('--huber-cutoff', type=float, help='Require this Huber cutoff in every intervention iteration')
    p.add_argument('--checkpoint', type=Path, help='Extracted optimizer checkpoint JSON for same-sample IMU context')
    args = p.parse_args()
    control, c_hash = read_trace(args.control, args.window, args.iterations)
    intervention, a_hash = read_trace(args.intervention, args.window, args.iterations)
    if args.huber_cutoff is not None:
        if not math.isfinite(args.huber_cutoff) or args.huber_cutoff <= 0:
            raise ValueError('Expected Huber cutoff must be finite and positive')
        for d in intervention.values():
            if len(d.get('huber_support', [])) != 16 or any(
                    float(v['cutoff']) != args.huber_cutoff for v in d['huber_support']):
                raise ValueError('Missing or wrong Huber cutoff in intervention')
    for kind in ('poses_before', 'gravity'):
        if not np.array_equal(control[0][kind][0], intervention[0][kind][0]):
            raise ValueError(f'Different initial {kind}')
    samples = None
    acceleration_scale = None
    checkpoint_hash = None
    if args.checkpoint is not None:
        checkpoint_bytes = args.checkpoint.read_bytes()
        checkpoint_hash = hashlib.sha256(checkpoint_bytes).hexdigest()
        checkpoint = json.loads(checkpoint_bytes)
        if checkpoint['window'] != args.window:
            raise ValueError('Wrong checkpoint window')
        samples = np.array(checkpoint['prior']['node_samples'], dtype=float)
        acceleration_scale = float(checkpoint['prior']['window_input']['to_mps2'])
        if not np.isfinite(samples).all() or not math.isfinite(acceleration_scale) or acceleration_scale <= 0:
            raise ValueError('Invalid checkpoint samples or acceleration scale')
    report = {'window': args.window, 'required_huber_cutoff': args.huber_cutoff,
              'checkpoint_sha256': checkpoint_hash,
              'inputs': [{'path': str(args.control.resolve()), 'sha256': c_hash},
                         {'path': str(args.intervention.resolve()), 'sha256': a_hash}],
              'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'control': summarize(control, samples, acceleration_scale),
              'intervention': summarize(intervention, samples, acceleration_scale),
              'limitations': 'Estimated tilt relative to incoming gravity, not physical attitude error. '
              'Matched-only NDT costs omit unmatched support and other factors; both grids can use '
              'the same return. Rotated acceleration omits offset subtraction and is not a complete '
              'optimizer residual or independent physical acceleration. No full-route, cloud-quality '
              'or robustness verdict. Exact control '
              'reproduction and intervention isolation require the separate driver validation.'}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: report[k]['final'] for k in ('control', 'intervention')}, indent=2))


if __name__ == '__main__':
    main()
