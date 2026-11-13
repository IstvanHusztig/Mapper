"""Audit the W87 guard trace and its assembly-only alternatives; never runs SLAM."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
from analyse_livox4_rotation_trace import PAIRS, groups
from run_livox4_gain_comparison import save, sha


def analyse(log, output):
    run_file = log.parent / 'run.json'
    run = json.loads(run_file.read_text()) if run_file.exists() else {}
    reproduced = (run.get('status') == 'validated' and run.get('mismatched_first130') == [])
    records = dict(groups(log))
    expected = {(w, i) for w in range(84, 103)
                for i in (range(100) if w == 87 else (0, 1, 2, 10, 99))}
    actual = {key for key in records if key[1] >= 0}
    errors = []
    if actual != expected:
        errors.append('Unexpected actual iteration coverage')
    alternative = {}
    pattern = re.compile(r'LIVOX4_ALT window=(\d+) iteration=(\d+) mode=(\w+) kind=(\w+)(?: (.*))?')
    for line in log.open(encoding='utf-8'):
        m = pattern.search(line)
        if not m:
            continue
        fields = dict(PAIRS.findall(m[5] or ''))
        if 'values' in fields:
            value = np.fromstring(fields['values'][1:-1], sep=',').reshape(int(fields['rows']), int(fields['cols']))
            if not np.isfinite(value).all():
                raise ValueError('Nonfinite alternative matrix')
        else:
            value = fields
        key = (int(m[1]), int(m[2]), m[3])
        alternative.setdefault(key, {}).setdefault(m[4], []).append(value)
    if set(alternative) != {(87, i, mode) for i in (0, 99) for mode in ('frozen', 'refreshed', 'legacy')}:
        errors.append('Unexpected alternative coverage')

    def one(r, name):
        if len(r[name]) != 1:
            raise ValueError('Duplicate matrix: ' + name)
        return r[name][0]

    # Difference of rounded logged corrections (reference minus guard), not surveyed truth.
    displacement = np.array([-.061, .059, -.006])
    velocity = displacement / .36
    direction = np.r_[np.zeros(3), velocity]
    unit = displacement / np.linalg.norm(displacement)
    rows = []
    for (w, i), r in sorted(records.items()):
        if i < 0:
            continue
        check = one(r, 'response_check')
        if float(check['step_error']) > 1e-8 or float(check['rhs_error']) > 1e-6 or int(check['dropped_writes']):
            errors.append(f'Factor reconstruction failed {w}/{i}')
        if w != 87:
            continue
        names = [f['name'] for f in r['factor']]
        before, after = one(r, 'poses_before'), one(r, 'poses_after')
        node = len(before) - 1
        factors = {}
        for name in names:
            h = one(r, 'factor_direction_H_' + name)
            b = one(r, 'factor_direction_rhs_' + name).ravel()
            response = one(r, 'shared_H_response_' + name).ravel()[6*node:6*node+3]
            curvature = float(direction @ h @ direction)
            push = float(b @ direction)
            factors[name] = {'curvature': curvature, 'linear_push': push,
                             'quadratic_cost_change': curvature - 2*push,
                             'shared_H_endpoint_response_m': response,
                             'shared_H_along_missing_correction_m': float(unit @ response)}
        rows.append({'iteration': i, 'endpoint_step_m': after[node, [3, 7, 11]] - before[node, [3, 7, 11]],
                     'factors': factors})
    comparisons = []
    for i in (0, 99):
        r = records[(87, i)]
        frozen = alternative[(87, i, 'frozen')]
        f1, f2 = {f['name']: f for f in r['factor']}, {f['name']: f for f in frozen['factor']}
        if set(f1) != set(f2):
            errors.append(f'Frozen factor coverage differs {i}')
        for name in f1:
            if f1[name]['H_fnv1a64'] != f2[name]['H_fnv1a64'] or not np.array_equal(
                    one(r, 'factor_rhs_' + name), one(frozen, 'factor_rhs_' + name)):
                errors.append(f'Frozen assembly differs from production {i}/{name}')
        for mode in ('frozen', 'refreshed', 'legacy'):
            a = alternative[(87, i, mode)]
            h = one(a, 'factor_direction_H_ndt_projected')
            b = one(a, 'factor_direction_rhs_ndt_projected').ravel()
            curvature, push = float(direction @ h @ direction), float(direction @ b)
            comparisons.append({'iteration': i, 'mode': mode, 'curvature': curvature,
                                'linear_push': push, 'quadratic_cost_change': curvature-2*push,
                                'matches': a['matches'], 'huber_support': a.get('huber_support', [])})
    artifact = {'log_sha256': sha(log), 'errors': errors, 'actual_iterations': len(actual),
                'original_run_reproduced': reproduced,
                'equivalence_status': run.get('status', 'NOT_MEASURED'),
                'interpretation_scope': 'same-state local assemblies only; original-run attribution requires prefix equivalence',
                'alternative_assemblies': len(alternative), 'direction': {'displacement_m': displacement,
                    'velocity_ramp_mps': velocity, 'span_s': .36,
                    'meaning': 'rounded reference-minus-guard logged correction; fixed-attitude ramp with both velocity variables; not ground truth'},
                'w87_iterations': rows, 'same_state_alternatives': comparisons,
                'limits': ['Local quadratic comparisons; no alternative solves or trajectories',
                           'Alternative policies use candidate map, not reference map',
                           'Shared-H factor responses are not factor-removal experiments']}
    save(output, artifact)
    print(json.dumps({'errors': errors, 'actual_iterations': len(actual), 'alternative_assemblies': len(alternative)}))
    return 1 if errors else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    raise SystemExit(analyse(args.log, args.output))
