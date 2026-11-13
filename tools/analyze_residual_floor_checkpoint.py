"""Read a completed W415 floor checkpoint; never starts processing.

Reports local optimizer/support/covariance evidence, not map accuracy.
Fixed-pose world torque is an infinitesimal world-axis diagnostic, not Euler
parameter torque or the coupled/prior-conditioned pose update.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse(path):
    records = {}
    for line in path.open():
        m = re.fullmatch(r'LIVOX2_TRACE window=415 iteration=(\d+) kind=(\w+)(.*)\n?', line)
        if not m:
            raise ValueError('Unexpected trace line')
        it, kind, body = int(m[1]), m[2], m[3]
        matrix = re.fullmatch(r' rows=(\d+) cols=(\d+) values=(\[.*\])', body)
        value = (np.asarray(json.loads(matrix[3]), dtype=float).reshape(int(matrix[1]), int(matrix[2]))
                 if matrix else dict(re.findall(r'(\w+)=([^ ]+)', body)))
        records.setdefault(it, {}).setdefault(kind, []).append(value)
    if set(records) != set(range(100)):
        raise ValueError('Expected all 100 iterations')
    return records


def one(record, key):
    values = record[key]
    if len(values) != 1:
        raise ValueError('Nonunique record: ' + key)
    return values[0]


def summarize(records):
    initial, final = records[0], records[99]
    finite_states = all(np.isfinite(one(records[i], key)).all() for i in range(100)
        for key in ('poses_after', 'gyro_after', 'offset_after',
                    'velocity_start_after', 'velocity_end_after'))
    seed = one(initial, 'poses_before').reshape(-1, 3, 4)
    poses = one(final, 'poses_after').reshape(-1, 3, 4)
    rotations = Rotation.from_matrix(seed[:, :, :3].transpose(0, 2, 1) @ poses[:, :, :3])
    covariances = {}
    for key, units in [('gyro_cov_after', '(rad/s)^2'),
                       ('velocity_end_cov_after', '(m/s)^2'),
                       ('offset_cov_after', '(m/s^2)^2')]:
        matrices = np.stack([one(records[i], key) for i in range(100)])
        eigs = np.linalg.eigvalsh((matrices + matrices.transpose(0, 2, 1)) / 2)
        covariances[key] = dict(units=units, records=100,
            all_finite=bool(np.isfinite(matrices).all()),
            all_positive_eigenvalues=bool((eigs > 0).all()),
            max_asymmetry=float(np.max(np.abs(matrices-matrices.transpose(0, 2, 1)))),
            minimum_eigenvalue=float(eigs.min()), final_matrix=matrices[-1].tolist(),
            final_axis_sigmas=np.sqrt(np.diag(matrices[-1])).tolist())
    support = {}
    for grid in ('indoor', 'outdoor'):
        series = [next(x for x in records[i]['matches'] if x['stage'] == grid+'_after') for i in range(100)]
        weights = [sum(float(x['sum_weight']) for x in records[i]['huber_support']
                       if x['stage'] == grid+'_after') for i in range(100)]
        support[grid] = dict(final_matches=int(series[-1]['matched']),
            min_matches=min(int(x['matched']) for x in series),
            max_matches=max(int(x['matched']) for x in series),
            final_sum_weight=weights[-1], min_sum_weight=min(weights), max_sum_weight=max(weights))
    velocity = one(final, 'velocity_end_after').ravel()
    prior = one(initial, 'velocity_prior_mean').ravel()
    vcov = one(initial, 'velocity_prior_cov')
    start_velocity = one(final, 'velocity_start_after').ravel()
    innovation = start_velocity-prior
    rhs = one(initial, 'factor_rhs_ndt_projected').ravel()[:len(poses)*6].reshape(-1, 6)
    return dict(iterations=100, all_states_finite=bool(finite_states),
        accepted_steps=sum(int(one(records[i], 'decision')['applied']) for i in range(100)),
        last_rotation_from_seed_deg=float(np.degrees(rotations.magnitude()[-1])),
        max_rotation_from_seed_deg=float(np.degrees(rotations.magnitude().max())),
        last_position=poses[-1, :, 3].tolist(),
        end_velocity_mps=velocity.tolist(), end_velocity_norm_mps=float(np.linalg.norm(velocity)),
        start_velocity_mps=start_velocity.tolist(),
        start_velocity_prior_innovation_mps=innovation.tolist(),
        start_velocity_prior_quadratic=float(innovation @ np.linalg.solve(vcov, innovation)),
        gyro_deg_s=np.degrees(one(final, 'gyro_after').ravel()).tolist(),
        offset_mps2=one(final, 'offset_after').ravel().tolist(),
        initial_projected_ndt_rhs_sum_xyz_euler=rhs.sum(axis=0).tolist(),
        covariances=covariances, support=support,
        carry='Outgoing blocks only; subsequent real window propagation NOT MEASURED'), poses, seed


def fixed_influence(home, seed, candidate_initial):
    result = {}
    for grid in ('indoor', 'outdoor'):
        path = home / f'iteration-0-{grid}.csv'
        rows = np.genfromtxt(path, delimiter=',', names=True)
        columns = lambda names: np.column_stack([rows[n] for n in names])
        cov = columns([f'cov{i}{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
        info = columns([f'info{i}{j}' for i in range(3) for j in range(3)]).reshape(-1, 3, 3)
        residual = columns(['residual_'+a for a in 'xyz'])
        eig, axes = np.linalg.eigh(cov)
        if not np.isfinite(cov).all() or not (eig > 0).all():
            raise ValueError('Invalid fixed-state cell covariance')
        floored = (axes * (1 / np.maximum(eig, .006**2))[:, None, :]) @ axes.transpose(0, 2, 1)
        lever = columns(['world_'+a for a in 'xyz']) - seed[rows['pose_index'].astype(int), :, 3]
        changed = eig[:, 0] < .006**2
        variants = {}
        per_variant = {}
        for label, matrix in [('control', info), ('floor6mm', floored)]:
            ir = np.einsum('nij,nj->ni', matrix, residual)
            q = np.einsum('ni,ni->n', residual, ir)
            if not (q >= 0).all():
                raise ValueError('Invalid Mahalanobis square')
            w = np.minimum(1, 3 / np.sqrt(np.maximum(q, np.finfo(float).tiny)))
            force = -.1 * w[:, None] * ir
            torque = np.cross(lever, force)
            variants[label] = dict(sum_weight=float(w.sum()), q_median=float(np.median(q)),
                signed_world_translation_rhs=force.sum(axis=0).tolist(),
                signed_world_rotation_rhs=torque.sum(axis=0).tolist())
            per_variant[label] = (w, force, torque)
        np.testing.assert_allclose(per_variant['control'][0], rows['huber_weight'], rtol=1e-9, atol=1e-12)
        recorded_weight = sum(float(x['sum_weight']) for x in candidate_initial['huber_support']
                              if x['stage'] == grid+'_before')
        np.testing.assert_allclose(variants['floor6mm']['sum_weight'], recorded_weight, rtol=1e-8)
        cells = {}
        for key in (85899345919, 85899345918, 85899280383):
            mask = rows['cell_key'] == key
            if mask.any():
                cells[str(key)] = dict(count=int(mask.sum()), changed=int((mask & changed).sum()),
                    variants={label: dict(sum_weight=float(data[0][mask].sum()),
                        signed_translation_rhs=data[1][mask].sum(axis=0).tolist(),
                        signed_world_rotation_rhs=data[2][mask].sum(axis=0).tolist())
                        for label, data in per_variant.items()})
        result[grid] = dict(rows=len(rows), affected_returns=int(changed.sum()),
            affected_cells=len(np.unique(rows['cell_key'][changed])), variants=variants,
            known_cells=cells, csv_sha256=sha(path),
            units='Translation RHS m^-1; world infinitesimal rotation RHS rad^-1; alpha=0.1 included, before degeneracy projection/priors; grids may share returns')
    return result


def analyze(bundle):
    home = bundle/'result'
    run = json.loads((bundle/'run.json').read_text())
    validation = json.loads((home/'validation.json').read_text())
    if run['status'] != 'checkpoint_complete_accuracy_not_accepted' or not validation['optimizer']['control_all100_exact']:
        raise ValueError('Completed exact control gate required')
    records = {label: parse(home/(label+'-trace.log')) for label in ('control', 'floor6mm')}
    report, poses, seeds = {}, {}, {}
    for label in records:
        report[label], poses[label], seeds[label] = summarize(records[label])
        report[label]['trace_sha256'] = sha(home/(label+'-trace.log'))
    np.testing.assert_array_equal(seeds['control'], seeds['floor6mm'])
    for key in ('gyro_before', 'offset_before', 'velocity_start_before', 'velocity_end_before',
                'gyro_prior_mean', 'gyro_prior_cov', 'offset_prior_cov', 'velocity_prior_mean', 'velocity_prior_cov'):
        np.testing.assert_array_equal(one(records['control'][0], key), one(records['floor6mm'][0], key))
    for stage in ('indoor_before', 'outdoor_before'):
        matches = [next(x for x in records[label][0]['matches'] if x['stage'] == stage) for label in records]
        for key in ('source_count', 'matched', 'source_fnv1a64', 'match_cell_fnv1a64', 'matched_cell_geometry_fnv1a64'):
            if matches[0][key] != matches[1][key]:
                raise ValueError('Fixed-state association mismatch: '+key)
    report['fixed_state'] = fixed_influence(home, seeds['control'], records['floor6mm'][0])
    counts = [one(records['floor6mm'][i], 'ndt_residual_floor') for i in range(100)]
    report['floor_skip_diagnostics'] = {k: dict(min=min(int(x[k]) for x in counts),
        max=max(int(x[k]) for x in counts)) for k in counts[0] if k != 'sigma_m'}
    report['comparison'] = dict(max_node_position_difference_m=float(np.linalg.norm(
        poses['control'][:, :, 3]-poses['floor6mm'][:, :, 3], axis=1).max()),
        initial_states_priors_associations_equal=True,
        limitation='Single restored window; no full-route accuracy, next-window carry, robustness, or corrected cloud result. Cross-sigma costs are not geometry scores.')
    report['native_elapsed_s'] = run['elapsed_s']
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = analyze(args.bundle)
    with args.output.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    print(json.dumps({label: {k: report[label][k] for k in
          ('accepted_steps', 'last_rotation_from_seed_deg', 'end_velocity_norm_mps', 'gyro_deg_s')}
          for label in ('control', 'floor6mm')}, indent=2))
