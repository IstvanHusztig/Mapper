"""Run the prepared two-prefix AHRS gain comparison only after explicit approval."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

import numpy as np

import run_health
import snapshot_trajectory
import trajectory_seams


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(chunk)
    return h.hexdigest()


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Unsupported JSON type: {type(value).__name__}')


def save(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False, default=json_value) + '\n')
    temp.replace(path)


def require(value, message):
    if not value:
        raise RuntimeError(message)


def window_bytes(path):
    # Structural/finiteness validation uses the existing reader first.
    snapshot_trajectory.load_snapshot(path)
    data = Path(path).read_bytes()
    _, _, _, count = struct.unpack_from('<8sIQQ', data)
    offset = struct.calcsize('<8sIQQ')
    result = []
    for _ in range(count):
        start = offset
        for width in (16, 16, 2, 3, 3):
            n, = struct.unpack_from('<Q', data, offset)
            offset += 8 + n * width * 8
        offset += 27 * 8
        result.append(data[start:offset])
    return result


def no_mapper():
    result = subprocess.run(['powershell', '-NoProfile', '-Command',
        "@(Get-Process -Name dMapper -ErrorAction SilentlyContinue).Count"],
        capture_output=True, text=True, check=True)
    require(result.stdout.strip() == '0', 'Another dMapper process owns a recording cache')


def verify(manifest, bundle):
    required = [bundle / 'settings.toml', Path(manifest['full_reference_snapshot'])]
    required += [bundle / 'bin' / name / file for name in ('control', 'gain-fix')
                 for file in ('dMapper.exe', 'freeglut.dll', 'laszip3.dll', 'tbb12.dll')]
    required += [bundle / 'input' / name for name in ('imu0001.csv', 'lidar0001.laz')]
    required += [Path(__file__), Path(run_health.__file__),
                 Path(snapshot_trajectory.__file__), Path(trajectory_seams.__file__)]
    covered = {str(Path(p).resolve()).casefold() for p in manifest['hashes']}
    for path in required:
        require(str(path.resolve()).casefold() in covered, 'Missing frozen identity: ' + str(path))
    for path, expected in manifest['hashes'].items():
        require(sha(path) == expected, 'Changed prepared artifact: ' + path)


def run(bundle, approved, resume_control=False):
    require(approved, 'No launch: explicit --approved required after user approval')
    manifest = json.loads((bundle / 'preparation.json').read_text())
    require(manifest['status'] == 'prepared', 'Bundle not prepared')
    require(manifest['control_windows'] == 325, 'Unexpected comparison boundary')
    if resume_control:
        record = json.loads((bundle / 'comparison.json').read_text())
        require(record['approved'] and record['status'] == 'needs_review', 'Not a stopped approved comparison')
        require(record.get('error') == "TypeError('Object of type ndarray is not JSON serializable')",
                'Recovery only supports the known collection error')
        require(set(record['runs']) == {'control'} and record['runs']['control'].get('exit') == 0,
                'Recovery requires exactly one successful native control')
        require(not (bundle / 'runs' / 'gain-fix').exists(), 'Candidate already exists: no retry')
    else:
        require(not (bundle / 'runs').exists(), 'Existing runs: no retry or overwrite')
    verify(manifest, bundle)
    no_mapper()
    require(shutil.disk_usage(bundle).free > 15 * 1024**3, 'Need 15 GiB free')
    if not resume_control:
        (bundle / 'runs').mkdir()
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('DMAPPER_'):
            del env[key]
    env['DMAPPER_DIAGNOSTIC_NDT_HUBER3'] = '1'
    if not resume_control:
        record = {'status': 'running', 'approved': True, 'runs': {}, 'accuracy_accepted': False}
    else:
        record['collection_recovery'] = record.pop('error')
        record['status'] = 'running'
    save(bundle / 'comparison.json', record)
    try:
        for name in ('control', 'gain-fix'):
            no_mapper()
            folder = bundle / 'runs' / name
            result = folder / 'result'
            if resume_control and name == 'control':
                info = record['runs']['control']
            else:
                result.mkdir(parents=True)
                command = [str(bundle / 'bin' / name / 'dMapper.exe'),
                           str(bundle / 'input'), str(bundle / 'settings.toml'), str(result)]
                info = {'command': command, 'status': 'running'}
                record['runs'][name] = info
                save(bundle / 'comparison.json', record)
                start = time.monotonic()
                with (folder / 'run.log').open('xb') as log:
                    process = None
                    try:
                        process = subprocess.Popen(command, env=env, cwd=bundle / 'bin' / name,
                                                   stdout=log, stderr=subprocess.STDOUT)
                        info['pid'] = process.pid
                        save(bundle / 'comparison.json', record)
                        code = process.wait()
                    except BaseException:
                        if process is not None and process.poll() is None:
                            process.terminate()
                            try:
                                process.wait(timeout=15)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait()
                        raise
                info.update(exit=code, wall_s=time.monotonic() - start)
                require(code == 0, name + ' failed; no automatic retry')
            verify(manifest, bundle)
            text = (folder / 'run.log').read_text()
            require(text.count('NDT_HUBER_DIAGNOSTIC enabled=1') == 1, 'Missing Huber marker')
            require('AHRS_GRAVITY_DIAGNOSTIC enabled=1' not in text, 'Wrong gravity model')
            health = run_health.analyse(str(folder / 'run.log'), str(bundle / 'settings.toml'))
            require(health['completed'] and health['windows'] == health['expected_windows'],
                    'Incomplete window coverage')
            save(folder / 'run-health.json', health)
            info['diagnostics_complete'] = not health['missing_required']
            info['windows'] = health['windows']
            snapshot = result / 'odometry_snapshot.bin'
            _, metadata = snapshot_trajectory.load_snapshot(snapshot, with_metadata=True)
            require(len(metadata['windows']) == health['windows'], 'Snapshot/log window mismatch')
            cloud = result / 'scan_lio_merged.laz'
            require(cloud.is_file() and cloud.stat().st_size > 1000, 'Missing or empty merged cloud')
            if name == 'control':
                a, b = window_bytes(manifest['full_reference_snapshot']), window_bytes(snapshot)
                require(len(b) >= 325, 'Prefix too short')
                mismatch = next((i + 1 for i in range(325) if a[i] != b[i]), None)
                info['first_mismatched_window'] = mismatch
                require(mismatch is None, 'Prefix control differs from full run; candidate NOT launched')
                info['exact_control_windows'] = 325
            else:
                reference = bundle / 'runs' / 'control' / 'result'
                info['trajectory_comparison'] = {}
                for stage in ('odometry', 'final'):
                    a = trajectory_seams.load(str(reference), stage)
                    b = trajectory_seams.load(str(result), stage)
                    info['trajectory_comparison'][stage] = trajectory_seams.compare(a, b, [.05, .2, .5])
            info['outputs'] = {str(p.relative_to(folder)): sha(p) for p in result.iterdir() if p.is_file()}
            info['log_sha256'] = sha(folder / 'run.log')
            info['status'] = 'collected'
            save(bundle / 'comparison.json', record)
        record['status'] = 'collected'
    except BaseException as error:
        record.update(status='needs_review', error=repr(error))
        raise
    finally:
        save(bundle / 'comparison.json', record)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--approved', action='store_true')
    parser.add_argument('--resume-control-collection', action='store_true')
    args = parser.parse_args()
    run(args.bundle.resolve(), args.approved, args.resume_control_collection)
