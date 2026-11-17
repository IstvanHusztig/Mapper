"""One explicitly approved Livox4 prefix diagnostic; never retries or changes settings."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from run_livox4_gain_comparison import no_mapper, require, save, sha, window_bytes
import run_health


def prepare(reference, bundle, build, source):
    require(not bundle.exists(), 'Destination exists; no overwrite')
    no_mapper()
    require(shutil.disk_usage(bundle.parent).free > 15 * 1024**3, 'Need 15 GiB free')
    old = json.loads((reference / 'preparation.json').read_text())
    inputs = [reference / 'input' / n for n in ('imu0001.csv', 'lidar0001.laz')]
    for p in inputs + [reference / 'settings.toml']:
        require(sha(p) == old['hashes'][str(p)], 'Changed reference input/settings: ' + str(p))
    snapshot = reference / 'runs/gain-fix/result/odometry_snapshot.bin'
    collected = json.loads((reference / 'comparison.json').read_text())
    expected = collected['runs']['gain-fix']['outputs'][str(Path('result') / 'odometry_snapshot.bin')]
    require(sha(snapshot) == expected, 'Changed reference snapshot')
    bundle.mkdir()
    (bundle / 'bin').mkdir()
    shutil.copy2(reference / 'settings.toml', bundle / 'settings.toml')
    shutil.copy2(build / 'dMapper.exe', bundle / 'bin/dMapper.exe')
    for name in ('freeglut.dll', 'laszip3.dll', 'tbb12.dll'):
        p = reference / 'bin/gain-fix' / name
        require(sha(p) == old['hashes'][str(p)], 'Changed frozen DLL')
        shutil.copy2(p, bundle / 'bin' / name)
    diff = subprocess.run(['git', '-C', str(source), 'diff', 'fc73404', '--', 'src'],
                          check=True, capture_output=True).stdout
    (bundle / 'diagnostic.patch').write_bytes(diff)
    files = inputs + [snapshot, reference / 'runs/gain-fix/run.log', bundle / 'settings.toml',
                     bundle / 'diagnostic.patch', Path(__file__).resolve()]
    files += list((bundle / 'bin').iterdir())
    files += [source / 'src/domain/scan_to_map.cpp', source / 'src/application/run_odometry.cpp',
              source / 'out/build/x64-Release/CMakeCache.txt', source / 'out/build/x64-Release/build.ninja']
    record = {'status': 'prepared', 'reference': str(reference), 'input': str(reference / 'input'),
              'snapshot': str(snapshot), 'source': str(source), 'hashes': {str(p): sha(p) for p in files},
              'scope': 'one unchanged 125s candidate prefix; trace W297-300; exact first325 gate',
              'actual_alpha': 1, 'actual_beta': 1,
              'provenance_correction': 'Earlier gain-only label invalid: control alpha0.1, candidate alpha1/beta1.',
              'estimated_minutes': [10, 20]}
    save(bundle / 'manifest.json', record)


def run(bundle, approved):
    require(approved, 'Explicit user approval required')
    m = json.loads((bundle / 'manifest.json').read_text())
    require(m['status'] == 'prepared', 'Not prepared')
    require(not (bundle / 'run.json').exists(), 'Run already attempted; no retry')
    no_mapper()
    for p, expected in m['hashes'].items():
        require(sha(p) == expected, 'Changed prepared identity: ' + p)
    env = {k: v for k, v in os.environ.items() if not k.startswith('DMAPPER_')}
    env.update(DMAPPER_TRACE_LIVOX4='1', DMAPPER_DIAGNOSTIC_NDT_HUBER3='1')
    result = bundle / 'result'
    command = [str(bundle / 'bin/dMapper.exe'), m['input'], str(bundle / 'settings.toml'), str(result)]
    record = {'status': 'running', 'approved': True, 'command': command,
              'environment': {k: v for k, v in env.items() if k.startswith('DMAPPER_')}}
    save(bundle / 'run.json', record)
    start = time.monotonic()
    process = None
    try:
        with (bundle / 'run.log').open('xb') as log:
            process = subprocess.Popen(command, cwd=bundle / 'bin', env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
            record['pid'] = process.pid
            save(bundle / 'run.json', record)
            code = process.wait()
        record.update(exit=code, wall_s=time.monotonic() - start)
        require(code == 0, 'Native run failed; no retry')
        for p, expected in m['hashes'].items():
            require(sha(p) == expected, 'Prepared identity changed during replay: ' + p)
        a, b = window_bytes(m['snapshot']), window_bytes(result / 'odometry_snapshot.bin')
        require(len(a) == len(b) and len(b) >= 325, 'Snapshot coverage mismatch')
        record['mismatched_windows'] = [i + 1 for i, (x, y) in enumerate(zip(a, b)) if x != y]
        require(not record['mismatched_windows'], 'Trace changed trajectory; causal attribution blocked')
        record['exact_windows'] = len(b)
        health = run_health.analyse(str(bundle / 'run.log'), str(bundle / 'settings.toml'))
        save(bundle / 'run-health.json', health)
        require(health['completed'] and health['windows'] == health['expected_windows'], 'Incomplete run')
        record['status'] = 'collected'
        record['log_sha256'] = sha(bundle / 'run.log')
        record['snapshot_sha256'] = sha(result / 'odometry_snapshot.bin')
    except BaseException as error:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        record.update(status='needs_review', error=repr(error), wall_s=time.monotonic() - start)
        raise
    finally:
        save(bundle / 'run.json', record)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    prep = sub.add_parser('prepare')
    for name in ('reference', 'bundle', 'build', 'source'):
        prep.add_argument(name, type=Path)
    launch = sub.add_parser('run')
    launch.add_argument('bundle', type=Path)
    launch.add_argument('--approved', action='store_true')
    args = vars(p.parse_args())
    action = args.pop('action')
    (prepare if action == 'prepare' else run)(**args)
