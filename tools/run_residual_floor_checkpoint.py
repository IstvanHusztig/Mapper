"""Preflight a frozen checkpoint manifest; execute only with recorded approval.

No retry/resume and no output reuse. --preflight never invokes the optimizer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def run(manifest, preflight=False):
    manifest = Path(manifest).resolve()
    m = json.loads(manifest.read_text())
    if not preflight and m['approval']['authorized'] is not True:
        raise ValueError('Checkpoint is not authorized; record explicit user approval first')
    output = Path(m['output'])
    command = m['command']
    if (len(command) != 9 or command[8] != '--floor6mm' or
            Path(command[6]).resolve() != output.resolve() or
            Path(command[0]).name != 'reconstruct_w754.exe' or
            m['window'] != 415 or m['candidate_sigma_m'] != .006):
        raise ValueError('Command is not the declared W415 control/floor6mm experiment')
    pinned = {Path(entry['path']).resolve() for entry in m['files']}
    if any(Path(command[k]).resolve() not in pinned for k in (0,1,2,4,5,7)):
        raise ValueError('Every executable/input command file must have a pinned hash')
    if output.exists():
        raise FileExistsError(f'Preserve existing output: {output}')
    for entry in m['files']:
        if sha(entry['path']) != entry['sha256']:
            raise ValueError(f"Frozen file changed: {entry['path']}")
    if preflight:
        print(f"Preflight passed: {len(m['files'])} hashes; optimizer NOT started")
        return 0
    record = manifest.parent / 'run.json'
    facts = dict(status='running', manifest_sha256=sha(manifest),
                 command=m['command'], approval=m['approval'], automatic_retry=False)
    # Exclusive reservation prevents duplicate launches even from concurrent callers.
    with record.open('x') as f:
        json.dump(facts, f, indent=2)
    start = time.monotonic()
    try:
        with (manifest.parent / 'checkpoint.log').open('x') as log:
            result = subprocess.run(m['command'], stdout=log, stderr=subprocess.STDOUT)
        facts.update(status='native_finished' if result.returncode == 0 else 'failed',
                     exit=result.returncode, elapsed_s=time.monotonic()-start)
        if result.returncode == 0:
            validation = json.loads((output / 'validation.json').read_text())
            optimizer = validation['optimizer']
            assert validation['optimizer_control_exact'] and optimizer['control_all100_exact']
            assert optimizer['control_iterations'] == optimizer['floor6mm_iterations'] == 100
            assert optimizer['floor_sigma_m'] == .006
            facts['status'] = 'checkpoint_complete_accuracy_not_accepted'
        for entry in m['files']:
            if sha(entry['path']) != entry['sha256']:
                raise ValueError(f"Input changed during checkpoint: {entry['path']}")
        return result.returncode
    except BaseException as error:
        facts.update(status='failed', error=str(error))
        raise
    finally:
        temporary = record.with_suffix('.tmp')
        with temporary.open('x') as f:
            json.dump(facts, f, indent=2, allow_nan=False)
        temporary.replace(record)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    raise SystemExit(run(args.manifest, args.preflight))
