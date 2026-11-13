"""Read-only partial-log jump monitoring; never stops or changes the mapper."""
from pathlib import Path
import hashlib
import json
import math
import re
import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('log', type=Path)
parser.add_argument('--output', type=Path, help='Optional directory for immutable prefix snapshots')
parser.add_argument('--compact', action='store_true')
args = parser.parse_args()
LOG = args.log
OUT = args.output
N = r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?'
V = '(' + N + ') (' + N + ') (' + N + ')'
raw = LOG.read_bytes()
complete = raw.rsplit(b'\n', 1)[0] if b'\n' in raw else b''
text = complete.decode(errors='replace')
windows = {}
for m in re.finditer(r'^window diagnostics (\d+): t (' + N + r') s, dt (' + N +
                     r') s,.*?start ' + V + r', guess move ' + V + r', result move ' + V +
                     r', previous-velocity move .*?, result-guess ' + V, text, re.M):
    windows[int(m[1])] = {'window': int(m[1]), 'time_s': float(m[2]), 'dt_s': float(m[3]),
                         'start': [float(m[k]) for k in (4, 5, 6)],
                         'result_guess_m': math.sqrt(sum(float(m[k])**2 for k in (13, 14, 15)))}
for m in re.finditer(r'^window diagnostics (\d+): velocity start \S+ \S+ \S+, end ' + V, text, re.M):
    if int(m[1]) in windows:
        windows[int(m[1])]['speed_mps'] = math.sqrt(sum(float(m[k])**2 for k in (2, 3, 4)))
for m in re.finditer(r'^window diagnostics (\d+): heading \S+ deg, yaw correction first \S+ last (' + N + ') deg', text, re.M):
    if int(m[1]) in windows:
        windows[int(m[1])]['yaw_correction_deg'] = float(m[2])
for m in re.finditer(r'^window diagnostics (\d+): weak axis.*?start-velocity jump ' + V, text, re.M):
    if int(m[1]) in windows:
        windows[int(m[1])]['conditional_velocity_jump_mps'] = math.sqrt(sum(float(m[k])**2 for k in (2, 3, 4)))
rows = [w for _, w in sorted(windows.items()) if 'speed_mps' in w and 'yaw_correction_deg' in w]
for first, second in zip(rows, rows[1:]):
    if second['window'] == first['window'] + 1 and second['time_s'] > first['time_s']:
        second['start_displacement_m'] = math.dist(first['start'], second['start'])
        second['start_displacement_speed_mps'] = second['start_displacement_m'] / (second['time_s'] - first['time_s'])


def events(key, threshold):
    values = [dict(window=w['window'], time_s=w['time_s'], value=w[key]) for w in rows if key in w and abs(w[key]) > threshold]
    maxima = sorted([dict(window=w['window'], time_s=w['time_s'], value=w[key]) for w in rows if key in w], key=lambda x: abs(x['value']), reverse=True)
    return {'threshold': threshold, 'count': len(values), 'events': values, 'first': values[:5], 'largest': maxima[:5]}


guards = [line for line in text.splitlines() if re.search(r'warning|skipped|big.jump|reset|rebuilt', line, re.I)
          and not line.startswith('LIVOX1_TRACE')]
summary = {'status': 'PARTIAL LIVE OBSERVATION; NOT A COMPLETION OR ACCURACY VERDICT',
           'last_complete_window': rows[-1]['window'] if rows else None,
           'last_time_s': rows[-1]['time_s'] if rows else None, 'parsed_windows': len(rows),
           'end_speed_mps': events('speed_mps', 2.5),
           'successive_window_start_displacement_speed_mps': events('start_displacement_speed_mps', 2.5),
           'result_minus_guess_m': events('result_guess_m', 1.0),
           'yaw_correction_deg_gt3': events('yaw_correction_deg', 3.),
           'yaw_correction_deg_gt10': events('yaw_correction_deg', 10.),
           'conditional_start_velocity_jump_mps': events('conditional_velocity_jump_mps', 2.5),
           'warning_guard_lines': guards, 'source': str(LOG),
           'monitor_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
           'complete_line_prefix_bytes': len(complete), 'complete_line_prefix_sha256': hashlib.sha256(complete).hexdigest(),
           'limitation': 'Window-start displacement uses logged millimetre/centisecond precision; it is not an instantaneous node jump. '
                         'Velocity-jump diagnostics cover eligible weak-axis windows only. Guard absence does not prove no sub-window jump.'}
if OUT is not None:
    OUT.mkdir(exist_ok=True)
    destination = OUT / (str(summary['last_complete_window']) + '-' + summary['complete_line_prefix_sha256'][:12] + '.json')
    if not destination.exists():
        with destination.open('x', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
if args.compact:
    summary = {key: value for key, value in summary.items() if key in ('status','last_complete_window','last_time_s','parsed_windows','end_speed_mps','yaw_correction_deg_gt3','yaw_correction_deg_gt10')}
    for value in summary.values():
        if isinstance(value,dict):
            value.pop('events',None)
            value['first'] = value['first'][:1]
            value['largest'] = value['largest'][:1]
print(json.dumps(summary, indent=None if args.compact else 2, allow_nan=False))
