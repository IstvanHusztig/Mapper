"""Summarize existing NDT refresh events and saved attitudes; never runs SLAM."""
import argparse
import json
from pathlib import Path
import re
import struct
import numpy as np
from run_livox4_gain_comparison import save, sha, window_bytes


def analyse(log, snapshot, output):
    windows = {w: {'admission': [], 'turnover': [], 'diagnostics': [], 'bias': []} for w in range(296, 314)}
    events = []
    for line in log.open(encoding='utf-8'):
        m = re.match(r'NDT_MATCH_(GUARD|REFRESH) window=(\d+) iteration=(\d+) (.*)', line)
        if m and int(m[2]) in windows:
            row = dict(re.findall(r'(\w+)=(\S+)', m[4]))
            row['iteration'] = int(m[3])
            windows[int(m[2])]['admission' if m[1] == 'GUARD' else 'turnover'].append(row)
        m = re.match(r'window diagnostics (\d+):', line)
        if m and int(m[1]) in windows: windows[int(m[1])]['diagnostics'].append(line.strip())
        m = re.match(r'gyro bias \[deg/s\] window (\d+):', line)
        if m and int(m[1]) in windows: windows[int(m[1])]['bias'].append(line.strip())
        if any(t in line for t in ('silent events:', 'rebuild', 'big jump on trajectory:')) and len(line)<1000:
            events.append(line.strip())
    for w, row in windows.items():
        admissions = row['admission']
        row['iteration_ids_complete'] = [a['iteration'] for a in admissions] == list(range(100))
        if not row['iteration_ids_complete']:
            raise ValueError(f'Incomplete admission iterations in window {w}')
        row['support_range'] = {k: [min(int(a[k]) for a in admissions), max(int(a[k]) for a in admissions)]
                               for k in ('unique_sources','fine','coarse','invalid_fine','invalid_coarse','incompatible_fine','incompatible_coarse')}
        row['turnover_summary'] = {}
        for grid in ('fine','coarse'):
            records = [a for a in row['turnover'] if a['grid']==grid]
            if [a['iteration'] for a in records] != list(range(1,100)):
                raise ValueError(f'Incomplete turnover iterations in window {w}/{grid}')
            row['turnover_summary'][grid] = {k: {'sum_events':sum(int(a[k]) for a in records),
                 'peak':max(int(a[k]) for a in records),
                 'peak_iteration':max(records,key=lambda a:int(a[k]))['iteration']}
                 for k in ('gained','lost','reassigned')}
    for w, payload in enumerate(window_bytes(snapshot), 1):
        if w not in windows: continue
        offset=0; arrays=[]
        for width in (16,16,2,3,3):
            n,=struct.unpack_from('<Q',payload,offset); offset+=8
            a=np.frombuffer(payload,dtype='<f8',count=n*width,offset=offset).reshape(n,width);offset+=n*width*8;arrays.append(a)
        poses,model,stamps,angles,acc=arrays
        rotations=np.stack([p.reshape(4,4,order='F')[:3,:3] for p in poses])
        windows[w]['snapshot']={'first_sensor_time_s':float(stamps[0,0]),'last_sensor_time_s':float(stamps[-1,0]),
            'max_abs_pitch_deg':float(np.max(np.abs(np.rad2deg(np.arcsin(np.clip(rotations[:,0,2],-1,1)))))),
            'max_orthogonality_error':float(np.max(np.abs(np.transpose(rotations,(0,2,1))@rotations-np.eye(3)))),
            'determinant_range':[float(np.linalg.det(rotations).min()),float(np.linalg.det(rotations).max())],
            'model_equals_final':bool(np.array_equal(poses,model)),
            'imu_node_gap_max_s':float(np.diff(stamps[:,0]).max()),
            'limits':'Saved post-handoff poses; not pre-solve state or original correspondence replay'}
    report={'log_sha256':sha(log),'snapshot_sha256':sha(snapshot),'tool_sha256':sha(Path(__file__)),
            'windows':windows,'events':events,'limits':['Turnover sums count repeated events, not unique sources',
            'Admission support does not measure angular observability','No factor traces at W301-302; cannot isolate initiating factor']}
    save(output,report)
    for w in (298,300,301,302,303,312):
        r=windows[w]
        print(json.dumps({'window':w,'support_range':r['support_range'],'turnover':r['turnover_summary'],
                         'snapshot':r['snapshot']}))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('log',type=Path);p.add_argument('snapshot',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();analyse(a.log,a.snapshot,a.output)
