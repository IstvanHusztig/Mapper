"""Stream a validated LIVOX2 trace into compact analysis records; never runs SLAM.

Preserves repeated scalar kinds as lists. Large matrices are reduced explicitly;
the immutable source trace remains authoritative. Input must pass the replay's
schema/completeness validator before causal use.
"""
import argparse, hashlib, json, re
from pathlib import Path
import numpy as np

HEADER = re.compile(rb'LIVOX2_TRACE window=(\d+) iteration=(-?\d+) kind=(\w+) (.*)')
PAIRS = re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')
POSES = {'poses_before','poses_after','seed_before_prediction','after_prediction_before_bias',
         'after_bias_before_integration','after_handoff_tail'}

def build(source, output):
    output.mkdir(exist_ok=False)
    digest=hashlib.sha256(); groups=0; records=0; current=None; group=None
    samples={}; gravity={}; scale={}
    with source.open('rb') as stream, (output/'groups.jsonl').open('w',encoding='utf-8',newline='\n') as dst:
        for line in stream:
            digest.update(line); m=HEADER.fullmatch(line.rstrip(b'\r\n'))
            if not m: raise ValueError('Malformed trace header')
            w,it=int(m[1]),int(m[2]); kind=m[3].decode(); key=(w,it)
            if key!=current:
                if group is not None: dst.write(json.dumps(group,allow_nan=False)+'\n');groups+=1
                current=key;group={'window':w,'iteration':it,'records':{}}
            records+=1
            # Large factor matrices are not needed for this first reduction.
            if kind.startswith(('factor_rhs_','factor_direction_H_','range_normal_sum_',
                                'range_information_sum_','range_residual_sum_')): continue
            fields=dict(PAIRS.findall(m[4].decode()))
            if 'values' in fields:
                a=np.fromstring(fields['values'][1:-1],sep=',').reshape(int(fields['rows']),int(fields['cols']))
                if not np.isfinite(a).all(): raise ValueError('Nonfinite matrix')
                if kind=='node_samples': samples[w]=a;value={'rows':len(a),'first':a[0].tolist(),'last':a[-1].tolist()}
                elif kind in POSES:
                    pose=a.reshape(-1,3,4); value={'rows':len(a),'first':pose[0].tolist(),'last':pose[-1].tolist(),
                        'displacement':(pose[-1,:,3]-pose[0,:,3]).tolist()}
                    if w in samples and w in gravity and w in scale and len(a)==len(samples[w]):
                        acc=np.einsum('nij,nj->ni',pose[:,:,:3],samples[w][:,1:4])
                        value['mean_world_acc_minus_incoming_gravity_mps2']=((acc-gravity[w])*scale[w]).mean(axis=0).tolist()
                elif kind=='solver_update_before_bias_clamp':
                    step=a.ravel();n=(len(step)-12)//6;pose=step[:n*6].reshape(n,6)
                    value={'last_pose_step':pose[-1].tolist(),'max_translation_step':float(np.linalg.norm(pose[:,:3],axis=1).max())}
                else:
                    value=a.tolist()
                    if kind=='incoming_gravity':gravity[w]=a.ravel()
            else:
                value=fields
                if kind=='window_input':scale[w]=float(fields['to_mps2'])
            group['records'].setdefault(kind,[]).append(value)
        if group is not None:dst.write(json.dumps(group,allow_nan=False)+'\n');groups+=1
    manifest={'source':str(source),'source_sha256':digest.hexdigest(),'records':records,'groups':groups,
              'tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'output_sha256':hashlib.sha256((output/'groups.jsonl').read_bytes()).hexdigest(),
              'limits':'Pose endpoints and mean world acceleration are reductions, not ground truth; matrices omitted as documented in source.'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(manifest))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trace',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();build(args.trace,args.output)
