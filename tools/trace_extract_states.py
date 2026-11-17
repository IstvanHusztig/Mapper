"""Extract exact selected iteration states from a validated trace for offline checks."""
import argparse,hashlib,json,re
from pathlib import Path

HEADER=re.compile(rb'LIVOX[124]_TRACE window=(\d+) iteration=(-?\d+) kind=(\w+) (.*)')
PAIRS=re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')
KINDS={'poses_before','poses_after','solver_update_before_bias_clamp','projection_matrix','matches','experiment_weights','range_step','projection'}

def extract(source,destination,window,iterations):
    digest=hashlib.sha256();result={i:{'iteration':i,'matches':[],'range_step':[],'projection':[]} for i in iterations}
    with source.open('rb') as stream:
        for line in stream:
            digest.update(line);m=HEADER.fullmatch(line.rstrip(b'\r\n'))
            if not m:raise ValueError('Malformed trace header')
            w,i=int(m[1]),int(m[2]);kind=m[3].decode()
            if w!=window or i not in result or kind not in KINDS:continue
            f=dict(PAIRS.findall(m[4].decode()));r=result[i]
            if 'values' in f:
                v=json.loads(f['values']);rows,cols=int(f['rows']),int(f['cols'])
                assert len(v)==rows*cols
                value=v if cols==1 else [v[k*cols:(k+1)*cols] for k in range(rows)]
            else:value=f
            if kind in ('matches','range_step','projection'):r[kind].append(value)
            else:
                assert kind not in r,'Duplicate state'
                r[kind]=value
    for r in result.values():
        assert KINDS<=r.keys() and len(r['matches'])==4 and len(r['range_step'])==4 and len(r['projection'])==3,'Incomplete iteration'
    artifact=dict(window=window,source_trace_sha256=digest.hexdigest(),
                  extractor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  iterations=[result[i] for i in sorted(result)])
    with destination.open('x',encoding='utf-8',newline='\n') as f:json.dump(artifact,f,allow_nan=False)
    print(json.dumps(dict(window=window,iterations=sorted(result),source_trace_sha256=digest.hexdigest())))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trace',type=Path);p.add_argument('destination',type=Path)
    p.add_argument('--window',type=int,required=True);p.add_argument('--iterations',type=int,nargs='+',required=True)
    a=p.parse_args();extract(a.trace,a.destination,a.window,set(a.iterations))
