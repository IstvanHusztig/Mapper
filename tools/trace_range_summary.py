"""Stream selected windows' range diagnostics to CSV; does not run mapping.

Information and normal sums describe matched cells, not the joint pose Hessian.
Costs are alpha-scaled; information sums are unscaled. Both grids may use the
same returns. Never interpret their summed match count as unique points.
"""
import argparse,csv,hashlib,json,re
from pathlib import Path
import numpy as np

HEADER=re.compile(rb'LIVOX2_TRACE window=(\d+) iteration=(-?\d+) kind=(\w+) (.*)')
PAIR=re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')

def build(source,output,windows):
    output.mkdir(exist_ok=False);digest=hashlib.sha256();current=None;records={};count=0
    columns=['window','iteration','stage','bin','source','eligible','matched','cost','residual_sq',
             'residual_rms','mean_residual_x','mean_residual_y','mean_residual_z',
             'info_trace_per_match','info_eig_min_per_match','info_eig_max_per_match',
             'info_max_axis_x','info_max_axis_y','info_max_axis_z','normal_max_fraction']
    with (output/'ranges.csv').open('w',newline='',encoding='utf-8') as dst, source.open('rb') as stream:
        writer=csv.DictWriter(dst,fieldnames=columns);writer.writeheader()
        def flush():
            nonlocal count
            if current is None:return
            for stage in ('indoor_before','outdoor_before','indoor_after','outdoor_after'):
                for b in range(4):
                    suffix=f'{stage}_{b}';r=records['support_'+suffix];n=int(r['matched'])
                    info=records['range_information_sum_'+suffix];normal=records['range_normal_sum_'+suffix]
                    residual=records['range_residual_sum_'+suffix].ravel();e,v=np.linalg.eigh(info)
                    row=dict(window=current[0],iteration=current[1],stage=stage,bin=b,
                             **{k:r[k] for k in ['source','eligible','matched','cost','residual_sq']})
                    if n:
                        row.update(residual_rms=(float(r['residual_sq'])/n)**.5,
                          **dict(zip(['mean_residual_x','mean_residual_y','mean_residual_z'],residual/n)),
                          info_trace_per_match=float(np.trace(info)/n),info_eig_min_per_match=float(e[0]/n),
                          info_eig_max_per_match=float(e[-1]/n),
                          **dict(zip(['info_max_axis_x','info_max_axis_y','info_max_axis_z'],v[:,-1])),
                          normal_max_fraction=float(np.linalg.eigvalsh(normal)[-1]/np.trace(normal)) if np.trace(normal)>0 else '')
                    writer.writerow(row);count+=1
        for line in stream:
            digest.update(line);m=HEADER.fullmatch(line.rstrip(b'\r\n'))
            if not m:raise ValueError('Malformed header')
            w,it=int(m[1]),int(m[2])
            if w not in windows or it<0:continue
            if current!=(w,it):flush();records={};current=(w,it)
            kind=m[3].decode()
            if kind!='range_support' and not kind.startswith(('range_information_sum_','range_normal_sum_','range_residual_sum_')):continue
            f=dict(PAIR.findall(m[4].decode()))
            if kind=='range_support':records['support_'+f['stage']+'_'+f['bin']]=f
            else:
                a=np.fromstring(f['values'][1:-1],sep=',').reshape(int(f['rows']),int(f['cols']))
                if not np.isfinite(a).all():raise ValueError('Nonfinite matrix')
                records[kind]=a
        flush()
    if count!=len(windows)*100*16:raise ValueError('Incomplete selected coverage')
    manifest=dict(source=str(source),source_sha256=digest.hexdigest(),windows=sorted(windows),rows=count,
                  tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  csv_sha256=hashlib.sha256((output/'ranges.csv').read_bytes()).hexdigest())
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8');print(json.dumps(manifest))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trace',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--windows',type=int,nargs='+',required=True);a=p.parse_args();build(a.trace,a.output,set(a.windows))
