"""Build a standalone, offline timeline from retained diagnostic artifacts. Never runs SLAM."""
import argparse, csv, hashlib, json
from pathlib import Path
import numpy as np

def build(root, output):
    rows=json.loads((root/'jump-diagnosis/all-windows.json').read_text())
    assert rows and all(b['time_s']>a['time_s'] for a,b in zip(rows,rows[1:]))
    ranges=list(csv.DictReader((root/'cloud-context/raw_ranges_by_second.csv').open()))
    for r in ranges:
        for k in r:r[k]=float(r[k])
    samples={}
    for key,name in [('raw','raw_sensor_frame_sample.npy'),('world','final_world_sample.npy')]:
        a=np.load(root/'cloud-context'/name)
        # Deterministic display sample; original extracts remain untouched.
        a=a[::max(1,len(a)//70000)]
        assert a.shape[1]==4 and np.isfinite(a).all()
        samples[key]=np.round(a,4).tolist()
    payload=json.dumps(dict(rows=rows,ranges=ranges,samples=samples),allow_nan=False,separators=(',',':'))
    html=TEMPLATE.replace('__DATA__',payload)
    output.mkdir(exist_ok=False)
    # Hash exactly the bytes written, including on Windows (no CRLF conversion).
    html_bytes=html.encode('utf-8')
    (output/'index.html').write_bytes(html_bytes)
    inputs=[root/'jump-diagnosis/all-windows.json',root/'cloud-context/raw_ranges_by_second.csv',
            root/'cloud-context/raw_sensor_frame_sample.npy',root/'cloud-context/final_world_sample.npy',Path(__file__)]
    manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    manifest['index.html']=hashlib.sha256(html_bytes).hexdigest()
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(output/'index.html')

TEMPLATE=r'''<!doctype html><html><meta charset="utf-8"><title>Livox2 flyaway timeline</title>
<style>body{background:#101923;color:#e8eff6;font:15px system-ui;margin:24px}h1{font-size:25px}button,select{padding:8px;background:#25384b;color:white;border:1px solid #668;margin:4px;cursor:pointer}input{width:98%}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}canvas{width:100%;height:210px;background:#172635;border-radius:6px}section{padding:12px;background:#17212d;border-radius:8px}p{line-height:1.5}.muted{color:#afc2d4}a{color:#75d9ff}#status{white-space:pre-wrap}label{display:block} .scene{height:320px}</style>
<h1>Livox2 — synchronized flyaway timeline</h1>
<p class="muted">Retained run only. Estimated motion is not ground truth. No replay or solver changes. Click any chart, drag the slider, or step through windows.</p>
<button id="prev">Previous window</button><button id="play">Play</button><button id="next">Next window</button>
<button data-time="268.56">First disruption</button><button data-time="271.08">Speed growth</button><button data-time="312.84">Second escalation</button>
<input id="seek" type="range" min="0" step="1"><section id="status"></section>
<div class="grid" id="charts"></div>
<p>Scene window: ±1 second. Display is subsampled. Cloud coverage exists only for 240–285s and 300–320s; blank views outside these intervals mean NOT MEASURED.</p>
<label>Scene projection <select id="plane"><option value="12">XY</option><option value="13">XZ</option><option value="23">YZ</option></select></label>
<div class="grid"><section><b>Raw cloud — sensor coordinates, no SLAM pose or deskew</b><canvas class="scene" id="raw"></canvas></section><section><b>Mapped cloud — final world coordinates, inherits trajectory errors</b><canvas class="scene" id="world"></canvas></section></div>
<p>Chart motion times label window starts; endpoint velocity belongs near the window end. Range quantiles have 0.25m bins. Counts concern recorded returns, not emitted-pulse coverage. Detailed factor tracing: NOT MEASURED in this retained run.</p>
<p><a href="../cloud-context/final_context_240_285s.laz">First mapped extract</a> · <a href="../cloud-context/final_context_300_320s.laz">Second mapped extract</a> · <a href="../cloud-context/raw_sensor_frame_266_266.2s.laz">Raw 266s snapshot</a></p>
<script>const D=__DATA__, rows=D.rows, seek=document.getElementById('seek');let index=0,timer=null;
seek.max=rows.length-1;
const configs=[['Estimated speed [m/s]',[['end_speed','#ffbb55'],['start_step_speed','#62d6ff']]],['Within-window displacement [m]',[['guess_move_norm','#ffbb55'],['result_move_norm','#62d6ff'],['result_guess_norm','#dd80ff']]],['Yaw correction [degrees]',[['yaw_correction','#62d6ff']]],['Endpoint velocity sigma [m/s]',[['end_sigma','#dd80ff']]],['Geometric eigenvalue ratio', [['ratio','#76e5a3']]],['Raw return range [m]',[['range_p50_lower_bin_m','#ffbb55'],['range_p90_lower_bin_m','#62d6ff']]]];
const charts=configs.map((cfg,i)=>{let s=document.createElement('section');s.innerHTML='<b>'+cfg[0]+'</b><p class="muted">'+cfg[1].map(x=>'<span style="color:'+x[1]+'">'+x[0]+'</span>').join(' · ')+'</p>';let c=document.createElement('canvas');s.append(c);document.getElementById('charts').append(s);c.onclick=e=>{let q=(e.clientX-c.getBoundingClientRect().left)/c.clientWidth;selectTime(Math.max(0,Math.min(1,(q*900-55)/825))*rows.at(-1).time_s)};return c});
function selectTime(t){let best=0;for(let i=1;i<rows.length;i++)if(Math.abs(rows[i].time_s-t)<Math.abs(rows[best].time_s-t))best=i;index=best;draw()}
function canvas(c,w=900,h=240){c.width=w;c.height=h;let g=c.getContext('2d');g.fillStyle='#172635';g.fillRect(0,0,w,h);g.font='13px system-ui';return g}
function chart(c,cfg,n){let g=canvas(c),data=n===5?D.ranges:rows,key=n===5?'second':'time_s';let values=data.flatMap(r=>cfg[1].map(f=>r[f[0]])).filter(Number.isFinite),lo=Math.min(0,...values),hi=Math.max(...values);if(hi===lo)hi=lo+1;
let x=t=>55+825*t/rows.at(-1).time_s,y=v=>205-175*(v-lo)/(hi-lo);g.strokeStyle='#526273';g.strokeRect(55,30,825,175);g.fillStyle='#ccd9e4';g.fillText(hi.toPrecision(3),4,36);g.fillText(lo.toPrecision(3),4,205);g.fillText('0 s',55,225);g.fillText(rows.at(-1).time_s.toFixed(1)+' s',820,225);
for(let [field,color] of cfg[1]){g.strokeStyle=color;g.beginPath();let active=false;for(let r of data){if(!Number.isFinite(r[field])){active=false;continue}if(active)g.lineTo(x(r[key]),y(r[field]));else g.moveTo(x(r[key]),y(r[field]));active=true}g.stroke()}g.strokeStyle='white';g.beginPath();g.moveTo(x(rows[index].time_s),30);g.lineTo(x(rows[index].time_s),205);g.stroke()}
function scene(name){let c=document.getElementById(name),g=canvas(c,700,360),t=rows[index].time_s,pts=D.samples[name].filter(p=>Math.abs(p[0]-t)<=1);g.fillStyle='#ccd9e4';if(!pts.length){g.fillText('NOT MEASURED at this time',190,180);return}let axes=document.getElementById('plane').value.split('').map(Number),center=axes.map(k=>name==='raw'?0:pts.reduce((s,p)=>s+p[k],0)/pts.length),extent=name==='raw'?6:20;
g.strokeStyle='#526273';g.strokeRect(190,20,320,320);g.fillText('±'+extent+' m; '+pts.length+' displayed returns',190,355);for(let p of pts){let a=(p[axes[0]]-center[0])/extent,b=(p[axes[1]]-center[1])/extent;if(Math.abs(a)>1||Math.abs(b)>1)continue;g.fillStyle=p[0]<t?'#62d6ff':'#ffbb55';g.fillRect(350+160*a,180-160*b,2,2)}g.fillStyle='#ff6666';if(name==='raw')g.fillText('+ sensor',350,180);else g.fillText('View centred on selected points; world axes',170,15)}
function draw(){seek.value=index;let r=rows[index],raw=D.ranges.find(x=>x.second===Math.floor(r.time_s));let flags=[];if(r.end_speed>2.5)flags.push('Speed diagnostic >2.5 m/s');if(Math.abs(r.yaw_correction)>3)flags.push('Heading correction >3°');if(!Number.isFinite(r.end_speed))flags.push('Speed NOT MEASURED');document.getElementById('status').textContent='Window '+r.window+' | start '+r.time_s.toFixed(2)+'s | end approximately '+(r.time_s+r.dt_s).toFixed(2)+'s\nEndpoint estimate '+r.end_speed.toFixed(3)+' m/s; sigma '+r.end_sigma.toFixed(5)+' m/s; change in displacement '+r.result_guess_norm.toFixed(3)+' m\n'+(flags.join(' · ')||'No selected trigger; this does not certify accuracy')+'\n'+(raw?'Recorded returns within 5m: '+(100*raw.under5m_share).toFixed(2)+'%; returns: '+raw.valid_returns:'Raw range coverage: NOT MEASURED');charts.forEach((c,i)=>chart(c,configs[i],i));scene('raw');scene('world')}
seek.oninput=()=>{index=+seek.value;draw()};document.getElementById('plane').onchange=draw;document.getElementById('prev').onclick=()=>{index=Math.max(0,index-1);draw()};document.getElementById('next').onclick=()=>{index=Math.min(rows.length-1,index+1);draw()};document.querySelectorAll('[data-time]').forEach(b=>b.onclick=()=>selectTime(+b.dataset.time));document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null}else timer=setInterval(()=>{index=(index+1)%rows.length;draw()},250);document.getElementById('play').textContent=timer?'Pause':'Play'};selectTime(268.56);
</script></html>'''

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('experiment',type=Path);ap.add_argument('output',type=Path)
    a=ap.parse_args();build(a.experiment,a.output)
