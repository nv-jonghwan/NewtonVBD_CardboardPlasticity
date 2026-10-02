"""Compare sag, grasp/lift and accepted crush response of a recorded candidate."""
import argparse
import csv
import json
import numpy as np
from cardboard import ROOT

p=argparse.ArgumentParser(__doc__);p.add_argument('directory');a=p.parse_args()
out=ROOT/a.directory
reference=np.load(ROOT/'outputs/early_top_fold/canonical/diagnostic.npz')
def measure(folder):
    z=np.load(folder/'trajectory.npz')
    rows=list(csv.DictReader((folder/'state.csv').open()))
    sag=[]
    for q in z['points']:
        c=q[reference['rim']].mean(0)
        n=np.linalg.svd(q[reference['rim']]-c)[2][-1];n*=1 if n[2]>=0 else -1
        sag.append(float((c-q[reference['center']].mean(0))@n*1000))
    samples={}
    for t in [1,2,5,6,7,8,9,10,11,12,13,16]:
        if t>z['t'][-1]+1e-6:continue
        row=min(rows,key=lambda r:abs(float(r['t'])-t))
        samples[str(t)]={k:float(v) for k,v in row.items()}
        samples[str(t)]['top_sag_mm']=sag[np.argmin(abs(z['t']-t))]
    return dict(samples=samples,t=z['t'].tolist(),sag_mm=sag,
                finite_geometry=bool(np.isfinite(z['points']).all()),
                performance=json.loads((folder/'performance.json').read_text())['wall_time_s'])
result=measure(out)
result['baseline']=measure(ROOT/'outputs/vbd_optimized/guarded_b')
(out/'tuning_metrics.json').write_text(json.dumps(result,indent=2))
for t,r in result['samples'].items():
    print(t,{k:round(r[k],4) for k in ['top_sag_mm','actual_gap_m','plastic_hinges',
           'plastic_work_J','max_plastic_angle_rad','min_z_m','max_speed_m_s']})
print('wall seconds',result['performance'])
