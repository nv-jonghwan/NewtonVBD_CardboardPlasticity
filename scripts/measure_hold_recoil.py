"""Separate continued closure from direction reversals during a held squeeze.

This is a diagnostic of raw physics records; it never filters the displayed
trajectory. Panel PCA measures backtracking along the dominant local motion.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('directories',nargs='+');a=p.parse_args()
for name in a.directories:
    out=Path(name)
    rows=list(csv.DictReader((out/'state.csv').open()))
    times=np.array([float(r['t']) for r in rows]);gap=np.array([float(r['actual_gap_m']) for r in rows])
    select=(times>=11.2)&(times<12.);g=gap[select];delta=np.diff(g)
    with np.load(out/'trajectory.npz') as data:
        points=data['points'].astype(float);t=data['t'];rest=points[0]-points[0].mean(0)
    half=np.max(abs(rest),axis=0)
    panels=[np.flatnonzero((abs(rest[:,axis]-sign*half[axis])<1e-4)&np.all(abs(np.delete(rest,axis,axis=1))<np.delete(half,axis)*.9,axis=1)) for axis in range(3) for sign in [-1,1]]
    q=points[(t>=11.2)&(t<12.)];backtrack=[];net=[]
    for panel in panels:
        x=q[0,panel]-q[0,panel].mean(0);aligned=[]
        for frame in q:
            y=frame[panel]-frame[panel].mean(0);u,_,vt=np.linalg.svd(y.T@x)
            if np.linalg.det(u@vt)<0:u[:,-1]*=-1
            aligned.append(y@(u@vt))
        r=np.array(aligned);r-=r.mean(0)
        covariance=np.einsum('tni,tnj->nij',r,r)
        _,vectors=np.linalg.eigh(covariance);axis=vectors[:,:,-1]
        z=np.einsum('tni,ni->tn',r,axis)
        z*=np.where(z[-1]-z[0]>=0,1.,-1.)
        backtrack.extend(np.maximum(-np.diff(z,axis=0),0).sum(0));net.extend(abs(z[-1]-z[0]))
    report=dict(interval_s=[11.2,12.],jaw_net_closure_m=float(g[0]-g[-1]),
                jaw_reopening_path_m=float(np.maximum(delta,0).sum()),
                panel_dominant_motion_backtrack_mean_m=float(np.mean(backtrack)),
                panel_dominant_motion_backtrack_p95_m=float(np.percentile(backtrack,95)),
                panel_net_motion_mean_m=float(np.mean(net)),
                method='Jaw reversal path from60Hz actual gap; panel dominant-motion reversal path from30Hz physical positions after per-panel rigid alignment. Excludes monotone drift along each principal axis, but not a frequency-resolved modal analysis; does not replace total-motion or instantaneous-speed metrics.')
    (out/'held_recoil.json').write_text(json.dumps(report,indent=2));print(name,json.dumps(report))
