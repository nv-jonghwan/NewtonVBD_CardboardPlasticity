"""Compare frame-to-frame panel deformation after fitting away each panel's rigid motion."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+');a=p.parse_args()
for name in a.directories:
 out=Path(name)
 with np.load(out/'trajectory.npz') as d:q=d['points'].astype(float);t=d['t']
 r=q[0]-q[0].mean(0);half=np.max(abs(r),axis=0)
 panels=[np.flatnonzero((abs(r[:,axis]-sign*half[axis])<1e-4)&np.all(abs(np.delete(r,axis,axis=1))<np.delete(half,axis)*.9,axis=1)) for axis in range(3) for sign in [-1,1]]
 if min(map(len,panels))<20:raise ValueError('Cannot identify broad panels')
 speeds=[]
 for i in range(1,len(q)):
  e=[]
  for panel in panels:
   x=q[i-1,panel]-q[i-1,panel].mean(0);y=q[i,panel]-q[i,panel].mean(0)
   u,_,vt=np.linalg.svd(x.T@y);rot=u@vt
   if np.linalg.det(rot)<0:u[:,-1]*=-1;rot=u@vt
   e.extend(np.sum((y-x@rot)**2,axis=1))
  speeds.append(np.sqrt(np.mean(e))/(t[i]-t[i-1]))
 speeds=np.array(speeds);times=t[1:];report={'sampling_hz':float(1/np.median(np.diff(t))),'scope':'Per-panel deformation speed after rigid fit; finite differences include creep and do not resolve above Nyquist.'}
 for label,lo,hi in [('lift',7.8,9),('held',11.2,12),('release_and_impact',12,13),('after_impact',13,14),('late',15,16)]:
  v=speeds[(times>=lo)&(times<=hi+.001)];report[label]={'mean_m_s':float(v.mean()),'peak_m_s':float(v.max())}
 (out/'panel_flutter.json').write_text(json.dumps(report,indent=2));print(name,json.dumps(report))
