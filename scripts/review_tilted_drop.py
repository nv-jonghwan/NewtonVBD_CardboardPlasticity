"""Measure whole-box orientation and which original panels contact the table."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from cardboard import ROOT
p=argparse.ArgumentParser(__doc__);p.add_argument('--output',required=True);a=p.parse_args();folder=ROOT/a.output
z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t'];b=z['body_q'];labels=list(z['body_labels']);rest=q[0]-q[0].mean(0)
low=rest.min(0);high=rest.max(0);masks={}
for axis,name in enumerate('xyz'):
 for sign,limit in (('-',low[axis]),('+',high[axis])):masks[name+sign]=np.isclose(rest[:,axis],limit,atol=1e-5)
rows=list(csv.DictReader((folder/'state.csv').open()));samples={}
for time in (0,5,7,9,12,12.5,13,14,15,16):
 i=int(np.argmin(abs(t-time)));pts=q[i];centered=pts-pts.mean(0)
 u,_,vt=np.linalg.svd(rest.T@centered);d=np.eye(3);d[-1,-1]=np.linalg.det(u@vt);rotation=u@d@vt
 up=np.array([0.,0.,1.])@rotation;tilt=float(np.degrees(np.arccos(np.clip(up[2],-1,1))))
 palm=Rotation.from_quat(b[i,labels.index('/World/Gripper/Palm'),3:]).as_matrix()
 contact=pts[:,2]<pts[:,2].min()+.003
 panels={name:dict(centroid_z_m=float(pts[mask,2].mean()),near_lowest_count=int(np.count_nonzero(contact&mask))) for name,mask in masks.items()}
 row=min(rows,key=lambda r:abs(float(r['t'])-t[i]))
 samples[str(time)]=dict(t=float(t[i]),box_best_fit_tilt_deg=tilt,box_best_fit_up=up.tolist(),palm_tilt_deg=float(np.degrees(np.arccos(np.clip(-palm[2,2],-1,1)))),panels=panels,max_speed_mm_s=float(row['max_speed_m_s'])*1000)
final=samples['16'];side=min(('x-','x+','y-','y+'),key=lambda name:final['panels'][name]['centroid_z_m'])
checks={'orientation_over_60deg':final['box_best_fit_tilt_deg']>60,'side_centroid_below_original_bottom':final['panels'][side]['centroid_z_m']<final['panels']['z-']['centroid_z_m'],'side_has_near_lowest_vertices':final['panels'][side]['near_lowest_count']>0,'side_centroid_below_original_top':final['panels'][side]['centroid_z_m']<final['panels']['z+']['centroid_z_m']}
result=dict(samples=samples,lowest_side_panel=side,side_topple_checks=checks,side_topple_observed=all(checks.values()),qualification='Best-fit whole-mesh orientation plus original-panel support; deformation means this is not a rigid-body pose or a settling acceptance.')
(folder/'tilt_review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
