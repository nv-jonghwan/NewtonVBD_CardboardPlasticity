"""Continuous pre-release grip check; end-of-lift contact alone is insufficient."""
import argparse,csv,json
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from cardboard import ROOT
p=argparse.ArgumentParser(__doc__);p.add_argument('--output',required=True);a=p.parse_args();folder=ROOT/a.output
config=json.loads((folder/'run_config.json').read_text());stage=Usd.Stage.Open(str(ROOT/config['scene']))
z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t'];body=z['body_q'];labels=list(z['body_labels'])
rows=list(csv.DictReader((folder/'state.csv').open()));v={k:np.array([float(r[k]) for r in rows]) for k in rows[0]};phase_ends=np.array(stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:phaseEnds').Get())
start=float(phase_ends[3]);release=float(phase_ends[5]);lift_end=float(phase_ends[4]);ref=int(np.argmin(abs(t-start)));sel=(t>=start-1e-6)&(t<=release+1e-6)
initial_floor=float(v['min_z_m'][v['phase']==2][-1]);loaded=(v['t']>=lift_end-.01)&(v['t']<=release+1e-6)
clearance=(v['min_z_m'][loaded]-initial_floor)*1000
palm=body[:,labels.index('/World/Gripper/Palm')];local=np.einsum('nij,nkj->nki',Rotation.from_quat(palm[:,3:]).as_matrix().transpose(0,2,1),q-palm[:,None,:3]);com=local.mean(1);patches={}
for name in ('Left','Right'):
 b=body[:,labels.index('/World/Gripper/'+name)];rot=Rotation.from_quat(b[:,3:]).as_matrix();loc=np.einsum('nij,nkj->nki',rot.transpose(0,2,1),q-b[:,None,:3])
 prim=stage.GetPrimAtPath('/World/Gripper/'+name);pad=stage.GetPrimAtPath(str(prim.GetPath())+'/Geometry/Fingertip_01');bb=UsdGeom.BBoxCache(0,['default','render']).ComputeRelativeBound(pad,prim).ComputeAlignedRange();lo=np.array(bb.GetMin());hi=np.array(bb.GetMax());c=(lo+hi)/2;c[1]=lo[1] if name=='Left' else hi[1]
 mask=(abs(loc[ref,:,1]-c[1])<.015)&np.all((loc[ref][:,[0,2]]>=lo[[0,2]])&(loc[ref][:,[0,2]]<=hi[[0,2]]),axis=1)
 ids=np.flatnonzero(mask);ids=ids[np.argsort(np.linalg.norm(loc[ref,ids]-c,axis=1))[:20]]
 if len(ids)<3:raise RuntimeError('Too few initial material vertices at '+name+' pad')
 patch=loc[:,ids].mean(1);delta=patch-patch[ref];dist=np.linalg.norm(delta[:,[0,2]],axis=1)
 collider=stage.GetPrimAtPath(str(prim.GetPath())+'/Collider_Fingertip')
 m=np.array(UsdGeom.XformCache().ComputeRelativeTransform(collider,prim)[0]).T
 lengths=np.linalg.norm(m[:3,:3],axis=0)
 unit=(loc-m[:3,3])@np.linalg.inv(m[:3,:3]).T
 d=abs(unit*lengths)-lengths*.5
 signed_distance=np.linalg.norm(np.maximum(d,0),axis=2)+np.minimum(np.max(d,axis=2),0)
 nearby=(signed_distance<.007).sum(1)
 loaded_frames=(t>=start+.2)&(t<=release+1e-6)
 patches[name]={'tracked_vertices':ids.tolist(),'reference_distance_to_pad_center_mm':float(np.linalg.norm(patch[ref]-c)*1000),'max_tangential_patch_drift_mm':float(dist[sel].max()*1000),'max_lift_tangential_patch_drift_mm':float(dist[sel & (t<=lift_end+1e-6)].max()*1000),'minimum_tip_nearby_vertices':int(nearby[loaded_frames].min()),'samples':{str(x):float(dist[np.argmin(abs(t-x))]*1000) for x in (7,8,9,10,11,12)}}
forces=np.minimum(v['left_contact_N'],v['right_contact_N']);holding=(v['t']>=start+.2)&(v['t']<=release+1e-6)
checks={'complete_hold_window':bool(t[-1]>=release-1e-6),'no_table_return_before_release':bool(clearance.min()>20),'bilateral_contact_through_hold':bool((forces[holding]>1).all()),'pad_material_drift_within20mm':all(r['max_tangential_patch_drift_mm']<20 for r in patches.values()),'both_fingertips_geometrically_engaged':all(r['minimum_tip_nearby_vertices']>=3 for r in patches.values())}
result=dict(checks=checks,passed=all(checks.values()),minimum_pre_release_clearance_mm=float(clearance.min()),minimum_bilateral_force_N=float(forces[holding].min()),palm_center_relative_drift_mm=(1000*(com[sel]-com[ref])).max(0).tolist(),patches=patches,qualification='20mm tangent drift/clearance are demo retention limits; tracked material can deform. Report together with full sequence and actual release checks, not as a calibrated friction measurement.')
(folder/'grip_review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
