"""Validate contact sequence and native linkage closure from recorded states."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdPhysics
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/robotiq_scaled');a=p.parse_args();out=ROOT/a.output
rows=list(csv.DictReader((out/'state.csv').open()));v={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
def end(phase,key):return float(v[key][v['phase']==phase][-1])
config=json.loads((out/'run_config.json').read_text());scene=Usd.Stage.Open(str(ROOT/config['scene']))
with np.load(out/'trajectory.npz') as d:body=d['body_q'];labels=list(d['body_labels']);points=d['points']
worst_anchor=0.;worst_axis=0.
for prim in scene.Traverse():
    if not str(prim.GetPath()).startswith('/World/Gripper/') or not prim.IsA(UsdPhysics.Joint):continue
    centers=[];axes=[]
    for side in (0,1):
        path=str(prim.GetRelationship('physics:body'+str(side)).GetTargets()[0]);idx=labels.index(path);pose=body[:,idx]
        pos=np.array(prim.GetAttribute('physics:localPos'+str(side)).Get())
        quat=prim.GetAttribute('physics:localRot'+str(side)).Get();rot=Rotation.from_quat([*quat.GetImaginary(),quat.GetReal()])
        axis=rot.apply([0,0,1]);r=Rotation.from_quat(pose[:,3:])
        centers.append(pose[:,:3]+r.apply(pos));axes.append(r.apply(axis))
    worst_anchor=max(worst_anchor,float(np.linalg.norm(centers[0]-centers[1],axis=1).max()))
    worst_axis=max(worst_axis,float(np.rad2deg(np.arccos(np.clip(np.sum(axes[0]*axes[1],axis=1),-1,1))).max()))
# Verify physical flange alignment independently of the authored Mount frame.
wrist=body[:,labels.index('/World/UR10/wrist_3_link')]
palm=body[:,labels.index('/World/Gripper/Palm')]
normal=Rotation.from_quat(wrist[:,3:]).apply(np.tile([0,0,1],(len(body),1)))
approach=Rotation.from_quat(palm[:,3:]).apply(np.tile([0,0,1],(len(body),1)))
flange_axis_error=float(np.rad2deg(np.arccos(np.clip(np.sum(normal*approach,axis=1),-1,1))).max())
offset=palm[:,:3]-wrist[:,:3]
flange_lateral_error=float(np.linalg.norm(offset-normal*np.sum(offset*normal,axis=1)[:,None],axis=1).max())
checks={
    'flange_coaxial_within_0_01deg':flange_axis_error<.01,
    'flange_centered_within_0_01mm':flange_lateral_error<.00001,
    'eight_phases':set(v['phase'])==set(range(8)),
    'complete_16s':abs(v['t'][-1]-16)<.02,
    'finite_states':bool(np.isfinite(points).all() and np.isfinite(body).all() and all(np.isfinite(x).all() for x in v.values())),
    'no_pregrasp_plasticity':bool((v['plastic_hinges'][v['phase']<3]==0).all()),
    'bilateral_contact_during_lift':min(end(4,'left_contact_N'),end(4,'right_contact_N'))>1,
    'box_lift_over_5cm':end(4,'min_z_m')-end(2,'min_z_m')>.05,
    'squeeze_closes_over_5mm':end(4,'actual_gap_m')-end(5,'actual_gap_m')>.005,
    'permanent_creases':end(5,'plastic_hinges')>end(4,'plastic_hinges') and end(7,'plastic_hinges')>0,
    'released':max(end(7,'left_contact_N'),end(7,'right_contact_N'))<.01,
    'landed':abs(end(7,'min_z_m')-end(2,'min_z_m'))<.004,
    'settled':end(7,'max_speed_m_s')<.01,
    'all_gripper_joint_anchors_within_2mm':worst_anchor<.002,
    'hinge_axes_within_2deg':worst_axis<2.,
}
checks={k:bool(value) for k,value in checks.items()}
report={'passed':all(checks.values()),'checks':checks,'lift_m':end(4,'min_z_m')-end(2,'min_z_m'),'flange_axis_error_deg':flange_axis_error,'flange_lateral_error_m':flange_lateral_error,'loaded_crush_gap_m':end(5,'actual_gap_m'),'final_plastic_hinges':end(7,'plastic_hinges'),'max_joint_anchor_error_m':worst_anchor,'max_joint_axis_error_deg':worst_axis,'active_wall_s':float(v['active_wall_s'][-1]),'final_speed_m_s':end(7,'max_speed_m_s'),'qualification':'Hypothetical enlarged geometry/density/drives; prescribed UR10 arm, dynamic native gripper loops. Not Robotiq performance or UR10 payload qualification; no half-width compression claim.'}
(out/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
