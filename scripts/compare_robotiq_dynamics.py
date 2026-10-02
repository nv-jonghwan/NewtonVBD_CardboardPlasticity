"""Compare actual contact squeeze and airborne motion; no command-only gates."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--baseline',default='outputs/robotiq_mount_fixed');p.add_argument('--candidate',required=True);a=p.parse_args()
def measure(path):
    out=ROOT/path
    rows=list(csv.DictReader((out/'state.csv').open()))
    v={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    with np.load(out/'trajectory.npz') as data:
        t=data['t'];points=data['points'];body=data['body_q'];labels=list(data['body_labels'])
    phase=v['phase'][np.clip(np.searchsorted(v['t'],t),0,len(v['t'])-1)]
    palm=body[:,labels.index('/World/Gripper/Palm')]
    # Keep only airborne lift frames, excluding the expected transition off table.
    rest=float(v['min_z_m'][v['phase']==2][-1])
    airborne=(phase==4)&(points[:,:,2].min(axis=1)>rest+.015)
    pairs=airborne[1:]&airborne[:-1]
    relative=points.mean(axis=1)-palm[:,:3]
    velocity=np.diff(relative,axis=0)/np.diff(t)[:,None]
    angular=[];tilt=[]
    for label in ['/World/Gripper/Left','/World/Gripper/Right']:
        r=Rotation.from_quat(palm[:,3:]).inv()*Rotation.from_quat(body[:,labels.index(label),3:])
        omega=(r[1:]*r[:-1].inv()).magnitude()/np.diff(t)
        angular.extend(omega[pairs])
        mean=r[airborne].mean();tilt.extend((mean.inv()*r[airborne]).magnitude())
    def end(phase,key):return float(v[key][v['phase']==phase][-1])
    metrics=dict(airborne_lift_samples=int(airborne.sum()),
        airborne_box_relative_speed_rms_m_s=float(np.sqrt(np.mean(np.sum(velocity[pairs]**2,axis=1)))),
        airborne_pad_angular_speed_rms_deg_s=float(np.rad2deg(np.sqrt(np.mean(np.array(angular)**2)))),
        airborne_pad_tilt_peak_deg=float(np.rad2deg(max(tilt))),
        lift_peak_box_speed_m_s=float(v['max_speed_m_s'][v['phase']==4].max()),
        lift_m=end(4,'min_z_m')-rest,
        loaded_crush_gap_m=end(5,'actual_gap_m'),
        squeeze_closure_m=end(4,'actual_gap_m')-end(5,'actual_gap_m'),
        squeeze_plastic_work_J=end(5,'plastic_work_J')-end(4,'plastic_work_J'),
        crush_max_plastic_angle_deg=float(np.rad2deg(end(5,'max_plastic_angle_rad'))),
        final_plastic_hinges=end(7,'plastic_hinges'))
    return metrics,v
baseline,bv=measure(a.baseline);candidate,cv=measure(a.candidate)
checks={
 'enough_airborne_samples':min(baseline['airborne_lift_samples'],candidate['airborne_lift_samples'])>=10,
 'pad_angular_oscillation_at_least_halved':candidate['airborne_pad_angular_speed_rms_deg_s']<.5*baseline['airborne_pad_angular_speed_rms_deg_s'],
 'relative_box_motion_at_least_halved':candidate['airborne_box_relative_speed_rms_m_s']<.5*baseline['airborne_box_relative_speed_rms_m_s'],
 'lift_peak_speed_reduced':candidate['lift_peak_box_speed_m_s']<.7*baseline['lift_peak_box_speed_m_s'],
 'loaded_gap_under_210mm':candidate['loaded_crush_gap_m']<.21,
 'additional_closure_over_40mm':baseline['loaded_crush_gap_m']-candidate['loaded_crush_gap_m']>.04,
 'crush_adds_over_50mm_closure':candidate['squeeze_closure_m']>.05,
 'squeeze_plastic_work_at_least_doubled':candidate['squeeze_plastic_work_J']>2*baseline['squeeze_plastic_work_J'],
}
# The imported box mesh and constitutive parameters must remain identical.
def material(path):
    config=json.loads((ROOT/path/'run_config.json').read_text());stage=Usd.Stage.Open(str(ROOT/config['scene']))
    mesh=stage.GetPrimAtPath('/World/Box/SimMesh');mat=stage.GetPrimAtPath(mesh.GetRelationship('cardboard:material').GetTargets()[0])
    return {str(p.GetPath())+':'+attr.GetName():str(attr.Get()) for p in [mesh,mat] for attr in p.GetAttributes()}
checks['same_box_mesh_and_material']=material(a.baseline)==material(a.candidate)
checks={k:bool(v) for k,v in checks.items()}
report=dict(passed=all(checks.values()),checks=checks,baseline=baseline,candidate=candidate,measurement='Airborne lift with box bottom >15mm off table; pad angular velocity relative to palm; box mean-position velocity relative to palm. No filtered display or position pinning.')
(ROOT/a.candidate/'dynamics_comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
