"""Compare physical fold localization and panel motion at identical resolution."""
import argparse
import csv
import json
import numpy as np
from pxr import Usd, UsdGeom
from cardboard import ROOT
from cardboard.usd_utils import register

p=argparse.ArgumentParser()
p.add_argument('--baseline',default='outputs/robotiq_fine24')
p.add_argument('--candidate',default='outputs/robotiq_creased')
p.add_argument('--thickness-ratio',type=float,default=1.,help='Explicit requested gauge change; check dimensional scaling and preserved exterior')
a=p.parse_args();register()

def measure(folder):
    out=ROOT/folder;config=json.loads((out/'run_config.json').read_text())
    stage=Usd.Stage.Open(str(ROOT/config['scene']));mesh=stage.GetPrimAtPath('/World/Box/SimMesh')
    get=lambda name:np.asarray(mesh.GetAttribute('cardboard:'+name).Get())
    rest=get('restPoints').astype(float);edges=get('hingeIndices');reference=get('referenceAngles');dual=get('dualWidths')
    faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    material=stage.GetPrimAtPath('/World/Box/Materials/Cardboard')
    intact={name:material.GetAttribute('cardboard:'+name).Get() for name in
            ['bendingMD','bendingCD','bendingReferenceThickness','bendingThicknessExponent','membraneShear','membraneArea','thickness','arealDensity']}
    internal=np.abs(reference)<1e-4
    weight=(np.linalg.norm(rest[edges[:,3]]-rest[edges[:,2]],axis=1)*dual)[internal]
    def area(q):
        tri=q[faces];return float(np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1).sum()*.5)
    def folds(q):
        n0=np.cross(q[edges[:,2]]-q[edges[:,0]],q[edges[:,3]]-q[edges[:,0]])
        n1=np.cross(q[edges[:,3]]-q[edges[:,1]],q[edges[:,2]]-q[edges[:,1]])
        n0/=np.maximum(np.linalg.norm(n0,axis=1)[:,None],1e-15)
        n1/=np.maximum(np.linalg.norm(n1,axis=1)[:,None],1e-15)
        tangent=q[edges[:,3]]-q[edges[:,2]];tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-15)
        theta=np.arctan2(np.sum(np.cross(n0,n1)*tangent,axis=1),np.sum(n0*n1,axis=1))
        angle=np.abs(theta-reference)[internal];curvature=angle/dual[internal]
        order=np.argsort(-curvature);energy=weight*curvature**2
        selected=np.cumsum(weight[order])<=.1*weight.sum()
        return dict(low_curvature_dual_area_fraction=float(weight[curvature<5].sum()/weight.sum()),
                    top_tenth_squared_curvature_fraction=float(energy[order][selected].sum()/max(energy.sum(),1e-15)),
                    peak_panel_fold_deg=float(np.rad2deg(angle.max())),
                    panel_edges_over_25deg=int((np.rad2deg(angle)>25).sum()),area_ratio=area(q)/area(rest))
    with np.load(out/'trajectory.npz') as d:
        points=d['points'];times=d['t']
        body=d['body_q'];labels=list(d['body_labels'])
        samples={str(t):folds(points[np.argmin(abs(times-t))].astype(float)) for t in [9.,11.8,16.]}
        same_rest=points[0].copy()
    rows=list(csv.DictReader((out/'state.csv').open()))
    row_times=np.array([float(row['t']) for row in rows])
    phase=np.array([int(float(row['phase'])) for row in rows])[np.clip(np.searchsorted(row_times,times),0,len(rows)-1)]
    rest_height=float([row for row in rows if float(row['phase'])==2][-1]['min_z_m'])
    airborne=(phase==4)&(points[:,:,2].min(axis=1)>rest_height+.015)
    pairs=airborne[1:]&airborne[:-1]
    relative=points.mean(axis=1)-body[:,labels.index('/World/Gripper/Palm'),:3]
    relative_velocity=np.diff(relative,axis=0)/np.diff(times)[:,None]
    lift_rms=float(np.sqrt(np.mean(np.sum(relative_velocity[pairs]**2,axis=1)))) if np.any(pairs) else float('inf')
    flutter=json.loads((out/'panel_flutter.json').read_text())
    recoil=json.loads((out/'held_recoil.json').read_text())
    work=np.array([float(row['plastic_work_J']) for row in rows])
    return dict(samples=samples,panel_flutter=flutter,held_recoil=recoil,airborne_box_relative_speed_rms_m_s=lift_rms,
                final_instantaneous_speed_m_s=float(rows[-1]['max_speed_m_s']),
                accumulated_work_monotone=bool(np.all(np.diff(work)>=-1e-5))),intact,rest,faces,same_rest

baseline,bi,br,bf,bq=measure(a.baseline)
candidate,ci,cr,cf,cq=measure(a.candidate)
out=ROOT/a.candidate
sequence=json.loads((out/'validation.json').read_text())
bs=baseline['samples']['16.0'];cs=candidate['samples']['16.0']
ratio=a.thickness_ratio
if not 0<ratio<=1:raise ValueError('thickness-ratio must be in (0,1]')
expected={k:v*(ratio if k in ['membraneShear','membraneArea','thickness','arealDensity'] else 1.) for k,v in bi.items()}
center=(br.min(0)+br.max(0))*.5;half=np.ptp(br,axis=0)*.5
expected_rest=center+(br-center)*(half+(bi['thickness']-ci['thickness'])*.5)/half
checks=dict(full_contact_and_settling_sequence=sequence['passed'],
            same_intact_stiffness_thickness_mass=bi==ci,
            same_physical_mesh=bool(np.array_equal(br,cr) and np.array_equal(bf,cf) and np.allclose(bq,cq,atol=1e-7)),
            more_low_curvature_panel_area=cs['low_curvature_dual_area_fraction']>bs['low_curvature_dual_area_fraction']+.03,
            more_concentrated_bending=cs['top_tenth_squared_curvature_fraction']>bs['top_tenth_squared_curvature_fraction']+.01,
            permanent_sharp_fold=cs['peak_panel_fold_deg']>60.,
            surface_area_preserved=abs(cs['area_ratio']-1)<.02,
            held_panel_motion_reduced=candidate['panel_flutter']['held']['mean_m_s']<.8*baseline['panel_flutter']['held']['mean_m_s'],
            held_jaw_recoil_reduced=candidate['held_recoil']['jaw_reopening_path_m']<.8*baseline['held_recoil']['jaw_reopening_path_m'],
            held_panel_recoil_reduced=candidate['held_recoil']['panel_dominant_motion_backtrack_mean_m']<.8*baseline['held_recoil']['panel_dominant_motion_backtrack_mean_m'],
            late_panel_motion_reduced=candidate['panel_flutter']['late']['mean_m_s']<.8*baseline['panel_flutter']['late']['mean_m_s'],
            nonnegative_accumulating_dissipation=candidate['accumulated_work_monotone'])
if ratio!=1.:
    # An explicit user gauge change replaces only the two equality checks;
    # every contact, crease, motion, area and history quality threshold remains.
    del checks['same_intact_stiffness_thickness_mass'];del checks['same_physical_mesh']
    checks['requested_thickness_and_constitutive_scaling']=all(np.isclose(ci[k],v,rtol=1e-6) for k,v in expected.items())
    checks['same_mesh_resolution_and_preserved_outer_envelope']=bool(np.array_equal(bf,cf) and cr.shape==br.shape and np.allclose(cr,expected_rest,atol=1e-7) and np.allclose(cq-bq,cr-br,atol=1e-7))
with np.load(out/'final_material_state.npz') as d:
    checks['valid_terminal_material_history']=bool(np.isfinite(d['damage']).all() and (d['damage']>=0).all() and
        (d['damage']<=.82001).all() and (d['plastic_work']>=0).all() and
        (d['accumulated_angle']>=np.abs(d['plastic_angle'])-1e-5).all())
    candidate['damage_percentiles']=np.percentile(d['damage'],[0,25,50,75,95,100]).tolist()
report=dict(passed=all(checks.values()),checks=checks,baseline=baseline,candidate=candidate,
            requested_thickness_ratio=ratio,
            method='Same physical grid; factory seams excluded. Low curvature <5/m; top10% by rest hinge dual area. Squared curvature is a geometric concentration metric, not actual damaged-material energy. Panel motion from30Hz rigid-fit residuals includes creep and cannot resolve subframe vibration. Instantaneous speed retains the original solver gate.')
(out/'crease_comparison.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
