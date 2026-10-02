"""Measure material stretch and internal motion without global velocity damping."""
import argparse,csv,json,time
from pathlib import Path
import numpy as np
from cardboard.scenario import PickCrushDrop
p=argparse.ArgumentParser();p.add_argument('--profile',choices=['baseline','paper','paper_gentle','damping_only','panel_crease','plate','plate_crease','authored'],required=True);p.add_argument('--output',required=True);p.add_argument('--scene');p.add_argument('--iterations',type=int);args=p.parse_args()
material=dict(membraneShear=12600.,membraneArea=21000.,membraneDamping=.0002,bendingMD=.042,bendingCD=.02016,bendingDamping=.0002,yieldCurvature=12.,hardeningRatio=.02,damageRate=.35,residualStiffness=.25)
if args.profile=='paper':material.update(membraneShear=18900.,membraneArea=31500.,membraneDamping=.2,bendingDamping=.01)
if args.profile=='paper_gentle':material.update(membraneShear=13860.,membraneArea=23100.,membraneDamping=.02,bendingDamping=.002)
if args.profile=='damping_only':material.update(membraneDamping=.005,bendingDamping=.0006)
if args.profile=='panel_crease':material.update(membraneShear=17640.,membraneArea=29400.,membraneDamping=.02,bendingDamping=.0006,yieldCurvature=8.4,hardeningRatio=.01,damageRate=.5)
if args.profile in ['plate','plate_crease']:material.update(membraneShear=17640.,membraneArea=29400.,membraneDamping=.02,bendingMD=.2,bendingCD=.1,bendingDamping=.003,yieldCurvature=12.,hardeningRatio=.02,damageRate=.15,residualStiffness=.65)
if args.profile=='plate_crease':material['yieldCurvature']=5.
if args.profile=='authored':material={}
out=Path(args.output);out.mkdir(parents=True,exist_ok=True);(out/'parameters.json').write_text(json.dumps(material,indent=2))
s=PickCrushDrop(material_overrides=material,scene_path=args.scene,iterations=args.iterations)
if args.profile=='authored':
 material={str(name).split(':',1)[1]:s.mat.GetAttribute(name).Get() for name in s.mat.GetPropertyNames() if name.startswith('cardboard:') and s.mat.GetAttribute(name)}
 (out/'parameters.json').write_text(json.dumps(material,indent=2))
(out/'run_config.json').write_text(json.dumps({'newton':s.newton_version,'scene':args.scene or 'assets/demo_scene.usda','iterations':s.iterations,'substeps':s.substeps,'fps':s.fps,'gravityRampSeconds':s.gravity_ramp,'scenario':s.config},indent=2,default=list))
f=s.faces;r=s.local.astype(float);e1=r[f[:,1]]-r[f[:,0]];e2=r[f[:,2]]-r[f[:,0]]
a=np.linalg.norm(e1,axis=1);b=(e1*e2).sum(1)/a;c=np.sqrt((e2*e2).sum(1)-b*b)
dm=np.zeros((len(f),2,2));dm[:,0,0]=a;dm[:,0,1]=b;dm[:,1,1]=c;inv=np.linalg.inv(dm)
# Fit away the rigid motion of each broad panel, so rotating intact faces
# are not mistaken for thin-sheet flutter. Exclude the one-cell box seams.
half=np.max(np.abs(r),axis=0)
panels=[np.flatnonzero((np.abs(r[:,axis]-sign*half[axis])<1e-6)&np.all(np.abs(np.delete(r,axis,axis=1))<np.delete(half,axis)*.9,axis=1)) for axis in range(3) for sign in [-1,1]]
rows=[];samples=[];bodies=[];times=[];last=-1;started=time.monotonic()
for i in range(round(s.duration*s.fps)):
 row=s.advance();q=s.a.particle_q.numpy().astype(float);v=s.a.particle_qd.numpy().astype(float)
 x=q-q.mean(0);vel=v-v.mean(0)
 inertia=np.eye(3)*(x*x).sum()-x.T@x
 omega=np.linalg.solve(inertia,np.cross(x,vel).sum(0))
 internal=vel-np.cross(omega,x)
 ds=np.stack([q[f[:,1]]-q[f[:,0]],q[f[:,2]]-q[f[:,0]]],axis=-1)
 strain=np.linalg.svd(ds@inv,compute_uv=False)-1
 row.update(internal_speed_rms_m_s=float(np.sqrt(np.mean(np.sum(internal*internal,axis=1)))),membrane_strain_rms=float(np.sqrt(np.mean(strain*strain))),membrane_strain_p95=float(np.percentile(abs(strain),95)))
 panel_speeds=[]
 for panel in panels:
  xp=q[panel]-q[panel].mean(0);vp=v[panel]-v[panel].mean(0)
  ip=np.eye(3)*(xp*xp).sum()-xp.T@xp
  om=np.linalg.solve(ip,np.cross(xp,vp).sum(0))
  panel_speeds.extend(np.sum((vp-np.cross(om,xp))**2,axis=1))
 row['panel_deformation_speed_rms_m_s']=float(np.sqrt(np.mean(panel_speeds)))
 rows.append(row)
 body=s.a.body_q.numpy();status=dict(t=s.time,phase=s.phase,playing=True,done=s.time>=s.duration-1e-6,duration=s.duration)
 with (out/'state.tmp').open('wb') as file:np.savez(file,points=q.astype(np.float32),body_q=body,t=s.time,plastic_hinges=row['plastic_hinges'],status=json.dumps(status))
 (out/'state.tmp').replace(out/'state.npz')
 if s.phase!=last:print(args.profile,'phase',s.phase,'t',round(s.time,2),flush=True);last=s.phase
 if i%2==1: samples.append(q.astype(np.float32));bodies.append(body);times.append(s.time)
 if s.phase>=4:(out/f'phase-{s.phase:02d}.npz').write_bytes((out/'state.npz').read_bytes())
with (out/'state.csv').open('w') as file:
 w=csv.DictWriter(file,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
np.savez_compressed(out/'trajectory.npz',points=np.array(samples),body_q=np.array(bodies),t=np.array(times),faces=f)
summary={'profile':args.profile,'wall_time_s':time.monotonic()-started}
for name,lo,hi in [('held',11.2,12),('settling',15,16)]:
 subset=[r for r in rows if lo<=r['t']<=hi+.001]
 summary[name]={k:float(np.mean([r[k] for r in subset])) for k in ['internal_speed_rms_m_s','panel_deformation_speed_rms_m_s','membrane_strain_rms','membrane_strain_p95']}
summary['pre_release']=[r for r in rows if r['phase']==5][-1];summary['final']=rows[-1]
(out/'panel_metrics.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
