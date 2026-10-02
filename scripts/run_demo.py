import argparse,csv,json,time
from pathlib import Path
import numpy as np
from cardboard import ROOT
from cardboard.simulation import Simulation
from cardboard.recording import Recorder
p=argparse.ArgumentParser();p.add_argument('--device',default='cuda:1');p.add_argument('--duration',type=float);p.add_argument('--output',default='outputs/plastic');p.add_argument('--elastic',action='store_true');p.add_argument('--iterations',type=int);p.add_argument('--substeps',type=int);p.add_argument('--viewer',action='store_true');p.add_argument('--no-usd',action='store_true');p.add_argument('--checkpoint');p.add_argument('--yield-curvature',type=float);p.add_argument('--force-limit',type=float);p.add_argument('--bend-scale',type=float,default=1);args=p.parse_args()
overrides={}
if args.yield_curvature is not None:overrides['yieldCurvature']=args.yield_curvature
if args.bend_scale!=1:overrides.update(bendingMD=.04*args.bend_scale,bendingCD=.0192*args.bend_scale)
sim=Simulation(args.device,not args.elastic,args.iterations,args.substeps,visual=args.viewer,material_overrides=overrides,force_limit=args.force_limit)
duration=args.duration if args.duration is not None else sim.attr(sim.settings,'duration')
if args.checkpoint:sim.load_box_checkpoint(ROOT/args.checkpoint)
out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
record=Recorder(sim,out) if not args.no_usd else None
if record:record.capture()
viewer=None
if args.viewer:
    import newton
    viewer=newton.viewer.ViewerGL(width=1280,height=800);viewer.set_model(sim.model)
start=time.perf_counter()
for i in range(round(duration*sim.fps)):
    row=sim.advance()
    if record and i%2==1:record.capture()
    if viewer:
        viewer.begin_frame(sim.time);viewer.log_state(sim.a);viewer.end_frame()
        if not viewer.is_running():break
    if i%30==0:print(json.dumps(row),flush=True)
if record:record.finish()
with (out/'metrics.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=sim.rows[0]);w.writeheader();w.writerows(sim.rows)
q=sim.a.particle_q.numpy();centered=q-q.mean(0);reference=sim.initial-sim.initial.mean(0)
u,_,vt=np.linalg.svd(centered.T@reference);r=u@vt
if np.linalg.det(r)<0:u[:,-1]*=-1;r=u@vt
residual=np.linalg.norm(centered@r-reference,axis=1)
report={'newton_version':'1.5.0','warp_version':'1.16.0','plasticity_enabled':sim.enabled,'duration_s':sim.time,'wall_time_s':time.perf_counter()-start,'iterations':sim.iterations,'substeps':sim.substeps,'finite':bool(np.isfinite(q).all()),'shape_residual_rms_m':float(np.sqrt((residual**2).mean())),'shape_residual_max_m':float(residual.max()),'plastic_hinges':sim.rows[-1]['plastic_hinges'],'plastic_work_J':sim.rows[-1]['plastic_work_J'],'peak_finger_contact_N':max(max(x['left_contact_N'],x['right_contact_N']) for x in sim.rows),'peak_wrist_contact_Nm':max(x['wrist_contact_Nm'] for x in sim.rows),'minimum_z_m':min(x['min_z_m'] for x in sim.rows),'final':sim.rows[-1],'material_overrides':overrides,'force_limit_N':sim.attr(sim.settings,'maxFingerForce'),'calibration':'illustrative; no measured cardboard properties supplied'}
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
