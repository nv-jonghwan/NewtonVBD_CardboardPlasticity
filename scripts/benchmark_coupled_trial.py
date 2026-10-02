"""Separate, fully recorded same-mesh coupling experiment with bounded duration."""
import argparse,csv,json,time,hashlib,sys
from importlib.metadata import version
from pathlib import Path
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop

p=argparse.ArgumentParser()
p.add_argument('--scene',default='assets/demo_scene_robotiq_board10_support20_trial.usda')
p.add_argument('--output',required=True)
p.add_argument('--duration',type=float,default=1.)
p.add_argument('--baseline',action='store_true')
p.add_argument('--proxy-iterations',type=int,default=1)
p.add_argument('--mass-scale',type=float,default=1.)
p.add_argument('--mode',choices=['lagged','staggered'],default='lagged')
p.add_argument('--relaxation',type=float,default=1.)
p.add_argument('--mujoco-iterations',type=int,default=20)
p.add_argument('--partition',choices=['full','gripper'],default='full')
p.add_argument('--loop-time-constant',type=float,default=.0025)
p.add_argument('--proxy-joints',action='store_true')
a=p.parse_args()
out=ROOT/a.output
out.mkdir(parents=True,exist_ok=False)
options=None if a.baseline else dict(proxy_iterations=a.proxy_iterations,mass_scale=a.mass_scale,
    mode=a.mode,relaxation=a.relaxation,mujoco_iterations=a.mujoco_iterations,partition=a.partition,
    loop_time_constant=a.loop_time_constant,proxy_joints=a.proxy_joints)
(out/'provenance.json').write_text(json.dumps(dict(
    argv=sys.argv,python=sys.version,packages={name:version(name) for name in
        (['newton','warp-lang','numpy'] if a.baseline else ['newton','warp-lang','numpy','mujoco','mujoco-warp'])},
    source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in sorted((ROOT/'src/cardboard').glob('*.py'))}),indent=2))
start=time.perf_counter()
s=PickCrushDrop(scene_path=ROOT/a.scene,coupled_options=options)
wp.synchronize();init=time.perf_counter()-start
ts=[s.time];points=[s.a.particle_q.numpy()];bodies=[s.a.body_q.numpy()];frame_times=[]
start=time.perf_counter()
error=None
try:
    for i in range(round(a.duration*s.fps)):
        t0=time.perf_counter();row=s.advance();wp.synchronize()
        frame_times.append(time.perf_counter()-t0)
        row['active_wall_s']=time.perf_counter()-start
        if i%2==1:
            ts.append(s.time);points.append(s.a.particle_q.numpy());bodies.append(s.a.body_q.numpy())
        if i%60==0:print(json.dumps(dict(frame=i,wall=time.perf_counter()-start,**row)),flush=True)
        # Revolute IK coordinates may differ by 2*pi while representing the
        # same pose. Compare the physical angle, not its quaternion branch.
        errors=[row[n+'_input_rad']-row[n+'_input_target_rad'] for n in ['left','right']]
        if row['phase']<3 and max(abs(np.arctan2(np.sin(errors),np.cos(errors))))>.2:
            raise RuntimeError('Pre-contact gripper tracking diverged; stop unqualified trial early')
        if len(s.rows)>1 and s.rows[-2]['phase']==4 and row['phase']==5:
            lifted=s.rows[-2]
            floor=[r for r in s.rows if r['phase']==2][-1]['min_z_m']
            if lifted['min_z_m']-floor<=.05 or min(lifted['left_contact_N'],lifted['right_contact_N'])<=1:
                raise RuntimeError('Existing lift/contact gate failed; stop unqualified trial before crush')
except BaseException as exc:
    error=repr(exc)
finally:
    elapsed=time.perf_counter()-start
    if s.rows:
        with (out/'state.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=s.rows[0]);w.writeheader();w.writerows(s.rows)
    np.savez_compressed(out/'trajectory.npz',t=np.asarray(ts),points=np.asarray(points),
                        body_q=np.asarray(bodies),body_labels=np.asarray(s.model.body_label))
    np.savez_compressed(out/'final_material_state.npz',t=s.time,points=s.a.particle_q.numpy(),
        velocity=s.a.particle_qd.numpy(),plastic_angle=s.a.cardboard.plastic_angle.numpy(),
        accumulated_angle=s.a.cardboard.accumulated_angle.numpy(),
        plastic_work=s.a.cardboard.plastic_work.numpy(),damage=s.a.cardboard.damage.numpy())
    (out/'run_config.json').write_text(json.dumps(dict(scene=a.scene,newton=s.newton_version,
        iterations=s.iterations,substeps=s.active_substeps,coupled=options,
        active_shell_iterations=len([i for i in range(s.iterations)
            if i%s.active_particle_interval==0 or i==s.iterations-1]),
        supported_shell_iterations=s.iterations,
        physical_vertices=s.model.particle_count,physical_triangles=s.model.tri_count),indent=2))
    (out/'parameters.json').write_text(json.dumps(dict(thickness=float(s.attr(s.mat,'thickness')))))
    report=dict(scene=a.scene,coupled=options,init_seconds=init,wall_time_s=elapsed,
        duration_s=s.time,frames=len(s.rows),frame_times=frame_times,error=error,
        vertices=s.model.particle_count,triangles=s.model.tri_count,
        final=s.rows[-1] if s.rows else None)
    (out/'performance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='frame_times'},indent=2),flush=True)
if error:raise RuntimeError(error)
