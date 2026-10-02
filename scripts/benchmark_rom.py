"""Bounded, fully recorded ROM/FOM comparison; never changes default scenes."""
import argparse,csv,json,time,hashlib,sys
from importlib.metadata import version
from pathlib import Path
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop

p=argparse.ArgumentParser()
p.add_argument('--scene',default='assets/demo_scene_robotiq_board10_primitive.usda')
p.add_argument('--output',required=True)
p.add_argument('--basis')
p.add_argument('--tolerance',type=float,default=.35)
p.add_argument('--full-every',type=int,default=4)
p.add_argument('--duration',type=float,default=16.)
p.add_argument('--device',default='cuda:0')
a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=False)
if not np.isfinite(a.duration) or a.duration <= 0:
    p.error('--duration must be finite and positive')
options=dict(basis=str(ROOT/a.basis),tolerance=a.tolerance,full_every=a.full_every) if a.basis else None
config=dict(scene=a.scene,device=a.device,rom=options,
    requested_duration_s=a.duration,argv=sys.argv,
    packages={name:version(name) for name in ['newton','warp-lang','numpy']},
    basis_sha256=hashlib.sha256((ROOT/a.basis).read_bytes()).hexdigest() if a.basis else None,
    scene_sha256=hashlib.sha256((ROOT/a.scene).read_bytes()).hexdigest(),
    source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (ROOT/'src/cardboard').glob('*.py')})
(out/'run_config.json').write_text(json.dumps(config,indent=2))
start=time.perf_counter()
try:
    s=PickCrushDrop(scene_path=ROOT/a.scene,device=a.device,rom_options=options)
    wp.synchronize()
except Exception as exc:
    failure=dict(init_seconds=time.perf_counter()-start,wall_time_s=0.,duration_s=0.,
                 error=repr(exc),failure_phase='initialization',frame_times=[],rom_counts=None)
    (out/'performance.json').write_text(json.dumps(failure,indent=2))
    (out/'progress.json').write_text(json.dumps(dict(t=0.,finished=False,error=repr(exc)),indent=2))
    raise
init=time.perf_counter()-start
config.update(newton=s.newton_version,iterations=s.iterations,substeps=s.active_substeps,physical_vertices=s.model.particle_count,physical_triangles=s.model.tri_count)
(out/'run_config.json').write_text(json.dumps(config,indent=2))
(out/'parameters.json').write_text(json.dumps(dict(thickness=float(s.attr(s.mat,'thickness')))))
ts=[0.];points=[s.a.particle_q.numpy()];bodies=[s.a.body_q.numpy()];frame_times=[];error=None
start=time.perf_counter()
try:
    for i in range(round(a.duration*s.fps)):
        t=time.perf_counter();row=s.advance();wp.synchronize();frame_times.append(time.perf_counter()-t)
        row['active_wall_s']=time.perf_counter()-start
        if i%2==1:ts.append(s.time);points.append(s.a.particle_q.numpy());bodies.append(s.a.body_q.numpy())
        if i%60==0:
            progress=dict(frame=i,wall=row['active_wall_s'],t=s.time,speed=row['max_speed_m_s'],plastic=row['plastic_hinges'],rom_counts=s.solver.rom.counters.numpy().tolist() if options else None)
            print(json.dumps(progress),flush=True)
            # Durable progress evidence after interruption; not a solver checkpoint.
            tmp=out/'progress.tmp';tmp.write_text(json.dumps(progress,indent=2));tmp.replace(out/'progress.json')
        if not np.isfinite(row['max_speed_m_s']) or row['max_speed_m_s']>10:raise RuntimeError('Nonfinite/explosive dynamics')
        if row['phase']<3 and row['plastic_hinges']>0:raise RuntimeError('Pregrasp plasticity gate failed')
        if len(s.rows)>1 and s.rows[-2]['phase']==4 and row['phase']==5:
            lift=s.rows[-2];floor=[r for r in s.rows if r['phase']==2][-1]['min_z_m']
            if lift['min_z_m']-floor<.05 or min(lift['left_contact_N'],lift['right_contact_N'])<=1:raise RuntimeError('Lift/bilateral contact gate failed')
except BaseException as exc:error=repr(exc)
finally:
    elapsed=time.perf_counter()-start
    if s.rows:
        with (out/'state.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=s.rows[0]);w.writeheader();w.writerows(s.rows)
    np.savez_compressed(out/'trajectory.npz',t=ts,points=points,body_q=bodies,body_labels=s.model.body_label)
    np.savez_compressed(out/'final_material_state.npz',t=s.time,points=s.a.particle_q.numpy(),velocity=s.a.particle_qd.numpy(),
        plastic_angle=s.a.cardboard.plastic_angle.numpy(),accumulated_angle=s.a.cardboard.accumulated_angle.numpy(),
        plastic_work=s.a.cardboard.plastic_work.numpy(),damage=s.a.cardboard.damage.numpy())
    report=dict(init_seconds=init,wall_time_s=elapsed,duration_s=s.time,error=error,frame_times=frame_times,
                rom_counter_labels=['attempts','accepted','representation_rejects','residual_rejects'],
                rom_counts=s.solver.rom.counters.numpy().tolist() if options else None)
    (out/'performance.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='frame_times'}),flush=True)
    (out/'progress.json').write_text(json.dumps(dict(t=s.time,finished=error is None,error=error),indent=2))
if error:raise RuntimeError(error)
