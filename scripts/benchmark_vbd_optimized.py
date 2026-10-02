"""Fresh full-order VBD scheduling comparison, with durable trajectory evidence."""
import argparse,csv,json,time,hashlib,sys,os,subprocess
from importlib.metadata import version
from pathlib import Path
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop

p=argparse.ArgumentParser()
p.add_argument('--scene',default='assets/demo_scene_robotiq_board10_primitive.usda')
p.add_argument('--output',required=True)
p.add_argument('--schedule',choices=['baseline','guarded','adaptive'],required=True)
p.add_argument('--duration',type=float,default=16.)
p.add_argument('--device',default='cuda:1')
p.add_argument('--panel-diagnostics',action='store_true',help='Record top-center sag relative to its fitted rim plane')
p.add_argument('--small-bend-scale',type=float,default=1.)
p.add_argument('--small-bend-knee',type=float,default=.30)
p.add_argument('--small-bend-end',type=float,default=5.9)
p.add_argument('--small-bend-memory-curvature',type=float,default=0.,help='Opt-in gradual reinforcement loss with accumulated plastic curvature; zero keeps legacy switch')
p.add_argument('--crease-friction-curvature',type=float,default=0.)
p.add_argument('--rom-basis')
p.add_argument('--rom-tolerance',type=float,default=.35)
p.add_argument('--rom-full-every',type=int,default=2)
p.add_argument('--rom-start',action='store_true',help='Start each substep with ROM; every Nth and last sweep are full VBD')
p.add_argument('--rom-fast',action='store_true',help='Experimental: omit second residual check; masked GPU updates and defer rejected corrections to scheduled VBD')
p.add_argument('--iterations',type=int)
p.add_argument('--rom-element-fraction',type=float)
p.add_argument('--rom-full-after-yield',action='store_true',help='Use all internal elements after plastic history appears; switch stays on GPU')
p.add_argument('--local-vbd',action='store_true')
p.add_argument('--patch-rings',type=int,default=1)
p.add_argument('--patch-curvature',type=float,default=7.5)
p.add_argument('--patch-full-every',type=int,default=8,help='Full VBD at every Nth solver iteration and at substep end')
p.add_argument('--patch-solver',choices=['colored','jacobi'],default='colored')
p.add_argument('--patch-relaxation',type=float,default=.5)
p.add_argument('--exploratory',action='store_true',help='Record legacy grasp quality failures without stopping the experimental trajectory')
a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=False)
if not np.isfinite(a.duration) or a.duration <= 0:
    p.error('--duration must be finite and positive')
if not np.isfinite(a.small_bend_memory_curvature) or a.small_bend_memory_curvature<0:
    p.error('--small-bend-memory-curvature must be finite and nonnegative')
if not np.isfinite(a.crease_friction_curvature) or a.crease_friction_curvature<0:
    p.error('--crease-friction-curvature must be finite and nonnegative')
if (a.small_bend_memory_curvature>0 or a.crease_friction_curvature>0) and a.small_bend_scale<=1:
    p.error('Gradual reinforcement requires --small-bend-scale greater than one')
os.environ['CARDBOARD_VBD_SCHEDULE']=a.schedule
options=dict(basis=str(ROOT/a.rom_basis),tolerance=a.rom_tolerance,full_every=a.rom_full_every,
             start_with_rom=a.rom_start,check_residual=not a.rom_fast,defer_fallback=a.rom_fast) if a.rom_basis else None
if a.iterations is not None and a.iterations<1:p.error('--iterations must be positive')
if (a.rom_element_fraction is not None or a.local_vbd) and not a.rom_basis:p.error('Representative/local corrections require --rom-basis')
if options is not None:
    options.update(element_fraction=a.rom_element_fraction,local_vbd=a.local_vbd,patch_rings=a.patch_rings,
                   patch_curvature=a.patch_curvature,patch_full_every=a.patch_full_every,
                   patch_solver=a.patch_solver,patch_relaxation=a.patch_relaxation,element_full_after_yield=a.rom_full_after_yield)
config=dict(scene=a.scene,device=a.device,schedule=a.schedule,rom=options,
    gpu_start=subprocess.run(['nvidia-smi'],capture_output=True,text=True).stdout,
    requested_duration_s=a.duration,argv=sys.argv,exploratory=a.exploratory,
    basis_sha256=hashlib.sha256((ROOT/a.rom_basis).read_bytes()).hexdigest() if a.rom_basis else None,
    packages={name:version(name) for name in ['newton','warp-lang','numpy']},
    scene_sha256=hashlib.sha256((ROOT/a.scene).read_bytes()).hexdigest(),
    source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (ROOT/'src/cardboard').glob('*.py')})
(out/'run_config.json').write_text(json.dumps(config,indent=2))
start=time.perf_counter()
try:
    if a.small_bend_scale!=1.:
        from cardboard.small_bend_trial import SmallBendPickCrushDrop
        config['small_bend']=dict(scale=a.small_bend_scale,knee=a.small_bend_knee,end=a.small_bend_end,memory_curvature=a.small_bend_memory_curvature,crease_friction_curvature=a.crease_friction_curvature)
        s=SmallBendPickCrushDrop(scene_path=ROOT/a.scene,device=a.device,iterations=a.iterations,rom_options=options,small_bend_options=config['small_bend'])
    else:
        s=PickCrushDrop(scene_path=ROOT/a.scene,device=a.device,iterations=a.iterations,rom_options=options)
    wp.synchronize()
except Exception as exc:
    failure=dict(init_seconds=time.perf_counter()-start,wall_time_s=0.,duration_s=0.,
                 error=repr(exc),failure_phase='initialization',frame_times=[],rom_counts=None)
    (out/'performance.json').write_text(json.dumps(failure,indent=2))
    (out/'progress.json').write_text(json.dumps(dict(t=0.,finished=False,error=repr(exc)),indent=2))
    raise
init=time.perf_counter()-start
if a.panel_diagnostics:
    local=s.local-(s.local.min(0)+s.local.max(0))/2;half=np.ptp(local,axis=0)/2
    top=np.isclose(local[:,2],half[2],atol=1e-6)
    panel_center=top&(np.abs(local[:,0])<.2*half[0])&(np.abs(local[:,1])<.2*half[1])
    panel_rim=top&((np.abs(local[:,0])>.999*half[0])|(np.abs(local[:,1])>.999*half[1]))
config.update(newton=s.newton_version,iterations=s.iterations,substeps=s.active_substeps,physical_vertices=s.model.particle_count,physical_triangles=s.model.tri_count)
representatives=getattr(getattr(s.solver,'rom',None),'representatives',None)
if representatives is not None:(out/'representative_elements.json').write_text(json.dumps(representatives.report,indent=2))
(out/'run_config.json').write_text(json.dumps(config,indent=2))
(out/'parameters.json').write_text(json.dumps(dict(thickness=float(s.attr(s.mat,'thickness')))))
ts=[0.];points=[s.a.particle_q.numpy()];bodies=[s.a.body_q.numpy()];frame_times=[];error=None
patch=getattr(s.solver,'local_patch',None);previous_patch_counts=np.zeros(4,np.int64)
start=time.perf_counter()
try:
    for i in range(round(a.duration*s.fps)):
        t=time.perf_counter();row=s.advance();wp.synchronize();frame_times.append(time.perf_counter()-t)
        row['active_wall_s']=time.perf_counter()-start
        if patch is not None:
            counts=patch.stats.numpy();delta=counts-previous_patch_counts;previous_patch_counts=counts
            row['patch_local_fraction']=float(delta[1]/max(delta[2],1))
            row['patch_local_sweeps']=int(delta[0]);row['patch_full_sweeps']=int(delta[3])
        if representatives is not None and representatives.full_after_yield:
            row['rom_full_elements']=int(representatives.full.numpy()[0])
        if a.panel_diagnostics:
            q=s.a.particle_q.numpy();center=q[panel_rim].mean(0)
            normal=np.linalg.svd(q[panel_rim]-center)[2][-1];normal*=1 if normal[2]>=0 else -1
            row['top_sag_mm']=float((center-q[panel_center].mean(0))@normal*1000)
        if i%2==1:ts.append(s.time);points.append(s.a.particle_q.numpy());bodies.append(s.a.body_q.numpy())
        if i%60==0:
            progress=dict(frame=i,wall=row['active_wall_s'],t=s.time,speed=row['max_speed_m_s'],plastic=row['plastic_hinges'],rom_counts=s.solver.rom.counters.numpy().tolist() if options else None)
            if a.panel_diagnostics:progress.update(top_sag_mm=row['top_sag_mm'],actual_gap_mm=row['actual_gap_m']*1000)
            print(json.dumps(progress),flush=True)
            # Durable progress evidence after interruption; not a solver checkpoint.
            tmp=out/'progress.tmp';tmp.write_text(json.dumps(progress,indent=2));tmp.replace(out/'progress.json')
        if not np.isfinite(row['max_speed_m_s']) or row['max_speed_m_s']>10:raise RuntimeError('Nonfinite/explosive dynamics')
        if not a.exploratory and row['phase']<3 and row['plastic_hinges']>0:raise RuntimeError('Pregrasp plasticity gate failed')
        if not a.exploratory and len(s.rows)>1 and s.rows[-2]['phase']==4 and row['phase']==5:
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
        plastic_work=s.a.cardboard.plastic_work.numpy(),damage=s.a.cardboard.damage.numpy(),
        **({'crease_friction_work':s.crease_friction_work.numpy()} if hasattr(s,'crease_friction_work') else {}))
    report=dict(schedule=s.solver.contact_schedule,gpu_end=subprocess.run(['nvidia-smi'],capture_output=True,text=True).stdout,init_seconds=init,wall_time_s=elapsed,duration_s=s.time,error=error,frame_times=frame_times,
                rom_counter_labels=['attempts','accepted','representation_rejects','residual_rejects'],
                rom_counts=s.solver.rom.counters.numpy().tolist() if options else None)
    patch=getattr(s.solver,'local_patch',None)
    if patch is not None:
        report['patch_counts']=patch.stats.numpy().tolist()
        report['patch_counter_labels']=['local_sweeps','selected_vertex_sum','eligible_vertex_sum','full_sweeps']
    (out/'performance.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='frame_times'}),flush=True)
    (out/'progress.json').write_text(json.dumps(dict(t=s.time,finished=error is None,error=error),indent=2))
if error:raise RuntimeError(error)
