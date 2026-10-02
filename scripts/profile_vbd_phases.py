"""Fresh VBD phase profile; graph events and instrumented kernel samples are separate.

No timestep, solver budget, contact law or scene edits. Detailed samples replace
one normal graph replay with an equivalent graph containing CUDA timing events
around the FIRST substep's kernels. Those frames are excluded from steady timing.
"""
import argparse
from collections import defaultdict
import csv
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop
from cardboard.surface import PanelSurface


def bucket(name, module):
    if module.endswith('plasticity'): return 'plasticity'
    if module.endswith('internal_damping'): return 'internal_damping'
    if module.endswith(('rigid_subspace','rotation_block','block_solver')): return 'global_shell_correction'
    if 'solve_elasticity' in name: return 'shell_elasticity'
    if 'accumulate_particle_body_contact' in name: return 'particle_rigid_contact'
    if 'accumulate_compact_self_contact' in name: return 'self_contact_force'
    if 'truncat' in name: return 'self_contact_truncation'
    if 'solve_rigid_body' in name: return 'rigid_solve'
    if 'accumulate_body' in name: return 'rigid_contact_accumulation'
    if 'dual' in name: return 'dual_updates'
    if 'colliding' in name or 'collision_detection' in name: return 'self_contact_detection'
    if any(x in module for x in ['collide','collision','bvh','broad_phase','narrow_phase']): return 'collision_detection'
    return 'other'


def gpu_snapshot():
    commands = {
        'gpus':['nvidia-smi','--query-gpu=timestamp,index,utilization.gpu,memory.used,clocks.sm,power.draw','--format=csv,noheader'],
        'compute_processes':['nvidia-smi','--query-compute-apps=gpu_uuid,pid,process_name,used_memory','--format=csv,noheader'],
    }
    return {name:subprocess.run(cmd,capture_output=True,text=True).stdout.strip() for name,cmd in commands.items()}


def summarize(values):
    values=np.asarray(values,float)
    return dict(n=len(values),mean=float(values.mean()),median=float(np.median(values)),p95=float(np.percentile(values,95)),total=float(values.sum())) if len(values) else None


parser=argparse.ArgumentParser()
parser.add_argument('--scene',default='assets/demo_scene_robotiq_board10_primitive.usda')
parser.add_argument('--device',default='cuda:1')
parser.add_argument('--duration',type=float,default=16.)
parser.add_argument('--samples',type=float,nargs='+',default=[.5,2.,6.,8.,11.,13.5,15.])
parser.add_argument('--output',required=True)
parser.add_argument('--small-bend-scale',type=float,default=1.)
parser.add_argument('--small-bend-knee',type=float,default=.3)
parser.add_argument('--small-bend-end',type=float,default=5.9)
args=parser.parse_args()
out=ROOT/args.output;out.mkdir(parents=True,exist_ok=False)
config=dict(scene=args.scene,device=args.device,args=vars(args),
            scene_sha256=hashlib.sha256((ROOT/args.scene).read_bytes()).hexdigest(),
            source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/cardboard').glob('*.py')},
            profiler_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            packages={name:version(name) for name in ['newton','warp-lang','numpy']},gpu_start=gpu_snapshot())
(out/'run_config.json').write_text(json.dumps(config,indent=2))
start=time.perf_counter()
if args.small_bend_scale!=1.:
    from cardboard.small_bend_trial import SmallBendPickCrushDrop
    s=SmallBendPickCrushDrop(scene_path=ROOT/args.scene,device=args.device,
        small_bend_options=dict(scale=args.small_bend_scale,knee=args.small_bend_knee,end=args.small_bend_end))
else:s=PickCrushDrop(scene_path=ROOT/args.scene,device=args.device)
wp.synchronize_device(args.device)
config.update(init_seconds=time.perf_counter()-start,newton=s.newton_version,iterations=s.iterations,substeps=s.active_substeps,
              physical_vertices=s.model.particle_count,physical_triangles=s.model.tri_count,
              bodies=s.model.body_count,shapes=s.model.shape_count,colors=[len(c) for c in s.model.particle_color_groups],
              body_colors=[len(c) for c in s.model.body_color_groups],soft_contact_capacity=s.contacts.soft_contact_max,
              tile_solve=s.solver.use_particle_tile_solve,nsight_available=False)
(out/'run_config.json').write_text(json.dumps(config,indent=2))
(out/'parameters.json').write_text(json.dumps(dict(thickness=float(s.attr(s.mat,'thickness')))))
render=s.stage.GetPrimAtPath('/World/Box/RenderMesh');attr=lambda n:render.GetAttribute('cardboard:'+n).Get()
surface=PanelSurface(s.local,s.faces,np.asarray(attr('bindingIndices')),np.asarray(attr('bindingWeights')),np.asarray(attr('bindingOffsets')),
                     attr('visualCreaseAngleDegrees'),attr('visualCreaseTransitionDegrees'),bool(attr('visualSmoothThickness')))
graph_start=wp.Event(args.device,enable_timing=True);graph_end=wp.Event(args.device,enable_timing=True)
original_capture=wp.capture_launch;original_launch=wp.launch;original_numpy=wp.array.numpy
frame_context={};samples=[];frames=[];times=[0.];points=[s.a.particle_q.numpy()];bodies=[s.a.body_q.numpy()]
sample_frames={round(t*s.fps) for t in args.samples};mode_age=0;old_mode=None


def traced_numpy(array,*a,**kw):
    start=time.perf_counter();result=original_numpy(array,*a,**kw)
    if array.device.is_cuda:
        frame_context['downloads']+=1;frame_context['download_bytes']+=result.nbytes
        frame_context['download_wall_ms']+=(time.perf_counter()-start)*1000
    return result


def traced_capture(graph,*a,**kw):
    frame_context['graph_used']=True
    if frame_context['sample']:
        events=[];counts=defaultdict(int);substep=-1;shape_meta={}
        def launch(*la,**lk):
            nonlocal substep
            kernel=lk.get('kernel',la[0] if la else None)
            name=kernel.key;module=kernel.module.name;key=module+':'+name
            if name=='set_kinematic_arm':substep+=1
            counts[key]+=1
            if substep==0:
                dim=lk.get('dim',la[1] if len(la)>1 else 0)
                shape_meta[key]=dict(dim=[int(x) for x in dim] if isinstance(dim,(tuple,list)) else int(dim),
                                     block_dim=lk.get('block_dim'),max_blocks=lk.get('max_blocks'),category=bucket(name,module))
                begin=wp.Event(args.device,enable_timing=True);end=wp.Event(args.device,enable_timing=True)
                wp.record_event(begin);result=original_launch(*la,**lk);wp.record_event(end)
                events.append((key,begin,end));return result
            return original_launch(*la,**lk)
        state_a,state_b=s.a,s.b
        build_start=time.perf_counter();wp.launch=launch
        try:
            with wp.ScopedCapture(device=args.device) as capture:s._integrate()
        finally:wp.launch=original_launch
        assert s.a is state_a and s.b is state_b, 'Timing capture must retain state buffer identities'
        frame_context['trace_build_ms']=(time.perf_counter()-build_start)*1000
        frame_context['trace_events']=events;frame_context['trace_counts']=counts;frame_context['trace_shapes']=shape_meta
        graph=capture.graph
        frame_context['trace_graph']=graph  # retain event graph until readback
    wp.record_event(graph_start);original_capture(graph,*a,**kw);wp.record_event(graph_end)


wp.capture_launch=traced_capture;wp.array.numpy=traced_numpy
error=None;script_start=time.perf_counter()
try:
    for i in range(round(args.duration*s.fps)):
        frame_context.clear();frame_context.update(sample=i in sample_frames,graph_used=False,downloads=0,download_bytes=0,download_wall_ms=0.)
        t0=time.perf_counter();row=s.advance();wp.synchronize_device(args.device);wall_ms=(time.perf_counter()-t0)*1000
        mode_age=mode_age+1 if s.graph_mode==old_mode else 0;old_mode=s.graph_mode
        gpu_ms=wp.get_event_elapsed_time(graph_start,graph_end,synchronize=False) if frame_context['graph_used'] else None
        record=dict(frame=i,t=s.time,phase=s.phase,mode=s.graph_mode,mode_age=mode_age,wall_ms=wall_ms,gpu_graph_ms=gpu_ms,
                    detailed_sample=bool(frame_context.get('trace_events')),downloads=frame_context['downloads'],download_bytes=frame_context['download_bytes'],
                    download_wall_ms_including_gpu_wait=frame_context['download_wall_ms'],substeps=s.substeps,iterations=s.solver.iterations,
                    particle_interval=s.solver.particle_solve_interval)
        frames.append(record);row['active_wall_s']=sum(f['wall_ms'] for f in frames)/1000
        if i%2==1:times.append(s.time);points.append(original_numpy(s.a.particle_q));bodies.append(original_numpy(s.a.body_q))
        if frame_context.get('trace_events'):
            totals=defaultdict(float);calls=defaultdict(int)
            for key,begin,end in frame_context['trace_events']:
                totals[key]+=wp.get_event_elapsed_time(begin,end,synchronize=False);calls[key]+=1
            kernels=[dict(name=key,first_substep_ms=totals[key],first_substep_calls=calls[key],frame_calls=frame_context['trace_counts'][key],
                          **frame_context['trace_shapes'][key]) for key in sorted(totals,key=totals.get,reverse=True)]
            contact=original_numpy(s.contacts.soft_contact_count).tolist()
            body_contact=original_numpy(s.solver.body_particle_contact_counts)
            occupancy=dict(soft_count=contact,soft_capacity=s.contacts.soft_contact_max,
                           self_pairs=original_numpy(s.solver.compact_counts).tolist(),
                           body_particle_contacts_max=int(body_contact.max()),body_particle_contacts_sum=int(body_contact.sum()),
                           body_contact_buffer=s.solver.body_particle_contact_buffer_pre_alloc)
            q=original_numpy(s.a.particle_q)-s.center;surface.evaluate(q);display=[]
            for repeat in range(5):
                st=time.perf_counter();surface.evaluate(q);display.append((time.perf_counter()-st)*1000)
            sample=dict(**record,trace_build_ms=frame_context['trace_build_ms'],first_substep_kernels=kernels,
                        first_substep_kernel_event_sum_ms=sum(totals.values()),frame_launch_calls=sum(frame_context['trace_counts'].values()),
                        contacts=occupancy,display_ms=summarize(display),gpu=gpu_snapshot())
            samples.append(sample);(out/f'kernel_frame_{i}.json').write_text(json.dumps(sample,indent=2))
        if i%60==0:
            progress=dict(t=s.time,mode=s.graph_mode,wall_ms=wall_ms,gpu_graph_ms=gpu_ms,samples=len(samples))
            (out/'progress.json').write_text(json.dumps(progress,indent=2));print(json.dumps(progress),flush=True)
except BaseException as exc:
    error=repr(exc)
finally:
    wp.capture_launch=original_capture;wp.launch=original_launch;wp.array.numpy=original_numpy
    if s.rows:
        with (out/'state.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=s.rows[0]);writer.writeheader();writer.writerows(s.rows)
    np.savez_compressed(out/'trajectory.npz',t=times,points=points,body_q=bodies,body_labels=s.model.body_label)
    np.savez_compressed(out/'final_material_state.npz',t=s.time,points=s.a.particle_q.numpy(),velocity=s.a.particle_qd.numpy(),
                        plastic_angle=s.a.cardboard.plastic_angle.numpy(),accumulated_angle=s.a.cardboard.accumulated_angle.numpy(),
                        plastic_work=s.a.cardboard.plastic_work.numpy(),damage=s.a.cardboard.damage.numpy())
    groups={}
    for phase in range(8):
        f=[r for r in frames if r['phase']==phase and not r['detailed_sample'] and r['mode_age']>=2 and r['gpu_graph_ms'] is not None]
        groups[str(phase)]=dict(wall_ms=summarize([r['wall_ms'] for r in f]),gpu_ms=summarize([r['gpu_graph_ms'] for r in f]),
                               outside_graph_ms=summarize([r['wall_ms']-r['gpu_graph_ms'] for r in f]),
                               modes={mode:sum(r['mode']==mode for r in f) for mode in ['active','supported','sleep']})
    result=dict(error=error,duration_s=s.time,script_wall_s=time.perf_counter()-script_start,phase_summary=groups,
                frames=frames,samples=samples,gpu_end=gpu_snapshot(),
                caveats='Detailed CUDA-event frames and mode startup excluded from steady timing. Individual kernel events perturb scheduling; first substep attribution is not uninstrumented hardware counters. Download wall includes GPU waiting. Display and telemetry occur outside frame timing.')
    (out/'profile.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(error=error,duration_s=s.time,phases=groups),indent=2),flush=True)
if error:raise RuntimeError(error)
