"""Isolated Newton worker; atomic command/status files keep Kit responsive."""
import argparse,json,os,time,csv
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--stream',required=True);p.add_argument('--duration',type=float)
p.add_argument('--device',default='cuda:1');p.add_argument('--control');p.add_argument('--scenario',action='store_true')
p.add_argument('--scene');p.add_argument('--record',action='store_true');p.add_argument('--small-bend-scale',type=float,default=None);p.add_argument('--small-bend-knee',type=float,default=.3);p.add_argument('--small-bend-end',type=float,default=5.9)
p.add_argument('--solver-config',help='Explicit benchmark run_config JSON supplying iterations and ROM options')
p.add_argument('--small-bend-memory-curvature',type=float,default=0.)
p.add_argument('--crease-friction-curvature',type=float,default=0.)
args=p.parse_args();path=Path(args.stream);path.parent.mkdir(parents=True,exist_ok=True)
from cardboard import ROOT
from cardboard.usd_solver import read_solver_config, write_runtime_layer
usd_config = read_solver_config(ROOT/args.scene) if args.scenario and args.scene else None
runtime_config = dict(usd_config or {})
if args.solver_config:
    config=json.loads((ROOT/args.solver_config).read_text())
    if config.get('solver_source')=='usd':
        from cardboard.release import load_profile
        profile_path = (ROOT/args.solver_config).resolve().relative_to(ROOT.resolve())
        config=load_profile(profile_path)
    runtime_config.update({k:config[k] for k in ('iterations','rom','small_bend','schedule') if k in config})
solver_options={};rom_options=runtime_config.get('rom')
if runtime_config:
    iterations=runtime_config.get('iterations')
    if not isinstance(iterations,int) or iterations<1:p.error('Solver config requires positive integer iterations')
    if rom_options is not None:
        rom_options=dict(rom_options);rom_options['basis']=str(ROOT/rom_options['basis'])
    solver_options=dict(iterations=iterations,rom_options=rom_options)
    os.environ.setdefault('CARDBOARD_VBD_SCHEDULE',runtime_config.get('schedule','baseline'))
small_bend=runtime_config.get('small_bend')
if args.small_bend_scale is not None:
    if not args.scenario:p.error('--small-bend-scale requires --scenario')
    small_bend=(dict(scale=args.small_bend_scale,knee=args.small_bend_knee,end=args.small_bend_end,
                     memory_curvature=args.small_bend_memory_curvature,crease_friction_curvature=args.crease_friction_curvature)
                if args.small_bend_scale!=1. else None)
runtime_config.update(rom=rom_options,small_bend=small_bend)
if args.scenario:
    if small_bend is not None:
        from cardboard.small_bend_trial import SmallBendPickCrushDrop
        s=SmallBendPickCrushDrop(args.device,scene_path=args.scene,small_bend_options=small_bend,**solver_options)
    else:
        from cardboard.scenario import PickCrushDrop
        s=PickCrushDrop(args.device,scene_path=args.scene,**solver_options)
else:
    from cardboard.simulation import Simulation
    s=Simulation(args.device,scene_path=args.scene,**solver_options)
requested_schedule=os.environ.get('CARDBOARD_VBD_SCHEDULE','baseline')
if requested_schedule!='baseline' and getattr(s.solver,'contact_schedule','baseline')!=requested_schedule:
    raise RuntimeError('Requested VBD scheduling is unavailable for this scene/solver')
duration=args.duration or (float(s.attr(s.settings,'duration')) if usd_config else getattr(s,'duration',11))
if args.record:write_runtime_layer(s,path.parent/'effective_scene.usda',runtime_config)
realtime=bool(s.attr(s.settings,'realtimePacing'))
active_wall=0.;active_sim=0.;block_started=None;block_sim_start=0.
def snapshot(row=None,playing=False):
    row=row or {};status={'engine_version':s.newton_version,'vbd_schedule':getattr(s.solver,'contact_schedule','baseline'),'thickness_mm':float(s.attr(s.mat,'thickness'))*1000,'t':s.time,'phase':getattr(s,'phase',0),'playing':playing,
        'done':s.time>=duration-1e-6,'duration':duration,'plastic_hinges':row.get('plastic_hinges',0),
        'left_contact_N':row.get('left_contact_N',0.),'right_contact_N':row.get('right_contact_N',0.),
        'command_gap_m':row.get('command_gap_m',float(s.attr(s.settings,'openGap'))),'realtime_factor':row.get('realtime_factor',0.),'active_wall_s':row.get('active_wall_s',0.),'actual_gap_m':row.get('actual_gap_m',s.gripper.gap(s.a.body_q.numpy()) if s.is_robotiq else .32),'motor_limit_N':float(s.motor_force_limit.numpy()[0]),
        'min_z_m':float(s.a.particle_q.numpy()[:,2].min()),'box_sleeping':bool(row.get('box_sleeping',False)),'max_speed_m_s':row.get('max_speed_m_s',0.)}
    if small_bend is not None:status['small_bend']=small_bend
    if runtime_config.get('iterations'):
        status['solver_iterations']=s.iterations;status['rom']=rom_options
        patch=getattr(s.solver,'local_patch',None)
        if patch is not None:status['patch_counts']=patch.stats.numpy().tolist()
    if args.scenario and any(np.any(s.config[k]) for k in ('graspTiltDegrees','liftTiltDegrees','crushTiltDegrees','releaseTiltDegrees')):
        status['gripper_tilt_degrees']={k:s.config[k].tolist() for k in ('graspTiltDegrees','liftTiltDegrees','crushTiltDegrees','releaseTiltDegrees')}
    if s.is_robotiq:status.update(gripper_label=f'Virtual Robotiq 2F-140 x{s.gripper.scale:g}',drive_label='Drive reference')
    with path.with_suffix('.tmp').open('wb') as f:
        np.savez(f,body_labels=np.array(s.model.body_label),points=s.a.particle_q.numpy(),body_q=s.a.body_q.numpy(),t=s.time,
                 plastic_hinges=status['plastic_hinges'],status=json.dumps(status))
    os.replace(path.with_suffix('.tmp'),path)
snapshot();last_mode=None;row=None
record_t=[s.time];record_points=[s.a.particle_q.numpy()];record_bodies=[s.a.body_q.numpy()]
while s.time<duration-1e-6:
    mode='play'
    if args.control:
        try:mode=json.loads(Path(args.control).read_text()).get('mode','pause')
        except (OSError,ValueError):mode='pause'
    if mode=='stop':break
    if mode!='play':
        if block_started is not None:
            active_wall+=time.perf_counter()-block_started;active_sim+=s.time-block_sim_start;block_started=None
        if last_mode!=mode:snapshot(row,False)
        last_mode=mode;time.sleep(.03);continue
    if block_started is None:block_started=time.perf_counter();block_sim_start=s.time
    last_mode=mode;row=s.advance()
    if realtime:time.sleep(max(0.,(s.time-block_sim_start)-(time.perf_counter()-block_started)))
    elapsed=active_wall+time.perf_counter()-block_started
    row['active_wall_s']=elapsed;row['realtime_factor']=(active_sim+s.time-block_sim_start)/max(elapsed,1e-9)
    snapshot(row,True)
    if args.record and round(s.time*s.fps)%2==0:
        record_t.append(s.time);record_points.append(s.a.particle_q.numpy());record_bodies.append(s.a.body_q.numpy())
if s.rows:
    with path.with_suffix('.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=s.rows[0]);w.writeheader();w.writerows(s.rows)
snapshot(row,False)
if args.record:
    write_runtime_layer(s,path.parent/'final_state.usda',runtime_config,state=True)
    np.savez_compressed(path.parent/'trajectory.npz',t=np.array(record_t),points=np.array(record_points),body_q=np.array(record_bodies),body_labels=np.array(s.model.body_label))
    # Preserve material history and actual terminal velocities for crease and
    # settling diagnostics. This is not an exact restart of solver multipliers.
    np.savez_compressed(path.parent/'final_material_state.npz',t=s.time,
                        points=s.a.particle_q.numpy(),velocity=s.a.particle_qd.numpy(),
                        plastic_angle=s.a.cardboard.plastic_angle.numpy(),
                        accumulated_angle=s.a.cardboard.accumulated_angle.numpy(),
                        plastic_work=s.a.cardboard.plastic_work.numpy(),damage=s.a.cardboard.damage.numpy(),
                        **({'crease_friction_work':s.crease_friction_work.numpy()} if hasattr(s,'crease_friction_work') else {}))
    (path.parent/'run_config.json').write_text(json.dumps({'newton':s.newton_version,'scene':args.scene,'small_bend':small_bend,'iterations':s.iterations,'substeps':s.active_substeps,'particle_solve_interval':s.active_particle_interval,'supported_refinement':s.rest_refinement,'supported_substeps':32 if s.rest_refinement else None,'supported_iterations':s.iterations if s.rest_refinement else None,'rotation_block':bool(getattr(s.solver,'rotation_block',False)),'internal_damping_rate':s.internal_damping.rate if s.internal_damping else 0.,'supported_sleep':s.rest is not None,'coupled_supported':s.coupled_supported,'supported_internal_damping_rate':s.rest_damping_rate,'supported_rigid_damping_rate':s.support_rigid_damping,'sleep_iterations':s.sleep_iterations,'gripper_model':s.attr(s.settings,'gripperModel'),'gripper_scale':s.attr(s.settings,'gripperScale')}))
    (path.parent/'parameters.json').write_text(json.dumps({'thickness':float(s.attr(s.mat,'thickness'))}))
    if runtime_config.get('iterations'):
        metadata_path=path.parent/'run_config.json'
        metadata=json.loads(metadata_path.read_text())
        metadata.update(rom=rom_options,solver_config=str(args.solver_config),solver_source='usd' if usd_config else 'arguments',solver_class=type(s.solver).__name__,schedule=getattr(s.solver,'contact_schedule','baseline'))
        metadata_path.write_text(json.dumps(metadata,indent=2))
print('Newton stream complete',s.time,flush=True)
