"""Live Newton -> Isaac Sim bridge using atomic, local state snapshots.

Newton 1.6/Warp 1.17 stays in a separate process because Kit may preload another
Warp version. Isaac consumes deformed geometry and transforms; PhysX is disabled.
"""
import argparse,asyncio,csv,json,os,subprocess,sys,time,traceback,faulthandler,signal
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--physics-device',default='cuda:1');p.add_argument('--max-fps',type=int,default=30,help='Presentation refresh limit; Newton keeps its own fixed timestep');p.add_argument('--quality',choices=['realtime','board15-gentle','realtime5','reference','robotiq','robotiq-fine','robotiq-creased','robotiq-board10','robotiq-primitive','robotiq-conservative'],default='realtime');p.add_argument('--scene');p.add_argument('--replay-directory');p.add_argument('--duration',type=float,default=None);p.add_argument('--exit-when-done',action='store_true');p.add_argument('--autoplay',action='store_true');p.add_argument('--verify-controls',action='store_true');p.add_argument('--verify-recording',action='store_true');p.add_argument('--vbd-schedule',choices=['baseline','guarded']);p.add_argument('--small-bend-scale',type=float,default=None,help='Opt-in experimental small-strain reinforcement');p.add_argument('--small-bend-knee',type=float,default=.3);p.add_argument('--small-bend-end',type=float,default=5.9)
p.add_argument('--record-directory',help='Save full live trajectory and material state in this local directory')
p.add_argument('--solver-config',help='Explicit benchmark run_config JSON for experimental solver options')
p.add_argument('--small-bend-memory-curvature',type=float,default=0.)
p.add_argument('--crease-friction-curvature',type=float,default=0.)
args=p.parse_args();args.verify_controls=args.verify_controls or args.verify_recording
if args.max_fps <= 0:p.error('--max-fps must be positive')
# The measured scheduling path is the primitive GUI default; other modes retain their prior path.
args.vbd_schedule=args.vbd_schedule or ('guarded' if args.quality in ('robotiq-primitive','robotiq-conservative') else 'baseline')
args.scene=args.scene or {'robotiq-conservative':'assets/demo_scene_robotiq_board10_conservative_bend1p2_force120.usda','robotiq-primitive':'assets/demo_scene_robotiq_board10_primitive.usda','robotiq-board10':'assets/demo_scene_robotiq_board10.usda','robotiq-creased':'assets/demo_scene_robotiq_creased.usda','robotiq-fine':'assets/demo_scene_robotiq_fine24.usda','robotiq':'assets/demo_scene_robotiq_scaled.usda','realtime':'assets/demo_scene_board15_half.usda','board15-gentle':'assets/demo_scene_board15_kd85.usda','realtime5':'assets/demo_scene_realtime.usda','reference':'assets/demo_scene_thick16_stronger'}[args.quality]
args.replay_directory=args.replay_directory or {'robotiq-conservative':'outputs/conservative_tuning_round2/bend1p2_force120_original_damping/run','robotiq-primitive':'outputs/vbd_optimized/guarded_b','robotiq-board10':'outputs/robotiq_board10','robotiq-creased':'outputs/robotiq_creased','robotiq-fine':'outputs/robotiq_fine24','robotiq':'outputs/robotiq_scaled','realtime':'outputs/board15_half','board15-gentle':'outputs/board15_kd85','realtime5':'outputs/realtime','reference':'outputs/thick16_stronger'}[args.quality]
ROOT=Path(__file__).resolve().parents[1];out=ROOT/args.record_directory if args.record_directory else ROOT/'outputs/live';out.mkdir(parents=True,exist_ok=True)
faulthandler.enable(all_threads=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'width':1280,'height':800,'active_gpu':0,'multi_gpu':False,'extra_args':['--/rtx/hydra/readTransformsFromFabricInRenderDelegate=false']})
def record_signal(signum,frame):
    print(f'GUI_SIGNAL pid={os.getpid()} signal={signum}',file=sys.stderr,flush=True)
    raise SystemExit(128+signum)
for observed_signal in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(observed_signal,record_signal)
import omni.usd
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf,Vt,Sdf
from cardboard.geometry import skin
from cardboard.surface import PanelSurface
from cardboard.camera import OVERVIEW, BOX_DETAIL, set_camera_view
omni.usd.get_context().open_stage(str(ROOT/args.scene))
for _ in range(20):app.update()
stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
mesh=stage.GetPrimAtPath('/World/Box/SimMesh');render=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/RenderMesh'))
center=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(mesh).ExtractTranslation())
inds=np.array(render.GetPrim().GetAttribute('cardboard:bindingIndices').Get());weights=np.array(render.GetPrim().GetAttribute('cardboard:bindingWeights').Get());offsets=np.array(render.GetPrim().GetAttribute('cardboard:bindingOffsets').Get())
surface = None
if render.GetPrim().GetAttribute('cardboard:surfaceInterpolation').Get() == 'creaseAwareCubic':
    surface = PanelSurface(np.array(mesh.GetAttribute('cardboard:restPoints').Get()), np.array(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3), inds, weights, offsets, render.GetPrim().GetAttribute('cardboard:visualCreaseAngleDegrees').Get(), render.GetPrim().GetAttribute('cardboard:visualCreaseTransitionDegrees').Get(), bool(render.GetPrim().GetAttribute('cardboard:visualSmoothThickness').Get()))
    render.SetNormalsInterpolation('vertex')
# Explicit order matches the imported articulated chain; fail closed on missing prims.
names=['base_link','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','ee_link']
labels=['/World/UR10/'+n for n in names]+['/World/Gripper/'+n for n in ['Palm','Left','Right']]
ops={}
for label in [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]:
    prim=stage.GetPrimAtPath(label);parent=UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent()).GetInverse();x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();ops[label]=(x.AddTransformOp(opSuffix='liveNewton'),parent);UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
light=UsdLux.DomeLight.Define(stage,'/World/LiveLight');light.CreateIntensityAttr(800)
camera=UsdGeom.Camera.Define(stage,'/World/LiveCamera');set_camera_view(camera,*OVERVIEW)
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path='/World/LiveCamera'
stream=out/f'state-{os.getpid()}.npz';control=out/f'control-{os.getpid()}.json'
env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src');env['CARDBOARD_VBD_SCHEDULE']=args.vbd_schedule
live_samples=[];live_reported=False
proc=None;log=None;panel=None;last=0;received=0;latest_t=0;failure_shown=False
replay=None;replay_rows=None;replay_info={};replay_active=False;replay_playing=False;replay_elapsed=0.;replay_origin=0.;replay_last=-1
status={'t':0.,'duration':args.duration or 16.,'phase':0,'playing':False,'done':False}
def command(mode):
    temp=control.with_suffix('.tmp');temp.write_text(json.dumps({'mode':mode}));os.replace(temp,control)
def start(autoplay=False):
    global proc,log,last,latest_t,status,failure_shown,replay_active,replay_playing,live_samples,live_reported
    replay_active=False;replay_playing=False
    if proc is not None and proc.poll() is None:
        command('stop')
        try:proc.wait(timeout=2)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
    if log:log.close()
    if stream.exists():stream.unlink()
    last=0;latest_t=0;failure_shown=False;live_samples=[];live_reported=False
    status={'t':0.,'duration':args.duration or 16.,'phase':0,'playing':False,'done':False}
    command('play' if autoplay else 'pause')
    log=(out/f'newton-{os.getpid()}-{time.time_ns()}.log').open('w')
    cmd=[str(ROOT/'scripts/python.sh'),str(ROOT/'scripts/stream_newton.py'),'--stream',str(stream),'--control',str(control),'--scenario','--scene',str(ROOT/args.scene),'--device',args.physics_device]
    if args.small_bend_scale is not None:cmd+=['--small-bend-scale',str(args.small_bend_scale),'--small-bend-knee',str(args.small_bend_knee),'--small-bend-end',str(args.small_bend_end),'--small-bend-memory-curvature',str(args.small_bend_memory_curvature),'--crease-friction-curvature',str(args.crease_friction_curvature)]
    if args.record_directory:cmd+=['--record']
    if args.duration is not None:cmd+=['--duration',str(args.duration)]
    if args.solver_config:cmd+=['--solver-config',str(ROOT/args.solver_config)]
    proc=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    if panel:panel.update(status,'Loading initial state...')
def action(name):
    print(f'GUI_ACTION {name} sim_time={latest_t:.6f}',flush=True)
    global replay,replay_rows,replay_info,replay_active,replay_playing,replay_elapsed,replay_origin,replay_last
    if name=='replay':
        folder=ROOT/args.replay_directory
        if not (folder/'trajectory.npz').exists() or not (folder/'state.csv').exists():
            if panel:panel.update(status,'Recorded replay is not available yet')
            return
        if replay is None:
            with np.load(folder/'trajectory.npz') as data:replay={k:data[k] for k in ['points','body_q','t']+(['body_labels'] if 'body_labels' in data else [])}
            replay_rows=list(csv.DictReader((folder/'state.csv').open()))
            config=json.loads((folder/'run_config.json').read_text()) if (folder/'run_config.json').exists() else {}
            material=json.loads((folder/'parameters.json').read_text()) if (folder/'parameters.json').exists() else {}
            replay_info=dict(engine_version=config.get('newton','unspecified'),thickness_mm=1000*material.get('thickness',0))
            if config.get('gripper_model')=='robotiq_scaled':replay_info.update(gripper_label=f"Virtual Robotiq 2F-140 x{config['gripper_scale']:g}",drive_label='Drive reference')
            replay['row_times']=np.array([float(row['t']) for row in replay_rows])
        if replay['points'].shape[1]!=len(UsdGeom.Mesh(mesh).GetPointsAttr().Get()):
            if panel:panel.update(status,'Replay topology differs from this scene')
            replay=None;return
        command('pause');replay_active=True;replay_playing=True;replay_elapsed=0.;replay_origin=time.monotonic();replay_last=-1
    elif name=='reset':start(False)
    elif name=='play':
        if replay_active:
            if replay_elapsed>=float(replay['t'][-1]):action('replay')
            else:replay_playing=True;replay_origin=time.monotonic()-replay_elapsed
            return
        if proc.poll() is not None:start(True)
        else:command('play')
    elif name=='pause':
        if replay_active:replay_playing=False
        else:command('pause')
    elif name in ('view_box','view_scene'):
        close=name=='view_box'
        set_camera_view(camera,*(BOX_DETAIL if close else OVERVIEW))
def display_state(q,b,body_labels=None):
    if surface is not None:
        points,normals=surface.evaluate(q-center)
    else:points=skin(q-center,inds,weights,offsets).astype(np.float32)
    with Sdf.ChangeBlock():
        render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points))
        if surface is not None:render.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
    UsdGeom.Mesh(mesh).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((q-center).astype(np.float32)))
    for label,pose in zip(labels if body_labels is None else body_labels,b):
        op,parent=ops[str(label)]
        m=Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]),Gf.Vec3d(*pose[3:6].tolist())))
        m.SetTranslateOnly(Gf.Vec3d(*pose[:3].tolist()));op.Set(m*parent)

if not args.headless:
    import omni.kit.app
    manager=omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(ROOT/'exts'))
    manager.set_extension_enabled_immediate('cardboard.scenario',True)
    from cardboard_scenario import connect
    panel=connect(action)
# This stage is a Newton display. Disable PhysX participation in the session
# layer so the generic toolbar can safely forward Play to the scenario.
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
import omni.timeline
timeline=omni.timeline.get_timeline_interface()
def timeline_event(event):
    if event.type==int(omni.timeline.TimelineEventType.PLAY):
        timeline.stop();action('play')
timeline_subscription=timeline.get_timeline_event_stream().create_subscription_to_pop(timeline_event)
start(False if args.verify_controls else (args.autoplay or args.exit_when_done))
verify_step=0;pause_started=0.;paused_time=0.;capture_task=None
exit_code=0
try:
    while app.is_running():
        presentation_started=time.monotonic()
        app.update()
        if replay_active:
            if replay_playing:replay_elapsed=min(time.monotonic()-replay_origin,float(replay['t'][-1]))
            index=max(0,min(int(np.searchsorted(replay['t'],replay_elapsed,side='right'))-1,len(replay['t'])-1))
            done=replay_elapsed>=float(replay['t'][-1])
            if done:replay_playing=False
            row=replay_rows[min(int(np.searchsorted(replay['row_times'],float(replay['t'][index]))),len(replay_rows)-1)]
            status={k:float(v) for k,v in row.items()};status.update(replay_info);status.update(phase=int(status['phase']),plastic_hinges=int(status['plastic_hinges']),playing=replay_playing,done=done,recorded=True,duration=float(replay['t'][-1]))
            if index!=replay_last:display_state(replay['points'][index],replay['body_q'][index],replay.get('body_labels'));replay_last=index
            if panel:panel.update(status)
        if not replay_active and stream.exists() and stream.stat().st_mtime_ns!=last:
            last=stream.stat().st_mtime_ns
            with np.load(stream) as data:
                q=data['points'];b=data['body_q'];latest_t=float(data['t']);status=json.loads(str(data['status']));body_labels=data['body_labels'] if 'body_labels' in data else labels
            display_state(q,b,body_labels)
            received+=1
            if status.get('playing') or status.get('done'):
                live_samples.append((time.monotonic(),latest_t))
            if status.get('done') and not live_reported and len(live_samples)>1:
                elapsed=live_samples[-1][0]-live_samples[0][0];span=live_samples[-1][1]-live_samples[0][1]
                perf=dict(physics_device=args.physics_device,max_presentation_fps=args.max_fps,sim_duration_s=latest_t,worker_active_wall_s=status.get('active_wall_s'),worker_realtime_factor=status.get('realtime_factor'),display_progress_wall_s=elapsed,display_progress_sim_s=span,display_realtime_factor=span/elapsed,delivered_fps=(len(live_samples)-1)/elapsed,received_frames=len(live_samples))
                (ROOT/args.replay_directory/'live_performance.json').write_text(json.dumps(perf,indent=2))
                (out/f'performance-{os.getpid()}.json').write_text(json.dumps(perf,indent=2))
                print('LIVE_PERFORMANCE '+json.dumps(perf),flush=True);live_reported=True
            if panel:panel.update(status)
        if args.verify_controls and received:
            invoke=panel.action if panel else action
            if verify_step==0 and stream.exists():
                invoke('play');verify_step=1
            elif verify_step==1 and latest_t>=.10:
                invoke('pause');verify_step=2
            elif verify_step==2 and not status.get('playing',False):
                pause_started=time.monotonic();paused_time=latest_t;verify_step=3
            elif verify_step==3 and time.monotonic()-pause_started>.4:
                if latest_t!=paused_time:raise RuntimeError('Pause did not freeze simulation')
                invoke('reset');verify_step=4
            elif verify_step==4 and stream.exists() and latest_t==0:
                if status.get('plastic_hinges')!=0:raise RuntimeError('Reset did not clear plastic state')
                invoke('play');verify_step=5
            elif verify_step==5 and status.get('done'):
                print('CONTROL_TEST play/pause/reset/replay PASS',flush=True)
                if args.verify_recording:invoke('view_box');invoke('replay');verify_step=6
                else:break
            elif verify_step==6 and replay_elapsed>.25:
                invoke('pause');paused_time=replay_elapsed;pause_started=time.monotonic();verify_step=7
            elif verify_step==7 and time.monotonic()-pause_started>.4:
                if replay_elapsed!=paused_time:raise RuntimeError('Recorded replay pause failed')
                invoke('play');verify_step=8
            elif verify_step==8 and not args.headless and capture_task is None and status.get('t',0)>=11.8:
                # Hold the requested frame while the renderer catches up with
                # animated geometry; otherwise the capture races replay updates.
                invoke('pause');pause_started=time.monotonic();verify_step=10
            elif verify_step==10 and time.monotonic()-pause_started>.5:
                from omni.kit.viewport.utility import capture_viewport_to_file,get_active_viewport
                capture=capture_viewport_to_file(get_active_viewport(),str(ROOT/args.replay_directory/'gui_crush.png'))
                capture_task=asyncio.ensure_future(capture.wait_for_result())
                verify_step=11
            elif verify_step==11 and capture_task.done():
                capture_task.result();invoke('play');verify_step=8
            elif verify_step==8 and status.get('done') and (capture_task is None or capture_task.done()):
                if not status.get('recorded') or replay_last!=len(replay['t'])-1:raise RuntimeError('Incomplete recorded replay')
                if capture_task is not None:capture_task.result()
                if not args.headless:
                    capture=capture_viewport_to_file(get_active_viewport(),str(ROOT/args.replay_directory/'gui_settled.png'))
                    capture_task=asyncio.ensure_future(capture.wait_for_result());verify_step=9
                else:
                    print('RECORDED_REPLAY pause/resume/final-frame PASS',flush=True);break
            elif verify_step==9 and capture_task.done():
                capture_task.result()
                print('RECORDED_REPLAY pause/resume/final-frame PASS',flush=True);break
        if proc.poll() is not None:
            if proc.returncode:
                if args.headless:raise RuntimeError('Newton producer failed; see '+log.name)
                if panel and not failure_shown:
                    panel.update(status,'Newton error: '+Path(log.name).name);failure_shown=True
            if args.exit_when_done and not args.verify_controls:break
        # Presentation pacing does not change producer timesteps or solver work.
        time.sleep(max(0.,1./args.max_fps-(time.monotonic()-presentation_started)))
    print(f'LIVE_BRIDGE received={received} final_sim_time={latest_t:.3f}',flush=True)
    if args.exit_when_done and (received==0 or latest_t<(args.duration or 16)-.05):raise RuntimeError('Incomplete live bridge')
except BaseException as error:
    exit_code=int(error.code or 0) if isinstance(error,SystemExit) else 1
    details=dict(pid=os.getpid(),sim_time=latest_t,status=status,error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),worker_returncode=proc.poll(),worker_log=log.name)
    (out/f'gui-error-{os.getpid()}.json').write_text(json.dumps(details,indent=2))
    print('GUI_EXCEPTION '+json.dumps(details),file=sys.stderr,flush=True)
    traceback.print_exc()
    raise
finally:
    print(f'GUI_SHUTDOWN pid={os.getpid()} exit_code={exit_code} sim_time={latest_t:.6f} worker_returncode={proc.poll()}',flush=True)
    try:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=15)
    finally:
        log.close();sys.stdout.flush();sys.stderr.flush();app.close(exit_code=exit_code)
