"""Render the recorded Newton USD in Isaac Sim; no PhysX stepping is performed."""
import argparse,os
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--replay',default='outputs/plastic/replay.usdc');p.add_argument('--output',default='outputs/plastic/render');p.add_argument('--frames',type=int,default=0);p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720);p.add_argument('--gpu',type=int,default=0);args=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':args.width,'height':args.height,'renderer':'RayTracedLighting','active_gpu':args.gpu,'physics_gpu':args.gpu,'multi_gpu':False,'extra_args':['--/rtx/hydra/readTransformsFromFabricInRenderDelegate=false','--/renderer/multiGpu/enabled=false']})
import omni.usd,omni.timeline
import omni.replicator.core as rep
from pxr import Usd,UsdGeom
from cardboard.camera import set_camera_clipping
import numpy as np
from PIL import Image
root=Path(__file__).resolve().parents[1];out=root/args.output;out.mkdir(parents=True,exist_ok=True)
omni.usd.get_context().open_stage(str(root/args.replay))
for _ in range(30):app.update()
stage=omni.usd.get_context().get_stage()
# Replay is a recorded USD animation; disable physical actuation in the session layer.
from pxr import UsdPhysics
stage.SetEditTarget(stage.GetSessionLayer())
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
set_camera_clipping(UsdGeom.Camera(stage.GetPrimAtPath('/World/Camera')))
timeline=omni.timeline.get_timeline_interface()
product=rep.create.render_product('/World/Camera',(args.width,args.height));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
last=int(stage.GetEndTimeCode());step=2;frames=list(range(0,last+1,step))
if args.frames:frames=np.linspace(0,last,args.frames).astype(int).tolist()
for n,frame in enumerate(frames):
    timeline.set_current_time(frame/stage.GetTimeCodesPerSecond())
    rep.orchestrator.step(rt_subframes=4,delta_time=0.0,pause_timeline=True)
    for _ in range(2):app.update()
    data=rgb.get_data()
    if not isinstance(data,np.ndarray) or data.size==0:
        for _ in range(10):app.update()
        data=rgb.get_data()
    if not isinstance(data,np.ndarray) or data.size==0:raise RuntimeError('RGB annotator returned no pixels')
    Image.fromarray(data[:,:,:3]).save(out/f'{n:04d}.png')
    if n%30==0:print(f'RENDER {n+1}/{len(frames)} time={frame/60:.2f}',flush=True)
app.close()
