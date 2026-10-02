"""Render a worker snapshot without running another physics simulation."""
import argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--state',required=True);p.add_argument('--scene',required=True);p.add_argument('--output',required=True);p.add_argument('--eye',type=float,nargs=3,default=[1.6,-1.9,1.7]);p.add_argument('--target',type=float,nargs=3,default=[.18,0,1.02]);p.add_argument('--crease-angle',type=float);p.add_argument('--crease-transition',type=float);a=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':0,'multi_gpu':False,'width':1280,'height':900,'extra_args':['--/rtx/hydra/readTransformsFromFabricInRenderDelegate=false']})
import omni.usd,omni.replicator.core as rep
import numpy as np
from pxr import UsdGeom,UsdPhysics,UsdLux,Gf,Vt
from PIL import Image
from cardboard.geometry import skin
from cardboard.surface import PanelSurface
root=Path(__file__).resolve().parents[1]
omni.usd.get_context().open_stage(str(root/a.scene))
for _ in range(20):app.update()
s=omni.usd.get_context().get_stage();s.SetEditTarget(s.GetSessionLayer())
with np.load(root/a.state) as d:points=d['points'];bodies=d['body_q'];labels=d['body_labels']
cache=UsdGeom.XformCache()
for label,pose in zip(labels,bodies):
 prim=s.GetPrimAtPath(str(label));parent=cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse();x=UsdGeom.Xformable(prim);x.ClearXformOpOrder()
 m=Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]),Gf.Vec3d(*pose[3:6].tolist())));m.SetTranslateOnly(Gf.Vec3d(*pose[:3].tolist()));x.AddTransformOp().Set(m*parent)
for prim in s.Traverse():
 if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
 if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
mesh=s.GetPrimAtPath('/World/Box/SimMesh');r=UsdGeom.Mesh(s.GetPrimAtPath('/World/Box/RenderMesh'));center=np.array(cache.GetLocalToWorldTransform(mesh).ExtractTranslation())
inds=np.array(r.GetPrim().GetAttribute('cardboard:bindingIndices').Get());w=np.array(r.GetPrim().GetAttribute('cardboard:bindingWeights').Get());off=np.array(r.GetPrim().GetAttribute('cardboard:bindingOffsets').Get())
if r.GetPrim().GetAttribute('cardboard:surfaceInterpolation').Get() == 'creaseAwareCubic':
 angle=a.crease_angle if a.crease_angle is not None else r.GetPrim().GetAttribute('cardboard:visualCreaseAngleDegrees').Get()
 transition=a.crease_transition if a.crease_transition is not None else r.GetPrim().GetAttribute('cardboard:visualCreaseTransitionDegrees').Get()
 surface=PanelSurface(np.array(mesh.GetAttribute('cardboard:restPoints').Get()),np.array(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3),inds,w,off,angle,transition,bool(r.GetPrim().GetAttribute('cardboard:visualSmoothThickness').Get()))
 display,normals=surface.evaluate(points-center)
 r.SetNormalsInterpolation('vertex');r.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
else:display=skin(points-center,inds,w,off).astype(np.float32)
r.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(display))
UsdLux.DomeLight.Define(s,'/World/InspectLight').CreateIntensityAttr(1000)
cam=UsdGeom.Camera.Define(s,'/World/InspectCamera');cam.CreateFocalLengthAttr(35);cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*a.eye),Gf.Vec3d(*a.target),Gf.Vec3d(0,0,1)).GetInverse())
product=rep.create.render_product('/World/InspectCamera',(1280,900));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product]);rep.orchestrator.step(rt_subframes=8,delta_time=0,pause_timeline=True)
for _ in range(30):app.update()
image=rgb.get_data();assert isinstance(image,np.ndarray) and image.size
Image.fromarray(image[:,:,:3]).save(root/a.output);print('SAVED',a.output,flush=True)
app.close()
