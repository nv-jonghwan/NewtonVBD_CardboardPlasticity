import json,os
import numpy as np
from pxr import Usd,UsdGeom,Sdf,Gf,Vt,UsdLux
from . import ROOT
from .geometry import skin
from .camera import set_camera_view

class Recorder:
    def __init__(self,sim,directory):
        self.sim=sim;self.directory=directory;directory.mkdir(parents=True,exist_ok=True)
        self.path=directory/'replay.usdc'
        self.stage=Usd.Stage.CreateNew(str(self.path)) if not self.path.exists() else Usd.Stage.Open(str(self.path))
        self.stage.GetRootLayer().Clear();self.stage.GetRootLayer().subLayerPaths=[os.path.relpath(ROOT/'assets/demo_scene.usda',directory)]
        self.stage.SetStartTimeCode(0);self.stage.SetTimeCodesPerSecond(sim.fps);self.stage.SetFramesPerSecond(sim.fps)
        render=sim.stage.GetPrimAtPath('/World/Box/RenderMesh')
        self.inds=np.asarray(render.GetAttribute('cardboard:bindingIndices').Get());self.weights=np.asarray(render.GetAttribute('cardboard:bindingWeights').Get());self.offsets=np.asarray(render.GetAttribute('cardboard:bindingOffsets').Get())
        self.render=UsdGeom.Mesh(self.stage.GetPrimAtPath(render.GetPath()));self.mesh=UsdGeom.Mesh(self.stage.GetPrimAtPath('/World/Box/SimMesh'))
        self.ops=[]
        for label in sim.model.body_label:
            prim=self.stage.GetPrimAtPath(label)
            if not prim:raise RuntimeError(f'Missing USD body {label}')
            x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();op=x.AddTransformOp(opSuffix='newton')
            parent=UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent()).GetInverse();self.ops.append((op,parent))
        self.samples=[];self.bodies=[];self.joints=[];self.times=[]
        light=UsdLux.DomeLight.Define(self.stage,'/World/Lighting/Dome');light.CreateIntensityAttr(450)
        key=UsdLux.DistantLight.Define(self.stage,'/World/Lighting/Key');key.CreateIntensityAttr(2500);key.AddRotateXYZOp().Set(Gf.Vec3f(30,-25,-35));key.CreateAngleAttr(12)
        camera=UsdGeom.Camera.Define(self.stage,'/World/Camera');set_camera_view(camera,(1.7,-2.,1.65),(.05,0.,.90),26)
        self.frame=0
    def capture(self):
        s=self.sim;q=s.a.particle_q.numpy();b=s.a.body_q.numpy();local=(q-s.center).astype(np.float32);frame=round(s.time*s.fps)
        self.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local),frame)
        r=skin(local,self.inds,self.weights,self.offsets).astype(np.float32)
        self.render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(r),frame)
        # Clear obsolete static extents by authoring current tight bounds.
        self.render.CreateExtentAttr().Set([Gf.Vec3f(*r.min(0).tolist()),Gf.Vec3f(*r.max(0).tolist())],frame)
        for (op,parent),pose in zip(self.ops,b):
            m=Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]),Gf.Vec3d(*pose[3:6].astype(float))));m.SetTranslateOnly(Gf.Vec3d(*pose[:3].astype(float)));op.Set(m*parent,frame)
        self.times.append(s.time);self.samples.append(q);self.bodies.append(b);self.joints.append(s.a.joint_q.numpy())
        self.frame=frame
    def finish(self):
        self.stage.SetEndTimeCode(self.frame);self.stage.GetRootLayer().Save()
        # Portable state layer: geometry + velocities + constitutive history.
        checkpoint=Usd.Stage.CreateInMemory();checkpoint.GetRootLayer().subLayerPaths=[str(ROOT/'assets/cardboard.usda')]
        p=checkpoint.OverridePrim('/Box/SimMesh')
        for attr,arr in [('plasticAngles',self.sim.a.cardboard.plastic_angle),('accumulatedAngles',self.sim.a.cardboard.accumulated_angle),('plasticDissipation',self.sim.a.cardboard.plastic_work),('damage',self.sim.a.cardboard.damage)]:p.GetAttribute('cardboard:'+attr).Set(arr.numpy().tolist())
        UsdGeom.Mesh(p).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((self.sim.a.particle_q.numpy()-self.sim.center).astype(np.float32)))
        p.GetAttribute('cardboard:velocities').Set(Vt.Vec3fArray.FromNumpy(self.sim.a.particle_qd.numpy()))
        rendered=skin((self.sim.a.particle_q.numpy()-self.sim.center).astype(np.float32),self.inds,self.weights,self.offsets).astype(np.float32)
        visual=UsdGeom.Mesh(checkpoint.GetPrimAtPath('/Box/RenderMesh'));visual.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(rendered));visual.CreateExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.stack([rendered.min(0),rendered.max(0)])))
        checkpoint.GetRootLayer().subLayerPaths=[os.path.relpath(ROOT/'assets/cardboard.usda',self.directory)]
        checkpoint.GetRootLayer().Export(str(self.directory/'deformed_checkpoint.usda'))
        np.savez_compressed(self.directory/'trajectory.npz',t=np.array(self.times),points=np.array(self.samples),body_q=np.array(self.bodies),joint_q=np.array(self.joints),faces=self.sim.faces,initial=self.sim.initial,body_labels=np.array(self.sim.model.body_label),plastic_angles=self.sim.a.cardboard.plastic_angle.numpy())
