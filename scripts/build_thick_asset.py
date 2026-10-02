"""Equivalent thick sandwich board, with thickness-dependent plate rigidity."""
import argparse,numpy as np
from pxr import Usd,UsdGeom,Gf
from cardboard import ROOT
from cardboard.usd_utils import register
p=argparse.ArgumentParser();p.add_argument('--variant',default='_thick16');p.add_argument('--relaxation',type=float,default=.006);p.add_argument('--thickness',type=float,default=.005);p.add_argument('--yield-curvature',type=float,default=3.);p.add_argument('--crush-force',type=float,default=100.);p.add_argument('--crush-gap',type=float,default=.15);a=p.parse_args();register()
asset=Usd.Stage.Open(str(ROOT/'assets/cardboard_plate_crease.usda'));asset.GetRootLayer().Export(str(ROOT/f'assets/cardboard{a.variant}.usda'))
asset=Usd.Stage.Open(str(ROOT/f'assets/cardboard{a.variant}.usda'));mat=asset.GetPrimAtPath('/Box/Materials/Cardboard')
values=dict(thickness=a.thickness,arealDensity=.9,bendingReferenceThickness=.003,bendingThicknessExponent=2.,bendingMD=.2,bendingCD=.1,bendingRelaxationTime=a.relaxation,selfContactThicknessScale=1.,yieldCurvature=a.yield_curvature,hardeningRatio=.02,damageRate=.15,residualStiffness=.65)
for k,v in values.items():mat.GetAttribute('cardboard:'+k).Set(v)
mesh=asset.GetPrimAtPath('/Box/SimMesh');mesh.GetAttribute('cardboard:solver').Set('newtonVBD16')
r=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get());e=np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get());h=np.asarray(mesh.GetAttribute('cardboard:dualWidths').Get());edge=r[e[:,3]]-r[e[:,2]];w=(edge[:,1]/np.linalg.norm(edge,axis=1))**2
scale=(a.thickness/.003)**2;mesh.GetAttribute('cardboard:edgeStiffness').Set(((.1+.1*w)*scale/h).tolist());asset.GetRootLayer().Save()
scene=Usd.Stage.Open(str(ROOT/'assets/demo_scene_plate_crease.usda'));scene.GetRootLayer().Export(str(ROOT/f'assets/demo_scene{a.variant}.usda'));scene=Usd.Stage.Open(str(ROOT/f'assets/demo_scene{a.variant}.usda'))
box=scene.GetPrimAtPath('/World/Box');box.GetReferences().ClearReferences();box.GetReferences().AddReference(f'./cardboard{a.variant}.usda')
attr=box.GetAttribute('xformOp:translate');pos=attr.Get();attr.Set(Gf.Vec3d(pos[0],pos[1],pos[2]+(a.thickness-.003)/2))
physics=scene.GetPrimAtPath('/World/Physics')
for k,v in dict(graspForce=40.,graspGap=.23,crushForce=a.crush_force,crushGap=a.crush_gap,iterations=96,gravityRampSeconds=1.).items():physics.GetAttribute('cardboard:'+k).Set(v)
scene.GetRootLayer().Save();print('Thick board',values,'effective D',.2*scale,.1*scale)
