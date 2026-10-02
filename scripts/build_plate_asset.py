"""Separate high-bending-stiffness variant; preserve previous topology/replays."""
import argparse
import numpy as np
from pxr import Usd
from cardboard import ROOT
from cardboard.usd_utils import register
parser=argparse.ArgumentParser();parser.add_argument('--variant',default='_plate_crease');parser.add_argument('--yield-curvature',type=float,default=5.);args=parser.parse_args()
register()
source=ROOT/'assets/cardboard_dense.usda'
asset=Usd.Stage.Open(str(source));asset.GetRootLayer().Export(str(ROOT/f'assets/cardboard{args.variant}.usda'))
asset=Usd.Stage.Open(str(ROOT/f'assets/cardboard{args.variant}.usda'))
material=asset.GetPrimAtPath('/Box/Materials/Cardboard')
values=dict(membraneShear=17640.,membraneArea=29400.,membraneDamping=.02,bendingMD=.2,bendingCD=.1,bendingDamping=.003,yieldCurvature=args.yield_curvature,hardeningRatio=.02,damageRate=.15,residualStiffness=.65)
for key,value in values.items():material.GetAttribute('cardboard:'+key).Set(value)
mesh=asset.GetPrimAtPath('/Box/SimMesh')
points=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get());edges=np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get());dual=np.asarray(mesh.GetAttribute('cardboard:dualWidths').Get())
e=points[edges[:,3]]-points[edges[:,2]];w=(e[:,1]/np.linalg.norm(e,axis=1))**2
mesh.GetAttribute('cardboard:edgeStiffness').Set(((values['bendingCD']+(values['bendingMD']-values['bendingCD'])*w)/dual).tolist())
render=asset.GetPrimAtPath('/Box/RenderMesh')
render.GetAttribute('cardboard:surfaceInterpolation').Set('creaseAwareCubic')
render.GetAttribute('cardboard:visualCreaseAngleDegrees').Set(45.)
asset.GetRootLayer().Save()
scene=Usd.Stage.Open(str(ROOT/'assets/demo_scene_dense.usda'));scene.GetRootLayer().Export(str(ROOT/f'assets/demo_scene{args.variant}.usda'))
scene=Usd.Stage.Open(str(ROOT/f'assets/demo_scene{args.variant}.usda'))
scene.GetPrimAtPath('/World/Box').GetReferences().ClearReferences()
scene.GetPrimAtPath('/World/Box').GetReferences().AddReference(f'./cardboard{args.variant}.usda')
physics=scene.GetPrimAtPath('/World/Physics')
for key,value in dict(graspForce=30.,graspGap=.24,crushGap=.16).items():physics.GetAttribute('cardboard:'+key).Set(value)
scene.GetRootLayer().Save()
print(values)
