"""Author the validated interactive approximation alongside the dense reference."""
import numpy as np
from pxr import Usd
from cardboard import ROOT
from cardboard.build_asset import build
build(n=8,variant='_realtime',render_max_edge=.012)
reference=Usd.Stage.Open(str(ROOT/'assets/cardboard_thick16_stronger.usda'))
asset=Usd.Stage.Open(str(ROOT/'assets/cardboard_realtime.usda'))
source=reference.GetPrimAtPath('/Box/Materials/Cardboard');material=asset.GetPrimAtPath('/Box/Materials/Cardboard')
for attr in source.GetAttributes():
 if attr.GetName().startswith('cardboard:'):material.GetAttribute(attr.GetName()).Set(attr.Get())
mesh=asset.GetPrimAtPath('/Box/SimMesh');mesh.GetAttribute('cardboard:solver').Set('newtonVBD16')
points=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get());edges=np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get());edge=points[edges[:,3]]-points[edges[:,2]]
a=lambda k:material.GetAttribute('cardboard:'+k).Get()
scale=(a('thickness')/a('bendingReferenceThickness'))**a('bendingThicknessExponent')
stiffness=(a('bendingCD')+(a('bendingMD')-a('bendingCD'))*(edge[:,1]/np.linalg.norm(edge,axis=1))**2)*scale/np.asarray(mesh.GetAttribute('cardboard:dualWidths').Get())
mesh.GetAttribute('cardboard:edgeStiffness').Set(stiffness.tolist())
r=asset.GetPrimAtPath('/Box/RenderMesh');src=reference.GetPrimAtPath('/Box/RenderMesh')
for name in ['surfaceInterpolation','visualCreaseAngleDegrees','visualCreaseTransitionDegrees']:r.GetAttribute('cardboard:'+name).Set(src.GetAttribute('cardboard:'+name).Get())
asset.GetRootLayer().Save()
scene=Usd.Stage.Open(str(ROOT/'assets/demo_scene_thick16_stronger.usda'));scene.GetRootLayer().Export(str(ROOT/'assets/demo_scene_realtime.usda'));scene=Usd.Stage.Open(str(ROOT/'assets/demo_scene_realtime.usda'))
box=scene.GetPrimAtPath('/World/Box');box.GetReferences().ClearReferences();box.GetReferences().AddReference('./cardboard_realtime.usda')
p=scene.GetPrimAtPath('/World/Physics')
for k,v in dict(iterations=8,substeps=8,kinematicArm=True,precomputeCommands=True,realtimePacing=True,rigidCompliantALM=False,contactStiffness=180000.,contactDamping=.9,openGap=.35,releaseRampFraction=.25).items():p.GetAttribute('cardboard:'+k).Set(v)
scene.GetRootLayer().Save();print('Realtime386-node shell; dense57280-triangle display;480Hz substeps; dynamic fingers; prescribed arm.')
