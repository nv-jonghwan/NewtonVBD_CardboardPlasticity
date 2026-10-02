"""Equivalent multilayer board; similar effective material, increased total gauge.

This is an explicit homogenized assumption, not a measured corrugated grade.
Unlike increasing core spacing alone, the amount of paper per area also grows.
"""
import argparse
import numpy as np
from pxr import Usd, UsdGeom, Gf, Vt
from cardboard import ROOT
from cardboard.usd_utils import register
from cardboard.geometry import bind

p = argparse.ArgumentParser()
p.add_argument('--variant', default='multilayer15')
p.add_argument('--thickness', type=float, default=.015)
p.add_argument('--iterations', type=int, default=8)
p.add_argument('--substeps', type=int, default=8)
p.add_argument('--preserve-outer-size', action='store_true')
p.add_argument('--translation-block', action='store_true')
p.add_argument('--grasp-force', type=float, default=40.)
p.add_argument('--crush-force', type=float, default=150.)
p.add_argument('--crush-gap', type=float, default=.09)
p.add_argument('--grasp-gap', type=float, default=.23)
p.add_argument('--finger-kp', type=float, default=1200.)
p.add_argument('--finger-kd', type=float, default=12.)
a = p.parse_args()
register()
source = Usd.Stage.Open(str(ROOT/'assets/cardboard_realtime.usda'))
source.GetRootLayer().Export(str(ROOT/f'assets/cardboard_{a.variant}.usda'))
asset = Usd.Stage.Open(str(ROOT/f'assets/cardboard_{a.variant}.usda'))
mat = asset.GetPrimAtPath('/Box/Materials/Cardboard')
get = lambda k: mat.GetAttribute('cardboard:'+k).Get()
old_thickness = get('thickness')
ratio = a.thickness/old_thickness
old_scale = (old_thickness/get('bendingReferenceThickness'))**get('bendingThicknessExponent')
# Re-anchor at the actual 5mm baseline. D ~ t^3, A ~ t, mass/area ~ t.
values = dict(thickness=a.thickness, arealDensity=get('arealDensity')*ratio,
              membraneShear=get('membraneShear')*ratio,
              membraneArea=get('membraneArea')*ratio,
              bendingReferenceThickness=old_thickness, bendingThicknessExponent=3.,
              bendingMD=get('bendingMD')*old_scale, bendingCD=get('bendingCD')*old_scale,
              yieldCurvature=get('yieldCurvature')/ratio)
for k, v in values.items():
    mat.GetAttribute('cardboard:'+k).Set(v)
mat.SetCustomDataByKey('equivalentBoardAssumption', 'Similar homogenized multilayer material: A and areal mass scale with thickness; D with thickness cubed; constant outer-fiber yield strain. No resolved flute crushing or delamination.')
mesh = asset.GetPrimAtPath('/Box/SimMesh')
points = np.asarray(mesh.GetAttribute('cardboard:restPoints').Get())
edges = np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get())
if a.preserve_outer_size:
    # Keep the NVIDIA exterior and UVs; place the physical mid-surface inside it.
    half = np.max(np.abs(points), axis=0)
    points = (points * ((half-a.thickness/2)/half)).astype(np.float32)
    mesh.GetAttribute('cardboard:restPoints').Set(Vt.Vec3fArray.FromNumpy(points))
    UsdGeom.Mesh(mesh).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points))
    e = points[edges[:,3]]-points[edges[:,2]]
    dual = sum(np.linalg.norm(np.cross(points[edges[:,i]]-points[edges[:,2]],e),axis=1)
               for i in [0,1])/(2*np.linalg.norm(e,axis=1))
    mesh.GetAttribute('cardboard:dualWidths').Set(dual.tolist())
    r = UsdGeom.Mesh(asset.GetPrimAtPath('/Box/RenderMesh'))
    indices,weights,offsets = bind(np.asarray(r.GetPointsAttr().Get()), points,
                                 np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3))
    for name,value in [('bindingIndices',Vt.Vec3iArray.FromNumpy(indices)),
                       ('bindingWeights',Vt.Vec3fArray.FromNumpy(weights)),
                       ('bindingOffsets',Vt.Vec3fArray.FromNumpy(offsets))]:
        r.GetPrim().GetAttribute('cardboard:'+name).Set(value)
edge = points[edges[:,3]]-points[edges[:,2]]
weight = (edge[:,1]/np.linalg.norm(edge, axis=1))**2
stiffness = (get('bendingCD')+(get('bendingMD')-get('bendingCD'))*weight)*ratio**3
mesh.GetAttribute('cardboard:edgeStiffness').Set((stiffness/np.asarray(mesh.GetAttribute('cardboard:dualWidths').Get())).tolist())
asset.GetRootLayer().Save()
scene = Usd.Stage.Open(str(ROOT/'assets/demo_scene_realtime.usda'))
scene.GetRootLayer().Export(str(ROOT/f'assets/demo_scene_{a.variant}.usda'))
scene = Usd.Stage.Open(str(ROOT/f'assets/demo_scene_{a.variant}.usda'))
box = scene.GetPrimAtPath('/World/Box')
box.GetReferences().ClearReferences()
box.GetReferences().AddReference(f'./cardboard_{a.variant}.usda')
translation = box.GetAttribute('xformOp:translate')
pos = translation.Get()
z_shift = -old_thickness/2 if a.preserve_outer_size else (a.thickness-old_thickness)/2
translation.Set(Gf.Vec3d(pos[0], pos[1], pos[2]+z_shift))
physics = scene.GetPrimAtPath('/World/Physics')
for k,v in dict(iterations=a.iterations,substeps=a.substeps,translationBlockSolve=a.translation_block,
                graspForce=a.grasp_force,crushForce=a.crush_force,crushGap=a.crush_gap,graspGap=a.grasp_gap,
                fingerPositionStiffness=a.finger_kp,fingerVelocityDamping=a.finger_kd).items():
    physics.GetAttribute('cardboard:'+k).Set(v)
scene.GetRootLayer().Save()
print(values, 'effective D:',get('bendingMD')*ratio**3,get('bendingCD')*ratio**3)
