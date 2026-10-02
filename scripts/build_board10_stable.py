"""Fine shell at 2/3 gauge, preserving the source exterior and all mesh counts."""
import json
import numpy as np
from pxr import Usd,UsdGeom,Vt,Sdf
from cardboard import ROOT
from cardboard.geometry import bind
from cardboard.usd_utils import register
register()
source=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_creased.usda'))
asset_path=ROOT/'assets/cardboard_robotiq_board10.usda'
asset=Usd.Stage.Open(str(ROOT/'assets/cardboard_robotiq_fine24.usda'))
asset.GetRootLayer().Export(str(asset_path));asset=Usd.Stage.Open(str(asset_path))
source_mat=source.GetPrimAtPath('/World/Box/Materials/Cardboard');mat=asset.GetPrimAtPath('/Box/Materials/Cardboard')
get=lambda n:source_mat.GetAttribute('cardboard:'+n).Get()
ratio=2/3;old_thickness=get('thickness');thickness=old_thickness*ratio
# Preserve the accepted crease law overrides in the independent asset.
for attr in source_mat.GetAttributes():
    if attr.GetName().startswith('cardboard:') and attr.Get() is not None:
        dest=mat.GetAttribute(attr.GetName()) or mat.CreateAttribute(attr.GetName(),attr.GetTypeName(),custom=True)
        dest.Set(attr.Get())
changes={'thickness':thickness,'arealDensity':get('arealDensity')*ratio,
         'membraneShear':get('membraneShear')*ratio,'membraneArea':get('membraneArea')*ratio,
         'yieldCurvature':get('yieldCurvature')/ratio,
         'creaseDamageLength':get('creaseDamageLength')*ratio,'internalVelocityDamping':60.,'supportedInternalDamping':3000.}
for name,value in changes.items():
    dest=mat.GetAttribute('cardboard:'+name) or mat.CreateAttribute('cardboard:'+name,Sdf.ValueTypeNames.Float,custom=True)
    dest.Set(value)
mesh=UsdGeom.Mesh(asset.GetPrimAtPath('/Box/SimMesh'));prim=mesh.GetPrim()
rest=np.asarray(prim.GetAttribute('cardboard:restPoints').Get(),np.float32)
center=(rest.min(0)+rest.max(0))*.5;half=np.ptp(rest,axis=0)*.5
points=(center+(rest-center)*(half+(old_thickness-thickness)*.5)/half).astype(np.float32)
mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points));prim.GetAttribute('cardboard:restPoints').Set(Vt.Vec3fArray.FromNumpy(points))
edges=np.asarray(prim.GetAttribute('cardboard:hingeIndices').Get());e=points[edges[:,3]]-points[edges[:,2]];length=np.linalg.norm(e,axis=1)
dual=sum(np.linalg.norm(np.cross(points[edges[:,i]]-points[edges[:,2]],e),axis=1) for i in [0,1])/(2*length)
weight=(e[:,1]/length)**2
ke=(get('bendingCD')+(get('bendingMD')-get('bendingCD'))*weight)*(thickness/get('bendingReferenceThickness'))**get('bendingThicknessExponent')/dual
prim.GetAttribute('cardboard:dualWidths').Set(dual.tolist());prim.GetAttribute('cardboard:edgeStiffness').Set(ke.tolist())
render=UsdGeom.Mesh(asset.GetPrimAtPath('/Box/RenderMesh'));rp=render.GetPrim();graphics=np.asarray(render.GetPointsAttr().Get());faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
indices,weights,offsets=bind(graphics,points,faces)
for name,value in [('bindingIndices',Vt.Vec3iArray.FromNumpy(indices)),('bindingWeights',Vt.Vec3fArray.FromNumpy(weights)),('bindingOffsets',Vt.Vec3fArray.FromNumpy(offsets))]:rp.GetAttribute('cardboard:'+name).Set(value)
for name in ['visualCreaseAngleDegrees','visualCreaseTransitionDegrees','visualSmoothThickness']:
    src=source.GetPrimAtPath('/World/Box/RenderMesh').GetAttribute('cardboard:'+name)
    dest=rp.GetAttribute('cardboard:'+name) or rp.CreateAttribute('cardboard:'+name,src.GetTypeName(),custom=True)
    dest.Set(src.Get())
asset.GetRootLayer().Save()
scene_path=ROOT/'assets/demo_scene_robotiq_board10.usda';source.GetRootLayer().Export(str(scene_path));scene=Usd.Stage.Open(str(scene_path))
box=scene.GetPrimAtPath('/World/Box');box.GetReferences().ClearReferences();box.GetReferences().AddReference('./'+asset_path.name)
# Scene overrides take precedence over the referenced material.
material=scene.GetPrimAtPath('/World/Box/Materials/Cardboard')
for name,value in changes.items():
    dest=material.GetAttribute('cardboard:'+name) or material.CreateAttribute('cardboard:'+name,Sdf.ValueTypeNames.Float,custom=True)
    dest.Set(value)
physics=scene.GetPrimAtPath('/World/Physics')
settings={'iterations':48,'substeps':16,'particleSolveInterval':3,'rotationBlockSolve':True,'supportedSleep':True,'restRefinement':True,'coupledSupportedSolve':True,'selfContactRestExclusion':thickness+1e-5}
for name,value in settings.items():
    kind=Sdf.ValueTypeNames.Bool if isinstance(value,bool) else (Sdf.ValueTypeNames.Int if isinstance(value,int) else Sdf.ValueTypeNames.Float)
    dest=physics.GetAttribute('cardboard:'+name) or physics.CreateAttribute('cardboard:'+name,kind,custom=True);dest.Set(value)
scene.GetRootLayer().Save()
report={'scene':str(scene_path.relative_to(ROOT)),'asset':str(asset_path.relative_to(ROOT)),'ratio':ratio,'material_changes':changes,'settings':settings,'physics_vertices':len(points),'physics_triangles':len(faces),'render_vertices':len(graphics),'render_triangles':len(render.GetFaceVertexIndicesAttr().Get())//3,'equivalent_bending_ratio':ratio**3,'qualification':'Same homogenized board at lower gauge: areal mass/membrane scale with t, D with t^3, yield curvature inversely with t. Outer graphics and topology preserved; midsurface/binding rebuilt. Illustrative, not measured corrugation.'}
(ROOT/'assets/build_report_robotiq_board10.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
