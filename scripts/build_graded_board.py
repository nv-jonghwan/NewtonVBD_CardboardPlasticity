"""Build independent 4 mm board scenes with fresh topology/history/bindings."""
import argparse
import json
from pathlib import Path
import newton
import numpy as np
import warp as wp
from pxr import Usd, UsdGeom, Vt, Sdf
from cardboard import ROOT
from cardboard.geometry import shell_grid, graded_shell_grid, source_render, refine_render, bind
from cardboard.usd_utils import register

p = argparse.ArgumentParser(__doc__)
p.add_argument('--source', default='assets/demo_scene_robotiq_board10_secure_tilt.usda')
p.add_argument('--variant', required=True)
p.add_argument('--fine-reference', action='store_true')
p.add_argument('--counts', type=int, nargs=3, default=[18,14,14])
p.add_argument('--thickness', type=float, default=.004)
p.add_argument('--render-edge', type=float, default=.012)
a = p.parse_args()
if not (0 < a.thickness <= .02 and a.render_edge > 0):raise ValueError('Invalid thickness/render edge')
register()
source = Usd.Stage.Open(str(ROOT/a.source))
dest = ROOT/f'assets/demo_scene_{a.variant}.usda'
if dest.exists():raise FileExistsError(dest)
stage = Usd.Stage.CreateNew(str(dest))
stage.GetRootLayer().subLayerPaths = [str(Path(a.source).name)]
UsdGeom.SetStageMetersPerUnit(stage, 1.)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/SimMesh'))
prim = mesh.GetPrim()
mat = stage.GetPrimAtPath(prim.GetRelationship('cardboard:material').GetTargets()[0])
old = lambda name: source.GetPrimAtPath(str(mat.GetPath())).GetAttribute('cardboard:'+name).Get()
def set_value(prim, name, value):
    attr = prim.GetAttribute('cardboard:'+name)
    if not attr:raise ValueError('Unknown schema attribute '+name)
    attr.Set(value)
ratio = a.thickness/old('thickness')
changes = {'thickness':a.thickness, 'arealDensity':old('arealDensity')*ratio,
           'membraneShear':old('membraneShear')*ratio, 'membraneArea':old('membraneArea')*ratio,
           'yieldCurvature':old('yieldCurvature')/ratio,
           'creaseDamageLength':old('creaseDamageLength')*ratio}
for name, value in changes.items():set_value(mat,name,value)
old_points = np.asarray(prim.GetAttribute('cardboard:restPoints').Get(),float)
center = (old_points.min(0)+old_points.max(0))*.5
# Keep the physical exterior fixed while changing the midsurface and radius.
size = np.ptp(old_points,axis=0)+old('thickness')-a.thickness
points, faces, directions = shell_grid(size,24) if a.fine_reference else graded_shell_grid(size,a.counts)
points += center.astype(np.float32)
b = newton.ModelBuilder()
b.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),
                 vertices=points,indices=faces.ravel(),density=changes['arealDensity'],edge_ke=1.)
edges = np.asarray(b.edge_indices,np.int32)
edge = points[edges[:,3]]-points[edges[:,2]]
length = np.linalg.norm(edge,axis=1)
dual = sum(np.linalg.norm(np.cross(points[edges[:,i]]-points[edges[:,2]],edge),axis=1)
           for i in (0,1))/(2*length)
factor = (a.thickness/old('bendingReferenceThickness'))**old('bendingThicknessExponent')
weight = (edge[:,1]/length)**2
stiffness = (old('bendingCD')+(old('bendingMD')-old('bendingCD'))*weight)*factor/dual
mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points))
mesh.GetFaceVertexCountsAttr().Set([3]*len(faces))
mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(faces.ravel()))
mesh.GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([points.min(0),points.max(0)],np.float32)))
values = {'restPoints':Vt.Vec3fArray.FromNumpy(points), 'hingeIndices':Vt.Vec4iArray.FromNumpy(edges),
          'referenceAngles':list(b.edge_rest_angle), 'dualWidths':dual.tolist(),
          'edgeStiffness':stiffness.tolist(), 'materialDirection':Vt.Vec3fArray.FromNumpy(directions),
          'velocities':Vt.Vec3fArray.FromNumpy(np.zeros_like(points)), 'solver':'newtonVBD16'}
values.update({key:[0.]*len(edges) for key in ('plasticAngles','accumulatedAngles','plasticDissipation','damage')})
for name, value in values.items():set_value(prim,name,value)
source_path = prim.GetAttribute('cardboard:sourceAsset').Get()
render_points, render_faces, uv, _ = source_render(source_path.resolvedPath,scale=prim.GetAttribute('cardboard:sourceScale').Get(),levels=0)
render_points, render_faces, uv = refine_render(render_points,render_faces,uv,a.render_edge)
indices, weights, offsets = bind(render_points,points,faces)
render = UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/RenderMesh'))
render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(render_points))
render.GetFaceVertexCountsAttr().Set([3]*len(render_faces))
render.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(render_faces.ravel()))
UsdGeom.PrimvarsAPI(render).GetPrimvar('st').Set(Vt.Vec2fArray.FromNumpy(uv))
for name,value in [('bindingIndices',Vt.Vec3iArray.FromNumpy(indices)),
                   ('bindingWeights',Vt.Vec3fArray.FromNumpy(weights)),
                   ('bindingOffsets',Vt.Vec3fArray.FromNumpy(offsets))]:set_value(render.GetPrim(),name,value)
set_value(stage.GetPrimAtPath('/World/Physics'),'selfContactRestExclusion',a.thickness+1e-5)
stage.GetRootLayer().Save()
tri = points[faces];edge_lengths = np.linalg.norm(np.roll(tri,-1,axis=1)-tri,axis=2)
area = np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1).sum()*.5
report = dict(scene=str(dest.relative_to(ROOT)),source=a.source,physical_vertices=len(points),
              physical_triangles=len(faces),hinges=len(edges),counts=[24]*3 if a.fine_reference else a.counts,
              graded=not a.fine_reference,render_vertices=len(render_points),render_triangles=len(render_faces),
              material_changes=changes,bending_ratio=ratio**old('bendingThicknessExponent'),
              small_bend=dict(scale=16.,knee=.3/ratio,end=5.9/ratio),
              mass_kg=float(area*changes['arealDensity']),midsurface_size_m=size.tolist(),
              physical_edge_mm=dict(min=float(edge_lengths.min()*1000),max=float(edge_lengths.max()*1000)),
              max_edge_aspect=float((edge_lengths.max(1)/edge_lengths.min(1)).max()),
              rest_render_binding_error_m=float(np.linalg.norm((points[indices]*weights[:,:,None]).sum(1)+offsets-render_points,axis=1).max()),
              qualification='Static feature-graded shell. Fresh state required. Same exterior, density scales with t, membrane with t, bending with t^3; illustrative homogenized material, not measured corrugation.')
(ROOT/f'assets/build_report_{a.variant}.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
