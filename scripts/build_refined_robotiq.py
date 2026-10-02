"""Refine the physical shell and its graphics without changing board material."""
import argparse
import json

import newton
import numpy as np
import warp as wp
from pxr import Usd, UsdGeom, Vt, Sdf

from cardboard import ROOT
from cardboard.geometry import shell_grid, bind, refine_render
from cardboard.usd_utils import register

p = argparse.ArgumentParser()
p.add_argument('--n', type=int, default=24)
p.add_argument('--render-edge', type=float, default=.006)
p.add_argument('--iterations', type=int, default=16)
p.add_argument('--substeps', type=int, default=64)
p.add_argument('--source-scene', default='assets/demo_scene_robotiq_coarse_preserved.usda')
p.add_argument('--source-asset', default='assets/cardboard_board15_half.usda')
p.add_argument('--variant', default='robotiq_fine24')
a = p.parse_args()
if a.n < 8 or a.render_edge <= 0 or a.iterations < 1 or a.substeps < 1:
    raise ValueError('Resolution and solver parameters must be positive (n>=8)')
register()
asset_path = ROOT / f'assets/cardboard_{a.variant}.usda'
scene_path = ROOT / f'assets/demo_scene_{a.variant}.usda'
source = Usd.Stage.Open(str(ROOT / a.source_asset))
source.GetRootLayer().Export(str(asset_path))
asset = Usd.Stage.Open(str(asset_path))
mesh = UsdGeom.Mesh(asset.GetPrimAtPath('/Box/SimMesh'))
material = asset.GetPrimAtPath('/Box/Materials/Cardboard')
attr = lambda n: material.GetAttribute('cardboard:' + n).Get()
old_rest = np.asarray(mesh.GetPrim().GetAttribute('cardboard:restPoints').Get())
points, faces, directions = shell_grid(np.ptp(old_rest, axis=0), a.n)
points += (old_rest.min(0) + old_rest.max(0)) * .5
builder = newton.ModelBuilder()
builder.add_cloth_mesh(pos=wp.vec3(0), rot=wp.quat_identity(), scale=1,
                       vel=wp.vec3(0), vertices=points, indices=faces.ravel(),
                       density=attr('arealDensity'), edge_ke=1.)
edges = np.asarray(builder.edge_indices, dtype=np.int32)
edge = points[edges[:, 3]] - points[edges[:, 2]]
length = np.linalg.norm(edge, axis=1)
dual = sum(np.linalg.norm(np.cross(points[edges[:, i]] - points[edges[:, 2]], edge), axis=1)
           for i in (0, 1)) / (2 * length)
scale = (attr('thickness') / attr('bendingReferenceThickness')) ** attr('bendingThicknessExponent')
weight = (edge[:, 1] / length) ** 2
stiffness = (attr('bendingCD') + (attr('bendingMD') - attr('bendingCD')) * weight) * scale / dual
mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points))
mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(faces.ravel()))
values = dict(restPoints=Vt.Vec3fArray.FromNumpy(points),
              hingeIndices=Vt.Vec4iArray.FromNumpy(edges),
              referenceAngles=list(builder.edge_rest_angle), dualWidths=dual.tolist(),
              edgeStiffness=stiffness.tolist(), materialDirection=Vt.Vec3fArray.FromNumpy(directions),
              velocities=Vt.Vec3fArray.FromNumpy(np.zeros_like(points)), solver='newtonVBD16')
for name in ('plasticAngles', 'accumulatedAngles', 'plasticDissipation', 'damage'):
    values[name] = [0.] * len(edges)
for name, value in values.items():
    mesh.GetPrim().GetAttribute('cardboard:' + name).Set(value)
render = UsdGeom.Mesh(asset.GetPrimAtPath('/Box/RenderMesh'))
graphics = np.asarray(render.GetPointsAttr().Get())
triangles = np.asarray(render.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
uv = UsdGeom.PrimvarsAPI(render).GetPrimvar('st')
graphics, triangles, coords = refine_render(graphics, triangles, np.asarray(uv.ComputeFlattened()), a.render_edge)
indices, weights, offsets = bind(graphics, points, faces)
render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(graphics))
render.GetFaceVertexCountsAttr().Set([3] * len(triangles))
render.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(triangles.ravel()))
uv.Set(Vt.Vec2fArray.FromNumpy(coords))
for name, value in [('bindingIndices', Vt.Vec3iArray.FromNumpy(indices)),
                    ('bindingWeights', Vt.Vec3fArray.FromNumpy(weights)),
                    ('bindingOffsets', Vt.Vec3fArray.FromNumpy(offsets))]:
    render.GetPrim().GetAttribute('cardboard:' + name).Set(value)
asset.GetRootLayer().Save()
scene = Usd.Stage.Open(str(ROOT / a.source_scene))
scene.GetRootLayer().Export(str(scene_path))
scene = Usd.Stage.Open(str(scene_path))
box = scene.GetPrimAtPath('/World/Box')
box.GetReferences().ClearReferences()
box.GetReferences().AddReference('./' + asset_path.name)
physics = scene.GetPrimAtPath('/World/Physics')
physics.GetAttribute('cardboard:iterations').Set(a.iterations)
physics.GetAttribute('cardboard:substeps').Set(a.substeps)
rest_exclusion=float(attr('thickness') * (attr('selfContactThicknessScale') or .5)) + 1e-5
physics.CreateAttribute('cardboard:selfContactRestExclusion', Sdf.ValueTypeNames.Float, custom=True).Set(rest_exclusion)
scene.GetRootLayer().Save()
report = dict(scene=str(scene_path.relative_to(ROOT)), asset=str(asset_path.relative_to(ROOT)),
              panel_subdivisions=a.n, simulation_vertices=len(points), physics_triangles=len(faces),
              hinges=len(edges), render_vertices=len(graphics), render_triangles=len(triangles),
              render_max_edge_m=a.render_edge, iterations=a.iterations, substeps=a.substeps,
              self_contact_rest_exclusion_m=rest_exclusion,
              source_scene=a.source_scene, source_asset=a.source_asset,
              qualification='Same material and outer source appearance; fresh physics required for new topology.')
(ROOT / f'assets/build_report_{a.variant}.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
