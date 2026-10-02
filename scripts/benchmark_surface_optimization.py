"""Paired CPU display benchmark and geometric equivalence at 21 real frames."""
import importlib.util,json,time
import numpy as np
from pxr import Usd,UsdGeom
from cardboard import ROOT
from cardboard.surface import PanelSurface
from cardboard.usd_utils import register
register();stage=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_creased.usda'))
mesh=stage.GetPrimAtPath('/World/Box/SimMesh');prim=stage.GetPrimAtPath('/World/Box/RenderMesh')
a=lambda n:prim.GetAttribute('cardboard:'+n).Get()
rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get())
faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
args=[rest,faces,np.asarray(a('bindingIndices')),np.asarray(a('bindingWeights')),np.asarray(a('bindingOffsets')),a('visualCreaseAngleDegrees'),a('visualCreaseTransitionDegrees'),bool(a('visualSmoothThickness'))]
spec=importlib.util.spec_from_file_location('surface_before',ROOT/'outputs/optimization/before/surface.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
surfaces=[module.PanelSurface(*args),PanelSurface(*args)]
with np.load(ROOT/'outputs/robotiq_creased/trajectory.npz') as data:
    points=data['points'];center=points[0].mean(0)-rest.mean(0)
    selected=points[np.linspace(0,len(points)-1,21).astype(int)]-center
for surface in surfaces:surface.evaluate(selected[0])
times=[[],[]];position_error=0.;normal_error=0.
for i,q in enumerate(selected):
    results=[None,None]
    for j in ([0,1] if i%2 else [1,0]):
        start=time.perf_counter();results[j]=surfaces[j].evaluate(q);times[j].append((time.perf_counter()-start)*1000)
    position_error=max(position_error,float(np.max(abs(results[0][0]-results[1][0]))))
    normal_error=max(normal_error,float(np.max(abs(results[0][1]-results[1][1]))))
report=dict(passed=position_error<1e-7 and normal_error<1e-6,frames=len(selected),
            old_median_ms=float(np.median(times[0])),new_median_ms=float(np.median(times[1])),
            speedup=float(np.median(times[0])/np.median(times[1])),
            max_position_component_error_m=position_error,max_normal_component_error=normal_error,
            render_vertices=len(surfaces[1].output_indices),physical_vertices=len(rest),physical_triangles=len(faces),
            method='Alternating order, warmed evaluators, same21 full-resolution recorded frames; CPU only, no USD/RTX upload included.')
(ROOT/'outputs/optimization/surface_benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
