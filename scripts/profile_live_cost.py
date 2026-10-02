"""Profile actual physics frame and cubic display cost with warmed GPU graphs."""
import time,json,cProfile,pstats,io
from pathlib import Path
import numpy as np
import warp as wp
from cardboard.scenario import PickCrushDrop
from cardboard.surface import PanelSurface
from pxr import UsdGeom
from cardboard import ROOT
s=PickCrushDrop(scene_path=ROOT/'assets/demo_scene_thick16_stronger.usda')
for _ in range(3):s.advance()
wp.synchronize();acc=[];launch=wp.capture_launch
def timed(graph):
 wp.synchronize();t=time.perf_counter();launch(graph);wp.synchronize();acc.append(time.perf_counter()-t)
wp.capture_launch=timed
pr=cProfile.Profile();pr.enable();start=time.perf_counter()
for _ in range(20):s.advance()
wp.synchronize();elapsed=time.perf_counter()-start;pr.disable();buf=io.StringIO();pstats.Stats(pr,stream=buf).sort_stats('cumtime').print_stats(25)
r=s.stage.GetPrimAtPath('/World/Box/RenderMesh');a=lambda n:r.GetAttribute('cardboard:'+n).Get()
surface=PanelSurface(s.local,s.faces,np.asarray(a('bindingIndices')),np.asarray(a('bindingWeights')),np.asarray(a('bindingOffsets')),a('visualCreaseAngleDegrees'),a('visualCreaseTransitionDegrees'),bool(a('visualSmoothThickness')))
q=s.a.particle_q.numpy()-s.center;t=time.perf_counter()
for _ in range(20):surface.evaluate(q)
result=dict(frame_ms=elapsed/20*1000,gpu_graph_ms=float(np.mean(acc))*1000,display_surface_ms=(time.perf_counter()-t)/20*1000,vertices=s.model.particle_count,iterations=s.iterations,substeps=s.substeps,profile=buf.getvalue())
(ROOT/'outputs/realtime_profile_baseline.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
