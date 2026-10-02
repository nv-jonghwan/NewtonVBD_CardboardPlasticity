"""Reproducible current-scene CPU/GPU/display profile (no physics shortcuts)."""
import argparse, cProfile, io, json, pstats, time
from collections import defaultdict
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop
from cardboard.surface import PanelSurface

p=argparse.ArgumentParser()
p.add_argument('--label', required=True)
p.add_argument('--frames', type=int, default=12)
p.add_argument('--kernels', action='store_true')
p.add_argument('--scene', type=str, default='assets/demo_scene_robotiq_creased.usda')
p.add_argument('--trajectory', type=str, default='outputs/robotiq_creased/trajectory.npz')
a=p.parse_args()
s=PickCrushDrop(scene_path=ROOT/a.scene)
for _ in range(3):s.advance()
wp.synchronize()
graph_times=[];launch=wp.capture_launch
def timed(graph):
    wp.synchronize();t=time.perf_counter();launch(graph);wp.synchronize()
    graph_times.append((time.perf_counter()-t)*1000)
wp.capture_launch=timed
pr=cProfile.Profile();pr.enable();start=time.perf_counter()
for _ in range(a.frames):s.advance()
wp.synchronize();elapsed=time.perf_counter()-start;pr.disable()
wp.capture_launch=launch
buf=io.StringIO();pstats.Stats(pr,stream=buf).sort_stats('cumtime').print_stats(20)
r=s.stage.GetPrimAtPath('/World/Box/RenderMesh');attr=lambda n:r.GetAttribute('cardboard:'+n).Get()
surface=PanelSurface(s.local,s.faces,np.asarray(attr('bindingIndices')),np.asarray(attr('bindingWeights')),np.asarray(attr('bindingOffsets')),attr('visualCreaseAngleDegrees'),attr('visualCreaseTransitionDegrees'),bool(attr('visualSmoothThickness')))
with np.load(ROOT/a.trajectory) as d:q=d['points'][min(354,len(d['points'])-1)]-s.center
surface.evaluate(q);display=[]
for _ in range(12):
    t=time.perf_counter();surface.evaluate(q);display.append((time.perf_counter()-t)*1000)
result=dict(frame_ms=elapsed/a.frames*1000,gpu_graph_ms=float(np.median(graph_times)),display_surface_ms=float(np.median(display)),vertices=s.model.particle_count,triangles=s.model.tri_count,render_vertices=len(surface.output_indices),iterations=s.iterations,substeps=s.substeps,soft_contact_capacity=s.contacts.soft_contact_max,particle_colors=len(s.model.particle_color_groups),tile_solve=s.solver.use_particle_tile_solve,profile=buf.getvalue())
result.update(scene=str(s.stage.GetRootLayer().realPath),trajectory=str(ROOT/a.trajectory),phase='initial settling',graph_mode=s.graph_mode,particle_solve_interval=s.solver.particle_solve_interval)
if a.kernels:
    # Eager single substep gives kernel attribution; event instrumentation and
    # Python dispatch make this different from the graph throughput above.
    s.substeps=1
    wp.timing_begin(wp.TIMING_KERNEL);s._integrate();timings=wp.timing_end()
    totals=defaultdict(float);counts=defaultdict(int)
    for t in timings:totals[t.name]+=t.elapsed;counts[t.name]+=1
    result['eager_substep_kernels']=[dict(name=k,ms=v,calls=counts[k]) for k,v in sorted(totals.items(),key=lambda x:-x[1])]
out=ROOT/'outputs/optimization'/f'profile_{a.label}.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
