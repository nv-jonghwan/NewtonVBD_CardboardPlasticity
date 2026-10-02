"""Same settled box, mesh, contacts, iterations and trajectory command per runtime.

Warm CUDA graphs/JIT before timing. Excludes render and file writing, includes
advance()'s CPU control/diagnostics. This is a stationary scene benchmark, not
an estimate of speedup for all simulation workloads.
"""
import argparse,json,time,platform
from importlib.metadata import version
from pathlib import Path
import numpy as np
import warp as wp
from cardboard.scenario import PickCrushDrop
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--warmup',type=int,default=60);p.add_argument('--frames',type=int,default=30);p.add_argument('--blocks',type=int,default=3);a=p.parse_args()
s=PickCrushDrop(scene_path=Path(__file__).resolve().parents[1]/'assets/demo_scene_plate_crease.usda')
command=s.command;s.command=lambda t:command(0.)
for _ in range(a.warmup):s.advance()
wp.synchronize();times=[]
for block in range(a.blocks):
 start=time.perf_counter()
 for _ in range(a.frames):s.advance()
 wp.synchronize();times.append((time.perf_counter()-start)/a.frames)
 print('benchmark block',block,'seconds_per_frame',times[-1],flush=True)
r=dict(newton=version('newton'),warp=version('warp-lang'),python=platform.python_version(),device=str(wp.get_device()),iterations=s.iterations,substeps=s.substeps,vertices=s.model.particle_count,warmup_frames=a.warmup,frames_per_block=a.frames,seconds_per_frame=times,median_seconds_per_frame=float(np.median(times)),last=s.rows[-1],scope='Stationary home-command scene; warm JIT/graph, host control and diagnostics included, rendering/files excluded.')
Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)
