"""Bounded startup cost/stability screening; not full-sequence qualification."""
import argparse,time,json
from pathlib import Path
import numpy as np
import warp as wp
from cardboard.scenario import PickCrushDrop
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--device',default='cuda:1');p.add_argument('--iterations',type=int,required=True);p.add_argument('--substeps',type=int,required=True);p.add_argument('--scene',default='assets/demo_scene_thick16_stronger.usda');p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=2.);a=p.parse_args()
s=PickCrushDrop(device=a.device,scene_path=ROOT/a.scene,iterations=a.iterations)
s.substeps=a.substeps;s.dt=1/s.fps/s.substeps
samples=[];start=time.perf_counter()
for i in range(round(a.seconds*s.fps)):
 t=time.perf_counter();row=s.advance();wp.synchronize();samples.append(time.perf_counter()-t)
 if not np.isfinite(s.a.particle_q.numpy()).all():break
r=dict(iterations=a.iterations,substeps=a.substeps,vertices=s.model.particle_count,mean_warm_ms=float(np.mean(samples[5:]))*1000,max_plastic_hinges=max(x['plastic_hinges'] for x in s.rows),last=row,wall_s=time.perf_counter()-start)
Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)
