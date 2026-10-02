"""Summarize full-run speed and unchanged existing quality gates."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from cardboard import ROOT

p=argparse.ArgumentParser()
p.add_argument('--baseline',default='outputs/robotiq_creased')
p.add_argument('--candidate',default='outputs/robotiq_optimized')
a=p.parse_args();base=ROOT/a.baseline;candidate=ROOT/a.candidate
read=lambda folder,name:json.loads((folder/(name+'.json')).read_text())
bv=read(base,'validation');cv=read(candidate,'validation')
bc=read(base,'crease_comparison')['candidate'];cc=read(candidate,'crease_comparison')
kernel=read(ROOT/'outputs/optimization','kernel_equivalence')
surface=read(ROOT/'outputs/optimization','surface_benchmark')
config=[read(folder,'run_config') for folder in (base,candidate)]
same_scene=(ROOT/'assets/demo_scene_robotiq_creased.usda').read_bytes()==(ROOT/'outputs/optimization/before/scene.usda').read_bytes()
with np.load(base/'trajectory.npz') as b,np.load(candidate/'trajectory.npz') as c:
    same_shape=b['points'].shape==c['points'].shape
    same_initial=np.array_equal(b['points'][0],c['points'][0])
checks=dict(sequence=cv['passed'],crease_and_recoil=cc['passed'],kernel_equivalence=kernel['passed'],
            display_equivalence=surface['passed'],same_scene_material_and_render=same_scene,
            same_physical_resolution_and_initial_state=bool(same_shape and same_initial),
            same_iterations_and_substeps=all(config[0][k]==config[1][k] for k in ['iterations','substeps']),
            faster_full_sequence=cv['active_wall_s']<bv['active_wall_s'])
report=dict(passed=all(checks.values()),checks=checks,baseline=a.baseline,candidate=a.candidate,
            old_wall_s=bv['active_wall_s'],new_wall_s=cv['active_wall_s'],
            physics_speedup=bv['active_wall_s']/cv['active_wall_s'],
            physics_time_reduction_percent=100*(1-cv['active_wall_s']/bv['active_wall_s']),
            surface=surface,sequence=cv,
            prior_creased_motion={k:bc[k] for k in ['panel_flutter','held_recoil']},
            optimized_motion={k:cc['candidate'][k] for k in ['panel_flutter','held_recoil']},
            scene_sha256=hashlib.sha256((ROOT/'assets/demo_scene_robotiq_creased.usda').read_bytes()).hexdigest(),
            note='Full worker timing includes frame diagnostics and snapshots; startup before active loop is excluded. Floating-point contact reduction order changes, so trajectories need not be bit-identical. All previous sequence and crease/recoil gates retained.')
(ROOT/'outputs/optimization/report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
