"""Matched GPU timing and force diagnostics; fixed poses, pristine history.

This isolates internal-force integration, not whole-trajectory correctness.
The full force and sampled force use identical pose/material/inertia inputs.
"""
import argparse,json,time
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.small_bend_trial import SmallBendPickCrushDrop
from cardboard.rom_solver import ReducedCorrection
from cardboard.representative_elements import RepresentativeElements

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--device',default='cuda:0')
a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=False)
s=SmallBendPickCrushDrop(device=a.device,scene_path=ROOT/'assets/demo_scene_robotiq_board4_graded.usda',
    small_bend_options=dict(scale=16.,knee=.75,end=14.75))
s.solver.particle_enable_self_contact=False
basis=np.load(ROOT/'outputs/graded_board4/basis8.npz')['basis'];flat=basis.transpose(0,2,1).reshape(-1,8)
rom=ReducedCorrection(s.model,basis);trajectory=np.load(ROOT/'outputs/graded_board4/graded48/trajectory.npz')
report={'qualification':'Fixed trajectory poses with pristine material history; no contacts. Isolates internal force cost and spatial quadrature error, not full simulation accuracy.','poses':{}}
for when in (5.,9.,12.):
    q=trajectory['points'][np.argmin(abs(trajectory['t']-when))]
    s.a.particle_q.assign(q);s.solver.particle_q_prev.assign(q);s.solver.inertia.assign(q)
    reference=None;results={}
    for fraction in (None,1.,.5,.25,.125):
        rom.representatives=None if fraction is None else RepresentativeElements(s.model,fraction)
        rom.residual(s.solver,s.a.particle_q,s.a,None,s.dt)
        with wp.ScopedCapture(device=a.device) as capture:rom.residual(s.solver,s.a.particle_q,s.a,None,s.dt)
        for _ in range(5):wp.capture_launch(capture.graph)
        samples=[]
        for repeat in range(5):
            start=wp.Event(a.device,enable_timing=True);end=wp.Event(a.device,enable_timing=True)
            wp.record_event(start)
            for _ in range(50):wp.capture_launch(capture.graph)
            wp.record_event(end);wp.synchronize_device(a.device)
            samples.append(wp.get_event_elapsed_time(start,end)/50)
        f=rom.force.numpy();projected=flat.T@f.ravel()
        if reference is None:reference=projected
        results[str(fraction)]=dict(gpu_ms_median=float(np.median(samples)),gpu_ms_samples=samples,
            projected_force_relative_error=float(np.linalg.norm(projected-reference)/max(np.linalg.norm(reference),1e-12)),
            projected_force_norm=float(np.linalg.norm(projected)),net_internal_force_norm=float(np.linalg.norm(f.sum(0))),
            triangles=s.model.tri_count if fraction is None else rom.representatives.tri_ids.size,
            hinges=s.model.edge_count if fraction is None else rom.representatives.edge_ids.size)
    report['poses'][str(when)]=results
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
