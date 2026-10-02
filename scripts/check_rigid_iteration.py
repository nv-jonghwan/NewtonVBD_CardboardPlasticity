"""Compare complete rigid iterations from identical loaded states, then time them."""
import argparse,json,os,time
from pathlib import Path
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--time',type=float,default=11.);p.add_argument('--device',default='cuda:1');a=p.parse_args()
out=ROOT/a.output;out.mkdir(parents=True,exist_ok=False)
os.environ['CARDBOARD_VBD_SCHEDULE']='baseline'
s=PickCrushDrop(scene_path=ROOT/'assets/demo_scene_robotiq_board10_primitive.usda',device=a.device)
while s.time<a.time-1e-9:s.advance()
wp.synchronize()
solver=s.solver
# Solver outputs are flat arrays; capture all to include every contact/joint dual.
arrays={}
for prefix,obj in [('solver',solver),('a',s.a),('b',s.b)]:
    for name,value in vars(obj).items():
        if isinstance(value,wp.array) and value.size and value.device.is_cuda:
            arrays.setdefault(value.ptr,(prefix+'.'+name,value,wp.clone(value)))
outputs=['body_forces','body_torques','body_hessian_ll','body_hessian_al','body_hessian_aa',
         'body_body_contact_penalty_k','body_particle_contact_penalty_k','body_body_contact_lambda',
         'joint_penalty_k','joint_lambda_lin','joint_lambda_ang','joint_drive_lambda','joint_limit_lambda']

def reset():
    for _,value,snapshot in arrays.values():wp.copy(value,snapshot)

def evaluate():solver._solve_rigid_body_iteration(s.a,s.b,s.control,s.contacts,s.dt)

def read_outputs():
    return {'body_q':s.a.body_q.numpy(),**{k:getattr(solver,k).numpy() for k in outputs}}

results={};reference=None
for mode in ['baseline','guarded']:
    solver.contact_schedule=mode;solver.contact_workers=4
    reset();evaluate();wp.synchronize();actual=read_outputs()
    if reference is None:reference={k:v.copy() for k,v in actual.items()}
    differences={k:dict(max_absolute=float(np.max(np.abs(v-reference[k]))) if v.size else 0.,
                        equal=bool(np.allclose(v,reference[k],rtol=1e-5,atol=1e-5))) for k,v in actual.items()}
    with wp.ScopedCapture() as capture:
        for _ in range(32):reset();evaluate()
    for _ in range(3):wp.capture_launch(capture.graph)
    start=wp.Event(enable_timing=True);end=wp.Event(enable_timing=True);times=[]
    for _ in range(9):
        wp.record_event(start);wp.capture_launch(capture.graph);wp.record_event(end);wp.synchronize()
        times.append(wp.get_event_elapsed_time(start,end)*1000/32)
    results[mode]=dict(outputs=differences,median_us=float(np.median(times)),samples_us=times)
reset()
report=dict(t=s.time,device=a.device,results=results,passed=all(v['equal'] for r in results.values() for v in r['outputs'].values()),
            observed_speedup=results['baseline']['median_us']/results['guarded']['median_us'],
            caveat='Frozen complete rigid iteration; includes restoration of all flat solver/state arrays. Preserves rigid color ordering and compares all force/Hessian/joint/contact dual outputs.')
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
