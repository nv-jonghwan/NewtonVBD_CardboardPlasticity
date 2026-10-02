"""Frozen-state kernel launch experiments, never installed in the live solver.

Active-count launches are an oracle microbenchmark, not a dynamic graph solution.
Each invocation restores private output buffers, including input/output aliases.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop

parser=argparse.ArgumentParser()
parser.add_argument('--scene',default='assets/demo_scene_robotiq_board10_primitive.usda')
parser.add_argument('--device',default='cuda:1')
parser.add_argument('--times',type=float,nargs='+',default=[.5,11.])
parser.add_argument('--output',required=True)
parser.add_argument('--parallel-contact',action='store_true')
args=parser.parse_args();out=ROOT/args.output;out.mkdir(parents=True,exist_ok=False)
s=PickCrushDrop(scene_path=ROOT/args.scene,device=args.device)
original_launch=wp.launch;results=[]
for target in sorted(args.times):
    while s.time<target-1e-9:s.advance()
    wp.synchronize_device(args.device)
    captured={}
    def capture_call(*a,**kw):
        kernel=kw.get('kernel',a[0] if a else None)
        targets=['accumulate_body_particle_contacts_per_body'] if args.parallel_contact else ['solve_rigid_body','accumulate_particle_body_contact_force_and_hessian']
        if kernel.key in targets:
            inputs=kw['inputs']
            group=int(inputs[1]) if kernel.key=='accumulate_particle_body_contact_force_and_hessian' else int(inputs[1].ptr)
            captured.setdefault((kernel.key,group),dict(kw))
        return original_launch(*a,**kw)
    state_a,state_b=s.a,s.b;wp.launch=capture_call
    try:
        with wp.ScopedCapture(device=args.device) as capture:s._integrate()
    finally:wp.launch=original_launch
    assert s.a is state_a and s.b is state_b
    active_count=int(s.contacts.soft_contact_count.numpy()[0])
    probe=dict(t=s.time,mode=s.graph_mode,soft_count=active_count,soft_capacity=s.contacts.soft_contact_max,kernels=[])
    for (name,group),call in captured.items():
        kernel=call['kernel'];source_outputs=call['outputs']
        originals=[wp.clone(a) for a in source_outputs]
        buffers=[wp.clone(a) for a in source_outputs]
        by_ptr={a.ptr:b for a,b in zip(source_outputs,buffers)}
        inputs=[by_ptr.get(a.ptr,a) if isinstance(a,wp.array) else a for a in call['inputs']]
        variants=[('default256',call['dim'],256),('block32',call['dim'],32),('block64',call['dim'],64),('block128',call['dim'],128)]
        if name.startswith('accumulate'):
            if not args.parallel_contact:variants.append(('active_count_oracle',max(active_count,1),256))
        if args.parallel_contact:
            from vbd_contact_parallel_trial import parallel_body_particle_contact
            variants=[('original4',call['dim'],256)]+[(f'workers{n}',call['inputs'][1].size*n,256) for n in [4,8,16,32,64]]
        record=dict(name=name,group=group,original_dim=int(call['dim']),variants=[])
        reference=None
        for label,dim,block in variants:
            launch_kernel=kernel;launch_inputs=inputs
            if label.startswith('workers'):
                launch_kernel=parallel_body_particle_contact
                launch_inputs=[inputs[0],int(label.removeprefix('workers')),*inputs[1:]]
            def evaluate():
                for dst,src in zip(buffers,originals):wp.copy(dst,src)
                wp.launch(launch_kernel,dim=dim,inputs=launch_inputs,outputs=buffers,device=args.device,block_dim=block)
            evaluate();wp.synchronize_device(args.device)
            actual=[a.numpy() for a in buffers]
            if reference is None:reference=[a.copy() for a in actual]
            errors=[float(np.max(np.abs(a.astype(float)-b.astype(float)))) for a,b in zip(actual,reference)]
            scales=[max(float(np.max(np.abs(a))),1e-12) for a in reference]
            passed=all(np.allclose(a,b,rtol=1e-5,atol=1e-5) for a,b in zip(actual,reference))
            batch=128
            with wp.ScopedCapture(device=args.device) as graph_capture:
                for _ in range(batch):evaluate()
            graph=graph_capture.graph
            for _ in range(3):wp.capture_launch(graph)
            wp.synchronize_device(args.device)
            begin=wp.Event(args.device,enable_timing=True);end=wp.Event(args.device,enable_timing=True)
            measurements=[]
            for _ in range(9):
                wp.record_event(begin);wp.capture_launch(graph);wp.record_event(end)
                wp.synchronize_device(args.device)
                measurements.append(wp.get_event_elapsed_time(begin,end,synchronize=False)*1000/batch)
            record['variants'].append(dict(label=label,dim=int(dim),block_dim=block,median_us=float(np.median(measurements)),
                min_us=float(min(measurements)),max_us=float(max(measurements)),samples_us=measurements,
                max_absolute_errors=errors,max_relative_to_global_scale=[e/scale for e,scale in zip(errors,scales)],passed=passed,
                compiled_properties=wp.get_cuda_kernel_properties(launch_kernel,args.device,block_dim=block)))
        probe['kernels'].append(record)
    results.append(probe)
    (out/'benchmark.json').write_text(json.dumps(dict(probes=results,
        caveats='Frozen end-of-frame state; installed kernels or isolated parameterized worker kernel, private output buffers restored per invocation. Timing includes output reset copies, graph-batched128 and9 samples. Oracle active count requires host read and is not a production graph implementation. Other-project CUDA context observed on GPU1; timings are observational. Production settings untouched.'),indent=2))
    print(json.dumps(dict(t=s.time,kernels=len(probe['kernels']),all_equal=all(v['passed'] for k in probe['kernels'] for v in k['variants']))),flush=True)
