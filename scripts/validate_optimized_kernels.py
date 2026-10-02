"""Compare scheduling-only overlays to the installed Newton 1.6 equations."""
import importlib.util, json, time
import numpy as np
import warp as wp
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop
from cardboard.compact_contact import accumulate_compact_self_contact, truncate_compact_pairs, PAIR_THREADS
from newton._src.solvers.vbd.particle_vbd_kernels import (
    accumulate_self_contact_force_and_hessian, apply_planar_truncation_parallel_by_collision,
)
from cardboard.block_solver import inertia_block, solve_block, BLOCK_PARTIALS
from cardboard.surface import PanelSurface

s=PickCrushDrop(scene_path=ROOT/'assets/demo_scene_robotiq_creased.usda');v=s.solver;m=s.model
rng=np.random.default_rng(14);checks={};metrics={}
with np.load(ROOT/'outputs/robotiq_creased/trajectory.npz') as d:
    samples=[d['points'][i].copy() for i in [0,354,480]]
for sample,q in enumerate(samples):
    s.a.particle_q.assign(q);v._collision_detection_penetration_free(s.a)
    previous=wp.array(q+rng.normal(0,1e-5,q.shape).astype(np.float32),dtype=wp.vec3,device=v.device)
    info=v.trimesh_collision_detector.collision_info
    counts=v.compact_counts.numpy();metrics[f'pairs_{sample}']=counts.tolist()
    for kind,packed,capacity,offsets,ids,count in [
        ('edge',v.compact_edges,info.edge_colliding_edges_buffer_sizes,info.edge_colliding_edges_offsets,info.edge_colliding_edges,info.edge_colliding_edges_count),
        ('vertex',v.compact_vertices,info.vertex_colliding_triangles_buffer_sizes,info.vertex_colliding_triangles_offsets,info.vertex_colliding_triangles,info.vertex_colliding_triangles_count)]:
        capacity=capacity.numpy();offsets=offsets.numpy();ids=ids.numpy();count=count.numpy()
        expected=np.array([(i,int(ids[2*(offsets[i]+j)+1])) for i in range(len(count)) for j in range(min(count[i],capacity[i]))],dtype=np.int32).reshape(-1,2)
        actual=packed.numpy()[:counts[0 if kind=='edge' else 1]]
        order=lambda x:x[np.lexsort((x[:,1],x[:,0]))]
        checks[f'{sample}_{kind}_no_lost_or_duplicated_pairs']=bool(np.array_equal(order(expected),order(actual)))
    for color in range(len(m.particle_color_groups)):
        forces=[wp.zeros(m.particle_count,dtype=wp.vec3,device=v.device) for _ in range(2)]
        hessians=[wp.zeros(m.particle_count,dtype=wp.mat33,device=v.device) for _ in range(2)]
        common=[s.dt,color,previous,s.a.particle_q,m.particle_colors,m.tri_indices,m.edge_indices]
        tail=[v.particle_self_contact_margin,m.soft_contact_ke,m.soft_contact_kd,m.soft_contact_mu,v.friction_epsilon,v._self_contact_edge_edge_parallel_epsilon]
        wp.launch(accumulate_self_contact_force_and_hessian,dim=v.particle_self_contact_evaluation_kernel_launch_size,
                  inputs=common+[v.trimesh_collision_info]+tail+[forces[0],hessians[0]],device=v.device)
        wp.launch(accumulate_compact_self_contact,dim=PAIR_THREADS,
                  inputs=common+[v.compact_edges,v.compact_vertices,v.compact_counts]+tail+[forces[1],hessians[1]],device=v.device)
        for kind,arrays in [('force',forces),('hessian',hessians)]:
            ref,actual=[x.numpy() for x in arrays]
            error=float(np.max(abs(ref-actual)));scale=max(float(np.max(abs(ref))),1.)
            checks[f'{sample}_{color}_{kind}']=error/scale<1e-5
            metrics[f'{sample}_{color}_{kind}_relative_max_error']=error/scale
    displacement=wp.array(rng.normal(0,.003,q.shape).astype(np.float32),dtype=wp.vec3,device=v.device)
    ts=[wp.full(m.particle_count,1.,dtype=float,device=v.device) for _ in range(2)]
    common=[s.a.particle_q,displacement,m.tri_indices,m.edge_indices]
    tail=[v._self_contact_edge_edge_parallel_epsilon,v.particle_conservative_bound_relaxation]
    wp.launch(apply_planar_truncation_parallel_by_collision,dim=v.particle_self_contact_evaluation_kernel_launch_size,
              inputs=common+[v.trimesh_collision_info]+tail+[ts[0]],device=v.device)
    wp.launch(truncate_compact_pairs,dim=PAIR_THREADS,
              inputs=common+[v.compact_edges,v.compact_vertices,v.compact_counts]+tail+[ts[1]],device=v.device)
    truncation_error=float(np.max(abs(ts[0].numpy()-ts[1].numpy())))
    metrics[f'{sample}_truncation_max_error']=truncation_error
    checks[f'{sample}_same_planar_truncation']=truncation_error<1e-6

# Independent float64 reference for the inertial translation block/trust region.
q=samples[-1];target=q+rng.normal(0,.002,q.shape).astype(np.float32)
target_gpu=wp.array(target,dtype=wp.vec3,device=v.device)
wp.launch(inertia_block,dim=BLOCK_PARTIALS*128,block_dim=128,
          inputs=[s.a.particle_q,target_gpu,m.particle_mass,s.dt,v.block_force,v.block_hessian],device=v.device)
wp.launch(solve_block,dim=1,inputs=[v.block_force,v.block_hessian,.001,v.block_delta],device=v.device)
mass=m.particle_mass.numpy().astype(float);delta=((target.astype(float)-q)*mass[:,None]).sum(0)/mass.sum()
delta*=min(.8,.001/max(np.linalg.norm(delta),1e-12))
checks['inertial_block_float64_reference']=bool(np.allclose(delta,v.block_delta.numpy()[0],rtol=2e-5,atol=1e-9))

# Identical high-resolution geometry before/after, including the crushed frame.
spec=importlib.util.spec_from_file_location('surface_before',ROOT/'outputs/optimization/before/surface.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
prim=s.stage.GetPrimAtPath('/World/Box/RenderMesh');a=lambda n:prim.GetAttribute('cardboard:'+n).Get()
args=[s.local,s.faces,np.asarray(a('bindingIndices')),np.asarray(a('bindingWeights')),np.asarray(a('bindingOffsets')),a('visualCreaseAngleDegrees'),a('visualCreaseTransitionDegrees'),bool(a('visualSmoothThickness'))]
old=module.PanelSurface(*args);new=PanelSurface(*args)
for i,q in enumerate(samples):
    before=old.evaluate(q-s.center);after=new.evaluate(q-s.center)
    checks[f'{i}_same_display_positions']=bool(np.allclose(before[0],after[0],rtol=0,atol=1e-7))
    checks[f'{i}_same_display_normals']=bool(np.allclose(before[1],after[1],rtol=0,atol=1e-6))
report=dict(passed=all(checks.values()),checks=checks,metrics=metrics)
(ROOT/'outputs/optimization/kernel_equivalence.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
