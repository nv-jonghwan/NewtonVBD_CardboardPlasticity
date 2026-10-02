"""Identical cold-start crushed shell, no gravity/rigid contacts or initial velocity.

This isolates residual internal stress; it is deliberately not a warm full-scene
restart. Each branch restores exactly the same geometry and plastic history.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
import warp as wp
from scipy.spatial import ConvexHull
from cardboard import ROOT
from cardboard.small_bend_trial import SmallBendPickCrushDrop
from cardboard.plasticity import return_map_crease,dihedral

@wp.kernel
def inspect_angles(q:wp.array[wp.vec3],edges:wp.array2d[int],reference:wp.array[float],
                   plastic:wp.array[float],stats:wp.array[float]):
    i=wp.tid()
    if edges[i,0]>=0 and edges[i,1]>=0:
        theta=dihedral(q[edges[i,0]],q[edges[i,1]],q[edges[i,2]],q[edges[i,3]])
        elastic=wp.abs(theta-reference[i]-plastic[i])
        wp.atomic_max(stats,0,elastic)
        if elastic>3.14159265:wp.atomic_add(stats,1,1.)

p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);p.add_argument('--output',required=True)
p.add_argument('--device',default='cuda:1');p.add_argument('--duration',type=float,default=2.)
a=p.parse_args();folder=ROOT/a.checkpoint;out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
cfg=json.loads((folder/'run_config.json').read_text());saved=np.load(folder/'final_material_state.npz');summary={}
for name,friction,plastic in [('elastic_only',0.,False),('plastic',0.,True),('friction15',15.,True),('friction45',45.,True)]:
    options=dict(cfg['small_bend']);options['crease_friction_curvature']=friction
    s=SmallBendPickCrushDrop(device=a.device,scene_path=ROOT/cfg['scene'],iterations=cfg['iterations'],rom_options=cfg['rom'],small_bend_options=options)
    s.model.gravity.zero_()
    for state in [s.a,s.b]:
        state.particle_q.assign(saved['points']);state.particle_qd.zero_()
        for key in ['plastic_angle','accumulated_angle','plastic_work','damage']:getattr(state.cardboard,key).assign(saved[key])
    s.model.edge_rest_angle.assign(s.reference+saved['plastic_angle'])
    stiffness=s.ke*(1-saved['damage']);s.model.edge_bending_properties.assign(np.column_stack([stiffness,s.bend_relaxation*stiffness]).astype(np.float32))
    stats=wp.zeros(2,dtype=float,device=a.device)
    def integrate():
        for _ in range(s.substeps):
            s.a.clear_forces();s.solver.step(s.a,s.b,s.control,None,s.dt)
            s.internal_damping.apply(s.b,s.dt)
            wp.launch(inspect_angles,dim=s.model.edge_count,inputs=[s.b.particle_q,s.model.edge_indices,s.ref_gpu,s.a.cardboard.plastic_angle,stats],device=a.device)
            wp.launch(return_map_crease,dim=s.model.edge_count,inputs=[s.b.particle_q,s.model.edge_indices,s.model.edge_rest_length,s.ref_gpu,s.dual,s.ke_gpu,s.attr(s.mat,'yieldCurvature'),s.attr(s.mat,'hardeningRatio'),s.attr(s.mat,'damageRate'),s.attr(s.mat,'residualStiffness'),int(plastic),s.bend_relaxation,s.a.cardboard.plastic_angle,s.a.cardboard.accumulated_angle,s.a.cardboard.plastic_work,s.b.cardboard.plastic_angle,s.b.cardboard.accumulated_angle,s.b.cardboard.plastic_work,s.b.cardboard.damage,s.model.edge_rest_angle,s.model.edge_bending_properties,s.crease_damage_length],device=a.device)
            s.a,s.b=s.b,s.a
    def shape(q):
        x=q.astype(float);x-=x.mean(0);tri=x[s.faces]
        return [abs(np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])).sum()/6),ConvexHull(x).volume]
    q0=saved['points'];initial=shape(q0);points=[q0.copy()];ts=[0.];start=time.perf_counter()
    integrate();points.append(s.a.particle_q.numpy());ts.append(1/s.fps)
    with wp.ScopedCapture() as capture:integrate()
    for i in range(1,round(a.duration*s.fps)):
        wp.capture_launch(capture.graph)
        if i%2==1:points.append(s.a.particle_q.numpy());ts.append((i+1)/s.fps)
    wp.synchronize();q=s.a.particle_q.numpy();final=shape(q)
    report=dict(initial_shape=initial,final_shape=final,volume_change_percent=(np.asarray(final)/initial-1).tolist(),max_speed_m_s=float(np.linalg.norm(s.a.particle_qd.numpy(),axis=1).max()),max_elastic_angle=float(stats.numpy()[0]),substep_branch_exceedances=int(stats.numpy()[1]),new_accumulated_angle_sum=float((s.a.cardboard.accumulated_angle.numpy()-saved['accumulated_angle']).sum()),wall_s=time.perf_counter()-start)
    report['volume_change_percent']=[100*x for x in report['volume_change_percent']]
    summary[name]=report;np.savez_compressed(out/(name+'.npz'),points=points,t=ts)
    (out/'report.json').write_text(json.dumps(summary,indent=2));print(name,json.dumps(report),flush=True)
