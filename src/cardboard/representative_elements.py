"""Positive spatial quadrature of shell elements (not fitted cubature).

Keep box seams exact and stratify flat panels by orientation. Farthest-point
representatives carry the rest-area/length of their nearest-neighbor cluster.
All corners of a selected element are evaluated, preserving balanced internal
forces. Inertia, contacts and plastic history updates are never subsampled.
"""
import numpy as np
import warp as wp
from scipy.spatial import cKDTree
from newton._src.solvers.vbd.particle_vbd_kernels import (
    evaluate_neo_hookean_membrane_force_hessian,
    evaluate_dihedral_angle_based_bending_force_hessian,
)
from .small_bend_kernels import evaluate_small_bend_force_hessian

wp.set_module_options({'enable_backward':False})


def spatial_quadrature(centers,measure,groups,fraction):
    if not np.isfinite(fraction) or not 0<fraction<=1:
        raise ValueError('Representative fraction must be in (0,1]')
    centers=np.asarray(centers,float);measure=np.asarray(measure,float)
    if not np.isfinite(centers).all() or not np.isfinite(measure).all() or np.any(measure<=0):
        raise ValueError('Finite geometry and positive element measures required')
    selected=[];weights=[]
    for group in np.unique(groups):
        ids=np.flatnonzero(groups==group);x=centers[ids]
        count=len(ids) if group<0 else max(1,int(np.ceil(len(ids)*fraction)))
        if count==len(ids):chosen=np.arange(len(ids))
        else:
            chosen=[];distance=np.full(len(ids),np.inf)
            nxt=int(np.argmax(np.sum((x-x.mean(0))**2,axis=1)))
            for _ in range(count):
                chosen.append(nxt)
                distance=np.minimum(distance,np.sum((x-x[nxt])**2,axis=1))
                distance[chosen]=-1.
                nxt=int(np.argmax(distance))
            chosen=np.array(chosen)
        nearest=cKDTree(x[chosen]).query(x)[1]
        # Always assign each representative itself, including coincident centers.
        nearest[chosen]=np.arange(len(chosen))
        cluster_measure=np.bincount(nearest,weights=measure[ids],minlength=len(chosen))
        selected.extend(ids[chosen]);weights.extend(cluster_measure/measure[ids[chosen]])
    order=np.argsort(selected)
    return np.asarray(selected,np.int32)[order],np.asarray(weights,np.float32)[order]


def shell_quadrature(q,tri,edges,areas,lengths,angles,fraction):
    normal=np.cross(q[tri[:,1]]-q[tri[:,0]],q[tri[:,2]]-q[tri[:,0]])
    axis=np.argmax(abs(normal),axis=1)
    groups=2*axis+(normal[np.arange(len(tri)),axis]>0)
    ti,tw=spatial_quadrature(q[tri].mean(1),areas,groups,fraction)
    valid=np.all(edges>=0,axis=1)
    ids=np.flatnonzero(valid)
    points=q[edges[valid]];spread=np.ptp(points,axis=1)
    plane=np.argmin(spread,axis=1)
    center=points.mean(1);box_center=(q.min(0)+q.max(0))/2
    panel=2*plane+(center[np.arange(len(ids)),plane]>box_center[plane])
    direction=np.argmax(abs(q[edges[valid,3]]-q[edges[valid,2]]),axis=1)
    group=panel*3+direction
    # Creased rest edges and nonplanar neighborhoods are exact strata.
    seam=(abs(angles[valid])>.1)|(spread[np.arange(len(ids)),plane]>1e-5)
    group[seam]=-1
    ei,ew=spatial_quadrature(center,lengths[valid],group,fraction)
    return ti,tw,ids[ei].astype(np.int32),ew


@wp.kernel
def detect_yield(alpha:wp.array[float],threshold:float,full:wp.array[int]):
    i=wp.tid()
    if alpha[i]>threshold:wp.atomic_max(full,0,1)


@wp.kernel
def add_inertia(dt:float,q:wp.array[wp.vec3],mass:wp.array[float],target:wp.array[wp.vec3],
                force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    i=wp.tid();k=mass[i]/(dt*dt)
    force[i]+=k*(target[i]-q[i]);hessian[i]+=k*wp.identity(3,wp.float32)


@wp.kernel
def sampled_membrane(dt:float,ids:wp.array[int],weights:wp.array[float],previous:wp.array[wp.vec3],
    q:wp.array[wp.vec3],tri:wp.array2d[int],poses:wp.array[wp.mat22],materials:wp.array2d[float],areas:wp.array[float],
    full:wp.array[int],force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    j,o=wp.tid();t=ids[j];i=tri[t,o]
    weight=weights[j]
    if full[0]!=0:weight=1.0
    if weight==0.0:return
    f,h=evaluate_neo_hookean_membrane_force_hessian(t,o,q,previous,tri,poses[t],areas[t],materials[t,0],materials[t,1],materials[t,2],dt)
    wp.atomic_add(force,i,weight*f);wp.atomic_add(hessian,i,weight*h)


@wp.kernel
def sampled_bending(dt:float,ids:wp.array[int],weights:wp.array[float],previous:wp.array[wp.vec3],
    q:wp.array[wp.vec3],edges:wp.array2d[int],angles:wp.array[float],lengths:wp.array[float],bending:wp.array2d[float],
    enabled:int,dual:wp.array[float],alpha:wp.array[float],scale:float,knee:float,end:float,memory:float,crease_friction:float,
    full:wp.array[int],force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    j,o=wp.tid();e=ids[j];i=edges[e,o]
    weight=weights[j]
    if full[0]!=0:weight=1.0
    if weight==0.0:return
    if bending[e,0]>0.0:
        f=wp.vec3(0.0);h=wp.mat33(0.0)
        if enabled!=0:
            f,h=evaluate_small_bend_force_hessian(e,o,q,previous,edges,angles,lengths,bending[e,0],bending[e,1],dt,dual,alpha,scale,knee,end,memory,crease_friction)
        else:
            f,h=evaluate_dihedral_angle_based_bending_force_hessian(e,o,q,previous,edges,angles,lengths,bending[e,0],bending[e,1],dt)
        wp.atomic_add(force,i,weight*f);wp.atomic_add(hessian,i,weight*h)


@wp.kernel
def finish_residual(force:wp.array[wp.vec3],hessian:wp.array[wp.mat33],local:wp.array[wp.vec3]):
    i=wp.tid();local[i]=wp.inverse(hessian[i])*force[i]


class RepresentativeElements:
    def __init__(self,model,fraction,full_after_yield=False):
        q=model.particle_q.numpy();tri=model.tri_indices.numpy();edges=model.edge_indices.numpy()
        ti,tw,ei,ew=shell_quadrature(q,tri,edges,model.tri_areas.numpy(),model.edge_rest_length.numpy(),model.edge_rest_angle.numpy(),fraction)
        self.tri_ids=wp.array(ti,dtype=int,device=model.device);self.tri_weights=wp.array(tw,device=model.device)
        self.edge_ids=wp.array(ei,dtype=int,device=model.device);self.edge_weights=wp.array(ew,device=model.device)
        self.report=dict(method='positive spatial quadrature; seams exact; no fitted cubature',fraction=float(fraction),
            triangles_selected=len(ti),triangles_total=len(tri),hinges_selected=len(ei),hinges_total=len(edges),
            triangle_ids=ti.tolist(),triangle_weights=tw.tolist(),hinge_ids=ei.tolist(),hinge_weights=ew.tolist())
        self.full_after_yield=bool(full_after_yield)
        self.full=wp.zeros(1,dtype=int,device=model.device)
        self.report['full_after_yield']=self.full_after_yield
        if self.full_after_yield:
            dense_tri=np.zeros(len(tri),np.float32);dense_tri[ti]=tw
            valid=np.flatnonzero(np.all(edges>=0,axis=1)).astype(np.int32)
            dense_edge=np.zeros(len(edges),np.float32);dense_edge[ei]=ew
            self.tri_ids=wp.array(np.arange(len(tri),dtype=np.int32),dtype=int,device=model.device)
            self.tri_weights=wp.array(dense_tri,device=model.device)
            self.edge_ids=wp.array(valid,dtype=int,device=model.device);self.edge_weights=wp.array(dense_edge[valid],device=model.device)

    def add_elastic_inertia(self,s,q,state,dt,force,hessian,local):
        m=s.model;device=m.device
        config=getattr(s,'small_bend_config',None)
        small=([1,s.small_bend_dual,state.cardboard.accumulated_angle,*config,getattr(s,'small_bend_memory_curvature',0.),getattr(s,'crease_friction_curvature',0.)] if config is not None
               else [0,m.edge_rest_length,m.edge_rest_angle,1.,.3,5.9,0.,0.])
        if self.full_after_yield:
            # Re-evaluate instead of a host-side state transition or conditional
            # child graph. Plastic history is constant within the substep.
            self.full.zero_()
            wp.launch(detect_yield,dim=m.edge_count,inputs=[state.cardboard.accumulated_angle,1e-4,self.full],device=device)
        wp.launch(add_inertia,dim=m.particle_count,inputs=[dt,q,m.particle_mass,s.inertia,force,hessian],device=device)
        wp.launch(sampled_membrane,dim=(self.tri_ids.size,3),inputs=[dt,self.tri_ids,self.tri_weights,s.particle_q_prev,q,
            m.tri_indices,m.tri_poses,m.tri_materials,m.tri_areas,self.full,force,hessian],device=device)
        if self.edge_ids.size:
            wp.launch(sampled_bending,dim=(self.edge_ids.size,4),inputs=[dt,self.edge_ids,self.edge_weights,s.particle_q_prev,q,
                m.edge_indices,m.edge_rest_angle,m.edge_rest_length,m.edge_bending_properties,*small,self.full,force,hessian],device=device)
        wp.launch(finish_residual,dim=m.particle_count,inputs=[force,hessian,local],device=device)
