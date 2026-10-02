"""GPU contact/crease seed masks with deterministic topological dilation.

The mask only gates coordinate updates/contact accumulation, never collision
detection, truncation or the constitutive history update. All incident elements
of an active vertex retain their full material law. Periodic full sweeps allow
unrepresented deformation outside the patch.
"""
import numpy as np
import warp as wp
from .plasticity import dihedral
from .compact_contact import PAIR_THREADS
from newton._src.solvers.vbd.particle_vbd_kernels import triangle_closest_point

wp.set_module_options({'enable_backward':False})


@wp.kernel
def mark_soft_contacts(indices:wp.array[wp.vec3i],count:wp.array[int],bary:wp.array[wp.vec3],
    q:wp.array[wp.vec3],radius:wp.array[float],shape:wp.array[int],shape_body:wp.array[int],
    body_q:wp.array[wp.transform],body_pos:wp.array[wp.vec3],normal:wp.array[wp.vec3],margin:wp.array[float],
    padding:float,mask:wp.array[int]):
    t=wp.tid()
    if t<wp.min(count[0],indices.shape[0]):
        s=shape[t];b=shape_body[s];pose=wp.transform_identity()
        if b>=0:pose=body_q[b]
        x=wp.vec3(0.0);r=float(0.0)
        for j in range(3):
            i=indices[t][j]
            if i>=0:
                x+=bary[t][j]*q[i];r=wp.max(r,radius[i])
        m=float(0.0)
        if margin.shape[0]>0:m=margin[s]
        if wp.dot(normal[t],x-wp.transform_point(pose,body_pos[t]))-r-m>padding:return
        for j in range(3):
            i=indices[t][j]
            if i>=0:wp.atomic_max(mask,i,1)


@wp.kernel
def mark_self_contacts(ep:wp.array[wp.vec2i],vp:wp.array[wp.vec2i],counts:wp.array[int],
    edges:wp.array2d[int],tri:wp.array2d[int],q:wp.array[wp.vec3],distance:float,parallel:float,mask:wp.array[int]):
    tid=wp.tid()
    for p in range(tid,counts[0],PAIR_THREADS):
        a=ep[p][0];b=ep[p][1]
        if a>=0 and b>=0:
            st=wp.closest_point_edge_edge(q[edges[a,2]],q[edges[a,3]],q[edges[b,2]],q[edges[b,3]],parallel)
            if st[2]<=distance:
                for k in range(2):
                    e=ep[p][k]
                    for j in range(2,4):wp.atomic_max(mask,edges[e,j],1)
    for p in range(tid,counts[1],PAIR_THREADS):
        v=vp[p][0];t=vp[p][1]
        if v>=0 and t>=0:
            closest,bary,feature=triangle_closest_point(q[tri[t,0]],q[tri[t,1]],q[tri[t,2]],q[v])
            if wp.length(q[v]-closest)<=distance:
                wp.atomic_max(mask,v,1)
                for j in range(3):wp.atomic_max(mask,tri[t,j],1)


@wp.kernel
def mark_creases(q:wp.array[wp.vec3],edges:wp.array2d[int],rest_angle:wp.array[float],
    dual:wp.array[float],alpha:wp.array[float],curvature:float,mask:wp.array[int]):
    e=wp.tid()
    if edges[e,0]>=0 and edges[e,1]>=0:
        theta=dihedral(q[edges[e,0]],q[edges[e,1]],q[edges[e,2]],q[edges[e,3]])
        if alpha[e]>1.e-4 or wp.abs(theta-rest_angle[e])>curvature*dual[e]:
            for j in range(4):wp.atomic_max(mask,edges[e,j],1)


@wp.kernel
def dilate_patch(neighbors:wp.array2d[int],old:wp.array[int],new:wp.array[int]):
    i=wp.tid();value=old[i]
    for j in range(neighbors.shape[1]):
        v=neighbors[i,j]
        if v>=0:value=wp.max(value,old[v])
    new[i]=value


@wp.kernel
def mask_colors(mask:wp.array[int],colors:wp.array[int],local_colors:wp.array[int],parallel_colors:wp.array[int],stats:wp.array[wp.int64]):
    i=wp.tid();local_colors[i]=-1;parallel_colors[i]=-1
    if mask[i]!=0:
        local_colors[i]=colors[i];parallel_colors[i]=0
        wp.atomic_add(stats,1,wp.int64(1))
    if i==0:
        wp.atomic_add(stats,0,wp.int64(1))
        wp.atomic_add(stats,2,wp.int64(mask.shape[0]))


@wp.kernel
def damp_patch_update(previous:wp.array[wp.vec3],displacement:wp.array[wp.vec3],relaxation:float):
    i=wp.tid();displacement[i]=previous[i]+relaxation*(displacement[i]-previous[i])


@wp.kernel
def count_full_sweep(stats:wp.array[wp.int64]):
    wp.atomic_add(stats,3,wp.int64(1))


class LocalPatch:
    def __init__(self,model,rings=1,curvature=7.5,solver='colored',relaxation=.5):
        self.rings=int(rings);self.curvature=float(curvature)
        self.jacobi=solver=='jacobi';self.relaxation=float(relaxation)
        if solver not in ('colored','jacobi') or not 0<self.relaxation<=1:
            raise ValueError('Patch solver must be colored/jacobi and relaxation in (0,1]')
        if self.rings<0 or self.rings>8 or not np.isfinite(self.curvature) or self.curvature<=0:
            raise ValueError('Patch rings must be 0..8 and curvature positive/finite')
        q=model.particle_q.numpy();edges=model.edge_indices.numpy();tri=model.tri_indices.numpy()
        neighbors=[set() for _ in q]
        # A bending element couples opposite vertices as well as triangle edges.
        for element in list(tri)+list(edges[np.all(edges>=0,axis=1)]):
            for i in element:neighbors[i].update(int(j) for j in element if j!=i)
        ids=np.full((len(q),max(map(len,neighbors))),-1,np.int32)
        for i,row in enumerate(neighbors):ids[i,:len(row)]=sorted(row)
        dual=np.ones(len(edges),np.float32)
        good=np.all(edges>=0,axis=1);ee=edges[good];v=q[ee[:,3]]-q[ee[:,2]]
        dual[good]=sum(np.linalg.norm(np.cross(q[ee[:,i]]-q[ee[:,2]],v),axis=1) for i in (0,1))/(2*np.linalg.norm(v,axis=1))
        self.dual=wp.array(dual,device=model.device)
        self.neighbors=wp.array(ids,dtype=int,device=model.device)
        self.mask=wp.zeros(len(q),dtype=int,device=model.device);self.other=wp.zeros_like(self.mask)
        self.colors=wp.zeros_like(self.mask);self.stats=wp.zeros(4,dtype=wp.int64,device=model.device)
        self.parallel_colors=wp.zeros_like(self.mask)
        self.all_ids=wp.array(np.arange(len(q),dtype=np.int32),dtype=int,device=model.device)
        self.previous_displacements=wp.empty(len(q),dtype=wp.vec3,device=model.device)

    def refresh(self,s,state,contacts):
        m=s.model;self.mask.zero_()
        if contacts is not None:
            wp.launch(mark_soft_contacts,dim=contacts.soft_contact_max,
                inputs=[contacts.soft_contact_indices,contacts.soft_contact_count,contacts.soft_contact_barycentric,
                    state.particle_q,m.particle_radius,contacts.soft_contact_shape,m.shape_body,state.body_q,
                    contacts.soft_contact_body_pos,contacts.soft_contact_normal,m.shape_margin,.002,self.mask],device=m.device)
        if s.particle_enable_self_contact:
            wp.launch(mark_self_contacts,dim=PAIR_THREADS,inputs=[s.compact_edges,s.compact_vertices,s.compact_counts,
                m.edge_indices,m.tri_indices,state.particle_q,s.particle_self_contact_margin+.002,
                s._self_contact_edge_edge_parallel_epsilon,self.mask],device=m.device)
        wp.launch(mark_creases,dim=m.edge_count,inputs=[state.particle_q,m.edge_indices,m.edge_rest_angle,
            self.dual,state.cardboard.accumulated_angle,self.curvature,self.mask],device=m.device)
        for _ in range(self.rings):
            wp.launch(dilate_patch,dim=m.particle_count,inputs=[self.neighbors,self.mask,self.other],device=m.device)
            wp.copy(self.mask,self.other)
        wp.launch(mask_colors,dim=m.particle_count,inputs=[self.mask,m.particle_colors,self.colors,self.parallel_colors,self.stats],device=m.device)
