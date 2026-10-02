"""Newton 1.6 VBD with a translation subspace solve for one free shell.

Element energies and self-contact are invariant under common translation.
The three-dimensional block therefore contains only inertia and external
contact terms. It accelerates the slow rigid translation mode of stiff plates,
without changing strains, plastic angles, gravity, or the contact law.
This project overlay deliberately pins private VBD interfaces to Newton 1.6.
"""
import numpy as np
from importlib.metadata import version
import warp as wp
import newton
from newton._src.solvers.vbd.rigid_vbd_kernels import _eval_soft_ef_contact


@wp.kernel
def inertia_block(q:wp.array[wp.vec3], target:wp.array[wp.vec3], mass:wp.array[float],
                  dt:float, force:wp.array[wp.vec3], hessian:wp.array[wp.mat33]):
    i=wp.tid()
    k=mass[i]/(dt*dt)
    wp.atomic_add(force,0,k*(target[i]-q[i]))
    wp.atomic_add(hessian,0,k*wp.identity(3,wp.float32))


@wp.kernel
def contact_block(q:wp.array[wp.vec3], previous:wp.array[wp.vec3], radius:wp.array[float],
                  count:wp.array[int], indices:wp.array[wp.vec3i], bary:wp.array[wp.vec3],
                  ke:wp.array[float],kd:wp.array[float],mu:wp.array[float],eps:float,
                  shape_body:wp.array[int],body_q:wp.array[wp.transform],body_previous:wp.array[wp.transform],
                  body_qd:wp.array[wp.spatial_vector],body_com:wp.array[wp.vec3],
                  shape:wp.array[int],body_pos:wp.array[wp.vec3],body_vel:wp.array[wp.vec3],
                  normal:wp.array[wp.vec3],margin:wp.array[float],dt:float,
                  force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    i=wp.tid()
    if i<count[0] and indices[i][0]>=0:
        f,h,cp=_eval_soft_ef_contact(i,indices[i],bary[i],q,previous,radius,ke[i],kd[i],mu[i],eps,
                                   shape_body,body_q,body_previous,body_qd,body_com,
                                   shape,body_pos,body_vel,normal,margin,dt)
        wp.atomic_add(force,0,f)
        wp.atomic_add(hessian,0,h)


@wp.kernel
def translate_block(force:wp.array[wp.vec3],hessian:wp.array[wp.mat33],
                    max_step:float,q:wp.array[wp.vec3],out:wp.array[wp.vec3]):
    i=wp.tid()
    delta=wp.inverse(hessian[0])*force[0]
    norm=wp.length(delta)
    # Under-relaxed Newton step with a contact-query-sized trust region.
    delta*=wp.min(0.8,max_step/wp.max(norm,1.e-12))
    q[i]+=delta
    out[i]=q[i]


class TranslationBlockVBD(newton.solvers.SolverVBD):
    """One connected unpinned shell only; no tether/spring to the world."""
    def __init__(self,model,**kwargs):
        if tuple(map(int,version('newton').split('.')[:2])) != (1,6):
            raise RuntimeError('TranslationBlockVBD is qualified only for Newton 1.6')
        if np.any(model.particle_mass.numpy()<=0) or model.spring_count:
            raise ValueError('Translation block requires a free shell without pinned particles or springs')
        faces=model.tri_indices.numpy()
        parent=list(range(model.particle_count))
        def root(i):
            while parent[i]!=i:i=parent[i]
            return i
        for a,b,c in faces:
            parent[root(int(b))]=root(int(a));parent[root(int(c))]=root(int(a))
        if len({root(i) for i in range(model.particle_count)})!=1:
            raise ValueError('Translation block requires exactly one connected shell')
        super().__init__(model,**kwargs)
        self.block_force=wp.zeros(1,dtype=wp.vec3,device=self.device)
        self.block_hessian=wp.zeros(1,dtype=wp.mat33,device=self.device)

    def _solve_particle_iteration(self,state_in,state_out,contacts,dt,iter_num):
        super()._solve_particle_iteration(state_in,state_out,contacts,dt,iter_num)
        self.block_force.zero_();self.block_hessian.zero_()
        m=self.model
        wp.launch(inertia_block,dim=m.particle_count,
                  inputs=[state_in.particle_q,self.inertia,m.particle_mass,dt,self.block_force,self.block_hessian],device=self.device)
        if contacts is not None:
            c=contacts
            wp.launch(contact_block,dim=c.soft_contact_max,inputs=[
                state_in.particle_q,self.particle_q_prev,m.particle_radius,c.soft_contact_count,
                c.soft_contact_indices,c.soft_contact_barycentric,self.body_particle_contact_penalty_k,
                self.body_particle_contact_material_kd,self.body_particle_contact_material_mu,float(self.friction_epsilon),
                m.shape_body,state_in.body_q,self.body_q_prev,state_in.body_qd,m.body_com,
                c.soft_contact_shape,c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,dt,
                self.block_force,self.block_hessian],device=self.device)
        wp.launch(translate_block,dim=m.particle_count,inputs=[self.block_force,self.block_hessian,.001,
                    state_in.particle_q,state_out.particle_q],device=self.device)
