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
from .compact_contact import CompactContactMixin
from .rigid_schedule import RigidScheduleMixin


BLOCK_PARTIALS = 32


@wp.kernel
def inertia_block(q:wp.array[wp.vec3], target:wp.array[wp.vec3], mass:wp.array[float],
                  dt:float, force:wp.array[wp.vec3], hessian:wp.array[wp.mat33]):
    # Block-local tree reduction; one output per block, no global contention.
    tid=wp.tid()
    f=wp.vec3(0.0)
    ksum=float(0.0)
    for i in range(tid,q.shape[0],BLOCK_PARTIALS*128):
        k=mass[i]/(dt*dt)
        f+=k*(target[i]-q[i])
        ksum+=k
    fx=wp.tile_sum(wp.tile(f[0]))
    fy=wp.tile_sum(wp.tile(f[1]))
    fz=wp.tile_sum(wp.tile(f[2]))
    ks=wp.tile_sum(wp.tile(ksum))
    total=ks[0]
    total_force=wp.vec3(fx[0],fy[0],fz[0])
    if tid % 128 == 0:
        force[tid//128]=total_force
        hessian[tid//128]=total*wp.identity(3,wp.float32)

@wp.kernel
def contact_block(q:wp.array[wp.vec3], previous:wp.array[wp.vec3], radius:wp.array[float],
                  count:wp.array[int], indices:wp.array[wp.vec3i], bary:wp.array[wp.vec3],
                  ke:wp.array[float],kd:wp.array[float],mu:wp.array[float],eps:float,
                  shape_body:wp.array[int],body_q:wp.array[wp.transform],body_previous:wp.array[wp.transform],
                  body_qd:wp.array[wp.spatial_vector],body_com:wp.array[wp.vec3],
                  shape:wp.array[int],body_pos:wp.array[wp.vec3],body_vel:wp.array[wp.vec3],
                  normal:wp.array[wp.vec3],margin:wp.array[float],dt:float,
                  force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    tid=wp.tid()
    for i in range(tid,wp.min(count[0],indices.shape[0]),2048):
        if indices[i][0]>=0:
            f,h,cp=_eval_soft_ef_contact(i,indices[i],bary[i],q,previous,radius,ke[i],kd[i],mu[i],eps,
                                       shape_body,body_q,body_previous,body_qd,body_com,
                                       shape,body_pos,body_vel,normal,margin,dt)
            wp.atomic_add(force,i % BLOCK_PARTIALS,f)
            wp.atomic_add(hessian,i % BLOCK_PARTIALS,h)


@wp.kernel
def solve_block(force:wp.array[wp.vec3],hessian:wp.array[wp.mat33],
                max_step:float,delta_out:wp.array[wp.vec3]):
    f=wp.vec3(0.0)
    h=wp.mat33(0.0)
    for i in range(BLOCK_PARTIALS):
        f+=force[i]
        h+=hessian[i]
    delta=wp.inverse(h)*f
    delta*=wp.min(0.8,max_step/wp.max(wp.length(delta),1.e-12))
    delta_out[0]=delta


@wp.kernel
def translate_block(delta:wp.array[wp.vec3],q:wp.array[wp.vec3],out:wp.array[wp.vec3]):
    i=wp.tid()
    q[i]+=delta[0]
    out[i]=q[i]


class TranslationBlockVBD(RigidScheduleMixin,CompactContactMixin,newton.solvers.SolverVBD):
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
        self._init_compact_contacts()
        self.block_force=wp.zeros(BLOCK_PARTIALS,dtype=wp.vec3,device=self.device)
        self.block_hessian=wp.zeros(BLOCK_PARTIALS,dtype=wp.mat33,device=self.device)

        self.block_delta=wp.zeros(1,dtype=wp.vec3,device=self.device)
        self.particle_solve_interval=1
        self.box_sleeping=False
        self.rotation_block=False
        self.rigid_subspace=None
        self.rigid_subspace_enabled=False
        self.rotation_origin=wp.vec3(*np.average(model.particle_q.numpy(),axis=0,weights=model.particle_mass.numpy()))

    def _initialize_particles(self,state_in,state_out,dt):
        if not self.box_sleeping:
            return super()._initialize_particles(state_in,state_out,dt)
        wp.copy(self.particle_q_prev,state_in.particle_q)
        wp.copy(state_out.particle_q,state_in.particle_q)
        state_in.particle_qd.zero_()
        state_out.particle_qd.zero_()

    def _finalize_particles(self,state_out,dt):
        if self.box_sleeping:
            state_out.particle_qd.zero_()
        else:
            super()._finalize_particles(state_out,dt)

    def _solve_particle_iteration(self,state_in,state_out,contacts,dt,iter_num):
        # Preserve the first collision refresh and the final coupled solve.
        # Intervening rigid iterations refine the linkage/contact duals while
        # holding the particle block fixed, as an inexact alternating solve.
        if self.box_sleeping or (iter_num % self.particle_solve_interval and iter_num != self.iterations-1):
            return
        super()._solve_particle_iteration(state_in,state_out,contacts,dt,iter_num)
        if self.rigid_subspace and self.rigid_subspace_enabled:
            self.rigid_subspace.apply(self,state_in,state_out,contacts,dt)
            return
        m=self.model
        wp.launch(inertia_block,dim=BLOCK_PARTIALS*128,block_dim=128,
                  inputs=[state_in.particle_q,self.inertia,m.particle_mass,dt,self.block_force,self.block_hessian],device=self.device)
        if contacts is not None:
            c=contacts
            wp.launch(contact_block,dim=2048,inputs=[
                state_in.particle_q,self.particle_q_prev,m.particle_radius,c.soft_contact_count,
                c.soft_contact_indices,c.soft_contact_barycentric,self.body_particle_contact_penalty_k,
                self.body_particle_contact_material_kd,self.body_particle_contact_material_mu,float(self.friction_epsilon),
                m.shape_body,state_in.body_q,self.body_q_prev,state_in.body_qd,m.body_com,
                c.soft_contact_shape,c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,dt,
                self.block_force,self.block_hessian],device=self.device)
        wp.launch(solve_block,dim=1,inputs=[self.block_force,self.block_hessian,.001,self.block_delta],device=self.device)
        wp.launch(translate_block,dim=m.particle_count,inputs=[self.block_delta,
                    state_in.particle_q,state_out.particle_q],device=self.device)
        if self.rotation_block:
            from .rotation_block import apply_rotation
            apply_rotation(self,state_in,state_out,contacts,dt)
