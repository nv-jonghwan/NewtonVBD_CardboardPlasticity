# SPDX-License-Identifier: Apache-2.0
"""Experimental guarded POD corrections inside Newton1.6 VBD.

Solve (U^T D U) dz = U^T f with the current full force and VBD's
positive local Hessian blocks D. This is a projected quasi-Newton solve,
not exact Newton/Galerkin stiffness or an implementation of condensation.
By default every element is evaluated. The opt-in representative-element path
uses positive spatial quadrature; contact, inertia and history updates stay
full-order. In guarded mode, unrepresented local corrections, a
nondecreasing preconditioned force residual, or nonfinite values trigger VBD.
Exploratory mode can omit the residual check and defer rejected updates to
the next scheduled VBD sweep with a fixed graph of masked GPU kernels.
Periodic full sweeps retain access to all physical deformation directions.
"""
import numpy as np
import warp as wp
from newton._src.utils.mesh import (
    MeshAdjacencyData, get_vertex_num_adjacent_faces, get_vertex_adjacent_face_id_order,
    get_vertex_num_adjacent_edges, get_vertex_adjacent_edge_id_order,
)
from newton._src.solvers.vbd.particle_vbd_kernels import (
    evaluate_neo_hookean_membrane_force_hessian,
    evaluate_dihedral_angle_based_bending_force_hessian,
    accumulate_particle_body_contact_force_and_hessian,
)
from .compact_contact import accumulate_compact_self_contact, PAIR_THREADS
from .block_solver import TranslationBlockVBD
from .rom_basis import mesh_digest
from .small_bend_kernels import evaluate_small_bend_force_hessian

wp.set_module_options({'enable_backward':False})
PARTS=16
WIDTH=32


@wp.kernel
def elastic_residual(dt:float,previous:wp.array[wp.vec3],q:wp.array[wp.vec3],
    mass:wp.array[float],target:wp.array[wp.vec3],tri:wp.array2d[int],poses:wp.array[wp.mat22],
    materials:wp.array2d[float],areas:wp.array[float],edges:wp.array2d[int],angles:wp.array[float],
    lengths:wp.array[float],bending:wp.array2d[float],adj:MeshAdjacencyData,
    force:wp.array[wp.vec3],hessian:wp.array[wp.mat33],local:wp.array[wp.vec3],
    small_enabled:int,dual:wp.array[float],alpha:wp.array[float],scale:float,knee:float,end:float,memory:float,crease_friction:float):
    i=wp.tid()
    k=mass[i]/(dt*dt)
    f=k*(target[i]-q[i])+force[i]
    h=k*wp.identity(3,wp.float32)+hessian[i]
    for j in range(get_vertex_num_adjacent_faces(adj,i)):
        t,o=get_vertex_adjacent_face_id_order(adj,i,j)
        ft,ht=evaluate_neo_hookean_membrane_force_hessian(t,o,q,previous,tri,poses[t],areas[t],materials[t,0],materials[t,1],materials[t,2],dt)
        f+=ft;h+=ht
    for j in range(get_vertex_num_adjacent_edges(adj,i)):
        e,o=get_vertex_adjacent_edge_id_order(adj,i,j)
        if bending[e,0]>0.0:
            fe=wp.vec3(0.0);he=wp.mat33(0.0)
            if small_enabled != 0:
                fe,he=evaluate_small_bend_force_hessian(e,o,q,previous,edges,angles,lengths,bending[e,0],bending[e,1],dt,dual,alpha,scale,knee,end,memory,crease_friction)
            else:
                fe,he=evaluate_dihedral_angle_based_bending_force_hessian(e,o,q,previous,edges,angles,lengths,bending[e,0],bending[e,1],dt)
            f+=fe;h+=he
    force[i]=f;hessian[i]=h
    local[i]=wp.inverse(h)*f


@wp.kernel
def project_system(basis:wp.array2d[wp.vec3],f:wp.array[wp.vec3],h:wp.array[wp.mat33],
    local:wp.array[wp.vec3],hp:wp.array3d[wp.float64],fp:wp.array2d[wp.float64],
    dp:wp.array2d[wp.float64]):
    r,c,tid=wp.tid()
    hr=wp.float64(0.0);fr=wp.float64(0.0);dr=wp.float64(0.0)
    for i in range(tid,basis.shape[0],PARTS*WIDTH):
        u=basis[i,r]
        hr+=wp.float64(wp.dot(u,h[i]*basis[i,c]))
        if c==0:
            fr+=wp.float64(wp.dot(u,f[i]))
            dr+=wp.float64(wp.dot(u,local[i]))
    hv=wp.tile_sum(wp.tile(hr));fv=wp.tile_sum(wp.tile(fr));dv=wp.tile_sum(wp.tile(dr))
    if tid%WIDTH==0:
        hp[r,c,tid//WIDTH]=hv[0]
        if c==0:
            fp[r,tid//WIDTH]=fv[0];dp[r,tid//WIDTH]=dv[0]


@wp.kernel
def solve_reduced(hp:wp.array3d[wp.float64],fp:wp.array2d[wp.float64],
    dp:wp.array2d[wp.float64],matrix:wp.array2d[wp.float64],rhs:wp.array[wp.float64],
    dz:wp.array[float],projected_local:wp.array[float],valid:wp.array[int]):
    n=dz.shape[0];ok=int(1)
    for i in range(n):
        f=wp.float64(0.0);d=wp.float64(0.0)
        for p in range(PARTS):f+=fp[i,p];d+=dp[i,p]
        rhs[i]=f;projected_local[i]=float(d)
        for j in range(n):
            v=wp.float64(0.0)
            for p in range(PARTS):v+=hp[i,j,p]
            matrix[i,j]=v
    # Cholesky of SPD local-block projection; reject instead of hiding failure.
    for i in range(n):
        for j in range(i+1):
            v=matrix[i,j]
            for k in range(j):v-=matrix[i,k]*matrix[j,k]
            if i==j:
                if not wp.isfinite(v) or v<=wp.float64(1.e-12):ok=0
                matrix[i,j]=wp.sqrt(wp.max(v,wp.float64(1.e-12)))
            else:matrix[i,j]=v/matrix[j,j]
    for i in range(n):
        v=rhs[i]
        for j in range(i):v-=matrix[i,j]*rhs[j]
        rhs[i]=v/matrix[i,i]
    for ii in range(n):
        i=n-1-ii;v=rhs[i]
        for j in range(i+1,n):v-=matrix[j,i]*wp.float64(dz[j])
        dz[i]=float(v/matrix[i,i])
        if not wp.isfinite(dz[i]):ok=0
    valid[0]=ok


@wp.kernel
def build_trial(basis:wp.array2d[wp.vec3],dz:wp.array[float],pd:wp.array[float],
    q:wp.array[wp.vec3],collision_anchor:wp.array[wp.vec3],local:wp.array[wp.vec3],
    displacement:wp.array[wp.vec3],stats:wp.array[wp.float64]):
    i=wp.tid();step=wp.vec3(0.0);represented=wp.vec3(0.0)
    for r in range(dz.shape[0]):
        step+=basis[i,r]*dz[r]
        represented+=basis[i,r]*pd[r]
    # Uniform coefficient scaling is performed in decide_trial; this first
    # pass collects maxima and full-space correction representation error.
    wp.atomic_add(stats,0,wp.float64(wp.length_sq(local[i])))
    wp.atomic_add(stats,1,wp.float64(wp.length_sq(local[i]-represented)))
    wp.atomic_max(stats,2,wp.float64(wp.length(step)))
    if not wp.isfinite(wp.length_sq(local[i])) or not wp.isfinite(wp.length_sq(step)):
        wp.atomic_add(stats,3,wp.float64(1.0))
    displacement[i]=step


@wp.kernel
def decide_trial(stats:wp.array[wp.float64],valid:wp.array[int],tolerance:float,
                 n:int,allowed:wp.array[int],scale:wp.array[float],counters:wp.array[int]):
    # Both relative omitted correction and an absolute micrometer floor.
    threshold=wp.max(wp.float64(tolerance*tolerance)*stats[0],wp.float64(n)*wp.float64(1.e-14))
    allowed[0]=int(valid[0]!=0 and stats[3]==wp.float64(0.0) and stats[1]<=threshold)
    scale[0]=wp.min(1.0,float(wp.float64(.0001)/wp.max(stats[2],wp.float64(1.e-20))))
    wp.atomic_add(counters,0,1)
    if allowed[0]==0:wp.atomic_add(counters,2,1)


@wp.kernel
def prepare_displacement(q:wp.array[wp.vec3],anchor:wp.array[wp.vec3],
    scale:wp.array[float],step:wp.array[wp.vec3]):
    i=wp.tid();step[i]=q[i]-anchor[i]+scale[0]*step[i]


@wp.kernel
def residual_norm(local:wp.array[wp.vec3],stats:wp.array[wp.float64]):
    i=wp.tid();v=wp.length_sq(local[i])
    wp.atomic_add(stats,4,wp.float64(v))
    if not wp.isfinite(v):wp.atomic_add(stats,3,wp.float64(1.0))


@wp.kernel
def accept_trial(stats:wp.array[wp.float64],allowed:wp.array[int],counters:wp.array[int]):
    allowed[0]=int(stats[3]==wp.float64(0.0) and stats[4]<=stats[0]*wp.float64(1.0001)+wp.float64(1.e-20))
    if allowed[0]!=0:wp.atomic_add(counters,1,1)
    else:wp.atomic_add(counters,3,1)


class ReducedCorrection:
    def __init__(self,model,basis,tolerance=.35,check_residual=True,defer_fallback=False):
        self.n=model.particle_count;self.rank=basis.shape[1];self.tolerance=float(tolerance)
        self.check_residual=bool(check_residual)
        self.defer_fallback=bool(defer_fallback)
        if self.defer_fallback and self.check_residual:raise ValueError('Deferred fallback requires exploratory residual mode')
        if not 0<=self.tolerance<=1:raise ValueError('ROM tolerance must be in [0,1]')
        self.basis=wp.array(basis,dtype=wp.vec3,device=model.device)
        self.zero_colors=wp.zeros(self.n,dtype=int,device=model.device)
        self.force=wp.zeros(self.n,dtype=wp.vec3,device=model.device)
        self.hessian=wp.zeros(self.n,dtype=wp.mat33,device=model.device)
        self.local=wp.empty_like(self.force);self.trial=wp.empty_like(self.force)
        self.hp=wp.zeros((self.rank,self.rank,PARTS),dtype=wp.float64,device=model.device)
        self.fp=wp.zeros((self.rank,PARTS),dtype=wp.float64,device=model.device);self.dp=wp.zeros_like(self.fp)
        self.matrix=wp.zeros((self.rank,self.rank),dtype=wp.float64,device=model.device)
        self.rhs=wp.zeros(self.rank,dtype=wp.float64,device=model.device)
        self.dz=wp.zeros(self.rank,dtype=float,device=model.device);self.pd=wp.zeros_like(self.dz)
        self.valid=wp.zeros(1,dtype=int,device=model.device);self.allowed=wp.zeros_like(self.valid)
        self.scale=wp.ones(1,dtype=float,device=model.device)
        self.stats=wp.zeros(5,dtype=wp.float64,device=model.device)
        self.counters=wp.zeros(4,dtype=int,device=model.device)
        self.representatives=None

    def residual(self,s,q,state,c,dt):
        m=s.model;self.force.zero_();self.hessian.zero_()
        if c is not None:
            wp.launch(accumulate_particle_body_contact_force_and_hessian,dim=c.soft_contact_max,inputs=[
                dt,0,s.particle_q_prev,q,self.zero_colors,s.friction_epsilon,m.particle_radius,
                c.soft_contact_indices,c.soft_contact_count,c.soft_contact_max,s.body_particle_contact_penalty_k,
                s.body_particle_contact_material_ke,s.body_particle_contact_material_kd,s.body_particle_contact_material_mu,
                m.shape_body,state.body_q,s.body_q_prev,state.body_qd,m.body_com,c.soft_contact_shape,
                c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,c.soft_contact_barycentric,
                self.force,self.hessian],device=s.device)
        if s.particle_enable_self_contact:
            wp.launch(accumulate_compact_self_contact,dim=PAIR_THREADS,inputs=[dt,0,s.particle_q_prev,q,self.zero_colors,
                m.tri_indices,m.edge_indices,s.compact_edges,s.compact_vertices,s.compact_counts,
                s.particle_self_contact_margin,m.soft_contact_ke,m.soft_contact_kd,m.soft_contact_mu,s.friction_epsilon,
                s._self_contact_edge_edge_parallel_epsilon,self.force,self.hessian],device=s.device,max_blocks=m.device.sm_count)
        if self.representatives is not None:
            self.representatives.add_elastic_inertia(s,q,state,dt,self.force,self.hessian,self.local)
            return
        config=getattr(s,'small_bend_config',None)
        small=([1,s.small_bend_dual,state.cardboard.accumulated_angle,*config,getattr(s,'small_bend_memory_curvature',0.),getattr(s,'crease_friction_curvature',0.)] if config is not None
               else [0,m.edge_rest_length,m.edge_rest_angle,1.,.3,5.9,0.,0.])
        wp.launch(elastic_residual,dim=self.n,inputs=[dt,s.particle_q_prev,q,m.particle_mass,s.inertia,m.tri_indices,
            m.tri_poses,m.tri_materials,m.tri_areas,m.edge_indices,m.edge_rest_angle,m.edge_rest_length,
            m.edge_bending_properties,s.particle_adjacency,self.force,self.hessian,self.local,*small],device=s.device)

    def apply(self,s,state,out,c,dt,fallback):
        self.residual(s,state.particle_q,state,c,dt)
        wp.launch(project_system,dim=(self.rank,self.rank,PARTS*WIDTH),block_dim=WIDTH,
            inputs=[self.basis,self.force,self.hessian,self.local,self.hp,self.fp,self.dp],device=s.device)
        wp.launch(solve_reduced,dim=1,inputs=[self.hp,self.fp,self.dp,self.matrix,self.rhs,self.dz,self.pd,self.valid],device=s.device)
        self.stats.zero_()
        wp.launch(build_trial,dim=self.n,inputs=[self.basis,self.dz,self.pd,state.particle_q,
            s.pos_prev_collision_detection,self.local,s.particle_displacements,self.stats],device=s.device)
        wp.launch(decide_trial,dim=1,inputs=[self.stats,self.valid,self.tolerance,self.n,self.allowed,self.scale,self.counters],device=s.device)
        if self.defer_fallback:
            # Fixed GPU graph without conditional child graphs. An invalid ROM
            # correction is skipped; the next scheduled VBD resolves all DOFs.
            wp.launch(prepare_masked_displacement,dim=self.n,inputs=[state.particle_q,s.pos_prev_collision_detection,
                self.allowed,self.scale,s.particle_displacements],device=s.device)
            s._penetration_free_truncation(self.trial)
            wp.launch(check_trial_finite,dim=self.n,inputs=[self.trial,self.stats],device=s.device)
            wp.launch(accept_deferred_trial,dim=1,inputs=[self.stats,self.allowed,self.counters],device=s.device)
            wp.launch(commit_masked_trial,dim=self.n,inputs=[self.allowed,self.trial,state.particle_q,out.particle_q,
                s.pos_prev_collision_detection,s.particle_displacements],device=s.device)
            return
        def attempt():
            wp.launch(prepare_displacement,dim=self.n,inputs=[state.particle_q,s.pos_prev_collision_detection,self.scale,s.particle_displacements],device=s.device)
            s._penetration_free_truncation(self.trial)
            if self.check_residual:
                self.residual(s,self.trial,state,c,dt)
                wp.launch(residual_norm,dim=self.n,inputs=[self.local,self.stats],device=s.device)
                wp.launch(accept_trial,dim=1,inputs=[self.stats,self.allowed,self.counters],device=s.device)
            else:
                # Exploratory mode: retain finite-trial and collision truncation
                # checks, but do not evaluate all element forces a second time.
                wp.launch(check_trial_finite,dim=self.n,inputs=[self.trial,self.stats],device=s.device)
                wp.launch(accept_finite_trial,dim=1,inputs=[self.stats,self.allowed,self.counters],device=s.device)
            def accept():
                wp.copy(state.particle_q,self.trial);wp.copy(out.particle_q,self.trial)
            # Full VBD uses displacement relative to the collision anchor.
            def reject():
                wp.launch(reset_displacement,dim=self.n,inputs=[state.particle_q,s.pos_prev_collision_detection,s.particle_displacements],device=s.device)
                fallback()
            wp.capture_if(self.allowed,accept,reject)
        def reject_before_trial():
            wp.launch(reset_displacement,dim=self.n,inputs=[state.particle_q,s.pos_prev_collision_detection,s.particle_displacements],device=s.device)
            fallback()
        wp.capture_if(self.allowed,attempt,reject_before_trial)


@wp.kernel
def prepare_masked_displacement(q:wp.array[wp.vec3],anchor:wp.array[wp.vec3],allowed:wp.array[int],
    scale:wp.array[float],step:wp.array[wp.vec3]):
    i=wp.tid()
    if allowed[0]!=0:step[i]=q[i]-anchor[i]+scale[0]*step[i]
    else:step[i]=q[i]-anchor[i]


@wp.kernel
def accept_deferred_trial(stats:wp.array[wp.float64],allowed:wp.array[int],counters:wp.array[int]):
    if allowed[0]!=0:
        allowed[0]=int(stats[3]==wp.float64(0.0))
        if allowed[0]!=0:wp.atomic_add(counters,1,1)
        else:wp.atomic_add(counters,3,1)


@wp.kernel
def commit_masked_trial(allowed:wp.array[int],trial:wp.array[wp.vec3],q:wp.array[wp.vec3],
    out:wp.array[wp.vec3],anchor:wp.array[wp.vec3],displacement:wp.array[wp.vec3]):
    i=wp.tid()
    if allowed[0]!=0:q[i]=trial[i]
    out[i]=q[i]
    displacement[i]=q[i]-anchor[i]


@wp.kernel
def check_trial_finite(q:wp.array[wp.vec3],stats:wp.array[wp.float64]):
    i=wp.tid()
    if not wp.isfinite(wp.length_sq(q[i])):wp.atomic_add(stats,3,wp.float64(1.0))


@wp.kernel
def accept_finite_trial(stats:wp.array[wp.float64],allowed:wp.array[int],counters:wp.array[int]):
    allowed[0]=int(stats[3]==wp.float64(0.0))
    if allowed[0]!=0:wp.atomic_add(counters,1,1)
    else:wp.atomic_add(counters,3,1)


def use_full_sweep(ordinal,last,full_every,start_with_rom=False):
    """Static ROM/VBD slot schedule; local mode separately selects full audits."""
    return last or ((ordinal+1)%full_every==0 if start_with_rom else ordinal%full_every==0)


@wp.kernel
def reset_displacement(q:wp.array[wp.vec3],anchor:wp.array[wp.vec3],d:wp.array[wp.vec3]):
    i=wp.tid();d[i]=q[i]-anchor[i]


class AdaptiveROMVBD(TranslationBlockVBD):
    def __init__(self,model,*,rom_options,**kwargs):
        super().__init__(model,**kwargs)
        if model.tet_count or model.spring_count:raise ValueError('ROM prototype supports the free triangle shell only')
        with np.load(rom_options['basis']) as d:
            basis=d['basis'];digest=str(d['mesh_digest'])
        # model particle positions include the scene translation; the caller
        # checks the rest-local topology digest before creating the solver.
        if basis.shape[0]!=model.particle_count or basis.shape[2]!=3 or not np.isfinite(basis).all():raise ValueError('ROM basis incompatible')
        flat=basis.transpose(0,2,1).reshape(-1,basis.shape[1]).astype(float)
        if not np.allclose(flat.T@flat,np.eye(basis.shape[1]),atol=2e-5):raise ValueError('ROM basis not orthonormal')
        self.rom=ReducedCorrection(model,basis,rom_options.get('tolerance',.35),rom_options.get('check_residual',True),rom_options.get('defer_fallback',False))
        self.rom_start=bool(rom_options.get('start_with_rom',False))
        self.rom_full_every=int(rom_options.get('full_every',4))
        if self.rom_full_every<1:raise ValueError('full_every must be positive')
        if rom_options.get('element_fraction') is not None:
            if self.rom.check_residual or not self.rom.defer_fallback:
                raise ValueError('Representative elements require the explicit exploratory masked ROM mode')
            from .representative_elements import RepresentativeElements
            self.rom.representatives=RepresentativeElements(model,rom_options['element_fraction'],rom_options.get('element_full_after_yield',False))
        self.local_patch=None;self.local_sweep=False
        self.patch_full_every=int(rom_options.get('patch_full_every',8))
        if self.patch_full_every<1:raise ValueError('Patch full sweep interval must be positive')
        if rom_options.get('local_vbd',False):
            from .local_vbd import LocalPatch
            self.local_patch=LocalPatch(model,rom_options.get('patch_rings',1),rom_options.get('patch_curvature',7.5),
                rom_options.get('patch_solver','colored'),rom_options.get('patch_relaxation',.5))

    def _vbd_correction(self,state_in,state_out,contacts,dt,iter_num,force_full=False):
        # This schedule is fixed at capture; selecting patch vertices is GPU work.
        local=not force_full and self.local_patch is not None and iter_num!=self.iterations-1 and (iter_num+1)%self.patch_full_every!=0
        self.local_sweep=local
        if self.local_patch is not None and not local:
            from .local_vbd import count_full_sweep
            wp.launch(count_full_sweep,dim=1,inputs=[self.local_patch.stats],device=self.device)
        try:super()._solve_particle_iteration(state_in,state_out,contacts,dt,iter_num)
        finally:self.local_sweep=False

    def _solve_particle_iteration(self,state_in,state_out,contacts,dt,iter_num):
        if self.box_sleeping or (iter_num%self.particle_solve_interval and iter_num!=self.iterations-1):return
        full=lambda: self._vbd_correction(state_in,state_out,contacts,dt,iter_num)
        ordinal=iter_num//self.particle_solve_interval
        if use_full_sweep(ordinal,iter_num==self.iterations-1,self.rom_full_every,self.rom_start):
            full()
        else:
            # A ROM-first schedule must honor the same self-contact refresh as
            # VBD, including PRE_POST_INIT on the first particle iteration.
            if self.particle_enable_self_contact:
                from newton._src.solvers.solver import SolverBase
                frequency=SolverBase.CollisionFrequencyType
                if (self._sc_mode_this_step==frequency.PRE_POST_INIT and iter_num==0) or (
                    self._sc_mode_this_step==frequency.ITERATIONS and (iter_num+1)%self._sc_freq_this_step==0):
                    self._collision_detection_penetration_free(state_in)
            fallback=lambda:self._vbd_correction(state_in,state_out,contacts,dt,iter_num,force_full=True)
            self.rom.apply(self,state_in,state_out,contacts,dt,fallback)
