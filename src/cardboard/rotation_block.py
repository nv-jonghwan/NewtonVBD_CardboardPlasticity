"""Rigid rotation subspace of one free shell; internal strain stays invariant."""
import warp as wp
from newton._src.solvers.vbd.rigid_vbd_kernels import _eval_soft_ef_contact

PARTIALS=32

@wp.func
def cross_matrix(r:wp.vec3):
    return wp.mat33(0.,-r[2],r[1],r[2],0.,-r[0],-r[1],r[0],0.)

@wp.kernel
def inertia_rotation(q:wp.array[wp.vec3],target:wp.array[wp.vec3],mass:wp.array[float],
                     origin:wp.vec3,dt:float,force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    tid=wp.tid();f=wp.vec3(0.);h=wp.mat33(0.)
    for i in range(tid,q.shape[0],PARTIALS*128):
        k=mass[i]/(dt*dt);r=q[i]-origin;s=cross_matrix(r)
        f+=wp.cross(r,k*(target[i]-q[i]));h-=k*s*s
    fx=wp.tile_sum(wp.tile(f[0]));fy=wp.tile_sum(wp.tile(f[1]));fz=wp.tile_sum(wp.tile(f[2]))
    h00=wp.tile_sum(wp.tile(h[0,0]));h01=wp.tile_sum(wp.tile(h[0,1]));h02=wp.tile_sum(wp.tile(h[0,2]))
    h11=wp.tile_sum(wp.tile(h[1,1]));h12=wp.tile_sum(wp.tile(h[1,2]));h22=wp.tile_sum(wp.tile(h[2,2]))
    if tid%128==0:
        force[tid//128]=wp.vec3(fx[0],fy[0],fz[0])
        hessian[tid//128]=wp.mat33(h00[0],h01[0],h02[0],h01[0],h11[0],h12[0],h02[0],h12[0],h22[0])

@wp.kernel
def contact_rotation(q:wp.array[wp.vec3],previous:wp.array[wp.vec3],radius:wp.array[float],
                     count:wp.array[int],indices:wp.array[wp.vec3i],bary:wp.array[wp.vec3],
                     ke:wp.array[float],kd:wp.array[float],mu:wp.array[float],eps:float,
                     shape_body:wp.array[int],body_q:wp.array[wp.transform],body_previous:wp.array[wp.transform],
                     body_qd:wp.array[wp.spatial_vector],body_com:wp.array[wp.vec3],
                     shape:wp.array[int],body_pos:wp.array[wp.vec3],body_vel:wp.array[wp.vec3],
                     normal:wp.array[wp.vec3],margin:wp.array[float],origin:wp.vec3,dt:float,
                     force:wp.array[wp.vec3],hessian:wp.array[wp.mat33]):
    tid=wp.tid()
    for i in range(tid,wp.min(count[0],indices.shape[0]),2048):
        if indices[i][0]>=0:
            f,h,cp=_eval_soft_ef_contact(i,indices[i],bary[i],q,previous,radius,ke[i],kd[i],mu[i],eps,
                                       shape_body,body_q,body_previous,body_qd,body_com,
                                       shape,body_pos,body_vel,normal,margin,dt)
            x=wp.vec3(0.)
            for k in range(3):
                j=indices[i][k]
                if j>=0:x+=bary[i][k]*q[j]
            r=x-origin;s=cross_matrix(r)
            wp.atomic_add(force,i%PARTIALS,wp.cross(r,f))
            wp.atomic_add(hessian,i%PARTIALS,-s*h*s)

@wp.kernel
def rotate_block(delta:wp.array[wp.vec3],origin:wp.vec3,q:wp.array[wp.vec3],out:wp.array[wp.vec3]):
    i=wp.tid();angle=wp.length(delta[0]);rotation=wp.quat_identity()
    if angle>1.e-12:rotation=wp.quat_from_axis_angle(delta[0]/angle,angle)
    q[i]=origin+wp.quat_rotate(rotation,q[i]-origin);out[i]=q[i]


def apply_rotation(s,state_in,state_out,contacts,dt):
    from .block_solver import solve_block
    m=s.model;origin=s.rotation_origin
    wp.launch(inertia_rotation,dim=PARTIALS*128,block_dim=128,inputs=[state_in.particle_q,s.inertia,m.particle_mass,origin,dt,s.block_force,s.block_hessian],device=s.device)
    if contacts is not None:
        c=contacts
        wp.launch(contact_rotation,dim=2048,inputs=[state_in.particle_q,s.particle_q_prev,m.particle_radius,c.soft_contact_count,
            c.soft_contact_indices,c.soft_contact_barycentric,s.body_particle_contact_penalty_k,
            s.body_particle_contact_material_kd,s.body_particle_contact_material_mu,float(s.friction_epsilon),
            m.shape_body,state_in.body_q,s.body_q_prev,state_in.body_qd,m.body_com,c.soft_contact_shape,
            c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,origin,dt,
            s.block_force,s.block_hessian],device=s.device)
    # <=0.2 degrees per update; inertial/contact Gauss-Newton block.
    wp.launch(solve_block,dim=1,inputs=[s.block_force,s.block_hessian,.003,s.block_delta],device=s.device)
    wp.launch(rotate_block,dim=m.particle_count,inputs=[s.block_delta,origin,state_in.particle_q,state_out.particle_q],device=s.device)
