"""Coupled six-DOF rigid subspace of a free shell, including contact friction.

A common SE(3) transform preserves shell strains and all self-contact distances.
Solving translation and rotation together avoids artificial pivot resistance
from alternating translation/rotation blocks at a sticking contact.
"""
import warp as wp
from newton._src.solvers.vbd.rigid_vbd_kernels import _eval_soft_ef_contact
from .rotation_block import cross_matrix
PARTS=32
MatD=wp.mat33d
VecD=wp.vec3d

@wp.kernel
def inertia_rigid(q:wp.array[wp.vec3],target:wp.array[wp.vec3],mass:wp.array[float],origin:wp.vec3,dt:float,
                  f_out:wp.array[wp.vec3],g_out:wp.array[wp.vec3],a_out:wp.array[wp.mat33],
                  b_out:wp.array[wp.mat33],c_out:wp.array[wp.mat33],radius:wp.array[float]):
    tid=wp.tid();f=wp.vec3(0.);g=wp.vec3(0.);kr=wp.vec3(0.);ks=float(0.);c=wp.mat33(0.);bound=float(0.)
    for i in range(tid,q.shape[0],PARTS*128):
        k=mass[i]/(dt*dt);r=q[i]-origin;s=cross_matrix(r);force=k*(target[i]-q[i])
        f+=force;g+=wp.cross(r,force);kr+=k*r;ks+=k;c-=k*s*s;bound=wp.max(bound,wp.length(r))
    fx=wp.tile_sum(wp.tile(f[0]));fy=wp.tile_sum(wp.tile(f[1]));fz=wp.tile_sum(wp.tile(f[2]))
    gx=wp.tile_sum(wp.tile(g[0]));gy=wp.tile_sum(wp.tile(g[1]));gz=wp.tile_sum(wp.tile(g[2]))
    rx=wp.tile_sum(wp.tile(kr[0]));ry=wp.tile_sum(wp.tile(kr[1]));rz=wp.tile_sum(wp.tile(kr[2]));total=wp.tile_sum(wp.tile(ks))
    c00=wp.tile_sum(wp.tile(c[0,0]));c01=wp.tile_sum(wp.tile(c[0,1]));c02=wp.tile_sum(wp.tile(c[0,2]))
    c11=wp.tile_sum(wp.tile(c[1,1]));c12=wp.tile_sum(wp.tile(c[1,2]));c22=wp.tile_sum(wp.tile(c[2,2]));max_r=wp.tile_max(wp.tile(bound))
    if tid%128==0:
        slot=tid//128;f_out[slot]=wp.vec3(fx[0],fy[0],fz[0]);g_out[slot]=wp.vec3(gx[0],gy[0],gz[0])
        a_out[slot]=total[0]*wp.identity(3,wp.float32);b_out[slot]=-cross_matrix(wp.vec3(rx[0],ry[0],rz[0]))
        c_out[slot]=wp.mat33(c00[0],c01[0],c02[0],c01[0],c11[0],c12[0],c02[0],c12[0],c22[0]);radius[slot]=max_r[0]

@wp.kernel
def contact_rigid(q:wp.array[wp.vec3],previous:wp.array[wp.vec3],radius:wp.array[float],count:wp.array[int],
                  indices:wp.array[wp.vec3i],bary:wp.array[wp.vec3],ke:wp.array[float],kd:wp.array[float],mu:wp.array[float],eps:float,
                  shape_body:wp.array[int],body_q:wp.array[wp.transform],body_previous:wp.array[wp.transform],
                  body_qd:wp.array[wp.spatial_vector],body_com:wp.array[wp.vec3],shape:wp.array[int],
                  body_pos:wp.array[wp.vec3],body_vel:wp.array[wp.vec3],normal:wp.array[wp.vec3],margin:wp.array[float],origin:wp.vec3,dt:float,
                  f_out:wp.array[wp.vec3],g_out:wp.array[wp.vec3],a_out:wp.array[wp.mat33],b_out:wp.array[wp.mat33],c_out:wp.array[wp.mat33]):
    tid=wp.tid()
    for i in range(tid,wp.min(count[0],indices.shape[0]),2048):
        if indices[i][0]>=0:
            f,h,cp=_eval_soft_ef_contact(i,indices[i],bary[i],q,previous,radius,ke[i],kd[i],mu[i],eps,
                shape_body,body_q,body_previous,body_qd,body_com,shape,body_pos,body_vel,normal,margin,dt)
            x=wp.vec3(0.)
            for k in range(3):
                j=indices[i][k]
                if j>=0:x+=bary[i][k]*q[j]
            r=x-origin;s=cross_matrix(r);slot=i%PARTS
            wp.atomic_add(f_out,slot,f);wp.atomic_add(g_out,slot,wp.cross(r,f))
            wp.atomic_add(a_out,slot,h);wp.atomic_add(b_out,slot,-h*s);wp.atomic_add(c_out,slot,-s*h*s)

@wp.kernel
def solve_rigid(f_array:wp.array[wp.vec3],g_array:wp.array[wp.vec3],a_array:wp.array[wp.mat33],
                b_array:wp.array[wp.mat33],c_array:wp.array[wp.mat33],radius:wp.array[float],step:wp.array[wp.vec3]):
    f=VecD(wp.float64(0.));g=VecD(wp.float64(0.));a=MatD(wp.float64(0.));b=MatD(wp.float64(0.));c=MatD(wp.float64(0.));bound=float(0.)
    for i in range(PARTS):
        f+=VecD(f_array[i]);g+=VecD(g_array[i]);a+=MatD(a_array[i]);b+=MatD(b_array[i]);c+=MatD(c_array[i]);bound=wp.max(bound,radius[i])
    inv_a=wp.inverse(a);bt=wp.transpose(b);schur=c-bt*inv_a*b
    omega=wp.inverse(schur)*(g-bt*inv_a*f);translation=inv_a*(f-b*omega)
    w=wp.vec3(omega);v=wp.vec3(translation)
    scale=wp.min(.8,.001/wp.max(wp.length(v)+bound*wp.length(w),1.e-12))
    scale=wp.min(scale,.003/wp.max(wp.length(w),1.e-12))
    step[0]=scale*v;step[1]=scale*w

@wp.kernel
def apply_rigid(step:wp.array[wp.vec3],origin:wp.vec3,q:wp.array[wp.vec3],out:wp.array[wp.vec3]):
    i=wp.tid();angle=wp.length(step[1]);rotation=wp.quat_identity()
    if angle>1.e-12:rotation=wp.quat_from_axis_angle(step[1]/angle,angle)
    q[i]=origin+wp.quat_rotate(rotation,q[i]-origin)+step[0];out[i]=q[i]

class RigidSubspace:
    def __init__(self,model,origin):
        self.origin=origin;self.device=model.device
        wp.load_module(module=__name__,device=self.device,block_dim=128)
        wp.load_module(module=__name__,device=self.device,block_dim=256)
        self.f=wp.empty(PARTS,dtype=wp.vec3,device=self.device);self.g=wp.empty_like(self.f)
        self.a=wp.empty(PARTS,dtype=wp.mat33,device=self.device);self.b=wp.empty_like(self.a);self.c=wp.empty_like(self.a)
        self.radius=wp.empty(PARTS,dtype=float,device=self.device);self.step=wp.empty(2,dtype=wp.vec3,device=self.device)
    def apply(self,s,state_in,state_out,contacts,dt):
        m=s.model
        wp.launch(inertia_rigid,dim=PARTS*128,block_dim=128,inputs=[state_in.particle_q,s.inertia,m.particle_mass,self.origin,dt,self.f,self.g,self.a,self.b,self.c,self.radius],device=self.device)
        if contacts is not None:
            c=contacts
            wp.launch(contact_rigid,dim=2048,inputs=[state_in.particle_q,s.particle_q_prev,m.particle_radius,c.soft_contact_count,c.soft_contact_indices,c.soft_contact_barycentric,
                s.body_particle_contact_penalty_k,s.body_particle_contact_material_kd,s.body_particle_contact_material_mu,float(s.friction_epsilon),m.shape_body,state_in.body_q,s.body_q_prev,state_in.body_qd,m.body_com,
                c.soft_contact_shape,c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,self.origin,dt,self.f,self.g,self.a,self.b,self.c],device=self.device)
        wp.launch(solve_rigid,dim=1,inputs=[self.f,self.g,self.a,self.b,self.c,self.radius,self.step],device=self.device)
        wp.launch(apply_rigid,dim=m.particle_count,inputs=[self.step,self.origin,state_in.particle_q,state_out.particle_q],device=self.device)
