"""Objective internal velocity damping preserving rigid-body momenta.

Mass-weighted rigid velocity fit, following Mueller et al., Position Based
Dynamics, section 3.5 (2007). Exponential rate uses physical seconds. Positions
and plastic history are never changed. The default damps only nonrigid velocity.
An explicit supported rigid rate additionally models external support losses;
that option intentionally dissipates linear and angular momenta.
"""
import numpy as np
import warp as wp

PARTS=32


@wp.kernel
def moments(q:wp.array[wp.vec3],v:wp.array[wp.vec3],m:wp.array[float],origin:wp.vec3,
            x:wp.array[wp.vec3],p:wp.array[wp.vec3],l:wp.array[wp.vec3],inertia:wp.array[wp.mat33]):
    lane=wp.tid()
    sx=wp.vec3(0.0);sp=wp.vec3(0.0);sl=wp.vec3(0.0);si=wp.mat33(0.0)
    for i in range(lane,q.shape[0],PARTS):
        r=q[i]-origin;mass=m[i]
        sx+=mass*r;sp+=mass*v[i];sl+=mass*wp.cross(r,v[i])
        si+=mass*(wp.dot(r,r)*wp.identity(3,wp.float32)-wp.outer(r,r))
    x[lane]=sx;p[lane]=sp;l[lane]=sl;inertia[lane]=si


@wp.kernel
def rigid_fit(x:wp.array[wp.vec3],p:wp.array[wp.vec3],l:wp.array[wp.vec3],inertia:wp.array[wp.mat33],
              mass:float,fit:wp.array[wp.vec3]):
    sx=wp.vec3(0.0);sp=wp.vec3(0.0);sl=wp.vec3(0.0);si=wp.mat33(0.0)
    for i in range(PARTS):sx+=x[i];sp+=p[i];sl+=l[i];si+=inertia[i]
    center=sx/mass
    si-=mass*(wp.dot(center,center)*wp.identity(3,wp.float32)-wp.outer(center,center))
    fit[0]=center;fit[1]=sp/mass
    fit[2]=wp.inverse(si+1.e-12*wp.identity(3,wp.float32))*(sl-wp.cross(center,sp))


@wp.kernel
def damp_internal(q:wp.array[wp.vec3],v:wp.array[wp.vec3],origin:wp.vec3,
                  fit:wp.array[wp.vec3],factor:float,rigid_factor:float):
    i=wp.tid()
    rigid=fit[1]+wp.cross(fit[2],q[i]-origin-fit[0])
    v[i]=rigid_factor*rigid+factor*(v[i]-rigid)


class InternalDamping:
    def __init__(self,model,origin,rate):
        if not np.isfinite(rate) or rate<0:raise ValueError('Internal damping rate must be finite and nonnegative')
        self.model=model;self.origin=wp.vec3(*origin);self.rate=float(rate)
        self.mass=float(model.particle_mass.numpy().astype(float).sum())
        self.x=wp.empty(PARTS,dtype=wp.vec3,device=model.device)
        self.p=wp.empty_like(self.x);self.l=wp.empty_like(self.x)
        self.inertia=wp.empty(PARTS,dtype=wp.mat33,device=model.device)
        self.fit=wp.empty(3,dtype=wp.vec3,device=model.device)

    def apply(self,state,dt,rate=None,rigid_rate=0.):
        m=self.model
        damping_rate=self.rate if rate is None else float(rate)
        if not np.isfinite(damping_rate) or damping_rate<0:raise ValueError('Damping rate must be finite and nonnegative')
        if not np.isfinite(rigid_rate) or rigid_rate<0:raise ValueError('Support damping rate must be finite and nonnegative')
        wp.launch(moments,dim=PARTS,inputs=[state.particle_q,state.particle_qd,m.particle_mass,self.origin,self.x,self.p,self.l,self.inertia],device=m.device)
        wp.launch(rigid_fit,dim=1,inputs=[self.x,self.p,self.l,self.inertia,self.mass,self.fit],device=m.device)
        wp.launch(damp_internal,dim=m.particle_count,inputs=[state.particle_q,state.particle_qd,self.origin,self.fit,float(np.exp(-damping_rate*dt)),float(np.exp(-rigid_rate*dt))],device=m.device)
