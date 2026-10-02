"""Supported-shell sleeping, with proximity/load wake and preserved plastic state.

This is a physical sleeping tolerance, not a timeline or rendering filter.
The rigid mechanism and collision detection continue while the shell sleeps.
"""
from collections import deque
import numpy as np
import warp as wp


@wp.kernel
def contact_activity(q:wp.array[wp.vec3],qd:wp.array[wp.vec3],radius:wp.array[float],count:wp.array[int],
                     corners:wp.array[wp.vec3i],bary:wp.array[wp.vec3],
                     shapes:wp.array[int],body:wp.array[int],margin:wp.array[float],
                     position:wp.array[wp.vec3],normal:wp.array[wp.vec3],
                     body_q:wp.array[wp.transform],body_qd:wp.array[wp.spatial_vector],body_com:wp.array[wp.vec3],
                     body_velocity:wp.array[wp.vec3],
                     flags:wp.array[int],bounds:wp.array[float]):
    tid=wp.tid()
    for i in range(tid,wp.min(count[0],corners.shape[0]),1024):
        if corners[i][0]>=0:
            shape=shapes[i]
            x=wp.vec3(0.0);v=wp.vec3(0.0);r=float(0.0)
            for k in range(3):
                vertex=corners[i][k]
                if vertex>=0:
                    x+=bary[i][k]*q[vertex];v+=bary[i][k]*qd[vertex];r=wp.max(r,radius[vertex])
            if body[shape]>=0:
                b=body[shape];pose=body_q[b];point=wp.transform_point(pose,position[i])
                center=wp.transform_point(pose,body_com[b])
                rigid_velocity=wp.spatial_top(body_qd[b])+wp.cross(wp.spatial_bottom(body_qd[b]),point-center)+wp.transform_vector(pose,body_velocity[i])
                gap=wp.dot(normal[i],x-point)-r-margin[shape]
                # Wake for contact or approaching proximity. A nearby gripper
                # moving away must not keep a supported shell needlessly awake.
                if gap<=.0002 or wp.dot(normal[i],v-rigid_velocity)<-.0001:
                    wp.atomic_max(flags,1,1)
            else:
                gap=wp.dot(normal[i],x-position[i])-r-margin[shape]
                if normal[i][2]>.95 and gap<=.0002:
                    wp.atomic_add(flags,0,1)
                    wp.atomic_min(bounds,0,x[0]);wp.atomic_max(bounds,1,x[0])
                    wp.atomic_min(bounds,2,x[1]);wp.atomic_max(bounds,3,x[1])
                    wp.atomic_max(bounds,4,position[i][2])


class RestPolicy:
    """Require small motion of EVERY vertex over a continuous supported window."""
    def __init__(self,quiet_seconds=.25,excursion=.0005,max_speed=.02):
        if not all(np.isfinite(x) and x>0 for x in (quiet_seconds,excursion,max_speed)):
            raise ValueError('Sleep tolerances must be finite and positive')
        self.quiet_seconds=quiet_seconds;self.excursion=excursion;self.max_speed=max_speed
        self.history=deque();self.sleeping=False

    def update(self,t,q,v,supported,disturbed):
        if disturbed or not supported:
            self.history.clear();self.sleeping=False
        elif not self.sleeping:
            if np.linalg.norm(v,axis=1).max()>self.max_speed:
                self.history.clear()
            else:
                self.history.append((t,np.array(q,copy=True)))
                while len(self.history)>1 and self.history[1][0]<=t-self.quiet_seconds:
                    self.history.popleft()
                if t-self.history[0][0]>=self.quiet_seconds-1e-8:
                    # Bounding-box diagonal conservatively bounds pairwise
                    # excursion, including an oscillation returning to its start.
                    positions=np.stack([x[1] for x in self.history])
                    span=np.linalg.norm(np.ptp(positions,axis=0),axis=1).max()
                    self.sleeping=bool(span<=self.excursion)
        return self.sleeping


class SupportedRest:
    def __init__(self,sim):
        self.sim=sim;self.policy=RestPolicy()
        self.flags=wp.zeros(2,dtype=int,device=sim.model.device)
        self.bounds=wp.empty(5,dtype=float,device=sim.model.device)
        self.gravity=None;self.events=[];self.refine=False;self.support_height=None
        self.radius=sim.model.particle_radius.numpy()

    def update(self,q,v,external_change=False):
        s=self.sim;c=s.contacts;m=s.model
        # Consume the last substep's contacts. The rigid/collision pipeline keeps
        # running during sleep; its 6 mm proximity gap exceeds this prescribed
        # mechanism's travel per display frame. Do not invoke a second, eager
        # collision pipeline here: it also changes graph-owned contact buffers.
        self.flags.zero_();self.bounds.assign(np.array([1e6,-1e6,1e6,-1e6,-1e6],np.float32))
        wp.launch(contact_activity,dim=1024,inputs=[s.a.particle_q,s.a.particle_qd,m.particle_radius,
            c.soft_contact_count,c.soft_contact_indices,c.soft_contact_barycentric,
            c.soft_contact_shape,m.shape_body,m.shape_margin,c.soft_contact_body_pos,
            c.soft_contact_normal,s.a.body_q,s.a.body_qd,m.body_com,c.soft_contact_body_vel,self.flags,self.bounds],device=m.device)
        flags=self.flags.numpy();bounds=self.bounds.numpy()
        center=np.average(q,axis=0,weights=s.particle_mass_cpu)
        supported=flags[0]>=3 and bounds[0]<=center[0]<=bounds[1] and bounds[2]<=center[1]<=bounds[3]
        gravity=m.gravity.numpy()
        changed=self.gravity is not None and not np.array_equal(gravity,self.gravity)
        self.gravity=gravity.copy()
        force_applied=bool(np.any(s.a.particle_f.numpy()))
        disturbed=bool(flags[1] or changed or external_change or force_applied or s.time<s.gravity_ramp)
        # More accurate solves near floor equilibrium, even before the center
        # of mass enters the support bounds. Never replace a tipping state by sleep.
        # A nearby gripper must wake the shell, but must NOT switch a loaded
        # floor contact back to the coarser timestep. That numerical compliance
        # change otherwise reintroduces rocking during the robot's retreat.
        loaded=bool(s.rows and max(s.rows[-1]['left_contact_N'],s.rows[-1]['right_contact_N'])>.1)
        if flags[0]>0:self.support_height=float(bounds[4])
        # Retain the accurate timestep across tiny contact separations. Without
        # this hysteresis, contact chatter repeatedly changes inertia dt^-2 and
        # can inject visible motion. A real lift or loading command clears it.
        near_support=self.support_height is not None and float(np.min(q[:,2]-self.radius))<self.support_height+.006
        self.refine=retain_refinement(self.refine,flags[0]>0,near_support,loaded or external_change)
        previous=self.policy.sleeping
        sleeping=self.policy.update(s.time,q,v,supported,disturbed)
        if sleeping!=previous:
            self.events.append({'t':s.time,'sleeping':sleeping})
            print('BOX REST',self.events[-1],flush=True)
        return sleeping


def retain_refinement(previous,contact,near_support,loading):
    return bool(not loading and (contact or (previous and near_support)))
