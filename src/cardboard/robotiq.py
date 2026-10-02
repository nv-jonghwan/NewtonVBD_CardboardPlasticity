"""Virtual enlarged Robotiq: native closed loops with two symmetric input drives.

Two implicit drives approximate the source synchronizing transmission;
spring-effort references are saturated, but damping/contact reactions are not
hard force bounds. This is not the real product's adaptive transmission model.
"""
import numpy as np
import warp as wp
from pxr import UsdGeom
from scipy.spatial.transform import Rotation

@wp.kernel
def drive(q:wp.array[float],qd:wp.array[float],target:wp.array[float],force_limit:wp.array[float],indices:wp.array[int],scale:float,stiffness:float,out:wp.array[float],implicit_target:wp.array[float]):
    i=wp.tid();out[i]=0.;implicit_target[i]=target[i]
    if i==indices[0] or i==indices[1]:
        limit=force_limit[0]*.08*scale
        current=wp.atan2(wp.sin(q[i]),wp.cos(q[i]))
        implicit_target[i]=current+wp.clamp(target[i]-current,-limit/stiffness,limit/stiffness)
    if i==indices[2] or i==indices[3]:
        implicit_target[i]=0.

class RobotiqDrive:
    def __init__(self,sim,b):
        self.scale=float(sim.attr(sim.settings,'gripperScale'))
        names=['finger_joint','right_outer_knuckle_joint','left_outer_finger_joint','right_outer_finger_joint']
        self.indices=[b.joint_q_start[b.joint_label.index('/World/Gripper/'+n)] for n in names]
        self.gpu_indices=wp.array(self.indices,dtype=int)
        self.stiffness=float(sim.attr(sim.settings,'robotiqDriveStiffness') or 500.)
        damping=float(sim.attr(sim.settings,'robotiqDriveDamping') or 8.)
        pinch_stiffness=float(sim.attr(sim.settings,'robotiqPinchStiffness') or 500.)
        pinch_damping=float(sim.attr(sim.settings,'robotiqPinchDamping') or 8.)
        for index,i in enumerate(self.indices):
            b.joint_target_ke[i]=self.stiffness if index<2 else pinch_stiffness
            b.joint_target_kd[i]=damping if index<2 else pinch_damping
        self.left=sim.left;self.right=sim.right
        # Source rubber pad inner-face sample at its middle, converted to body frame.
        self.pad=[]
        for name in ['Left','Right']:
            body=sim.stage.GetPrimAtPath('/World/Gripper/'+name)
            prim=sim.stage.GetPrimAtPath('/World/Gripper/'+name+'/Geometry/Fingertip_01')
            bb=UsdGeom.BBoxCache(0,['default','render']).ComputeRelativeBound(prim,body).ComputeAlignedRange()
            # Source inner-finger local X points along pad length; Y closes.
            lo=np.array(bb.GetMin());hi=np.array(bb.GetMax());v=(lo+hi)/2
            v[1]=lo[1] if name=='Left' else hi[1]
            self.pad.append(v)
        from .kinematics import RobotIK
        self.passive_indices=[b.joint_q_start[b.joint_label.index('/World/Gripper/'+n)] for n in ['left_inner_finger_joint','right_inner_finger_joint','left_inner_knuckle_joint','right_inner_knuckle_joint']]
        ik=RobotIK(b,sim.tip);q=np.array(b.joint_q,dtype=float)
        fk=ik.fk(q);palm=b.body_label.index('/World/Gripper/Palm')
        ee_to_palm=np.linalg.inv(fk[sim.tip])@fk[palm]
        self.approach_axis=ee_to_palm[:3,2]
        # Preserve downward fingers and closing along world X while respecting
        # the actual ee-to-gripper mounting rotation.
        downward_palm=Rotation.from_euler('x',np.pi).as_matrix()@Rotation.from_euler('z',np.pi/2).as_matrix()
        self.downward_ee_rotation=downward_palm@ee_to_palm[:3,:3].T
        self.angles=np.linspace(0,np.pi/4,201);gaps=[];depths=[]
        for angle in self.angles:
            self.set_angle(q,angle);f=ik.fk(q)
            pts=[f[i][:3,3]+f[i][:3,:3]@p for i,p in zip([self.left,self.right],self.pad)]
            gaps.append(np.linalg.norm(pts[0]-pts[1]))
            depths.append(self.approach_axis@(f[sim.tip][:3,:3].T@((pts[0]+pts[1])/2-f[sim.tip][:3,3])))
        self.gaps=np.array(gaps);self.depths=np.array(depths)
        if not np.all(np.diff(self.gaps)<0):raise ValueError('Non-monotone Robotiq pinch opening')
    def set_angle(self,q,angle):
        q[self.indices[:2]]=angle
        q[self.passive_indices]=[-angle,-angle,angle,-angle]
    def angle(self,gap):
        return float(np.interp(gap,self.gaps[::-1],self.angles[::-1]))
    def set_gap(self,q,gap):
        self.set_angle(q,self.angle(gap))
    def depth_shift(self,gap):
        return float(np.interp(self.angle(gap),self.angles,self.depths)-np.interp(self.angle(.14*self.scale),self.angles,self.depths))
    def apply(self,s):
        wp.launch(drive,dim=s.dof_count,inputs=[s.a.joint_q,s.a.joint_qd,s.target_array,s.motor_force_limit,self.gpu_indices,self.scale,self.stiffness,s.control.joint_f,s.control.joint_target_q])
    def gap(self,body):
        points=[body[i,:3]+Rotation.from_quat(body[i,3:]).apply(p) for i,p in zip([self.left,self.right],self.pad)]
        return float(np.linalg.norm(points[0]-points[1]))
