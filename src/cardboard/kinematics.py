import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

def matrix(t):
    a=np.asarray(t,dtype=float);m=np.eye(4);m[:3,:3]=Rotation.from_quat(a[3:]).as_matrix();m[:3,3]=a[:3];return m

class RobotIK:
    def __init__(self,builder,tip):
        self.parent=list(builder.joint_parent);self.child=list(builder.joint_child);self.types=list(builder.joint_type)
        self.xp=[matrix(t) for t in builder.joint_X_p];self.xc=[np.linalg.inv(matrix(t)) for t in builder.joint_X_c]
        self.qstart=list(builder.joint_q_start);self.dstart=list(builder.joint_qd_start);self.axes=np.asarray(builder.joint_axis);self.tip=tip
        self.body_count=builder.body_count
    def fk(self,q):
        out=[np.eye(4) for _ in range(self.body_count)]
        visited=set()
        for j,(p,c) in enumerate(zip(self.parent,self.child)):
            if c in visited:continue  # Loop closures are solved dynamically, not FK.
            visited.add(c)
            motion=np.eye(4);typ=int(self.types[j]);k=self.qstart[j];d=self.dstart[j]
            # newton JointType: PRISMATIC=0, REVOLUTE=1, FIXED=3
            if typ==0:motion[:3,3]=self.axes[d]*q[k]
            elif typ==1:motion[:3,:3]=Rotation.from_rotvec(self.axes[d]*q[k]).as_matrix()
            out[c]=(out[p] if p>=0 else np.eye(4))@self.xp[j]@motion@self.xc[j]
        return out
    def solve(self,target,seed):
        seed=np.asarray(seed,float)
        def residual(arm):
            q=seed.copy();q[:6]=arm;t=self.fk(q)[self.tip]
            return np.r_[t[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3]@t[:3,:3].T).as_rotvec()*.3]
        r=least_squares(residual,seed[:6],max_nfev=150,xtol=1e-11,ftol=1e-11,gtol=1e-11)
        q=seed.copy();q[:6]=r.x
        if np.linalg.norm(residual(r.x))>.001:raise RuntimeError(f'UR10 IK failed: {residual(r.x)}')
        return q
