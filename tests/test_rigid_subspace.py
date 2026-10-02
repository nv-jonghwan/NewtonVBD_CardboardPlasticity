import unittest
import numpy as np
import warp as wp
from cardboard.rigid_subspace import inertia_rigid,solve_rigid,apply_rigid,PARTS

class RigidSubspaceTest(unittest.TestCase):
    def setUp(self):
        self.device='cuda:1'
        self.f=wp.zeros(PARTS,dtype=wp.vec3,device=self.device);self.g=wp.zeros_like(self.f)
        self.a=wp.zeros(PARTS,dtype=wp.mat33,device=self.device);self.b=wp.zeros_like(self.a);self.c=wp.zeros_like(self.a)
        self.radius=wp.zeros(PARTS,dtype=float,device=self.device);self.step=wp.zeros(2,dtype=wp.vec3,device=self.device)
    def solve(self):
        wp.launch(solve_rigid,dim=1,inputs=[self.f,self.g,self.a,self.b,self.c,self.radius,self.step],device=self.device)
        return self.step.numpy()
    def test_inertia_against_dense_float64_and_internal_shape_invariant(self):
        rng=np.random.default_rng(18);q=(rng.uniform(-.12,.12,(913,3))+[.3,0,.8]).astype(np.float32);origin=np.array([.31,-.01,.79],np.float32)
        mass=rng.uniform(.001,.005,len(q)).astype(np.float32);dt=.002
        target=q+np.array([.00003,-.00001,.00002],np.float32)+np.cross([.0001,.0002,-.0003],q-origin).astype(np.float32)
        points=wp.array(q,dtype=wp.vec3,device=self.device);out=wp.empty_like(points)
        wp.launch(inertia_rigid,dim=PARTS*128,block_dim=128,inputs=[points,wp.array(target,dtype=wp.vec3,device=self.device),wp.array(mass,device=self.device),wp.vec3(*origin),dt,self.f,self.g,self.a,self.b,self.c,self.radius],device=self.device)
        h=np.zeros((6,6));f=np.zeros(6)
        for pos,y,m in zip(q.astype(float)-origin,target.astype(float)-q,mass.astype(float)):
            x,y0,z=pos;skew=np.array([[0,-z,y0],[z,0,-x],[-y0,x,0]]);j=np.c_[np.eye(3),-skew]
            h+=(m/dt**2)*j.T@j;f+=(m/dt**2)*j.T@y
        actual_h=np.block([[self.a.numpy().sum(0),self.b.numpy().sum(0)],[self.b.numpy().sum(0).T,self.c.numpy().sum(0)]])
        np.testing.assert_allclose(actual_h,h,rtol=2e-5,atol=.005)
        np.testing.assert_allclose(np.r_[self.f.numpy().sum(0),self.g.numpy().sum(0)],f,rtol=2e-5,atol=3e-6)
        step=self.solve();np.testing.assert_allclose(step.ravel(),.8*np.linalg.solve(h,f),rtol=2e-5,atol=3e-8)
        wp.launch(apply_rigid,dim=len(q),inputs=[self.step,wp.vec3(*origin),points,out],device=self.device);result=out.numpy()
        np.testing.assert_allclose(np.linalg.norm(np.diff(result,axis=0),axis=1),np.linalg.norm(np.diff(q,axis=0),axis=1),atol=2e-7)
        self.assertLess(np.sum(mass[:,None]*(result-target)**2),.1*np.sum(mass[:,None]*(q-target)**2))
    def test_sticking_pivot_coupling_matches_dense_solve(self):
        r=np.array([.03,-.04,-.1]);x,y,z=r;s=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);j=np.c_[np.eye(3),-s]
        h=np.diag([1e5]*3+[1e3]*3)+j.T@np.diag([1e8,1e8,1e6])@j
        force=np.array([0,0,-1,.001,.002,0.])
        for array,value in [(self.a,h[:3,:3]),(self.b,h[:3,3:]),(self.c,h[3:,3:]),(self.f,force[:3]),(self.g,force[3:])]:
            values=array.numpy();values[0]=value;array.assign(values)
        radius=self.radius.numpy();radius[0]=.3;self.radius.assign(radius)
        step=self.solve().ravel();expected=.8*np.linalg.solve(h,force)
        np.testing.assert_allclose(step,expected,rtol=3e-4,atol=2e-9)
        self.assertLess(np.linalg.norm(j@step),1e-6)

if __name__=='__main__':unittest.main()
