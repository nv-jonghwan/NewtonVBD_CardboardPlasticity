import unittest
import numpy as np
import warp as wp
from cardboard.rotation_block import inertia_rotation,rotate_block,PARTIALS
from cardboard.block_solver import solve_block

class RotationBlockTest(unittest.TestCase):
    def test_force_hessian_and_rigid_invariance(self):
        rng=np.random.default_rng(89);n=1025;origin=np.array([.3,0,.8],np.float32)
        q=(rng.uniform(-.15,.15,(n,3))+origin).astype(np.float32)
        target=q+np.cross([.0002,-.0001,.0003],q-origin).astype(np.float32)
        m=rng.uniform(.001,.005,n).astype(np.float32);dt=.002
        device='cuda:1';points=wp.array(q,dtype=wp.vec3,device=device);out=wp.empty_like(points)
        f=wp.zeros(PARTIALS,dtype=wp.vec3,device=device);h=wp.zeros(PARTIALS,dtype=wp.mat33,device=device);delta=wp.zeros(1,dtype=wp.vec3,device=device)
        wp.launch(inertia_rotation,dim=PARTIALS*128,block_dim=128,inputs=[points,wp.array(target,dtype=wp.vec3,device=device),wp.array(m,device=device),wp.vec3(*origin),dt,f,h],device=device)
        r=q.astype(float)-origin;k=m.astype(float)/dt**2
        expected_f=np.cross(r,k[:,None]*(target.astype(float)-q)).sum(0)
        expected_h=np.einsum('n,n,ij->ij',k,(r*r).sum(1),np.eye(3))-np.einsum('n,ni,nj->ij',k,r,r)
        np.testing.assert_allclose(f.numpy().sum(0),expected_f,rtol=2e-5,atol=2e-6)
        np.testing.assert_allclose(h.numpy().sum(0),expected_h,rtol=2e-5,atol=.003)
        wp.launch(solve_block,dim=1,inputs=[f,h,.003,delta],device=device)
        wp.launch(rotate_block,dim=n,inputs=[delta,wp.vec3(*origin),points,out],device=device)
        result=out.numpy()
        np.testing.assert_allclose(np.linalg.norm(np.diff(result,axis=0),axis=1),np.linalg.norm(np.diff(q,axis=0),axis=1),atol=2e-7)
        before=np.sum(m[:,None]*(q-target)**2);after=np.sum(m[:,None]*(result-target)**2)
        self.assertLess(after,before*.1)
        np.testing.assert_array_equal(points.numpy(),result)

if __name__=='__main__':unittest.main()
