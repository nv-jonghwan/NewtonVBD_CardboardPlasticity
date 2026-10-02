import unittest
import numpy as np
import warp as wp
from cardboard.plasticity import project_scalar, return_map

class ReturnMappingTest(unittest.TestCase):
    def test_below_yield_recovers(self):
        p,a,w=project_scalar(.02,0,0,0,2,.1,.05)
        self.assertEqual((p,a,w),(0,0,0))
    def test_yield_surface_and_positive_work(self):
        p,a,w=project_scalar(.4,0,0,0,2,.1,.05)
        self.assertGreater(p,0);self.assertGreater(w,0)
        self.assertAlmostEqual(abs(2*(.4-p)),.1+.05*a)
        p2,a2,w2=project_scalar(p,0,p,a,2,.1,.05)
        self.assertEqual((p2,a2,w2),(p,a,0))
    def test_gpu_return_matches_independent_scalar_reference(self):
        # Edge along +X, first triangle +Z normal, second rotated by angle.
        theta=.4
        q=np.array([[0,1,0],[0,-np.cos(theta),-np.sin(theta)],[0,0,0],[1,0,0]],np.float32)
        with wp.ScopedDevice('cuda:1'):
            out=[wp.zeros(1,dtype=float) for _ in range(4)];rest=wp.zeros(1,dtype=float);props=wp.array([[2.,0]],dtype=float)
            wp.launch(return_map,dim=1,inputs=[wp.array(q,dtype=wp.vec3),wp.array([[0,1,2,3]],dtype=int),wp.array([1.],dtype=float),wp.array([0.],dtype=float),wp.array([.1],dtype=float),wp.array([2.],dtype=float),.5,.025,0.,.25,1,0.,wp.zeros(1),wp.zeros(1),wp.zeros(1),*out,rest,props])
            p,a,w=[x.numpy()[0] for x in out[:3]]
            expected=project_scalar(theta,0,0,0,2,.1,.05)
            np.testing.assert_allclose([p,a,w],expected,rtol=1e-5,atol=1e-6)
            self.assertAlmostEqual(float(rest.numpy()[0]),float(p))
    def test_relaxation_damping_tracks_damage_and_preserves_legacy_mode(self):
        q=np.array([[0,1,0],[0,-np.cos(.4),-np.sin(.4)],[0,0,0],[1,0,0]],np.float32)
        with wp.ScopedDevice('cuda:0'):
            for tau in [0.,.006]:
                out=[wp.zeros(1,dtype=float) for _ in range(4)];rest=wp.zeros(1);props=wp.array([[2.,.37]],dtype=float)
                wp.launch(return_map,dim=1,inputs=[wp.array(q,dtype=wp.vec3),wp.array([[0,1,2,3]],dtype=int),wp.array([1.],dtype=float),wp.array([0.],dtype=float),wp.array([.1],dtype=float),wp.array([2.],dtype=float),.5,.025,.5,.65,1,tau,wp.zeros(1),wp.zeros(1),wp.zeros(1),*out,rest,props])
                stiffness,damping=props.numpy()[0]
                self.assertLess(stiffness,2.);self.assertGreaterEqual(stiffness,1.3)
                self.assertAlmostEqual(float(damping),float(tau*stiffness if tau else .37),places=6)
if __name__=='__main__':unittest.main()
