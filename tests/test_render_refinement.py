import collections
import unittest
import numpy as np
from cardboard.geometry import refine_render

class RenderRefinementTest(unittest.TestCase):
    def test_refinement_keeps_closed_surface_and_uv_mapping(self):
        p=np.array([[0,0,0],[1,0,0],[0,.7,0],[0,0,.4]],dtype=np.float32)
        f=np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]],dtype=np.int32)
        q,g,uv=refine_render(p,f,p[:,:2],.12)
        np.testing.assert_allclose(uv,q[:,:2],atol=1e-7)
        self.assertLessEqual(np.linalg.norm(np.roll(q[g],-1,axis=1)-q[g],axis=2).max(),.120001)
        _,indices=np.unique(np.round(q,6),axis=0,return_inverse=True)
        triangles=indices[g]
        edges=collections.Counter(tuple(sorted(e)) for t in triangles for e in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])])
        self.assertEqual(set(edges.values()),{2},'Refinement created a crack or T-junction')
        def volume(x,t):return np.einsum('ij,ij->i',x[t[:,0]],np.cross(x[t[:,1]],x[t[:,2]])).sum()/6
        self.assertAlmostEqual(float(volume(p,f)),float(volume(q,g)),places=6)

if __name__=='__main__':unittest.main()
