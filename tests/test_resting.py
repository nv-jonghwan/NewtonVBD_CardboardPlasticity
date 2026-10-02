import unittest
import numpy as np
from cardboard.resting import RestPolicy


class RestTest(unittest.TestCase):
    def test_supported_rest_and_wake(self):
        p=RestPolicy();q=np.zeros((8,3));v=q.copy()
        for i in range(20):
            sleeping=p.update(i/60,q,v,True,False)
        self.assertTrue(sleeping)
        self.assertFalse(p.update(.34,q,v,True,True))
        for i in range(21,42):p.update(i/60,q,v,True,False)
        self.assertTrue(p.sleeping)
        self.assertFalse(p.update(.7,q,v,False,False))

    def test_single_vertex_oscillation_cannot_hide_in_mean(self):
        p=RestPolicy();q=np.zeros((3458,3));v=q.copy()
        for i in range(60):
            q[0,0]=.0004*np.sin(i*np.pi/4)
            self.assertFalse(p.update(i/60,q,v,True,False))

    def test_motion_and_velocity_prevent_sleep(self):
        for fast in [False,True]:
            p=RestPolicy();q=np.zeros((4,3));v=q.copy()
            for i in range(60):
                if fast:v[0,0]=.021
                else:q[0,0]=i*.0001
                self.assertFalse(p.update(i/60,q,v,True,False))

    def test_unsupported_or_changing_load_never_sleeps(self):
        q=np.zeros((4,3))
        for supported,disturbed in [(False,False),(True,True)]:
            p=RestPolicy()
            for i in range(60):self.assertFalse(p.update(i/60,q,q,supported,disturbed))


class ContactActivityTest(unittest.TestCase):
    def test_approach_wakes_retreat_does_not_and_touch_always_wakes(self):
        import warp as wp
        from cardboard.resting import contact_activity
        device='cuda:1'
        arr=lambda x,dtype:wp.array(x,dtype=dtype,device=device)
        for x,velocity,expected in [(.006,.1,1),(.006,-.1,0),(.004,-.1,1),(.006,0.,0)]:
            flags=wp.zeros(2,dtype=int,device=device);bounds=arr([1e6,-1e6,1e6,-1e6,-1e6],float)
            args=[arr([[x,0,0]],wp.vec3),arr([[0,0,0]],wp.vec3),arr([.005],float),arr([1],int),
                  arr([[0,-1,-1]],wp.vec3i),arr([[1,0,0]],wp.vec3),arr([0],int),arr([0],int),arr([0.],float),
                  arr([[0,0,0]],wp.vec3),arr([[1,0,0]],wp.vec3),arr([[0,0,0,0,0,0,1]],wp.transform),
                  arr([[velocity,0,0,0,0,0]],wp.spatial_vector),arr([[0,0,0]],wp.vec3),arr([[0,0,0]],wp.vec3),flags,bounds]
            wp.launch(contact_activity,dim=1024,inputs=args,device=device)
            self.assertEqual(flags.numpy().tolist(),[0,expected])



class RefinementHysteresisTest(unittest.TestCase):
    def test_contact_chatter_cannot_toggle_timestep(self):
        from cardboard.resting import retain_refinement
        active=False
        for contact in [True,False,False,True,False]:
            active=retain_refinement(active,contact,True,False)
            self.assertTrue(active)
        self.assertFalse(retain_refinement(active,False,False,False))
        self.assertFalse(retain_refinement(active,True,True,True))
        self.assertFalse(retain_refinement(False,False,True,False))

if __name__=='__main__':unittest.main()
