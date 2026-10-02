import unittest
import numpy as np
import warp as wp
from cardboard.simulation import set_kinematic_arm

class KinematicArmTest(unittest.TestCase):
    def test_prescribed_arm_does_not_overwrite_dynamic_fingers(self):
        start=np.array([[0,0,0,0,0,0,1]]*3,dtype=np.float32)
        end=start.copy();end[0,:3]=[1,2,3]
        current=start.copy();current[1,:3]=[4,5,6];current[2,:3]=[-4,-5,-6]
        velocity=np.arange(18,dtype=np.float32).reshape(3,6)
        q=wp.array(current,dtype=wp.transform,device='cpu');qd=wp.zeros(3,dtype=wp.spatial_vector,device='cpu')
        wp.launch(set_kinematic_arm,dim=3,inputs=[wp.array(start,dtype=wp.transform,device='cpu'),wp.array(end,dtype=wp.transform,device='cpu'),wp.array(velocity,dtype=wp.spatial_vector,device='cpu'),wp.array([0,1,1],dtype=int,device='cpu'),.25,q,qd],device='cpu')
        np.testing.assert_allclose(q.numpy()[0,:3],[.25,.5,.75])
        np.testing.assert_array_equal(q.numpy()[1:],current[1:])
        np.testing.assert_array_equal(qd.numpy()[0],velocity[0])
        np.testing.assert_array_equal(qd.numpy()[1:],0)

if __name__=='__main__':unittest.main()
