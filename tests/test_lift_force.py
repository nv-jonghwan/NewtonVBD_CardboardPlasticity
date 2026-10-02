import unittest
import numpy as np
from cardboard.scenario import force_reference


class LiftForceTest(unittest.TestCase):
    def test_zero_lift_preserves_legacy_force(self):
        for phase in range(8):
            for u in np.linspace(0, 1, 11):
                expected=250+(4000-250)*u if phase==5 else (4000 if phase>5 else 250)
                self.assertEqual(force_reference(phase,u,u,250,0,4000,.25),expected)

    def test_continuous_grasp_lift_crush_boundaries(self):
        f=lambda phase,t,u:force_reference(phase,t,u,650,300,6000,.25)
        self.assertEqual(f(3,1,1),f(4,0,0))
        self.assertEqual(f(4,1,1),f(5,0,0))
        self.assertEqual(f(5,1,1),f(6,0,0))
        values=np.array([f(4,u,u) for u in np.linspace(0,1,101)])
        self.assertTrue(np.all(np.diff(values)<=1e-9))
        self.assertGreaterEqual(values.min(),300)
        self.assertLessEqual(values.max(),650)
        self.assertEqual(f(4,.25,.25),300)


if __name__=='__main__':unittest.main()
