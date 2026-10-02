import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from cardboard.scenario import tilt_about_grasp_center


class GripperTiltTest(unittest.TestCase):
    def test_zero_tilt_preserves_exact_pose(self):
        pose=np.eye(4);pose[:3,3]=[.3,.2,1.1]
        pose[:3,:3]=Rotation.from_euler('xyz',[31,19,-80],degrees=True).as_matrix()
        np.testing.assert_array_equal(tilt_about_grasp_center(pose,np.array([.3,.2,.8]),np.zeros(3)),pose)

    def test_grasp_center_stays_fixed_in_tool_frame(self):
        center=np.array([.3,.2,.8]);pose=np.eye(4);pose[:3,3]=center+[0,0,.38]
        pose[:3,:3]=Rotation.from_euler('x',180,degrees=True).as_matrix()
        point_in_tool=pose[:3,:3].T@(center-pose[:3,3])
        for angles in ([10,0,0],[30,0,0],[20,-15,5]):
            result=tilt_about_grasp_center(pose,center,angles)
            np.testing.assert_allclose(result[:3,3]+result[:3,:3]@point_in_tool,center,atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(result[:3,3]-center),.38)
            self.assertAlmostEqual(np.linalg.det(result[:3,:3]),1.)
        result=tilt_about_grasp_center(pose,center,[30,0,0])
        np.testing.assert_allclose(result[:3,3],center+[0,-.19,.38*np.sqrt(3)/2],atol=1e-12)
        np.testing.assert_allclose(pose[:3,3],center+[0,0,.38],atol=1e-12)


class GripperTiltDefaultsTest(unittest.TestCase):
    def build(self,lift,crush=None,release=None):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from pxr import Usd,Gf
        from cardboard.usd_utils import register
        from cardboard.scenario import PickCrushDrop,Simulation
        register()
        def fake_init(sim,**kwargs):
            sim.home=np.zeros(6);sim.settings=None;sim.attr=lambda prim,name:False
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.usda';stage=Usd.Stage.CreateNew(str(path));prim=stage.DefinePrim('/World/Physics');self.assertTrue(prim.ApplyAPI('CardboardScenarioAPI'))
            prim.GetAttribute('cardboard:liftTiltDegrees').Set(Gf.Vec3f(*lift))
            if crush is not None:prim.GetAttribute('cardboard:crushTiltDegrees').Set(Gf.Vec3f(*crush))
            if release is not None:prim.GetAttribute('cardboard:releaseTiltDegrees').Set(Gf.Vec3f(*release))
            stage.GetRootLayer().Save()
            with patch.object(Simulation,'__init__',fake_init):return PickCrushDrop(scene_path=path).config

    def test_missing_crush_angle_inherits_lift(self):
        c=self.build([30,0,0]);np.testing.assert_array_equal(c['crushTiltDegrees'],[30,0,0])

    def test_explicit_vertical_crush_is_not_treated_as_missing(self):
        c=self.build([30,0,0],[0,0,0]);np.testing.assert_array_equal(c['crushTiltDegrees'],[0,0,0])

    def test_nonfinite_crush_angle_is_rejected_before_simulation(self):
        with self.assertRaises(ValueError):self.build([30,0,0],[float('nan'),0,0])

    def test_missing_release_inherits_crush(self):
        c=self.build([30,0,0],[45,0,0]);np.testing.assert_array_equal(c['releaseTiltDegrees'],[45,0,0])

    def test_explicit_zero_release_is_not_inherited(self):
        c=self.build([30,0,0],[45,0,0],[0,0,0]);np.testing.assert_array_equal(c['releaseTiltDegrees'],[0,0,0])

    def test_nonfinite_release_is_rejected(self):
        with self.assertRaises(ValueError):self.build([30,0,0],[45,0,0],[0,float('nan'),0])

if __name__=='__main__':unittest.main()
