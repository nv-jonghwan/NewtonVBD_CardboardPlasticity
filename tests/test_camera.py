import unittest
from pxr import Gf, Usd, UsdGeom
from cardboard.camera import BOX_DETAIL, OVERVIEW, set_camera_view


class CameraTest(unittest.TestCase):
    def camera(self, units=1.):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, units)
        return stage, UsdGeom.Camera.Define(stage, '/Camera')

    def test_close_inspection_surface_survives_near_clipping(self):
        stage, camera = self.camera()
        # A surface 20 cm ahead was clipped by the default 1 m plane.
        self.assertGreater(camera.GetClippingRangeAttr().Get()[0], .2)
        eye, target, focal = BOX_DETAIL
        set_camera_view(camera, eye, target, focal)
        forward = (Gf.Vec3d(*target)-Gf.Vec3d(*eye)).GetNormalized()
        point = Gf.Vec3d(*eye)+forward*.2
        frustum = camera.GetCamera().frustum
        self.assertTrue(frustum.Intersects(point))

    def test_orbit_target_updates_when_switching_views(self):
        stage, camera = self.camera()
        for eye, target, focal in (BOX_DETAIL, OVERVIEW, BOX_DETAIL):
            set_camera_view(camera, eye, target, focal)
            local = camera.GetPrim().GetAttribute('omni:kit:centerOfInterest').Get()
            world = UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim()).Transform(local)
            self.assertLess((world-Gf.Vec3d(*target)).GetLength(), 1e-9)
            self.assertAlmostEqual(camera.GetFocusDistanceAttr().Get(), -local[2], places=6)
            self.assertEqual(len(camera.GetOrderedXformOps()), 1)

    def test_clipping_is_physical_distance_in_centimeter_stages(self):
        stage, camera = self.camera(.01)
        set_camera_view(camera, *BOX_DETAIL)
        near, far = camera.GetClippingRangeAttr().Get()
        self.assertAlmostEqual(near*.01, .01)
        self.assertAlmostEqual(far*.01, 100.)


if __name__ == '__main__':
    unittest.main()
