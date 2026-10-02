"""Shared meter-scale camera settings for the live viewport and replay renders."""
from pxr import Gf, Sdf, UsdGeom

NEAR_CLIP_M = 0.01
FAR_CLIP_M = 100.0
OVERVIEW = ((1.7, -2.0, 1.65), (.05, 0., 1.02), 24.)
BOX_DETAIL = ((1.05, -.85, 1.25), (.32, 0., .87), 35.)


def set_camera_clipping(camera):
    """USD's default near plane is one stage unit (one meter in our scenes)."""
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(camera.GetPrim().GetStage())
    camera.CreateClippingRangeAttr(Gf.Vec2f(NEAR_CLIP_M/meters_per_unit,
                                          FAR_CLIP_M/meters_per_unit))


def set_camera_view(camera, eye, target, focal_length):
    """Set a view and its real orbit/zoom target, including after a preset switch."""
    eye, target = Gf.Vec3d(*eye), Gf.Vec3d(*target)
    distance = (eye-target).GetLength()
    if distance <= 0:
        raise ValueError('Camera eye and target must differ')
    set_camera_clipping(camera)
    camera.CreateFocalLengthAttr(focal_length)
    camera.CreateFocusDistanceAttr(distance)
    # Kit consumes this camera-local point for orbit and dolly gestures.
    camera.GetPrim().CreateAttribute('omni:kit:centerOfInterest',
        Sdf.ValueTypeNames.Vector3d, custom=True).Set(Gf.Vec3d(0., 0., -distance))
    camera.ClearXformOpOrder()
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
        eye, target, Gf.Vec3d(0., 0., 1.)).GetInverse())
