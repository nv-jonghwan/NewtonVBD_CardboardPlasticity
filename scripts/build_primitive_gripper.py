"""Keep distal gripper box colliders and unchanged vendor render geometry.

Fingertips retain their planar contact face; neighboring links use a minimum
area rectangle in the source mesh XY plane, extruded through mesh Z bounds.
All remaining gripper collision shapes are disabled, including the palm.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from cardboard import ROOT


def box_frame(points, fingertip=False):
    angles = [0.0]
    if not fingertip:
        hull = points[ConvexHull(points[:, :2]).vertices, :2]
        edges = np.roll(hull, -1, axis=0) - hull
        angles = np.arctan2(edges[:, 1], edges[:, 0])
    best = None
    for angle in angles:
        c, s = np.cos(angle), np.sin(angle)
        axes = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        projected = points @ axes.T
        lo, hi = projected.min(0), projected.max(0)
        volume = float(np.prod(hi-lo))
        if best is None or volume < best[0]:
            best = volume, axes, lo, hi
    _, axes, lo, hi = best
    matrix = np.eye(4)
    matrix[:3, :3] = (hi-lo)[:, None] * axes
    matrix[3, :3] = ((lo+hi)*.5) @ axes
    return Gf.Matrix4d(*matrix.ravel().tolist())


def build(source, output, lift_height):
    stage = Usd.Stage.Open(str(source))
    stage.GetRootLayer().Export(str(output))
    stage = Usd.Stage.Open(str(output))
    cache = UsdGeom.XformCache()
    retained = {'Left', 'Right', 'left_outer_finger', 'right_outer_finger',
                'left_inner_knuckle', 'right_inner_knuckle'}
    sources = []
    disabled = []
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath('/World/Gripper'))):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        collision = UsdPhysics.CollisionAPI(prim)
        if collision.GetCollisionEnabledAttr().Get():
            disabled.append(str(prim.GetPath()))
            if prim.IsA(UsdGeom.Mesh):
                body = prim
                while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if body.GetName() in retained:
                    sources.append((prim, body))
        collision.CreateCollisionEnabledAttr(False)
        # Disabled CollisionAPI prims are still imported as non-colliding
        # shapes by Newton1.6. Remove the APIs as well so the physics-only
        # importer omits them entirely; vendor render geometry is preserved.
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    for name in ['Leg0', 'Leg1', 'Leg2', 'Leg3']:
        prim = stage.GetPrimAtPath('/World/'+name)
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    report = []
    for prim, body in sources:
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        frame = box_frame(points, fingertip=prim.GetName() == 'Fingertip')
        relative = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()
        cube = UsdGeom.Cube.Define(stage, body.GetPath().AppendChild('Collider_'+prim.GetName()))
        cube.CreateSizeAttr(1.0)
        cube.AddTransformOp().Set(frame * relative)
        cube.CreatePurposeAttr('guide')
        cube.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
        if material:
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, materialPurpose='physics')
        cube.GetPrim().CreateAttribute('cardboard:sourceCollider', Sdf.ValueTypeNames.String).Set(str(prim.GetPath()))
        report.append(dict(source=str(prim.GetPath()), collider=str(cube.GetPath()),
                           dimensions_m=np.linalg.norm(np.asarray(frame*relative)[:3, :3], axis=1).tolist()))
    assert len(report) == 8, report
    stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:liftHeight').Set(lift_height)
    stage.GetRootLayer().Save()
    return dict(scene=str(output.relative_to(ROOT)), lift_height_m=lift_height,
                disabled_colliders=disabled, disabled_table_legs=['Leg0','Leg1','Leg2','Leg3'], boxes=report)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', default='assets/demo_scene_robotiq_board10_support20_trial.usda')
    parser.add_argument('--output', default='assets/demo_scene_robotiq_board10_primitive.usda')
    parser.add_argument('--lift-height', type=float, default=.20)
    args = parser.parse_args()
    report = build(ROOT/args.source, ROOT/args.output, args.lift_height)
    folder = ROOT/'outputs/primitive_gripper'
    folder.mkdir(exist_ok=True, parents=True)
    (folder/(Path(args.output).stem+'_build.json')).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
