"""Build a hypothetical enlarged Robotiq, keeping vendor source files immutable.

Length s, mass s^3 and inertia s^5 are constant-density assumptions, not a
commercial Robotiq model or a qualified UR10 payload. Native loop joints retained.
"""
import argparse,json
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
from cardboard import ROOT
from cardboard.usd_utils import register


def build(scale=2.3, iterations=32, substeps=16):
    if scale*.14<.30:raise ValueError("This scene needs at least 300mm opening for the 280mm box")
    if iterations<1 or substeps<1:raise ValueError("Solver counts must be positive")
    register()
    source_path=ROOT/'assets/source/robotiq_2f140/Robotiq_2F_140_physics_edit.usd'
    source=Usd.Stage.Open(str(source_path)); cache=UsdGeom.XformCache()
    bodies=[p for p in source.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    base=source.GetPrimAtPath('/Robotiq_2F_140/robotiq_base_link')
    base_inverse=cache.GetLocalToWorldTransform(base).GetInverse()
    names={'robotiq_base_link':'Palm','left_inner_finger':'Left','right_inner_finger':'Right'}
    paths={str(p.GetPath()):'/World/Gripper/'+names.get(p.GetName(),p.GetName()) for p in bodies}
    path=ROOT/'assets/demo_scene_robotiq_scaled.usda'
    original=Usd.Stage.Open(str(ROOT/'assets/demo_scene_board15_half.usda'))
    original.GetRootLayer().Export(str(path)); scene=Usd.Stage.Open(str(path))
    scene.RemovePrim('/World/Gripper'); UsdGeom.Xform.Define(scene,'/World/Gripper')
    ee=UsdGeom.XformCache().GetLocalToWorldTransform(scene.GetPrimAtPath('/World/UR10/ee_link'))
    # The source ee_joint maps wrist_3 +Z (the physical flange normal) to
    # ee_link +X. Robotiq base +Z must follow that normal, not ee_link +Z.
    # USD matrices use row vectors: gripper X,Y,Z map to ee Y,Z,X.
    mount=Gf.Matrix4d(0,1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1)
    base_bottom=float(UsdGeom.BBoxCache(0,['default','render']).ComputeRelativeBound(base,base).ComputeAlignedRange().GetMin()[2])*scale
    adapter_height=.025
    base_offset=adapter_height-base_bottom
    mount.SetTranslateOnly(Gf.Vec3d(base_offset,0,0)); placement=mount*ee
    mass=0.
    for p in bodies:
        dst=paths[str(p.GetPath())]; body=UsdGeom.Xform.Define(scene,dst)
        relative=cache.GetLocalToWorldTransform(p)*base_inverse
        relative.SetTranslateOnly(relative.ExtractTranslation()*scale)
        body.AddTransformOp().Set(relative*placement)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        m=UsdPhysics.MassAPI.Apply(body.GetPrim());old=UsdPhysics.MassAPI(p)
        m.CreateMassAttr(float(old.GetMassAttr().Get() or .65)*scale**3);mass+=m.GetMassAttr().Get()
        m.CreateCenterOfMassAttr(Gf.Vec3f(old.GetCenterOfMassAttr().Get() or Gf.Vec3f(0,0,.04))*scale)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(old.GetDiagonalInertiaAttr().Get() or Gf.Vec3f(.0006,.0006,.0005))*scale**5)
        m.CreatePrincipalAxesAttr(old.GetPrincipalAxesAttr().Get() or Gf.Quatf(1))
        geo=UsdGeom.Xform.Define(scene,dst+'/Geometry')
        geo.GetPrim().GetReferences().AddReference('./source/robotiq_2f140/Robotiq_2F_140_physics_edit.usd',p.GetPath())
        geo.GetPrim().SetInstanceable(False)
        # Instance-local specs hidden by the vendor payload would reappear on
        # de-instancing. Preserve exactly the source's composed visible children.
        allowed={c.GetName() for c in Usd.PrimRange(p,Usd.TraverseInstanceProxies()) if c.GetParent()==p}
        for child in geo.GetPrim().GetChildren():
            if child.GetName() not in allowed:child.SetActive(False)
        geo.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit([a for a in geo.GetPrim().GetAppliedSchemas() if a not in ['PhysicsRigidBodyAPI','PhysicsMassAPI','PhysxRigidBodyAPI']]))
        geo.ClearXformOpOrder();geo.AddScaleOp().Set(Gf.Vec3f(scale))
    # The vendor authors collision on Xforms; Newton requires geometric prims.
    for prim in list(Usd.PrimRange(scene.GetPrimAtPath('/World/Gripper'))):
        if prim.IsInstance():prim.SetInstanceable(False)
    for prim in Usd.PrimRange(scene.GetPrimAtPath('/World/Gripper')):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        elif prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.IsA(UsdGeom.Gprim):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
    rubber=UsdShade.Material.Define(scene,'/World/Gripper/RubberPhysics')
    pm=UsdPhysics.MaterialAPI.Apply(rubber.GetPrim());pm.CreateStaticFrictionAttr(.8);pm.CreateDynamicFrictionAttr(.7)
    for prim in Usd.PrimRange(scene.GetPrimAtPath('/World/Gripper')):
        if prim.HasRelationship('material:binding:physics'):
            prim.GetRelationship('material:binding:physics').SetTargets(['/World/Gripper/RubberPhysics'])
    for p in source.Traverse():
        if not p.IsA(UsdPhysics.Joint):continue
        dst='/World/Gripper/'+p.GetName();j=UsdPhysics.RevoluteJoint.Define(scene,dst)
        for end in (0,1):
            old=str(p.GetRelationship('physics:body'+str(end)).GetTargets()[0])
            j.GetPrim().CreateRelationship('physics:body'+str(end)).SetTargets([paths[old]])
            pos=p.GetAttribute('physics:localPos'+str(end)).Get()
            j.GetPrim().CreateAttribute('physics:localPos'+str(end),Sdf.ValueTypeNames.Point3f).Set(Gf.Vec3f(pos)*scale)
            rot=p.GetAttribute('physics:localRot'+str(end)).Get()
            j.GetPrim().CreateAttribute('physics:localRot'+str(end),Sdf.ValueTypeNames.Quatf).Set(rot)
        j.CreateAxisAttr(p.GetAttribute('physics:axis').Get())
        j.CreateLowerLimitAttr(p.GetAttribute('physics:lowerLimit').Get());j.CreateUpperLimitAttr(p.GetAttribute('physics:upperLimit').Get())
        j.CreateCollisionEnabledAttr(False)
        if 'pad_joint' in p.GetName():j.CreateExcludeFromArticulationAttr(True)
    j=UsdPhysics.FixedJoint.Define(scene,'/World/Gripper/Mount')
    j.CreateBody0Rel().SetTargets(['/World/UR10/ee_link']);j.CreateBody1Rel().SetTargets(['/World/Gripper/Palm'])
    j.CreateLocalPos0Attr(Gf.Vec3f(base_offset,0,0));q=mount.ExtractRotationQuat();j.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(),Gf.Vec3f(q.GetImaginary())))
    # Two-piece virtual adapter: stem starts on the robot flange plane;
    # wider top plate meets the scaled base's actual bottom surface.
    for name,radius,height,center in [('FlangeAdapter',.038,.018,.009),('AdapterPlate',.038*scale,.007,.0215)]:
        adapter=UsdGeom.Cylinder.Define(scene,'/World/Gripper/Palm/'+name)
        adapter.CreateRadiusAttr(radius);adapter.CreateHeightAttr(height)
        adapter.AddTranslateOp().Set(Gf.Vec3d(0,0,center-base_offset))
        UsdPhysics.CollisionAPI.Apply(adapter.GetPrim())
    phys=scene.GetPrimAtPath('/World/Physics')
    for k,v in dict(openGap=.14*scale,graspGap=.27,crushGap=.14,iterations=iterations,substeps=substeps,precomputeCommands=True,realtimePacing=False,graspForce=250.,crushForce=4000.).items():phys.GetAttribute('cardboard:'+k).Set(v)
    for k,t,v in [('gripperModel',Sdf.ValueTypeNames.String,'robotiq_scaled'),('gripperScale',Sdf.ValueTypeNames.Float,scale),('toolReach',Sdf.ValueTypeNames.Float,base_offset+.17*scale)]:phys.CreateAttribute('cardboard:'+k,t,custom=True).Set(v)
    # Hold after grasp/lift and use zero-end-acceleration motion. These settings
    # were validated together with 32x16 iterations and unchanged box material.
    for k,v in dict(graspRampFraction=.7,liftMoveFraction=.85,robotiqDriveStiffness=8000.,robotiqDriveDamping=80.,robotiqPinchStiffness=8000.,robotiqPinchDamping=80.).items():
        phys.CreateAttribute('cardboard:'+k,Sdf.ValueTypeNames.Float,custom=True).Set(v)
    phys.CreateAttribute('cardboard:smoothMotion',Sdf.ValueTypeNames.Bool,custom=True).Set(True)
    # Display-only refinement for the coarse thick shell: retain deep creases,
    # but avoid face-normal thickness steps and medium-angle grid faceting.
    render=scene.GetPrimAtPath('/World/Box/RenderMesh')
    render.GetAttribute('cardboard:visualCreaseAngleDegrees').Set(100.)
    render.GetAttribute('cardboard:visualCreaseTransitionDegrees').Set(40.)
    render.CreateAttribute('cardboard:visualSmoothThickness',Sdf.ValueTypeNames.Bool,custom=True).Set(True)
    phys.GetRelationship('cardboard:fingerJoints').SetTargets(['/World/Gripper/finger_joint','/World/Gripper/right_outer_knuckle_joint'])
    scene.GetPrimAtPath('/World/Gripper').SetCustomDataByKey('qualification',f'Hypothetical Robotiq 2F140 {scale}x; constant-density mass and inertia scaling; virtual drives; not a qualified UR10 payload.')
    scene.GetRootLayer().Save()
    print(json.dumps({'scene':str(path),'scale':scale,'mass_kg':mass,'nominal_open_gap_m':.14*scale},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scale',type=float,default=2.3);p.add_argument('--iterations',type=int,default=32);p.add_argument('--substeps',type=int,default=16);a=p.parse_args();build(a.scale,a.iterations,a.substeps)
