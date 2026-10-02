"""Author portable NVIDIA-derived box asset and standard USD robot/table scene."""
import json
import numpy as np
import newton
import warp as wp
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf,Vt
from . import ROOT
from .usd_utils import register,apply,set_attr,preview_material,cube
from .geometry import shell_grid,source_render,bind,refine_render

def build(n=12, variant='', render_max_edge=None):
    register(); out=ROOT/'assets'; file=out/f'cardboard{variant}.usda'
    render,rfaces,uv,size=source_render(out/'source/cardbox_a1/cardbox_a1.usd')
    if render_max_edge is not None:render,rfaces,uv=refine_render(render,rfaces,uv,render_max_edge)
    p,f,directions=shell_grid(size,n)
    b=newton.ModelBuilder();b.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1,vel=wp.vec3(0),vertices=p,indices=f.ravel(),density=.65,edge_ke=1)
    edges=np.array(b.edge_indices); ref=np.array(b.edge_rest_angle)
    e=p[edges[:,3]]-p[edges[:,2]];length=np.linalg.norm(e,axis=1)
    # h = mean of triangle altitudes: discrete hinge energy D*l/h*(theta-theta_p)^2/2.
    h0=np.linalg.norm(np.cross(p[edges[:,0]]-p[edges[:,2]],e),axis=1)/length
    h1=np.linalg.norm(np.cross(p[edges[:,1]]-p[edges[:,2]],e),axis=1)/length
    dual=(h0+h1)*.5
    # Directional bending surrogate, not a full orthotropic membrane constitutive law.
    across=e/length[:,None]; md_weight=across[:,1]**2
    defaults=Usd.Stage.CreateInMemory();material=defaults.DefinePrim('/Material');apply(material,'CardboardMaterialAPI')
    md=material.GetAttribute('cardboard:bendingMD').Get();cd=material.GetAttribute('cardboard:bendingCD').Get()
    D=cd+(md-cd)*md_weight
    stiffness=D/dual
    inds,weights,offsets=bind(render,p,f)
    s=Usd.Stage.CreateNew(str(file)) if not file.exists() else Usd.Stage.CreateInMemory()
    root=UsdGeom.Xform.Define(s,'/Box');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z')
    root.GetPrim().SetAssetInfoByKey('name','NVIDIA cardbox_a1 — Newton plastic shell')
    root.GetPrim().SetCustomDataByKey('source:simreadyVersion','0.9.1')
    root.GetPrim().SetCustomDataByKey('qualification','Research demo; not certified SimReady conformance or calibrated cardboard')
    mat=preview_material(s,'/Box/Materials/Cardboard',(.52,.31,.14),'./source/cardbox_a1/Textures/T_Cardbox_A1_Albedo.png')
    apply(mat.GetPrim(),'CardboardMaterialAPI');phys=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());phys.CreateStaticFrictionAttr(.65);phys.CreateDynamicFrictionAttr(.65);phys.CreateRestitutionAttr(0)
    # Author all fallback material fields explicitly for plugin-independent inspection.
    for name in mat.GetPrim().GetPropertyNames():
        if name.startswith('cardboard:'):
            a=mat.GetPrim().GetAttribute(name);v=a.Get()
            if v is not None:a.Set(v)
    mesh=UsdGeom.Mesh.Define(s,'/Box/SimMesh');mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(p));mesh.CreateFaceVertexCountsAttr([3]*len(f));mesh.CreateFaceVertexIndicesAttr(f.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreatePurposeAttr('guide')
    prim=mesh.GetPrim();apply(prim,'CardboardShellAPI');apply(prim,'CardboardPlasticStateAPI')
    for name,value in {'restPoints':Vt.Vec3fArray.FromNumpy(p),'hingeIndices':Vt.Vec4iArray.FromNumpy(edges.astype(np.int32)),'referenceAngles':ref.tolist(),'dualWidths':dual.tolist(),'edgeStiffness':stiffness.tolist(),'materialDirection':Vt.Vec3fArray.FromNumpy(directions),'sourceAsset':Sdf.AssetPath('./source/cardbox_a1/cardbox_a1.usd'),'sourceScale':.4,'schemaVersion':1,'solver':'newtonVBD15'}.items():set_attr(prim,'cardboard:'+name,value)
    prim.GetRelationship('cardboard:material').SetTargets([mat.GetPath()]);prim.GetRelationship('cardboard:renderMesh').SetTargets(['/Box/RenderMesh'])
    for name in ['plasticAngles','accumulatedAngles','plasticDissipation','damage']:set_attr(prim,'cardboard:'+name,[0.]*len(edges))
    set_attr(prim,'cardboard:velocities',Vt.Vec3fArray.FromNumpy(np.zeros_like(p)))
    visual=UsdGeom.Mesh.Define(s,'/Box/RenderMesh');visual.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(render));visual.CreateFaceVertexCountsAttr([3]*len(rfaces));visual.CreateFaceVertexIndicesAttr(rfaces.ravel().tolist());visual.CreateSubdivisionSchemeAttr('none');visual.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(visual).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(Vt.Vec2fArray.FromNumpy(uv));UsdShade.MaterialBindingAPI.Apply(visual.GetPrim()).Bind(mat)
    apply(visual.GetPrim(),'CardboardBindingAPI');visual.GetPrim().GetRelationship('cardboard:simulationMesh').SetTargets([prim.GetPath()])
    for name,value in [('bindingIndices',Vt.Vec3iArray.FromNumpy(inds)),('bindingWeights',Vt.Vec3fArray.FromNumpy(weights)),('bindingOffsets',Vt.Vec3fArray.FromNumpy(offsets))]:set_attr(visual.GetPrim(),'cardboard:'+name,value)
    s.GetRootLayer().Export(str(file))
    scene_path=out/f'demo_scene{variant}.usda'
    scene=Usd.Stage.Open(str(scene_path)) if scene_path.exists() else Usd.Stage.CreateNew(str(scene_path))
    scene.GetRootLayer().Clear()
    world=UsdGeom.Xform.Define(scene,'/World');scene.SetDefaultPrim(world.GetPrim());UsdGeom.SetStageMetersPerUnit(scene,1);UsdGeom.SetStageUpAxis(scene,'Z')
    physics=UsdPhysics.Scene.Define(scene,'/World/Physics');physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));physics.CreateGravityMagnitudeAttr(9.81)
    apply(physics.GetPrim(),'CardboardDemoAPI')
    apply(physics.GetPrim(),'CardboardScenarioAPI')
    for name in physics.GetPrim().GetPropertyNames():
        if name.startswith('cardboard:'):
            a=physics.GetPrim().GetAttribute(name)
            if a and a.Get() is not None:a.Set(a.Get())
    box=UsdGeom.Xform.Define(scene,'/World/Box');box.GetPrim().GetReferences().AddReference(f'./cardboard{variant}.usda');box.AddTranslateOp().Set(Gf.Vec3d(.32,0,.6516+size[2]/2))
    cube(scene,'/World/Table',(.12,0,.61),(1.8,1.0,.08),(.16,.20,.24))
    for i,(x,y) in enumerate([(-.62,-.40),(.86,-.40),(-.62,.40),(.86,.40)]):cube(scene,f'/World/Leg{i}',(x,y,.285),(.07,.07,.57),(.10,.12,.14))
    cube(scene,'/World/Floor',(0,0,-.03),(5,5,.05),(.08,.10,.12))
    robot=UsdGeom.Xform.Define(scene,'/World/UR10');robot.GetPrim().GetReferences().AddReference('./source/ur10/ur10.usd');robot.AddTranslateOp(opSuffix='placement').Set(Gf.Vec3d(-.46,0,.65))
    # Correct massless tool marker's meaningless source inertia in a composition override.
    ee=scene.GetPrimAtPath('/World/UR10/ee_link');mass=UsdPhysics.MassAPI.Apply(ee);mass.CreateMassAttr(.05);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.00005))
    tf=UsdGeom.XformCache().GetLocalToWorldTransform(ee);pos=tf.ExtractTranslation();rot=tf.ExtractRotationQuat()
    g=UsdGeom.Xform.Define(scene,'/World/Gripper')
    def body(name,local,size,mass):
        c=UsdGeom.Xform.Define(scene,'/World/Gripper/'+name);c.AddTranslateOp().Set(tf.Transform(Gf.Vec3d(*local)));c.AddOrientOp().Set(Gf.Quatf(rot))
        shape=UsdGeom.Cube.Define(scene,str(c.GetPath())+'/Shape');shape.CreateSizeAttr(1);shape.AddScaleOp().Set(Gf.Vec3f(*size));shape.CreateDisplayColorAttr([Gf.Vec3f(.15,.16,.18) if name=='Palm' else Gf.Vec3f(.12,.13,.15)])
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim());UsdPhysics.CollisionAPI.Apply(shape.GetPrim());m=UsdPhysics.MassAPI.Apply(c.GetPrim());m.CreateMassAttr(mass)
        # Explicit SI inertia avoids double-counting geometry scale in solvers.
        sx,sy,sz=size;m.CreateDiagonalInertiaAttr(Gf.Vec3f(mass*(sy*sy+sz*sz)/12,mass*(sx*sx+sz*sz)/12,mass*(sx*sx+sy*sy)/12))
        return c
    palm=body('Palm',(0,0,.115),(.40,.07,.055),1.2)
    fix=UsdPhysics.FixedJoint.Define(scene,'/World/Gripper/Mount');fix.CreateBody0Rel().SetTargets([ee.GetPath()]);fix.CreateBody1Rel().SetTargets([palm.GetPath()]);fix.CreateLocalPos0Attr(Gf.Vec3f(0,0,.115));fix.CreateLocalRot0Attr(Gf.Quatf(1));fix.CreateLocalRot1Attr(Gf.Quatf(1))
    spacer=UsdGeom.Cube.Define(scene,'/World/Gripper/Palm/Spacer');spacer.CreateSizeAttr(1);spacer.AddTranslateOp().Set(Gf.Vec3d(0,0,-.045));spacer.AddScaleOp().Set(Gf.Vec3f(.06,.06,.09));spacer.CreateDisplayColorAttr([Gf.Vec3f(.18,.20,.23)])
    joints=[]
    for name,sign in [('Left',-1),('Right',1)]:
        finger=body(name,(sign*.174,0,.315),(.028,.09,.11),.25)
        stem=UsdGeom.Cube.Define(scene,str(finger.GetPath())+'/Stem');stem.CreateSizeAttr(1);stem.AddTranslateOp().Set(Gf.Vec3d(sign*.022,0,-.11));stem.AddScaleOp().Set(Gf.Vec3f(.016,.045,.13));stem.CreateDisplayColorAttr([Gf.Vec3f(.22,.24,.26)]);UsdPhysics.CollisionAPI.Apply(stem.GetPrim())
        j=UsdPhysics.PrismaticJoint.Define(scene,'/World/Gripper/'+name+'Joint');j.CreateBody0Rel().SetTargets([palm.GetPath()]);j.CreateBody1Rel().SetTargets([finger.GetPath()]);j.CreateAxisAttr('X');j.CreateLocalPos0Attr(Gf.Vec3f(0,0,.20));j.CreateLocalRot0Attr(Gf.Quatf(1));j.CreateLocalRot1Attr(Gf.Quatf(1));j.CreateLowerLimitAttr(-.19 if sign<0 else .055);j.CreateUpperLimitAttr(-.055 if sign<0 else .19)
        drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0);drive.CreateDampingAttr(0)
        j.GetPrim().CreateAttribute('newton:linear:position',Sdf.ValueTypeNames.Float).Set(sign*.174);joints.append(j.GetPath())
    for name,targets in [('box',[box.GetPath()]),('robot',[robot.GetPath()]),('fingerJoints',joints)]:physics.GetPrim().GetRelationship('cardboard:'+name).SetTargets(targets)
    scene.GetRootLayer().Export(str(scene_path))
    (out/f'build_report{variant}.json').write_text(json.dumps({'source':'NVIDIA SimReady cardbox_a1','source_scale':.4,'size_m':size.tolist(),'simulation_vertices':len(p),'triangles':len(f),'hinges':len(edges),'render_vertices':len(render),'max_binding_offset_m':float(np.linalg.norm(offsets,axis=1).max()),'schemas':6,'panel_subdivisions':n,'render_max_edge_m':render_max_edge,'render_triangles':len(rfaces),'closed_seams':True,'full_orthotropic_membrane':False},indent=2))
    print((out/f'build_report{variant}.json').read_text())
if __name__=='__main__':build()
