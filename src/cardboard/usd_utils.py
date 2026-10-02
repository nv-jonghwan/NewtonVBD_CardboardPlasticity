from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf, Vt, Plug
from . import ROOT

def register():
    Plug.Registry().RegisterPlugins(str(ROOT/'schemas/cardboard/resources'))

def apply(prim, name):
    if not prim.ApplyAPI(name):
        raise RuntimeError(f'Cannot apply registered schema {name} to {prim.GetPath()}')

def set_attr(prim, name, value):
    a=prim.GetAttribute(name)
    if not a: raise KeyError(f'Schema attribute missing: {name}')
    a.Set(value)

def preview_material(stage, path, color, texture=None):
    mat=UsdShade.Material.Define(stage,path)
    sh=UsdShade.Shader.Define(stage,path+'/Preview');sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
    if texture:
        tex=UsdShade.Shader.Define(stage,path+'/Albedo');tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
        st=UsdShade.Shader.Define(stage,path+'/ST');st.CreateIdAttr('UsdPrimvarReader_float2')
        st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result')
        sh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
    return mat

def cube(stage,path,pos,size,color,rigid=False,mass=1):
    c=UsdGeom.Cube.Define(stage,path);c.CreateSizeAttr(1)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim());UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
    return c
