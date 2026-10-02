"""Independent dimensional and geometry checks for the requested 15 -> 10 mm gauge."""
import json
import numpy as np
from pxr import Usd,UsdGeom
from cardboard import ROOT
from cardboard.usd_utils import register
register()
sources=[Usd.Stage.Open(str(ROOT/'assets'/name)) for name in ['demo_scene_robotiq_creased.usda','demo_scene_robotiq_board10.usda']]
data=[]
for stage in sources:
    mesh=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/SimMesh'));prim=mesh.GetPrim();render=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/RenderMesh'));rp=render.GetPrim();mat=stage.GetPrimAtPath('/World/Box/Materials/Cardboard')
    names=['thickness','arealDensity','membraneShear','membraneArea','bendingCD','bendingMD','bendingReferenceThickness','bendingThicknessExponent','yieldCurvature','creaseDamageLength']
    material={n:mat.GetAttribute('cardboard:'+n).Get() for n in names}
    arr=lambda p,n:np.asarray(p.GetAttribute('cardboard:'+n).Get())
    q=arr(prim,'restPoints').astype(float);f=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3);tri=q[f]
    area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1).sum()*.5
    data.append(dict(material=material,q=q,f=f,edges=arr(prim,'hingeIndices'),dual=arr(prim,'dualWidths'),ke=arr(prim,'edgeStiffness'),render=np.asarray(render.GetPointsAttr().Get()),render_faces=np.asarray(render.GetFaceVertexIndicesAttr().Get()),uv=np.asarray(UsdGeom.PrimvarsAPI(render).GetPrimvar('st').ComputeFlattened()),indices=arr(rp,'bindingIndices'),weights=arr(rp,'bindingWeights'),offsets=arr(rp,'bindingOffsets'),area=float(area),mass=float(area*material['arealDensity'])))
a,b=data;ma,mb=a['material'],b['material'];ratio=2/3
close=lambda x,y:bool(np.allclose(x,y,rtol=1e-6,atol=1e-8))
checks={'thickness_10mm':close(mb['thickness'],.01),'membrane_and_areal_mass_linear':all(close(mb[n],ma[n]*ratio) for n in ['arealDensity','membraneShear','membraneArea']),
        'bending_cubic':all(close(mb[n]*(mb['thickness']/mb['bendingReferenceThickness'])**mb['bendingThicknessExponent'],ma[n]*(ma['thickness']/ma['bendingReferenceThickness'])**ma['bendingThicknessExponent']*ratio**3) for n in ['bendingCD','bendingMD']),
        'constant_outer_fiber_yield_strain':close(mb['yieldCurvature']*mb['thickness'],ma['yieldCurvature']*ma['thickness']),
        'constant_damage_per_outer_fiber_plastic_strain':close(mb['creaseDamageLength']/mb['thickness'],ma['creaseDamageLength']/ma['thickness']),
        'same_physical_topology':bool(a['q'].shape==b['q'].shape and np.array_equal(a['f'],b['f']) and np.array_equal(a['edges'],b['edges'])),
        'same_render_topology_uv_and_source_geometry':all(np.array_equal(a[k],b[k]) for k in ['render','render_faces','uv']),
        'same_collision_outer_envelope':close(np.ptp(a['q'],axis=0)+ma['thickness'],np.ptp(b['q'],axis=0)+mb['thickness']),
        'binding_reconstructs_source':close((b['q'][b['indices']]*b['weights'][:,:,None]).sum(1)+b['offsets'],b['render'])}
e=b['q'][b['edges'][:,3]]-b['q'][b['edges'][:,2]];length=np.linalg.norm(e,axis=1);w=(e[:,1]/length)**2
dual=sum(np.linalg.norm(np.cross(b['q'][b['edges'][:,i]]-b['q'][b['edges'][:,2]],e),axis=1) for i in [0,1])/(2*length)
expected=(mb['bendingCD']+(mb['bendingMD']-mb['bendingCD'])*w)*(mb['thickness']/mb['bendingReferenceThickness'])**mb['bendingThicknessExponent']/dual
checks['dual_geometry_and_hinge_stiffness_rebuilt']=close(b['dual'],dual) and close(b['ke'],expected)
report={'passed':all(checks.values()),'checks':checks,'thickness_mm':[ma['thickness']*1000,mb['thickness']*1000],'physical_vertices':len(b['q']),'physical_triangles':len(b['f']),'render_vertices':len(b['render']),'render_triangles':len(b['render_faces'])//3,'area_m2':[a['area'],b['area']],'mass_kg':[a['mass'],b['mass']],'effective_bending_ratio':ratio**3,'note':'Exterior unchanged; thinner gauge moves the midsurface outwards, so total mass also includes the slightly larger midsurface area.'}
(ROOT/'outputs/stability_speed/board10_asset_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
