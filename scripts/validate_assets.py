"""Validate this project's explicit schema/runtime contract (not NVIDIA certification)."""
import argparse,collections,hashlib,json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdUtils
from cardboard import ROOT
from cardboard.usd_utils import register
register();checks={}
ap=argparse.ArgumentParser();ap.add_argument('--asset',default='assets/cardboard.usda');ap.add_argument('--scene',default='assets/demo_scene.usda');ap.add_argument('--report',default='outputs/asset_validation.json');args=ap.parse_args()
s=Usd.Stage.Open(str(ROOT/args.asset));p=s.GetPrimAtPath('/Box/SimMesh');r=s.GetPrimAtPath('/Box/RenderMesh')
checks['meters_z_up']=UsdGeom.GetStageMetersPerUnit(s)==1 and UsdGeom.GetStageUpAxis(s)=='Z'
checks['registered_shell_api']='CardboardShellAPI' in p.GetAppliedSchemas() and bool(Usd.SchemaRegistry().FindAppliedAPIPrimDefinition('CardboardShellAPI'))
f=np.array(UsdGeom.Mesh(p).GetFaceVertexIndicesAttr().Get()).reshape(-1,3);v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
edges=collections.Counter(tuple(sorted(e)) for t in f for e in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])]);checks['closed_two_manifold']=set(edges.values())=={2}
checks['nondegenerate_triangles']=bool(np.all(np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]),axis=1)>1e-7))
inds=np.array(r.GetAttribute('cardboard:bindingIndices').Get());w=np.array(r.GetAttribute('cardboard:bindingWeights').Get());o=np.array(r.GetAttribute('cardboard:bindingOffsets').Get());render=np.array(UsdGeom.Mesh(r).GetPointsAttr().Get())
checks['binding_valid']=bool(inds.min()>=0 and inds.max()<len(v) and np.allclose(w.sum(1),1) and np.min(w)>=0)
checks['source_graphics_bind_exact']=bool(np.max(abs((v[inds]*w[:,:,None]).sum(1)+o-render))<1e-6)
scene=Usd.Stage.Open(str(ROOT/args.scene))
checks['all_six_custom_apis_registered']=all(Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(x) for x in ['CardboardMaterialAPI','CardboardShellAPI','CardboardPlasticStateAPI','CardboardBindingAPI','CardboardDemoAPI','CardboardScenarioAPI'])
checks['gripper_physical_joints']=all(scene.GetPrimAtPath('/World/Gripper/'+n).IsA(t) for n,t in [('Mount',UsdPhysics.FixedJoint),('LeftJoint',UsdPhysics.PrismaticJoint),('RightJoint',UsdPhysics.PrismaticJoint)])
checks['unscaled_gripper_bodies']=all(np.allclose(UsdGeom.Xformable(scene.GetPrimAtPath('/World/Gripper/'+n)).GetLocalTransformation().ExtractRotationMatrix().GetDeterminant(),1,atol=1e-5) for n in ['Palm','Left','Right'])
checks['source_hashes']=all(hashlib.sha256((ROOT/x['path']).read_bytes()).hexdigest()==x['sha256'] for x in json.loads((ROOT/'assets/source/manifest.json').read_text())['files'])
report={'checks':checks,'passed':all(checks.values()),'scope':'Project schema/geometry/provenance contract only; not full NVIDIA SimReady certification'}
(ROOT/args.report).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
