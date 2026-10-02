"""Audit the delivered USD-to-Newton asset contract, optionally against a completed run."""
import argparse
import collections
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom
from cardboard import ROOT
from cardboard.usd_utils import register
from cardboard.usd_solver import read_solver_config


def audit(scene, asset, run=None):
    register();checks={};stage=Usd.Stage.Open(str(scene));box=Usd.Stage.Open(str(asset))
    checks['usd_composition']=bool(stage and box and not stage.GetCompositionErrors() and not box.GetCompositionErrors())
    checks['meters_z_up']=all(UsdGeom.GetStageMetersPerUnit(s)==1 and UsdGeom.GetStageUpAxis(s)=='Z' for s in (stage,box))
    checks['standalone_default_prim']=box.GetDefaultPrim().GetPath()== '/Box'
    config=read_solver_config(stage);asset_config=read_solver_config(box,'/Box/SimMesh')
    checks['asset_and_scene_solver_match']=config==asset_config
    paths=['/World/Physics','/World/Box/SimMesh','/World/Box/RenderMesh','/World/Box/Materials/Cardboard']
    missing=[]
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        definitions=[Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(name) for name in prim.GetAppliedSchemas()]
        known=set().union(*(set(d.GetPropertyNames()) for d in definitions if d))
        missing.extend(str(a.GetPath()) for a in prim.GetAttributes() if a.GetName().startswith('cardboard:') and a.HasAuthoredValueOpinion() and a.GetName() not in known)
    checks['authored_properties_registered']=not missing
    mesh=stage.GetPrimAtPath('/World/Box/SimMesh');render=stage.GetPrimAtPath('/World/Box/RenderMesh')
    material=stage.GetPrimAtPath(mesh.GetRelationship('cardboard:material').GetTargets()[0])
    rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get());faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    edges=collections.Counter(tuple(sorted((int(a),int(b)))) for tri in faces for a,b in zip(tri,np.roll(tri,-1)))
    checks['closed_shell']=set(edges.values())=={2}
    checks['nondegenerate_mesh']=bool(np.all(np.linalg.norm(np.cross(rest[faces[:,1]]-rest[faces[:,0]],rest[faces[:,2]]-rest[faces[:,0]]),axis=1)>1e-8))
    inds=np.asarray(render.GetAttribute('cardboard:bindingIndices').Get());weights=np.asarray(render.GetAttribute('cardboard:bindingWeights').Get())
    checks['display_binding']=bool(inds.min()>=0 and inds.max()<len(rest) and np.allclose(weights.sum(1),1) and weights.min()>=0)
    checks['binding_relationships']=mesh.GetRelationship('cardboard:renderMesh').GetTargets()==[render.GetPath()] and render.GetRelationship('cardboard:simulationMesh').GetTargets()==[mesh.GetPath()]
    from cardboard.rom_basis import mesh_digest
    with np.load(config['rom']['basis']) as basis:
        checks['rom_geometry_digest']=str(basis['mesh_digest'])==mesh_digest(rest,faces)
    checks['newton_solver_target']=mesh.GetAttribute('cardboard:solver').Get()=='newtonVBD16'
    checks['positive_physical_material']=all(material.GetAttribute('cardboard:'+name).Get()>0 for name in ['thickness','arealDensity','membraneShear','membraneArea','bendingMD','bendingCD'])
    if run:
        run=Path(run);actual=json.loads((run/'run_config.json').read_text());effective=Usd.Stage.Open(str(run/'effective_scene.usda'));final=Usd.Stage.Open(str(run/'final_state.usda'))
        checks['output_usd_composition']=not effective.GetCompositionErrors() and not final.GetCompositionErrors()
        checks['output_usd_units_and_axis']=all(UsdGeom.GetStageMetersPerUnit(s)==1 and UsdGeom.GetStageUpAxis(s)=='Z' for s in (effective,final))
        effective_config=read_solver_config(effective)
        checks['runtime_iterations_from_usd']=actual['iterations']==effective_config['iterations']==config['iterations']
        checks['runtime_material_from_usd']=actual['small_bend']==effective_config['small_bend']==config['small_bend']
        checks['runtime_rom_from_usd']=all(actual['rom'][k]==v for k,v in config['rom'].items())
        checks['runtime_newton_version']=actual['newton']=='1.6.0'
        with np.load(run/'final_material_state.npz') as state:
            prim=final.GetPrimAtPath('/World/Box/SimMesh');center=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation())
            checks['final_usd_geometry_matches_gpu']=bool(np.allclose(np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())+center,state['points'],atol=1e-7))
            for attribute,key in [('plasticAngles','plastic_angle'),('accumulatedAngles','accumulated_angle'),('plasticDissipation','plastic_work'),('damage','damage'),('velocities','velocity'),('creaseFrictionWork','crease_friction_work')]:
                checks['final_usd_'+key]=bool(np.allclose(np.asarray(prim.GetAttribute('cardboard:'+attribute).Get()),state[key],rtol=1e-6,atol=1e-8))
            checks['finite_final_state']=bool(all(np.isfinite(state[key]).all() for key in state.files))
        if (run/'trajectory.npz').exists():
            with np.load(run/'trajectory.npz') as traj:checks['recorded_full_duration']=float(traj['t'][-1])>=19.99
    return {'passed':all(checks.values()),'scope':'Project Newton deformable USD contract; not formal SimReady certification',
            'checks':checks,'unregistered_properties':missing,'vertices':len(rest),'triangles':len(faces),'hinges':len(edges)}


def main():
    p=argparse.ArgumentParser(__doc__);p.add_argument('--scene',default='assets/demo_scene_robotiq_board4_vertical.usda');p.add_argument('--asset',default='assets/cardboard_simready.usda');p.add_argument('--run-directory');p.add_argument('--report');a=p.parse_args()
    report=audit(ROOT/a.scene,ROOT/a.asset,ROOT/a.run_directory if a.run_directory else None)
    if a.report:
        out=ROOT/a.report;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
