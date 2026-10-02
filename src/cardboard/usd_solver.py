"""Typed USD contract for the project's Newton deformable solver configuration."""
import os
import math
from pathlib import Path
from pxr import Usd, UsdGeom, Sdf, Gf, Vt
from .usd_utils import register, apply

ROM_FIELDS = {
    'basis': ('romBasis', Sdf.ValueTypeNames.Asset),
    'tolerance': ('romTolerance', Sdf.ValueTypeNames.Double),
    'full_every': ('romFullEvery', Sdf.ValueTypeNames.Int),
    'start_with_rom': ('romStartWithROM', Sdf.ValueTypeNames.Bool),
    'check_residual': ('romCheckResidual', Sdf.ValueTypeNames.Bool),
    'defer_fallback': ('romDeferFallback', Sdf.ValueTypeNames.Bool),
    'element_fraction': ('romElementFraction', Sdf.ValueTypeNames.Double),
    'local_vbd': ('romLocalVBD', Sdf.ValueTypeNames.Bool),
    'patch_rings': ('romPatchRings', Sdf.ValueTypeNames.Int),
    'patch_curvature': ('romPatchCurvature', Sdf.ValueTypeNames.Double),
    'patch_full_every': ('romPatchFullEvery', Sdf.ValueTypeNames.Int),
    'patch_solver': ('romPatchSolver', Sdf.ValueTypeNames.Token),
    'patch_relaxation': ('romPatchRelaxation', Sdf.ValueTypeNames.Double),
    'element_full_after_yield': ('romElementFullAfterYield', Sdf.ValueTypeNames.Bool),
}
SMALL_FIELDS = {'scale':'scale','knee':'knee','end':'end','memory_curvature':'memoryCurvature',
                'crease_friction_curvature':'creaseFrictionCurvature'}


def read_solver_config(scene, mesh_path='/World/Box/SimMesh'):
    register()
    stage=scene if isinstance(scene,Usd.Stage) else Usd.Stage.Open(str(scene))
    if not stage:raise ValueError('Cannot open solver USD: '+str(scene))
    mesh=stage.GetPrimAtPath(mesh_path)
    if not mesh or not mesh.HasAPI('CardboardSolverAPI'):return None
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        raise ValueError('Newton cardboard scenes require metersPerUnit = 1 and upAxis = Z on the root layer')
    def required(prim,name):
        value=prim.GetAttribute(name).Get()
        if value is None:raise ValueError('Missing USD solver property: '+name)
        return value
    prefix='cardboard:solver:'
    if required(mesh,prefix+'implementation')!='adaptiveROMVBD':raise ValueError('Unsupported USD solver implementation')
    targets = mesh.GetRelationship('cardboard:material').GetTargets()
    if len(targets) != 1 or not stage.GetPrimAtPath(targets[0]):
        raise ValueError('USD solver requires one valid cardboard:material relationship')
    material = stage.GetPrimAtPath(targets[0])
    rom={key:required(mesh,prefix+name) for key,(name,_) in ROM_FIELDS.items()}
    basis=rom['basis']
    if not basis.resolvedPath:raise ValueError('Unresolved USD ROM basis: '+basis.path)
    rom['basis']=basis.resolvedPath
    config = {'iterations': required(mesh, prefix+'iterations'),
              'schedule': required(mesh, prefix+'schedule'), 'rom': rom,
              'small_bend': {key: required(material, 'cardboard:smallBend:'+name)
                             for key, name in SMALL_FIELDS.items()}}
    if config['iterations'] < 1 or config['schedule'] not in ('baseline', 'guarded'):
        raise ValueError('Invalid USD solver iteration count or schedule')
    bend = config['small_bend']
    if not (all(math.isfinite(v) for v in bend.values()) and bend['scale'] > 1
            and 0 < bend['scale'] * bend['knee'] < bend['end']
            < required(material, 'cardboard:yieldCurvature') and bend['memory_curvature'] >= 0
            and bend['crease_friction_curvature'] >= 0):
        raise ValueError('Invalid USD small-bend material parameters')
    for key in ('full_every', 'patch_full_every', 'patch_rings'):
        if rom[key] < 1:
            raise ValueError('Invalid USD ROM parameter: '+key)
    for key in ('tolerance', 'element_fraction', 'patch_curvature', 'patch_relaxation'):
        if not math.isfinite(rom[key]) or rom[key] <= 0:
            raise ValueError('Invalid USD ROM parameter: '+key)
    if rom['tolerance'] > 1 or rom['element_fraction'] > 1 or rom['patch_relaxation'] > 1 or rom['patch_solver'] not in ('jacobi', 'colored'):
        raise ValueError('Invalid USD ROM patch parameters')
    return config


def author_solver_config(stage, config, mesh_path='/World/Box/SimMesh'):
    register()
    mesh=stage.GetPrimAtPath(mesh_path);apply(mesh,'CardboardSolverAPI')
    material=stage.GetPrimAtPath(mesh.GetRelationship('cardboard:material').GetTargets()[0])
    for name,value in [('implementation','adaptiveROMVBD'),('iterations',config['iterations']),('schedule',config['schedule'])]:
        mesh.GetAttribute('cardboard:solver:'+name).Set(value)
    for key,(name,_) in ROM_FIELDS.items():
        value=config['rom'][key]
        if key=='basis':
            value=Sdf.AssetPath(os.path.relpath(Path(value).resolve(),Path(stage.GetEditTarget().GetLayer().realPath).parent))
        mesh.GetAttribute('cardboard:solver:'+name).Set(value)
    for key,name in SMALL_FIELDS.items():material.GetAttribute('cardboard:smallBend:'+name).Set(config['small_bend'][key])


def write_runtime_layer(sim, path, config, *, state=False):
    """Persist effective inputs/final geometry and material history, not a restart checkpoint."""
    import numpy as np
    from .surface import PanelSurface
    from .geometry import skin
    path=Path(path).resolve();path.parent.mkdir(parents=True,exist_ok=True)
    # Use absolute composition while authoring, then anchor the saved layer relatively.
    layer=Sdf.Layer.CreateAnonymous();layer.subLayerPaths=[sim.stage.GetRootLayer().realPath]
    stage=Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(sim.stage))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(sim.stage))
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    if config.get('rom') and config.get('small_bend'):
        # Author with an absolute basis while anonymous; re-anchor at export below.
        mesh=stage.GetPrimAtPath('/World/Box/SimMesh');apply(mesh,'CardboardSolverAPI')
        for name,value in [('implementation','adaptiveROMVBD'),('iterations',sim.iterations),('schedule',getattr(sim.solver,'contact_schedule','baseline'))]:mesh.GetAttribute('cardboard:solver:'+name).Set(value)
        for key,(name,_) in ROM_FIELDS.items():mesh.GetAttribute('cardboard:solver:'+name).Set(Sdf.AssetPath(os.path.relpath(config['rom'][key],path.parent)) if key=='basis' else config['rom'][key])
        mat=stage.GetPrimAtPath('/World/Box/Materials/Cardboard')
        for key,name in SMALL_FIELDS.items():mat.GetAttribute('cardboard:smallBend:'+name).Set(config['small_bend'][key])
    stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:iterations').Set(sim.iterations)
    for attr in sim.mat.GetAttributes():
        if attr.GetName().startswith('cardboard:') and not attr.GetName().startswith('cardboard:smallBend:') and attr.Get() is not None:
            stage.GetPrimAtPath(str(sim.mat.GetPath())).GetAttribute(attr.GetName()).Set(attr.Get())
    if state:
        mesh=stage.GetPrimAtPath('/World/Box/SimMesh');q=sim.a.particle_q.numpy();local=q-sim.center
        UsdGeom.Mesh(mesh).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local.astype(np.float32)))
        for name,array in [('plasticAngles',sim.a.cardboard.plastic_angle),('accumulatedAngles',sim.a.cardboard.accumulated_angle),('plasticDissipation',sim.a.cardboard.plastic_work),('damage',sim.a.cardboard.damage),('velocities',sim.a.particle_qd)]:
            mesh.GetAttribute('cardboard:'+name).Set(Vt.Vec3fArray.FromNumpy(array.numpy()) if name=='velocities' else array.numpy().tolist())
        if hasattr(sim,'crease_friction_work'):mesh.GetAttribute('cardboard:creaseFrictionWork').Set(sim.crease_friction_work.numpy().tolist())
        render=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/RenderMesh'));attr=lambda n:render.GetPrim().GetAttribute('cardboard:'+n).Get()
        inds=np.asarray(attr('bindingIndices'));weights=np.asarray(attr('bindingWeights'));offsets=np.asarray(attr('bindingOffsets'))
        if attr('surfaceInterpolation')=='creaseAwareCubic':
            surface=PanelSurface(sim.local,sim.faces,inds,weights,offsets,attr('visualCreaseAngleDegrees'),attr('visualCreaseTransitionDegrees'),bool(attr('visualSmoothThickness')))
            points,normals=surface.evaluate(local);render.SetNormalsInterpolation('vertex');render.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
        else:points=skin(local,inds,weights,offsets).astype(np.float32)
        render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points));render.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([points.min(0),points.max(0)])))
        cache=UsdGeom.XformCache()
        for label,pose in zip(sim.model.body_label,sim.a.body_q.numpy()):
            prim=stage.GetPrimAtPath(label);parent=cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse();x=UsdGeom.Xformable(prim);x.ClearXformOpOrder()
            matrix=Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]),Gf.Vec3d(*pose[3:6].tolist())));matrix.SetTranslateOnly(Gf.Vec3d(*pose[:3].tolist()));x.AddTransformOp(opSuffix='newtonState').Set(matrix*parent)
    layer.customLayerData={'simulationTime':float(sim.time),'runtime':'Newton '+sim.newton_version,'scope':'visual/material snapshot; not full solver restart'}
    # Re-anchor a detached copy so the live anonymous stage never recomposes
    # relative paths without a filesystem anchor.
    saved = Sdf.Layer.CreateAnonymous()
    saved.TransferContent(layer)
    saved.subLayerPaths = [os.path.relpath(sim.stage.GetRootLayer().realPath, path.parent)]
    saved.Export(str(path))
