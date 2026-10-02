"""Validate composed geometry and Newton's actual collision/mass import."""
import json
import numpy as np
import newton
from pxr import Usd, UsdGeom, UsdPhysics
from cardboard import ROOT

source = Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_board10_support20_trial.usda'))
stage = Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'))
checks = {}

def equal(a, b):
    try:
        return bool(np.array_equal(np.asarray(a), np.asarray(b)))
    except Exception:
        return a == b

preserved = True
for prim in source.Traverse():
    other = stage.GetPrimAtPath(prim.GetPath())
    for attr in prim.GetAttributes():
        if attr.GetName() == 'physics:collisionEnabled' and (str(prim.GetPath()).startswith('/World/Gripper') or prim.GetName() in ['Leg0','Leg1','Leg2','Leg3']):
            continue
        if str(prim.GetPath()) == '/World/Physics' and attr.GetName() == 'cardboard:liftHeight':
            continue
        preserved &= equal(attr.Get(), other.GetAttribute(attr.GetName()).Get())
    for rel in prim.GetRelationships():
        preserved &= rel.GetTargets() == other.GetRelationship(rel.GetName()).GetTargets()
checks['original_attributes_and_relationships_preserved_except_requested_changes'] = bool(preserved)
checks['lift_target_20cm'] = abs(stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:liftHeight').Get()-.2)<1e-7
builders = []
for s in (source, stage):
    b = newton.ModelBuilder()
    b.add_usd(s, ignore_paths=['/World/Box'], enable_self_collisions=False,
              load_visual_shapes=False, hide_collision_shapes=True)
    builders.append(b)
a, b = builders
for name in ['body_label', 'body_mass', 'body_inertia', 'body_com', 'body_q',
             'joint_label', 'joint_type', 'joint_q', 'joint_target_ke', 'joint_target_kd']:
    checks['unchanged_'+name] = equal(getattr(a, name), getattr(b, name))
mask = int(newton.ShapeFlags.COLLIDE_SHAPES | newton.ShapeFlags.COLLIDE_PARTICLES)
indices = [i for i, label in enumerate(b.shape_label) if label.startswith('/World/Gripper/') and b.shape_flags[i]&mask]
checks['eight_active_gripper_boxes'] = len(indices)==8 and all(b.shape_type[i]==newton.GeoType.BOX for i in indices)
checks['both_particle_and_rigid_contacts_enabled'] = all(b.shape_flags[i]&mask==mask for i in indices)
checks['no_palm_or_proximal_knuckle_contact'] = all('/Palm/' not in b.shape_label[i] and '_outer_knuckle/' not in b.shape_label[i] for i in indices)
checks['no_disabled_gripper_shapes_imported'] = sum(label.startswith('/World/Gripper/') for label in b.shape_label)==8
checks['tabletop_box_only_no_legs_imported'] = '/World/Table' in b.shape_label and b.shape_type[b.shape_label.index('/World/Table')]==newton.GeoType.BOX and not any('/World/Leg' in label for label in b.shape_label)
checks['floor_collision_preserved'] = b.shape_flags[b.shape_label.index('/World/Floor')] == a.shape_flags[a.shape_label.index('/World/Floor')]
cache = UsdGeom.XformCache()
max_bound_error = 0.
pad_face_errors = []
for i in indices:
    cube = stage.GetPrimAtPath(b.shape_label[i])
    mesh = stage.GetPrimAtPath(cube.GetAttribute('cardboard:sourceCollider').Get())
    pts = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
    transform = np.asarray(cache.GetLocalToWorldTransform(mesh)*cache.GetLocalToWorldTransform(cube).GetInverse())
    local = np.c_[pts, np.ones(len(pts))]@transform
    max_bound_error = max(max_bound_error, float(np.max(np.abs(local[:, :3])-.5)))
    if mesh.GetName() == 'Fingertip':
        # Box bounds touch both original closing-axis support planes.
        pad_face_errors.append(float(max(abs(local[:, 0].min()+.5),abs(local[:, 0].max()-.5))))
checks['source_mesh_enclosed_by_primitive'] = max_bound_error<1e-6
checks['pad_closing_planes_preserved'] = len(pad_face_errors)==2 and max(pad_face_errors)<1e-6
checks['primitive_friction_matches_source'] = all(
    b.shape_material_mu[i] == a.shape_material_mu[a.shape_label.index(stage.GetPrimAtPath(b.shape_label[i]).GetAttribute('cardboard:sourceCollider').Get())]
    for i in indices)
report = dict(passed=all(checks.values()), checks=checks,
              active_gripper_colliders_before=sum(label.startswith('/World/Gripper/') and bool(a.shape_flags[i]&mask) for i,label in enumerate(a.shape_label)),
              active_gripper_colliders_after=len(indices),
              max_normalized_bound_error=max_bound_error,
              imported_shapes=[dict(path=b.shape_label[i], type=int(b.shape_type[i]),flags=int(b.shape_flags[i])) for i in indices])
(ROOT/'outputs/primitive_gripper/asset_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
