"""Check that the tuned overlay changes only the declared material/control fields."""
import hashlib
import json
import numpy as np
from pxr import Sdf, Usd
from cardboard import ROOT
from cardboard.usd_utils import register

register()
path=ROOT/'assets/demo_scene_robotiq_board10_grasp_tuned.usda'
base=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'))
tuned=Usd.Stage.Open(str(path))
trial=Usd.Stage.Open(str(ROOT/'outputs/grasp_tuning/balanced/scene.usda'))
material='/World/Box/Materials/Cardboard'
allowed={(material,'cardboard:'+k) for k in ['bendingMD','bendingCD','yieldCurvature',
    'hardeningRatio','damageRate','residualStiffness']}
allowed|={('/World/Physics','cardboard:'+k) for k in ['graspForce','liftForce','crushForce',
    'graspRampFraction','rigidParticleContactCapacity','rigidJointLinearStiffness']}
def equal(a,b):
    if isinstance(a,Sdf.AssetPath) and isinstance(b,Sdf.AssetPath):
        return (a.resolvedPath or a.path)==(b.resolvedPath or b.path)
    try:return bool(np.array_equal(np.asarray(a),np.asarray(b)))
    except Exception:return a==b

unexpected=[];trial_differences=[];changed=[]
for prim in tuned.Traverse():
    p=str(prim.GetPath());before=base.GetPrimAtPath(p);measured=trial.GetPrimAtPath(p)
    for attr in prim.GetAttributes():
        key=(p,attr.GetName());value=attr.Get()
        if not equal(value,before.GetAttribute(key[1]).Get()):
            changed.append(key)
            if key not in allowed:unexpected.append(key)
        if not equal(value,measured.GetAttribute(key[1]).Get()):trial_differences.append(key)
    for rel in prim.GetRelationships():
        if rel.GetTargets()!=before.GetRelationship(rel.GetName()).GetTargets():unexpected.append((p,rel.GetName()))
checks=dict(only_declared_overrides=not unexpected,all_expected_overrides_present=set(changed)==allowed,
            matches_tested_trial=not trial_differences,
            same_prim_paths=[str(p.GetPath()) for p in base.Traverse()]==[str(p.GetPath()) for p in tuned.Traverse()],
            original_scene_unchanged=hashlib.sha256((ROOT/'assets/demo_scene_robotiq_board10_primitive.usda').read_bytes()).hexdigest()=='cb9f8a94da3b7200fd5330b0aafc7c0fa810e84d44877421c814fbf1a3dc0684')
report=dict(passed=all(checks.values()),checks=checks,unexpected=unexpected,trial_differences=trial_differences,
            changed_attributes=changed,overlay_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
(ROOT/'outputs/grasp_tuning/asset_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
