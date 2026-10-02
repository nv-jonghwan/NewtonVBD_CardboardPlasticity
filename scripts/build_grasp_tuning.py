"""Build isolated, fixed-material candidates for sag/grasp/crush tuning."""
import argparse
import json
from pxr import Usd
from cardboard import ROOT
from cardboard.usd_utils import register

p = argparse.ArgumentParser(__doc__)
p.add_argument('--name', required=True)
p.add_argument('--bend', type=float, default=16.)
p.add_argument('--yield-curvature', type=float, default=6.)
p.add_argument('--damage-rate', type=float, default=4.)
p.add_argument('--residual', type=float, default=.18)
p.add_argument('--hardening', type=float, default=.01)
p.add_argument('--grasp-force', type=float, default=250.)
p.add_argument('--grasp-gap', type=float, default=.27)
p.add_argument('--lift-force', type=float, default=0.)
p.add_argument('--crush-force', type=float, default=4000.)
p.add_argument('--contact-capacity', type=int, default=256)
p.add_argument('--grasp-ramp', type=float)
p.add_argument('--joint-stiffness', type=float, default=100000.)
a=p.parse_args(); register()
source=ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'
out=ROOT/'outputs/grasp_tuning'/a.name;out.mkdir(parents=True,exist_ok=False)
s=Usd.Stage.Open(str(source));s.Flatten().Export(str(out/'scene.usda'));s=Usd.Stage.Open(str(out/'scene.usda'))
mesh=s.GetPrimAtPath('/World/Box/SimMesh')
mat=s.GetPrimAtPath(mesh.GetRelationship('cardboard:material').GetTargets()[0])
settings=s.GetPrimAtPath('/World/Physics')
for key in ['bendingMD','bendingCD']:
    attr=mat.GetAttribute('cardboard:'+key);attr.Set(attr.Get()*a.bend)
for key,value in [('yieldCurvature',a.yield_curvature),('damageRate',a.damage_rate),
                  ('residualStiffness',a.residual),('hardeningRatio',a.hardening)]:
    mat.GetAttribute('cardboard:'+key).Set(value)
for key,value in [('graspForce',a.grasp_force),('graspGap',a.grasp_gap),
                  ('liftForce',a.lift_force),('crushForce',a.crush_force)]:
    settings.GetAttribute('cardboard:'+key).Set(value)
settings.GetAttribute('cardboard:rigidParticleContactCapacity').Set(a.contact_capacity)
settings.GetAttribute('cardboard:rigidJointLinearStiffness').Set(a.joint_stiffness)
if a.grasp_ramp is not None:settings.GetAttribute('cardboard:graspRampFraction').Set(a.grasp_ramp)
s.GetRootLayer().Save()
(out/'candidate.json').write_text(json.dumps(vars(a),indent=2))
print(out/'scene.usda')
