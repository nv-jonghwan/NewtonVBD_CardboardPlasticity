"""Isolated fine-mesh variant with stiff intact panels and local crease damage."""
import json
from pxr import Usd, Sdf
from cardboard import ROOT
from cardboard.usd_utils import register

register()
source=ROOT/'assets/demo_scene_robotiq_fine24.usda'
target=ROOT/'assets/demo_scene_robotiq_creased.usda'
stage=Usd.Stage.Open(str(source));stage.GetRootLayer().Export(str(target))
stage=Usd.Stage.Open(str(target))
material=stage.GetPrimAtPath('/World/Box/Materials/Cardboard')
changes=dict(yieldCurvature=4.,hardeningRatio=.01,damageRate=4.,residualStiffness=.18,creaseDamageLength=.02,
             bendingRelaxationTime=.02)
for name,value in changes.items():
    attr=material.GetAttribute('cardboard:'+name)
    if not attr:attr=material.CreateAttribute('cardboard:'+name,Sdf.ValueTypeNames.Float,custom=True)
    attr.Set(value)
render=stage.GetPrimAtPath('/World/Box/RenderMesh')
render.GetAttribute('cardboard:visualCreaseAngleDegrees').Set(80.)
render.GetAttribute('cardboard:visualCreaseTransitionDegrees').Set(40.)
settings=stage.GetPrimAtPath('/World/Physics')
settings.GetAttribute('cardboard:iterations').Set(32)
settings.GetAttribute('cardboard:substeps').Set(32)
settings.GetAttribute('cardboard:robotiqDriveDamping').Set(160.)
settings.GetAttribute('cardboard:robotiqPinchDamping').Set(160.)
stage.GetRootLayer().Save()
report=dict(source=str(source.relative_to(ROOT)),scene=str(target.relative_to(ROOT)),material_overrides=changes,
            iterations=32,substeps=32,visual_crease_degrees=80.,visual_transition_degrees=40.,
            drive_damping=160.,pinch_damping=160.,
            qualification='Illustrative local crease softening; intact stiffness, membrane, thickness, mass and topology unchanged. Not calibrated or nonlocal fracture regularization.')
(ROOT/'assets/build_report_robotiq_creased.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
