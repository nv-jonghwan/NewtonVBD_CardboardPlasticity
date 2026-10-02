"""Build the higher-resolution paper-panel variant without changing old replays."""
from cardboard.build_asset import build
from cardboard.usd_utils import register
from cardboard import ROOT
from pxr import Usd
build(n=24,variant='_dense',render_max_edge=.012)
register();asset=Usd.Stage.Open(str(ROOT/'assets/cardboard_dense.usda'));material=asset.GetPrimAtPath('/Box/Materials/Cardboard')
for key,value in dict(membraneShear=17640.,membraneArea=29400.,membraneDamping=.02,bendingDamping=.0006,yieldCurvature=8.4,hardeningRatio=.01,damageRate=.5).items():material.GetAttribute('cardboard:'+key).Set(value)
asset.GetRootLayer().Save()
scene=Usd.Stage.Open(str(ROOT/'assets/demo_scene_dense.usda'));physics=scene.GetPrimAtPath('/World/Physics')
physics.GetAttribute('cardboard:crushGap').Set(.19)
physics.GetAttribute('cardboard:iterations').Set(64);physics.GetAttribute('cardboard:gravityRampSeconds').Set(.75)
scene.GetRootLayer().Save()
