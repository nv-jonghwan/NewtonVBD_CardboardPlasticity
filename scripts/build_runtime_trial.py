"""Separate reduced-resolution physics trial; retain dense source appearance."""
import argparse
from pxr import Usd
from cardboard.build_asset import build
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--n',type=int,required=True);a=p.parse_args();variant=f'_runtime_n{a.n}'
build(n=a.n,variant=variant,render_max_edge=.012)
baseline=Usd.Stage.Open(str(ROOT/'assets/cardboard_thick16_stronger.usda'));asset=Usd.Stage.Open(str(ROOT/f'assets/cardboard{variant}.usda'))
src=baseline.GetPrimAtPath('/Box/Materials/Cardboard');dst=asset.GetPrimAtPath('/Box/Materials/Cardboard')
for name in src.GetPropertyNames():
 if name.startswith('cardboard:'):dst.GetAttribute(name).Set(src.GetAttribute(name).Get())
asset.GetPrimAtPath('/Box/SimMesh').GetAttribute('cardboard:solver').Set('newtonVBD16')
src=baseline.GetPrimAtPath('/Box/RenderMesh');dst=asset.GetPrimAtPath('/Box/RenderMesh')
for name in ['surfaceInterpolation','visualCreaseAngleDegrees','visualCreaseTransitionDegrees']:
 dst.GetAttribute('cardboard:'+name).Set(src.GetAttribute('cardboard:'+name).Get())
asset.GetRootLayer().Save()
s=Usd.Stage.Open(str(ROOT/'assets/demo_scene_thick16_stronger.usda'));s.GetRootLayer().Export(str(ROOT/f'assets/demo_scene{variant}.usda'));s=Usd.Stage.Open(str(ROOT/f'assets/demo_scene{variant}.usda'));b=s.GetPrimAtPath('/World/Box');b.GetReferences().ClearReferences();b.GetReferences().AddReference(f'./cardboard{variant}.usda');s.GetRootLayer().Save()
