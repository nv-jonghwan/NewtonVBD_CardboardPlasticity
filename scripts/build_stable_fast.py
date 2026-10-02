"""Isolated fixed-resolution solver/damping candidate, preserving source scene."""
import argparse,json
from pxr import Usd,Sdf
from cardboard import ROOT
from cardboard.usd_utils import register
p=argparse.ArgumentParser();p.add_argument('--substeps',type=int,default=16);p.add_argument('--iterations',type=int,default=32);p.add_argument('--interval',type=int,default=2);p.add_argument('--damping',type=float,default=60.);p.add_argument('--sleep',action='store_true');p.add_argument('--rotation',action='store_true');p.add_argument('--name',default='robotiq_stable_fast');a=p.parse_args();register()
target=ROOT/'assets'/f'demo_scene_{a.name}.usda'
stage=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_creased.usda'));stage.GetRootLayer().Export(str(target));stage=Usd.Stage.Open(str(target))
physics=stage.GetPrimAtPath('/World/Physics');material=stage.GetPrimAtPath('/World/Box/Materials/Cardboard')
for prim,name,value,kind in [(physics,'iterations',a.iterations,Sdf.ValueTypeNames.Int),(physics,'substeps',a.substeps,Sdf.ValueTypeNames.Int),(physics,'particleSolveInterval',a.interval,Sdf.ValueTypeNames.Int),(physics,'supportedSleep',a.sleep,Sdf.ValueTypeNames.Bool),(physics,'rotationBlockSolve',a.rotation,Sdf.ValueTypeNames.Bool),(material,'internalVelocityDamping',a.damping,Sdf.ValueTypeNames.Float)]:
 attr=prim.GetAttribute('cardboard:'+name)
 if not attr:attr=prim.CreateAttribute('cardboard:'+name,kind,custom=True)
 attr.Set(value)
stage.GetRootLayer().Save();print(target)
