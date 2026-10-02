"""Author a held-out command variation while preserving stage units and axes."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdGeom

from cardboard import ROOT

parser = argparse.ArgumentParser()
parser.add_argument('--source', default='assets/demo_scene_robotiq_board10_primitive.usda')
parser.add_argument('--output', default='outputs/rom_trial/varied_scene_v2.usda')
args = parser.parse_args()
source, output = ROOT / args.source, ROOT / args.output
if output.exists():
    raise FileExistsError(output)
base = Usd.Stage.Open(str(source))
output.parent.mkdir(parents=True, exist_ok=True)
layer = Sdf.Layer.CreateNew(str(output))
layer.subLayerPaths = [os.path.relpath(source, output.parent)]
stage = Usd.Stage.Open(layer)
# Stage metadata is not inherited through a sublayer by UsdGeom accessors.
UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(base))
UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(base))
stage.SetDefaultPrim(stage.GetPrimAtPath(base.GetDefaultPrim().GetPath()))
physics = stage.GetPrimAtPath('/World/Physics')
changes = {}
for name, transform in [('liftHeight', lambda x: .18),
                        ('crushGap', lambda x: x + .01),
                        ('crushForce', lambda x: x * .9)]:
    attr = physics.GetAttribute('cardboard:' + name)
    old = attr.Get()
    attr.Set(transform(old))
    changes[name] = dict(baseline=old, varied=attr.Get())
layer.Save()
mesh_path = '/World/Box/SimMesh'
for name in ['cardboard:restPoints', 'faceVertexIndices', 'faceVertexCounts']:
    np.testing.assert_array_equal(base.GetPrimAtPath(mesh_path).GetAttribute(name).Get(),
                                  stage.GetPrimAtPath(mesh_path).GetAttribute(name).Get())
report = dict(scene=str(output.relative_to(ROOT)), changes=changes,
              up_axis=str(UsdGeom.GetStageUpAxis(stage)), meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
              source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              scene_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
              mesh_unchanged=True, purpose='Held-out commands with unchanged rank8 basis')
output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
