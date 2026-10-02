"""Check material/geometry preservation and measured behavior after remeshing."""
import argparse
import csv
import json

import numpy as np
from pxr import Usd, UsdGeom

from cardboard import ROOT
from cardboard.usd_utils import register

p = argparse.ArgumentParser()
p.add_argument('--output', default='outputs/robotiq_fine24')
p.add_argument('--baseline-scene', default='assets/demo_scene_robotiq_coarse_preserved.usda')
a = p.parse_args()
register()
out = ROOT / a.output
config = json.loads((out / 'run_config.json').read_text())
stages = [Usd.Stage.Open(str(ROOT / path)) for path in [a.baseline_scene, config['scene']]]
materials = [{attr.GetName(): str(attr.Get()) for attr in s.GetPrimAtPath('/World/Box/Materials/Cardboard').GetAttributes()}
             for s in stages]
meshes = [UsdGeom.Mesh(s.GetPrimAtPath('/World/Box/SimMesh')) for s in stages]
points = [np.asarray(m.GetPointsAttr().Get()) for m in meshes]
faces = [np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3) for m in meshes]
areas = []
edge_max = []
for q, f in zip(points, faces):
    tri = q[f]
    areas.append(float(np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1).sum() * .5))
    edge_max.append(float(np.linalg.norm(np.roll(tri, -1, axis=1) - tri, axis=2).max()))
f = faces[1]
edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
_, counts = np.unique(edges, axis=0, return_counts=True)
rows = list(csv.DictReader((out / 'state.csv').open()))
v = {k: np.array([float(row[k]) for row in rows]) for k in rows[0]}
def end(phase, key):
    return float(v[key][v['phase'] == phase][-1])
sequence = json.loads((out / 'validation.json').read_text())
checks = dict(
    full_contact_linkage_sequence_passed=sequence['passed'],
    material_identical=materials[0] == materials[1],
    midsurface_bounds_identical=bool(np.allclose(points[0].min(0), points[1].min(0), atol=1e-7) and
                                    np.allclose(points[0].max(0), points[1].max(0), atol=1e-7)),
    rest_area_and_mass_same=abs(areas[1] / areas[0] - 1) < 1e-6,
    physical_faces_at_least_four_times=len(faces[1]) >= 4 * len(faces[0]),
    physical_edge_length_at_least_halved=edge_max[1] < .51 * edge_max[0],
    closed_manifold=bool(np.all(counts == 2)),
    positive_dual_widths=bool(np.all(np.asarray(meshes[1].GetPrim().GetAttribute('cardboard:dualWidths').Get()) > 0)),
    strong_squeeze_under_210mm=end(5, 'actual_gap_m') < .21,
    squeeze_adds_over_50mm_closure=end(4, 'actual_gap_m') - end(5, 'actual_gap_m') > .05,
)
report = dict(passed=all(checks.values()), checks=checks, vertices=[len(q) for q in points],
              physical_faces=[len(f) for f in faces], max_rest_edge_m=edge_max,
              rest_areas_m2=areas, loaded_crush_gap_m=end(5, 'actual_gap_m'),
              active_wall_s=float(v['active_wall_s'][-1]),
              qualification='Same material with refined discretization and rest-neighbor collision exclusion; not a quantitative convergence/material-calibration claim.')
(out / 'resolution_validation.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
