"""Isolated early-fold probes; canonical assets and GUI are left untouched."""
import argparse
import csv
import hashlib
import json
import os
import time
from pathlib import Path

import numpy as np
import warp as wp
from pxr import Usd, UsdGeom, UsdPhysics

from cardboard import ROOT
from cardboard.scenario import PickCrushDrop


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--variant', choices=['canonical', 'zero_gravity', 'sweeps48',
                        'bend4', 'inner_knuckles_off', 'force80'], default='canonical')
    parser.add_argument('--duration', type=float, default=9.)
    parser.add_argument('--device', default='cuda:1')
    args = parser.parse_args()
    out = ROOT/'outputs/early_top_fold'/args.variant
    out.mkdir(parents=True, exist_ok=False)
    source = ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'
    # Flatten to preserve all composition and stage metadata in this local trial.
    stage = Usd.Stage.Open(str(source))
    stage.Flatten().Export(str(out/'scene.usda'))
    stage = Usd.Stage.Open(str(out/'scene.usda'))
    if args.variant == 'sweeps48':
        stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:particleSolveInterval').Set(1)
    if args.variant == 'force80':
        stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:graspForce').Set(80.)
    if args.variant == 'inner_knuckles_off':
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith('/World/Gripper/') and prim.HasAPI(UsdPhysics.CollisionAPI):
                if prim.GetParent().GetName() in {'left_inner_knuckle', 'right_inner_knuckle'}:
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    stage.GetRootLayer().Save()
    os.environ['CARDBOARD_VBD_SCHEDULE'] = 'guarded'
    overrides = None
    if args.variant == 'bend4':
        mesh = stage.GetPrimAtPath('/World/Box/SimMesh')
        mat = stage.GetPrimAtPath(mesh.GetRelationship('cardboard:material').GetTargets()[0])
        overrides = {key: 4*mat.GetAttribute('cardboard:'+key).Get() for key in ['bendingMD', 'bendingCD']}
    s = PickCrushDrop(scene_path=out/'scene.usda', device=args.device, material_overrides=overrides)
    if args.variant == 'zero_gravity':
        s.gravity_full *= 0
        s.model.gravity.zero_()
    local = s.local - (s.local.min(0)+s.local.max(0))/2
    half = np.ptp(local, axis=0)/2
    top = np.isclose(local[:, 2], half[2], atol=1e-6)
    center = top & (np.abs(local[:, 0]) < .2*half[0]) & (np.abs(local[:, 1]) < .2*half[1])
    rim = top & ((np.abs(local[:, 0]) > .999*half[0]) | (np.abs(local[:, 1]) > .999*half[1]))
    edges = s.model.edge_indices.numpy()
    top_edges = np.all(top[edges], axis=1)
    records, points, bodies, wrenches, plastics, times = [], [], [], [], [], []
    started = time.perf_counter()
    for i in range(round(args.duration*s.fps)):
        row = dict(s.advance())
        q = s.a.particle_q.numpy()
        plane_center = q[rim].mean(0)
        normal = np.linalg.svd(q[rim]-plane_center)[2][-1]
        normal *= 1 if normal[2] >= 0 else -1
        alpha = s.a.cardboard.accumulated_angle.numpy()
        w = s.wrench.numpy()
        row.update(top_sag_mm=float((plane_center-q[center].mean(0))@normal*1000),
                   top_plastic_hinges=int(np.count_nonzero(alpha[top_edges] > 1e-4)))
        for bi, label in enumerate(s.model.body_label):
            short = label.rsplit('/', 1)[-1]
            row[short+'_force_N'] = float(np.linalg.norm(w[bi, :3]))
            row[short+'_force_z_N'] = float(w[bi, 2])
        records.append(row)
        if i % 2 == 1:
            times.append(s.time); points.append(q); bodies.append(s.a.body_q.numpy())
            wrenches.append(w); plastics.append(alpha)
        if i % 60 == 59:
            print(json.dumps({key: row[key] for key in ['t', 'top_sag_mm', 'plastic_hinges',
                             'top_plastic_hinges', 'left_contact_N', 'right_contact_N']}), flush=True)
    with (out/'state.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=records[0]); writer.writeheader(); writer.writerows(records)
    np.savez_compressed(out/'diagnostic.npz', t=times, points=points, body_q=bodies,
                        wrench=wrenches, accumulated_angle=plastics, body_labels=s.model.body_label,
                        local=s.local, edges=edges, top_edges=top_edges, top=top, center=center, rim=rim)
    report = dict(variant=args.variant, duration_s=s.time, wall_s=time.perf_counter()-started,
                  device=args.device, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  material_overrides=overrides, particle_interval=s.active_particle_interval,
                  mass_kg=float(s.model.particle_mass.numpy().sum()),
                  top_yield_angle_degrees_percentiles=np.percentile(
                      s.dual.numpy()[top_edges]*float(s.attr(s.mat,'yieldCurvature'))*180/np.pi,
                      [0, 50, 100]).tolist(), final=records[-1])
    (out/'summary.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
