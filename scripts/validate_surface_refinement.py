"""Validate display-only refinement against the exact same recorded physics."""
import argparse
import hashlib
import json
import time

import numpy as np
from pxr import Usd, UsdGeom

from cardboard import ROOT
from cardboard.surface import PanelSurface, unit
from cardboard.usd_utils import register

p = argparse.ArgumentParser()
p.add_argument('--baseline', default='assets/demo_scene_robotiq_surface_baseline.usda')
p.add_argument('--candidate', default='assets/demo_scene_robotiq_scaled.usda')
p.add_argument('--recording', default='outputs/robotiq_scaled/trajectory.npz')
p.add_argument('--output', default='outputs/surface_refinement/validation.json')
a = p.parse_args()
register()


def load(path):
    stage = Usd.Stage.Open(str(ROOT / path))
    mesh = stage.GetPrimAtPath('/World/Box/SimMesh')
    render = stage.GetPrimAtPath('/World/Box/RenderMesh')
    attr = lambda n: render.GetAttribute('cardboard:' + n).Get()
    rest = np.asarray(mesh.GetAttribute('cardboard:restPoints').Get())
    faces = np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    settings = (attr('visualCreaseAngleDegrees'), attr('visualCreaseTransitionDegrees'), bool(attr('visualSmoothThickness')))
    surface = PanelSurface(rest, faces, attr('bindingIndices'), attr('bindingWeights'), attr('bindingOffsets'), *settings)
    center = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(mesh).ExtractTranslation())
    return stage, surface, center, settings


before_stage, before, center, before_settings = load(a.baseline)
after_stage, after, _, after_settings = load(a.candidate)
allowed = {'cardboard:visualCreaseAngleDegrees', 'cardboard:visualCreaseTransitionDegrees', 'cardboard:visualSmoothThickness'}


def scene_contract(stage):
    # Every authored/effective attribute, connection and relationship except the
    # three display settings must match, including UVs, materials and solver data.
    result = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        result[path] = (prim.GetTypeName(), tuple(prim.GetAppliedSchemas()))
        for attr in prim.GetAttributes():
            if path == '/World/Box/RenderMesh' and attr.GetName() in allowed:
                continue
            result[path + '.' + attr.GetName()] = (str(attr.Get()), str(attr.GetConnections()))
        for rel in prim.GetRelationships():
            result[path + '.' + rel.GetName()] = str(rel.GetTargets())
    return result


rest_before, _ = before.evaluate(before.rest)
rest_after, _ = after.evaluate(after.rest)
max_delta = 0.
rms = []
durations = []
finite = True
unit_normals = True
samples = {}
recording = ROOT / a.recording
digest = hashlib.sha256(recording.read_bytes()).hexdigest()
with np.load(recording) as data:
    for i in sorted(set(range(0, len(data['t']), 10)) | {len(data['t']) - 1, int(np.argmin(abs(data['t'] - 11.8)))}):
        q = data['points'][i] - center
        original, _ = before.evaluate(q)
        started = time.perf_counter()
        refined, normals = after.evaluate(q)
        durations.append(time.perf_counter() - started)
        delta = np.linalg.norm(refined - original, axis=1)
        max_delta = max(max_delta, float(delta.max()))
        rms.append(float(np.sqrt(np.mean(delta**2))))
        finite &= bool(np.isfinite(refined).all() and np.isfinite(normals).all())
        unit_normals &= bool(np.allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-5))
    for target in (11.8, 16.):
        i = int(np.argmin(abs(data['t'] - target)))
        q = data['points'][i] - center
        tri = q[before.faces]
        n = unit(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]))
        angles = np.rad2deg(np.arccos(np.clip((n[before.edge_adj[:, 0]] * n[before.edge_adj[:, 1]]).sum(-1), -1, 1)))
        edges = np.flatnonzero((~before.seam) & (angles < 40))
        pairs = before.edge_adj[edges]
        faces = before.faces[pairs.ravel()]
        weights = np.array([[.5 if v in edge else 0. for v in face]
                            for edge, pair in zip(before.edges[edges], pairs)
                            for face in before.faces[pair]])
        # Same 7.5mm skin offset on both sides isolates the artificial thickness
        # step; original box seams and genuine sharp folds are excluded.
        rest_tri = before.rest[faces]
        offset = unit(np.cross(rest_tri[:, 1] - rest_tri[:, 0], rest_tri[:, 2] - rest_tri[:, 0])) * .0075
        values = []
        for settings in (before_settings, after_settings):
            probe = PanelSurface(before.rest, before.faces, faces, weights, offset, *settings)
            points, _ = probe.evaluate(q)
            steps = np.linalg.norm(points[0::2] - points[1::2], axis=1)
            values.append(float(np.sqrt(np.mean(steps**2))))
        samples[str(target)] = dict(gentle_edges=len(edges), before_step_rms_m=values[0], after_step_rms_m=values[1])

checks = dict(
    only_display_settings_changed=scene_contract(before_stage) == scene_contract(after_stage),
    rest_shape_unchanged=bool(np.allclose(rest_before, rest_after, atol=2e-7)),
    finite_surfaces=finite,
    unit_normals=unit_normals,
    max_display_shift_under_8mm=max_delta < .008,
    gentle_edge_steps_reduced_over_50percent=all(v['after_step_rms_m'] < .5 * v['before_step_rms_m'] for v in samples.values()),
    recording_unchanged=hashlib.sha256(recording.read_bytes()).hexdigest() == digest,
)
report = dict(passed=all(checks.values()), checks=checks, evaluated_frames=len(durations),
              max_display_shift_m=max_delta, max_frame_rms_display_shift_m=max(rms),
              surface_evaluate_median_ms=float(np.median(durations) * 1000),
              thickness_step_probes=samples, trajectory_sha256=digest,
              qualification='Display reconstruction of unchanged physics. No new physical wrinkle detail or material calibration; thickness-step probes exclude seams and folds >=40deg.')
(ROOT / a.output).write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
