"""Quantitative summaries and plots from isolated early-fold probes."""
import csv
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation

from cardboard import ROOT

base = ROOT/'outputs/early_top_fold'
variants = ['canonical', 'zero_gravity', 'sweeps48', 'bend4', 'inner_knuckles_off', 'force80']
rows = {name: [{k: float(v) for k, v in row.items()} for row in csv.DictReader(
    (base/name/'state.csv').open())] for name in variants}
summary = {}
for name, data in rows.items():
    def first(predicate):
        return next((r['t'] for r in data if predicate(r)), None)
    summary[name] = dict(
        first_inner_knuckle_contact_s=first(lambda r: max(r['left_inner_knuckle_force_N'], r['right_inner_knuckle_force_N']) > 1),
        first_pad_body_contact_s=first(lambda r: max(r['left_contact_N'], r['right_contact_N']) > 1),
        first_plastic_s=first(lambda r: r['plastic_hinges'] > 0),
        first_top_plastic_s=first(lambda r: r['top_plastic_hinges'] > 0),
        samples={str(t): min(data, key=lambda r: abs(r['t']-t)) for t in [1, 2, 5, 6, 7, 8, 9]
                 if t <= data[-1]['t']+1e-6})

# Reconstruct OBB distances from recorded rigid poses. This locates nearby
# vertices, not exact active barycentric contact quadrature points.
z = np.load(base/'canonical/diagnostic.npz')
stage = Usd.Stage.Open(str(base/'canonical/scene.usda')); cache = UsdGeom.XformCache()
labels = list(z['body_labels']); proximity = []
for t in [5.5, 6, 7]:
    i = np.argmin(abs(z['t']-t)); q = z['points'][i]
    for name in ['left_inner_knuckle', 'right_inner_knuckle']:
        body = stage.GetPrimAtPath('/World/Gripper/'+name)
        prim = stage.GetPrimAtPath(str(body.GetPath())+'/Collider_Finger3')
        relative = np.asarray(cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(body).GetInverse())
        pose = z['body_q'][i, labels.index(str(body.GetPath()))]
        qb = (q-pose[:3])@Rotation.from_quat(pose[3:]).as_matrix()
        cube = (np.c_[qb, np.ones(len(q))]@np.linalg.inv(relative))[:, :3]
        scale = np.linalg.norm(relative[:3, :3], axis=1)
        distance = np.linalg.norm(np.maximum(abs(cube)-.5, 0)*scale, axis=1)
        hit = distance <= .007
        proximity.append(dict(t=float(z['t'][i]), body=name, threshold_m=.007,
            min_distance_m=float(distance.min()), vertices=int(hit.sum()),
            top_vertices=int(np.count_nonzero(hit & z['top'])),
            rest_bounds_m=[z['local'][hit].min(0).tolist(), z['local'][hit].max(0).tolist()]))
summary['collider_proximity'] = proximity
summary['previous_schedule_runs'] = {}
for name in ['baseline_b', 'guarded_a', 'guarded_b', 'guarded_c']:
    previous = np.load(ROOT/'outputs/vbd_optimized'/name/'trajectory.npz')
    samples = {}
    for t in [5, 7, 8]:
        q = previous['points'][np.argmin(abs(previous['t']-t))]
        rim_center = q[z['rim']].mean(0)
        normal = np.linalg.svd(q[z['rim']]-rim_center)[2][-1]
        normal *= 1 if normal[2] >= 0 else -1
        samples[str(t)] = float((rim_center-q[z['center']].mean(0))@normal*1000)
    summary['previous_schedule_runs'][name] = samples
(base/'analysis.json').write_text(json.dumps(summary, indent=2))

fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
colors = {'canonical': '#b53c36', 'inner_knuckles_off': '#187b87', 'force80': '#b98a10'}
for name, label in [('canonical', 'Current scene'), ('inner_knuckles_off', 'Inner-link collisions off (diagnostic)'),
                    ('force80', 'Grasp reference 80 N (diagnostic)')]:
    data = rows[name]; t = [r['t'] for r in data]
    axes[0].plot(t, [r['top_sag_mm'] for r in data], label=label, color=colors[name])
    axes[2].plot(t, [r['top_plastic_hinges'] for r in data], color=colors[name])
data = rows['canonical']; t = [r['t'] for r in data]
for key, label, color in [('left_contact_N', 'Left fingertip body', '#b53c36'),
                         ('left_inner_knuckle_force_N', 'Left inner link', '#264d7c'),
                         ('right_inner_knuckle_force_N', 'Right inner link', '#548ca8')]:
    axes[1].plot(t, [r[key] for r in data], label=label, color=color, linewidth=1)
for ax in axes:
    ax.axvline(5, color='gray', linestyle=':'); ax.axvline(7, color='gray', linestyle=':')
    ax.grid(alpha=.2); ax.set_xlim(0, 9)
axes[0].set_title('Top-panel deformation starts before the squeeze phase (9 s)')
axes[0].set_ylabel('Center sag relative to rim (mm)'); axes[0].legend(fontsize=9)
axes[1].set_ylabel('Contact-force norm (N)'); axes[1].legend(fontsize=9)
axes[2].set_ylabel('Yielded internal top hinges'); axes[2].set_xlabel('Simulation time (s); grasp 5–7 s, lift 7–9 s')
fig.savefig(base/'early_fold_comparison.png', dpi=170); plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
for name, label in [('canonical', 'Current: 17 shell sweeps'), ('zero_gravity', 'Zero gravity'),
                    ('sweeps48', '48 shell sweeps'), ('bend4', '4x bending stiffness')]:
    data = [r for r in rows[name] if r['t'] <= 2+1e-6]
    axes[0].plot([r['t'] for r in data], [r['top_sag_mm'] for r in data], label=label)
axes[0].set(xlabel='Time (s)', ylabel='Top-center sag (mm)', title='Before any gripper contact')
axes[0].legend(fontsize=8); axes[0].grid(alpha=.2)
mask = z['top'] & np.isclose(z['local'][:, 1], 0, atol=1e-6)
for t in [0, 5, 7, 8]:
    if t == 0:
        q = z['local']; label = 'Initial (flat)'
    else:
        q = z['points'][np.argmin(abs(z['t']-t))]; label = f'{t} s'
    pts = q[mask]; order = np.argsort(z['local'][mask, 0]); pts = pts[order]
    axes[1].plot((pts[:, 0]-pts[:, 0].mean())*1000,
                 (pts[:, 2]-(pts[0, 2]+pts[-1, 2])/2)*1000, label=label)
axes[1].set(xlabel='Across top panel (mm)', ylabel='Height relative to section ends (mm)',
            title='Physical mesh: center cross-section')
axes[1].legend(fontsize=8); axes[1].grid(alpha=.2)
fig.savefig(base/'initial_sag_and_sections.png', dpi=170); plt.close(fig)
print(json.dumps({name: {key: value for key, value in summary[name].items() if key != 'samples'}
                  for name in variants}, indent=2))
