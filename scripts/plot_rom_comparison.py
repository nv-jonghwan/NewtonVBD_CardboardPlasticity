"""Export measured ROM/FOM comparisons; alignment is diagnostic only."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_rom import read_run

parser = argparse.ArgumentParser()
parser.add_argument('--baseline', required=True)
parser.add_argument('--candidate', required=True)
args = parser.parse_args()
b, c = read_run(args.baseline), read_run(args.candidate)
bq, cq = b['trajectory']['points'].astype(float), c['trajectory']['points'].astype(float)
t = b['trajectory']['t']
if bq.shape != cq.shape or not np.allclose(t, c['trajectory']['t']):
    raise ValueError('Matched times and topology required')
ba, ca = bq-bq.mean(axis=1,keepdims=True), cq-cq.mean(axis=1,keepdims=True)
u, _, vt = np.linalg.svd(np.einsum('tni,tnj->tij',ca,ba))
u[:,:,-1] *= np.linalg.det(u @ vt)[:,None]
aligned = ca @ (u @ vt)
shape_rms = np.sqrt(np.mean(np.sum((aligned-ba)**2,axis=2),axis=1))
world_rms = np.sqrt(np.mean(np.sum((cq-bq)**2,axis=2),axis=1))
fig, axes = plt.subplots(2,2,figsize=(11,7),layout='constrained')
for label, run, color in [('Full VBD',b,'#3266a8'),('Guarded ROM',c,'#d56a28')]:
    v = run['values']
    axes[0,0].plot(v['t'],v['actual_gap_m']*1000,label=label,color=color)
    axes[0,1].plot(v['t'],v['plastic_work_J'],label=label,color=color)
    axes[1,1].plot(v['t'],np.cumsum(run['performance']['frame_times']),label=label,color=color)
axes[1,0].plot(t,world_rms*1000,label='World position RMS',color='#7846aa')
axes[1,0].plot(t,shape_rms*1000,label='Rigid-aligned shape RMS (diagnostic)',color='#299369')
axes[1,0].axhline(2,color='gray',ls='--',label='World RMS screening limit')
for ax, title, unit in zip(axes.flat,
        ['Measured jaw opening','Accumulated plastic work','ROM difference from full VBD','Measured compute time (shared GPUs)'],
        ['mm','J','mm','wall seconds']):
    ax.set(title=title,xlabel='simulation seconds',ylabel=unit)
    ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('Fresh 16-second simulations, same mesh and material')
out=Path(args.candidate)
fig.savefig(out/'comparison.png',dpi=170)
fig.savefig(out/'comparison.svg')
report=dict(max_aligned_shape_rms_m=float(shape_rms.max()),final_aligned_shape_rms_m=float(shape_rms[-1]),
            note='Rigid alignment diagnoses shape vs pose changes; it does not replace world-space acceptance gates.')
(out/'shape_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
