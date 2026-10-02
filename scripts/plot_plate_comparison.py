"""Measured geometry/motion comparison for the plate correction."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,3,figsize=(13,3.8),layout='constrained')
for folder,label,color in [('panel_dense_final','Previous dense shell','#b75b46'),('plate_crease','Stiff panels + local creases','#2666a5')]:
    path=Path('outputs')/folder
    rows=list(csv.DictReader((path/'state.csv').open()))
    shape=json.loads((path/'shape_retention.json').read_text())['samples']
    t=np.array([float(r['t']) for r in rows])
    axes[0].plot([v['t'] for v in shape.values()],[100*v['volume_fraction_of_initial'] for v in shape.values()],'-o',label=label,color=color)
    axes[1].plot(t,[100*float(r['plastic_hinges'])/10368 for r in rows],color=color)
    axes[2].plot(t,[1000*float(r['internal_speed_rms_m_s']) for r in rows],color=color)
for ax,title,ylabel in zip(axes,['Enclosed volume retained','Hinges with permanent history','Motion within the box'],['% of initial volume','% of 10,368 hinges','Internal RMS velocity (mm/s)']):
    ax.set(title=title,xlabel='Simulation time (s)',ylabel=ylabel,xlim=(9,16));ax.axvline(12,color='.4',ls=':',lw=1);ax.grid(alpha=.2)
axes[0].legend(fontsize=8);axes[2].set_yscale('log');axes[2].set_ylim(.1,300)
fig.suptitle('Same mesh; material and grasp settings changed. Dotted line: release begins. Uncalibrated demo.',fontsize=10)
fig.savefig('outputs/plate_crease/behavior_comparison.png',dpi=170)
