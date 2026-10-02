"""Review user-requested visual response, contact coverage and sequence metrics."""
import argparse
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cardboard import ROOT

p=argparse.ArgumentParser(__doc__);p.add_argument('directory');p.add_argument('--log',required=True);a=p.parse_args()
out=ROOT/a.directory
metrics=json.loads((out/'tuning_metrics.json').read_text())
rows=list(csv.DictReader((out/'state.csv').open()))
v={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
samples=metrics['samples'];baseline=metrics['baseline']['samples']
state=np.load(out/'final_material_state.npz')
log=(ROOT/a.log).read_text()
ratio={t:samples[t]['top_sag_mm']/baseline[t]['top_sag_mm'] for t in ['7','8','9','12']}
checks=dict(
    finite_geometry=metrics['finite_geometry'],
    no_pregrasp_yield=bool(np.all(v['plastic_hinges'][v['phase']<3]==0)),
    pregrasp_sag_about_1mm=.5<=samples['5']['top_sag_mm']<=1.5,
    # "About half" is assessed explicitly, rather than claiming an exact ratio.
    grasp_lift_sag_35_to_65_percent=all(.35<=ratio[t]<=.65 for t in ['7','8','9']),
    crush_sag_within_10_percent=abs(ratio['12']-1)<=.1,
    crush_gap_within_15mm=abs(samples['12']['actual_gap_m']-baseline['12']['actual_gap_m'])<=.015,
    no_logged_contact_overflow='overflow' not in log.lower(),
    positive_monotonic_plastic_work=bool(np.all(state['plastic_work']>=0) and np.all(np.diff(v['plastic_work_J'])>=-1e-5)),
    valid_accumulated_history=bool(np.all(state['accumulated_angle']>=abs(state['plastic_angle'])-1e-5)),
    finite_material_state=all(np.isfinite(state[k]).all() for k in state.files),
    damage_bounded=bool(np.all((state['damage']>=0)&(state['damage']<1))),
)
result=dict(response_checks_passed=all(checks.values()),checks=checks,sag_ratios=ratio,
            wall_s=metrics['performance'],
            qualification='Qualitative user-requested tuning; separate sequence validation records settling and linkage limits. Timings are single observations, not an isolated speedup study.')
(out/'tuning_review.json').write_text(json.dumps(result,indent=2))

fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
for key,label,color in [('baseline','Before','#a65435'),(None,'Tuned','#187b87')]:
    m=metrics[key] if key else metrics
    axes[0].plot(m['t'],m['sag_mm'],label=label,color=color)
base_rows=list(csv.DictReader((ROOT/'outputs/vbd_optimized/guarded_b/state.csv').open()))
axes[1].plot([float(r['t']) for r in base_rows],[1000*float(r['actual_gap_m']) for r in base_rows],color='#a65435')
axes[1].plot(v['t'],1000*v['actual_gap_m'],color='#187b87')
for ax in axes:
    for t in [5,7,9,12,13]:ax.axvline(t,color='gray',ls=':',alpha=.5)
    ax.grid(alpha=.2)
axes[0].set(ylabel='Top-center sag relative to rim (mm)',title='Lower initial/grasp sag, retained final crush depth')
axes[0].legend();axes[1].set(xlabel='Simulation time (s)',ylabel='Actual fingertip gap (mm)')
fig.savefig(out/'tuning_comparison.png',dpi=160)
plt.close(fig)
print(json.dumps(result,indent=2))
