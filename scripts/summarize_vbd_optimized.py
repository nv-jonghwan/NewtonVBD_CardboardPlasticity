"""Summarize sequential nominal repeats without hiding sleep-mode differences."""
import csv,json,statistics
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cardboard import ROOT

out=ROOT/'outputs/vbd_optimized'
groups={'baseline':['baseline_b','baseline_c','baseline_d'],
        'guarded':['guarded_a','guarded_b','guarded_c']}
runs={}
for name in sum(groups.values(),[])+['varied_baseline','varied_guarded']:
    folder=out/name
    performance=json.loads((folder/'performance.json').read_text())
    validation=json.loads((folder/'validation.json').read_text())
    config=json.loads((folder/'run_config.json').read_text())
    rows=list(csv.DictReader((folder/'state.csv').open()))
    times=np.asarray(performance['frame_times'])*1000
    phase=np.asarray([int(row['phase']) for row in rows]);mode=np.asarray([int(row['solver_mode']) for row in rows])
    phases={str(i):dict(median_ms=float(np.median(times[phase==i])),total_s=float(times[phase==i].sum()/1000)) for i in range(8)}
    modes={str(i):dict(n=int(np.sum(mode==i)),median_ms=float(np.median(times[mode==i])) if np.any(mode==i) else None) for i in range(3)}
    runs[name]=dict(device=config['device'],wall_s=performance['wall_time_s'],complete=performance['error'] is None and abs(performance['duration_s']-16)<.02,
                    phases=phases,modes=modes,validation=validation,scene_sha256=config['scene_sha256'])

summary={}
for label,names in groups.items():
    walls=[runs[n]['wall_s'] for n in names]
    summary[label]=dict(wall_median_s=statistics.median(walls),wall_min_s=min(walls),wall_max_s=max(walls),
                        phase_median_ms={str(i):statistics.median(runs[n]['phases'][str(i)]['median_ms'] for n in names) for i in range(8)},
                        sequence_pass_counts=[sum(runs[n]['validation']['checks'].values()) for n in names])
b,c=summary['baseline'],summary['guarded']
phase_reductions={str(i):(1-c['phase_median_ms'][str(i)]/b['phase_median_ms'][str(i)])*100 for i in range(8)}
result=dict(groups=groups,runs=runs,summary=summary,nominal_wall_reduction_percent=(1-c['wall_median_s']/b['wall_median_s'])*100,
            nominal_speedup=b['wall_median_s']/c['wall_median_s'],phase_time_reduction_percent=phase_reductions,
            varied_wall_reduction_percent=(1-runs['varied_guarded']['wall_s']/runs['varied_baseline']['wall_s'])*100,
            all_complete=all(r['complete'] for r in runs.values()),
            nonsettling_sequence_checks_pass=all(all(v for k,v in r['validation']['checks'].items() if k!='settled') for r in runs.values()),
            strict_trajectory_equivalence_claimed=False,physical_budgets_unchanged=True,
            caveats=['Same device within each comparison: nominal GPU1, varied GPU0. No GPU clock lock or universal speed claim.',
                     'Early baseline_a had different sleep timing and is retained outside the three-repeat timing summary.',
                     'Original repeats themselves differ in final shape and settling; report both phase cost and overall wall time.',
                     '32-worker adaptive candidate was rejected; guarded uses original four workers per body.'])
(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for i,(label,names) in enumerate(groups.items()):
    values=[runs[n]['wall_s'] for n in names]
    axes[0].scatter([i]*len(values),values,label=label,s=45)
    axes[0].plot([i-.15,i+.15],[statistics.median(values)]*2,lw=3)
axes[0].set(xticks=[0,1],xticklabels=['Original','Guarded'],ylabel='Wall time (s)',title='Fresh 16-second scenario: three repeats')
phases=[3,4,5,7];x=np.arange(len(phases))
for i,label in enumerate(groups):
    axes[1].bar(x+(i-.5)*.35,[summary[label]['phase_median_ms'][str(p)] for p in phases],width=.35,label=label)
axes[1].set(xticks=x,xticklabels=['Grasp','Lift','Crush','Settle'],ylabel='Median ms / frame',title='Median of per-run phase medians')
axes[1].legend()
for ax in axes:ax.grid(axis='y',alpha=.2)
fig.savefig(out/'performance.png',dpi=170);fig.savefig(out/'performance.svg')
print(json.dumps({k:result[k] for k in ['nominal_wall_reduction_percent','nominal_speedup','phase_time_reduction_percent','varied_wall_reduction_percent','all_complete','nonsettling_sequence_checks_pass']},indent=2))
