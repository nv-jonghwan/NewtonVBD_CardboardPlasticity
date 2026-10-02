"""Build a reviewable report dataset and plots from the recorded VBD experiments."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from cardboard import ROOT

out=ROOT/'outputs/vbd_profile'
profile=json.loads((out/'full/profile.json').read_text())
config=json.loads((out/'full/run_config.json').read_text())
frames=[f for f in profile['frames'] if not f['detailed_sample'] and f['mode_age']>=2 and f['gpu_graph_ms'] is not None]
wall=sum(f['wall_ms'] for f in frames);gpu=sum(f['gpu_graph_ms'] for f in frames)
samples=[]
for sample in profile['samples']:
    groups={}
    for kernel in sample['first_substep_kernels']:
        category=kernel['category']
        # Post-process geometry query names that the initial classifier put in other.
        if 'colliding' in kernel['name'] or 'collision_detection' in kernel['name']:category='self_contact_detection'
        groups[category]=groups.get(category,0.)+kernel['first_substep_ms']
    samples.append(dict(t=sample['t'],mode=sample['mode'],categories_ms=groups,
                        percentages={k:v/sum(groups.values())*100 for k,v in groups.items()},
                        contacts=sample['contacts'],frame_launch_calls=sample['frame_launch_calls'],
                        display_median_ms=sample['display_ms']['median']))
micro=[];checks=0
for folder in ['launch_bench','contact_parallel','contact_parallel_unloaded']:
    data=json.loads((out/folder/'benchmark.json').read_text())
    for probe in data['probes']:
        for name in sorted({k['name'] for k in probe['kernels']}):
            kernels=[k for k in probe['kernels'] if k['name']==name]
            labels=[v['label'] for v in kernels[0]['variants']]
            totals={label:sum(next(v['median_us'] for v in k['variants'] if v['label']==label) for k in kernels) for label in labels}
            all_passed=all(v['passed'] for k in kernels for v in k['variants'])
            checks+=sum(len(k['variants']) for k in kernels)
            micro.append(dict(folder=folder,t=probe['t'],name=name,median_sum_us=totals,
                              speedups={k:totals[labels[0]]/v for k,v in totals.items()},passed=all_passed,
                              max_global_normalized_difference=max(e for k in kernels for v in k['variants'] for e in v['max_relative_to_global_scale'])))
analysis=dict(complete=profile['error'] is None and abs(profile['duration_s']-16)<1e-8,
              steady_frames=len(frames),steady_sample_wall_s=wall/1000,gpu_graph_fraction=gpu/wall,
              supported_time_fraction=sum(f['wall_ms'] for f in frames if f['mode']=='supported')/wall,
              host_only_ideal_speedup=wall/gpu,phase_summary=profile['phase_summary'],samples=samples,microbench=micro,
              numerical_comparisons=checks,all_microbench_equal=all(m['passed'] for m in micro),
              downloads_per_frame=sorted({f['downloads'] for f in frames}),download_bytes_per_frame=sorted({f['download_bytes'] for f in frames}),
              scene=config['scene'],production_solver_changed=False,
              limitations=['Detailed samples use CUDA events and first substep only, with timing perturbation.',
                          'Other project CUDA context appeared on GPU1 after profiling began; no exclusive GPU claim.',
                          'Nsight Systems/Compute not installed; achieved occupancy, bandwidth and stall counters unmeasured.',
                          'Microbench timings include private output resets and do not demonstrate end-to-end speedup.',
                          'Existing full-scenario settling gate remains failed; no production optimization promoted.'])
(out/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n')

fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
t=[f['t'] for f in frames]
axes[0].plot(t,[f['wall_ms'] for f in frames],label='Frame wall time',lw=1.4)
axes[0].plot(t,[f['gpu_graph_ms'] for f in frames],label='GPU physics graph',lw=1.2)
axes[0].axvspan(12.3,16,color='#ffd18b',alpha=.3,label='Supported refinement region (approx.)')
axes[0].set(xlabel='Simulation seconds',ylabel='ms / frame',title='VBD: steady frames; timing samples and mode startup excluded')
axes[0].legend();axes[0].grid(alpha=.2)
categories=sorted({k for sample in samples for k in sample['percentages']})
bottom=np.zeros(len(samples))
for category_index,category in enumerate(categories):
    values=np.array([sample['percentages'].get(category,0) for sample in samples])
    axes[1].bar(range(len(samples)),values,bottom=bottom,label=category.replace('_',' '),color=plt.get_cmap('tab20')(category_index));bottom+=values
axes[1].set_xticks(range(len(samples)),[f"{s['t']:.1f}s\n{s['mode']}" for s in samples])
axes[1].set(ylabel='% of instrumented kernel time',title='First-substep attribution; event overhead included')
axes[1].legend(loc='upper left',bbox_to_anchor=(1,1),fontsize=8)
fig.savefig(out/'phase_profile.png',dpi=170);fig.savefig(out/'phase_profile.svg')

fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
for record in sorted([m for m in micro if m['folder'].startswith('contact_parallel')],key=lambda x:x['t']):
    labels=['original4','workers8','workers16','workers32','workers64']
    ax.plot([4,8,16,32,64],[record['speedups'][label] for label in labels],marker='o',label=f"t={record['t']:.1f}s")
ax.axhline(1,color='gray',ls='--');ax.set_xticks([4,8,16,32,64])
ax.set(xlabel='Contact workers per rigid body',ylabel='Measured kernel batch speedup',
       title='Rigid-side contact accumulation: loaded gains, unloaded penalty')
ax.legend();ax.grid(alpha=.2)
fig.savefig(out/'contact_workers.png',dpi=170);fig.savefig(out/'contact_workers.svg')
print(json.dumps({k:analysis[k] for k in ['steady_frames','gpu_graph_fraction','supported_time_fraction','host_only_ideal_speedup','numerical_comparisons','all_microbench_equal','downloads_per_frame','download_bytes_per_frame']},indent=2))
