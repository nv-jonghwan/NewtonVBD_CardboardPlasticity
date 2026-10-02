"""Topology-aware scalar and shape comparison of completed 4mm experiments."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pxr import Usd,UsdGeom
from cardboard import ROOT

p=argparse.ArgumentParser();p.add_argument('--runs',nargs='+',required=True)
p.add_argument('--reference',default='graded48',help='Run directory basename used for matched-topology shape diagnostics')
p.add_argument('--output',required=True);a=p.parse_args()
out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
runs=[];summary={}
for name in a.runs:
    folder=ROOT/name;config=json.loads((folder/'run_config.json').read_text())
    performance=json.loads((folder/'performance.json').read_text())
    rows=list(csv.DictReader((folder/'state.csv').open()))
    values={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t']
    scene=Usd.Stage.Open(str(ROOT/config['scene']))
    mesh=scene.GetPrimAtPath('/World/Box/SimMesh')
    faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get())
    phase_wall={str(phase):float(np.sum(np.asarray(performance['frame_times'])[values['phase']==phase])) for phase in range(8)}
    samples={}
    for when in (5,7,9,12,16):
        i=int(np.argmin(abs(values['t']-when)))
        samples[str(when)]={k:float(values[k][i]) for k in ('t','actual_gap_m','min_z_m','max_speed_m_s','plastic_hinges','top_sag_mm')}
    summary[folder.name]=dict(wall_s=performance['wall_time_s'],error=performance['error'],
        vertices=len(rest),triangles=len(faces),iterations=config['iterations'],rom=config['rom'],
        rom_counts=performance['rom_counts'],phase_wall_s=phase_wall,samples=samples)
    if 'patch_counts' in performance:
        summary[folder.name]['patch_counts']=performance['patch_counts']
    if 'patch_local_fraction' in values:
        coverage={}
        for phase in range(8):
            select=values['phase']==phase;weight=values['patch_local_sweeps'][select]
            coverage[str(phase)]=float(np.average(values['patch_local_fraction'][select],weights=weight)) if weight.sum()>0 else None
        summary[folder.name]['local_patch_coverage_by_phase']=coverage
    if 'rom_full_elements' in values:
        full=values['rom_full_elements']>0
        summary[folder.name]['full_elements_first_t']=float(values['t'][full][0]) if full.any() else None
    if (folder/'validation.json').exists():summary[folder.name]['legacy_quality']=json.loads((folder/'validation.json').read_text())
    runs.append((folder.name,q,t,rest,faces,values,performance))
# Report same-topology comparisons only. Rigid alignment separates shape and pose.
reference=next((r for r in runs if r[0]==a.reference),None)
if reference:
    _,bq,bt,br,bf,*_=reference
    for name,q,t,rest,faces,*_ in runs:
        if q.shape!=bq.shape or not np.array_equal(rest,br) or not np.array_equal(faces,bf) or not np.allclose(t,bt):continue
        centered=q-q.mean(1,keepdims=True);base=bq-bq.mean(1,keepdims=True)
        u,_,vt=np.linalg.svd(np.einsum('tni,tnj->tij',centered,base))
        u[:,:,-1]*=np.linalg.det(u@vt)[:,None]
        rms=np.sqrt(np.mean(np.sum((centered@(u@vt)-base)**2,axis=2),axis=1))*1000
        summary[name]['aligned_shape_rms_mm_vs_'+a.reference]={str(w):float(rms[np.argmin(abs(t-w))]) for w in (5,7,9,12,16)}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,axes=plt.subplots(1,3,figsize=(14,4),layout='constrained')
for name,_,_,_,_,v,perf in runs:
    for ax,key,scale in zip(axes,('actual_gap_m','max_speed_m_s','active_wall_s'),(1000,1000,1)):
        ax.plot(v['t'],v[key]*scale,label=name)
for ax,title in zip(axes,('Jaw gap (mm)','Maximum vertex speed (mm/s)','Compute time (shared GPUs, s)')):
    ax.set(title=title,xlabel='Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=7)
axes[1].set_yscale('symlog',linthresh=1.)
fig.savefig(out/'metrics.png',dpi=150);plt.close(fig)
fig=plt.figure(figsize=(12,3.6*len(runs)),layout='constrained')
for row,(name,q,t,rest,faces,*_) in enumerate(runs):
    for col,when in enumerate((7,12,16)):
        ax=fig.add_subplot(len(runs),3,row*3+col+1,projection='3d')
        pts=q[np.argmin(abs(t-when))].copy();pts-=pts.mean(0)
        ax.add_collection3d(Poly3DCollection(pts[faces],facecolors='#bd925f',edgecolors='#6f593f',linewidths=.1,alpha=1.))
        ax.set(xlim=(-.2,.2),ylim=(-.2,.2),zlim=(-.2,.2),title=f'{name}: {when}s')
        ax.set_box_aspect((1,1,1));ax.view_init(24,-58);ax.set_axis_off()
fig.savefig(out/'shapes.png',dpi=130);plt.close(fig)
print(json.dumps({k:{f:v[f] for f in ('wall_s','vertices','triangles','iterations','rom_counts')} for k,v in summary.items()},indent=2))
