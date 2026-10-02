"""Read-only post-load geometry/history diagnosis of an existing trajectory."""
import argparse,csv,json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from pxr import Usd,UsdGeom
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cardboard import ROOT

p=argparse.ArgumentParser(__doc__)
p.add_argument('--run',required=True);p.add_argument('--live-csv');p.add_argument('--output',required=True)
a=p.parse_args();folder=ROOT/a.run;out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
cfg=json.loads((folder/'run_config.json').read_text())
stage=Usd.Stage.Open(str(ROOT/cfg['scene']));mesh=stage.GetPrimAtPath('/World/Box/SimMesh')
rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get(),float)
faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
edges=np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get());edges=edges[(edges[:,:2]>=0).all(1)]
counts=Counter(tuple(sorted((int(f[i]),int(f[j])))) for f in faces for i,j in ((0,1),(1,2),(2,0)))
assert all(c==2 for c in counts.values()),'Volume proxy requires a closed shell'
z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t']
assert np.isfinite(q).all()

def geometry(points):
    tri=(points-points.mean(0))[faces]
    volume=abs(np.einsum('ij,ij->',tri[:,0],np.cross(tri[:,1],tri[:,2]))/6)
    area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1).sum()/2
    return np.array([volume,area])

# Numerical sanity check: translation and proper rotation cannot mimic collapse.
rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
np.testing.assert_allclose(geometry(q[-1]@rotation+[2.,3.,4.]),geometry(q[-1]),rtol=1e-10,atol=1e-12)
g=np.array([geometry(x) for x in q]);hull=np.array([ConvexHull(x).volume for x in q])
separation=[]
for axis in range(3):
    lo=np.isclose(rest[:,axis],rest[:,axis].min(),atol=1e-6)
    hi=np.isclose(rest[:,axis],rest[:,axis].max(),atol=1e-6)
    separation.append(np.linalg.norm(q[:,hi].mean(1)-q[:,lo].mean(1),axis=1))
separation=np.array(separation).T
def read_csv(path):
    rows=list(csv.DictReader(path.open()))
    return {key:np.array([float(row[key]) for row in rows]) for key in rows[0]}
v=read_csv(folder/'state.csv')
phase_ends=np.asarray(stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:phaseEnds').Get())
release=float(phase_ends[5])
def force_history(values):
    time=values['t'];force=np.maximum(values['left_contact_N'],values['right_contact_N'])
    after=time>=release-1e-8;loaded=np.flatnonzero(after&(force>.01))
    last=int(loaded[-1]) if len(loaded) else int(np.searchsorted(time,release))
    support=np.flatnonzero(after&(values['min_z_m']<=.653))
    samples={}
    for when in [x for x in (11,12,12.5,13,14,15,16,18,20) if x<=t[-1]+1e-6]:
        i=int(np.argmin(abs(time-when)))
        samples[str(when)]={k:float(values[k][i]) for k in ('t','actual_gap_m','left_contact_N','right_contact_N','plastic_hinges','plastic_work_J','max_plastic_angle_rad','min_z_m','max_speed_m_s','solver_mode')}
    return {'last_finger_force_over_0p01N_s':float(time[last]),
            'first_near_table_after_release_s':float(time[support[0]]) if len(support) else None,
            'samples':samples}
samples={}
for when in [x for x in (0,9,11,12,12.5,13,14,15,16,18,20) if x<=t[-1]+1e-6]:
    i=int(np.argmin(abs(t-when)))
    samples[str(when)]={'t':float(t[i]),'signed_shell_volume_magnitude_liters':float(g[i,0]*1000),
                       'convex_envelope_liters':float(hull[i]*1000),'surface_area_m2':float(g[i,1]),
                       'original_opposite_panel_centroid_separation_mm':(separation[i]*1000).tolist()}
changes={}
windows=[(11,12),(12,14),(12,16),(14,16)]
if t[-1]>=20-1e-6:windows.extend([(12,20),(16,20),(18,20)])
for start,end in windows:
    i=int(np.argmin(abs(t-start)));j=int(np.argmin(abs(t-end)))
    changes[f'{start}-{end}']={'shell_volume_change_percent':float((g[j,0]/g[i,0]-1)*100),
                              'convex_envelope_change_percent':float((hull[j]/hull[i]-1)*100),
                              'area_change_percent':float((g[j,1]/g[i,1]-1)*100),
                              'panel_separation_change_mm':((separation[j]-separation[i])*1000).tolist()}
x0,x1,x2,x3=q[:,edges].transpose(2,0,1,3)
unit=lambda x:x/np.maximum(np.linalg.norm(x,axis=-1,keepdims=True),1e-12)
n0=unit(np.cross(x2-x0,x3-x0));n1=unit(np.cross(x3-x1,x2-x1));e=unit(x3-x2)
angle=np.arctan2((np.cross(n0,n1)*e).sum(-1),(n0*n1).sum(-1))
material=np.load(folder/'final_material_state.npz')
result={'run':a.run,'release_command_start_s':release,'trajectory_samples':samples,'changes':changes,
        'replay_force_and_history':force_history(v),
        'live_force_and_history':force_history(read_csv(ROOT/a.live_csv)) if a.live_csv else None,
        'final_damage_quantiles':np.quantile(material['damage'],[.5,.9,1.]).tolist(),
        'final_fraction_hinges_damage_at_least_0p8':float(np.mean(material['damage']>=.8)),
        'final_max_plastic_angle_rad':float(abs(material['plastic_angle']).max()),
        'sampled_dihedral_jumps_above_pi':int((abs(np.diff(angle,axis=0))>np.pi).sum()),
        'checks':{'closed_shell':True,'finite_points':True,'rigid_invariant_geometry':True},
        'limitations':['Volume is a signed closed-shell geometry proxy; self-intersection may invalidate a literal cavity-volume interpretation.',
                       'Convex envelope and panel-centroid distances independently measure changing shape, not material density.',
                       'Force telemetry is sampled at frame ends; near-table height is an impact proxy, not a full table-force trace.',
                       'No substep history or controlled causal ablation; 30Hz angle trace cannot exclude all substep branch crossings.',
                       'Live run has CSV and final points only; detailed geometry curves describe the selected recorded replay.']}
(out/'report.json').write_text(json.dumps(result,indent=2))
with (out/'geometry.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['t','shell_volume_l','convex_envelope_l','surface_area_m2','panel_x_mm','panel_y_mm','panel_z_mm'])
    w.writerows(np.column_stack([t,g[:,0]*1000,hull*1000,g[:,1],separation*1000]))
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
axes[0,0].plot(v['t'],v['left_contact_N'],label='Left actual contact')
axes[0,0].plot(v['t'],v['right_contact_N'],label='Right actual contact');axes[0,0].set(ylabel='Force (N)',yscale='symlog')
axes[0,1].plot(t,g[:,0]/g[0,0]*100,label='Shell volume proxy')
axes[0,1].plot(t,hull/hull[0]*100,label='Convex envelope');axes[0,1].set(ylabel='% of initial geometry')
axes[1,0].plot(v['t'],v['plastic_hinges'],label='Hinges with plastic history');axes[1,0].set(ylabel='Hinge count')
for axis,label in enumerate('XYZ'):axes[1,1].plot(t,separation[:,axis]*1000,label=f'Original {label} panel separation')
axes[1,1].set(ylabel='Panel-centroid distance (mm)')
for ax in axes.flat:
    ax.axvline(release,color='black',linestyle='--',label='Start opening')
    ax.axvline(result['replay_force_and_history']['first_near_table_after_release_s'],color='gray',linestyle=':',label='Near table')
    ax.set(xlim=(9,float(t[-1])),xlabel='Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(out/'postload_timeline.png',dpi=150);plt.close(fig)
fig=plt.figure(figsize=(12,4),layout='constrained');base=q[0]-q[0].mean(0)
for col,when in enumerate((12,14,16)):
    pts=q[np.argmin(abs(t-when))].copy();pts-=pts.mean(0)
    u,_,vt=np.linalg.svd(pts.T@base);u[:,-1]*=np.linalg.det(u@vt);pts=pts@(u@vt)
    ax=fig.add_subplot(1,3,col+1,projection='3d')
    ax.add_collection3d(Poly3DCollection(pts[faces],facecolors='#bd925f',edgecolors='#705437',linewidths=.12))
    ax.set(xlim=(-.2,.2),ylim=(-.2,.2),zlim=(-.18,.18),title=f'{when}s: physical mesh, aligned')
    ax.set_box_aspect((1,1,.9));ax.view_init(26,-55);ax.set_axis_off()
fig.savefig(out/'physical_shapes.png',dpi=150);plt.close(fig)
print(json.dumps({'changes':changes,'replay_events':{k:v for k,v in result['replay_force_and_history'].items() if k!='samples'}},indent=2))
