"""Compare panel morphology as well as sag; these diagnostics are not acceptance."""
import argparse,csv,json
import textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pxr import Usd,UsdGeom
from cardboard import ROOT

p=argparse.ArgumentParser(__doc__)
p.add_argument('directories',nargs='+')
p.add_argument('--output',default='outputs/conservative_tuning')
p.add_argument('--azim',type=float,default=-58.)
a=p.parse_args();out=ROOT/a.output;out.mkdir(exist_ok=True,parents=True)
stage=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'))
prim=stage.GetPrimAtPath('/World/Box/SimMesh');faces=np.asarray(UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
rest=np.asarray(prim.GetAttribute('cardboard:restPoints').Get())
reference=np.load(ROOT/'outputs/early_top_fold/canonical/diagnostic.npz')
def normals(q):
 tri=q[faces];n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);return n/np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12)
edge_faces={}
for i,tri in enumerate(faces):
 for j in range(3):edge_faces.setdefault(tuple(sorted((tri[j],tri[(j+1)%3]))),[]).append(i)
adj=np.asarray([v for v in edge_faces.values() if len(v)==2]);nr=normals(rest)
# Exclude the box's authored 90-degree seams; count folds inside the six panels.
adj=adj[np.sum(nr[adj[:,0]]*nr[adj[:,1]],axis=1)>.999]
times=[5,7,8,9,12,16];light=np.array([.4,-.5,1.]);light/=np.linalg.norm(light)
fig=plt.figure(figsize=(18,3*len(a.directories)),layout='constrained');result={}
base=None
for row,directory in enumerate(a.directories):
 folder=ROOT/directory;z=np.load(folder/'trajectory.npz');rows=list(csv.DictReader((folder/'state.csv').open()));samples={}
 if base is None:base=z
 for col,t in enumerate(times):
  q=z['points'][np.argmin(abs(z['t']-t))].astype(float);r=min(rows,key=lambda r:abs(float(r['t'])-t))
  c=q[reference['rim']].mean(0);normal=np.linalg.svd(q[reference['rim']]-c)[2][-1];normal*=1 if normal[2]>=0 else -1
  n=normals(q);angles=np.degrees(np.arccos(np.clip(np.sum(n[adj[:,0]]*n[adj[:,1]],axis=1),-1,1)))
  qr=base['points'][np.argmin(abs(base['t']-t))].astype(float);qr-=qr.mean(0);qc=q-q.mean(0)
  u,_,vt=np.linalg.svd(qc.T@qr);d=np.eye(3);d[-1,-1]=np.linalg.det(u@vt);rot=u@d@vt
  samples[str(t)]={k:float(r[k]) for k in ['actual_gap_m','plastic_hinges','plastic_work_J','max_speed_m_s','min_z_m']}
  samples[str(t)].update(top_sag_mm=float((c-q[reference['center']].mean(0))@normal*1000),panel_edges_over_15deg=int((angles>15).sum()),panel_angle_p95_deg=float(np.percentile(angles,95)),aligned_rms_to_first_mm=float(np.sqrt(np.mean(np.sum((qc@rot-qr)**2,axis=1)))*1000))
  ax=fig.add_subplot(len(a.directories),len(times),row*len(times)+col+1,projection='3d')
  # No per-frame rotation alignment in visualization; only remove translation.
  shade=.35+.65*np.abs(n@light);colors=shade[:,None]*np.array([.8,.60,.31])[None,:]
  ax.add_collection3d(Poly3DCollection(qc[faces],facecolors=colors,edgecolors='none',rasterized=True))
  label=folder.parent.name if folder.name=="run" else folder.name
  title='\n'.join(textwrap.wrap(label.replace('_',' '),width=22))+f'\n{t}s'
  ax.set(xlim=(-.16,.16),ylim=(-.12,.12),zlim=(-.12,.12))
  ax.text2D(.5,.98,title,transform=ax.transAxes,ha='center',va='top',fontsize=9)
  ax.set_box_aspect((.32,.24,.24));ax.view_init(elev=26,azim=a.azim);ax.set_proj_type('ortho');ax.set_axis_off()
 result[directory]=dict(samples=samples,finite_geometry=bool(np.isfinite(z['points']).all()))
 perf=folder/'performance.json'
 if perf.exists():result[directory]['wall_s']=json.loads(perf.read_text())['wall_time_s']
fig.suptitle('Physical mesh, same view and scale; panel fold counts exclude original box seams')
fig.savefig(out/'shape_comparison.png',dpi=150);plt.close(fig)
(out/'morphology.json').write_text(json.dumps(result,indent=2))
for directory,r in result.items():
 print(directory)
 for t,s in r['samples'].items():print(t,{k:round(v,3) for k,v in s.items()})
