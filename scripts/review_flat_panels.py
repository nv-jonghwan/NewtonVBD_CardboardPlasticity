"""Measure solver-panel flatness independently of display smoothing.

One-ring plane errors include transmitted loads and fold neighborhoods. They
are geometric diagnostics, not evidence that a region has zero contact force.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cardboard import ROOT

p=argparse.ArgumentParser(__doc__)
p.add_argument('--runs',nargs='+',required=True)
p.add_argument('--output',required=True)
a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
report={};plots=[]
for name in a.runs:
    folder=ROOT/name;config=json.loads((folder/'run_config.json').read_text())
    stage=Usd.Stage.Open(str(ROOT/config['scene']))
    mesh=stage.GetPrimAtPath('/World/Box/SimMesh')
    rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get(),float)
    faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    edges=np.asarray(mesh.GetAttribute('cardboard:hingeIndices').Get())
    reference=np.asarray(mesh.GetAttribute('cardboard:referenceAngles').Get())
    dual=np.asarray(mesh.GetAttribute('cardboard:dualWidths').Get())
    valid=(edges[:,:2]>=0).all(1)&(abs(reference)<1e-5)
    edges=edges[valid];dual=dual[valid];reference=reference[valid]
    # Separate original material panels at seams; fit within each panel only.
    tri=rest[faces];rn=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    panel=2*np.argmax(abs(rn),axis=1)+(rn.sum(1)>0)
    neighborhoods={}
    for f,k in zip(faces,panel):
        for v in f:neighborhoods.setdefault((int(k),int(v)),set()).update(f.tolist())
    rings=[np.array(sorted(ids)) for ids in neighborhoods.values() if len(ids)>=5]
    z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t']
    rows=list(csv.DictReader((folder/'state.csv').open()))
    samples={}
    for when in (5,7,9,12,16):
        pts=q[np.argmin(abs(t-when))]
        errors=[]
        for ring in rings:
            patch=pts[ring];singular=np.linalg.svd(patch-patch.mean(0),compute_uv=False)
            errors.append(singular[-1]/np.sqrt(len(patch))*1000)
        errors=np.asarray(errors)
        x0,x1,x2,x3=pts[edges].transpose(1,0,2)
        norm=lambda x:x/np.maximum(np.linalg.norm(x,axis=1,keepdims=True),1e-12)
        n0=norm(np.cross(x2-x0,x3-x0));n1=norm(np.cross(x3-x1,x2-x1));e=norm(x3-x2)
        angle=np.arctan2((np.cross(n0,n1)*e).sum(1),(n0*n1).sum(1))
        curvature=abs(angle-reference)/dual
        row=min(rows,key=lambda row:abs(float(row['t'])-when))
        samples[str(when)]={
            'ring_plane_rms_median_mm':float(np.median(errors)),
            'ring_plane_rms_p90_mm':float(np.quantile(errors,.9)),
            'rings_below_half_mm_fraction':float(np.mean(errors<.5)),
            'hinges_below_half_per_m_fraction':float(np.mean(curvature<.5)),
            'hinges_above_15_per_m_fraction':float(np.mean(curvature>15)),
            **{key:float(row[key]) for key in ('top_sag_mm','plastic_hinges','min_z_m','actual_gap_m','max_speed_m_s')},
        }
    performance=json.loads((folder/'performance.json').read_text())
    report[folder.name]={'samples':samples,'wall_s':performance['wall_time_s'],
                         'error':performance['error'],'small_bend':config.get('small_bend'),
                         'qualification':'Unweighted vertex one-ring plane fits within original panels; includes fold neighborhoods and all contact states. Not unloaded-area classification or calibrated material accuracy.'}
    plots.append((folder.name,q,t,faces))
(out/'summary.json').write_text(json.dumps(report,indent=2))
fig=plt.figure(figsize=(15,3.5*len(plots)),layout='constrained')
for row,(name,q,t,faces) in enumerate(plots):
    # Align to the initial mesh for comparing panel deformation, not world pose.
    base=q[0]-q[0].mean(0)
    for col,when in enumerate((5,7,9,12)):
        pts=q[np.argmin(abs(t-when))].copy();pts-=pts.mean(0)
        u,_,vt=np.linalg.svd(pts.T@base);u[:,-1]*=np.linalg.det(u@vt);pts=pts@(u@vt)
        ax=fig.add_subplot(len(plots),4,row*4+col+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(pts[faces],facecolors='#bd925f',edgecolors='#705437',linewidths=.14))
        ax.set(xlim=(-.2,.2),ylim=(-.2,.2),zlim=(-.18,.18),title=f'{name}: {when}s')
        ax.set_box_aspect((1,1,.9));ax.view_init(26,-55);ax.set_axis_off()
fig.savefig(out/'solver_shapes.png',dpi=140);plt.close(fig)
print(json.dumps(report,indent=2))
