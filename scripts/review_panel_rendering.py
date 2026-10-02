"""Compare two display interpolants with exactly the same solver points."""
import argparse,json
import numpy as np
from pxr import Usd,UsdGeom
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cardboard import ROOT
from cardboard.surface import PanelSurface

p=argparse.ArgumentParser(__doc__);p.add_argument('--run',required=True);p.add_argument('--output',required=True);a=p.parse_args()
folder=ROOT/a.run;out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
cfg=json.loads((folder/'run_config.json').read_text());stage=Usd.Stage.Open(str(ROOT/cfg['scene']))
mesh=stage.GetPrimAtPath('/World/Box/SimMesh');render=stage.GetPrimAtPath('/World/Box/RenderMesh')
rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get())
faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
bindings=np.asarray(render.GetAttribute('cardboard:bindingIndices').Get())
weights=np.asarray(render.GetAttribute('cardboard:bindingWeights').Get())
offsets=np.asarray(render.GetAttribute('cardboard:bindingOffsets').Get())
rf=np.asarray(UsdGeom.Mesh(render).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
surfaces=[PanelSurface(rest,faces,bindings,weights,offsets,80,40,True),PanelSurface(rest,faces,bindings,weights,offsets,35,20,True)]
z=np.load(folder/'trajectory.npz');translation=(z['points'][0]-rest).mean(0)
fig=plt.figure(figsize=(10,10),layout='constrained');differences={}
for row,when in enumerate((9,12)):
    q=z['points'][np.argmin(abs(z['t']-when))].astype(float)-translation;original=q.copy()
    centered=q-q.mean(0);base=rest-rest.mean(0)
    u,_,vt=np.linalg.svd(centered.T@base);u[:,-1]*=np.linalg.det(u@vt);rotation=u@vt
    results=[surface.evaluate(q) for surface in surfaces]
    assert np.array_equal(q,original),'Rendering edited physics points'
    differences[str(when)]={'same_solver_points':True,'max_display_change_mm':float(np.linalg.norm(results[0][0]-results[1][0],axis=1).max()*1000)}
    for col,(points,normals) in enumerate(results):
        pts=(points-q.mean(0))@rotation;n=normals@rotation
        light=np.array([-.3,-.5,1.]);light/=np.linalg.norm(light)
        intensity=.35+.65*np.clip(n[rf].mean(1)@light,0,1)
        colors=np.array([.72,.50,.28])[None,:]*intensity[:,None]
        ax=fig.add_subplot(2,2,row*2+col+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(pts[rf],facecolors=colors,edgecolors='none',linewidths=0))
        ax.set(xlim=(-.18,.18),ylim=(-.18,.18),zlim=(-.17,.17),title=f'{["Previous smoothing","Fold-preserving smoothing"][col]}: {when}s')
        ax.set_box_aspect((1,1,1));ax.view_init(28,-55);ax.set_axis_off()
fig.savefig(out/'render_comparison.png',dpi=130);plt.close(fig)
report={'solver_vertices':len(rest),'solver_triangles':len(faces),'render_vertices':len(bindings),'render_triangles':len(rf),'samples':differences,
        'qualification':'Same solver trajectory; PN display interpolation only. Dense display cannot recover folds absent from physical mesh.'}
(out/'render_review.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
