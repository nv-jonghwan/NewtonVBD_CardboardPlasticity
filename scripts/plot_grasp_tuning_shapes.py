"""Scientific physical-mesh comparison; translation removed, fixed view/scale."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from pxr import Usd, UsdGeom
from cardboard import ROOT

stage=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_board10_primitive.usda'))
faces=np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/SimMesh')).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
fig=plt.figure(figsize=(13,6),layout='constrained')
light=np.array([.4,-.5,1.]);light/=np.linalg.norm(light)
for row,(name,path) in enumerate([('Before','outputs/vbd_optimized/guarded_b'),('Tuned','outputs/grasp_tuning/balanced/run')]):
    z=np.load(ROOT/path/'trajectory.npz')
    for col,t in enumerate([5,7,8,12]):
        q=z['points'][np.argmin(abs(z['t']-t))].astype(float);q-=q.mean(0)
        triangles=q[faces]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        shade=.35+.65*np.abs(normals@light)
        colors=shade[:,None]*np.array([.8,.60,.31])[None,:]
        ax=fig.add_subplot(2,4,row*4+col+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(triangles,facecolors=colors,edgecolors='none',rasterized=True))
        ax.set(xlim=(-.16,.16),ylim=(-.12,.12),zlim=(-.12,.12),title=f'{name}: {t} s')
        ax.set_box_aspect((.32,.24,.24));ax.view_init(elev=26,azim=-58);ax.set_proj_type('ortho');ax.set_axis_off()
fig.suptitle('Physical mesh: before grasp / grasp end / lifting / crush end (same view and scale)')
fig.savefig(ROOT/'outputs/grasp_tuning/shape_comparison.png',dpi=170)
print('Saved outputs/grasp_tuning/shape_comparison.png')
