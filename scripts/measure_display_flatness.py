"""Compare identical source display triangles, independent of physics resolution."""
import argparse,json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom
from cardboard import ROOT
from cardboard.surface import PanelSurface
from cardboard.usd_utils import register
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+');a=p.parse_args();register()
for folder in a.directories:
 out=ROOT/folder;config=json.loads((out/'run_config.json').read_text());stage=Usd.Stage.Open(str(ROOT/config['scene']));m=stage.GetPrimAtPath('/World/Box/SimMesh');r=stage.GetPrimAtPath('/World/Box/RenderMesh');attr=lambda n:r.GetAttribute('cardboard:'+n).Get()
 rest=np.asarray(m.GetAttribute('cardboard:restPoints').Get());faces=np.asarray(UsdGeom.Mesh(m).GetFaceVertexIndicesAttr().Get()).reshape(-1,3);rf=np.asarray(UsdGeom.Mesh(r).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
 surf=PanelSurface(rest,faces,attr('bindingIndices'),attr('bindingWeights'),attr('bindingOffsets'),attr('visualCreaseAngleDegrees'),attr('visualCreaseTransitionDegrees'),bool(attr('visualSmoothThickness')))
 ref,_=surf.evaluate(rest);_,ids=np.unique(np.round(ref,7),axis=0,return_inverse=True)
 def geometry(q):
  tri=q[rf];cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);length=np.linalg.norm(cross,axis=1);return cross/np.maximum(length[:,None],1e-14),length*.5
 normal0,_=geometry(ref);edges={}
 for i,tri in enumerate(ids[rf]):
  for j,k in [(0,1),(1,2),(2,0)]:edges.setdefault(tuple(sorted((tri[j],tri[k]))),[]).append(i)
 pairs=np.array([v for v in edges.values() if len(v)==2 and normal0[v[0]]@normal0[v[1]]>.999])
 center=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(m).ExtractTranslation())
 with np.load(out/'trajectory.npz') as d:
  result={'display_triangles':len(rf),'comparable_flat_rest_edges':len(pairs),'method':'Original graphics topology welded at1e-7m; factory seams excluded; same3deg neighboring display-triangle normal threshold for all physics meshes. Visual geometry metric, not a solver convergence test.','samples':{}}
  for t in [9.,12.,16.]:
   i=min(np.searchsorted(d['t'],t),len(d['t'])-1);q,_=surf.evaluate(d['points'][i]-center);normal,area=geometry(q);angles=np.rad2deg(np.arccos(np.clip(np.sum(normal[pairs[:,0]]*normal[pairs[:,1]],axis=1),-1,1)));curved=np.unique(pairs[angles>3].ravel());result['samples'][str(t)]={'flat_area_fraction':float(1-area[curved].sum()/area.sum())}
 (out/'display_flatness.json').write_text(json.dumps(result,indent=2));print(folder,json.dumps(result))
