"""Report geometric shrink/curl after release; never infer it from speed alone."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+');a=p.parse_args()
for directory in a.directories:
    out=Path(directory)
    with np.load(out/'trajectory.npz') as data:q=data['points'].astype(float);f=data['faces'];t=data['t']
    tri=q[:,f]
    cross=np.cross(tri[:,:,1]-tri[:,:,0],tri[:,:,2]-tri[:,:,0])
    area=np.linalg.norm(cross,axis=-1).sum(axis=1)*.5
    volume=np.einsum('tfi,tfi->t',tri[:,:,0],np.cross(tri[:,:,1],tri[:,:,2]))/6
    # Face pairs within the same original flat panel (exclude manufactured box seams).
    normal0=cross[0]/np.linalg.norm(cross[0],axis=-1,keepdims=True)
    edges={}
    for face,ids in enumerate(f):
        for i,j in [(0,1),(1,2),(2,0)]:edges.setdefault(tuple(sorted((ids[i],ids[j]))),[]).append(face)
    pairs=np.array([v for v in edges.values() if len(v)==2 and normal0[v[0]]@normal0[v[1]]>.99])
    # Translation/rotation-independent volume and surface area, not axis-aligned bounds.
    result={'samples':{},'late_volume_change_fraction':float((volume[-1]-volume[np.searchsorted(t,14.)])/volume[np.searchsorted(t,14.)]),'late_area_change_fraction':float((area[-1]-area[np.searchsorted(t,14.)])/area[np.searchsorted(t,14.)])}
    for when in [1.,9.,12.,13.,14.,15.,16.]:
        i=min(np.searchsorted(t,when),len(t)-1)
        normals=cross[i]/np.maximum(np.linalg.norm(cross[i],axis=-1,keepdims=True),1e-12)
        angles=np.rad2deg(np.arccos(np.clip((normals[pairs[:,0]]*normals[pairs[:,1]]).sum(-1),-1,1)))
        curved=np.unique(pairs[angles>3.].ravel())
        triangle_area=np.linalg.norm(cross[i],axis=-1)
        flat_fraction=1-triangle_area[curved].sum()/triangle_area.sum()
        rest=q[0]-q[0].mean(axis=0);current=q[i]-q[i].mean(axis=0)
        u,_,vt=np.linalg.svd(rest.T@current);rotation=u@vt
        if np.linalg.det(rotation)<0:u[:,-1]*=-1;rotation=u@vt
        residual=np.linalg.norm(current-rest@rotation,axis=1)
        result['samples'][str(when)]={'t':float(t[i]),'shape_change_rms_after_rigid_fit_m':float(np.sqrt(np.mean(residual**2))),'locally_flat_area_fraction_3deg':float(flat_fraction),'volume_m3':float(volume[i]),'area_m2':float(area[i]),'volume_fraction_of_initial':float(volume[i]/volume[0]),'area_fraction_of_initial':float(area[i]/area[0])}
    (out/'shape_retention.json').write_text(json.dumps(result,indent=2));print(directory,json.dumps(result))
