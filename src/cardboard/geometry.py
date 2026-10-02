import numpy as np
from scipy.spatial import cKDTree
from pxr import Usd, UsdGeom

def shell_grid(size, n=12):
    """Closed manifold mid-surface; shared vertices at box seams, outward winding."""
    points=[]; faces=[]; directions=[]; ids={}
    half=np.asarray(size)*.5
    for axis in range(3):
        u=(axis+1)%3; v=(axis+2)%3
        for sign in [-1,1]:
            grid={}
            for i in range(n+1):
                for j in range(n+1):
                    key=[0,0,0]; key[axis]=sign*n;key[u]=2*i-n;key[v]=2*j-n;key=tuple(key)
                    if key not in ids:ids[key]=len(points);points.append(np.array(key)/n*half)
                    grid[i,j]=ids[key]
            for i in range(n):
                for j in range(n):
                    a,b,c,d=[grid[k] for k in [(i,j),(i+1,j),(i+1,j+1),(i,j+1)]]
                    tris=[[a,b,c],[a,c,d]] if (i+j)%2==0 else [[a,b,d],[b,c,d]]
                    direction=np.zeros(3);direction[u]=1
                    for tri in tris:
                        faces.append(tri if sign>0 else tri[::-1]);directions.append(direction)
    return np.array(points,dtype=np.float32),np.array(faces,dtype=np.int32),np.array(directions,dtype=np.float32)


def graded_shell_grid(size, counts=(18, 14, 14), graded=True):
    """Conforming box shell with axis-dependent resolution and smooth grading.

    Shared axis coordinates keep all panel seams watertight. Extra samples
    cover the central fold band (X), the narrow fingertip footprint (Y), and
    the upper gripping region (Z). This is a static rest-space mesh, not
    remeshing of plastic history during simulation.
    """
    size = np.asarray(size, dtype=float)
    counts = np.asarray(counts)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('size must contain three finite positive lengths')
    if counts.shape != (3,) or np.any(counts < 2) or np.any(counts != counts.astype(int)):
        raise ValueError('counts must contain three integer subdivisions >= 2')
    counts = counts.astype(int)
    axes = []
    # Density fields are deliberately smooth to avoid abrupt element ratios.
    bands = [(0., .30, 1.5), (0., .30, 2.), (.65, .35, 1.5)]
    for count, (center, width, amplitude) in zip(counts, bands):
        x = np.linspace(-1., 1., 4097)
        density = 1. + amplitude * np.exp(-((x-center)/width)**2) if graded else np.ones_like(x)
        cdf = np.r_[0., np.cumsum((density[:-1]+density[1:])*.5*np.diff(x))]
        axes.append(np.interp(np.linspace(0., cdf[-1], count+1), cdf, x))
    points, faces, directions, ids = [], [], [], {}
    for axis in range(3):
        u, v = (axis+1) % 3, (axis+2) % 3
        for sign in (-1, 1):
            grid = {}
            for i in range(counts[u]+1):
                for j in range(counts[v]+1):
                    key = [0, 0, 0]
                    key[axis], key[u], key[v] = (0 if sign < 0 else counts[axis]), i, j
                    key = tuple(key)
                    if key not in ids:
                        ids[key] = len(points)
                        points.append([axes[k][key[k]]*size[k]*.5 for k in range(3)])
                    grid[i, j] = ids[key]
            for i in range(counts[u]):
                for j in range(counts[v]):
                    a, b, c, d = [grid[k] for k in ((i,j), (i+1,j), (i+1,j+1), (i,j+1))]
                    tris = ((a,b,c), (a,c,d)) if (i+j) % 2 == 0 else ((a,b,d), (b,c,d))
                    direction = np.eye(3)[u]
                    for tri in tris:
                        faces.append(tri if sign > 0 else tri[::-1])
                        directions.append(direction)
    return np.asarray(points, np.float32), np.asarray(faces, np.int32), np.asarray(directions, np.float32)

def source_render(path, scale=.4, levels=2):
    stage=Usd.Stage.Open(str(path),load=Usd.Stage.LoadNone)
    # Load only the model payload, not the unrelated thumbnail rig.
    stage.Load('/RootNode')
    mesh=next(UsdGeom.Mesh(p) for p in Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))
    m=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    vertices=np.array([m.Transform(p) for p in mesh.GetPointsAttr().Get()])
    lo=vertices.min(0);hi=vertices.max(0);center=(lo+hi)*.5
    vertices=(vertices-center)*scale
    uv=np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar('st').ComputeFlattened())
    counts=np.array(mesh.GetFaceVertexCountsAttr().Get());idx=np.array(mesh.GetFaceVertexIndicesAttr().Get())
    tris=[];uvtris=[];offset=0
    for count in counts:
        for j in range(1,count-1):
            corners=offset+np.array([0,j,j+1]);tris.append(vertices[idx[corners]]);uvtris.append(uv[corners])
        offset+=count
    p=np.array(tris);uv=np.array(uvtris)
    for _ in range(levels):
        def sub(a):
            x,y,z=a[:,0],a[:,1],a[:,2];xy=(x+y)*.5;yz=(y+z)*.5;zx=(z+x)*.5
            return np.concatenate([np.stack(t,axis=1) for t in [(x,xy,zx),(xy,y,yz),(zx,yz,z),(xy,yz,zx)]])
        p=sub(p);uv=sub(uv)
    return p.reshape(-1,3).astype(np.float32),np.arange(p.size//3,dtype=np.int32).reshape(-1,3),uv.reshape(-1,2).astype(np.float32),(hi-lo)*scale

def bind(render, points, faces):
    """Closest candidates with clamped barycentrics; preserve small source bevel offsets."""
    tri=points[faces]; centers=tri.mean(1)
    _,cand=cKDTree(centers).query(render,k=min(24,len(faces)))
    a=tri[cand,0];e0=tri[cand,1]-a;e1=tri[cand,2]-a;q=render[:,None,:]-a
    d00=(e0*e0).sum(-1);d01=(e0*e1).sum(-1);d11=(e1*e1).sum(-1)
    d20=(q*e0).sum(-1);d21=(q*e1).sum(-1);den=d00*d11-d01*d01
    b=(d11*d20-d01*d21)/den;c=(d00*d21-d01*d20)/den
    w=np.maximum(np.stack([1-b-c,b,c],-1),0);w/=w.sum(-1,keepdims=True)
    nearest=(tri[cand]*w[:,:,:,None]).sum(2)
    k=((nearest-render[:,None,:])**2).sum(-1).argmin(1);r=np.arange(len(render));inds=faces[cand[r,k]];weights=w[r,k]
    offsets=render-(points[inds]*weights[:,:,None]).sum(1)
    return inds.astype(np.int32),weights.astype(np.float32),offsets.astype(np.float32)

def skin(points,inds,weights,offsets):
    return (points[inds]*weights[:,:,None]).sum(1)+offsets


def refine_render(points, faces, uv, max_edge=.012):
    """Conforming edge splits of source triangles, preserving per-corner UVs.

    Every shared overlong geometric edge is bisected on both sides, including
    UV seams. Red/green splits avoid hanging nodes on adjacent triangles.
    """
    triangles=points[faces];tex=uv[faces]
    for _ in range(16):
        lengths=np.linalg.norm(np.roll(triangles,-1,axis=1)-triangles,axis=2)
        masks=lengths>max_edge*(1+1e-6)
        if not masks.any():
            p=triangles.reshape(-1,3).astype(np.float32)
            return p,np.arange(len(p),dtype=np.int32).reshape(-1,3),tex.reshape(-1,2).astype(np.float32)
        newp=[];newuv=[]
        for vertices,coords,mask in zip(triangles,tex,masks):
            count=int(mask.sum())
            if count==0:newp.append(vertices);newuv.append(coords);continue
            if count==1:rotation=int(np.flatnonzero(mask)[0])
            elif count==2:rotation=(int(np.flatnonzero(~mask)[0])+1)%3
            else:rotation=0
            v=np.roll(vertices,-rotation,axis=0);st=np.roll(coords,-rotation,axis=0)
            def split(x):
                a,b,c=x;ab=(a+b)*.5;bc=(b+c)*.5;ca=(c+a)*.5
                if count==1:return [[a,ab,c],[ab,b,c]]
                if count==2:return [[b,bc,ab],[a,ab,c],[ab,bc,c]]
                return [[a,ab,ca],[ab,b,bc],[ca,bc,c],[ab,bc,ca]]
            newp.extend(split(v));newuv.extend(split(st))
        triangles=np.array(newp);tex=np.array(newuv)
    raise RuntimeError('Render subdivision exceeded iteration limit')
