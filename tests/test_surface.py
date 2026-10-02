import unittest
import numpy as np
from cardboard.geometry import shell_grid
from cardboard.surface import PanelSurface

class SurfaceTest(unittest.TestCase):
    def make(self):
        p,f,_=shell_grid([.28,.21,.2],4)
        # Three interior samples and three edge midpoint samples per triangle.
        w=np.tile([[.6,.2,.2],[.2,.6,.2],[.2,.2,.6],[.5,.5,0],[0,.5,.5],[.5,0,.5]],(len(f),1))
        inds=np.repeat(f,6,axis=0)
        return p,f,inds,w,PanelSurface(p,f,inds,w,np.zeros((len(w),3)))
    def test_flat_panels_exact_and_normals_unit(self):
        p,f,inds,w,s=self.make();q,n=s.evaluate(p)
        np.testing.assert_allclose(q,(p[inds]*w[:,:,None]).sum(1),atol=2e-8)
        np.testing.assert_allclose(np.linalg.norm(n,axis=1),1,atol=1e-6)
    def test_rigid_transform_invariant(self):
        p,f,inds,w,s=self.make()
        offsets=np.tile([.001,-.0005,.0002],(len(w),1))
        s=PanelSurface(p,f,inds,w,offsets)
        angle=.7;r=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        q,n=s.evaluate(p);q2,n2=s.evaluate(p@r.T+[.4,-.1,.2])
        np.testing.assert_allclose(q2,q@r.T+[.4,-.1,.2],atol=5e-8)
        np.testing.assert_allclose(n2,n@r.T,atol=1e-6)
    def test_smooth_thickness_rest_and_rigid_transform(self):
        p,f,inds,w,_=self.make()
        offsets=np.tile([.001,-.0005,.0075],(len(w),1))
        s=PanelSurface(p,f,inds,w,offsets,100.,40.,smooth_thickness=True)
        original=p.copy();q,n=s.evaluate(p)
        np.testing.assert_allclose(q,(p[inds]*w[:,:,None]).sum(1)+offsets,atol=3e-8)
        np.testing.assert_array_equal(p,original)
        angle=.7;r=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        q2,n2=s.evaluate(p@r.T+[.4,-.1,.2])
        np.testing.assert_allclose(q2,q@r.T+[.4,-.1,.2],atol=5e-8)
        np.testing.assert_allclose(n2,n@r.T,atol=1e-6)
    def test_thick_skin_has_no_step_at_smooth_shared_edge(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]])
        weights=np.array([[.5,0,.5],[.5,.5,0]])
        offsets=np.tile([0,0,.0075],(2,1))
        q=rest.copy();angle=np.deg2rad(25);q[3]=[-np.cos(angle),0,np.sin(angle)]
        old=PanelSurface(rest,faces,faces,weights,offsets,100,40)
        new=PanelSurface(rest,faces,faces,weights,offsets,100,40,smooth_thickness=True)
        old_points,_=old.evaluate(q);points,normals=new.evaluate(q)
        self.assertGreater(np.linalg.norm(old_points[0]-old_points[1]),.003)
        np.testing.assert_allclose(points[0],points[1],atol=2e-8)
        np.testing.assert_allclose(normals[0],normals[1],atol=1e-6)
    def test_smooth_thickness_preserves_deep_fold(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]])
        weights=np.array([[.3,.3,.4],[.3,.3,.4]])
        # The physical crease stays sharp; only its outer thickness skin rounds.
        offsets=np.zeros((2,3))
        q=rest.copy();angle=np.deg2rad(130);q[3]=[-np.cos(angle),0,np.sin(angle)]
        old=PanelSurface(rest,faces,faces,weights,offsets,100,40)
        new=PanelSurface(rest,faces,faces,weights,offsets,100,40,smooth_thickness=True)
        old_points,old_normals=old.evaluate(q);points,normals=new.evaluate(q)
        np.testing.assert_allclose(points,old_points,atol=2e-8)
        np.testing.assert_allclose(normals,old_normals,atol=1e-6)
        self.assertLess(float(normals[0]@normals[1]),-.49)
    def test_thickness_positions_meet_across_deep_fold(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]])
        weights=np.array([[.5,0,.5],[.5,.5,0]])
        offsets=np.tile([0,0,.0075],(2,1))
        q=rest.copy();angle=np.deg2rad(130);q[3]=[-np.cos(angle),0,np.sin(angle)]
        surface=PanelSurface(rest,faces,faces,weights,offsets,100,40,smooth_thickness=True)
        points,normals=surface.evaluate(q)
        np.testing.assert_allclose(points[0],points[1],atol=2e-8)
        # Position continuity must not erase the sharp shading crease.
        self.assertLess(float(normals[0]@normals[1]),-.49)
    def test_sharp_crease_stays_piecewise_planar(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]])
        weights=np.array([[.3,.3,.4],[.3,.3,.4]])
        surface=PanelSurface(rest,faces,faces,weights,np.zeros((2,3)))
        q=rest.copy();q[3]=[-.5,0,np.sqrt(.75)]
        actual,normals=surface.evaluate(q)
        np.testing.assert_allclose(actual,(q[faces]*weights[:,:,None]).sum(1),atol=3e-8)
        self.assertLess(float(normals[0]@normals[1]),.51)
    def test_no_visual_pop_at_crease_threshold(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]]);weights=np.array([[.3,.3,.4],[.3,.3,.4]])
        surface=PanelSurface(rest,faces,faces,weights,np.zeros((2,3)))
        results=[]
        for angle in [44.999,45.001]:
            q=rest.copy();r=np.deg2rad(angle);q[3]=[-np.cos(r),0,np.sin(r)]
            results.append(surface.evaluate(q))
        self.assertLess(float(np.max(abs(results[0][0]-results[1][0]))),2e-4)
        self.assertLess(float(np.max(abs(results[0][1]-results[1][1]))),.001)
    def test_curved_shared_edges_no_cracks(self):
        p,f,inds,w,s=self.make();deformed=p.copy();deformed[:,2]+=.8*deformed[:,0]**2
        q,n=s.evaluate(deformed);edges={}
        for i,face in enumerate(f):
            for j,(a,b) in enumerate([(0,1),(1,2),(2,0)]):
                key=tuple(sorted((face[a],face[b])));point=q[i*6+3+j]
                if key in edges:np.testing.assert_allclose(point,edges[key],atol=2e-8)
                else:edges[key]=point

    def test_flat_profile_preserves_moderate_fold_and_solver_vertices(self):
        rest=np.array([[0,0,0],[1,0,0],[0,1,0],[-1,0,0]],float)
        faces=np.array([[0,1,2],[0,2,3]])
        # Dense samples include exact physical vertices and a shared edge.
        bary=np.array([[i/12,j/12,1-(i+j)/12] for i in range(13) for j in range(13-i)])
        indices=np.repeat(faces,len(bary),axis=0);weights=np.tile(bary,(2,1))
        q=rest.copy();angle=np.deg2rad(50);q[3]=[-np.cos(angle),0,np.sin(angle)]
        offsets=np.zeros((len(weights),3))
        old=PanelSurface(rest,faces,indices,weights,offsets,80,40,True)
        new=PanelSurface(rest,faces,indices,weights,offsets,35,20,True)
        linear=(q[indices]*weights[:,:,None]).sum(1)
        actual,normals=new.evaluate(q)
        np.testing.assert_allclose(actual,linear,atol=6e-8)
        self.assertGreater(float(np.max(abs(old.evaluate(q)[0]-linear))),.01)
        # Interpolation never edits the solver state.
        saved=q.copy();new.evaluate(q);np.testing.assert_array_equal(q,saved)
if __name__=='__main__':unittest.main()
