import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
import newton
from cardboard.geometry import shell_grid
from cardboard.block_solver import TranslationBlockVBD
from cardboard.compact_contact import CompactContactMixin
from cardboard.rom_solver import ReducedCorrection,AdaptiveROMVBD
from cardboard.representative_elements import RepresentativeElements,spatial_quadrature
from cardboard.local_vbd import LocalPatch,dilate_patch,mark_soft_contacts,mark_self_contacts,PAIR_THREADS


class LocalROMTest(unittest.TestCase):
    def test_guarded_fallback_cannot_be_restricted_to_local_patch(self):
        from unittest.mock import patch
        s=object.__new__(AdaptiveROMVBD);s.iterations=24;s.patch_full_every=8;s.model=SimpleNamespace(device='cuda:0')
        s.local_patch=SimpleNamespace(stats=wp.zeros(4,dtype=wp.int64,device=s.device))
        seen=[]
        with patch.object(TranslationBlockVBD,'_solve_particle_iteration',lambda solver,*args:seen.append(solver.local_sweep)):
            s._vbd_correction(None,None,None,.001,1)
            s._vbd_correction(None,None,None,.001,1,force_full=True)
            s._vbd_correction(None,None,None,.001,23)
        self.assertEqual(seen,[True,False,False]);self.assertFalse(s.local_sweep)
        self.assertEqual(s.local_patch.stats.numpy()[3],2)

    def model(self):
        q,tri,_=shell_grid([.2,.16,.14],n=4)
        b=newton.ModelBuilder()
        b.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),vertices=q,
            indices=tri.ravel(),density=.72,tri_ke=14000.,tri_ka=23000.,tri_kd=.01,edge_ke=.1,edge_kd=.001)
        b.color(include_bending=True)
        m=b.finalize(device='cuda:0');s=TranslationBlockVBD(m,iterations=2,particle_enable_self_contact=False,rigid_compliant_alm=False)
        a=m.state();a.cardboard=SimpleNamespace(accumulated_angle=wp.zeros(m.edge_count,device=m.device))
        patch=LocalPatch(m);s.small_bend_config=(16.,.75,14.75);s.small_bend_dual=patch.dual
        q=q.copy();q[:,2]+=.001*np.sin(q[:,0]*13)*np.cos(q[:,1]*11)
        a.particle_q.assign(q);s.particle_q_prev.assign(q);s.inertia.assign(q)
        return m,s,a

    def test_positive_quadrature_preserves_group_measure(self):
        rng=np.random.default_rng(13);x=rng.normal(size=(80,3));measure=rng.uniform(.2,2.,80);group=np.arange(80)%4
        group[-7:]=-1
        ids,w=spatial_quadrature(x,measure,group,.25)
        self.assertTrue((w>0).all());self.assertEqual(len(ids),len(set(ids)))
        self.assertTrue(set(range(73,80)).issubset(ids))
        for g in np.unique(group):
            self.assertAlmostEqual(float(np.sum(w[group[ids]==g]*measure[ids[group[ids]==g]])),float(measure[group==g].sum()),places=5)
        for f in (0,1.01,np.nan):
            with self.assertRaises(ValueError):spatial_quadrature(x,measure,group,f)

    def test_all_elements_match_full_residual_and_sampling_balances_force(self):
        m,s,a=self.model();n=m.particle_count
        u=np.tile(np.eye(3),(n,1))/np.sqrt(n)
        extra=np.arange(n*3,dtype=float);extra-=u@(u.T@extra);extra/=np.linalg.norm(extra)
        basis=np.c_[u,extra].reshape(n,3,4).transpose(0,2,1).astype(np.float32)
        rom=ReducedCorrection(m,basis)
        rom.residual(s,a.particle_q,a,None,.001)
        f=rom.force.numpy();h=rom.hessian.numpy()
        rom.representatives=RepresentativeElements(m,1.)
        rom.residual(s,a.particle_q,a,None,.001)
        np.testing.assert_allclose(rom.force.numpy(),f,rtol=3e-5,atol=2e-5)
        np.testing.assert_allclose(rom.hessian.numpy(),h,rtol=2e-5,atol=.003)
        rom.representatives=RepresentativeElements(m,.25)
        with wp.ScopedCapture(device=m.device) as capture:rom.residual(s,a.particle_q,a,None,.001)
        wp.capture_launch(capture.graph)
        force=rom.force.numpy()
        # Complete element corner forces cancel, even with nonuniform weights.
        self.assertLess(np.linalg.norm(force.sum(0)),1e-4*max(np.linalg.norm(force),1.))
        self.assertTrue(np.isfinite(rom.local.numpy()).all())
        sampled=rom.force.numpy().copy()
        adaptive=RepresentativeElements(m,.25,full_after_yield=True);rom.representatives=adaptive
        rom.residual(s,a.particle_q,a,None,.001)
        with wp.ScopedCapture(device=m.device) as capture:rom.residual(s,a.particle_q,a,None,.001)
        wp.capture_launch(capture.graph)
        self.assertEqual(adaptive.full.numpy()[0],0)
        np.testing.assert_allclose(rom.force.numpy(),sampled,rtol=4e-5,atol=2e-5)
        a.cardboard.accumulated_angle.fill_(.01)
        wp.capture_launch(capture.graph)
        adaptive_force=rom.force.numpy().copy();self.assertEqual(adaptive.full.numpy()[0],1)
        rom.representatives=None;rom.residual(s,a.particle_q,a,None,.001)
        np.testing.assert_allclose(adaptive_force,rom.force.numpy(),rtol=4e-5,atol=2e-5)

    def test_local_sweep_all_active_matches_full_and_inactive_preserves_displacement(self):
        m,s,a=self.model();out=m.state();q=a.particle_q.numpy().copy();n=len(q)
        s.pos_prev_collision_detection.assign(q);s.particle_displacements.zero_()
        # Apply a nonuniform external inertia target so the sweep has real work.
        target=q.copy();target[:,2]+=.0004*np.sin(q[:,0]*17);s.inertia.assign(target)
        CompactContactMixin._solve_particle_iteration(s,a,out,None,.001,0)
        expected=a.particle_q.numpy().copy()
        mask=wp.ones(n,dtype=int,device=m.device)
        s.local_patch=SimpleNamespace(mask=mask,colors=m.particle_colors,refresh=lambda *args:None)
        s.local_sweep=True;a.particle_q.assign(q);s.particle_displacements.zero_()
        CompactContactMixin._solve_particle_iteration(s,a,out,None,.001,0)
        np.testing.assert_allclose(a.particle_q.numpy(),expected,atol=1e-8)
        active=np.zeros(n,np.int32);active[:n//3]=1;mask.assign(active)
        a.particle_q.assign(q);s.particle_displacements.zero_()
        CompactContactMixin._solve_particle_iteration(s,a,out,None,.001,0)
        np.testing.assert_array_equal(a.particle_q.numpy()[active==0],q[active==0])
        self.assertGreater(np.linalg.norm(a.particle_q.numpy()[active!=0]-q[active!=0]),1e-6)

    def test_graph_dilation_respects_one_ring_and_soft_contact_distance(self):
        device='cuda:0';arr=lambda x,dtype:wp.array(x,dtype=dtype,device=device)
        neighbors=arr([[1,-1],[0,2],[1,3],[2,-1]],int)
        old=arr([1,0,0,0],int);new=wp.zeros_like(old)
        with wp.ScopedCapture(device=device) as capture:
            wp.launch(dilate_patch,dim=4,inputs=[neighbors,old,new],device=device)
        wp.capture_launch(capture.graph)
        np.testing.assert_array_equal(new.numpy(),[1,1,0,0])
        mask=wp.zeros(4,dtype=int,device=device)
        inputs=[arr([[0,1,-1],[2,-1,-1],[3,-1,-1]],wp.vec3i),arr([3],int),
                arr([[.5,.5,0],[1,0,0],[1,0,0]],wp.vec3),arr([[0,0,.001],[0,0,.001],[0,0,.1],[0,0,.001]],wp.vec3),
                arr([.002]*4,float),arr([0]*3,int),arr([-1],int),wp.empty(0,dtype=wp.transform,device=device),
                arr([[0,0,0]]*3,wp.vec3),arr([[0,0,1]]*3,wp.vec3),arr([0.],float),.002,mask]
        wp.launch(mark_soft_contacts,dim=3,inputs=inputs,device=device)
        np.testing.assert_array_equal(mask.numpy(),[1,1,0,1])

    def test_parallel_patch_matches_damped_frozen_position_block_update(self):
        m,s,a=self.model();n=m.particle_count;q=a.particle_q.numpy().copy()
        s.small_bend_memory_curvature=15.
        s.crease_friction_curvature=15.
        a.cardboard.accumulated_angle.assign(.5*15*s.small_bend_dual.numpy())
        target=q.copy();target[:,2]+=.0004*np.cos(q[:,0]*13);s.inertia.assign(target)
        u=np.eye(n*3,4,dtype=np.float32).reshape(n,3,4).transpose(0,2,1)
        rom=ReducedCorrection(m,u);rom.residual(s,a.particle_q,a,None,.001)
        delta=rom.local.numpy()
        patch=LocalPatch(m,solver='jacobi',relaxation=.5)
        active=np.zeros(n,np.int32);active[:n//2]=1
        patch.mask.assign(active);patch.parallel_colors.assign(np.where(active,0,-1).astype(np.int32))
        patch.refresh=lambda *args:None;s.local_patch=patch;s.local_sweep=True
        s.pos_prev_collision_detection.assign(q);s.particle_displacements.zero_();out=m.state()
        CompactContactMixin._solve_particle_iteration(s,a,out,None,.001,0)
        expected=q+.5*delta*active[:,None]
        np.testing.assert_allclose(a.particle_q.numpy(),expected,atol=2e-8)

    def test_memory_force_shared_by_representatives_and_full_residual(self):
        m,s,a=self.model();n=m.particle_count
        s.small_bend_memory_curvature=15.
        s.crease_friction_curvature=15.
        a.cardboard.accumulated_angle.assign(np.linspace(0,1.2,m.edge_count).astype(np.float32)*15*s.small_bend_dual.numpy())
        basis=np.eye(n*3,4,dtype=np.float32).reshape(n,3,4).transpose(0,2,1)
        rom=ReducedCorrection(m,basis);rom.residual(s,a.particle_q,a,None,.001)
        force=rom.force.numpy();hessian=rom.hessian.numpy()
        rom.representatives=RepresentativeElements(m,1.)
        rom.residual(s,a.particle_q,a,None,.001)
        np.testing.assert_allclose(rom.force.numpy(),force,rtol=3e-5,atol=1e-5)
        # Atomic element scatter and vertex gathers sum in different orders;
        # near-zero off-diagonals can differ by ~1e-3 beside 1e5 diagonals.
        actual=rom.hessian.numpy()
        np.testing.assert_allclose(actual,hessian,rtol=2e-5,atol=.003)
        relative=np.linalg.norm(actual-hessian,axis=(1,2))/np.linalg.norm(hessian,axis=(1,2))
        self.assertLess(float(relative.max()),2e-6)

    def test_self_contact_seeds_include_both_sides_and_exclude_far_candidates(self):
        device='cuda:0';arr=lambda x,dtype:wp.array(x,dtype=dtype,device=device)
        q=arr([[0,0,0],[1,0,0],[0,1,0],[.2,.2,.001],[.2,.2,.1],
               [.1,.1,.002],[.8,.1,.002],[.1,.1,.5],[.8,.1,.5]],wp.vec3)
        tri=arr([[0,1,2]],int)
        edges=arr([[-1,-1,0,1],[-1,-1,5,6],[-1,-1,7,8]],int)
        ep=arr([[0,1],[0,2]],wp.vec2i);vp=arr([[3,0],[4,0]],wp.vec2i)
        count=arr([2,2],int);mask=wp.zeros(9,dtype=int,device=device)
        # Edge pair0 is .1m apart and must not activate5/6 at a5mm threshold;
        # only the close vertex/triangle pair activates its four endpoints.
        with wp.ScopedCapture(device=device) as capture:
            wp.launch(mark_self_contacts,dim=PAIR_THREADS,inputs=[ep,vp,count,edges,tri,q,.005,1e-6,mask],device=device)
        wp.capture_launch(capture.graph)
        np.testing.assert_array_equal(mask.numpy(),[1,1,1,1,0,0,0,0,0])
        coords=q.numpy();coords[5:7,1]=.001;q.assign(coords);mask.zero_()
        wp.capture_launch(capture.graph)
        np.testing.assert_array_equal(mask.numpy(),[1,1,1,1,0,1,1,0,0])


if __name__=='__main__':unittest.main()
