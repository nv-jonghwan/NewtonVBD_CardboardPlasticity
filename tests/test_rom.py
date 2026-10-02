import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
from cardboard.rom_basis import fit_increment_basis, mesh_digest
from cardboard.rom_solver import ReducedCorrection, project_system, solve_reduced, decide_trial, accept_trial, use_full_sweep, PARTS, WIDTH


@wp.kernel
def quadratic_residual(q:wp.array[wp.vec3],target:wp.array[wp.vec3],
                       force:wp.array[wp.vec3],hessian:wp.array[wp.mat33],local:wp.array[wp.vec3]):
    i=wp.tid()
    force[i]=target[i]-q[i]
    hessian[i]=wp.identity(3,wp.float32)
    local[i]=force[i]


@wp.kernel
def trial_position(anchor:wp.array[wp.vec3],displacement:wp.array[wp.vec3],
                   result:wp.array[wp.vec3],offset:float):
    i=wp.tid()
    result[i]=anchor[i]+displacement[i]+wp.vec3(offset,0.0,0.0)


class ROMTest(unittest.TestCase):
    def test_rom_residual_uses_pristine_and_yielded_bending_law(self):
        import newton
        from cardboard.block_solver import TranslationBlockVBD
        device='cuda:0'
        builder=newton.ModelBuilder()
        rest=np.array([[0.,1.,0.],[0.,-1.,0.],[-.5,0.,0.],[.5,0.,0.]],np.float32)
        builder.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),
            vertices=rest,indices=[0,2,3,1,3,2],density=1.,tri_ke=0.,tri_ka=0.,tri_kd=0.,edge_ke=1.,edge_kd=0.)
        builder.color(include_bending=True)
        model=builder.finalize(device=device)
        solver=TranslationBlockVBD(model,iterations=2,rigid_compliant_alm=False)
        solver.particle_enable_self_contact=False
        q=rest.copy();q[1,2]=.001
        state=model.state();state.particle_q.assign(q)
        solver.particle_q_prev.assign(q);solver.inertia.assign(q)
        state.cardboard=SimpleNamespace(accumulated_angle=wp.zeros(model.edge_count,device=device))
        basis=np.eye(12,dtype=np.float32)[:,:4].reshape(4,3,4).transpose(0,2,1)
        rom=ReducedCorrection(model,basis)
        rom.residual(solver,state.particle_q,state,None,.01)
        original_force=rom.force.numpy();original_h=rom.hessian.numpy()
        self.assertGreater(np.linalg.norm(original_force),1e-5)
        inertia=model.particle_mass.numpy()[:,None,None]/.01**2*np.eye(3)
        solver.small_bend_dual=wp.ones(model.edge_count,device=device)
        for scale,yielded in [(1.,False),(16.,False),(16.,True)]:
            solver.small_bend_config=(scale,.3,5.9)
            state.cardboard.accumulated_angle.fill_(.01 if yielded else 0.)
            rom.residual(solver,state.particle_q,state,None,.01)
            factor=1. if yielded else scale
            np.testing.assert_allclose(rom.force.numpy(),factor*original_force,rtol=3e-6,atol=1e-7)
            np.testing.assert_allclose(rom.hessian.numpy()-inertia,factor*(original_h-inertia),rtol=2e-3,atol=.003)

    def test_alternating_schedule_and_final_full_correction(self):
        self.assertEqual([use_full_sweep(i,i==7,2,True) for i in range(8)], [False,True]*4)
        self.assertEqual([use_full_sweep(i,i==6,2,True) for i in range(7)], [False,True,False,True,False,True,True])
        self.assertEqual([use_full_sweep(i,i==7,4) for i in range(8)], [True,False,False,False,True,False,False,True])

    def test_captured_accept_and_both_fallback_paths(self):
        # Exercise real nested CUDA conditionals and rollback, independently of
        # the nonlinear force evaluator. A rejected trial must leave the original
        # state and collision-anchor displacement available to the full solver.
        device='cuda:0';n=16
        rng=np.random.default_rng(311)
        u,_=np.linalg.qr(rng.normal(size=(n*3,5)))
        basis=u[:,:4].reshape(n,3,4).transpose(0,2,1).astype(np.float32)
        initial=np.zeros((n,3),np.float32)
        anchor=np.full((n,3),-.002,np.float32)
        for branch in ['accept','representation_reject','residual_reject','fast_accept','fast_nonfinite',
                       'deferred_accept','deferred_representation_reject','deferred_nonfinite']:
            with self.subTest(branch=branch):
                deferred=branch.startswith('deferred')
                representation_reject='representation_reject' in branch
                direction=u[:,4] if representation_reject else u[:,0]
                target=(direction*1.e-5).reshape(n,3).astype(np.float32)
                state=SimpleNamespace(particle_q=wp.array(initial,dtype=wp.vec3,device=device))
                out=SimpleNamespace(particle_q=wp.zeros(n,dtype=wp.vec3,device=device))
                solver=SimpleNamespace(device=device,
                    pos_prev_collision_detection=wp.array(anchor,dtype=wp.vec3,device=device),
                    particle_displacements=wp.zeros(n,dtype=wp.vec3,device=device))
                rom=ReducedCorrection(SimpleNamespace(particle_count=n,device=device),basis,
                    check_residual=not (branch.startswith('fast') or deferred),defer_fallback=deferred)
                target_gpu=wp.array(target,dtype=wp.vec3,device=device)
                def residual(s,q,state,c,dt):
                    wp.launch(quadratic_residual,dim=n,inputs=[q,target_gpu,rom.force,rom.hessian,rom.local],device=device)
                rom.residual=residual
                def truncate(result):
                    wp.launch(trial_position,dim=n,inputs=[solver.pos_prev_collision_detection,
                        solver.particle_displacements,result,float('nan') if branch.endswith('nonfinite') else (.001 if branch in ('residual_reject','fast_accept') else 0.)],device=device)
                solver._penetration_free_truncation=truncate
                fallback_state=wp.full(n,wp.vec3(7.),dtype=wp.vec3,device=device)
                fallback_displacement=wp.empty_like(fallback_state)
                def fallback():
                    wp.copy(fallback_state,state.particle_q)
                    wp.copy(fallback_displacement,solver.particle_displacements)
                # Compile before capture, then restore state/counters.
                rom.apply(solver,state,out,None,.01,fallback)
                state.particle_q.assign(initial);rom.counters.zero_()
                with wp.ScopedCapture(device=device) as capture:
                    rom.apply(solver,state,out,None,.01,fallback)
                wp.capture_launch(capture.graph);wp.synchronize_device(device)
                if branch in ('accept','fast_accept','deferred_accept'):
                    expected=target+np.array([.001,0.,0.],np.float32) if branch=='fast_accept' else target
                    np.testing.assert_allclose(state.particle_q.numpy(),expected,atol=3e-10)
                    np.testing.assert_array_equal(out.particle_q.numpy(),state.particle_q.numpy())
                    np.testing.assert_array_equal(rom.counters.numpy(),[1,1,0,0])
                else:
                    np.testing.assert_array_equal(state.particle_q.numpy(),initial)
                    if deferred:
                        np.testing.assert_array_equal(fallback_state.numpy(),np.full_like(initial,7.))
                        np.testing.assert_allclose(solver.particle_displacements.numpy(),initial-anchor,atol=1e-10)
                        np.testing.assert_array_equal(out.particle_q.numpy(),initial)
                    else:
                        np.testing.assert_array_equal(fallback_state.numpy(),initial)
                        np.testing.assert_allclose(fallback_displacement.numpy(),initial-anchor,atol=1e-10)
                    expected=[1,0,1,0] if representation_reject else [1,0,0,1]
                    np.testing.assert_array_equal(rom.counters.numpy(),expected)

    def test_pod_orthogonality_and_translation(self):
        rng=np.random.default_rng(73)
        q=rng.normal(size=(22,17,3))*.01
        u,captured=fit_increment_basis([q],8)
        flat=u.transpose(0,2,1).reshape(-1,8).astype(float)
        np.testing.assert_allclose(flat.T@flat,np.eye(8),atol=1e-7)
        translation=np.tile([.01,-.03,.02],17)
        np.testing.assert_allclose(flat@(flat.T@translation),translation,atol=1e-8)
        self.assertGreater(captured,0);self.assertLess(captured,1)

    def test_digest_rejects_geometry_or_topology_changes(self):
        q=np.zeros((4,3));f=np.array([[0,1,2],[1,2,3]])
        baseline=mesh_digest(q,f);q[0,0]=.001
        self.assertNotEqual(baseline,mesh_digest(q,f))
        self.assertNotEqual(mesh_digest(q,f),mesh_digest(q,f[:,::-1]))

    def test_reduced_system_against_independent_dense_float64(self):
        rng=np.random.default_rng(13);n=83;r=8;device='cuda:0'
        u,_=np.linalg.qr(rng.normal(size=(n*3,r)))
        basis=u.reshape(n,3,r).transpose(0,2,1).astype(np.float32)
        a=rng.normal(size=(n,3,3));h=(a@a.transpose(0,2,1)+np.eye(3)*3).astype(np.float32)
        f=rng.normal(size=(n,3)).astype(np.float32);local=np.linalg.solve(h,f[...,None])[...,0]
        hp=wp.zeros((r,r,PARTS),dtype=wp.float64,device=device)
        fp=wp.zeros((r,PARTS),dtype=wp.float64,device=device);dp=wp.zeros_like(fp)
        matrix=wp.zeros((r,r),dtype=wp.float64,device=device);rhs=wp.zeros(r,dtype=wp.float64,device=device)
        dz=wp.zeros(r,device=device);pd=wp.zeros_like(dz);valid=wp.zeros(1,dtype=int,device=device)
        wp.launch(project_system,dim=(r,r,PARTS*WIDTH),block_dim=WIDTH,inputs=[wp.array(basis,dtype=wp.vec3,device=device),wp.array(f,dtype=wp.vec3,device=device),wp.array(h,dtype=wp.mat33,device=device),wp.array(local,dtype=wp.vec3,device=device),hp,fp,dp],device=device)
        expected_h=np.einsum('nri,nij,nsj->rs',basis.astype(float),h,basis.astype(float))
        expected_f=np.einsum('nri,ni->r',basis.astype(float),f)
        np.testing.assert_allclose(hp.numpy().sum(2),expected_h,rtol=2e-6,atol=2e-7)
        np.testing.assert_allclose(fp.numpy().sum(1),expected_f,rtol=2e-6,atol=2e-7)
        wp.launch(solve_reduced,dim=1,inputs=[hp,fp,dp,matrix,rhs,dz,pd,valid],device=device)
        self.assertEqual(valid.numpy()[0],1)
        np.testing.assert_allclose(dz.numpy(),np.linalg.solve(expected_h,expected_f),rtol=3e-6,atol=2e-7)
        np.testing.assert_allclose(pd.numpy(),u.T@local.ravel(),rtol=3e-6,atol=2e-7)

    def test_unrepresented_and_nonfinite_corrections_rejected(self):
        device='cuda:0';valid=wp.array([1],dtype=int,device=device);allowed=wp.zeros(1,dtype=int,device=device)
        scale=wp.zeros(1,device=device);counts=wp.zeros(4,dtype=int,device=device)
        for values in [[1.,.9,.001,0.,0.],[1.,0.,.001,1.,0.]]:
            stats=wp.array(values,dtype=wp.float64,device=device)
            wp.launch(decide_trial,dim=1,inputs=[stats,valid,.35,10,allowed,scale,counts],device=device)
            self.assertEqual(allowed.numpy()[0],0)
        self.assertEqual(counts.numpy()[2],2)

    def test_residual_increase_rejected_and_decrease_accepted(self):
        device='cuda:0';allowed=wp.zeros(1,dtype=int,device=device);counts=wp.zeros(4,dtype=int,device=device)
        for new,expected in [(1.1,0),(.9,1)]:
            stats=wp.array([1.,0.,0.,0.,new],dtype=wp.float64,device=device)
            wp.launch(accept_trial,dim=1,inputs=[stats,allowed,counts],device=device)
            self.assertEqual(allowed.numpy()[0],expected)

if __name__=='__main__':unittest.main()
