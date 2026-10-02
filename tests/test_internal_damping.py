import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
from cardboard.internal_damping import InternalDamping


class InternalDampingTest(unittest.TestCase):
    def setUp(self):
        self.device=wp.get_device('cuda:1');rng=np.random.default_rng(42)
        self.q=(rng.uniform(-.1,.1,(385,3))+[.3,0,.8]).astype(np.float32)
        self.mass=rng.uniform(.001,.005,len(self.q)).astype(np.float32)
        self.v=rng.normal(0,.15,self.q.shape).astype(np.float32)
        self.model=SimpleNamespace(device=self.device,particle_mass=wp.array(self.mass,device=self.device),particle_count=len(self.q))
    def state(self,v):
        return SimpleNamespace(particle_q=wp.array(self.q,dtype=wp.vec3,device=self.device),particle_qd=wp.array(v,dtype=wp.vec3,device=self.device))
    def reference(self,v):
        q=self.q.astype(float);m=self.mass.astype(float);v=np.asarray(v,float)
        center=np.average(q,axis=0,weights=m);r=q-center;linear=np.average(v,axis=0,weights=m)
        inertia=np.einsum('n,n,ij->ij',m,(r*r).sum(1),np.eye(3))-np.einsum('n,ni,nj->ij',m,r,r)
        angular=np.linalg.solve(inertia,(m[:,None]*np.cross(r,v)).sum(0))
        return linear+np.cross(angular,r)
    def test_momenta_and_energy_with_independent_float64_reference(self):
        state=self.state(self.v);damp=InternalDamping(self.model,[.3,0,.8],60.)
        dt=.002;rigid=self.reference(self.v);expected=rigid+np.exp(-60*dt)*(self.v-rigid)
        damp.apply(state,dt);actual=state.particle_qd.numpy()
        np.testing.assert_allclose(actual,expected,atol=2e-7,rtol=1e-6)
        for values in [lambda v:(self.mass[:,None]*v).sum(0),lambda v:(self.mass[:,None]*np.cross(self.q-[.3,0,.8],v)).sum(0)]:
            np.testing.assert_allclose(values(actual),values(self.v),atol=1e-7,rtol=1e-5)
        before=np.sum(self.mass[:,None]*(self.v-rigid)**2);after=np.sum(self.mass[:,None]*(actual-rigid)**2)
        self.assertAlmostEqual(after/before,np.exp(-120*dt),places=6)
        np.testing.assert_array_equal(state.particle_q.numpy(),self.q)
    def test_translation_and_rotation_are_undamped(self):
        velocity=np.array([.2,-.1,.3])+np.cross([.5,.8,-.3],self.q-[.3,0,.8])
        state=self.state(velocity);damp=InternalDamping(self.model,[.3,0,.8],1000.)
        damp.apply(state,.01)
        np.testing.assert_allclose(state.particle_qd.numpy(),velocity,atol=3e-7)
    def test_rate_is_time_step_independent_for_fixed_geometry(self):
        a=self.state(self.v);b=self.state(self.v);damp=InternalDamping(self.model,[.3,0,.8],60.)
        damp.apply(a,.01)
        for _ in range(10):damp.apply(b,.001)
        np.testing.assert_allclose(a.particle_qd.numpy(),b.particle_qd.numpy(),atol=3e-7)
    def test_invalid_rate_rejected(self):
        for value in [-1.,np.nan,np.inf]:
            with self.assertRaises(ValueError):InternalDamping(self.model,[0,0,0],value)

    def test_supported_rate_override_is_local_and_energy_dissipative(self):
        state=self.state(self.v);damp=InternalDamping(self.model,[.3,0,.8],60.)
        rigid=self.reference(self.v);dt=1/1920
        damp.apply(state,dt,3000.)
        expected=rigid+np.exp(-3000*dt)*(self.v-rigid)
        np.testing.assert_allclose(state.particle_qd.numpy(),expected,atol=3e-7)
        self.assertEqual(damp.rate,60.)
        np.testing.assert_array_equal(state.particle_q.numpy(),self.q)

    def test_support_loss_damps_rigid_and_internal_modes_independently(self):
        state=self.state(self.v);damp=InternalDamping(self.model,[.3,0,.8],60.)
        rigid=self.reference(self.v);dt=.004
        damp.apply(state,dt,60.,20.)
        expected=np.exp(-20*dt)*rigid+np.exp(-60*dt)*(self.v-rigid)
        np.testing.assert_allclose(state.particle_qd.numpy(),expected,atol=3e-7)
        self.assertLess(np.sum(self.mass[:,None]*state.particle_qd.numpy()**2),np.sum(self.mass[:,None]*self.v**2))
        np.testing.assert_array_equal(state.particle_q.numpy(),self.q)
        for rate in [-1.,np.nan,np.inf]:
            with self.assertRaises(ValueError):damp.apply(state,dt,rigid_rate=rate)


if __name__=='__main__':unittest.main()
