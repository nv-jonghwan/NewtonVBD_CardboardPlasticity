import unittest
import numpy as np
import warp as wp
from cardboard.plasticity import return_map_crease


class CreaseMaterialTest(unittest.TestCase):
    def step(self, theta, dual=.01, state=(0.,0.,0.), enabled=1, damage_rate=4., residual=.18, hardening=.01):
        q=np.array([[0,1,0],[0,-np.cos(theta),-np.sin(theta)],[0,0,0],[1,0,0]],np.float32)
        with wp.ScopedDevice('cuda:1'):
            output=[wp.zeros(1,dtype=float) for _ in range(4)]
            rest=wp.zeros(1,dtype=float);props=wp.array([[1/dual,.006/dual]],dtype=float)
            wp.launch(return_map_crease,dim=1,inputs=[wp.array(q,dtype=wp.vec3),wp.array([[0,1,2,3]],dtype=int),
                wp.array([1.],dtype=float),wp.zeros(1),wp.array([dual],dtype=float),wp.array([1/dual],dtype=float),
                4.,hardening,damage_rate,residual,enabled,.006,*[wp.array([v],dtype=float) for v in state],*output,rest,props,.02])
            return np.array([v.numpy()[0] for v in output]),props.numpy()[0]

    def test_intact_response_preserved_below_yield(self):
        state,props=self.step(.02)
        np.testing.assert_array_equal(state,np.zeros(4))
        np.testing.assert_allclose(props,[100.,.6],rtol=1e-6)

    def test_same_curvature_history_has_same_damage_across_element_widths(self):
        histories=[]
        for width in [.005,.01,.02]:
            state=np.zeros(4);history=[]
            for curvature in [2.,6.,12.,30.,20.,-10.]:
                state,props=self.step(curvature*width,width,state[:3])
                history.append([state[0]/width,state[1]/width,state[2]/width,state[3],props[0]*width])
            histories.append(history)
        np.testing.assert_allclose(histories[0],histories[1],rtol=2e-5,atol=2e-6)
        np.testing.assert_allclose(histories[0],histories[2],rtol=2e-5,atol=2e-6)

    def test_fold_localizes_by_softening_without_erasing_permanent_history(self):
        state=np.zeros(4);moments=[];damage=[];work=[]
        for theta in [.04,.10,.30,.60]:
            state,props=self.step(theta,state=state[:3])
            moments.append(props[0]*(theta-state[0]));damage.append(state[3]);work.append(state[2])
        self.assertLess(moments[-1],.5*moments[0])
        self.assertTrue(np.all(np.diff(damage)>=0));self.assertTrue(np.all(np.diff(work)>=0))
        self.assertGreaterEqual(props[0],17.999)
        unloaded,_=self.step(float(state[0]),state=state[:3])
        np.testing.assert_allclose(unloaded,state,rtol=1e-5,atol=1e-6)

    def test_damage_energy_and_plastic_work_match_scalar_update(self):
        theta=.1;k0=100.;y=4.;H=1.
        dg=(k0*theta-y)/(k0+H);damage=1-np.exp(-4*dg*.02/.01)
        expected_work=y*dg+.5*damage*(k0*(theta-dg)**2+H*dg**2)
        state,props=self.step(theta)
        np.testing.assert_allclose(state,[dg,dg,expected_work,damage],rtol=1e-5,atol=1e-6)
        np.testing.assert_allclose(props,[k0*(1-damage),.006*k0*(1-damage)],rtol=1e-5)

    def test_localized_profile_leaves_intact_material_and_retains_fold(self):
        intact,props=self.step(.02,damage_rate=8.,residual=.08)
        np.testing.assert_array_equal(intact,np.zeros(4))
        np.testing.assert_allclose(props,[100.,.6],rtol=1e-6)
        state=np.zeros(4);history=[]
        for theta in [.04,.10,.30,.60,.80]:
            state,props=self.step(theta,state=state[:3],damage_rate=8.,residual=.08)
            history.append(state.copy())
        self.assertGreaterEqual(props[0],7.999)
        self.assertTrue(np.all(np.diff(np.array(history)[:,1:],axis=0)>=-1e-6))
        unloaded,_=self.step(float(state[0]),state=state[:3],damage_rate=8.,residual=.08)
        np.testing.assert_allclose(unloaded,state,rtol=1e-5,atol=1e-6)

    def test_retained_stiffness_preserves_fold_at_equilibrium_and_allows_reload(self):
        state,props=self.step(.6,residual=.45)
        self.assertGreater(state[0],.1)
        self.assertGreaterEqual(props[0],44.999)
        held=state.copy()
        # Unloading to the plastic rest angle must not accumulate time creep.
        for _ in range(20):
            state,_=self.step(float(held[0]),state=state[:3],residual=.45)
        np.testing.assert_allclose(state,held,rtol=1e-5,atol=1e-6)
        reloaded,_=self.step(float(held[0]+.15),state=state[:3],residual=.45)
        self.assertGreater(reloaded[0],held[0])
        self.assertGreater(reloaded[2],held[2])

    def test_hardening_candidates_match_return_mapping_and_keep_permanent_angle(self):
        for hardening in [.1,.2]:
            theta=.3;k0=100.;H=hardening*k0;y=4.
            dg=(k0*theta-y)/(k0+H)
            damage=min(.82,1-np.exp(-4*dg*.02/.01))
            work=y*dg+.5*damage*(k0*(theta-dg)**2+H*dg*dg)
            state,props=self.step(theta,hardening=hardening)
            np.testing.assert_allclose(state,[dg,dg,work,damage],rtol=1e-5,atol=1e-6)
            unloaded,_=self.step(float(state[0]),state=state[:3],hardening=hardening)
            np.testing.assert_allclose(unloaded,state,rtol=1e-5,atol=1e-6)


if __name__=='__main__':unittest.main()
