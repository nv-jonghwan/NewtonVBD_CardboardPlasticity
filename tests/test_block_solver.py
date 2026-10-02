import unittest
import numpy as np
import warp as wp
import newton
from cardboard.block_solver import TranslationBlockVBD
from cardboard.simulation import motor_control

class TranslationBlockTest(unittest.TestCase):
    def make_model(self,pin=False):
        b=newton.ModelBuilder()
        b.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),
                         vertices=np.array([[0.,0.,1.],[.1,0.,1.],[0.,.1,1.]],np.float32),
                         indices=[0,1,2],density=2.7,tri_ke=52920.,tri_ka=88200.,tri_kd=.02)
        if pin:b.particle_mass[0]=0.
        b.color(include_bending=True)
        return b.finalize(device='cuda:1')

    def test_free_flight_preserves_shape_and_linear_momentum(self):
        with wp.ScopedDevice('cuda:1'):
            m=self.make_model();s=TranslationBlockVBD(m,iterations=2,rigid_compliant_alm=False)
            a,b=m.state(),m.state();c=m.control();initial=a.particle_q.numpy().copy()
            velocity=np.array([.12,-.04,.03],np.float32)
            a.particle_qd.assign(np.tile(velocity,(3,1)));dt=1/480;n=120
            for _ in range(n):
                a.clear_forces();s.step(a,b,c,None,dt);a,b=b,a
            g=m.gravity.numpy()[0];expected=initial+velocity*n*dt+g*(n*(n+1)/2*dt*dt)
            np.testing.assert_allclose(a.particle_q.numpy(),expected,atol=2e-4)
            np.testing.assert_allclose(a.particle_qd.numpy().mean(0),velocity+g*n*dt,atol=.003)
            q=a.particle_q.numpy();np.testing.assert_allclose(q-q.mean(0),initial-initial.mean(0),atol=1e-5)

    def test_rejects_pinned_shell(self):
        with wp.ScopedDevice('cuda:1'):
            with self.assertRaisesRegex(ValueError,'free shell'):
                TranslationBlockVBD(self.make_model(pin=True),iterations=2,rigid_compliant_alm=False)

    def test_finger_drive_respects_force_cap_and_damping(self):
        with wp.ScopedDevice('cuda:1'):
            q=wp.zeros(8);qd=wp.array([0.]*6+[.2,-.2],dtype=float)
            target=wp.array([0.]*6+[.1,-.1],dtype=float)
            force=wp.zeros(8);limits=wp.array([150.]*8,dtype=float)
            for kp,kd,cap,expected in [(1200.,12.,150.,117.6),(7200.,85.,600.,600.)]:
                wp.launch(motor_control,dim=8,inputs=[q,qd,target,limits,wp.array([cap],dtype=float),kp,kd,force])
                np.testing.assert_allclose(force.numpy(),[0.]*6+[expected,-expected],rtol=1e-6)

if __name__=='__main__':unittest.main()
