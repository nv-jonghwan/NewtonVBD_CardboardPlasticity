import unittest
import numpy as np
import warp as wp
from cardboard.small_bend_kernels import crease_friction_moment,evaluate_small_bend_force_hessian,record_crease_dissipation
from test_small_bend import angle

@wp.kernel
def law(delta:wp.array[float],limit:float,epsilon:float,out:wp.array[wp.vec2]):
    i=wp.tid();out[i]=crease_friction_moment(delta[i],limit,epsilon)

@wp.kernel
def force(q:wp.array[wp.vec3],old:wp.array[wp.vec3],edges:wp.array2d[int],rest:wp.array[float],
          lengths:wp.array[float],dual:wp.array[float],alpha:wp.array[float],out:wp.array[wp.vec3]):
    i=wp.tid()
    f,h=evaluate_small_bend_force_hessian(0,i,q,old,edges,rest,lengths,.6,0.,.001,dual,alpha,1.,.75,14.75,0.,15.)
    f0,h0=evaluate_small_bend_force_hessian(0,i,q,old,edges,rest,lengths,.6,0.,.001,dual,alpha,1.,.75,14.75)
    out[i]=f-f0

class CreaseFrictionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):wp.init();cls.device='cuda:0' if wp.is_cuda_available() else 'cpu'
    def test_dissipation_cap_and_quadratic_majorizer(self):
        limit=.2;eps=5e-6;d=np.array([-.03,-eps/2,0.,eps/2,.03],np.float32)
        out=wp.empty(len(d),dtype=wp.vec2,device=self.device)
        wp.launch(law,dim=len(d),inputs=[wp.array(d,device=self.device),limit,eps],outputs=[out],device=self.device)
        m,h=out.numpy().T
        np.testing.assert_allclose(m,limit*np.clip(d/eps,-1,1),rtol=1e-6)
        self.assertTrue(np.all(m*d>=0));self.assertTrue(np.all(h>0));self.assertLessEqual(abs(m).max(),limit+1e-7)
        def potential(x):return limit*np.where(abs(x)<eps,.5*x*x/eps,abs(x)-.5*eps)
        for x,moment,metric in zip(d,m,h):
            y=np.linspace(-.05,.05,101)
            bound=potential(x)+moment*(y-x)+.5*metric*(y-x)**2
            self.assertTrue(np.all(potential(y)<=bound+1e-7))
    def test_nodal_force_matches_incremental_potential_and_preserves_rigid_momenta(self):
        q=np.array([[0,1,0],[0,-1,.03],[-.5,0,0],[.5,0,0]],np.float32);old=q.copy();old[1,2]-=.002
        arr=lambda x,dtype=float:wp.array(x,dtype=dtype,device=self.device)
        out=wp.empty(4,dtype=wp.vec3,device=self.device)
        wp.launch(force,dim=4,inputs=[arr(q,wp.vec3),arr(old,wp.vec3),arr([[0,1,2,3]],int),arr([0]),arr([1]),arr([.01]),arr([.05])],outputs=[out],device=self.device)
        observed=out.numpy();limit=.6*.01*15*(1-np.exp(-1));eps=.001*.005
        def energy(x):
            delta=angle(x)-angle(old.astype(float));return limit*(abs(delta)-.5*eps)
        numerical=np.zeros((4,3));q=q.astype(float);step=1e-6
        for i in range(4):
            for j in range(3):
                plus=q.copy();minus=q.copy();plus[i,j]+=step;minus[i,j]-=step
                numerical[i,j]=-(energy(plus)-energy(minus))/(2*step)
        np.testing.assert_allclose(observed,numerical,rtol=4e-4,atol=2e-6)
        np.testing.assert_allclose(observed.sum(0),0,atol=1e-6)
        np.testing.assert_allclose(np.cross(q,observed).sum(0),0,atol=1e-6)

    def test_work_ledger_is_nonnegative_and_zero_at_rest(self):
        q=np.array([[0,1,0],[0,-1,.03],[-.5,0,0],[.5,0,0]],np.float32);old=q.copy();old[1,2]-=.002
        arr=lambda x,dtype=float:wp.array(x,dtype=dtype,device=self.device)
        current=arr(q,wp.vec3);previous=arr(old,wp.vec3);work=wp.zeros(1,dtype=float,device=self.device)
        tail=[arr([[0,1,2,3]],int),arr([1]),arr([.01]),arr([.05]),arr([[.6,0]]),15.,.001]
        wp.launch(record_crease_dissipation,dim=1,inputs=[previous,current,*tail],outputs=[work],device=self.device)
        expected=.6*.01*15*(1-np.exp(-1))*abs(angle(q.astype(float))-angle(old.astype(float)))
        np.testing.assert_allclose(work.numpy(),[expected],rtol=1e-4)
        wp.launch(record_crease_dissipation,dim=1,inputs=[current,current,*tail],outputs=[work],device=self.device)
        np.testing.assert_allclose(work.numpy(),[expected],rtol=1e-4)
