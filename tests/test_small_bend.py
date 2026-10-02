import unittest
import numpy as np
import warp as wp
from cardboard.small_bend_trial import validate_small_bend
from cardboard.small_bend_kernels import evaluate_small_bend_force_hessian,release_small_bend_energy,release_memory_energy
from newton._src.solvers.vbd.particle_vbd_kernels import evaluate_dihedral_angle_based_bending_force_hessian

@wp.kernel
def evaluate(q:wp.array[wp.vec3],anchor:wp.array[wp.vec3],edges:wp.array2d[int],rest:wp.array[float],
             length:wp.array[float],dual:wp.array[float],alpha:wp.array[float],scale:float,damping:float,knee:float,end:float,memory:float,
             f:wp.array[wp.vec3],h:wp.array[wp.mat33],original_f:wp.array[wp.vec3],original_h:wp.array[wp.mat33]):
    i=wp.tid()
    fi,hi=evaluate_small_bend_force_hessian(0,i,q,anchor,edges,rest,length,.6,damping,.001,
                                          dual,alpha,scale,knee,end,memory)
    fo,ho=evaluate_dihedral_angle_based_bending_force_hessian(0,i,q,anchor,edges,rest,length,.6,damping,.001)
    f[i]=fi;h[i]=hi;original_f[i]=fo;original_h[i]=ho


def angle(q):
    n0=np.cross(q[2]-q[0],q[3]-q[0]);n0/=np.linalg.norm(n0)
    n1=np.cross(q[3]-q[1],q[2]-q[1]);n1/=np.linalg.norm(n1)
    e=q[3]-q[2];e/=np.linalg.norm(e)
    return np.arctan2(np.cross(n0,n1)@e,n0@n1)


def energy(q,alpha=0.,scale=16.,knee=.3,end=5.9,memory=0.):
    if memory>0:
        x=np.clip(alpha/(.01*memory),0,1)
        scale=1+(scale-1)*(1-x*x*(3-2*x));alpha=0.
    c=abs(angle(q))/.01
    if alpha>0 or scale==1:integral=.5*c*c
    elif c<=knee:integral=.5*scale*c*c
    elif c<end:
        x=c-knee;slope=(end-scale*knee)/(end-knee)
        integral=.5*scale*knee*knee+scale*knee*x+.5*slope*x*x
    else:integral=.5*c*c+.5*(scale-1)*knee*end
    return .6*.01**2*integral


class SmallBendTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init();cls.device='cuda:0' if wp.is_cuda_available() else 'cpu'

    def probe(self,curvature,alpha=0.,scale=16.,damping=0.,knee=.3,end=5.9,memory=0.):
        q=np.array([[0,1,0],[0,-1,np.tan(curvature*.01)],[-.5,0,0],[.5,0,0]],dtype=np.float32)
        anchor=q.copy()
        if damping:anchor[1,2]-=.001
        def array(x,dtype=float):return wp.array(x,dtype=dtype,device=self.device)
        f=wp.empty(4,dtype=wp.vec3,device=self.device);h=wp.empty(4,dtype=wp.mat33,device=self.device)
        fo=wp.empty_like(f);ho=wp.empty_like(h)
        wp.launch(evaluate,dim=4,inputs=[array(q,wp.vec3),array(anchor,wp.vec3),array([[0,1,2,3]],int),
                  array([0]),array([1]),array([.01]),array([alpha]),scale,damping,knee,end,memory],outputs=[f,h,fo,ho],device=self.device)
        return q.astype(float),f.numpy(),h.numpy(),fo.numpy(),ho.numpy()

    def test_force_is_negative_gradient_of_potential(self):
        for c in [-6.1,-2.,-.1,0.,.1,.31,2.,5.8,6.1]:
            q,f,h,_,_=self.probe(c)
            numerical=np.zeros_like(q);step=1e-6
            for i in range(4):
                for j in range(3):
                    plus=q.copy();minus=q.copy();plus[i,j]+=step;minus[i,j]-=step
                    numerical[i,j]=-(energy(plus)-energy(minus))/(2*step)
            np.testing.assert_allclose(f,numerical,rtol=3e-4,atol=2e-6)
            self.assertGreaterEqual(np.linalg.eigvalsh(h).min(),-2e-6)
            np.testing.assert_allclose(f.sum(axis=0),0,atol=2e-6)

    def test_original_force_and_damping_recovered(self):
        for c,alpha,scale in [(6.1,0.,16.),(-6.1,0.,16.),(.1,.01,16.),(2.,.01,16.),(.1,0.,1.)]:
            _,f,h,fo,ho=self.probe(c,alpha,scale,damping=.02)
            np.testing.assert_allclose(f,fo,rtol=1e-6,atol=1e-6)
            np.testing.assert_allclose(h,ho,rtol=1e-6,atol=1e-6)

    def test_flat_panel_profile_gradient_and_existing_crease(self):
        # The new profile strengthens pristine small bends, not yielded hinges.
        validate_small_bend(48.,.3,14.75,15.)
        for c in [-15.,-4.,-.1,.1,.4,4.,14.7,15.]:
            q,f,h,_,_=self.probe(c,scale=48.,end=14.75)
            numerical=np.zeros_like(q);step=1e-6
            for i in range(4):
                for j in range(3):
                    plus=q.copy();minus=q.copy();plus[i,j]+=step;minus[i,j]-=step
                    numerical[i,j]=-(energy(plus,scale=48.,end=14.75)-energy(minus,scale=48.,end=14.75))/(2*step)
            np.testing.assert_allclose(f,numerical,rtol=3e-4,atol=2e-6)
            self.assertGreaterEqual(np.linalg.eigvalsh(h).min(),-2e-6)
            _,ff,hh,fo,ho=self.probe(c,alpha=.01,scale=48.,end=14.75,damping=.02)
            np.testing.assert_allclose(ff,fo,rtol=1e-6,atol=1e-6)
            np.testing.assert_allclose(hh,ho,rtol=1e-6,atol=1e-6)
        # At equal small curvature, pristine resistance is three times S16.
        _,f48,_,_,_=self.probe(.1,scale=48.,end=14.75)
        _,f16,_,_,_=self.probe(.1,scale=16.,knee=.75,end=14.75)
        np.testing.assert_allclose(f48,3*f16,rtol=1e-6,atol=1e-6)

    def test_first_yield_accounts_for_released_energy_once(self):
        a=lambda x:wp.array(x,dtype=float,device=self.device)
        old=a([0.,.01,0.]);new=a([.01,.02,0.]);ke=a([.6]*3);length=a([1.]*3);dual=a([.01]*3);work=a([.2]*3)
        wp.launch(release_small_bend_energy,dim=3,inputs=[old,new,ke,length,dual,16.,.3,5.9],outputs=[work],device=self.device)
        q,_,_,_,_=self.probe(6.1)
        extra=energy(q)-energy(q,alpha=.01)
        np.testing.assert_allclose(work.numpy(),[.2+extra,.2,.2],rtol=1e-6,atol=1e-7)
        old.assign(new.numpy())
        wp.launch(release_small_bend_energy,dim=3,inputs=[old,new,ke,length,dual,16.,.3,5.9],outputs=[work],device=self.device)
        np.testing.assert_allclose(work.numpy(),[.2+extra,.2,.2],rtol=1e-6,atol=1e-7)

    def test_balanced_flat_profile_keeps_crease_response(self):
        validate_small_bend(32.,.4,14.75,15.)
        for c in [-15.,-3.,-.1,.1,3.,15.]:
            q,f,h,_,_=self.probe(c,scale=32.,knee=.4,end=14.75)
            numerical=np.zeros_like(q);step=1e-6
            for i in range(4):
                for j in range(3):
                    plus=q.copy();minus=q.copy();plus[i,j]+=step;minus[i,j]-=step
                    numerical[i,j]=-(energy(plus,scale=32.,knee=.4,end=14.75)-energy(minus,scale=32.,knee=.4,end=14.75))/(2*step)
            np.testing.assert_allclose(f,numerical,rtol=3e-4,atol=2e-6)
            self.assertGreaterEqual(np.linalg.eigvalsh(h).min(),-2e-6)
            _,ff,hh,fo,ho=self.probe(c,alpha=.01,scale=32.,knee=.4,end=14.75,damping=.02)
            np.testing.assert_allclose(ff,fo,rtol=1e-6,atol=1e-6)
            np.testing.assert_allclose(hh,ho,rtol=1e-6,atol=1e-6)

    def test_rejects_negative_tangent_or_changed_yield_branch(self):
        validate_small_bend(16.,.3,5.9,6.)
        for values in [(20.,.3,5.9,6.),(16.,.3,6.,6.),(16.,0.,5.9,6.),(float('nan'),.3,5.9,6.)]:
            with self.assertRaises(ValueError):validate_small_bend(*values)

    def test_gradual_memory_force_matches_energy_and_is_continuous_at_first_yield(self):
        for alpha in [0.,1e-8,.02,.075,.15,.3]:
            for curvature in [-16.,-2.,-.1,.1,2.,16.]:
                q,f,h,fo,ho=self.probe(curvature,alpha=alpha,knee=.75,end=14.75,memory=15.)
                numerical=np.zeros_like(q);step=1e-6
                for i in range(4):
                    for j in range(3):
                        plus=q.copy();minus=q.copy();plus[i,j]+=step;minus[i,j]-=step
                        numerical[i,j]=-(energy(plus,alpha=alpha,knee=.75,end=14.75,memory=15.)-energy(minus,alpha=alpha,knee=.75,end=14.75,memory=15.))/(2*step)
                np.testing.assert_allclose(f,numerical,rtol=3e-4,atol=2e-6)
                self.assertGreaterEqual(np.linalg.eigvalsh(h).min(),-2e-6)
                np.testing.assert_allclose(f.sum(0),0,atol=2e-6)
                if alpha>=.15 or abs(curvature)>=14.75:
                    np.testing.assert_allclose(f,fo,rtol=1e-6,atol=1e-6)
        initial=self.probe(.1,knee=.75,end=14.75,memory=15.)[1]
        yielded=self.probe(.1,alpha=1e-8,knee=.75,end=14.75,memory=15.)[1]
        np.testing.assert_allclose(initial,yielded,rtol=1e-6,atol=1e-6)

    def test_memory_energy_release_includes_damage_and_telescopes(self):
        arr=lambda x:wp.array(x,dtype=float,device=self.device)
        alphas=[np.array([0.,.02,.12]),np.array([.01,.05,.16]),np.array([.04,.09,.2])]
        damages=[np.array([0.,.1,.4]),np.array([.15,.3,.5]),np.array([.3,.5,.6])]
        ke=arr([.6]*3);length=arr([1.]*3);dual=arr([.01]*3)
        work=arr([0.]*3);whole=arr([0.]*3)
        def release(i,j,target):
            wp.launch(release_memory_energy,dim=3,inputs=[arr(alphas[i]),arr(alphas[j]),arr(damages[i]),arr(damages[j]),ke,length,dual,16.,.75,14.75,15.],outputs=[target],device=self.device)
        release(0,1,work);release(1,2,work);release(0,2,whole)
        def weight(alpha,damage):
            x=np.clip(alpha/.15,0,1);return (1-damage)*(1-x*x*(3-2*x))
        expected=.5*.6*.01**2*15*.75*14.75*(weight(alphas[0],damages[0])-weight(alphas[2],damages[2]))
        np.testing.assert_allclose(work.numpy(),expected,rtol=1e-6,atol=1e-7)
        np.testing.assert_allclose(work.numpy(),whole.numpy(),rtol=1e-6,atol=1e-7)
        release(2,2,work)
        np.testing.assert_allclose(work.numpy(),expected,rtol=1e-6,atol=1e-7)

if __name__=='__main__':unittest.main()
