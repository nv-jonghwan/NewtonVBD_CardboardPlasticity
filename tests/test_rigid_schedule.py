"""Contact scheduling must preserve all contact kinds and additive outputs."""
import unittest
import numpy as np
import warp as wp
from newton._src.solvers.vbd.rigid_vbd_kernels import accumulate_body_particle_contacts_per_body
from cardboard.scheduled_contact import scheduled_body_particle_contacts
from cardboard.rigid_schedule import clear_body_accumulators


class ScheduledContactTest(unittest.TestCase):
    def setUp(self):
        self.device='cuda:1'
        wp.set_device(self.device)

    def inputs(self,count,kind,capacity=256):
        n=max(300,capacity+44)
        arr=lambda x,dtype:wp.array(x,dtype=dtype,device=self.device)
        floats=lambda x:arr(np.full(n,x,np.float32),float)
        vec=lambda x:arr(np.tile(x,(n,1)),wp.vec3)
        pose=arr([[0,0,0,0,0,0,1]],wp.transform)
        corners=([0,-1,-1],[0,1,-1],[0,1,2])[kind]
        bary=([1,0,0],[.4,.6,0],[.2,.3,.5])[kind]
        # Mix active/separating records and particle padding; EF corner 0 must
        # remain valid under the upstream barycentric contact contract.
        indices=np.tile(corners,(n,1))
        if kind==0:indices[7::13,0]=-1
        normals=np.tile([0,0,1.],(n,1));normals[5::11]*=-1
        return [.001,arr([0],int),arr([[0,0,-.002],[.001,0,-.001],[0,.001,-.003]],wp.vec3),
                arr([[-.0001,0,-.002],[.0009,0,-.001],[-.0001,.001,-.003]],wp.vec3),arr([.001]*3,float),
                pose,pose,arr([[0]*6],wp.spatial_vector),arr([[.003,0,0]],wp.vec3),arr([1.],float),arr([0],int),
                .0005,floats(10000),floats(10000),floats(.02),floats(.4),arr([n],int),arr(indices,wp.vec3i),
                arr(np.zeros(n,int),int),vec([0,0,0]),vec([0,0,0]),arr(normals,wp.vec3),vec(bary),arr([0.],float),
                capacity,arr([count],int),arr(np.arange(capacity),int)]

    def test_expanded_capacity_preserves_contacts_above_256(self):
        for kind in range(3):
            for count in [257,300,511,512,550]:
                with self.subTest(kind=kind,count=count):
                    inputs=self.inputs(count,kind,512);reference=self.outputs();actual=self.outputs()
                    wp.launch(accumulate_body_particle_contacts_per_body,dim=4,inputs=inputs,outputs=reference,device=self.device)
                    wp.launch(scheduled_body_particle_contacts,dim=4,inputs=[inputs[0],4,*inputs[1:]],outputs=actual,device=self.device)
                    for a,b in zip(actual,reference):
                        np.testing.assert_array_equal(a.numpy(),b.numpy())

    def outputs(self):
        return [wp.array(np.full((1,3),.125,np.float32),dtype=wp.vec3,device=self.device) for _ in range(2)]+[
            wp.array(np.full((1,3,3),.125,np.float32),dtype=wp.mat33,device=self.device) for _ in range(3)]

    def test_all_contact_kinds_empty_small_large_and_capacity_clamp(self):
        for kind in range(3):
            for count in [0,1,3,31,32,65,244,256,300]:
                inputs=self.inputs(count,kind);reference=self.outputs()
                wp.launch(accumulate_body_particle_contacts_per_body,dim=4,inputs=inputs,outputs=reference,device=self.device)
                for workers in [4,32]:
                    with self.subTest(kind=kind,count=count,workers=workers):
                        actual=self.outputs()
                        wp.launch(scheduled_body_particle_contacts,dim=workers,inputs=[inputs[0],workers,*inputs[1:]],outputs=actual,device=self.device)
                        for a,b in zip(actual,reference):
                            np.testing.assert_allclose(a.numpy(),b.numpy(),rtol=2e-5,atol=1e-5)

    def test_graph_reads_changed_contact_counts_without_host_branch(self):
        inputs=self.inputs(0,0);actual=self.outputs()
        with wp.ScopedCapture(device=self.device) as capture:
            wp.launch(scheduled_body_particle_contacts,dim=32,inputs=[inputs[0],32,*inputs[1:]],outputs=actual,device=self.device)
        for count in [0,65,1,244,0]:
            inputs[-2].assign(np.array([count],np.int32))
            for x,seed in zip(actual,self.outputs()):wp.copy(x,seed)
            wp.capture_launch(capture.graph)
            reference=self.outputs()
            wp.launch(accumulate_body_particle_contacts_per_body,dim=4,inputs=inputs,outputs=reference,device=self.device)
            for a,b in zip(actual,reference):np.testing.assert_allclose(a.numpy(),b.numpy(),rtol=2e-5,atol=1e-5)

    def test_fused_clear_resets_all_bodies_and_components(self):
        arrays=[wp.array(np.ones((17,3),np.float32),dtype=wp.vec3) for _ in range(2)]+[
            wp.array(np.ones((17,3,3),np.float32),dtype=wp.mat33) for _ in range(3)]
        wp.launch(clear_body_accumulators,dim=17,inputs=arrays,device=self.device)
        for array in arrays:self.assertTrue(np.all(array.numpy()==0))


if __name__=='__main__':unittest.main()
