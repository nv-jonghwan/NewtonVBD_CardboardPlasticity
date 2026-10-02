"""Independent geometric gates for the scaled commercial linkage."""
import unittest
import numpy as np
import newton
from pxr import Usd,UsdGeom,UsdPhysics
from cardboard import ROOT
from cardboard.usd_utils import register
from cardboard.kinematics import RobotIK
from cardboard.robotiq import RobotiqDrive

class RobotiqGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        register();cls.stage=Usd.Stage.Open(str(ROOT/'assets/demo_scene_robotiq_scaled.usda'))
        b=cls.builder=newton.ModelBuilder();b.add_usd(cls.stage,ignore_paths=['/World/Box'],enable_self_collisions=False,load_visual_shapes=False)
        b.joint_target_ke=[0.]*b.joint_dof_count;b.joint_target_kd=[0.]*b.joint_dof_count
        class Sim:pass
        sim=Sim();sim.stage=cls.stage;sim.settings=cls.stage.GetPrimAtPath('/World/Physics');sim.attr=lambda p,n:p.GetAttribute('cardboard:'+n).Get()
        sim.left=b.body_label.index('/World/Gripper/Left');sim.right=b.body_label.index('/World/Gripper/Right');sim.tip=b.body_label.index('/World/UR10/ee_link')
        cls.drive=RobotiqDrive(sim,b);cls.ik=RobotIK(b,sim.tip)
    def test_visible_meshes_not_duplicated_by_deinstancing(self):
        meshes=[p for p in self.stage.Traverse() if str(p.GetPath()).startswith('/World/Gripper/') and p.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(meshes),11)
        self.assertTrue(all(p.HasAPI(UsdPhysics.CollisionAPI) for p in meshes))
    def test_loop_closure_through_stroke(self):
        b=self.builder
        for angle in [0,.2,.4,.6,.78]:
            q=np.array(b.joint_q);self.drive.set_angle(q,angle);f=self.ik.fk(q)
            for side in ['left','right']:
                j=b.joint_label.index('/World/Gripper/'+side+'_inner_finger_pad_joint')
                a=f[b.joint_parent[j]]@self.ik.xp[j];c=f[b.joint_child[j]]@np.linalg.inv(self.ik.xc[j])
                self.assertLess(np.linalg.norm(a[:3,3]-c[:3,3]),2e-6)
    def test_opening_fits_280mm_box_with_clearance(self):
        self.assertGreater(self.drive.gaps[0],.32)
        self.assertLess(self.drive.gaps[-1],.001)
        self.assertTrue(np.all(np.diff(self.drive.gaps)<0))
        self.assertGreater(self.drive.depth_shift(.14),.03)
    def test_physical_flange_and_adapter_mating_planes(self):
        # Independent source geometry: wrist3's cylindrical mesh ends at Z=0
        # and occupies negative Z. Its +Z is the outward flange normal.
        wrist=self.stage.GetPrimAtPath('/World/UR10/wrist_3_link')
        palm=self.stage.GetPrimAtPath('/World/Gripper/Palm')
        cache=UsdGeom.XformCache();bb=UsdGeom.BBoxCache(0,['default','render'])
        inverse=cache.GetLocalToWorldTransform(wrist).GetInverse()
        relative=np.asarray(cache.GetLocalToWorldTransform(palm)*inverse).T
        np.testing.assert_allclose(relative[:3,2],[0,0,1],atol=2e-6)
        np.testing.assert_allclose(relative[:2,3],0,atol=2e-6)
        wristmesh=self.stage.GetPrimAtPath('/World/UR10/wrist_3_link/visuals/wrist3/ur10_wrist_3')
        self.assertLess(abs(bb.ComputeRelativeBound(wristmesh,wrist).ComputeAlignedRange().GetMax()[2]),.0001)
        ranges=[]
        for name in ['FlangeAdapter','AdapterPlate','Geometry']:
            prim=self.stage.GetPrimAtPath('/World/Gripper/Palm/'+name)
            ranges.append(bb.ComputeRelativeBound(prim,wrist).ComputeAlignedRange())
        self.assertAlmostEqual(ranges[0].GetMin()[2],0,places=5)
        self.assertAlmostEqual(ranges[0].GetMax()[2],ranges[1].GetMin()[2],places=5)
        self.assertAlmostEqual(ranges[1].GetMax()[2],ranges[2].GetMin()[2],places=5)
    def test_downward_ik_keeps_flange_and_gripper_coaxial(self):
        b=self.builder;q=np.array(b.joint_q);q[:6]=[0,-1.1,1.7,-2.1,-1.57,0]
        for position in [[.32,0,1.19],[.32,0,1.34],[.28,0,1.43]]:
            target=np.eye(4);target[:3,:3]=self.drive.downward_ee_rotation;target[:3,3]=position
            q=self.ik.solve(target,q);f=self.ik.fk(q)
            wrist=f[b.body_label.index('/World/UR10/wrist_3_link')]
            palm=f[b.body_label.index('/World/Gripper/Palm')]
            np.testing.assert_allclose(wrist[:3,2],[0,0,-1],atol=2e-6)
            np.testing.assert_allclose(palm[:3,2],wrist[:3,2],atol=2e-6)
            np.testing.assert_allclose(palm[:3,1],[-1,0,0],atol=2e-6)
            np.testing.assert_allclose((palm[:3,3]-wrist[:3,3])[:2],0,atol=2e-6)
    def test_fixed_attachment_and_unscaled_robot(self):
        joint=UsdPhysics.FixedJoint(self.stage.GetPrimAtPath('/World/Gripper/Mount'))
        self.assertEqual(str(joint.GetBody0Rel().GetTargets()[0]),'/World/UR10/ee_link')
        self.assertEqual(str(joint.GetBody1Rel().GetTargets()[0]),'/World/Gripper/Palm')
        reference=Usd.Stage.Open(str(ROOT/'assets/demo_scene_board15_half.usda'))
        for name in ['base_link','ee_link']:
            path='/World/UR10/'+name
            np.testing.assert_allclose(UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(path)),UsdGeom.XformCache().GetLocalToWorldTransform(reference.GetPrimAtPath(path)))

if __name__=='__main__':unittest.main()
