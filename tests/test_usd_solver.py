import tempfile
import unittest
from pathlib import Path
from pxr import Usd,Sdf,UsdGeom
from cardboard import ROOT
from cardboard.usd_solver import read_solver_config, write_runtime_layer
from cardboard.release import load_profile

class USDContractTest(unittest.TestCase):
    def test_delivered_asset_preserves_solver_and_material_contract(self):
        scene=read_solver_config(ROOT/'assets/demo_scene_robotiq_board4_vertical.usda')
        asset=read_solver_config(ROOT/'assets/cardboard_simready.usda','/Box/SimMesh')
        self.assertEqual(scene,asset)
        self.assertEqual(scene['small_bend']['crease_friction_curvature'],45.)
        self.assertEqual(scene['iterations'],24)
        self.assertTrue(Path(scene['rom']['basis']).is_file())
        self.assertEqual(load_profile('config/vertical_pick.json')['iterations'],24)

    def test_authored_usd_override_changes_runtime_configuration(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'override.usda';s=Usd.Stage.CreateNew(str(p))
            UsdGeom.SetStageMetersPerUnit(s, 1);UsdGeom.SetStageUpAxis(s, 'Z')
            s.GetRootLayer().subLayerPaths=[str(ROOT/'assets/demo_scene_robotiq_board4_vertical.usda')]
            s.GetPrimAtPath('/World/Box/SimMesh').GetAttribute('cardboard:solver:iterations').Set(12)
            s.GetPrimAtPath('/World/Box/Materials/Cardboard').GetAttribute('cardboard:smallBend:creaseFrictionCurvature').Set(30.)
            s.GetRootLayer().Save();config=read_solver_config(p)
            self.assertEqual(config['iterations'],12)
            self.assertEqual(config['small_bend']['crease_friction_curvature'],30.)

    def test_invalid_usd_parameters_rejected_before_gpu_initialization(self):
        cases = [('/World/Box/SimMesh', 'cardboard:solver:iterations', 0),
                 ('/World/Box/SimMesh', 'cardboard:solver:schedule', 'unknown'),
                 ('/World/Box/Materials/Cardboard', 'cardboard:smallBend:scale', float('nan'))]
        for prim, name, value in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/'invalid.usda'
                stage = Usd.Stage.CreateNew(str(path))
                UsdGeom.SetStageMetersPerUnit(stage, 1);UsdGeom.SetStageUpAxis(stage, 'Z')
                stage.GetRootLayer().subLayerPaths = [str(ROOT/'assets/demo_scene_robotiq_board4_vertical.usda')]
                stage.GetPrimAtPath(prim).GetAttribute(name).Set(value)
                stage.GetRootLayer().Save()
                with self.assertRaisesRegex(ValueError, 'Invalid USD'):
                    read_solver_config(path)

    def test_runtime_layer_preserves_stage_units_and_relative_dependencies(self):
        from types import SimpleNamespace
        scene = ROOT/'assets/demo_scene_robotiq_board4_vertical.usda'
        stage = Usd.Stage.Open(str(scene))
        sim = SimpleNamespace(stage=stage, iterations=24, time=0, newton_version='1.6.0',
                              solver=SimpleNamespace(contact_schedule='guarded'),
                              mat=stage.GetPrimAtPath('/World/Box/Materials/Cardboard'))
        config = read_solver_config(stage)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'effective_scene.usda'
            write_runtime_layer(sim, path, config)
            saved = Usd.Stage.Open(str(path))
            self.assertEqual(UsdGeom.GetStageMetersPerUnit(saved), 1)
            self.assertEqual(UsdGeom.GetStageUpAxis(saved), 'Z')
            self.assertEqual(str(saved.GetDefaultPrim().GetPath()), '/World')
            self.assertFalse(Path(saved.GetRootLayer().subLayerPaths[0]).is_absolute())
            self.assertFalse(saved.GetCompositionErrors())
            self.assertEqual(read_solver_config(saved), config)

    def test_missing_basis_rejected_before_gpu_initialization(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'bad.usda';s=Usd.Stage.CreateNew(str(p))
            UsdGeom.SetStageMetersPerUnit(s, 1);UsdGeom.SetStageUpAxis(s, 'Z')
            s.GetRootLayer().subLayerPaths=[str(ROOT/'assets/demo_scene_robotiq_board4_vertical.usda')]
            s.GetPrimAtPath('/World/Box/SimMesh').GetAttribute('cardboard:solver:romBasis').Set(Sdf.AssetPath('missing.npz'))
            s.GetRootLayer().Save()
            with self.assertRaisesRegex(ValueError,'Unresolved USD ROM basis'):read_solver_config(p)

if __name__=='__main__':unittest.main()
