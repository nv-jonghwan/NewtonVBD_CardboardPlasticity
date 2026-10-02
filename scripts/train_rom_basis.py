"""Create a POD basis with explicit training provenance; no replay at runtime."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom
from cardboard import ROOT
from cardboard.rom_basis import fit_increment_basis,mesh_digest

p=argparse.ArgumentParser()
p.add_argument('--training',nargs='+',default=['outputs/primitive_gripper/mesh_lift20/trajectory.npz'])
p.add_argument('--scene',default='assets/demo_scene_robotiq_board10_primitive.usda')
p.add_argument('--rank',type=int,default=8)
p.add_argument('--output',default='outputs/rom_trial/basis8.npz')
a=p.parse_args();stage=Usd.Stage.Open(str(ROOT/a.scene));mesh=stage.GetPrimAtPath('/World/Box/SimMesh')
rest=np.asarray(mesh.GetAttribute('cardboard:restPoints').Get());faces=np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
trajectories=[]
for name in a.training:
    with np.load(ROOT/name) as d:trajectories.append(d['points'])
basis,captured=fit_increment_basis(trajectories,a.rank)
out=ROOT/a.output;out.parent.mkdir(parents=True,exist_ok=True)
np.savez_compressed(out,basis=basis,mesh_digest=mesh_digest(rest,faces))
report=dict(rank=a.rank,physical_dofs=len(rest)*3,captured_centered_increment_energy=captured,
            training=[dict(path=name,sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()) for name in a.training],
            basis_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),method='3 translation modes + POD of centered physical increments; Euclidean orthonormal; no trajectory playback')
out.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
