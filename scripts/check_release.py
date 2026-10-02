"""Check candidate inputs; --runtime also checks the compute environment and USD."""
import argparse
import json
import platform
from importlib.metadata import version

from cardboard import ROOT
from cardboard.release import load_profile, verify_inputs


def runtime_checks(p, device):
    import numpy as np
    import warp as wp
    from pxr import Usd, UsdGeom
    from cardboard.usd_utils import register
    from cardboard.rom_basis import mesh_digest
    expected = {'newton': '1.6.0', 'warp-lang': '1.17.0', 'numpy': '2.3.1', 'usd-core': '25.11'}
    packages = {name: version(name) for name in expected}
    if packages != expected:
        raise ValueError(f'Compute package versions differ from the validated lock: {packages}')
    register()
    stage = Usd.Stage.Open(str(ROOT / p['scene']))
    if not stage or stage.GetCompositionErrors():
        raise ValueError('USD composition failed')
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if str(attr.GetTypeName()) == 'asset':
                asset = attr.Get()
                if asset and asset.path and not asset.resolvedPath and asset.path != 'OmniPBR.mdl':
                    raise ValueError(f'Unresolved active asset: {prim.GetPath()} {asset.path}')
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/SimMesh'))
    rest = np.array(mesh.GetPrim().GetAttribute('cardboard:restPoints').Get())
    faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    edges = {tuple(sorted((int(a), int(b)))) for tri in faces for a, b in zip(tri, np.roll(tri, -1))}
    thickness = stage.GetPrimAtPath('/World/Box/Materials/Cardboard').GetAttribute('cardboard:thickness').Get()
    e = p['expected']
    if (len(rest), len(faces), len(edges)) != (e['vertices'], e['triangles'], e['hinges']):
        raise ValueError('Candidate topology differs from the reference')
    if abs(thickness - e['thickness_m']) > 1e-8:
        raise ValueError('Candidate thickness differs from the reference')
    with np.load(ROOT / p['rom']['basis'], allow_pickle=False) as data:
        if data['basis'].shape != (len(rest), e['basis_rank'], 3) or not np.isfinite(data['basis']).all():
            raise ValueError('Invalid ROM basis')
        if str(data['mesh_digest']) != mesh_digest(rest, faces):
            raise ValueError('ROM basis / mesh digest mismatch')
    wp.init()
    gpu = wp.get_device(device)
    if not gpu.is_cuda:
        raise ValueError('Candidate requires a CUDA device')
    # Actual allocation/transfer, rather than an availability string alone.
    probe = wp.array([1., 2., 3.], dtype=float, device=gpu)
    if not np.array_equal(probe.numpy(), [1., 2., 3.]):
        raise ValueError('GPU allocation/readback failed')
    return dict(packages=packages, device=str(gpu), gpu=gpu.name, python=platform.python_version(),
                vertices=len(rest), triangles=len(faces), hinges=len(edges), thickness_m=thickness,
                active_usd_dependencies='resolved; OmniPBR.mdl requires Kit', gpu_allocation=True)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--runtime', action='store_true')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--output')
    args = parser.parse_args()
    report = {'passed': False, 'production_qualified': False,
              'qualification': 'Integrity/preflight only; no full-cycle or material qualification implied.'}
    try:
        p = load_profile()
        lock = verify_inputs()
        report.update(profile=p['name'], locked_files=len(lock['files']))
        if args.runtime:
            report['runtime'] = runtime_checks(p, args.device)
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
    if args.output:
        out = ROOT / args.output
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
