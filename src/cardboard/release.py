"""Portable release profile and input integrity checks."""
import hashlib
import json
import math
from pathlib import Path

from . import ROOT

DEFAULT_PROFILE = 'config/vertical_pick.json'
LOCK_FILE = 'config/release-inputs.lock.json'


def local_path(value, root=ROOT):
    path = Path(value)
    if path.is_absolute() or not path.parts or '..' in path.parts:
        raise ValueError(f'Expected a repository-relative path: {value}')
    resolved = (root / path).resolve()
    if not resolved.is_relative_to(root.resolve()):
        raise ValueError(f'Path escapes the repository: {value}')
    return resolved


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load_profile(path=DEFAULT_PROFILE, root=ROOT):
    p = json.loads(local_path(path, root).read_text())
    if p['schema_version'] != 1:
        raise ValueError('Unsupported profile schema')
    if p.get('solver_source') == 'usd':
        from .usd_solver import read_solver_config
        authored = read_solver_config(local_path(p['scene'], root))
        if authored is None:
            raise ValueError('Profile requires CardboardSolverAPI on the USD simulation mesh')
        authored['rom']['basis'] = Path(authored['rom']['basis']).relative_to(root.resolve()).as_posix()
        p.update(authored)

    if type(p['iterations']) is not int or p['iterations'] < 1:
        raise ValueError('iterations must be a positive integer')
    if not math.isfinite(p['duration_s']) or p['duration_s'] <= 0:
        raise ValueError('duration_s must be finite and positive')
    if p['schedule'] not in ('baseline', 'guarded'):
        raise ValueError('Unsupported schedule')
    local_path(p['scene'], root)
    local_path(p['rom']['basis'], root)
    b = p['small_bend']
    if not all(math.isfinite(x) for x in b.values()):
        raise ValueError('small_bend parameters must be finite')
    if not (b['scale'] > 1 and 0 <= b['knee'] < b['end'] and
            b['memory_curvature'] >= 0 and b['crease_friction_curvature'] >= 0):
        raise ValueError('Invalid small_bend parameters')
    # These flags define the validated fast path. Reject silent runner mismatch.
    r = p['rom']
    if not (r['start_with_rom'] and not r['check_residual'] and r['defer_fallback'] and
            r['local_vbd'] and r['element_full_after_yield']):
        raise ValueError('This candidate runner requires the validated hybrid ROM flags')
    for k in ('full_every', 'patch_full_every', 'patch_rings'):
        if type(r[k]) is not int or r[k] < 1:
            raise ValueError(f'Invalid ROM {k}')
    for k in ('tolerance', 'element_fraction', 'patch_curvature', 'patch_relaxation'):
        if not math.isfinite(r[k]) or r[k] <= 0:
            raise ValueError(f'Invalid ROM {k}')
    if r['element_fraction'] > 1 or r['patch_relaxation'] > 1 or r['patch_solver'] not in ('jacobi', 'colored'):
        raise ValueError('Invalid local/representative ROM parameters')
    return p


def verify_inputs(root=ROOT):
    lock = json.loads((root / LOCK_FILE).read_text())
    failures = []
    for entry in lock['files']:
        path = local_path(entry['path'], root)
        if not path.is_file():
            failures.append(f'missing: {entry["path"]}')
        elif sha256(path) != entry['sha256']:
            failures.append(f'checksum mismatch: {entry["path"]}')
    if failures:
        raise ValueError('\n'.join(failures))
    return lock


def solver_arguments(p):
    b, r = p['small_bend'], p['rom']
    args = ['--scene', p['scene'], '--iterations', str(p['iterations']),
            '--schedule', p['schedule'], '--rom-basis', r['basis'],
            '--rom-start', '--rom-fast', '--rom-full-after-yield', '--local-vbd',
            '--exploratory', '--panel-diagnostics']
    for key in ('scale', 'knee', 'end', 'memory_curvature'):
        args += ['--small-bend-' + key.replace('_', '-'), str(b[key])]
    args += ['--crease-friction-curvature', str(b['crease_friction_curvature'])]
    for key in ('tolerance', 'full_every', 'element_fraction'):
        args += ['--rom-' + key.replace('_', '-'), str(r[key])]
    for key in ('patch_rings', 'patch_curvature', 'patch_full_every', 'patch_solver', 'patch_relaxation'):
        args += ['--' + key.replace('_', '-'), str(r[key])]
    return args
