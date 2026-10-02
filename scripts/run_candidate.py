"""Launch the reviewed profile; overrides are recorded by the underlying worker."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import shlex
import subprocess

from cardboard import ROOT
from cardboard.release import DEFAULT_PROFILE, load_profile, verify_inputs, solver_arguments


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('mode', choices=['headless', 'gui'])
    parser.add_argument('--profile', default=DEFAULT_PROFILE)
    parser.add_argument('--device', '--physics-device', dest='device')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--output', '--record-directory', dest='output', help='New run directory; defaults to a unique outputs/candidate timestamp')
    parser.add_argument('--replay-directory', default='outputs/candidate/reference')
    parser.add_argument('--dry-run', action='store_true')
    args, extra = parser.parse_known_args()
    p = load_profile(args.profile)
    verify_inputs()
    duration = p['duration_s'] if args.duration is None else args.duration
    if not math.isfinite(duration) or duration <= 0:
        parser.error('--duration must be finite and positive')
    device = args.device or p['device']
    output = args.output or 'outputs/candidate/' + datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    if (ROOT / output).exists():
        parser.error('Output directory already exists; use a new --output')
    command = [str(ROOT / 'scripts/python.sh')]
    if args.mode == 'headless':
        if extra:
            parser.error('Use --profile for solver changes; unknown arguments: ' + ' '.join(extra))
        command += [str(ROOT / 'scripts/benchmark_vbd_optimized.py'), *solver_arguments(p),
                    '--device', device, '--duration', str(duration), '--output', output]
    else:
        # Only presentation/control switches may bypass the reviewed profile.
        allowed = {'--headless', '--autoplay', '--exit-when-done', '--verify-controls', '--verify-recording'}
        if any(x not in allowed for x in extra):
            parser.error('Unsupported GUI switch; use --profile for solver changes')
        b = p['small_bend']
        command += [str(ROOT / 'scripts/live_isaac.py'), '--quality', 'robotiq-primitive',
                    '--scene', p['scene'], '--solver-config', args.profile,
                    '--physics-device', device, '--vbd-schedule', p['schedule'],
                    '--duration', str(duration), '--record-directory', output,
                    '--replay-directory', args.replay_directory]
        for key in ('scale', 'knee', 'end', 'memory_curvature'):
            command += ['--small-bend-' + key.replace('_', '-'), str(b[key])]
        command += ['--crease-friction-curvature', str(b['crease_friction_curvature']), *extra]
    print(shlex.join(command), flush=True)
    if not args.dry_run:
        env = os.environ.copy()
        env.setdefault('OPENBLAS_NUM_THREADS', '1')
        return subprocess.call(command, cwd=ROOT, env=env)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
