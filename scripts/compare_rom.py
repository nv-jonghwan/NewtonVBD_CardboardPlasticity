"""Fresh ROM/FOM comparison with fixed numerical screening gates."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_run(folder):
    folder = Path(folder)
    with np.load(folder / 'trajectory.npz') as data:
        trajectory = {key: data[key].copy() for key in ['t', 'points']}
    with np.load(folder / 'final_material_state.npz') as data:
        material = {key: data[key].copy() for key in data.files}
    with (folder / 'state.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    values = {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}
    return dict(trajectory=trajectory, material=material, values=values,
                performance=json.loads((folder / 'performance.json').read_text()),
                config=json.loads((folder / 'run_config.json').read_text()),
                sequence=json.loads((folder / 'validation.json').read_text()))


def compare(baseline, candidate):
    b, c = read_run(baseline), read_run(candidate)
    bt, ct = b['trajectory'], c['trajectory']
    if bt['points'].shape != ct['points'].shape or not np.allclose(bt['t'], ct['t'], atol=1e-9):
        raise ValueError('Complete runs with identical sample times and mesh are required')
    delta = np.linalg.norm(ct['points'].astype(float) - bt['points'], axis=2)
    bv, cv = b['values'], c['values']
    work_error = abs(cv['plastic_work_J'][-1] - bv['plastic_work_J'][-1]) / max(abs(bv['plastic_work_J'][-1]), 1e-9)
    m = c['material']
    history_valid = (all(np.isfinite(value).all() for value in m.values())
                     and np.all((m['damage'] >= 0) & (m['damage'] <= .82001))
                     and np.all(m['plastic_work'] >= 0)
                     and np.all(m['accumulated_angle'] >= np.abs(m['plastic_angle']) - 1e-5)
                     and np.all(np.diff(cv['plastic_work_J']) >= -1e-5))
    counts = c['performance']['rom_counts']
    speedup = b['performance']['wall_time_s'] / c['performance']['wall_time_s']
    phase_times = {}
    for phase in range(8):
        phase_times[str(phase)] = {
            label: float(np.asarray(run['performance']['frame_times'])[run['values']['phase'] == phase].sum())
            for label, run in [('baseline', b), ('candidate', c)]
        }
    checks = dict(
        same_scene=b['config']['scene_sha256'] == c['config']['scene_sha256'],
        complete_fresh_runs=all(run['performance']['error'] is None and abs(run['performance']['duration_s']-16) < .02 for run in [b,c]),
        candidate_sequence=c['sequence']['passed'],
        valid_full_mesh_plastic_history=history_valid,
        actual_reduced_corrections=counts is not None and counts[1] > 0,
        counters_partition_attempts=counts is not None and counts[0] == sum(counts[1:]),
        max_frame_position_rms_under_2mm=float(np.sqrt(np.mean(delta**2,axis=1)).max()) < .002,
        max_vertex_position_error_under_10mm=float(delta.max()) < .01,
        plastic_work_difference_under_10percent=float(work_error) < .1,
        lift_difference_under_5mm=abs(b['sequence']['lift_m']-c['sequence']['lift_m']) < .005,
        loaded_gap_difference_under_5mm=abs(b['sequence']['loaded_crush_gap_m']-c['sequence']['loaded_crush_gap_m']) < .005,
        at_least_10percent_speedup=speedup > 1.1,
    )
    return dict(passed=all(checks.values()), checks={k:bool(v) for k,v in checks.items()},
                baseline=str(baseline), candidate=str(candidate),
                baseline_wall_s=b['performance']['wall_time_s'], candidate_wall_s=c['performance']['wall_time_s'], speedup=speedup,
                baseline_sequence=b['sequence'], candidate_sequence=c['sequence'],
                max_frame_position_rms_m=float(np.sqrt(np.mean(delta**2,axis=1)).max()),
                max_vertex_position_error_m=float(delta.max()), final_position_rms_m=float(np.sqrt(np.mean(delta[-1]**2))),
                relative_plastic_work_error=float(work_error),rom_counts=counts,
                phase_compute_seconds=phase_times,
                accepted_fraction=counts[1]/max(counts[0],1) if counts else None,
                method='World-space matched vertices, no rigid alignment. Fixed engineering screening limits, not calibrated physical accuracy or convergence guarantees. Existing sequence gates unchanged; strict settling reported separately. Timings on shared GPUs are observational.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    args = parser.parse_args()
    report = compare(args.baseline, args.candidate)
    (Path(args.candidate) / 'rom_comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
