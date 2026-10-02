"""User-requested deep crush: loaded jaw gap near half the original box width.

This intentionally replaces the gentle-crush >90% volume/flatness objective.
Material and thickness stay identical; large folds are now the desired outcome.
"""
import csv,json,argparse
import numpy as np
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--live-pid',type=int);args=p.parse_args()
out=ROOT/'outputs/board15_half'
read=lambda name:json.loads((out/name).read_text())
path=ROOT/f'outputs/live/state-{args.live_pid}.csv' if args.live_pid else out/'state.csv'
rows=list(csv.DictReader(path.open()));a={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
end=lambda phase,key:float(a[key][a['phase']==phase][-1])
m=read('parameters.json');old=json.loads((ROOT/'outputs/board15_kd85/parameters.json').read_text())
checks={
 'unchanged_15mm_material':m==old and abs(m['thickness']-.015)<1e-7,
 'all_phases_complete':set(a['phase'])==set(range(8)) and abs(a['t'][-1]-16)<.02,
 'no_pregrasp_plasticity':bool(np.all(a['plastic_hinges'][a['phase']<3]==0)),
 'lift_over_5cm':end(4,'min_z_m')-end(2,'min_z_m')>.05,
 'actual_gap_within_15mm_of_half_280mm_width':abs(end(5,'actual_gap_m')-.14)<.015,
 'over_100mm_additional_closure':end(4,'actual_gap_m')-end(5,'actual_gap_m')>.10,
 'large_permanent_folds':end(5,'max_plastic_angle_rad')>.5 and end(5,'plastic_hinges')>end(4,'plastic_hinges'),
 'lands_with_thickness_clearance':abs(end(7,'min_z_m')-(.65+m['thickness']/2))<.002,
 'fingers_clear_after_release':end(7,'left_contact_N')<.01 and end(7,'right_contact_N')<.01,
 'final_global_speed_under_5mm_s':end(7,'max_speed_m_s')<.005,
 'finite_metrics':all(np.isfinite(v).all() for v in a.values()),
}
state=ROOT/f'outputs/live/state-{args.live_pid}.npz' if args.live_pid else out/'state.npz'
with np.load(state) as d:checks['finite_final_geometry']=bool(np.isfinite(d['points']).all())
if not args.live_pid:
 shape=read('shape_retention.json');flutter=read('panel_flutter.json')
 checks.update(retains_75_percent_enclosed_volume=shape['samples']['16.0']['volume_fraction_of_initial']>.75,
               late_volume_change_under_1_percent=abs(shape['late_volume_change_fraction'])<.01,
               late_panel_motion_under_half_mm_s=flutter['late']['mean_m_s']<.0005)
r=dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},
       actual_crush_gap_mm=end(5,'actual_gap_m')*1000,opening_fraction_of_original_width=end(5,'actual_gap_m')/.28,
       lift_mm=(end(4,'min_z_m')-end(2,'min_z_m'))*1000,final_speed_mm_s=end(7,'max_speed_m_s')*1000,
       motor_force_cap_N=end(5,'motor_limit_N'),plastic_hinges_after_squeeze=end(5,'plastic_hinges'),
       qualification='15mm equivalent multilayer board. Prescribed UR arm, dynamic custom high-force fingers. Not qualified hardware loads or calibrated corrugated material; large folds and stress accuracy are not converged.')
name='live_sequence_validation.json' if args.live_pid else 'report.json'
(out/name).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
raise SystemExit(0 if r['passed'] else 1)
