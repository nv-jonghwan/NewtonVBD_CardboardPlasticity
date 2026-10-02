"""Physical sequence and shape checks for the 15mm equivalent multilayer board."""
import csv,json
from pathlib import Path
import numpy as np
from cardboard import ROOT
out=ROOT/'outputs/board15'
read=lambda name:json.loads((out/name).read_text())
rows=list(csv.DictReader((out/'state.csv').open()))
a={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
end=lambda phase,key:float(a[key][a['phase']==phase][-1])
m=read('parameters.json');shape=read('shape_retention.json');flat=read('display_flatness.json');motion=read('panel_flutter.json')
old=json.loads((ROOT/'outputs/realtime/parameters.json').read_text())
D=lambda p:p['bendingMD']*(p['thickness']/p['bendingReferenceThickness'])**p['bendingThicknessExponent']
checks={
 'thickness_15mm':abs(m['thickness']-.015)<1e-7,
 'areal_mass_tripled':abs(m['arealDensity']/old['arealDensity']-3)<1e-5,
 'membrane_tripled':abs(m['membraneArea']/old['membraneArea']-3)<1e-5,
 'bending_27_times':abs(D(m)/D(old)-27)<1e-4,
 'same_outer_fiber_yield_strain':abs(m['thickness']*m['yieldCurvature']-old['thickness']*old['yieldCurvature'])<1e-7,
 'all_phases_complete':set(a['phase'])==set(range(8)) and abs(a['t'][-1]-16)<.02,
 'no_pregrasp_plasticity':bool(np.all(a['plastic_hinges'][a['phase']<3]==0)),
 'lift_over_5cm':end(4,'min_z_m')-end(2,'min_z_m')>.05,
 'further_compression_over_10mm':end(4,'actual_gap_m')-end(5,'actual_gap_m')>.01,
 'permanent_folds_during_squeeze':end(5,'max_plastic_angle_rad')>.1 and end(5,'plastic_hinges')>end(4,'plastic_hinges'),
 'lands_with_half_thickness_clearance':abs(end(7,'min_z_m')-(.65+m['thickness']/2))<.002,
 'fingers_clear_after_release':end(7,'left_contact_N')<.01 and end(7,'right_contact_N')<.01,
 'retains_90_percent_volume':shape['samples']['16.0']['volume_fraction_of_initial']>.90,
 'late_volume_change_under_1_percent':abs(shape['late_volume_change_fraction'])<.01,
 'late_area_change_under_1_percent':abs(shape['late_area_change_fraction'])<.01,
 'over_90_percent_display_locally_flat':min(flat['samples'][t]['flat_area_fraction'] for t in ['12.0','16.0'])>.90,
 'final_speed_under_2mm_s':end(7,'max_speed_m_s')<.002,
 'late_panel_internal_motion_under_half_mm_s':motion['late']['mean_m_s']<.0005,
 'finite_metrics':all(np.isfinite(v).all() for v in a.values()),
}
with np.load(out/'trajectory.npz') as data:checks['finite_geometry']=bool(np.isfinite(data['points']).all())
r=dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},
       lift_m=end(4,'min_z_m')-end(2,'min_z_m'),crush_gap_m=end(5,'actual_gap_m'),
       permanent_fold_degrees=float(np.rad2deg(end(7,'max_plastic_angle_rad'))),
       final_speed_mm_s=end(7,'max_speed_m_s')*1000,
       note='Qualitative15mm equivalent multilayer board, not measured flute/core behavior or quantitative convergence. The global translation energy block is a project extension, not upstream Newton.')
(out/'report.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
raise SystemExit(0 if r['passed'] else 1)
