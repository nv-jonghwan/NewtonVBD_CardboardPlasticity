"""Acceptance for this qualitative box demo, in addition to grasp/release checks."""
import argparse,csv,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/plate_balanced');a=p.parse_args();out=Path(a.output)
shape=json.loads((out/'shape_retention.json').read_text());metrics=json.loads((out/'panel_metrics.json').read_text())
rows=list(csv.DictReader((out/'state.csv').open()))
last=metrics['final'];crush=metrics['pre_release'];late=[r for r in rows if float(r['t'])>=14.]
checks={
 'retains_box_volume_after_landing':shape['samples']['16.0']['volume_fraction_of_initial']>.8,
 'late_volume_drift_below_3_percent':abs(shape['late_volume_change_fraction'])<.03,
 'surface_area_change_below_1_percent':abs(shape['samples']['16.0']['area_fraction_of_initial']-1)<.01,
 'final_max_speed_below_20mm_s':last['max_speed_m_s']<.02,
 'visible_permanent_crease_angle':last['max_plastic_angle_rad']>.15,
 'residual_shape_change_over_1mm':shape['samples']['16.0']['shape_change_rms_after_rigid_fit_m']>.001,
 'localized_predrop_plasticity':0<crush['plastic_hinges']<.15*10368,
 'majority_local_panel_area_flat':shape['samples']['16.0']['locally_flat_area_fraction_3deg']>.75,
 'late_plastic_growth_below_1_percent_of_hinges':float(late[-1]['plastic_hinges'])-float(late[0]['plastic_hinges'])<.01*10368,
}
report=dict(passed=all(checks.values()),checks=checks,late_volume_change_fraction=shape['late_volume_change_fraction'],final_max_speed_m_s=last['max_speed_m_s'],qualification='Scene-specific qualitative acceptance; not real-material calibration, solver convergence, or a general guarantee for arbitrary forces.')
(out/'plate_behavior_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
