"""Qualitative real-time mode contract, explicitly separate from dense reference."""
import json
from pathlib import Path
from cardboard import ROOT
out=ROOT/'outputs/realtime'
read=lambda p:json.loads(p.read_text())
metrics=read(out/'panel_metrics.json');shape=read(out/'shape_retention.json');surface=read(out/'display_flatness.json');flutter=read(out/'panel_flutter.json');perf=read(out/'live_performance.json');sequence=read(out/'report.json')
checks={
 'full_contact_sequence':sequence['passed'],
 'same_cardboard_material':read(out/'parameters.json')==read(ROOT/'outputs/thick16_stronger/parameters.json'),
 'retains_at_least_80_percent_volume':shape['samples']['16.0']['volume_fraction_of_initial']>.8,
 'late_volume_change_under_3_percent':abs(shape['late_volume_change_fraction'])<.03,
 'surface_area_change_under_1_percent':abs(shape['samples']['16.0']['area_fraction_of_initial']-1)<.01,
 'final_speed_under_2mm_s':metrics['final']['max_speed_m_s']<.002,
 'late_panel_deformation_under_point1mm_s':flutter['late']['mean_m_s']<.0001,
 'permanent_crease_history':metrics['final']['max_plastic_angle_rad']>.15,
 'same_dense_display_topology':surface['display_triangles']==57280,
 'majority_display_area_flat_after_crush':surface['samples']['12.0']['flat_area_fraction']>.75,
 'majority_display_area_flat_after_drop':surface['samples']['16.0']['flat_area_fraction']>.75,
 'physics_at_least_95_percent_realtime':perf['worker_realtime_factor']>=.95,
 'complete_gui_16s':abs(perf['sim_duration_s']-16)<.02,
 'gui_receives_at_least_25_frames_s':perf['delivered_fps']>=25,
 'diagnostic_trace_survives_shutdown':'GUI_EXCEPTION' in (ROOT/'outputs/gui_error_reporting_test.log').read_text() and 'exit_code=1' in (ROOT/'outputs/gui_error_reporting_test.log').read_text(),
}
r=dict(passed=all(checks.values()),checks=checks,performance=perf,qualification='Reduced386-node shell and prescribed UR arm; fingers remain dynamic. Same material parameters and57280 display triangles; deformation and force accuracy are not equivalent to3458-node/96-iteration reference. Fixed display topology makes visual flatness comparable across physics meshes. Prior physical-edge3deg flatness is retained in raw reports but not used across different edge lengths. Not calibrated or quantitatively converged.')
(ROOT/'outputs/realtime_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
