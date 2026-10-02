"""Additional thick-panel checks; scene sequence/plate checks are separate."""
import argparse,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/thick16_stronger');args=p.parse_args();out=root/args.output
ring=json.loads((root/'outputs/thick16_ringdown/ringdown.json').read_text())
new=json.loads((out/'panel_flutter.json').read_text());old=json.loads((root/'outputs/plate_crease/panel_flutter.json').read_text())
config=json.loads((out/'run_config.json').read_text());material=json.loads((out/'parameters.json').read_text())
checks=dict(newton_1_6=config['newton']=='1.6.0',thickness_5mm=abs(material['thickness']-.005)<1e-7,relaxation_enabled=material['bendingRelaxationTime']>0,
 ringdown_late_below_one_percent=ring['thick']['late_peak_fraction_of_initial']<.01,
 ringdown_at_least_tenfold_lower=ring['thick']['late_peak_fraction_of_initial']<ring['previous']['late_peak_fraction_of_initial']*.1,
 late_panel_deformation_speed_below_point1_mm_s=new['late']['mean_m_s']<.0001,
 late_panel_deformation_speed_at_least_tenfold_lower=new['late']['mean_m_s']<old['late']['mean_m_s']*.1)
report=dict(passed=all(checks.values()),checks=checks,held_mean_speed_m_s=new['held']['mean_m_s'],previous_held_mean_speed_m_s=old['held']['mean_m_s'],note='Held motion is reported separately under the changed grasp/squeeze. Do not claim every phase improved. Ringdown fixture and late settling demonstrate reduced free bending vibration.')
(out/'thick_behavior_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
