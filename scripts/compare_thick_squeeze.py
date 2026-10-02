"""Compare measured deformation of identical board material at two squeeze commands."""
import argparse
import json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--output',default='outputs/thick16_stronger')
p.add_argument('--baseline',default='outputs/thick16_final')
a=p.parse_args();out=Path(a.output);base=Path(a.baseline)
b=json.loads((base/'panel_metrics.json').read_text())['pre_release']
n=json.loads((out/'panel_metrics.json').read_text())['pre_release']
keys=['actual_gap_m','plastic_hinges','plastic_work_J','max_plastic_angle_rad','left_contact_N','right_contact_N','motor_limit_N']
checks={
 'at_least_10mm_additional_measured_closure':n['actual_gap_m']<b['actual_gap_m']-.010,
 'increased_pre_release_plastic_work':n['plastic_work_J']>b['plastic_work_J']*1.1,
 'same_board_material':json.loads((base/'parameters.json').read_text())==json.loads((out/'parameters.json').read_text()),
}
r={'passed':all(checks.values()),'checks':checks,'baseline':{k:b[k] for k in keys},'stronger':{k:n[k] for k in keys},'additional_closure_mm':1000*(b['actual_gap_m']-n['actual_gap_m']),'qualification':'Motor limit is not actual contact force; measured reactions are reported separately.'}
(out/'stronger_comparison.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
raise SystemExit(0 if r['passed'] else 1)
