"""Check measured contact/lift/release sequence, never prescribed mesh animation."""
import argparse,csv,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/scenario_stronger');p.add_argument('--baseline',default='outputs/scenario_validation_final');p.add_argument('--comparison-mode',choices=['stronger','report'],default='stronger');args=p.parse_args()
out=root/args.output
rows=list(csv.DictReader((out/'state.csv').open()))
a={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
def end(phase,key):return float(a[key][a['phase']==phase][-1])
checks={
 'all_eight_phases':set(a['phase'])==set(range(8)),
 'complete_16_seconds':abs(a['t'][-1]-16)<.02,
 'no_pregrasp_plasticity':bool(np.all(a['plastic_hinges'][a['phase']<3]==0)),
 'box_lifted_over_5cm':end(4,'min_z_m')>.70,
 'stronger_squeeze':end(5,'motor_limit_N')>end(3,'motor_limit_N'),
 'squeeze_reduces_gap':end(5,'actual_gap_m')<end(4,'actual_gap_m')-.005,
 'plasticity_increases_before_drop':end(5,'plastic_hinges')>end(4,'plastic_hinges'),
 'lands_on_table':abs(end(7,'min_z_m')-.6515)<.003,
 'fingers_clear_final_box':end(7,'left_contact_N')<.01 and end(7,'right_contact_N')<.01,
 'permanent_history':end(7,'plastic_hinges')>0,
 'finite_metrics':all(np.isfinite(v).all() for v in a.values()),
}
with np.load(out/'state.npz') as state:checks['finite_geometry']=bool(np.isfinite(state['points']).all())
baseline=root/args.baseline/'state.csv'
comparison={}
if baseline.exists() and baseline.resolve()!=(out/'state.csv').resolve():
    previous=list(csv.DictReader(baseline.open()));before=[r for r in previous if int(r['phase'])==5][-1]
    comparison={'previous_crush_gap_m':float(before['actual_gap_m']),'current_crush_gap_m':end(5,'actual_gap_m'),
                'previous_crush_plastic_hinges':int(before['plastic_hinges']),'current_crush_plastic_hinges':end(5,'plastic_hinges')}
    if args.comparison_mode=='stronger':
        checks['at_least_20mm_more_closure']=end(5,'actual_gap_m')<float(before['actual_gap_m'])-.020
        checks['at_least_triple_predrop_plastic_hinges']=end(5,'plastic_hinges')>3*int(before['plastic_hinges'])
checks={k:bool(v) for k,v in checks.items()}
report={'passed':all(checks.values()),'checks':checks,'comparison':comparison,'lift_clearance_m':end(4,'min_z_m')-.65,
 'crush_gap_reduction_m':end(4,'actual_gap_m')-end(5,'actual_gap_m'),
 'plastic_hinges_after_squeeze':end(5,'plastic_hinges'),'plastic_hinges_final':end(7,'plastic_hinges'),
 'final_max_speed_m_s':end(7,'max_speed_m_s'),
 'qualification':'Qualitative contact sequence; not material calibration or numerical convergence.'}
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
