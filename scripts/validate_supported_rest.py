"""Visible side-panel motion and physical sleep/wake regression, in world space."""
import argparse,csv,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--baseline',default='outputs/robotiq_creased');a=p.parse_args()

def measure(folder):
    out=Path(folder)
    with np.load(out/'trajectory.npz') as d:
        t=d['t'];q=d['points'].astype(float)
    r=q[0]-q[0].mean(0);half=abs(r).max(0)
    side=(abs(abs(r[:,0])-half[0])<1e-5)|(abs(abs(r[:,1])-half[1])<1e-5)
    result={}
    for start in [13.,13.5,14.,15.]:
        x=q[t>=start-1e-8][:,side]
        result[str(start)]={'max_excursion_from_final_m':float(np.linalg.norm(x-x[-1],axis=2).max()),
                           'max_frame_step_m':float(np.linalg.norm(np.diff(x,axis=0),axis=2).max())}
    rows=list(csv.DictReader((out/'state.csv').open()))
    sleeping=np.array([int(row.get('box_sleeping',0)) for row in rows],bool)
    rt=np.array([float(row['t']) for row in rows]);phase=np.array([int(row['phase']) for row in rows])
    transitions=[{'t':float(rt[i]),'sleeping':bool(sleeping[i])} for i in range(len(rows)) if sleeping[i]!=(sleeping[i-1] if i else False)]
    with np.load(out/'final_material_state.npz') as d:final_speed=float(np.linalg.norm(d['velocity'],axis=1).max())
    floor_height=float(rows[-1]['min_z_m'])
    unsupported=np.array([float(row['min_z_m'])>floor_height+.001 for row in rows])
    return result,transitions,sleeping,rt,phase,unsupported,final_speed,float(rows[-1]['active_wall_s'])

new,events,sleep,t,phase,unsupported,speed,wall=measure(a.output);old,_,_,_,_,_,_,old_wall=measure(a.baseline)
checks={'initial_supported_sleep':bool(np.any(sleep&(t<4))),
        'awake_through_loaded_grasp_lift_crush':not bool(np.any(sleep&np.isin(phase,[3,4,5]))),
        'never_sleep_while_airborne':not bool(np.any(sleep&unsupported)),
        'final_supported_sleep':bool(sleep[-1]),
        'no_visible_side_motion_after_14s':new['14.0']['max_excursion_from_final_m']<.0001,
        'terminal_velocity_zero':speed==0.,
        'full_run_under_three_minutes':wall<180.,
        'at_least_twice_faster':wall<.5*old_wall}
report={'passed':all(checks.values()),'checks':checks,'sleep_events':events,'baseline_side_motion':old,
        'candidate_side_motion':new,'baseline_wall_s':old_wall,'candidate_wall_s':wall,'speedup':old_wall/wall,
        'sleep_tolerances':{'window_s':.25,'all_vertex_window_excursion_m':.0005,'instantaneous_max_speed_m_s':.02},
        'method':'30Hz world-space motion of every side vertex; no Kabsch/mean-only filtering. Supported sleeping is a numerical rest tolerance; rigid mechanism continues, proximity/load changes wake shell.'}
(Path(a.output)/'supported_rest_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
