"""Summarize support-only dissipation and unfiltered post-drop panel motion."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from cardboard import ROOT
p=argparse.ArgumentParser(__doc__);p.add_argument('directories',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
report={}
for name in a.directories:
 folder=ROOT/name;rows=list(csv.DictReader((folder/'state.csv').open()));v={k:np.asarray([float(r[k]) for r in rows]) for k in rows[0]}
 z=np.load(folder/'trajectory.npz');q=z['points'].astype(float);t=z['t'];local=q[0]-q[0].mean(0);half=np.max(abs(local),axis=0)
 side=np.isclose(abs(local[:,0]),half[0],atol=1e-5)|np.isclose(abs(local[:,1]),half[1],atol=1e-5)
 state=np.load(folder/'final_material_state.npz');speed=float(np.linalg.norm(state['velocity'],axis=1).max())
 windows={}
 for start in [13,14,15]:
  pts=q[t>=start-1e-6][:,side]
  windows[str(start)]={'max_excursion_from_final_mm':float(np.linalg.norm(pts-pts[-1],axis=2).max()*1000),'max_frame_step_mm':float(np.linalg.norm(np.diff(pts,axis=0),axis=2).max()*1000)}
 sleeping=v['box_sleeping'].astype(bool);events=[{'t':float(v['t'][i]),'sleeping':bool(sleeping[i])} for i in range(len(rows)) if sleeping[i]!=(sleeping[i-1] if i else False)]
 changes=np.diff(v['active_wall_s'],prepend=0)
 report[name]={'end_speed_mm_s':speed*1000,'postdrop_side_motion':windows,'sleep_events':events,'supported_first_t':float(v['t'][v['solver_mode']==1][0]) if (v['solver_mode']==1).any() else None,'mode_frames':{str(int(k)):int((v['solver_mode']==k).sum()) for k in np.unique(v['solver_mode'])},'wall_s':float(v['active_wall_s'][-1]),'post12_wall_s':float(changes[v['t']>12].sum()),'material_checks':{'finite':all(np.isfinite(state[k]).all() for k in state.files),'nonnegative_monotonic_work':bool((state['plastic_work']>=0).all() and (np.diff(v['plastic_work_J'])>=-1e-5).all()),'valid_history':bool((state['accumulated_angle']>=abs(state['plastic_angle'])-1e-5).all()),'damage_bounded':bool(((state['damage']>=0)&(state['damage']<1)).all())},'no_loaded_sleep':bool(not(sleeping&np.isin(v['phase'],[3,4,5])).any())}
out=ROOT/a.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
