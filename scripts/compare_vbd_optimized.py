"""Full-order scheduling comparison using the existing trajectory screen limits."""
import argparse,csv,json
from pathlib import Path
import numpy as np


def read(folder):
    folder=Path(folder)
    data={name:json.loads((folder/(name+'.json')).read_text()) for name in ['performance','run_config','validation']}
    for name in ['trajectory','final_material_state']:
        with np.load(folder/(name+'.npz')) as f:data[name]={k:f[k].copy() for k in f.files}
    rows=list(csv.DictReader((folder/'state.csv').open()))
    data['rows']={k:np.asarray([float(row[k]) for row in rows]) for k in rows[0]}
    return data


def compare(baseline,candidate):
    b,c=read(baseline),read(candidate)
    bt,ct=b['trajectory'],c['trajectory']
    assert bt['points'].shape==ct['points'].shape and np.allclose(bt['t'],ct['t'],atol=1e-9)
    delta=np.linalg.norm(bt['points'].astype(float)-ct['points'],axis=2)
    work=[r['rows']['plastic_work_J'][-1] for r in [b,c]]
    work_error=abs(work[1]-work[0])/max(abs(work[0]),1e-9)
    m=c['final_material_state'];cv=c['rows']
    history=all(np.isfinite(v).all() for v in m.values()) and np.all(m['plastic_work']>=0) and np.all(np.diff(cv['plastic_work_J'])>=-1e-5) and np.all(m['accumulated_angle']>=np.abs(m['plastic_angle'])-1e-5) and np.all((m['damage']>=0)&(m['damage']<=.82001))
    checks=dict(same_scene=b['run_config']['scene_sha256']==c['run_config']['scene_sha256'],
                same_device=b['run_config']['device']==c['run_config']['device'],
                same_mesh_and_iteration_budget=all(b['run_config'][k]==c['run_config'][k] for k in ['iterations','substeps','physical_vertices','physical_triangles']),
                complete_fresh_runs=all(r['performance']['error'] is None and abs(r['performance']['duration_s']-16)<.02 for r in [b,c]),
                no_new_sequence_failures=all(not passed or c['validation']['checks'][name] for name,passed in b['validation']['checks'].items()),
                valid_plastic_history=bool(history),
                max_frame_position_rms_under_2mm=float(np.sqrt(np.mean(delta**2,axis=1)).max())<.002,
                max_vertex_position_error_under_10mm=float(delta.max())<.01,
                plastic_work_difference_under_10percent=work_error<.1,
                lift_difference_under_5mm=abs(b['validation']['lift_m']-c['validation']['lift_m'])<.005,
                loaded_gap_difference_under_5mm=abs(b['validation']['loaded_crush_gap_m']-c['validation']['loaded_crush_gap_m'])<.005)
    phase_times={}
    for phase in range(8):
        phase_times[str(phase)]={}
        for label,r in [('baseline',b),('candidate',c)]:
            x=np.asarray(r['performance']['frame_times'])[r['rows']['phase']==phase]
            phase_times[str(phase)][label]=dict(total_s=float(x.sum()),median_ms=float(np.median(x)*1000))
    mode_times={}
    for mode in [0,1,2]:
        mode_times[str(mode)]={}
        for label,r in [('baseline',b),('candidate',c)]:
            x=np.asarray(r['performance']['frame_times'])[r['rows']['solver_mode']==mode]
            mode_times[str(mode)][label]=dict(n=len(x),median_ms=float(np.median(x)*1000) if len(x) else None)
    return dict(physics_screen_passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},
                baseline=str(baseline),candidate=str(candidate),
                baseline_wall_s=b['performance']['wall_time_s'],candidate_wall_s=c['performance']['wall_time_s'],
                observed_speedup=b['performance']['wall_time_s']/c['performance']['wall_time_s'],
                max_frame_position_rms_m=float(np.sqrt(np.mean(delta**2,axis=1)).max()),
                max_vertex_error_m=float(delta.max()),relative_plastic_work_error=float(work_error),
                baseline_sequence=b['validation'],candidate_sequence=c['validation'],phase_times=phase_times,mode_times=mode_times,
                method='Matched world-space vertices; fixed prior 2mm RMS/10mm max,10% plastic work,5mm lift/gap screening limits. Compare baseline repeats separately to expose floating-point/contact sensitivity. Shared GPU observations; identical physical budgets.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline',required=True);p.add_argument('--candidate',required=True);a=p.parse_args()
    result=compare(a.baseline,a.candidate)
    target=Path(a.candidate)/('comparison_'+Path(a.baseline).name+'.json')
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
