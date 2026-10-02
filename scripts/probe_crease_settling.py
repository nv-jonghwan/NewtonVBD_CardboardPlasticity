"""Short solver probes from identical saved material state, not exact restarts."""
import argparse,json,time
from pathlib import Path
import numpy as np
from pxr import Usd,Sdf
from scipy.spatial.transform import Rotation
from cardboard import ROOT
from cardboard.scenario import PickCrushDrop
from cardboard.usd_utils import register

p=argparse.ArgumentParser();p.add_argument('--iterations',type=int,required=True);p.add_argument('--substeps',type=int,required=True)
p.add_argument('--source-output',default='outputs/robotiq_creased_i16s64')
p.add_argument('--duration',type=float,default=.5);a=p.parse_args();register()
out=ROOT/'outputs/crease_refinement';scene=out/f'probe_i{a.iterations}s{a.substeps}.usda'
checkpoint=ROOT/a.source_output
source_path=ROOT/json.loads((checkpoint/'run_config.json').read_text())['scene']
source=Usd.Stage.Open(str(source_path))
stage=Usd.Stage.CreateNew(str(scene));stage.GetRootLayer().subLayerPaths=[str(source_path)]
# Stage-wide unit/axis metadata belongs to the root/session layer.
for key in ['metersPerUnit','upAxis','timeCodesPerSecond']:stage.SetMetadata(key,source.GetMetadata(key))
settings=stage.OverridePrim('/World/Physics');settings.GetAttribute('cardboard:iterations').Set(a.iterations);settings.GetAttribute('cardboard:substeps').Set(a.substeps)
stage.GetRootLayer().Save()
s=PickCrushDrop(scene_path=scene)
with np.load(checkpoint/'final_material_state.npz') as d:
    for state in [s.a,s.b]:
        state.particle_q.assign(d['points']);state.particle_qd.assign(d['velocity'])
        for field in ['plastic_angle','accumulated_angle','plastic_work','damage']:getattr(state.cardboard,field).assign(d[field])
    s.model.edge_rest_angle.assign(s.reference+d['plastic_angle'])
    props=s.model.edge_bending_properties.numpy();props[:,0]=s.ke*(1-d['damage']);props[:,1]=s.bend_relaxation*props[:,0]
    s.model.edge_bending_properties.assign(props)
    s.time=float(d['t'])
with np.load(checkpoint/'trajectory.npz') as d:
    for state in [s.a,s.b]:state.body_q.assign(d['body_q'][-1]);state.body_qd.zero_()
s.command(s.time)
matrices=np.asarray(s.ik.fk(s.command_joint_q))
s.kin_previous=np.concatenate([matrices[:,:3,3],Rotation.from_matrix(matrices[:,:3,:3]).as_quat()],axis=1).astype(np.float32)
positions=[];velocities=[];begin=time.perf_counter()
for _ in range(round(a.duration*s.fps)):
    s.advance();positions.append(s.a.particle_q.numpy());velocities.append(s.a.particle_qd.numpy())
speed=np.linalg.norm(np.asarray(velocities),axis=2);positions=np.asarray(positions)
report=dict(source_output=a.source_output,iterations=a.iterations,substeps=a.substeps,duration_s=a.duration,wall_s=time.perf_counter()-begin,
            final_max_speed_m_s=float(speed[-1].max()),late_peak_speed_m_s=float(speed[len(speed)//2:].max()),
            late_rms_speed_m_s=float(np.sqrt(np.mean(speed[len(speed)//2:]**2))),
            mean_frame_motion_m_s=float(np.linalg.norm(np.diff(positions[len(positions)//2:],axis=0),axis=2).mean()*s.fps),
            qualification='Same particle position/velocity/plastic history; rigid velocities reset and solver multipliers/contact caches rebuilt. Diagnostic only; chosen settings require full sequence rerun.')
(out/f'probe_i{a.iterations}s{a.substeps}.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
