"""Actual Newton plate free-vibration test: 1 mm deflection, no gravity/contact.

Boundary particles are fixed in both comparisons. Fit the excited sine mode;
record at 240 Hz, with solver dt 1/960 s. No velocity damping/post-filtering.
"""
import argparse,json
from pathlib import Path
import numpy as np
import warp as wp
import newton
from pxr import Usd
from cardboard.usd_utils import register
from cardboard import ROOT
p=argparse.ArgumentParser();p.add_argument('--device',default='cuda:0');a=p.parse_args();wp.set_device(a.device);register()
output=ROOT/'outputs/thick16_ringdown';output.mkdir(exist_ok=True)
results={}
for label,asset in [('previous','cardboard_plate_crease.usda'),('thick','cardboard_thick16.usda')]:
 stage=Usd.Stage.Open(str(ROOT/'assets'/asset));mat=stage.GetPrimAtPath('/Box/Materials/Cardboard');get=lambda key:mat.GetAttribute('cardboard:'+key).Get()
 n=16;size=np.array([.28,.208]);verts=np.array([[i/n*size[0],j/n*size[1],0.] for i in range(n+1) for j in range(n+1)],np.float32)
 faces=[]
 for i in range(n):
  for j in range(n):
   x=i*(n+1)+j;y=x+n+1
   faces.extend([[x,y,y+1],[x,y+1,x+1]])
 b=newton.ModelBuilder();b.add_cloth_mesh(pos=wp.vec3(0),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),vertices=verts,indices=np.array(faces).ravel(),density=get('arealDensity'),tri_ke=get('membraneShear'),tri_ka=get('membraneArea'),tri_kd=get('membraneDamping'),edge_ke=1.,edge_kd=0.)
 edges=np.asarray(b.edge_indices);md=get('bendingMD');cd=get('bendingCD');scale=(get('thickness')/get('bendingReferenceThickness'))**get('bendingThicknessExponent');tau=get('bendingRelaxationTime')
 for k,(op0,op1,i,j) in enumerate(edges):
  if op0<0 or op1<0:continue
  edge=verts[j]-verts[i];length=np.linalg.norm(edge);h=(np.linalg.norm(np.cross(verts[op0]-verts[i],edge))+np.linalg.norm(np.cross(verts[op1]-verts[i],edge)))/(2*length)
  ke=(cd+(md-cd)*(edge[1]/length)**2)*scale/h
  b.edge_bending_properties[k]=(float(ke),float(tau*ke if tau>0 else get('bendingDamping')))
 boundary=(verts[:,0]<1e-7)|(verts[:,0]>size[0]-1e-7)|(verts[:,1]<1e-7)|(verts[:,1]>size[1]-1e-7)
 for i in np.flatnonzero(boundary):b.particle_mass[i]=0.
 b.color(include_bending=True);model=b.finalize();model.gravity.zero_()
 solver=newton.solvers.SolverVBD(model,iterations=64,particle_enable_self_contact=False,rigid_compliant_alm=False)
 states=[model.state(),model.state()];control=model.control();mode=np.sin(np.pi*verts[:,0]/size[0])*np.sin(np.pi*verts[:,1]/size[1]);initial=verts.copy();initial[:,2]=.001*mode
 for state in states:state.particle_q.assign(initial);state.particle_qd.zero_()
 def step4():
  for _ in range(4):
   states[0].clear_forces();solver.step(states[0],states[1],control,None,1/960);states.reverse()
 step4()
 for state in states:state.particle_q.assign(initial);state.particle_qd.zero_()
 with wp.ScopedCapture() as capture:step4()
 times=[0.];amplitudes=[.001];rms=[float(np.sqrt(np.mean(initial[:,2]**2)))];max_displacements=[.001]
 for i in range(120):
  wp.capture_launch(capture.graph);z=states[0].particle_q.numpy()[:,2]
  times.append((i+1)/240);amplitudes.append(float(z@mode/(mode@mode)));rms.append(float(np.sqrt(np.mean(z*z))));max_displacements.append(float(np.max(abs(z))))
 ts=np.array(times);late=np.array(max_displacements)[ts>=.25]
 result=dict(time_s=times,mode_amplitude_m=amplitudes,displacement_rms_m=rms,max_displacement_m=max_displacements,late_peak_fraction_of_initial=float(late.max()/.001),effective_bending_MD_Nm=float(md*scale),relaxation_time_s=tau)
 results[label]=result;print(label,'late peak fraction',result['late_peak_fraction_of_initial'],flush=True)
(output/'ringdown.json').write_text(json.dumps(results,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(7,3.6),layout='constrained')
for label,r in results.items():ax.plot(r['time_s'],np.array(r['mode_amplitude_m'])*1000,label=label)
ax.set(xlabel='Time (s)',ylabel='Mode amplitude (mm)',title='1 mm release: bending vibration of a fixed-edge plate');ax.legend();ax.grid(alpha=.25)
fig.savefig(output/'ringdown.png',dpi=170)
