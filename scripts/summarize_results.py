"""Generate physics evidence from recorded trajectories and measured contacts."""
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cardboard import ROOT

def rows(name):
    with (ROOT/f'outputs/{name}/metrics.csv').open() as f:return {k:np.array([float(r[k]) for r in values]) for values in [list(csv.DictReader(f))] for k in values[0]}
p=rows('plastic');e=rows('elastic');rp=json.loads((ROOT/'outputs/plastic/report.json').read_text());re=json.loads((ROOT/'outputs/elastic/report.json').read_text());rc=json.loads((ROOT/'outputs/convergence32/report.json').read_text())
a=np.load(ROOT/'outputs/plastic/trajectory.npz');b=np.load(ROOT/'outputs/elastic/trajectory.npz')
checks={'finite':rp['finite'] and re['finite'],'no_unloaded_plasticity':bool(p['plastic_hinges'][p['t']<=1].max()==0),'persistent_plastic_history':rp['plastic_hinges']>20 and rp['plastic_work_J']>0,'elastic_history_zero':re['plastic_hinges']==0 and re['plastic_work_J']==0,'released_gripper_contacts_zero':bool(np.max(p['left_contact_N'][p['t']>8])==0 and np.max(p['right_contact_N'][p['t']>8])==0),'flow_dissipation_monotone':bool(np.min(np.diff(p['plastic_work_J']))>=-1e-6),'final_plastic_velocity_below_1mm_s':rp['final']['max_speed_m_s']<.001,'table_contact_tolerance_1mm':rp['minimum_z_m']>.6505}
# Rotation/translation alignment of final plastic geometry to final elastic geometry.
x=a['points'][-1];y=b['points'][-1];xc=x-x.mean(0);yc=y-y.mean(0);u,_,vt=np.linalg.svd(xc.T@yc);rot=u@vt
if np.linalg.det(rot)<0:u[:,-1]*=-1;rot=u@vt
diff=np.linalg.norm(xc@rot-yc,axis=1)
sensitivity={key:abs(rc[key]-rp[key])/max(abs(rp[key]),1e-10) for key in ['shape_residual_rms_m','plastic_work_J','peak_finger_contact_N']}
report={'qualitative_demo_passed':all(checks.values()),'checks':checks,'plastic_final_rms_m':rp['shape_residual_rms_m'],'elastic_final_rms_m':re['shape_residual_rms_m'],'aligned_plastic_vs_elastic_rms_m':float(np.sqrt(np.mean(diff**2))),'aligned_plastic_vs_elastic_max_m':float(diff.max()),'peak_twist_axis_torque_Nm':float(abs(p['wrist_contact_z_Nm']).max()),'peak_commanded_twist_rad':float(p['twist_command_rad'].max()),'iteration_16_to_32_relative_change':sensitivity,'quantitatively_converged':False,'interpretation':'Permanent constitutive history and unloaded geometry demonstrated. Iteration sensitivity remains material; not a calibrated force prediction.'}
(ROOT/'outputs/validation_summary.json').write_text(json.dumps(report,indent=2))
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig=plt.figure(figsize=(15,9),layout='constrained')
f=a['faces'];centers=a['points'][0].mean(0)
for j,(t,title) in enumerate([(0,'Before loading'),(4,'30 N / finger drive'),(11,'Released: permanent state')]):
    ax=fig.add_subplot(2,3,j+1,projection='3d');idx=abs(a['t']-t).argmin();q=a['points'][idx];faces=q[f]
    pc=Poly3DCollection(faces,facecolor='#b58955',edgecolor='#654b30',linewidth=.17,alpha=.94);ax.add_collection3d(pc)
    ax.set_xlim(centers[0]-.17,centers[0]+.17);ax.set_ylim(-.14,.14);ax.set_zlim(.645,.88);ax.set_box_aspect((.34,.28,.235));ax.view_init(22,-65);ax.set_title(f'{title}\nt = {a["t"][idx]:.1f} s');ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
ax=fig.add_subplot(2,3,4);ax.plot(p['t'],p['left_contact_N'],label='left contact');ax.plot(p['t'],p['right_contact_N'],label='right contact');ax.axhline(30,color='k',linestyle=':',label='motor limit (not contact cap)');ax.set(xlabel='time [s]',ylabel='measured contact force [N]');ax.legend(fontsize=8);ax.grid(alpha=.2)
ax=fig.add_subplot(2,3,5);ax.plot(p['t'],p['plastic_work_J'],label='plastic');ax.plot(e['t'],e['plastic_work_J'],label='elastic control');ax.set(xlabel='time [s]',ylabel='plastic flow dissipation [J]');ax.legend();ax.grid(alpha=.2)
ax=fig.add_subplot(2,3,6);ax.plot(p['t'],p['wrist_contact_z_Nm'],color='#a64b39',label='measured twist-axis torque');ax.axhline(-2,color='k',linestyle=':',label='supervisor threshold');ax.axhline(2,color='k',linestyle=':');ax.set(xlabel='time [s]',ylabel='contact moment about world z [N m]');ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('Newton VBD cardboard shell: actual simulated state and contacts\nIllustrative material; geometry and constitutive history persist after release',fontsize=15)
fig.savefig(ROOT/'outputs/physics_evidence.png',dpi=180);fig.savefig(ROOT/'outputs/physics_evidence.pdf');print(json.dumps(report,indent=2))
