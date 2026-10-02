"""Plot actual world-space side motion and active solver time; no motion filtering."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--candidate',required=True);p.add_argument('--baseline',default='outputs/robotiq_creased');a=p.parse_args()
fig,axes=plt.subplots(1,2,figsize=(11,4.1),layout='constrained');times=[]
for path,label in [(a.baseline,'15 mm baseline'),(a.candidate,'10 mm candidate')]:
    out=Path(path)
    with np.load(out/'trajectory.npz') as d:t=d['t'];q=d['points'].astype(float)
    r=q[0]-q[0].mean(0);h=abs(r).max(0);side=(abs(abs(r[:,0])-h[0])<1e-5)|(abs(abs(r[:,1])-h[1])<1e-5)
    motion=np.linalg.norm(q[:,side]-q[-1,side],axis=2).max(1)*1000
    mask=t>=13-1e-8;axes[0].plot(t[mask],motion[mask],label=label)
    rows=list(csv.DictReader((out/'state.csv').open()));times.append(float(rows[-1]['active_wall_s']))
axes[0].axhline(.1,color='gray',ls='--',label='0.1 mm acceptance');axes[0].set_yscale('symlog',linthresh=.1)
axes[0].set(xlabel='Simulation time (s)',ylabel='Maximum side displacement from final (mm)',title='World-space motion of every side vertex');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
bars=axes[1].bar(['15 mm baseline','10 mm candidate'],times,color=['#777777','#267eae']);axes[1].bar_label(bars,fmt='%.1f s');axes[1].set(ylabel='Active calculation time (s)',title='Full 16-second scenario');axes[1].set_ylim(0,max(times)*1.15)
fig.suptitle('Requested gauge change + solver changes; identical mesh resolution',fontsize=11)
fig.savefig(Path(a.candidate)/'supported_rest_comparison.png',dpi=160)
