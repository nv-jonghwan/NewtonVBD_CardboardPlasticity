"""Inject one bridge read exception to verify it survives Kit fast shutdown."""
from pathlib import Path
import runpy,sys,numpy as np
R=Path(__file__).resolve().parents[1]
original=np.load

def fail_snapshot(file,*args,**kwargs):
    if '/outputs/live/state-' in str(file):
        raise RuntimeError('Injected bridge read error for diagnostic verification')
    return original(file,*args,**kwargs)
np.load=fail_snapshot
sys.argv=[str(R/'scripts/live_isaac.py'),'--headless','--scene',str(R/'assets/demo_scene_runtime_n8_fast.usda'),'--duration','.3','--exit-when-done']
runpy.run_path(sys.argv[0],run_name='__main__')
