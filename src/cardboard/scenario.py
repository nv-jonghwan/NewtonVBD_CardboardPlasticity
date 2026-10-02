"""Schema-driven contact grasp, lift, plastic squeeze and gravity release."""
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from . import ROOT
from .usd_utils import register
from .simulation import Simulation

PHASES = ['1  Home', '2  Move above box', '2  Lower to box',
          '3  Grasp', '3  Lift', '4  Squeeze / crumple', '5  Release / drop', '5  Settle on table']

def tilt_about_grasp_center(pose, center, angles_degrees):
    """World XYZ tilt about the translated box center, including tool reach."""
    if not np.any(angles_degrees):
        return pose
    rotation=Rotation.from_euler('xyz',angles_degrees,degrees=True).as_matrix()
    result=pose.copy()
    result[:3,:3]=rotation@pose[:3,:3]
    result[:3,3]=center+rotation@(pose[:3,3]-center)
    return result

def force_reference(phase, elapsed, gap_u, grasp, lift, crush, ramp_fraction):
    """Continuous optional lift effort; zero lift preserves legacy commands."""
    hold = lift if lift > 0 else grasp
    if phase == 4:
        u = float(np.clip(elapsed/ramp_fraction, 0, 1))
        blend = u**3*(10+u*(-15+6*u))
        return grasp+(hold-grasp)*blend
    if phase == 5:
        return hold+(crush-hold)*gap_u
    return crush if phase > 5 else grasp

class PickCrushDrop(Simulation):
    def __init__(self, device='cuda:1', material_overrides=None, scene_path=None, iterations=None, coupled_options=None, rom_options=None):
        register()
        stage=Usd.Stage.Open(str(scene_path or ROOT/'assets/demo_scene.usda'))
        prim=stage.GetPrimAtPath('/World/Physics')
        self.config={k:prim.GetAttribute('cardboard:'+k).Get() for k in
                     ['homeOffset','approachClearance','liftHeight','graspGap','crushGap','graspForce','liftForce','liftForceRampFraction','crushForce','crushRampFraction','releaseRampFraction','phaseEnds']}
        for key in ('graspTiltDegrees','liftTiltDegrees','crushTiltDegrees','releaseTiltDegrees'):
            value=prim.GetAttribute('cardboard:'+key).Get()
            fallback=self.config['crushTiltDegrees'] if key=='releaseTiltDegrees' else self.config['liftTiltDegrees'] if key=='crushTiltDegrees' else (0.,0.,0.)
            self.config[key]=np.asarray(value if value is not None else fallback,float)
            if self.config[key].shape!=(3,) or not np.isfinite(self.config[key]).all():
                raise ValueError(key+' must contain three finite world XYZ angles')
        if not np.isfinite(self.config['liftForce']) or self.config['liftForce'] < 0:
            raise ValueError('liftForce must be finite and nonnegative; zero inherits graspForce')
        if not 0<float(self.config['liftForceRampFraction'])<=1:
            raise ValueError('liftForceRampFraction must be in (0, 1]')
        if not 0<float(self.config['crushRampFraction'])<=1:raise ValueError('crushRampFraction must be in (0, 1]')
        self.ends=np.array(self.config['phaseEnds'],float)
        if len(self.ends)!=8 or np.any(np.diff(np.r_[0,self.ends])<=0):
            raise ValueError('phaseEnds must contain eight increasing positive times')
        super().__init__(device=device,initial_pose_offset=np.array(self.config['homeOffset']),material_overrides=material_overrides,scene_path=scene_path,iterations=iterations,coupled_options=coupled_options,rom_options=rom_options)
        self.duration=float(self.ends[-1]);self.phase=0;self.seed=self.home.copy()
        if self.attr(self.settings,'precomputeCommands'):
            seed=self.seed.copy();cache=[]
            for frame in range(round(self.duration*self.fps)+1):
                gap,twist=self.command(frame/self.fps)
                cache.append((self.seed.copy(),gap,twist,self.command_force,self.phase))
            self.seed=seed;self.command(0.);self.command_cache=cache

    def command(self,t):
        if hasattr(self,'command_cache'):
            q,gap,twist,force,self.phase=self.command_cache[min(round(t*self.fps),len(self.command_cache)-1)]
            self.seed=q.copy();self.command_joint_q=q
            self.target_array.assign(q.astype(np.float32));self.control.joint_target_q.assign(q.astype(np.float32))
            self.motor_force_limit.assign(np.array([force],dtype=np.float32));return gap,twist
        c=self.config;phase=min(int(np.searchsorted(self.ends,t,side='right')),7);self.phase=phase
        start=0 if phase==0 else self.ends[phase-1]
        elapsed=float(np.clip((t-start)/(self.ends[phase]-start),0,1))
        smooth_motion=bool(self.attr(self.settings,'smoothMotion'))
        def ease(value):
            return value**3*(10+value*(-15+6*value)) if smooth_motion else value*value*(3-2*value)
        move_fraction=float(self.attr(self.settings,'liftMoveFraction') or 1.) if phase==4 else 1.
        u=ease(float(np.clip(elapsed/move_fraction,0,1)))
        high=np.array([0.,0.,c['approachClearance']]);lift=np.array([0.,0.,c['liftHeight']]);zero=np.zeros(3)
        offsets=[np.array(c['homeOffset']),high,zero,zero,lift,lift,lift,lift+high]
        previous=offsets[max(0,phase-1)];offset=previous+(offsets[phase]-previous)*u
        open_gap=self.attr(self.settings,'openGap')
        gaps=[open_gap,open_gap,open_gap,c['graspGap'],c['graspGap'],c['crushGap'],open_gap,open_gap]
        gap_u=ease(elapsed)
        if phase==3:
            fraction=float(self.attr(self.settings,'graspRampFraction') or 1.)
            gap_u=ease(float(np.clip(elapsed/fraction,0,1)))
        if phase==5:
            ramp=float(np.clip((t-start)/((self.ends[phase]-start)*c['crushRampFraction']),0,1))
            gap_u=ease(ramp)
        if phase==6:
            ramp=float(np.clip((t-start)/((self.ends[phase]-start)*c['releaseRampFraction']),0,1));gap_u=ease(ramp)
        gap=gaps[max(0,phase-1)]+(gaps[phase]-gaps[max(0,phase-1)])*gap_u
        force=force_reference(phase,elapsed,gap_u,c['graspForce'],c['liftForce'],c['crushForce'],c['liftForceRampFraction'])
        self.command_force=float(force)
        self.motor_force_limit.assign(np.array([force],dtype=np.float32))
        pose=self.target_pose.copy();pose[:3,3]+=offset
        if self.is_robotiq:pose[2,3]+=self.gripper.depth_shift(gap)
        tilts=[np.zeros(3),c['graspTiltDegrees'],c['graspTiltDegrees'],c['graspTiltDegrees'],
               c['liftTiltDegrees'],c['crushTiltDegrees'],c['releaseTiltDegrees'],c['releaseTiltDegrees']]
        previous_tilt=tilts[max(0,phase-1)]
        tilt=previous_tilt+(tilts[phase]-previous_tilt)*u
        pose=tilt_about_grasp_center(pose,self.center+offset,tilt)
        self.seed=self.ik.solve(pose,self.seed)
        if self.is_robotiq:self.gripper.set_gap(self.seed,gap)
        else:self.seed[6:]=[-(gap+.028)/2,(gap+.028)/2]
        self.command_joint_q=self.seed.copy()
        self.target_array.assign(self.seed.astype(np.float32));self.control.joint_target_q.assign(self.seed.astype(np.float32))
        return gap,0.

    def advance(self):
        row=super().advance()
        row.update(phase=self.phase,box_center_z_m=float(self.a.particle_q.numpy()[:,2].mean()),
                   motor_limit_N=float(self.motor_force_limit.numpy()[0]))
        return row
