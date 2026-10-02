import argparse,json,time,csv
from importlib.metadata import version
import numpy as np
import warp as wp
import newton
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Vt,Gf,Sdf
from . import ROOT
from .usd_utils import register
from .plasticity import register_state,return_map,return_map_crease
from .kinematics import RobotIK

@wp.kernel
def motor_control(q:wp.array[float],qd:wp.array[float],target:wp.array[float],limits:wp.array[float],max_finger:wp.array[float],finger_kp:float,finger_kd:float,force:wp.array[float]):
    i=wp.tid()
    kp=float(4200.0);kd=float(180.0);limit=limits[i]
    if i>=6:
        kp=finger_kp;kd=finger_kd;limit=max_finger[0]
    error=target[i]-q[i]
    if i<6:error=wp.atan2(wp.sin(error),wp.cos(error))
    force[i]=wp.clamp(kp*error-kd*qd[i],-limit,limit)
    if i<6:force[i]=0.0

@wp.kernel
def set_kinematic_arm(start:wp.array[wp.transform],end:wp.array[wp.transform],velocity:wp.array[wp.spatial_vector],dynamic:wp.array[int],u:float,q:wp.array[wp.transform],qd:wp.array[wp.spatial_vector]):
    i=wp.tid()
    if dynamic[i]==0:
        a=start[i];b=end[i]
        q[i]=wp.transform(wp.lerp(wp.transform_get_translation(a),wp.transform_get_translation(b),u),wp.quat_slerp(wp.transform_get_rotation(a),wp.transform_get_rotation(b),u))
        qd[i]=velocity[i]

class Simulation:
    def __init__(self,device='cuda:1',plastic=True,iterations=None,substeps=None,visual=False,material_overrides=None,force_limit=None,initial_pose_offset=None,scene_path=None,coupled_options=None,rom_options=None):
        register();wp.set_device(device)
        self.stage=Usd.Stage.Open(str(scene_path or ROOT/'assets/demo_scene.usda'));self.mesh=self.stage.GetPrimAtPath('/World/Box/SimMesh');self.settings=self.stage.GetPrimAtPath('/World/Physics')
        self.mat=self.stage.GetPrimAtPath(self.mesh.GetRelationship('cardboard:material').GetTargets()[0])
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        for key,value in (material_overrides or {}).items():self.mat.GetAttribute('cardboard:'+key).Set(value)
        if force_limit is not None:self.settings.GetAttribute('cardboard:maxFingerForce').Set(force_limit)
        def attr(p,n):return p.GetAttribute('cardboard:'+n).Get()
        self.attr=attr;self.fps=attr(self.settings,'fps');self.substeps=substeps or attr(self.settings,'substeps');self.iterations=iterations or attr(self.settings,'iterations');self.dt=1/self.fps/self.substeps
        self.enabled=plastic and attr(self.mat,'plasticityEnabled');self.time=0.
        self.kinematic_arm=bool(attr(self.settings,'kinematicArm'))
        self.is_robotiq=attr(self.settings,'gripperModel')=='robotiq_scaled'
        self.newton_version=version('newton');self.newton16=tuple(map(int,self.newton_version.split('.')[:2]))>=(1,6)
        if attr(self.mesh,'solver')=='newtonVBD16' and not self.newton16:raise RuntimeError('This asset requires Newton 1.6; select the validated toolchain or CARDBOARD_PYTHON.')
        self.bend_relaxation=float(attr(self.mat,'bendingRelaxationTime') or 0.)
        self.crease_damage_length=float(attr(self.mat,'creaseDamageLength') or 0.)
        if not np.isfinite(self.crease_damage_length) or self.crease_damage_length<0:
            raise ValueError('creaseDamageLength must be finite and nonnegative')
        self.center=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(self.mesh).ExtractTranslation(),float)
        self.local=np.asarray(attr(self.mesh,'restPoints'),dtype=np.float32);self.initial=self.local+self.center
        self.faces=np.asarray(UsdGeom.Mesh(self.mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        b=newton.ModelBuilder();register_state(b)
        b.add_usd(self.stage,ignore_paths=['/World/Box'],enable_self_collisions=False,load_visual_shapes=visual,hide_collision_shapes=True)
        self.tip=b.body_label.index('/World/UR10/ee_link');self.palm=b.body_label.index('/World/Gripper/Palm');self.left=b.body_label.index('/World/Gripper/Left');self.right=b.body_label.index('/World/Gripper/Right')
        self.dynamic_bodies=[i for i,label in enumerate(b.body_label) if label.startswith('/World/Gripper/') and i!=self.palm] if self.is_robotiq else [self.left,self.right]
        self.dynamic_mask=wp.array([int(i in self.dynamic_bodies) for i in range(b.body_count)],dtype=int)
        self.dof_count=b.joint_dof_count
        b.joint_target_ke=[5000.]*6+[0.]*(self.dof_count-6);b.joint_target_kd=[100.]*6+[0.]*(self.dof_count-6)
        if self.is_robotiq:
            from .robotiq import RobotiqDrive
            self.gripper=RobotiqDrive(self,b)
        b.add_cloth_mesh(pos=wp.vec3(*self.center),rot=wp.quat_identity(),scale=1.,vel=wp.vec3(0),vertices=self.local,indices=self.faces.ravel(),density=attr(self.mat,'arealDensity'),tri_ke=attr(self.mat,'membraneShear'),tri_ka=attr(self.mat,'membraneArea'),tri_kd=attr(self.mat,'membraneDamping'),edge_ke=1.,edge_kd=attr(self.mat,'bendingDamping'),particle_radius=attr(self.mat,'thickness')*.5)
        expected=np.asarray(attr(self.mesh,'hingeIndices'));assert np.array_equal(expected,b.edge_indices),'Hinge order mismatch; reject state loading'
        self.reference=np.asarray(attr(self.mesh,'referenceAngles'),dtype=np.float32)
        self.ke=np.asarray(attr(self.mesh,'edgeStiffness'),dtype=np.float32)
        # Recompute directional bending from editable material schema values.
        edges=np.asarray(b.edge_indices);edge_vec=self.local[edges[:,3]]-self.local[edges[:,2]]
        direction_weight=(edge_vec[:,1]/np.linalg.norm(edge_vec,axis=1))**2
        self.ke=((attr(self.mat,'bendingCD')+(attr(self.mat,'bendingMD')-attr(self.mat,'bendingCD'))*direction_weight)/np.asarray(attr(self.mesh,'dualWidths'))).astype(np.float32)
        reference_thickness=float(attr(self.mat,'bendingReferenceThickness') or .003)
        self.thickness_scale=(attr(self.mat,'thickness')/reference_thickness)**float(attr(self.mat,'bendingThicknessExponent') or 0.)
        self.ke*=self.thickness_scale
        for i,k in enumerate(self.ke):b.edge_bending_properties[i]=(float(k),self.bend_relaxation*float(k) if self.bend_relaxation>0 else attr(self.mat,'bendingDamping'))
        # Filter robot-table contacts at the bolted base; no interpenetrating source visuals imported as colliders.
        if self.kinematic_arm:
            for i in range(b.body_count):
                if i not in self.dynamic_bodies:b.body_flags[i]=int(newton.BodyFlags.KINEMATIC)
        b.color(include_bending=True)
        if self.kinematic_arm:
            b.body_color_groups=[np.array([i for i in group if i in self.dynamic_bodies],dtype=np.int32) for group in b.body_color_groups]
            b.body_color_groups=[group for group in b.body_color_groups if len(group)]
        self.ik=RobotIK(b,self.tip)
        target=np.eye(4);target[:3,:3]=Rotation.from_euler('x',np.pi).as_matrix();target[:3,3]=self.center+[0,0,float(attr(self.settings,'toolReach') or .315)]
        if self.is_robotiq:target[:3,:3]=self.gripper.downward_ee_rotation
        seed=np.array(b.joint_q);seed[:6]=[0,-1.1,1.7,-2.1,-1.57,0]
        self.home=self.ik.solve(target,seed)
        if not self.is_robotiq:self.home[6:]=[-.174,.174]
        b.joint_q=self.home.tolist()
        if initial_pose_offset is not None:
            start=target.copy();start[:3,3]+=initial_pose_offset
            b.joint_q=self.ik.solve(start,self.home).tolist()
        self.target_pose=target;self.target_array=wp.array(self.home.astype(np.float32),dtype=float)
        self.limits=wp.array([150,150,110,40,40,25,14,14],dtype=float)
        self.model=b.finalize();self.gravity_full=self.model.gravity.numpy().copy();self.gravity_ramp=attr(self.settings,'gravityRampSeconds') or 0.
        self.model.soft_contact_ke=attr(self.settings,'contactStiffness');self.model.soft_contact_kd=attr(self.settings,'contactDamping');self.model.soft_contact_mu=attr(self.mat,'friction')
        newton.eval_fk(self.model,self.model.joint_q,self.model.joint_qd,self.model)
        self.pipeline=newton.CollisionPipeline(self.model,**({'soft_contact_gap':.006} if self.newton16 else {'soft_contact_margin':.006}))
        self.contacts=self.pipeline.contacts()
        self_contact=attr(self.mat,'thickness')*(attr(self.mat,'selfContactThicknessScale') or .5)
        contact_options=dict(particle_self_contact_radius=self_contact,particle_self_contact_margin=self_contact+.0025,particle_collision_detection_interval=0)
        if self.newton16:
            base=newton.solvers.SolverBase
            contact_options=dict(particle_self_contact_margin=self_contact,particle_self_contact_gap=.0025,rigid_compliant_alm=bool(attr(self.settings,'rigidCompliantALM')),collision_frequency_type={base.CollisionSlot.SOFT_SELF_CONTACT:base.CollisionFrequencyType.PRE_POST_INIT})
            # Fine shell elements can be closer than the board's contact
            # thickness already at rest. Exclude only those rest-neighbor pairs;
            # nonlocal folds and all gripper/table contacts remain active.
            rest_exclusion=float(attr(self.settings,'selfContactRestExclusion') or 0.)
            if rest_exclusion < 0:raise ValueError('selfContactRestExclusion must be nonnegative')
            contact_options['particle_rest_shape_contact_exclusion_radius']=rest_exclusion
        solver_class=newton.solvers.SolverVBD
        if attr(self.settings,'translationBlockSolve'):
            if not self.newton16:raise RuntimeError('Translation block requires Newton 1.6')
            if self.crease_damage_length:
                from .block_solver import TranslationBlockVBD
            else:
                from .legacy_block_solver import TranslationBlockVBD
            solver_class=TranslationBlockVBD
        self.coupled_stepper=None
        solver_options=dict(iterations=self.iterations,particle_enable_self_contact=True,friction_epsilon=.0005,**contact_options)
        contact_capacity=int(attr(self.settings,'rigidParticleContactCapacity'))
        if contact_capacity<1:raise ValueError('rigidParticleContactCapacity must be positive')
        if self.newton16:
            solver_options['rigid_body_particle_contact_buffer_size']=contact_capacity
        elif contact_capacity!=256:
            raise ValueError('Configurable rigid-particle contact capacity requires Newton 1.6')
        joint_stiffness=float(attr(self.settings,'rigidJointLinearStiffness'))
        if not np.isfinite(joint_stiffness) or joint_stiffness<=0:
            raise ValueError('rigidJointLinearStiffness must be finite and positive')
        if self.newton16:solver_options['rigid_joint_linear_ke']=joint_stiffness
        elif joint_stiffness!=100000.:
            raise ValueError('Configurable joint structural stiffness requires Newton 1.6')
        if rom_options is not None:
            if coupled_options is not None or not attr(self.settings,'translationBlockSolve') or not self.crease_damage_length:
                raise ValueError('ROM trial requires the original creased-shell VBD path')
            from .rom_basis import mesh_digest
            from .rom_solver import AdaptiveROMVBD
            with np.load(rom_options['basis']) as data:
                if str(data['mesh_digest'])!=mesh_digest(self.local,self.faces):raise ValueError('ROM basis topology/rest geometry mismatch')
            self.solver=AdaptiveROMVBD(self.model,rom_options=rom_options,**solver_options)
        elif coupled_options is not None:
            from .coupled_trial import CoupledTrial
            self.coupled_stepper=CoupledTrial(self.model,solver_class,solver_options,**coupled_options)
            self.solver=self.coupled_stepper.shell
        else:
            self.solver=solver_class(self.model,**solver_options)
        interval=int(attr(self.settings,'particleSolveInterval') or 1)
        if interval<1:raise ValueError('particleSolveInterval must be positive')
        if interval!=1 and not hasattr(self.solver,'particle_solve_interval'):
            raise ValueError('Separate particle budget requires the optimized block solver')
        self.solver.particle_solve_interval=interval
        if self.coupled_stepper:
            self.solver.iterations=len([i for i in range(self.iterations) if i%interval==0 or i==self.iterations-1])
            self.solver.particle_solve_interval=1
        if attr(self.settings,'rotationBlockSolve'):
            if not hasattr(self.solver,'rotation_block'):raise ValueError('Rotation block requires optimized shell solver')
            self.solver.rotation_block=True
        self.coupled_supported=bool(attr(self.settings,'coupledSupportedSolve'))
        self.coupled_always=bool(attr(self.settings,'rigidBlockSolve'))
        if self.coupled_always or self.coupled_supported:
            if not hasattr(self.solver,'rigid_subspace'):raise ValueError('Rigid block requires optimized shell solver')
            from .rigid_subspace import RigidSubspace
            self.solver.rigid_subspace=RigidSubspace(self.model,self.solver.rotation_origin)
            self.solver.rigid_subspace_enabled=self.coupled_always
        self.internal_damping=None
        damping_rate=float(attr(self.mat,'internalVelocityDamping') or 0.)
        if damping_rate:
            from .internal_damping import InternalDamping
            self.internal_damping=InternalDamping(self.model,self.center,damping_rate)
        self.a=self.model.state();self.b=self.model.state();self.control=self.model.control()
        self.rest=None
        if attr(self.settings,'supportedSleep'):
            if not hasattr(self.solver,'box_sleeping'):
                raise ValueError('Supported sleep requires optimized shell solver')
            from .resting import SupportedRest
            self.particle_mass_cpu=self.model.particle_mass.numpy()
            self.rest=SupportedRest(self)
        self.graph_cache={};self.graph_mode='active'
        self.active_substeps=self.substeps;self.active_particle_interval=interval
        self.rest_refinement=bool(attr(self.settings,'restRefinement'))
        self.rest_damping_rate=float(attr(self.mat,'supportedInternalDamping') or damping_rate)
        self.support_rigid_damping=float(attr(self.mat,'supportedRigidDamping') or 0.)
        self.sleep_iterations=int(attr(self.settings,'sleepIterations') or self.iterations)
        if self.sleep_iterations<1:raise ValueError('sleepIterations must be positive')
        newton.eval_fk(self.model,self.model.joint_q,self.model.joint_qd,self.a);newton.eval_fk(self.model,self.model.joint_q,self.model.joint_qd,self.b)
        self.ref_gpu=wp.array(self.reference,dtype=float);self.dual=wp.array(np.asarray(attr(self.mesh,'dualWidths')),dtype=float);self.ke_gpu=wp.array(self.ke,dtype=float)
        self.body_map=wp.array(np.arange(self.model.body_count,dtype=np.int32),dtype=int);self.wrench=wp.zeros(self.model.body_count,dtype=wp.spatial_vector)
        self.body_prev=wp.empty_like(self.a.body_q)
        if self.kinematic_arm:
            self.kin_previous=self.a.body_q.numpy()
            self.kin_start=wp.clone(self.a.body_q);self.kin_end=wp.clone(self.a.body_q);self.kin_velocity=wp.zeros(self.model.body_count,dtype=wp.spatial_vector)
        self.motor_force_limit=wp.array([float(attr(self.settings,'maxFingerForce'))],dtype=float)
        self.rows=[];self.max_tau=attr(self.settings,'maxWristTorque');self.twist_allowed=True
        print('MODEL',self.model.particle_count,'vertices',self.model.body_count,'bodies; IK',self.home[:6],flush=True)
    def command(self,t):
        # 0-1 settle; 1-4 close/crush; 4-5.5 twist; 5.5-6 hold; 6-7 release; 7-9 recovery.
        smooth=lambda x:np.clip(x,0,1)**2*(3-2*np.clip(x,0,1))
        a=self.attr;open_gap=a(self.settings,'openGap');min_gap=a(self.settings,'minimumGap')
        close=smooth((t-1)/3);release=smooth((t-6)/1)
        gap=open_gap-(open_gap-min_gap)*close*(1-release)
        twist=a(self.settings,'twistRadians')*smooth((t-4)/1.5)*(1-release)
        requested_twist=twist
        previous=getattr(self,'twist_command',0.)
        if self.rows and abs(self.rows[-1]['wrist_contact_z_Nm'])>self.max_tau:
            twist=max(0.,previous-.001)
        else:
            twist=previous+float(np.clip(requested_twist-previous,-.002,.002))
        self.twist_command=twist
        offset=getattr(self,'centering_offset',0.)
        if self.rows and 1.<t<6.:
            offset=float(np.clip(offset+.002*self.rows[-1]['wrist_force_x_N']/self.fps,-.025,.025))
        elif t>=6.:offset*=.97
        self.centering_offset=offset
        pose=self.target_pose.copy();pose[0,3]+=offset;pose[:3,:3]=Rotation.from_euler('z',twist).as_matrix()@self.target_pose[:3,:3]
        if self.is_robotiq:pose[2,3]+=self.gripper.depth_shift(gap)
        q=self.ik.solve(pose,self.home)
        if self.is_robotiq:self.gripper.set_gap(q,gap)
        else:q[6:]=[-(gap+.028)/2,(gap+.028)/2]
        self.target_array.assign(q.astype(np.float32));self.control.joint_target_q.assign(q.astype(np.float32));return gap,twist
    def _integrate(self):
        for substep in range(self.substeps):
            if self.kinematic_arm:
                wp.launch(set_kinematic_arm,dim=self.model.body_count,inputs=[self.kin_start,self.kin_end,self.kin_velocity,self.dynamic_mask,float(substep+1)/self.substeps,self.a.body_q,self.a.body_qd])
            self.a.clear_forces();newton.eval_ik(self.model,self.a,self.a.joint_q,self.a.joint_qd)
            if self.is_robotiq:self.gripper.apply(self)
            else:wp.launch(motor_control,dim=8,inputs=[self.a.joint_q,self.a.joint_qd,self.target_array,self.limits,self.motor_force_limit,float(self.attr(self.settings,'fingerPositionStiffness')),float(self.attr(self.settings,'fingerVelocityDamping')),self.control.joint_f])
            wp.copy(self.body_prev,self.a.body_q)
            self.pipeline.collide(self.a,self.contacts)
            (self.coupled_stepper or self.solver).step(self.a,self.b,self.control,self.contacts,self.dt)
            if not (self.rest and self.solver.box_sleeping):
                if hasattr(self,'record_crease_dissipation'):self.record_crease_dissipation()
                if self.internal_damping:
                    self.internal_damping.apply(self.b,self.dt,self.rest_damping_rate if self.graph_mode=='supported' else None,
                                                self.support_rigid_damping if self.graph_mode=='supported' else 0.)
                wp.launch(return_map_crease if self.crease_damage_length else return_map,dim=self.model.edge_count,inputs=[self.b.particle_q,self.model.edge_indices,self.model.edge_rest_length,self.ref_gpu,self.dual,self.ke_gpu,self.attr(self.mat,'yieldCurvature'),self.attr(self.mat,'hardeningRatio'),self.attr(self.mat,'damageRate'),self.attr(self.mat,'residualStiffness'),int(self.enabled),self.bend_relaxation,self.a.cardboard.plastic_angle,self.a.cardboard.accumulated_angle,self.a.cardboard.plastic_work,self.b.cardboard.plastic_angle,self.b.cardboard.accumulated_angle,self.b.cardboard.plastic_work,self.b.cardboard.damage,self.model.edge_rest_angle,self.model.edge_bending_properties]+([self.crease_damage_length] if self.crease_damage_length else []))
            self.a,self.b=self.b,self.a
    def advance(self):
        if self.gravity_ramp>0:
            ramp=float(np.clip(self.time/self.gravity_ramp,0,1));self.model.gravity.assign(self.gravity_full*(ramp*ramp*(3-2*ramp)))
        gap,twist=self.command(self.time)
        if self.rest:
            # The prescribed grasp/lift/crush controller anticipates external
            # loading, so request wake even before the first contact force.
            loading=3<=getattr(self,'phase',-1)<=5
            sleeping=self.rest.update(self.a.particle_q.numpy(),self.a.particle_qd.numpy(),external_change=loading)
            yielded=bool(self.rows and self.rows[-1]['plastic_hinges']>0)
            mode='sleep' if sleeping else ('supported' if self.rest_refinement and self.rest.refine and yielded else 'active')
            if mode!=self.graph_mode:
                if hasattr(self,'graph'):
                    self.graph_cache[self.graph_mode]=self.graph
                    del self.graph
                self.graph_mode=mode;self.solver.box_sleeping=sleeping
                self.substeps=32 if mode=='supported' else self.active_substeps
                self.dt=1/self.fps/self.substeps
                self.solver.iterations=self.sleep_iterations if sleeping else self.iterations
                self.solver.rigid_subspace_enabled=self.coupled_always or (self.coupled_supported and mode=='supported')
                self.solver.particle_solve_interval=1 if mode=='supported' else self.active_particle_interval
                if self.coupled_stepper:
                    self.solver.iterations=self.iterations if mode=='supported' else (self.sleep_iterations if sleeping else len([i for i in range(self.iterations) if i%self.active_particle_interval==0 or i==self.iterations-1]))
                    self.solver.particle_solve_interval=1
                if sleeping:
                    self.a.particle_qd.zero_();self.b.particle_qd.zero_()
                    wp.copy(self.b.particle_q,self.a.particle_q)
                    for key in ['plastic_angle','accumulated_angle','plastic_work','damage']:
                        wp.copy(getattr(self.b.cardboard,key),getattr(self.a.cardboard,key))
                # All kernels/buffers were warmed by the initial active frame.
                # A mode only changes counts or omits particle work; capture it
                # directly instead of executing another expensive eager frame.
                self.graph=self.graph_cache.get(mode)
        if self.kinematic_arm:
            matrices=np.asarray(self.ik.fk(self.command_joint_q if hasattr(self,'command_joint_q') else self.target_array.numpy()))
            target=np.concatenate([matrices[:,:3,3],Rotation.from_matrix(matrices[:,:3,:3]).as_quat()],axis=1).astype(np.float32)
            velocity=np.zeros((self.model.body_count,6),dtype=np.float32)
            velocity[:,:3]=(target[:,:3]-self.kin_previous[:,:3])*self.fps
            velocity[:,3:]=(Rotation.from_quat(target[:,3:])*Rotation.from_quat(self.kin_previous[:,3:]).inv()).as_rotvec()*self.fps
            self.kin_start.assign(self.kin_previous);self.kin_end.assign(target);self.kin_velocity.assign(velocity);self.kin_previous=target
        max_force=0.;max_torque=0.;motors=np.zeros(8)
        if not hasattr(self, 'graph'):
            self._integrate()
            self.graph = None
        else:
            if self.graph is None and self.substeps % 2 == 0:
                with wp.ScopedCapture() as capture:
                    self._integrate()
                self.graph = capture.graph
            if self.graph is not None:
                wp.capture_launch(self.graph)
            else:
                self._integrate()
        self.time+=1/self.fps
        # Version-pinned 1.5/1.6 private diagnostic uses the exact solver contact energy, not an actuator-force proxy.
        from newton._src.solvers.vbd.vbd_coupling_kernels import _harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel as harvest
        c=self.contacts;v=self.solver
        self.wrench.zero_()
        from newton._src.solvers.vbd import vbd_coupling_kernels
        harvest_dim=self.model.body_count*vbd_coupling_kernels._NUM_CONTACT_THREADS_PER_BODY if self.newton16 else c.soft_contact_max
        if self.coupled_stepper:
            self.coupled_stepper.harvest(self.dt,self.body_map,self.wrench)
        else:
            wp.launch(harvest,dim=harvest_dim,inputs=[self.dt,self.body_map,self.a.particle_q,v.particle_q_prev,self.model.particle_radius,self.a.body_q,self.body_prev,self.a.body_qd,self.model.body_com,float(v.friction_epsilon),v.body_particle_contact_penalty_k,v.body_particle_contact_material_kd,v.body_particle_contact_material_mu,c.soft_contact_count,c.soft_contact_indices,c.soft_contact_barycentric,c.soft_contact_shape,c.soft_contact_body_pos,c.soft_contact_body_vel,c.soft_contact_normal,self.model.shape_margin,self.model.shape_body]+([v.body_particle_contact_buffer_pre_alloc,v.body_particle_contact_counts,v.body_particle_contact_indices] if self.newton16 else [])+[self.wrench])
        q=self.a.particle_q.numpy();body=self.a.body_q.numpy();w=self.wrench.numpy();alpha=self.a.cardboard.accumulated_angle.numpy();newton.eval_ik(self.model,self.a,self.a.joint_q,self.a.joint_qd)
        ee=body[self.tip,:3];force=w[[self.left,self.right],:3];grip=[self.palm]+self.dynamic_bodies;torque=w[grip,3:]+np.cross(body[grip,:3]-ee,w[grip,:3])
        row={'t':self.time,'command_gap_m':gap,'twist_command_rad':twist,'actual_gap_m':self.gripper.gap(body) if self.is_robotiq else float(np.linalg.norm(body[self.left,:3]-body[self.right,:3])-.028),'left_contact_N':float(np.linalg.norm(force[0])),'right_contact_N':float(np.linalg.norm(force[1])),'wrist_contact_Nm':float(np.linalg.norm(torque.sum(0))),'wrist_contact_z_Nm':float(torque.sum(0)[2]),'wrist_force_x_N':float(w[grip,0].sum()),'centering_offset_m':getattr(self,'centering_offset',0.),'plastic_hinges':int((alpha>1e-4).sum()),'plastic_work_J':float(self.a.cardboard.plastic_work.numpy().sum()),'max_plastic_angle_rad':float(np.abs(self.a.cardboard.plastic_angle.numpy()).max()),'width_m':float(np.ptp(q[:,0])),'min_z_m':float(q[:,2].min()),'max_speed_m_s':float(np.linalg.norm(self.a.particle_qd.numpy(),axis=1).max()),'ee_error_m':float(np.linalg.norm(ee-self.target_pose[:3,3]))}
        if not np.isfinite(q).all() or not np.isfinite(body).all():raise RuntimeError('Nonfinite state')
        if self.is_robotiq:
            jq=self.a.joint_q.numpy();target=self.target_array.numpy()
            for name,index in zip(['left_input','right_input'],self.gripper.indices[:2]):
                row[name+'_rad']=float(jq[index]);row[name+'_target_rad']=float(target[index])
        if self.rest:
            row['box_sleeping']=int(self.solver.box_sleeping)
            row['solver_mode']={'active':0,'supported':1,'sleep':2}[self.graph_mode]
        if getattr(self.solver,'crease_friction_curvature',0.)>0:
            row['crease_friction_work_J']=float(self.crease_friction_work.numpy().sum())
        self.rows.append(row);return row

    def load_box_checkpoint(self,path):
        """Restore a free deformable's material/geometry state before first stepping.

        Robot state and VBD multipliers are intentionally not part of this portable
        asset checkpoint; this is not an exact whole-scene solver restart.
        """
        if hasattr(self,'graph'):raise RuntimeError('Load checkpoint before stepping')
        stage=Usd.Stage.Open(str(path));prim=stage.GetPrimAtPath('/Box/SimMesh')
        if not prim:raise ValueError('Missing /Box/SimMesh checkpoint')
        edges=np.asarray(prim.GetAttribute('cardboard:hingeIndices').Get())
        if not np.array_equal(edges,self.model.edge_indices.numpy()):raise ValueError('Checkpoint topology differs')
        points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),np.float32)
        velocity=np.asarray(prim.GetAttribute('cardboard:velocities').Get(),np.float32)
        if points.shape!=self.initial.shape or velocity.shape!=points.shape:raise ValueError('Checkpoint particle count mismatch')
        arrays={}
        for usd,key in [('plasticAngles','plastic_angle'),('accumulatedAngles','accumulated_angle'),('plasticDissipation','plastic_work'),('damage','damage')]:
            value=np.asarray(prim.GetAttribute('cardboard:'+usd).Get(),np.float32)
            if value.shape!=(self.model.edge_count,) or not np.isfinite(value).all():raise ValueError('Invalid state '+usd)
            arrays[key]=value
        if not np.isfinite(points).all() or not np.isfinite(velocity).all():raise ValueError('Nonfinite checkpoint')
        if np.any(arrays['accumulated_angle']<np.abs(arrays['plastic_angle'])-1e-5):raise ValueError('Invalid accumulated plastic angle')
        if np.any(arrays['plastic_work']<0) or np.any(arrays['damage']<0) or np.any(arrays['damage']>=1):raise ValueError('Invalid energy/damage')
        for state in [self.a,self.b]:
            state.particle_q.assign((points+self.center).astype(np.float32));state.particle_qd.assign(velocity)
            for key,value in arrays.items():getattr(state.cardboard,key).assign(value)
        self.model.edge_rest_angle.assign(self.reference+arrays['plastic_angle'])
        props=self.model.edge_bending_properties.numpy();props[:,0]=self.ke*(1-arrays['damage'])
        if self.bend_relaxation>0:props[:,1]=self.bend_relaxation*props[:,0]
        self.model.edge_bending_properties.assign(props)
        return arrays
