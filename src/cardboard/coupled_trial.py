"""Opt-in Newton 1.6 MuJoCo/VBD experiment; never selected by a scene default."""
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverMuJoCo
from newton.solvers.experimental.coupled import SolverCoupledProxy
from newton._src.solvers.coupled.model_view import ModelView


@wp.kernel
def sync_prescribed_joints(child:wp.array[int],flags:wp.array[int],
                           qstart:wp.array[int],dstart:wp.array[int],
                           mjq:wp.array[int],mjd:wp.array[int],
                           q:wp.array[float],qd:wp.array[float],
                           qpos:wp.array2d[float],qvel:wp.array2d[float]):
    j=wp.tid()
    if flags[child[j]] & int(newton.BodyFlags.KINEMATIC):
        if mjq[j]>=0:
            for k in range(qstart[j+1]-qstart[j]):qpos[0,mjq[j]+k]=q[qstart[j]+k]
            for k in range(dstart[j+1]-dstart[j]):qvel[0,mjd[j]+k]=qd[dstart[j]+k]


@wp.kernel
def move_root(body_q:wp.array[wp.transform],child_frame:wp.array[wp.transform],
              root_frame:wp.array[wp.transform],mocap_pos:wp.array2d[wp.vec3],
              mocap_quat:wp.array2d[wp.quat]):
    pose=body_q[0]
    root_frame[0]=pose*child_frame[0]
    mocap_pos[0,0]=wp.transform_get_translation(pose)
    q=wp.transform_get_rotation(pose)
    # MuJoCo quaternions are stored as w,x,y,z in a Warp vec4/quat.
    mocap_quat[0,0]=wp.quat(q[3],q[0],q[1],q[2])


@wp.kernel
def add_root_velocity(root_q:wp.array[wp.transform],root_qd:wp.array[wp.spatial_vector],
                      com:wp.array[wp.vec3],q:wp.array[wp.transform],qd:wp.array[wp.spatial_vector]):
    i=wp.tid()
    angular=wp.spatial_bottom(root_qd[0])
    offset=wp.transform_point(q[i],com[i])-wp.transform_point(root_q[0],com[0])
    linear=wp.spatial_top(root_qd[0])+wp.cross(angular,offset)
    qd[i]+=wp.spatial_vector(linear,angular)


class CoupledTrial:
    def __init__(self, model, shell_class, shell_options, proxy_iterations=1,
                 mass_scale=1., mode='lagged', relaxation=1., mujoco_iterations=20,
                 partition='full',loop_time_constant=.0025,proxy_joints=False):
        self.model=model
        self.proxy_iterations=proxy_iterations
        owner=self

        class ObservedShell(shell_class):
            def step(self, state_in, state_out, control, contacts, dt):
                owner.last_contacts=contacts
                owner.last_state=state_out
                return super().step(state_in,state_out,control,contacts,dt)

        def make_shell(view):
            # The full single shell and every rigid proxy preserve index order.
            # Plastic updates occur in the parent after each coupled substep.
            for name in ['particle_count','body_count','shape_count','edge_count','tri_count']:
                if getattr(view,name)!=getattr(model,name):
                    raise ValueError('Trial requires identity layout: '+name)
            if not np.array_equal(view.shape_body.numpy(),model.shape_body.numpy()):
                raise ValueError('Coupled shape/body mapping changed')
            if not np.array_equal(view.edge_indices.numpy(),model.edge_indices.numpy()):
                raise ValueError('Coupled hinge mapping changed')
            for name in ['edge_rest_angle','edge_bending_properties']:
                setattr(view,name,getattr(model,name))
            view.gravity=model.gravity
            self.shell=ObservedShell(view,**shell_options)
            return self.shell

        class MovingRootMuJoCo(SolverMuJoCo):
            def coupling_notify_input_state_update(self,state,flags,*,iteration_restart=False,dt=0.):
                if iteration_restart:
                    owner.restore_rigid_start()

            def step(self,state_in,state_out,control,contacts,dt):
                wp.launch(move_root,dim=1,inputs=[state_in.body_q,self.model.joint_X_c,
                    self.model.joint_X_p,self.mjw_data.mocap_pos,self.mjw_data.mocap_quat])
                super().step(state_in,state_out,control,contacts,dt)
                wp.launch(add_root_velocity,dim=self.model.body_count,inputs=[state_in.body_q,
                    state_in.body_qd,self.model.body_com,state_out.body_q,state_out.body_qd])

        class RetainedMuJoCo(SolverMuJoCo):
            def coupling_notify_input_state_update(self,state,flags,*,iteration_restart=False,dt=0.):
                if iteration_restart:owner.restore_rigid_start()

            def step(self,state_in,state_out,control,contacts,dt):
                m=self.model
                wp.launch(sync_prescribed_joints,dim=m.joint_count,inputs=[m.joint_child,
                    m.body_flags,m.joint_q_start,m.joint_qd_start,self.mj_q_start,self.mj_qd_start,
                    state_in.joint_q,state_in.joint_qd,self.mjw_data.qpos,self.mjw_data.qvel])
                return super().step(state_in,state_out,control,contacts,dt)

        def make_rigid(view):
            # Native VBD consumes the authored gains even when an imported
            # joint retains NONE. MuJoCo only creates an actuator for an
            # explicit target mode. Preserve the same four Robotiq PD drives.
            modes=view.joint_target_mode.numpy()
            modes[view.joint_target_ke.numpy()>0]=int(newton.JointTargetMode.POSITION)
            view.joint_target_mode=wp.array(modes,dtype=view.joint_target_mode.dtype,device=model.device)
            view.gravity=model.gravity
            self.rigid=(MovingRootMuJoCo if partition=='gripper' else RetainedMuJoCo)(view,use_mujoco_contacts=False,njmax=512,
                                    iterations=mujoco_iterations,update_data_interval=0)
            # Synthesized closed-loop CONNECT rows inherit MuJoCo's 20 ms
            # default, unlike the original near-rigid linkage. Tighten their
            # numerical compliance, leaving shell/contact material unchanged.
            if loop_time_constant is not None:
                if loop_time_constant<=0:raise ValueError('Loop time constant must be positive')
                solref=self.rigid.mjw_model.eq_solref.numpy()
                solref[...,0]=loop_time_constant;solref[...,1]=1.
                self.rigid.mjw_model.eq_solref.assign(solref)
                self.rigid.mj_model.eq_solref[:]=solref[0]
            return self.rigid

        bodies=list(range(model.body_count));joints=list(range(model.joint_count));other=[]
        coupling_model=model
        if partition=='gripper':
            bodies=[i for i,name in enumerate(model.body_label) if name.startswith('/World/Gripper/')]
            other=[i for i in range(model.body_count) if i not in bodies]
            joints=[i for i,name in enumerate(model.joint_label) if name.startswith('/World/Gripper/')]
            mount=model.joint_label.index('/World/Gripper/Mount')
            coupling_model=ModelView(model,'prescribed_palm_boundary')
            parents=model.joint_parent.numpy();parents[mount]=-1
            coupling_model.joint_parent=wp.array(parents,dtype=int,device=model.device)
            frames=model.joint_X_p.numpy()
            frames[mount]=np.asarray(wp.transform(*model.body_q.numpy()[bodies[0]])*wp.transform(*model.joint_X_c.numpy()[mount]))
            coupling_model.joint_X_p=wp.array(frames,dtype=wp.transform,device=model.device)
        self.coupler=SolverCoupledProxy(model=coupling_model,entries=[
            SolverCoupledProxy.Entry(name='rigid',solver=make_rigid,bodies=bodies,
                                      joints=joints),
            SolverCoupledProxy.Entry(name='shell',solver=make_shell,
                                      bodies=other,
                                      particles=list(range(model.particle_count))),
        ],coupling=SolverCoupledProxy.Config(proxies=[
            SolverCoupledProxy.Proxy(source='rigid',destination='shell',bodies=bodies,
                joints=joints if proxy_joints else [],
                mass_scale=mass_scale,mode=mode,proxy_relaxation=relaxation,
                collision_pipeline=lambda view:newton.CollisionPipeline(view,soft_contact_gap=.006),
                collide_interval=1)
        ],iterations=proxy_iterations))
        self.rigid_start={name:wp.empty_like(getattr(self.rigid.mjw_data,name))
                          for name in ['qpos','qvel','qacc_warmstart']}

    def restore_rigid_start(self):
        for name,array in self.rigid_start.items():wp.copy(getattr(self.rigid.mjw_data,name),array)

    def step(self,state_in,state_out,control,contacts,dt):
        if self.proxy_iterations>1:
            for name,array in self.rigid_start.items():wp.copy(array,getattr(self.rigid.mjw_data,name))
        wp.copy(self.rigid.mjw_model.opt.gravity,self.model.gravity)
        self.coupler._refresh_gravity_accelerations()
        self.coupler.step(state_in,state_out,control,contacts,dt)

    def harvest(self,dt,body_map,wrench):
        # Use the exact contacts and proxy poses consumed by the shell solve.
        from newton._src.solvers.vbd import vbd_coupling_kernels as kernels
        v=self.shell;m=v.model;c=self.last_contacts;s=self.last_state
        wp.launch(kernels._harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel,
            dim=m.body_count*kernels._NUM_CONTACT_THREADS_PER_BODY,
            inputs=[dt,body_map,s.particle_q,v.particle_q_prev,m.particle_radius,
                s.body_q,v._coupling_body_q_prev_snapshot,s.body_qd,m.body_com,float(v.friction_epsilon),
                v.body_particle_contact_penalty_k,v.body_particle_contact_material_kd,
                v.body_particle_contact_material_mu,c.soft_contact_count,c.soft_contact_indices,
                c.soft_contact_barycentric,c.soft_contact_shape,c.soft_contact_body_pos,
                c.soft_contact_body_vel,c.soft_contact_normal,m.shape_margin,m.shape_body,
                v.body_particle_contact_buffer_pre_alloc,v.body_particle_contact_counts,
                v.body_particle_contact_indices,wrench])
