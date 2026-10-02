# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Project-local Newton 1.6 rigid scheduling overlay.

Upstream iteration ordering and laws retained. Only accumulator clearing and
body-particle launch scheduling differ. No global monkey patch or host count read.
Upstream solver SHA256: 2eaf20bde0cfa436bdaad375b689f5e562130667801ceb3e6098343f4418bc13
"""
from __future__ import annotations
import os
import numpy as np
import warp as wp
from newton import State,Control,Contacts
from newton._src.solvers.vbd.rigid_vbd_kernels import (
    _NUM_CONTACT_THREADS_PER_BODY, accumulate_body_body_contacts_per_body,
    solve_rigid_body, update_duals_body_body_contacts,
    update_duals_body_particle_contacts, update_duals_joint,
)
from .scheduled_contact import scheduled_body_particle_contacts

@wp.kernel
def clear_body_accumulators(force:wp.array[wp.vec3],torque:wp.array[wp.vec3],
                            ll:wp.array[wp.mat33],al:wp.array[wp.mat33],aa:wp.array[wp.mat33]):
    i=wp.tid()
    force[i]=wp.vec3(0.0)
    torque[i]=wp.vec3(0.0)
    ll[i]=wp.mat33(0.0)
    al[i]=wp.mat33(0.0)
    aa[i]=wp.mat33(0.0)

class RigidScheduleMixin:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.contact_schedule=os.environ.get('CARDBOARD_VBD_SCHEDULE','baseline')
        if self.contact_schedule not in ('baseline','guarded','adaptive'):
            raise ValueError('CARDBOARD_VBD_SCHEDULE must be baseline, guarded or adaptive')
        self.contact_workers=32 if self.contact_schedule=='adaptive' else 4
        # Each dynamic body occurs in exactly one color. Particle positions and
        # that body's pose do not change before its color's rigid solve. Gather
        # body-particle terms once for all colors before any rigid solve; body-
        # body/joint terms remain in their original Gauss-Seidel color order.
        groups=[group.numpy() for group in self.model.body_color_groups]
        indices=np.concatenate(groups) if groups else np.empty(0,dtype=np.int32)
        if len(np.unique(indices))!=len(indices):
            raise ValueError('Rigid contact scheduling requires disjoint body colors')
        self.contact_body_group=wp.array(indices,dtype=int,device=self.device)

    def _solve_rigid_body_iteration(
        self,
        state_in: State,
        state_out: State,
        control: Control,
        contacts: Contacts | None,
        dt: float,
    ):
        """Solve one rigid-body VBD iteration (per-iteration phase).

        Accumulates contact and joint forces/hessians, solves 6x6 rigid body systems per color,
        and updates AVBD penalty parameters (dual update).
        """
        if self.contact_schedule == "baseline":
            return super()._solve_rigid_body_iteration(state_in,state_out,control,contacts,dt)
        model = self.model
        # Body-particle soft contacts still need penalty updates when VBD skips rigid solves:
        # external rigid mode uses state_out.body_q, while static-shape contacts use _empty_body_q.
        skip_rigid_solve = not self._integrates_rigid_bodies
        if skip_rigid_solve:
            if model.particle_count > 0 and contacts is not None:
                body_q = state_out.body_q if self.integrate_with_external_rigid_solver else state_in.body_q
                if body_q is None:
                    body_q = self._empty_body_q

                wp.launch(
                    kernel=update_duals_body_particle_contacts,
                    dim=contacts.soft_contact_max,
                    inputs=[
                        contacts.soft_contact_count,
                        contacts.soft_contact_indices,
                        contacts.soft_contact_shape,
                        contacts.soft_contact_body_pos,
                        contacts.soft_contact_normal,
                        contacts.soft_contact_barycentric,
                        state_in.particle_q,
                        model.particle_radius,
                        model.shape_body,
                        model.shape_margin,
                        body_q,
                        self.body_particle_contact_material_ke,
                        self.rigid_linear_beta,
                        self.body_particle_contact_penalty_k,  # input/output
                    ],
                    device=self.device,
                )
            return

        # Zero out forces and hessians
        wp.launch(clear_body_accumulators,dim=model.body_count,inputs=[self.body_forces,self.body_torques,self.body_hessian_ll,self.body_hessian_al,self.body_hessian_aa],device=self.device)

        # Independent body-particle terms for all dynamic bodies before color solves.
        if model.particle_count > 0 and contacts is not None:
            wp.launch(
                kernel=scheduled_body_particle_contacts,
                dim=self.contact_body_group.size * self.contact_workers,
                inputs=[
                    dt,
                    self.contact_workers,
                    self.contact_body_group,
                    state_in.particle_q,
                    self.particle_q_prev,
                    model.particle_radius,
                    self.body_q_prev,
                    state_in.body_q,
                    state_in.body_qd,
                    model.body_com,
                    self.body_inv_mass_effective,
                    model.shape_body,
                    self.friction_epsilon,
                    self.body_particle_contact_penalty_k,
                    self.body_particle_contact_material_ke,
                    self.body_particle_contact_material_kd,
                    self.body_particle_contact_material_mu,
                    contacts.soft_contact_count,
                    contacts.soft_contact_indices,
                    contacts.soft_contact_shape,
                    contacts.soft_contact_body_pos,
                    contacts.soft_contact_body_vel,
                    contacts.soft_contact_normal,
                    contacts.soft_contact_barycentric,
                    model.shape_margin,
                    self.body_particle_contact_buffer_pre_alloc,
                    self.body_particle_contact_counts,
                    self.body_particle_contact_indices,
                ],
                outputs=[
                    self.body_forces,
                    self.body_torques,
                    self.body_hessian_ll,
                    self.body_hessian_al,
                    self.body_hessian_aa,
                ],
                device=self.device,
            )


        body_color_groups = model.body_color_groups

        # Gauss-Seidel-style per-color updates
        for color in range(len(body_color_groups)):
            color_group = body_color_groups[color]

            # Accumulate body-body (rigid-rigid) contact forces and Hessians on bodies (per-body, per-color)
            if contacts is not None:
                wp.launch(
                    kernel=accumulate_body_body_contacts_per_body,
                    dim=color_group.size * _NUM_CONTACT_THREADS_PER_BODY,
                    inputs=[
                        dt,
                        color_group,
                        self.body_q_prev,
                        state_in.body_q,
                        model.body_com,
                        self.body_inv_mass_effective,
                        self.friction_epsilon,
                        self.body_body_contact_penalty_k,
                        self.body_body_contact_normal_rho,
                        self.body_body_contact_material_ke,
                        self.body_body_contact_material_kd,
                        self.body_body_contact_material_mu,
                        self.body_body_contact_tangent_rho,
                        self.body_body_contact_lambda,
                        self.body_body_contact_C0,
                        self.rigid_contact_alpha,
                        self.rigid_contact_hard,
                        self.rigid_compliant_alm,
                        contacts.rigid_contact_count,
                        contacts.rigid_contact_shape0,
                        contacts.rigid_contact_shape1,
                        contacts.rigid_contact_point0,
                        contacts.rigid_contact_point1,
                        contacts.rigid_contact_offset0,
                        contacts.rigid_contact_offset1,
                        contacts.rigid_contact_normal,
                        contacts.rigid_contact_margin0,
                        contacts.rigid_contact_margin1,
                        model.shape_body,
                        self.body_body_contact_buffer_pre_alloc,
                        self.body_body_contact_counts,
                        self.body_body_contact_indices,
                    ],
                    outputs=[
                        self.body_forces,
                        self.body_torques,
                        self.body_hessian_ll,
                        self.body_hessian_al,
                        self.body_hessian_aa,
                    ],
                    device=self.device,
                )

            wp.launch(
                kernel=solve_rigid_body,
                inputs=[
                    dt,
                    color_group,
                    state_in.body_q,
                    self.body_q_prev,
                    model.body_q,
                    model.body_mass,
                    self.body_inv_mass_effective,
                    model.body_inertia,
                    self.body_inertia_q,
                    model.body_com,
                    self.rigid_adjacency,
                    model.joint_type,
                    model.joint_enabled,
                    model.joint_parent,
                    model.joint_child,
                    model.joint_X_p,
                    model.joint_X_c,
                    model.joint_axis,
                    self.joint_rod_rest_kb_local,
                    self.joint_rod_rest_twist,
                    model.joint_qd_start,
                    model.joint_target_q_start,
                    self.joint_constraint_start,
                    self.joint_penalty_k,
                    self.joint_rho,
                    self.joint_material_k,
                    self.joint_penalty_kd,
                    self.joint_sigma_start,
                    self.joint_C_fric,
                    model.joint_target_ke,
                    model.joint_target_kd,
                    control.joint_target_q,
                    control.joint_target_qd,
                    model.joint_limit_lower,
                    model.joint_limit_upper,
                    model.joint_limit_ke,
                    model.joint_limit_kd,
                    self.joint_drive_limit_support,
                    self.joint_drive_lambda,
                    self.joint_limit_lambda,
                    self.joint_lambda_lin,
                    self.joint_lambda_ang,
                    self.joint_C0_lin,
                    self.joint_C0_ang,
                    self.joint_is_hard,
                    self.rigid_joint_alpha,
                    self.rigid_compliant_alm,
                    model.joint_dof_dim,
                    self.joint_rest_angle,
                    self.body_forces,
                    self.body_torques,
                    self.body_hessian_ll,
                    self.body_hessian_al,
                    self.body_hessian_aa,
                ],
                outputs=[
                    state_in.body_q,
                ],
                dim=color_group.size,
                device=self.device,
            )

        if contacts is not None and contacts.rigid_contact_max > 0:
            wp.launch(
                kernel=update_duals_body_body_contacts,
                dim=contacts.rigid_contact_max,
                inputs=[
                    contacts.rigid_contact_count,
                    contacts.rigid_contact_shape0,
                    contacts.rigid_contact_shape1,
                    contacts.rigid_contact_point0,
                    contacts.rigid_contact_point1,
                    contacts.rigid_contact_offset0,
                    contacts.rigid_contact_offset1,
                    contacts.rigid_contact_normal,
                    contacts.rigid_contact_margin0,
                    contacts.rigid_contact_margin1,
                    model.shape_body,
                    state_in.body_q,
                    self.body_q_prev,
                    self.body_body_contact_material_mu,
                    self.body_body_contact_C0,
                    self.rigid_contact_alpha,
                    self.rigid_contact_hard,
                    self.rigid_compliant_alm,
                    self.body_body_contact_material_ke,
                    self.body_body_contact_tangent_rho,
                    self.body_body_contact_normal_rho,
                    self.rigid_linear_beta,
                    self.body_body_contact_penalty_k,  # input/output
                    self.body_body_contact_lambda,  # input/output
                ],
                device=self.device,
            )
        if contacts is not None and model.particle_count > 0:
            wp.launch(
                kernel=update_duals_body_particle_contacts,
                dim=contacts.soft_contact_max,
                inputs=[
                    contacts.soft_contact_count,
                    contacts.soft_contact_indices,
                    contacts.soft_contact_shape,
                    contacts.soft_contact_body_pos,
                    contacts.soft_contact_normal,
                    contacts.soft_contact_barycentric,
                    state_in.particle_q,
                    model.particle_radius,
                    model.shape_body,
                    model.shape_margin,
                    state_in.body_q,
                    self.body_particle_contact_material_ke,
                    self.rigid_linear_beta,
                    self.body_particle_contact_penalty_k,  # input/output
                ],
                device=self.device,
            )

        if model.joint_count > 0:
            wp.launch(
                kernel=update_duals_joint,
                dim=model.joint_count,
                inputs=[
                    model.joint_type,
                    model.joint_enabled,
                    model.joint_parent,
                    model.joint_child,
                    model.joint_X_p,
                    model.joint_X_c,
                    model.joint_axis,
                    self.joint_rod_rest_kb_local,
                    self.joint_rod_rest_twist,
                    model.joint_qd_start,
                    model.joint_target_q_start,
                    self.joint_constraint_start,
                    state_in.body_q,
                    self.body_q_prev,
                    model.body_q,
                    model.joint_dof_dim,
                    self.joint_C0_lin,
                    self.joint_C0_ang,
                    self.joint_is_hard,
                    self.rigid_joint_alpha,
                    self.joint_material_k,
                    self.joint_rho,
                    self.rigid_compliant_alm,
                    self.rigid_linear_beta,
                    self.rigid_angular_beta,
                    model.joint_target_ke,
                    model.joint_target_kd,
                    control.joint_target_q,
                    control.joint_target_qd,
                    model.joint_limit_lower,
                    model.joint_limit_upper,
                    model.joint_limit_ke,
                    model.joint_limit_kd,
                    self.joint_rest_angle,
                    self.joint_drive_limit_support,
                    dt,
                    self.joint_penalty_k,  # input/output
                    self.joint_lambda_lin,  # input/output
                    self.joint_lambda_ang,  # input/output
                    self.joint_drive_lambda,  # input/output
                    self.joint_limit_lambda,  # input/output
                ],
                device=self.device,
            )
