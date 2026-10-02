# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Newton 1.6 contact law with GPU-only work scheduling and zero-contact exit.
The maximum workers is fixed in each captured graph. Counts select active workers.
Source SHA256: c88eb7b4357e0f36d563ffd0ffe29b780543acfacbb2c17a2dc33f0f5026bdf3
"""
import warp as wp
from newton._src.solvers.vbd.rigid_vbd_kernels import _compute_body_particle_contact_force, _eval_soft_ef_contact
wp.set_module_options({"enable_backward": False})

@wp.kernel
def scheduled_body_particle_contacts(
    dt: float,
    threads_per_body: int,
    color_group: wp.array[wp.int32],
    # Particle state
    particle_q: wp.array[wp.vec3],
    particle_q_prev: wp.array[wp.vec3],
    particle_radius: wp.array[float],
    # Rigid body state
    body_q_prev: wp.array[wp.transform],
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    body_inv_mass: wp.array[float],
    shape_body: wp.array[int],
    # AVBD body-particle soft contact penalties and material properties
    friction_epsilon: float,
    body_particle_contact_penalty_k: wp.array[float],
    body_particle_contact_material_ke: wp.array[float],
    body_particle_contact_material_kd: wp.array[float],
    body_particle_contact_material_mu: wp.array[float],
    # Soft contact data (body-particle)
    body_particle_contact_count: wp.array[int],
    soft_contact_indices: wp.array[wp.vec3i],
    body_particle_contact_shape: wp.array[int],
    body_particle_contact_body_pos: wp.array[wp.vec3],
    body_particle_contact_body_vel: wp.array[wp.vec3],
    body_particle_contact_normal: wp.array[wp.vec3],
    # Barycentric weights on each record's soft particles; (1, 0, 0) for a particle contact.
    soft_contact_barycentric: wp.array[wp.vec3],
    shape_margin: wp.array[float],
    # Per-body soft-contact adjacency (body-particle)
    body_particle_contact_buffer_pre_alloc: int,
    body_particle_contact_counts: wp.array[wp.int32],
    body_particle_contact_indices: wp.array[wp.int32],
    # Outputs
    body_forces: wp.array[wp.vec3],
    body_torques: wp.array[wp.vec3],
    body_hessian_ll: wp.array[wp.mat33],
    body_hessian_al: wp.array[wp.mat33],
    body_hessian_aa: wp.array[wp.mat33],
):
    """
    Per-body accumulation of body-particle soft contact forces and Hessians on rigid bodies.

    Handles both contact kinds from one per-body adjacency list, dispatching on each record's
    -1-padded ``soft_contact_indices``: a particle record ``(p, -1, -1)`` resolves single-particle
    geometry inline; an edge/face record evaluates the barycentric contact point over its 2-3 soft
    particles via ``_eval_soft_ef_contact``. Both apply the shared force law
    ``_compute_body_particle_contact_force`` and the equal-and-opposite body reaction. Body surface
    velocity uses the displacement-based path (body_q_prev).

    Notes:
      - Only dynamic bodies (inv_mass > 0) are updated.
      - Hessian contributions are accumulated into body_hessian_ll/al/aa.
      - Uses per-contact effective penalty/material parameters initialized once per step.
    """
    tid = wp.tid()
    body_idx_in_group = tid // threads_per_body
    thread_id_within_body = tid % threads_per_body

    if body_idx_in_group >= color_group.shape[0]:
        return

    body_id = color_group[body_idx_in_group]
    if body_inv_mass[body_id] <= 0.0:
        return

    num_contacts = body_particle_contact_counts[body_id]
    if num_contacts > body_particle_contact_buffer_pre_alloc:
        num_contacts = body_particle_contact_buffer_pre_alloc

    # No contribution for an empty list; outputs already contain other terms.
    if num_contacts == 0:
        return
    active_workers = int(4)
    if threads_per_body > 4 and num_contacts >= 32:
        active_workers = threads_per_body
    if thread_id_within_body >= active_workers or thread_id_within_body >= num_contacts:
        return

    max_contacts = body_particle_contact_count[0]  # single total soft-contact count

    X_wb = body_q[body_id]
    X_wb_prev = body_q_prev[body_id]
    com_world = wp.transform_point(X_wb, body_com[body_id])

    force_acc = wp.vec3(0.0)
    torque_acc = wp.vec3(0.0)
    h_ll_acc = wp.mat33(0.0)
    h_al_acc = wp.mat33(0.0)
    h_aa_acc = wp.mat33(0.0)

    i = thread_id_within_body
    while i < num_contacts:
        contact_idx = body_particle_contact_indices[body_id * body_particle_contact_buffer_pre_alloc + i]
        i += active_workers
        if contact_idx >= max_contacts:
            continue

        f_soft = wp.vec3(0.0)
        h_soft = wp.mat33(0.0)
        cp_world = wp.vec3(0.0)

        corners = soft_contact_indices[contact_idx]

        if corners[1] < 0:
            # Particle-vs-surface (p, -1, -1): single-particle geometry, resolved inline.
            particle_idx = corners[0]
            if particle_idx < 0:
                continue

            particle_pos = particle_q[particle_idx]
            cp_local = body_particle_contact_body_pos[contact_idx]
            cp_world = wp.transform_point(X_wb, cp_local)
            n = body_particle_contact_normal[contact_idx]
            radius = particle_radius[particle_idx]
            s_idx = body_particle_contact_shape[contact_idx]
            margin = shape_margin[s_idx] if s_idx >= 0 and shape_margin.shape[0] > 0 else 0.0
            penetration_depth = -(wp.dot(n, particle_pos - cp_world) - radius - margin)
            if penetration_depth <= 0.0:
                continue

            bx_prev = wp.transform_point(X_wb_prev, cp_local)
            bv = (cp_world - bx_prev) / dt + wp.transform_vector(X_wb, body_particle_contact_body_vel[contact_idx])
            dx = particle_pos - particle_q_prev[particle_idx]
            relative_translation = dx - bv * dt

            f_soft, h_soft = _compute_body_particle_contact_force(
                penetration_depth,
                n,
                relative_translation,
                body_particle_contact_penalty_k[contact_idx],
                body_particle_contact_material_kd[contact_idx],
                body_particle_contact_material_mu[contact_idx],
                friction_epsilon,
                dt,
            )
        else:
            # Edge/face: barycentric contact point over the record's 2-3 soft particles. Uses the
            # shared force law via _eval_soft_ef_contact -- the same evaluation as the particle side.
            bary = soft_contact_barycentric[contact_idx]
            f_soft, h_soft, cp_world = _eval_soft_ef_contact(
                contact_idx,
                corners,
                bary,
                particle_q,
                particle_q_prev,
                particle_radius,
                body_particle_contact_penalty_k[contact_idx],
                body_particle_contact_material_kd[contact_idx],
                body_particle_contact_material_mu[contact_idx],
                friction_epsilon,
                shape_body,
                body_q,
                body_q_prev,
                body_qd,
                body_com,
                body_particle_contact_shape,
                body_particle_contact_body_pos,
                body_particle_contact_body_vel,
                body_particle_contact_normal,
                shape_margin,
                dt,
            )

        # Equal-and-opposite reaction on the body at the rigid contact point (shared by both kinds).
        f_body = -f_soft
        r = cp_world - com_world
        tau_body = wp.cross(r, f_body)
        r_skew = wp.skew(r)
        r_skew_T_K = wp.transpose(r_skew) * h_soft

        force_acc += f_body
        torque_acc += tau_body
        h_ll_acc += h_soft
        h_al_acc += -r_skew_T_K
        h_aa_acc += r_skew_T_K * r_skew

    wp.atomic_add(body_forces, body_id, force_acc)
    wp.atomic_add(body_torques, body_id, torque_acc)
    wp.atomic_add(body_hessian_ll, body_id, h_ll_acc)
    wp.atomic_add(body_hessian_al, body_id, h_al_acc)
    wp.atomic_add(body_hessian_aa, body_id, h_aa_acc)

