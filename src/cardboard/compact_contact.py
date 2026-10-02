# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Newton 1.6 particle solve with compact self-contact scheduling.

Adapted from Newton 1.6 particle_vbd_kernels.py / solver_vbd.py. Contact
energy, planar truncation, coloring and iteration order are unchanged. Only
contact-list traversal is changed: pack every detected pair once after BVH
queries, then schedule one thread per pair with a bounded grid-stride loop.
No contact is discarded and the original per-primitive capacities are kept.
Opt-in local-ROM experiments additionally gate elastic/contact coordinate
updates with a GPU mask. The optional damped Jacobi patch uses frozen positions
within one parallel sweep; default full/color ordering remains unchanged.
"""
import warp as wp
from newton._src.solvers.solver import SolverBase
from newton._src.geometry.tri_mesh_collision import (
    TriMeshCollisionInfo, get_edge_colliding_edges_count, get_vertex_colliding_triangles_count,
)
from newton._src.solvers.vbd.particle_vbd_kernels import (
    evaluate_edge_edge_contact_2_vertices,
    evaluate_vertex_triangle_collision_force_hessian_4_vertices,
    create_edge_edge_division_plane_closest_pt,
    create_vertex_triangle_division_plane_closest_pt, planar_truncation_t,
    accumulate_particle_body_contact_force_and_hessian, accumulate_spring_force_and_hessian,
    solve_elasticity_tile, solve_elasticity, TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE,
    apply_truncation_ts,
)
wp.set_module_options({"enable_backward": False})
PAIR_THREADS = 65536


@wp.kernel
def pack_pairs(info_array:wp.array[TriMeshCollisionInfo],
               edge_pairs:wp.array[wp.vec2i], vertex_pairs:wp.array[wp.vec2i], counts:wp.array[int]):
    i=wp.tid()
    info=info_array[0]
    if i<info.edge_colliding_edges_buffer_sizes.shape[0]:
        n=get_edge_colliding_edges_count(info,i)
        start=wp.atomic_add(counts,0,n)
        offset=info.edge_colliding_edges_offsets[i]
        for j in range(n):
            edge_pairs[start+j]=wp.vec2i(i,info.edge_colliding_edges[2*(offset+j)+1])
    if i<info.vertex_colliding_triangles_buffer_sizes.shape[0]:
        n=get_vertex_colliding_triangles_count(info,i)
        start=wp.atomic_add(counts,1,n)
        offset=info.vertex_colliding_triangles_offsets[i]
        for j in range(n):
            vertex_pairs[start+j]=wp.vec2i(i,info.vertex_colliding_triangles[2*(offset+j)+1])


@wp.kernel
def accumulate_compact_self_contact(
    # inputs
    dt: float,
    current_color: int,
    pos_prev: wp.array[wp.vec3],
    pos: wp.array[wp.vec3],
    particle_colors: wp.array[int],
    tri_indices: wp.array2d[wp.int32],
    edge_indices: wp.array2d[wp.int32],
    # self contact
    edge_pairs: wp.array[wp.vec2i],
    vertex_pairs: wp.array[wp.vec2i],
    pair_counts: wp.array[int],
    collision_radius: float,
    soft_contact_ke: float,
    soft_contact_kd: float,
    friction_mu: float,
    friction_epsilon: float,
    edge_edge_parallel_epsilon: float,
    # outputs: particle force and hessian
    particle_forces: wp.array[wp.vec3],
    particle_hessians: wp.array[wp.mat33],
):
    tid = wp.tid()
    for pair in range(tid, pair_counts[0], PAIR_THREADS):
        e1_idx = edge_pairs[pair][0]
        e2_idx = edge_pairs[pair][1]
        if e1_idx != -1 and e2_idx != -1:
            e1_v1 = edge_indices[e1_idx, 2]
            e1_v2 = edge_indices[e1_idx, 3]

            c_e1_v1 = particle_colors[e1_v1]
            c_e1_v2 = particle_colors[e1_v2]
            if c_e1_v1 == current_color or c_e1_v2 == current_color:
                has_contact, collision_force_0, collision_force_1, collision_hessian_0, collision_hessian_1 = (
                    evaluate_edge_edge_contact_2_vertices(
                        e1_idx,
                        e2_idx,
                        pos,
                        pos_prev,
                        edge_indices,
                        collision_radius,
                        soft_contact_ke,
                        soft_contact_kd,
                        friction_mu,
                        friction_epsilon,
                        dt,
                        edge_edge_parallel_epsilon,
                    )
                )

                if has_contact:
                    # here we only handle the e1 side, because e2 will also detection this contact and add force and hessian on its own
                    if c_e1_v1 == current_color:
                        wp.atomic_add(particle_forces, e1_v1, collision_force_0)
                        wp.atomic_add(particle_hessians, e1_v1, collision_hessian_0)
                    if c_e1_v2 == current_color:
                        wp.atomic_add(particle_forces, e1_v2, collision_force_1)
                        wp.atomic_add(particle_hessians, e1_v2, collision_hessian_1)

    for pair in range(tid, pair_counts[1], PAIR_THREADS):
        particle_idx = vertex_pairs[pair][0]
        tri_idx = vertex_pairs[pair][1]
        if particle_idx != -1 and tri_idx != -1:
            tri_a = tri_indices[tri_idx, 0]
            tri_b = tri_indices[tri_idx, 1]
            tri_c = tri_indices[tri_idx, 2]

            c_v = particle_colors[particle_idx]
            c_tri_a = particle_colors[tri_a]
            c_tri_b = particle_colors[tri_b]
            c_tri_c = particle_colors[tri_c]

            if (
                c_v == current_color
                or c_tri_a == current_color
                or c_tri_b == current_color
                or c_tri_c == current_color
            ):
                (
                    has_contact,
                    collision_force_0,
                    collision_force_1,
                    collision_force_2,
                    collision_force_3,
                    collision_hessian_0,
                    collision_hessian_1,
                    collision_hessian_2,
                    collision_hessian_3,
                ) = evaluate_vertex_triangle_collision_force_hessian_4_vertices(
                    particle_idx,
                    tri_idx,
                    pos,
                    pos_prev,
                    tri_indices,
                    collision_radius,
                    soft_contact_ke,
                    soft_contact_kd,
                    friction_mu,
                    friction_epsilon,
                    dt,
                )

                if has_contact:
                    # particle
                    if c_v == current_color:
                        wp.atomic_add(particle_forces, particle_idx, collision_force_3)
                        wp.atomic_add(particle_hessians, particle_idx, collision_hessian_3)

                    # tri_a
                    if c_tri_a == current_color:
                        wp.atomic_add(particle_forces, tri_a, collision_force_0)
                        wp.atomic_add(particle_hessians, tri_a, collision_hessian_0)

                    # tri_b
                    if c_tri_b == current_color:
                        wp.atomic_add(particle_forces, tri_b, collision_force_1)
                        wp.atomic_add(particle_hessians, tri_b, collision_hessian_1)

                    # tri_c
                    if c_tri_c == current_color:
                        wp.atomic_add(particle_forces, tri_c, collision_force_2)
                        wp.atomic_add(particle_hessians, tri_c, collision_hessian_2)



@wp.kernel
def truncate_compact_pairs(
    # inputs
    pos: wp.array[wp.vec3],
    displacement_in: wp.array[wp.vec3],
    tri_indices: wp.array2d[wp.int32],
    edge_indices: wp.array2d[wp.int32],
    edge_pairs: wp.array[wp.vec2i],
    vertex_pairs: wp.array[wp.vec2i],
    pair_counts: wp.array[int],
    parallel_eps: float,
    gamma: float,
    truncation_t_out: wp.array[float],
):
    tid = wp.tid()
    for pair in range(tid, pair_counts[0], PAIR_THREADS):
        e1_idx = edge_pairs[pair][0]
        e2_idx = edge_pairs[pair][1]
        if e1_idx != -1 and e2_idx != -1:
            e1_v1 = edge_indices[e1_idx, 2]
            e1_v2 = edge_indices[e1_idx, 3]

            e1_v1_pos = pos[e1_v1]
            e1_v2_pos = pos[e1_v2]

            delta_e1_v1 = displacement_in[e1_v1]
            delta_e1_v2 = displacement_in[e1_v2]

            e2_v1 = edge_indices[e2_idx, 2]
            e2_v2 = edge_indices[e2_idx, 3]

            e2_v1_pos = pos[e2_v1]
            e2_v2_pos = pos[e2_v2]

            delta_e2_v1 = displacement_in[e2_v1]
            delta_e2_v2 = displacement_in[e2_v2]

            # n points to the edge 1 side
            is_dummy, n, d = create_edge_edge_division_plane_closest_pt(
                e1_v1_pos,
                delta_e1_v1,
                e1_v2_pos,
                delta_e1_v2,
                e2_v1_pos,
                delta_e2_v1,
                e2_v2_pos,
                delta_e2_v2,
            )

            # For each, check the corresponding is_dummy entry in the vec4 is_dummy
            if not is_dummy[0]:
                t = planar_truncation_t(e1_v1_pos, delta_e1_v1, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, e1_v1, t)
            if not is_dummy[1]:
                t = planar_truncation_t(e1_v2_pos, delta_e1_v2, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, e1_v2, t)
            if not is_dummy[2]:
                t = planar_truncation_t(e2_v1_pos, delta_e2_v1, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, e2_v1, t)
            if not is_dummy[3]:
                t = planar_truncation_t(e2_v2_pos, delta_e2_v2, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, e2_v2, t)

            # planar truncation for 2 sides

    for pair in range(tid, pair_counts[1], PAIR_THREADS):
        particle_idx = vertex_pairs[pair][0]
        tri_idx = vertex_pairs[pair][1]
        colliding_particle_pos = pos[particle_idx]
        colliding_particle_displacement = displacement_in[particle_idx]
        if particle_idx != -1 and tri_idx != -1:
            tri_a = tri_indices[tri_idx, 0]
            tri_b = tri_indices[tri_idx, 1]
            tri_c = tri_indices[tri_idx, 2]

            t1 = pos[tri_a]
            t2 = pos[tri_b]
            t3 = pos[tri_c]
            delta_t1 = displacement_in[tri_a]
            delta_t2 = displacement_in[tri_b]
            delta_t3 = displacement_in[tri_c]

            is_dummy, n, d = create_vertex_triangle_division_plane_closest_pt(
                colliding_particle_pos,
                colliding_particle_displacement,
                t1,
                delta_t1,
                t2,
                delta_t2,
                t3,
                delta_t3,
            )

            # planar truncation for 2 sides
            if not is_dummy[0]:
                t = planar_truncation_t(
                    colliding_particle_pos, colliding_particle_displacement, n, d, parallel_eps, gamma
                )
                wp.atomic_min(truncation_t_out, particle_idx, t)
            if not is_dummy[1]:
                t = planar_truncation_t(t1, delta_t1, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, tri_a, t)
            if not is_dummy[2]:
                t = planar_truncation_t(t2, delta_t2, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, tri_b, t)
            if not is_dummy[3]:
                t = planar_truncation_t(t3, delta_t3, n, d, parallel_eps, gamma)
                wp.atomic_min(truncation_t_out, tri_c, t)




class CompactContactMixin:
    """Private Newton 1.6 interface; used only by the version-checked overlay."""
    def _init_compact_contacts(self):
        if self.particle_enable_self_contact:
            info=self.trimesh_collision_detector.collision_info
            self.compact_edges=wp.empty(info.edge_colliding_edges.size//2,dtype=wp.vec2i,device=self.device)
            self.compact_vertices=wp.empty(info.vertex_colliding_triangles.size//2,dtype=wp.vec2i,device=self.device)
            self.compact_counts=wp.zeros(2,dtype=int,device=self.device)

    def _collision_detection_penetration_free(self,current_state):
        super()._collision_detection_penetration_free(current_state)
        self.compact_counts.zero_()
        wp.launch(pack_pairs,dim=max(self.model.particle_count,self.model.edge_count),
                  inputs=[self.trimesh_collision_info,self.compact_edges,self.compact_vertices,self.compact_counts],device=self.device)

    def _solve_particle_iteration(
        self, state_in, state_out, contacts, dt: float, iter_num: int
    ):
        """Solve one VBD iteration for particles."""
        model = self.model

        # Select rigid-body poses for particle-rigid contact evaluation
        if self.integrate_with_external_rigid_solver:
            body_q_for_particles = state_out.body_q
            body_q_prev_for_particles = state_in.body_q
            body_qd_for_particles = state_out.body_qd
        else:
            body_q_for_particles = state_in.body_q
            if model.body_count > 0:
                body_q_prev_for_particles = self.body_q_prev
            else:
                body_q_prev_for_particles = None
            body_qd_for_particles = state_in.body_qd

        # Early exit if no particles
        if model.particle_count == 0:
            return

        # Update collision detection if needed (penetration-free mode only)
        if self.particle_enable_self_contact:
            _Frequency = SolverBase.CollisionFrequencyType
            if (self._sc_mode_this_step == _Frequency.PRE_POST_INIT and iter_num == 0) or (
                self._sc_mode_this_step == _Frequency.ITERATIONS and (iter_num + 1) % self._sc_freq_this_step == 0
            ):
                self._collision_detection_penetration_free(state_in)

        local_only=bool(getattr(self,'local_sweep',False))
        patch=getattr(self,'local_patch',None)
        if local_only:
            patch.refresh(self,state_in,contacts)
        patch_jacobi=local_only and getattr(patch,'jacobi',False)
        contact_colors=(patch.parallel_colors if patch_jacobi else patch.colors) if local_only else model.particle_colors
        # Zero out forces and hessians
        self.particle_forces.zero_()
        self.particle_hessians.zero_()

        # Iterate over color groups
        groups=[patch.all_ids] if patch_jacobi else self.model.particle_color_groups
        for color,group in enumerate(groups):
            if contacts is not None:
                wp.launch(
                    kernel=accumulate_particle_body_contact_force_and_hessian,
                    dim=contacts.soft_contact_max,
                    inputs=[
                        dt,
                        color,
                        self.particle_q_prev,
                        state_in.particle_q,
                        contact_colors,
                        # body-particle contact
                        self.friction_epsilon,
                        model.particle_radius,
                        contacts.soft_contact_indices,
                        contacts.soft_contact_count,
                        contacts.soft_contact_max,
                        self.body_particle_contact_penalty_k,
                        self.body_particle_contact_material_ke,
                        self.body_particle_contact_material_kd,
                        self.body_particle_contact_material_mu,
                        model.shape_body,
                        body_q_for_particles,
                        body_q_prev_for_particles,
                        body_qd_for_particles,
                        model.body_com,
                        contacts.soft_contact_shape,
                        contacts.soft_contact_body_pos,
                        contacts.soft_contact_body_vel,
                        contacts.soft_contact_normal,
                        model.shape_margin,
                        contacts.soft_contact_barycentric,
                    ],
                    outputs=[
                        self.particle_forces,
                        self.particle_hessians,
                    ],
                    device=self.device,
                )

            if model.spring_count:
                wp.launch(
                    kernel=accumulate_spring_force_and_hessian,
                    inputs=[
                        dt,
                        color,
                        self.particle_q_prev,
                        state_in.particle_q,
                        contact_colors,
                        model.spring_count,
                        self.model.spring_indices,
                        self.model.spring_rest_length,
                        self.model.spring_stiffness,
                        self.model.spring_damping,
                    ],
                    outputs=[self.particle_forces, self.particle_hessians],
                    dim=model.spring_count,
                    device=self.device,
                )

            if self.particle_enable_self_contact:
                wp.launch(
                    kernel=accumulate_compact_self_contact,
                    dim=PAIR_THREADS,
                    inputs=[
                        dt,
                        color,
                        self.particle_q_prev,
                        state_in.particle_q,
                        contact_colors,
                        self.model.tri_indices,
                        self.model.edge_indices,
                        # self-contact
                        self.compact_edges, self.compact_vertices, self.compact_counts,
                        self.particle_self_contact_margin,
                        self.model.soft_contact_ke,
                        self.model.soft_contact_kd,
                        self.model.soft_contact_mu,
                        self.friction_epsilon,
                        self._self_contact_edge_edge_parallel_epsilon,
                    ],
                    outputs=[self.particle_forces, self.particle_hessians],
                    device=self.device,
                    max_blocks=self.model.device.sm_count,
                )
            small_config=getattr(self,'small_bend_config',None)
            use_small_kernel=small_config is not None or local_only
            if use_small_kernel:
                from .small_bend_kernels import solve_elasticity_tile_small_bend, solve_elasticity_small_bend
            small_inputs=([getattr(self,'small_bend_dual',model.edge_rest_length),state_in.cardboard.accumulated_angle,
                           *(small_config or (1.,.3,5.9)),getattr(self,'small_bend_memory_curvature',0.),getattr(self,'crease_friction_curvature',0.),int(local_only),patch.mask if local_only else model.particle_flags]
                          if use_small_kernel else [])
            if patch_jacobi:wp.copy(patch.previous_displacements,self.particle_displacements)
            if self.use_particle_tile_solve:
                wp.launch(
                    kernel=solve_elasticity_tile_small_bend if use_small_kernel else solve_elasticity_tile,
                    dim=group.size * TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE,
                    block_dim=TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE,
                    inputs=[
                        dt,
                        group,
                        self.particle_q_prev,
                        state_in.particle_q,
                        self.model.particle_mass,
                        self.inertia,
                        self.model.particle_flags,
                        self.model.tri_indices,
                        self.model.tri_poses,
                        self.model.tri_materials,
                        self.model.tri_areas,
                        self.model.edge_indices,
                        self.model.edge_rest_angle,
                        self.model.edge_rest_length,
                        self.model.edge_bending_properties,
                        self.model.tet_indices,
                        self.model.tet_poses,
                        self.model.tet_materials,
                        self.particle_adjacency,
                        self.particle_forces,
                        self.particle_hessians,
                    ]+small_inputs,
                    outputs=[
                        self.particle_displacements,
                    ],
                    device=self.device,
                )
            else:
                wp.launch(
                    kernel=solve_elasticity_small_bend if use_small_kernel else solve_elasticity,
                    dim=group.size,
                    inputs=[
                        dt,
                        group,
                        self.particle_q_prev,
                        state_in.particle_q,
                        self.model.particle_mass,
                        self.inertia,
                        self.model.particle_flags,
                        self.model.tri_indices,
                        self.model.tri_poses,
                        self.model.tri_materials,
                        self.model.tri_areas,
                        self.model.edge_indices,
                        self.model.edge_rest_angle,
                        self.model.edge_rest_length,
                        self.model.edge_bending_properties,
                        self.model.tet_indices,
                        self.model.tet_poses,
                        self.model.tet_materials,
                        self.particle_adjacency,
                        self.particle_forces,
                        self.particle_hessians,
                    ]+small_inputs,
                    outputs=[
                        self.particle_displacements,
                    ],
                    device=self.device,
                )
            if patch_jacobi:
                from .local_vbd import damp_patch_update
                wp.launch(damp_patch_update,dim=model.particle_count,
                    inputs=[patch.previous_displacements,self.particle_displacements,patch.relaxation],device=self.device)
            self._penetration_free_truncation(state_in.particle_q)

        wp.copy(state_out.particle_q, state_in.particle_q)

    def _penetration_free_truncation(self, particle_q_out=None):
        """
        Modify displacements_in in-place, also modify particle_q if its not None

        """
        if not self.particle_enable_self_contact:
            self.truncation_ts.fill_(1.0)
            wp.launch(
                kernel=apply_truncation_ts,
                dim=self.model.particle_count,
                inputs=[
                    self.pos_prev_collision_detection,  # pos: wp.array[wp.vec3],
                    self.particle_displacements,  # displacement_in: wp.array[wp.vec3],
                    self.truncation_ts,  # truncation_ts: wp.array[float],
                    wp.inf,  # max_displacement: float (input threshold)
                ],
                outputs=[
                    self.particle_displacements,  # displacement_out: wp.array[wp.vec3],
                    particle_q_out,  # pos_out: wp.array[wp.vec3],
                ],
                device=self.device,
            )

        else:
            ##  parallel by collision and atomic operation
            self.truncation_ts.fill_(1.0)
            wp.launch(
                kernel=truncate_compact_pairs,
                inputs=[
                    self.pos_prev_collision_detection,  # pos_prev_collision_detection: wp.array[wp.vec3],
                    self.particle_displacements,  # particle_displacements: wp.array[wp.vec3],
                    self.model.tri_indices,
                    self.model.edge_indices,
                    self.compact_edges, self.compact_vertices, self.compact_counts,
                    self._self_contact_edge_edge_parallel_epsilon,
                    self.particle_conservative_bound_relaxation,
                ],
                outputs=[
                    self.truncation_ts,
                ],
                dim=PAIR_THREADS,
                device=self.device,
            )

            wp.launch(
                kernel=apply_truncation_ts,
                dim=self.model.particle_count,
                inputs=[
                    self.pos_prev_collision_detection,
                    self.particle_displacements,
                    self.truncation_ts,
                    self._self_contact_query_radius
                    * self.particle_conservative_bound_relaxation
                    * 0.5,  # max_displacement: degenerate to isotropic truncation
                ],
                outputs=[
                    self.particle_displacements,
                    particle_q_out,
                ],
                device=self.device,
            )
