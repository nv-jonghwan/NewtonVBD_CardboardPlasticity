# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Optional pristine-hinge bending law, adapted from pinned Newton1.6 kernels.

Only the elastic moment and its positive Gauss-Newton tangent differ. Contact,
membrane/tet forces, damping, geometry derivatives and reductions are upstream.
The original large-curvature law is recovered before yielding. Default path
continues to call upstream kernels directly. See outputs/small_bend_trial/upstream.json.
"""
import warp as wp
from .plasticity import dihedral
from newton._src.solvers.vbd.particle_vbd_kernels import *
wp.set_module_options({"enable_backward": False})

@wp.func
def reinforcement_memory(alpha:float,dual:float,transition_curvature:float):
    # Zero retains the original binary pristine/yielded law. The opt-in law
    # depends on irreversible plastic curvature, never on phase or contact.
    if transition_curvature<=0.0:
        return float(alpha<=0.0)
    x=wp.clamp(alpha/(dual*transition_curvature),0.0,1.0)
    return 1.0-x*x*(3.0-2.0*x)

@wp.func
def small_bend_moment_tangent(elastic:float,k:float,dual:float,alpha:float,scale:float,knee:float,end:float):
    curvature=wp.abs(elastic)/dual
    if scale<=1.0 or alpha>0.0 or curvature>=end:
        return wp.vec2(k*elastic,k)
    if curvature<=knee:
        return wp.vec2(scale*k*elastic,scale*k)
    slope=(end-scale*knee)/(end-knee)
    moment=wp.sign(elastic)*k*dual*(scale*knee+slope*(curvature-knee))
    return wp.vec2(moment,k*slope)

@wp.func
def crease_friction_moment(delta:float,limit:float,epsilon:float):
    # Huber regularization of limit*abs(delta): a convex dissipation
    # potential. epsilon scales with dt so residual creep has a fixed rate.
    # Outside the quadratic core use a positive quadratic majorizer (IRLS),
    # not the zero exact tangent, to avoid overshooting the sticking angle.
    if wp.abs(delta)<epsilon:
        return wp.vec2(limit*delta/epsilon,limit/epsilon)
    return wp.vec2(limit*wp.sign(delta),limit/wp.abs(delta))

@wp.kernel
def release_small_bend_energy(old_alpha:wp.array[float],new_alpha:wp.array[float],
                              ke:wp.array[float],length:wp.array[float],dual:wp.array[float],
                              scale:float,knee:float,end:float,work:wp.array[float]):
    i=wp.tid()
    if old_alpha[i]==0.0 and new_alpha[i]>0.0:
        # Integral of extra moment up to end; constant thereafter. Original
        # yield lies above end. Removing the pristine branch dissipates this
        # stored energy exactly once, rather than silently deleting it.
        work[i]+=0.5*ke[i]*length[i]*dual[i]*dual[i]*(scale-1.0)*knee*end

@wp.kernel
def release_memory_energy(old_alpha:wp.array[float],new_alpha:wp.array[float],
                          old_damage:wp.array[float],new_damage:wp.array[float],
                          ke:wp.array[float],length:wp.array[float],dual:wp.array[float],
                          scale:float,knee:float,end:float,memory:float,work:wp.array[float]):
    i=wp.tid()
    before=(1.0-old_damage[i])*reinforcement_memory(old_alpha[i],dual[i],memory)
    after=(1.0-new_damage[i])*reinforcement_memory(new_alpha[i],dual[i],memory)
    # History changes only in return mapping above yield curvature > end.
    # The additional potential is then a constant offset. Its lost amount
    # telescopes over substeps, including damage of the remaining reinforcement.
    extra=0.5*ke[i]*length[i]*dual[i]*dual[i]*(scale-1.0)*knee*end
    work[i]+=wp.max(0.0,before-after)*extra

@wp.func
def evaluate_small_bend_force_hessian(
    bending_index: int,
    v_order: int,
    pos: wp.array[wp.vec3],
    pos_anchor: wp.array[wp.vec3],
    edge_indices: wp.array2d[wp.int32],
    edge_rest_angle: wp.array[float],
    edge_rest_length: wp.array[float],
    stiffness: float,
    damping: float,
    dt: float,
    small_dual: wp.array[float],
    small_alpha: wp.array[float],
    small_scale: float,
    small_knee: float,
    small_end: float,
    small_memory: float = 0.0,
    crease_friction: float = 0.0,
):
    # Skip invalid edges (boundary edges with missing opposite vertices)
    if edge_indices[bending_index, 0] == -1 or edge_indices[bending_index, 1] == -1:
        return wp.vec3(0.0), wp.mat33(0.0)

    eps = 1.0e-6

    vi0 = edge_indices[bending_index, 0]
    vi1 = edge_indices[bending_index, 1]
    vi2 = edge_indices[bending_index, 2]
    vi3 = edge_indices[bending_index, 3]

    x0 = pos[vi0]  # opposite 0
    x1 = pos[vi1]  # opposite 1
    x2 = pos[vi2]  # edge start
    x3 = pos[vi3]  # edge end

    # Compute edge vectors
    x02 = x2 - x0
    x03 = x3 - x0
    x13 = x3 - x1
    x12 = x2 - x1
    e = x3 - x2

    # Compute normals
    n1 = wp.cross(x02, x03)
    n2 = wp.cross(x13, x12)

    n1_norm = wp.length(n1)
    n2_norm = wp.length(n2)
    e_norm = wp.length(e)

    # Early exit for degenerate cases
    if n1_norm < eps or n2_norm < eps or e_norm < eps:
        return wp.vec3(0.0), wp.mat33(0.0)

    n1_hat = n1 / n1_norm
    n2_hat = n2 / n2_norm
    e_hat = e / e_norm

    sin_theta = wp.dot(wp.cross(n1_hat, n2_hat), e_hat)
    cos_theta = wp.dot(n1_hat, n2_hat)
    theta = wp.atan2(sin_theta, cos_theta)

    k = stiffness * edge_rest_length[bending_index]
    memory_weight=reinforcement_memory(small_alpha[bending_index],small_dual[bending_index],small_memory)
    effective_scale=1.0+(small_scale-1.0)*memory_weight
    moment_tangent = small_bend_moment_tangent(theta-edge_rest_angle[bending_index], k, small_dual[bending_index], 0.0, effective_scale, small_knee, small_end)
    dE_dtheta = moment_tangent[0]

    # Pre-compute skew matrices (shared across all angle derivative computations)
    skew_e = wp.skew(e)
    skew_x03 = wp.skew(x03)
    skew_x02 = wp.skew(x02)
    skew_x13 = wp.skew(x13)
    skew_x12 = wp.skew(x12)
    skew_n1 = wp.skew(n1_hat)
    skew_n2 = wp.skew(n2_hat)

    # Compute the derivatives of unit normals with respect to each vertex; required for computing angle derivatives
    dn1hat_dx0 = compute_normalized_vector_derivative(n1_norm, n1_hat, skew_e)
    dn2hat_dx0 = wp.mat33(0.0)

    dn1hat_dx1 = wp.mat33(0.0)
    dn2hat_dx1 = compute_normalized_vector_derivative(n2_norm, n2_hat, -skew_e)

    dn1hat_dx2 = compute_normalized_vector_derivative(n1_norm, n1_hat, -skew_x03)
    dn2hat_dx2 = compute_normalized_vector_derivative(n2_norm, n2_hat, skew_x13)

    dn1hat_dx3 = compute_normalized_vector_derivative(n1_norm, n1_hat, skew_x02)
    dn2hat_dx3 = compute_normalized_vector_derivative(n2_norm, n2_hat, -skew_x12)

    # Compute all angle derivatives (required for damping)
    dtheta_dx0 = compute_angle_derivative(
        n1_hat, n2_hat, e_hat, dn1hat_dx0, dn2hat_dx0, sin_theta, cos_theta, skew_n1, skew_n2
    )
    dtheta_dx1 = compute_angle_derivative(
        n1_hat, n2_hat, e_hat, dn1hat_dx1, dn2hat_dx1, sin_theta, cos_theta, skew_n1, skew_n2
    )
    dtheta_dx2 = compute_angle_derivative(
        n1_hat, n2_hat, e_hat, dn1hat_dx2, dn2hat_dx2, sin_theta, cos_theta, skew_n1, skew_n2
    )
    dtheta_dx3 = compute_angle_derivative(
        n1_hat, n2_hat, e_hat, dn1hat_dx3, dn2hat_dx3, sin_theta, cos_theta, skew_n1, skew_n2
    )

    # Use float masks for branch-free selection
    mask0 = float(v_order == 0)
    mask1 = float(v_order == 1)
    mask2 = float(v_order == 2)
    mask3 = float(v_order == 3)

    # Select the derivative for the current vertex without branching
    dtheta_dx = dtheta_dx0 * mask0 + dtheta_dx1 * mask1 + dtheta_dx2 * mask2 + dtheta_dx3 * mask3

    # Compute elastic force and hessian
    bending_force = -dE_dtheta * dtheta_dx
    bending_hessian = moment_tangent[1] * wp.outer(dtheta_dx, dtheta_dx)

    if damping > 0.0 or crease_friction > 0.0:
        inv_dt = 1.0 / dt
        x_prev0 = pos_anchor[vi0]
        x_prev1 = pos_anchor[vi1]
        x_prev2 = pos_anchor[vi2]
        x_prev3 = pos_anchor[vi3]

        x02_prev = x_prev2 - x_prev0
        x03_prev = x_prev3 - x_prev0
        x13_prev = x_prev3 - x_prev1
        x12_prev = x_prev2 - x_prev1
        e_prev = x_prev3 - x_prev2

        n1_prev_raw = wp.cross(x02_prev, x03_prev)
        n2_prev_raw = wp.cross(x13_prev, x12_prev)
        n1_prev_norm = wp.length(n1_prev_raw)
        n2_prev_norm = wp.length(n2_prev_raw)
        e_prev_norm = wp.length(e_prev)
        if n1_prev_norm < eps or n2_prev_norm < eps or e_prev_norm < eps:
            return bending_force, bending_hessian

        n1_prev = n1_prev_raw / n1_prev_norm
        n2_prev = n2_prev_raw / n2_prev_norm
        e_hat_prev = e_prev / e_prev_norm

        sin_theta_prev = wp.dot(wp.cross(n1_prev, n2_prev), e_hat_prev)
        cos_theta_prev = wp.dot(n1_prev, n2_prev)
        theta_prev = wp.atan2(sin_theta_prev, cos_theta_prev)

        dtheta = theta - theta_prev
        if dtheta > 3.141592653589793:
            dtheta = dtheta - 6.283185307179586
        elif dtheta < -3.141592653589793:
            dtheta = dtheta + 6.283185307179586

        dtheta_dt = dtheta * inv_dt

        rest_len = edge_rest_length[bending_index]
        damping_force = -damping * rest_len * dtheta_dt * dtheta_dx
        damping_hessian = damping * rest_len * inv_dt * wp.outer(dtheta_dx, dtheta_dx)

        bending_force = bending_force + damping_force
        bending_hessian = bending_hessian + damping_hessian
        # Convex incremental dissipation centered on the prior substep angle.
        # Only irreversible creases contribute. The capped torque always
        # opposes angular motion; it neither resets the plastic angle nor
        # depends on the gripper/timeline. epsilon is an angular slip rate.
        if crease_friction > 0.0:
            activation=1.0-wp.exp(-small_alpha[bending_index]/(5.0*small_dual[bending_index]))
            limit=k*small_dual[bending_index]*crease_friction*activation
            regularization=dt*0.005
            friction=crease_friction_moment(dtheta,limit,regularization)
            bending_force-=friction[0]*dtheta_dx
            bending_hessian+=friction[1]*wp.outer(dtheta_dx,dtheta_dx)

    return bending_force, bending_hessian


@wp.kernel
def solve_elasticity_tile_small_bend(
    dt: float,
    particle_ids_in_color: wp.array[wp.int32],
    pos_prev: wp.array[wp.vec3],
    pos: wp.array[wp.vec3],
    mass: wp.array[float],
    inertia: wp.array[wp.vec3],
    particle_flags: wp.array[wp.int32],
    tri_indices: wp.array2d[wp.int32],
    tri_poses: wp.array[wp.mat22],
    tri_materials: wp.array2d[float],
    tri_areas: wp.array[float],
    edge_indices: wp.array2d[wp.int32],
    edge_rest_angles: wp.array[float],
    edge_rest_length: wp.array[float],
    edge_bending_properties: wp.array2d[float],
    tet_indices: wp.array2d[wp.int32],
    tet_poses: wp.array[wp.mat33],
    tet_materials: wp.array2d[float],
    particle_adjacency: MeshAdjacencyData,
    particle_forces: wp.array[wp.vec3],
    particle_hessians: wp.array[wp.mat33],
    small_dual: wp.array[float],
    small_alpha: wp.array[float],
    small_scale: float,
    small_knee: float,
    small_end: float,
    small_memory: float,
    crease_friction: float,
    local_only: int,
    active_mask: wp.array[wp.int32],
    # output
    particle_displacements: wp.array[wp.vec3],
):
    tid = wp.tid()
    block_idx = tid // TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE
    thread_idx = tid % TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE
    particle_index = particle_ids_in_color[block_idx]

    # Preserve displacement already accumulated from ROM/previous colors.
    if local_only != 0 and active_mask[particle_index] == 0:
        return

    if not particle_flags[particle_index] & ParticleFlags.ACTIVE or mass[particle_index] == 0:
        if thread_idx == 0:
            particle_displacements[particle_index] = wp.vec3(0.0)
        return

    dt_sqr_reciprocal = 1.0 / (dt * dt)

    # elastic force and hessian
    f = wp.vec3(0.0)
    h = wp.mat33(0.0)

    batch_counter = wp.int32(0)

    if tri_indices.shape[0] > 0:
        num_adj_faces = get_vertex_num_adjacent_faces(particle_adjacency, particle_index)
        # loop through all the adjacent triangles using whole block
        while batch_counter + thread_idx < num_adj_faces:
            adj_tri_counter = thread_idx + batch_counter
            batch_counter += TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE
            # elastic force and hessian
            tri_index, vertex_order = get_vertex_adjacent_face_id_order(
                particle_adjacency, particle_index, adj_tri_counter
            )

            # fmt: off
            if wp.static("connectivity" in VBD_DEBUG_PRINTING_OPTIONS):
                wp.printf(
                    "particle: %d | num_adj_faces: %d | ",
                    particle_index,
                    get_vertex_num_adjacent_faces(particle_adjacency, particle_index),
                )
                wp.printf("i_face: %d | face id: %d | v_order: %d | ", adj_tri_counter, tri_index, vertex_order)
                wp.printf(
                    "face: %d %d %d\n",
                    tri_indices[tri_index, 0],
                    tri_indices[tri_index, 1],
                    tri_indices[tri_index, 2],
                )
            # fmt: on

            if tri_materials[tri_index, 0] > 0.0 or tri_materials[tri_index, 1] > 0.0:
                f_tri, h_tri = evaluate_neo_hookean_membrane_force_hessian(
                    tri_index,
                    vertex_order,
                    pos,
                    pos_prev,
                    tri_indices,
                    tri_poses[tri_index],
                    tri_areas[tri_index],
                    tri_materials[tri_index, 0],
                    tri_materials[tri_index, 1],
                    tri_materials[tri_index, 2],
                    dt,
                )

                f += f_tri
                h += h_tri

    if edge_indices.shape[0] > 0:
        batch_counter = wp.int32(0)
        num_adj_edges = get_vertex_num_adjacent_edges(particle_adjacency, particle_index)
        while batch_counter + thread_idx < num_adj_edges:
            adj_edge_counter = batch_counter + thread_idx
            batch_counter += TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE
            nei_edge_index, vertex_order_on_edge = get_vertex_adjacent_edge_id_order(
                particle_adjacency, particle_index, adj_edge_counter
            )
            if edge_bending_properties[nei_edge_index, 0] > 0.0:
                f_edge, h_edge = evaluate_small_bend_force_hessian(
                    nei_edge_index,
                    vertex_order_on_edge,
                    pos,
                    pos_prev,
                    edge_indices,
                    edge_rest_angles,
                    edge_rest_length,
                    edge_bending_properties[nei_edge_index, 0],
                    edge_bending_properties[nei_edge_index, 1],
                    dt,
                    small_dual, small_alpha, small_scale, small_knee, small_end, small_memory, crease_friction,
                )

                f += f_edge
                h += h_edge

    if tet_indices.shape[0] > 0:
        # solve tet elasticity
        batch_counter = wp.int32(0)
        num_adj_tets = get_vertex_num_adjacent_tets(particle_adjacency, particle_index)
        while batch_counter + thread_idx < num_adj_tets:
            adj_tet_counter = batch_counter + thread_idx
            batch_counter += TILE_SIZE_TRI_MESH_ELASTICITY_SOLVE
            nei_tet_index, vertex_order_on_tet = get_vertex_adjacent_tet_id_order(
                particle_adjacency, particle_index, adj_tet_counter
            )
            if tet_materials[nei_tet_index, 0] > 0.0 or tet_materials[nei_tet_index, 1] > 0.0:
                f_tet, h_tet = evaluate_volumetric_neo_hookean_force_and_hessian(
                    nei_tet_index,
                    vertex_order_on_tet,
                    pos_prev,
                    pos,
                    tet_indices,
                    tet_poses[nei_tet_index],
                    tet_materials[nei_tet_index, 0],
                    tet_materials[nei_tet_index, 1],
                    tet_materials[nei_tet_index, 2],
                    dt,
                )

                f += f_tet
                h += h_tet

    f_tile = wp.tile(f, preserve_type=True)
    h_tile = wp.tile(h, preserve_type=True)

    f_total = wp.tile_reduce(wp.add, f_tile)[0]
    h_total = wp.tile_reduce(wp.add, h_tile)[0]

    if thread_idx == 0:
        h_total = (
            h_total
            + mass[particle_index] * dt_sqr_reciprocal * wp.identity(n=3, dtype=float)
            + particle_hessians[particle_index]
        )
        if abs(wp.determinant(h_total)) > 1e-8:
            h_inv = wp.inverse(h_total)
            f_total = (
                f_total
                + mass[particle_index] * (inertia[particle_index] - pos[particle_index]) * (dt_sqr_reciprocal)
                + particle_forces[particle_index]
            )
            particle_displacements[particle_index] = particle_displacements[particle_index] + h_inv * f_total


@wp.kernel
def solve_elasticity_small_bend(
    dt: float,
    particle_ids_in_color: wp.array[wp.int32],
    pos_prev: wp.array[wp.vec3],
    pos: wp.array[wp.vec3],
    mass: wp.array[float],
    inertia: wp.array[wp.vec3],
    particle_flags: wp.array[wp.int32],
    tri_indices: wp.array2d[wp.int32],
    tri_poses: wp.array[wp.mat22],
    tri_materials: wp.array2d[float],
    tri_areas: wp.array[float],
    edge_indices: wp.array2d[wp.int32],
    edge_rest_angles: wp.array[float],
    edge_rest_length: wp.array[float],
    edge_bending_properties: wp.array2d[float],
    tet_indices: wp.array2d[wp.int32],
    tet_poses: wp.array[wp.mat33],
    tet_materials: wp.array2d[float],
    particle_adjacency: MeshAdjacencyData,
    particle_forces: wp.array[wp.vec3],
    particle_hessians: wp.array[wp.mat33],
    small_dual: wp.array[float],
    small_alpha: wp.array[float],
    small_scale: float,
    small_knee: float,
    small_end: float,
    small_memory: float,
    crease_friction: float,
    local_only: int,
    active_mask: wp.array[wp.int32],
    # output
    particle_displacements: wp.array[wp.vec3],
):
    t_id = wp.tid()

    particle_index = particle_ids_in_color[t_id]

    if local_only != 0 and active_mask[particle_index] == 0:
        return

    if not particle_flags[particle_index] & ParticleFlags.ACTIVE or mass[particle_index] == 0:
        particle_displacements[particle_index] = wp.vec3(0.0)
        return

    dt_sqr_reciprocal = 1.0 / (dt * dt)

    # inertia force and hessian
    f = mass[particle_index] * (inertia[particle_index] - pos[particle_index]) * (dt_sqr_reciprocal)
    h = mass[particle_index] * dt_sqr_reciprocal * wp.identity(n=3, dtype=float)

    # fmt: off
    if wp.static("inertia_force_hessian" in VBD_DEBUG_PRINTING_OPTIONS):
        wp.printf(
            "particle: %d after accumulate inertia\nforce:\n %f %f %f, \nhessian:, \n%f %f %f, \n%f %f %f, \n%f %f %f\n",
            particle_index,
            f[0], f[1], f[2], h[0, 0], h[0, 1], h[0, 2], h[1, 0], h[1, 1], h[1, 2], h[2, 0], h[2, 1], h[2, 2],
        )

    if tri_indices.shape[0] > 0:
        # elastic force and hessian
        for i_adj_tri in range(get_vertex_num_adjacent_faces(particle_adjacency, particle_index)):
            tri_index, vertex_order = get_vertex_adjacent_face_id_order(particle_adjacency, particle_index, i_adj_tri)

            # fmt: off
            if wp.static("connectivity" in VBD_DEBUG_PRINTING_OPTIONS):
                wp.printf(
                    "particle: %d | num_adj_faces: %d | ",
                    particle_index,
                    get_vertex_num_adjacent_faces(particle_adjacency, particle_index),
                )
                wp.printf("i_face: %d | face id: %d | v_order: %d | ", i_adj_tri, tri_index, vertex_order)
                wp.printf(
                    "face: %d %d %d\n",
                    tri_indices[tri_index, 0],
                    tri_indices[tri_index, 1],
                    tri_indices[tri_index, 2],
                )
            # fmt: on

            if tri_materials[tri_index, 0] > 0.0 or tri_materials[tri_index, 1] > 0.0:
                f_tri, h_tri = evaluate_neo_hookean_membrane_force_hessian(
                    tri_index,
                    vertex_order,
                    pos,
                    pos_prev,
                    tri_indices,
                    tri_poses[tri_index],
                    tri_areas[tri_index],
                    tri_materials[tri_index, 0],
                    tri_materials[tri_index, 1],
                    tri_materials[tri_index, 2],
                    dt,
                )

                f = f + f_tri
                h = h + h_tri

    if edge_indices.shape[0] > 0:
        for i_adj_edge in range(get_vertex_num_adjacent_edges(particle_adjacency, particle_index)):
            nei_edge_index, vertex_order_on_edge = get_vertex_adjacent_edge_id_order(particle_adjacency, particle_index, i_adj_edge)
            # vertex is on the edge; otherwise it only effects the bending energy n
            if edge_bending_properties[nei_edge_index, 0] > 0.0:
                f_edge, h_edge = evaluate_small_bend_force_hessian(
                    nei_edge_index, vertex_order_on_edge, pos, pos_prev, edge_indices, edge_rest_angles, edge_rest_length,
                    edge_bending_properties[nei_edge_index, 0], edge_bending_properties[nei_edge_index, 1], dt, small_dual, small_alpha, small_scale, small_knee, small_end, small_memory, crease_friction
                )

                f = f + f_edge
                h = h + h_edge

    if tet_indices.shape[0] > 0:
        # solve tet elasticity
        num_adj_tets = get_vertex_num_adjacent_tets(particle_adjacency, particle_index)
        for adj_tet_counter in range(num_adj_tets):
            nei_tet_index, vertex_order_on_tet = get_vertex_adjacent_tet_id_order(
                particle_adjacency, particle_index, adj_tet_counter
            )
            if tet_materials[nei_tet_index, 0] > 0.0 or tet_materials[nei_tet_index, 1] > 0.0:
                f_tet, h_tet = evaluate_volumetric_neo_hookean_force_and_hessian(
                    nei_tet_index,
                    vertex_order_on_tet,
                    pos_prev,
                    pos,
                    tet_indices,
                    tet_poses[nei_tet_index],
                    tet_materials[nei_tet_index, 0],
                    tet_materials[nei_tet_index, 1],
                    tet_materials[nei_tet_index, 2],
                    dt,
                )

                f += f_tet
                h += h_tet

    # fmt: off
    if wp.static("overall_force_hessian" in VBD_DEBUG_PRINTING_OPTIONS):
        wp.printf(
            "vertex: %d final\noverall force:\n %f %f %f, \noverall hessian:, \n%f %f %f, \n%f %f %f, \n%f %f %f\n",
            particle_index,
            f[0], f[1], f[2], h[0, 0], h[0, 1], h[0, 2], h[1, 0], h[1, 1], h[1, 2], h[2, 0], h[2, 1], h[2, 2],
        )

    # fmt: on
    h = h + particle_hessians[particle_index]
    f = f + particle_forces[particle_index]

    if abs(wp.determinant(h)) > 1e-8:
        h_inv = wp.inverse(h)
        particle_displacements[particle_index] = particle_displacements[particle_index] + h_inv * f


@wp.kernel
def record_crease_dissipation(previous:wp.array[wp.vec3],current:wp.array[wp.vec3],
    edges:wp.array2d[int],lengths:wp.array[float],dual:wp.array[float],alpha:wp.array[float],
    properties:wp.array2d[float],curvature:float,dt:float,work:wp.array[float]):
    i=wp.tid()
    if edges[i,0]>=0 and edges[i,1]>=0:
        a=edges[i,0];b=edges[i,1];c=edges[i,2];d=edges[i,3]
        delta=dihedral(current[a],current[b],current[c],current[d])-dihedral(previous[a],previous[b],previous[c],previous[d])
        if delta>3.141592653589793:delta-=6.283185307179586
        elif delta < -3.141592653589793:delta+=6.283185307179586
        activation=1.0-wp.exp(-alpha[i]/(5.0*dual[i]))
        limit=properties[i,0]*lengths[i]*dual[i]*curvature*activation
        work[i]+=crease_friction_moment(delta,limit,dt*.005)[0]*delta
