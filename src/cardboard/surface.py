"""Crease-aware cubic display patches; never modify simulation particles.

PN triangles interpolate physical vertices and share edge control points. Flat
panels remain exactly flat, original box seams and sharp folds remain straight.
UVs and source bevel offsets are retained. Optional smooth thickness transports
the normal offset along the reconstructed surface, avoiding triangle-edge steps.
This is a display approximation, not additional physics DOFs.
"""
import numpy as np
from scipy.sparse import csr_matrix


def unit(x):
    return x / np.maximum(np.linalg.norm(x, axis=-1, keepdims=True), 1e-12)


class PanelSurface:
    def __init__(self, rest, faces, binding_indices, weights, offsets, crease_degrees=45., transition_degrees=10., smooth_thickness=False):
        self.rest = np.asarray(rest, float)
        self.faces = np.asarray(faces, int)
        # Source graphics are face-corner indexed. Evaluate identical bindings once
        # and expand at the end, preserving the original topology and UV seams.
        key=np.concatenate([np.asarray(binding_indices),np.asarray(weights),np.asarray(offsets)],axis=1)
        _,unique,self.output_indices=np.unique(key,axis=0,return_index=True,return_inverse=True)
        binding_indices=np.asarray(binding_indices)[unique]
        self.weights = np.asarray(weights, float)[unique]
        self.offsets = np.asarray(offsets, float)[unique]
        self.smooth_thickness = smooth_thickness
        self.cos_crease = np.cos(np.deg2rad(crease_degrees))
        self.cos_smooth = np.cos(np.deg2rad(crease_degrees-transition_degrees/2))
        self.cos_sharp = np.cos(np.deg2rad(crease_degrees+transition_degrees/2))
        lookup = {tuple(f): i for i, f in enumerate(self.faces)}
        self.binding = np.array([lookup[tuple(f)] for f in binding_indices])
        tri = self.rest[self.faces]
        normal = unit(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]))
        # Each intact material panel has its own normal at the original box seam.
        panel = np.argmax(abs(normal), axis=1)*2 + (normal.sum(axis=1)>0)
        self.groups = self.faces*6 + panel[:, None]
        self.group_count = len(rest)*6
        adjacency = [[] for _ in range(self.group_count)]
        for face, groups in enumerate(self.groups):
            for group in groups: adjacency[group].append(face)
        max_neighbors = max(map(len, adjacency))
        self.neighbors = np.full((len(faces), 3, max_neighbors), -1, int)
        for face, groups in enumerate(self.groups):
            for corner, group in enumerate(groups):
                self.neighbors[face, corner, :len(adjacency[group])] = adjacency[group]
        self.edge_faces = {}
        for face, ids in enumerate(self.faces):
            for a, b in [(0, 1), (1, 2), (2, 0)]:
                self.edge_faces.setdefault(tuple(sorted((ids[a], ids[b]))), []).append(face)
        self.edges = np.array(list(self.edge_faces))
        self.edge_adj = np.array([v if len(v)==2 else [v[0],v[0]] for v in self.edge_faces.values()])
        self.seam = panel[self.edge_adj[:, 0]] != panel[self.edge_adj[:, 1]]
        edge_lookup = {tuple(e): i for i, e in enumerate(self.edges)}
        self.face_edges = np.array([[edge_lookup[tuple(sorted((f[a], f[b])))] for a,b in [(0,1),(1,2),(2,0)]] for f in self.faces])
        frame = np.stack([tri[:,1]-tri[:,0], tri[:,2]-tri[:,0], normal], axis=-1)
        self.offset_coordinates = np.einsum('nij,nj->ni', np.linalg.inv(frame)[self.binding], self.offsets)
        u, v, w = self.weights.T
        self.bernstein = np.stack([u**3,v**3,w**3,3*u*u*v,3*u*v*v,3*v*v*w,3*v*w*w,3*w*w*u,3*w*u*u,6*u*v*w],axis=1)
        # Fixed interpolation operators avoid gathering ten 3-vectors per
        # graphics binding on every frame. Keep float64 math and float32 output.
        rows=np.arange(len(self.binding))[:,None]
        self.patch_operator=csr_matrix((self.bernstein.ravel(),
            (np.broadcast_to(rows,self.bernstein.shape).ravel(),
             (self.binding[:,None]*10+np.arange(10)).ravel())),shape=(len(rows),len(self.faces)*10))
        self.corner_operator=csr_matrix((self.weights.ravel(),
            (np.broadcast_to(rows,self.weights.shape).ravel(),
             (self.binding[:,None]*3+np.arange(3)).ravel())),shape=(len(rows),len(self.faces)*3))
        self.patch_operator.eliminate_zeros()
        self.corner_operator.eliminate_zeros()
        self.edge_slots=[]
        counts=np.zeros(len(self.edges)*2)
        for side,(a,b) in enumerate([(0,1),(1,2),(2,0)]):
            reverse=(self.faces[:,a]>self.faces[:,b]).astype(int)
            slots=(self.face_edges[:,side]*2+reverse,self.face_edges[:,side]*2+1-reverse)
            self.edge_slots.append(slots)
            for slot in slots:counts+=np.bincount(slot,minlength=len(counts))
        self.edge_counts=np.maximum(counts,1)[:,None]
        self.group_flat=self.groups.T.ravel()

    def smooth_weight(self, cosine):
        x=np.clip((cosine-self.cos_sharp)/max(self.cos_smooth-self.cos_sharp,1e-9),0,1)
        return x*x*(3-2*x)

    def evaluate(self, points):
        points = np.asarray(points, float)
        tri = points[self.faces]
        area_normal = np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
        normal = unit(area_normal)
        candidates = normal[np.maximum(self.neighbors, 0)]
        mask = (self.neighbors>=0) * self.smooth_weight((candidates*normal[:,None,None,:]).sum(-1))
        corner = unit((area_normal[np.maximum(self.neighbors,0)]*mask[...,None]).sum(axis=2))
        sharp = 1-self.smooth_weight((normal[self.edge_adj[:,0]]*normal[self.edge_adj[:,1]]).sum(-1))
        sharp[self.seam]=1.
        # Average directed-edge controls between the two incident faces: exactly
        # the same boundary curve on both sides, including around sharp folds.
        sums = np.zeros((len(self.edges)*2,3))
        local_controls = []
        for side,(a,b) in enumerate([(0,1),(1,2),(2,0)]):
            delta=tri[:,b]-tri[:,a]
            ca=(2*tri[:,a]+tri[:,b]-(delta*corner[:,a]).sum(-1)[:,None]*corner[:,a])/3
            cb=(2*tri[:,b]+tri[:,a]+(delta*corner[:,b]).sum(-1)[:,None]*corner[:,b])/3
            for slot,values in zip(self.edge_slots[side],(ca,cb)):
                for axis in range(3):sums[:,axis]+=np.bincount(slot,weights=values[:,axis],minlength=len(sums))
        ec=(sums/self.edge_counts).reshape(-1,2,3)
        straight=np.stack([(2*points[self.edges[:,0]]+points[self.edges[:,1]])/3,(points[self.edges[:,0]]+2*points[self.edges[:,1]])/3],axis=1)
        ec=ec*(1-sharp[:,None,None])+straight*sharp[:,None,None]
        for side,(a,b) in enumerate([(0,1),(1,2),(2,0)]):
            edge=self.face_edges[:,side];reverse=(self.faces[:,a]>self.faces[:,b]).astype(int)
            local_controls.extend([ec[edge,reverse],ec[edge,1-reverse]])
        e=np.mean(local_controls,axis=0);center=e+(e-tri.mean(axis=1))/2
        controls=np.stack([tri[:,0],tri[:,1],tri[:,2],*local_controls,center],axis=1)
        result=self.patch_operator @ controls.reshape(-1,3)
        normals=unit(self.corner_operator @ corner.reshape(-1,3))
        result+=(tri[:,1]-tri[:,0])[self.binding]*self.offset_coordinates[:,0,None]
        result+=(tri[:,2]-tri[:,0])[self.binding]*self.offset_coordinates[:,1,None]
        if self.smooth_thickness:
            # A thick skin must follow the reconstructed normal too. Using a
            # constant face normal here creates steps at coarse triangle edges,
            # even when the midsurface itself is smoothly interpolated.
            # Thickness directions must also agree across a deep crease: split
            # shading normals are valid there, but split positions create a
            # sawtooth skin when render triangles cross the coarse edge.
            sums=np.stack([np.bincount(self.group_flat,weights=np.tile(area_normal[:,axis],3),
                                     minlength=self.group_count) for axis in range(3)],axis=1)
            face_area=np.linalg.norm(area_normal,axis=1)
            areas=np.bincount(self.group_flat,weights=np.tile(face_area,3),minlength=self.group_count)
            # Average offset vectors, not unit directions: renormalizing the
            # average inflates the outside of a tightly folded skin.
            thickness_corner=(sums/np.maximum(areas[:,None],1e-12))[self.groups]
            direction=self.corner_operator @ thickness_corner.reshape(-1,3)
        else:
            direction=normal[self.binding]
        result+=direction*self.offset_coordinates[:,2,None]
        return result.astype(np.float32)[self.output_indices],normals.astype(np.float32)[self.output_indices]
