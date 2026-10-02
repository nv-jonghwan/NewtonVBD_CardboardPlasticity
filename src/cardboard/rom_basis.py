"""Euclidean POD increment basis; affine corrections never project away state.

Training data provide directions only, never a time-indexed pose or a force.
Runtime contact forces and all plastic histories remain evaluated on the mesh.
"""
import hashlib
import numpy as np


def mesh_digest(rest, faces):
    h = hashlib.sha256()
    h.update(np.asarray(rest, dtype='<f4').tobytes())
    h.update(np.asarray(faces, dtype='<i4').tobytes())
    return h.hexdigest()


def fit_increment_basis(trajectories, rank=8):
    if not trajectories or not 4 <= rank <= 32:
        raise ValueError('Need training trajectories and rank in [4,32]')
    n = trajectories[0].shape[1]
    translations = np.tile(np.eye(3), (n, 1))/np.sqrt(n)
    snapshots = []
    for q in trajectories:
        if q.ndim != 3 or q.shape[1:] != (n,3) or not np.isfinite(q).all():
            raise ValueError('Invalid or mismatched trajectory')
        dq = np.diff(q.astype(np.float64), axis=0)
        dq -= dq.mean(axis=1, keepdims=True)
        snapshots.append(dq.reshape(len(dq), -1).T)
    data = np.concatenate(snapshots, axis=1)
    u, sigma, _ = np.linalg.svd(data, full_matrices=False)
    basis, _ = np.linalg.qr(np.c_[translations, u[:, :rank-3]])
    captured = float(np.sum(sigma[:rank-3]**2)/max(np.sum(sigma**2),1e-30))
    return basis[:, :rank].reshape(n,3,rank).transpose(0,2,1).astype(np.float32), captured
