`src/cardboard/compact_contact.py` adapts the particle iteration and contact
scheduling from Newton 1.6.0 `solver_vbd.py` and `particle_vbd_kernels.py`.
The Newton Developers' copyright and Apache-2.0 license are retained in the
source header and [newton-LICENSE.md](newton-LICENSE.md).

The project changes contact-list traversal and GPU work scheduling. It imports
the original contact energy and division-plane functions from the pinned runtime.
The installed Newton package itself is unchanged.

`src/cardboard/small_bend_kernels.py` also adapts Newton 1.6.0 bending kernels; its original copyright and Apache-2.0 notice are retained. The crease resistance and small-bend reinforcement are project extensions.

NVIDIA Cardbox, UR10 and Robotiq assets retain their original licensing terms. Source URLs and SHA-256 values for the selected release are in `config/release-inputs.lock.json`. Inclusion in a local review archive does not grant redistribution rights. This repository currently has no top-level project-code license declaration; external release requires the owner to determine licensing and review asset redistribution terms.
