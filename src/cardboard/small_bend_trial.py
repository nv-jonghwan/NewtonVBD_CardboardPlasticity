"""Isolated small-strain reinforcement trial; not a calibrated cardboard model."""
import math
from importlib.metadata import version
import warp as wp
from .scenario import PickCrushDrop
from .small_bend_kernels import release_small_bend_energy,release_memory_energy,record_crease_dissipation


def validate_small_bend(scale,knee,end,yield_curvature):
    if not all(math.isfinite(x) for x in (scale,knee,end,yield_curvature)):
        raise ValueError('Small-bend parameters must be finite')
    if not (scale>=1 and knee>0 and scale*knee<end<yield_curvature):
        raise ValueError('Require scale>=1 and 0<scale*knee<end<original yield curvature')


class SmallBendPickCrushDrop(PickCrushDrop):
    def __init__(self,*args,small_bend_options,**kwargs):
        if version('newton') != '1.6.0':
            raise ValueError('Small-bend trial kernels are pinned to Newton 1.6.0')
        super().__init__(*args,**kwargs)
        scale,knee,end=(float(small_bend_options[k]) for k in ('scale','knee','end'))
        validate_small_bend(scale,knee,end,float(self.attr(self.mat,'yieldCurvature')))
        from .compact_contact import CompactContactMixin
        if not self.newton16 or not self.crease_damage_length or not isinstance(self.solver,CompactContactMixin):
            raise ValueError('Small-bend trial requires Newton1.6 project-local creased-shell solver')
        self.solver.small_bend_config=(scale,knee,end)
        self.solver.small_bend_dual=self.dual
        memory=float(small_bend_options.get('memory_curvature',0.))
        if not math.isfinite(memory) or memory<0:
            raise ValueError('Memory curvature must be finite and nonnegative')
        self.solver.small_bend_memory_curvature=memory
        friction=float(small_bend_options.get('crease_friction_curvature',0.))
        if not math.isfinite(friction) or friction<0:
            raise ValueError('Crease friction curvature must be finite and nonnegative')
        self.solver.crease_friction_curvature=friction
        self.crease_friction_work=wp.zeros(self.model.edge_count,dtype=float,device=self.model.device)
        self.small_bend_start_alpha=wp.empty_like(self.a.cardboard.accumulated_angle)
        self.small_bend_start_damage=wp.empty_like(self.a.cardboard.damage)

    def record_crease_dissipation(self):
        if self.solver.crease_friction_curvature>0:
            m=self.model
            wp.launch(record_crease_dissipation,dim=m.edge_count,
                      inputs=[self.solver.particle_q_prev,self.b.particle_q,m.edge_indices,
                              m.edge_rest_length,self.dual,self.a.cardboard.accumulated_angle,
                              m.edge_bending_properties,self.solver.crease_friction_curvature,self.dt],
                      outputs=[self.crease_friction_work],device=m.device)

    def _integrate(self):
        # Snapshot once per frame. Work is diagnostic history, not an input to
        # forces/damage, so accounting the one-time release at frame end is exact
        # for the recorded ledger and avoids extra per-substep launches.
        wp.copy(self.small_bend_start_alpha,self.a.cardboard.accumulated_angle)
        memory=self.solver.small_bend_memory_curvature
        if memory>0:wp.copy(self.small_bend_start_damage,self.a.cardboard.damage)
        super()._integrate()
        if memory>0:
            wp.launch(release_memory_energy,dim=self.model.edge_count,
                      inputs=[self.small_bend_start_alpha,self.a.cardboard.accumulated_angle,
                              self.small_bend_start_damage,self.a.cardboard.damage,
                              self.ke_gpu,self.model.edge_rest_length,self.dual,
                              *self.solver.small_bend_config,memory],
                      outputs=[self.a.cardboard.plastic_work],device=self.model.device)
            return
        wp.launch(release_small_bend_energy,dim=self.model.edge_count,
                  inputs=[self.small_bend_start_alpha,self.a.cardboard.accumulated_angle,
                          self.ke_gpu,self.model.edge_rest_length,self.dual,
                          *self.solver.small_bend_config],
                  outputs=[self.a.cardboard.plastic_work],device=self.model.device)
