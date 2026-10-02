"""Kit extension UI. Physics is provided by the isolated project bridge."""
import asyncio
import omni.kit.app
import omni.ext
import omni.ui as ui

_instance=None

def connect(controller):
    if _instance is None:raise RuntimeError('Scenario extension is not enabled')
    _instance.controller=controller
    return _instance

class Extension(omni.ext.IExt):
    def on_startup(self,ext_id):
        global _instance
        _instance=self;self.controller=None
        self.window=ui.Window('Cardboard Scenario',width=370,height=575)
        with self.window.frame:
            with ui.VStack(spacing=10,style={'font_size':17},padding=12):
                ui.Label('UR10 | Grasp - Crush - Drop',height=28)
                self.material=ui.Label('Loading material...',height=25)
                self.gripper=ui.Label('',height=22)
                ui.Label('1  Home\n2  Approach the box\n3  Grasp and lift\n4  Squeeze and crumple\n5  Release onto the table',height=115)
                with ui.HStack(height=34,spacing=6):
                    ui.Button('Play / Resume',clicked_fn=lambda:self.action('play'))
                    ui.Button('Pause',clicked_fn=lambda:self.action('pause'))
                ui.Button('Reset to Home',height=32,clicked_fn=lambda:self.action('reset'))
                ui.Button('Recorded replay',height=30,clicked_fn=lambda:self.action('replay'))
                with ui.HStack(height=30,spacing=6):
                    ui.Button('Box detail',clicked_fn=lambda:self.action('view_box'))
                    ui.Button('Overview',clicked_fn=lambda:self.action('view_scene'))
                self.phase=ui.Label('Loading Newton...',height=25)
                self.progress=ui.ProgressBar(height=16)
                self.metrics=ui.Label('',height=102,word_wrap=True)
                ui.Label('Use these controls to run the scenario.\nReset restores the undeformed box.',height=40,word_wrap=True)
        async def dock():
            for _ in range(8):await omni.kit.app.get_app().next_update_async()
            stage=ui.Workspace.get_window('Stage')
            if stage:self.window.dock_in(stage,ui.DockPosition.SAME)
            prop=ui.Workspace.get_window('Property')
            if prop:prop.visible=False
            self.window.focus()
        self.dock_task=asyncio.ensure_future(dock())
    def action(self,action):
        if self.controller:self.controller(action)
    def update(self,status,message=None):
        names=['Home','Move above box','Lower to box','Grasp','Lift','Squeeze / crumple','Release / drop','Settle on table']
        if message:self.phase.text=message
        else:self.phase.text=('Recorded | ' if status.get('recorded') else '')+('Finished' if status['done'] else ('Running' if status['playing'] else 'Paused'))+' | '+names[status['phase']]
        duration=status.get('duration',16);t=status.get('t',0)
        thickness=status.get('thickness_mm',0);engine=status.get('engine_version','')
        self.gripper.text=status.get('gripper_label','')
        if thickness:self.material.text=f"Cardboard {thickness:.1f} mm | Newton {engine}"
        self.progress.model.set_value(t/duration)
        self.metrics.text=(f"Time {t:.2f} / {duration:.1f} s   |   Gap {status.get('actual_gap_m',.32)*1000:.0f} mm\n"
            f"Target {status.get('command_gap_m',.32)*1000:.0f} mm | {status.get('drive_label','Motor limit')} {status.get('motor_limit_N',0):.0f} N\n"
            f"Finger contacts {status.get('left_contact_N',0):.1f} / {status.get('right_contact_N',0):.1f} N\n"
            f"Plastic hinges {status.get('plastic_hinges',0)}" + (f" | Live {status['realtime_factor']:.2f}x" if status.get('realtime_factor') and not status.get('recorded') else ''))
    def on_shutdown(self):
        global _instance
        self.dock_task.cancel();self.controller=None;self.window=None;_instance=None
