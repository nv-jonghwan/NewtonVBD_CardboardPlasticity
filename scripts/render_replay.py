"""Render synchronized overview/detail views from a recorded Newton trajectory."""
import argparse
import hashlib
import io
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def encode_media(out, prefix, fps):
    prefix = Path(prefix); prefix.parent.mkdir(parents=True, exist_ok=True)
    common = ['ffmpeg', '-y', '-loglevel', 'error', '-threads', '4', '-framerate', str(fps), '-i', str(out/'%04d.png')]
    subprocess.run(common+['-c:v', 'libx264', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(prefix)+'.mp4'], check=True)
    subprocess.run(common+['-filter_complex_threads', '1', '-filter_complex',
        f"[0:v]fps={min(fps, 10)},scale=w='min(960,iw)':h=-2:flags=lanczos,split[a][b];"
        "[a]palettegen=max_colors=128:stats_mode=diff[p];"
        "[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
        '-loop', '0', str(prefix)+'.gif'], check=True)
    print('ENCODED', str(prefix)+'.gif', str(prefix)+'.mp4', flush=True)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--record-directory', required=True)
    parser.add_argument('--scene', default='assets/demo_scene_robotiq_board4_vertical.usda')
    parser.add_argument('--output', required=True, help='New directory for PNG frames and metadata')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--width', type=int, default=600, help='Width of each view')
    parser.add_argument('--height', type=int, default=450, help='Height of each view')
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--times', type=float, nargs='+', help='Render preview times only')
    parser.add_argument('--media-prefix', help='Encode PREFIX.gif and PREFIX.mp4 with FFmpeg')
    args = parser.parse_args()
    if min(args.fps, args.width, args.height) <= 0:
        parser.error('FPS and dimensions must be positive')
    if args.media_prefix and args.height % 2:
        parser.error('Use an even --height for H.264 video')
    if args.times and args.media_prefix:
        parser.error('--times is for still previews; omit --media-prefix')
    out = ROOT / args.output
    if out.exists():
        parser.error('Output directory already exists; choose a new --output')
    source = ROOT / args.record_directory / 'trajectory.npz'
    if not source.is_file():
        parser.error('Recorded trajectory.npz is missing')
    out.mkdir(parents=True)

    from isaacsim import SimulationApp
    app = SimulationApp({
        'headless': True, 'width': args.width, 'height': args.height,
        'renderer': 'RayTracedLighting', 'active_gpu': args.gpu,
        'physics_gpu': args.gpu, 'multi_gpu': False,
        'extra_args': ['--/rtx/hydra/readTransformsFromFabricInRenderDelegate=false'],
    })
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from PIL import Image, ImageDraw, ImageFont
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt
        from cardboard.geometry import skin
        from cardboard.surface import PanelSurface
        from cardboard.camera import OVERVIEW, BOX_DETAIL, set_camera_view, NEAR_CLIP_M, FAR_CLIP_M

        source_bytes = source.read_bytes()
        (out/'trajectory.npz').write_bytes(source_bytes)
        with np.load(io.BytesIO(source_bytes), allow_pickle=False) as data:
            times = data['t']; points = data['points']; bodies = data['body_q']
            labels = data['body_labels'].tolist()
        if not (np.all(np.diff(times) > 0) and np.isfinite(points).all() and np.isfinite(bodies).all()):
            raise ValueError('Trajectory contains invalid time or state data')
        # Same sample-and-hold state selection as the GUI Recorded replay.
        samples = np.asarray(args.times) if args.times else np.minimum(
            np.arange(int(np.ceil((times[-1]-times[0])*args.fps-1e-9))+1)/args.fps+times[0], times[-1])
        if samples.min() < times[0]-1e-6 or samples.max() > times[-1]+1e-6:
            raise ValueError('Requested preview times are outside the recording')
        indices = np.clip(np.searchsorted(times, samples+1e-9, side='right')-1, 0, len(times)-1)
        omni.usd.get_context().open_stage(str(ROOT / args.scene))
        for _ in range(20):
            app.update()
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        cache = UsdGeom.XformCache()
        ops = {}
        for label in labels:
            prim = stage.GetPrimAtPath(label)
            if not prim:
                raise ValueError('Recorded body missing from scene: '+label)
            parent = cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse()
            xform = UsdGeom.Xformable(prim); xform.ClearXformOpOrder()
            ops[label] = (xform.AddTransformOp(opSuffix='recorded'), parent)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        mesh = stage.GetPrimAtPath('/World/Box/SimMesh')
        render = UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/RenderMesh'))
        center = np.array(cache.GetLocalToWorldTransform(mesh).ExtractTranslation())
        attr = lambda name: render.GetPrim().GetAttribute('cardboard:'+name).Get()
        binding = np.array(attr('bindingIndices')); weights = np.array(attr('bindingWeights')); offsets = np.array(attr('bindingOffsets'))
        surface = None
        if attr('surfaceInterpolation') == 'creaseAwareCubic':
            surface = PanelSurface(np.array(mesh.GetAttribute('cardboard:restPoints').Get()),
                np.array(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1, 3),
                binding, weights, offsets, attr('visualCreaseAngleDegrees'),
                attr('visualCreaseTransitionDegrees'), bool(attr('visualSmoothThickness')))
            render.SetNormalsInterpolation('vertex')
        UsdLux.DomeLight.Define(stage, '/World/ReplayLight').CreateIntensityAttr(800)
        views = [
            ('Full sequence', *OVERVIEW),
            ('Box close-up', *BOX_DETAIL),
        ]
        annotators = []
        for n, (_, eye, target, focal) in enumerate(views):
            path = '/World/ReplayCamera'+str(n)
            camera = UsdGeom.Camera.Define(stage, path)
            set_camera_view(camera, eye, target, focal)
            product = rep.create.render_product(path, (args.width, args.height))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb'); rgb.attach([product]); annotators.append(rgb)
        font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
        font = ImageFont.truetype(str(font_path), 18) if font_path.exists() else ImageFont.load_default()
        phases = ['Initial pose', 'Approach', 'Vertical descent', 'Grasp', 'Vertical lift', 'Squeeze', 'Release', 'Settle']
        phase_ends = list(stage.GetPrimAtPath('/World/Physics').GetAttribute('cardboard:phaseEnds').Get())
        for n, (sample, index) in enumerate(zip(samples, indices)):
            local = points[index]-center
            display, normals = surface.evaluate(local) if surface is not None else (skin(local, binding, weights, offsets).astype(np.float32), None)
            with Sdf.ChangeBlock():
                render.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(display))
                if normals is not None:
                    render.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
                for label, pose in zip(labels, bodies[index]):
                    op, parent = ops[label]
                    transform = Gf.Matrix4d().SetRotate(Gf.Quatd(float(pose[6]), Gf.Vec3d(*pose[3:6].tolist())))
                    transform.SetTranslateOnly(Gf.Vec3d(*pose[:3].tolist())); op.Set(transform*parent)
            rep.orchestrator.step(rt_subframes=8 if n == 0 else 4, delta_time=0., pause_timeline=True)
            for _ in range(3):
                app.update()
            canvas = Image.new('RGB', (args.width*2, args.height+72), '#101c2c')
            for column, rgb in enumerate(annotators):
                pixels = rgb.get_data()
                if not isinstance(pixels, np.ndarray) or not pixels.size:
                    raise RuntimeError('RGB annotator returned no pixels')
                canvas.paste(Image.fromarray(pixels[:, :, :3]), (column*args.width, 36))
            draw = ImageDraw.Draw(canvas)
            for column, (title, *_rest) in enumerate(views):
                draw.text((column*args.width+16, 8), title, font=font, fill='#ffffff')
            phase = phases[min(int(np.searchsorted(phase_ends, sample, side='right')), 7)]
            draw.text((16, args.height+44), phase, font=font, fill='#ffffff')
            draw.text((args.width*2-280, args.height+44), f'Replay  |  {sample:04.1f} / {times[-1]:.0f} s', font=font, fill='#b6cee7')
            canvas.save(out/f'{n:04d}.png')
            if n % 30 == 0 or n == len(samples)-1:
                print(f'RENDER {n+1}/{len(samples)} t={sample:.2f}', flush=True)
        metadata = {'scene': args.scene, 'source': str(source.relative_to(ROOT)),
            'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
            'source_snapshot': str((out/'trajectory.npz').relative_to(ROOT)),
            'fps': args.fps, 'frame_count': len(samples), 'size': [args.width*2, args.height+72],
            'simulation_duration_s': float(times[-1]), 'physics_recomputed': False, 'clipping_range_m': [NEAR_CLIP_M, FAR_CLIP_M],
            'views': [dict(label=v[0], eye=v[1], target=v[2], focal_length=v[3]) for v in views]}
        (out/'render.json').write_text(json.dumps(metadata, indent=2)+'\n')
        if args.media_prefix:
            # Kit fast shutdown exits Python; finish encoding before app.close().
            encode_media(out, ROOT / args.media_prefix, args.fps)
    finally:
        app.close()


if __name__ == '__main__':
    main()
