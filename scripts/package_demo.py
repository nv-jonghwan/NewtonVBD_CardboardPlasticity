"""Build an auditable local deliverable; no upload/publication."""
from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1];dest=ROOT/'deliverables';dest.mkdir(exist_ok=True)
files=[]
for directory in ['src','scripts','schemas','assets','docs','tests','exts']:
    files.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
files.extend(ROOT/n for n in ['README.md','requirements-lock.txt','requirements-newton15-lock.txt','requirements-newton16-lock.txt'])
for name in ['validation_summary.json','asset_validation.json','checkpoint_validation.json','unit-tests.log','physics_evidence.png','physics_evidence.pdf']:
    files.append(ROOT/'outputs'/name)
for folder,names in {'plastic':['cardboard_crush_demo.mp4','replay.usdc','deformed_checkpoint.usda','metrics.csv','report.json','trajectory.npz'],'elastic':['metrics.csv','report.json','trajectory.npz','deformed_checkpoint.usda'],'convergence32':['metrics.csv','report.json']}.items():
    files.extend(ROOT/'outputs'/folder/n for n in names)
files.extend(ROOT/'outputs/scenario_validation_final'/n for n in ['state.csv','state.npz','report.json'])
files.extend((ROOT/'outputs/scenario_stronger').glob('*.npz'))
files.extend(ROOT/'outputs/scenario_stronger'/n for n in ['state.csv','report.json','shape_comparison.png'])
files.extend((ROOT/'outputs/scenario_stiffer').glob('*.npz'))
files.extend(ROOT/'outputs/scenario_stiffer'/n for n in ['state.csv','report.json','parameters.json'])
files.extend(ROOT/'outputs/panel_dense_final'/n for n in ['state.csv','state.npz','trajectory.npz','panel_metrics.json','report.json','comparison.json','parameters.json','run_config.json','gui_crush.png'])
files.extend(ROOT/'outputs/panel_baseline'/n for n in ['state.csv','state.npz','trajectory.npz','panel_metrics.json','parameters.json'])
files.extend(ROOT/'outputs'/n for n in ['dense_asset_validation.json','dense_gui_test.log'])
files.extend(ROOT/'outputs/plate_crease'/n for n in ['state.csv','state.npz','trajectory.npz','panel_metrics.json','report.json','parameters.json','run_config.json','shape_retention.json','panel_flutter.json','plate_behavior_validation.json','gui_crush.png','gui_settled.png','behavior_comparison.png'])
files.extend(ROOT/'outputs'/n for n in ['plate_crease_asset_validation.json','plate_crease_gui_test.log','plate_unit_tests.log'])
files.extend(ROOT/'outputs/thick16_stronger'/n for n in ['state.csv','state.npz','trajectory.npz','panel_metrics.json','report.json','parameters.json','run_config.json','shape_retention.json','panel_flutter.json','plate_behavior_validation.json','thick_behavior_validation.json','stronger_comparison.json','gui_crush.png','gui_settled.png'])
files.extend(ROOT/'outputs/thick16_final'/n for n in ['panel_metrics.json','report.json','parameters.json','run_config.json','shape_retention.json','panel_flutter.json','thick_behavior_validation.json'])
files.extend(ROOT/'outputs/thick16_100mm'/n for n in ['panel_metrics.json','report.json','parameters.json','run_config.json','plate_behavior_validation.json','thick_behavior_validation.json','stronger_comparison.json'])
files.extend(ROOT/'outputs/thick16_82mm'/n for n in ['panel_metrics.json','report.json','parameters.json','run_config.json','plate_behavior_validation.json','thick_behavior_validation.json','stronger_comparison.json'])
files.extend(ROOT/'outputs/thick16_ringdown'/n for n in ['ringdown.json','ringdown.png'])
files.extend(ROOT/'outputs'/n for n in ['benchmark_newton15.json','benchmark_newton16.json','newton16_performance.json','newton16_unit_tests.log','newton16_source_integrity.json','thick16_stronger_gui_test.log','thick16_stronger_asset_validation.json'])
for folder in ['realtime','board15_kd85','board15_half']:
    files.extend(ROOT/'outputs'/folder/n for n in [
        'state.csv','state.npz','trajectory.npz','parameters.json','run_config.json',
        'panel_metrics.json','shape_retention.json','panel_flutter.json','display_flatness.json',
        'report.json','live_performance.json','gui_crush.png','gui_settled.png'])
files.append(ROOT/'outputs/board15_half/live_sequence_validation.json')
files.extend(ROOT/'outputs'/n for n in ['board15_unit_tests.log','board15_half_asset_validation.json',
    'board15_half_gui_test.log','realtime_validation.json','realtime_asset_validation.json',
    'realtime_unit_tests.log','realtime_display_equivalence.json','gui_error_reporting_test.log'])
files=sorted(set(files))
manifest={'format':1,'qualification':'Research demo; custom Newton driver required; not calibrated or NVIDIA-certified','files':[{'path':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in files]}
(dest/'manifest.json').write_text(json.dumps(manifest,indent=2))
path=dest/'newton_cardboard_ur10_demo.zip'
with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as z:
    for p in files:z.write(p,'newton_cardboard_demo/'+str(p.relative_to(ROOT)))
    z.write(dest/'manifest.json','newton_cardboard_demo/package_manifest.json')
with zipfile.ZipFile(path) as z:assert z.testzip() is None
print(path,len(files),'files',path.stat().st_size,'bytes')
