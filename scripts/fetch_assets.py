"""Fetch NVIDIA-owned source assets verbatim and record hashes (no credentials)."""
import concurrent.futures, hashlib, json, urllib.request
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/'
GROUPS = {
 'cardbox_a1': ('simready_content/common_assets/props/cardbox_a1/', ['cardbox_a1.usd','cardbox_a1_base.usd','cardbox_a1_inst.usd','cardbox_a1_inst_base.usd','cardbox_a1.json','Textures/T_Cardbox_A1_Albedo.png','Textures/T_Cardbox_A1_Normal.png','Textures/T_Cardbox_A1_ORM.png']),
 'ur10': ('Isaac/5.0/Isaac/Robots/UniversalRobots/ur10/', ['ur10.usd','configuration/ur10_base.usd','configuration/ur10_physics.usd','configuration/ur10_robot_schema.usd','configuration/ur10_sensor.usd'])}
def fetch(job):
 group,prefix,name=job; dst=ROOT/'assets/source'/group/name; dst.parent.mkdir(parents=True,exist_ok=True); url=BASE+prefix+name
 if not dst.exists():
  with urllib.request.urlopen(url, timeout=120) as r: data=r.read()
  dst.write_bytes(data)
 data=dst.read_bytes()
 return {'path':str(dst.relative_to(ROOT)), 'url':url, 'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
if __name__=='__main__':
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as ex:
  records=list(ex.map(fetch,[(g,p,n) for g,(p,ns) in GROUPS.items() for n in ns]))
 (ROOT/'assets/source/manifest.json').write_text(json.dumps({'retrieved':'2026-09-30','license_note':'NVIDIA source assets retain original license/terms; project code does not relicense them.','files':records},indent=2))
 print('Fetched',len(records),'files;',sum(r['bytes'] for r in records),'bytes')
