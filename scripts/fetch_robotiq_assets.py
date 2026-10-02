"""Fetch immutable NVIDIA Robotiq source dependencies and record their hashes."""
import json,urllib.request,urllib.parse,hashlib,posixpath
from pathlib import Path
from pxr import Sdf
root=Path(__file__).resolve().parents[1]/'assets/source/robotiq_2f140'
base='https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Robotiq/2F-140/'
queue=['Robotiq_2F_140_physics_edit.usd'];done={}
while queue:
 name=queue.pop(0)
 if name in done:continue
 if name.startswith('../') or '://' in name:raise RuntimeError(name)
 p=root/name;p.parent.mkdir(parents=True,exist_ok=True)
 url=base+urllib.parse.quote(name)
 if not p.exists():
  with urllib.request.urlopen(url,timeout=30) as r:p.write_bytes(r.read())
 done[name]={'path':str(p),'url':url,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
 print(name,p.stat().st_size,flush=True)
 if p.suffix in ('.usd','.usda','.usdc'):
  l=Sdf.Layer.FindOrOpen(str(p));refs=set(l.GetExternalAssetDependencies()) | set(l.GetExternalReferences())
  for ref in refs:
   if ref and not ref.startswith('omniverse:'):
    queue.append(posixpath.normpath(posixpath.join(posixpath.dirname(name),ref)))
(root/'manifest.json').write_text(json.dumps({'retrieved':'2026-09-30','license_note':'NVIDIA/Robotiq source terms retained; derived scaled gripper is hypothetical.','files':list(done.values())},indent=2))
