"""Fetch only locked external assets, verifying hashes before atomic installation."""
import argparse
import hashlib
import json
import os
import sys
import tempfile
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from cardboard.release import LOCK_FILE, local_path, sha256, verify_inputs


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    lock = json.loads((ROOT / LOCK_FILE).read_text())
    # Refuse any modified existing input before starting a download.
    for entry in lock['files']:
        path = local_path(entry['path'])
        if path.exists() and sha256(path) != entry['sha256']:
            raise ValueError(f'Checksum mismatch; preserve and inspect this file: {entry["path"]}')
        if not path.exists() and (args.offline or 'url' not in entry):
            raise ValueError(f'Missing input: {entry["path"]}')
    for entry in lock['files']:
        path = local_path(entry['path'])
        if path.exists():
            continue
        if not entry['url'].startswith('https://omniverse-content-production.s3.us-west-2.amazonaws.com/'):
            raise ValueError('Unexpected external asset origin')
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as out:
                temporary = Path(out.name)
                with urlopen(entry['url'], timeout=60) as response:
                    while chunk := response.read(1024 * 1024):
                        out.write(chunk)
            if temporary.stat().st_size != entry['bytes'] or sha256(temporary) != entry['sha256']:
                raise ValueError(f'Download checksum mismatch: {entry["path"]}')
            # Never replace an input created by another concurrent process.
            os.link(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        print('Verified', entry['path'])
    verify_inputs()
    print(f'PASS: {len(lock["files"])} locked inputs')


if __name__ == '__main__':
    main()
