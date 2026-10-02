"""Build a local review bundle with hashes; excludes environments and run outputs."""
import argparse
import json
import zipfile

from cardboard import ROOT
from cardboard.release import sha256, verify_inputs


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--output', default='deliverables/cardboard-production-candidate-rc1.zip')
    args = parser.parse_args()
    lock = verify_inputs()
    files = {ROOT / entry['path'] for entry in lock['files']}
    for directory in ('src', 'scripts', 'tests', 'schemas', 'exts', 'docs', 'config', 'third_party'):
        files.update(p for p in (ROOT / directory).rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.suffix != '.pyc')
    # Keep historical generated scenes needed by the source regression suite.
    files.update(p for p in (ROOT / 'assets').glob('*') if p.is_file())
    files.update(ROOT.glob('requirements*lock.txt'))
    files.update([ROOT / 'README.md', ROOT / 'README_KR.md', ROOT / '.gitignore'])
    manifest = {'schema_version': 1, 'profile': lock['profile'], 'production_qualified': False,
                'scope': 'Local review/source bundle. No Python/Kit environments, caches or experimental outputs.',
                'files': [{'path': p.relative_to(ROOT).as_posix(), 'sha256': sha256(p),
                           'bytes': p.stat().st_size} for p in sorted(files)]}
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation avoids replacing an artifact already under review.
    with output.open('xb') as stream:
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for p in sorted(files):
                archive.write(p, 'plastic-deformation/' + p.relative_to(ROOT).as_posix())
            archive.writestr('plastic-deformation/RELEASE_MANIFEST.json', json.dumps(manifest, indent=2) + '\n')
    digest = sha256(output)
    output.with_suffix(output.suffix + '.sha256').write_text(f'{digest}  {output.name}\n')
    print(json.dumps({'archive': str(output), 'sha256': digest, 'files': len(files),
                      'bytes': output.stat().st_size}, indent=2))


if __name__ == '__main__':
    main()
