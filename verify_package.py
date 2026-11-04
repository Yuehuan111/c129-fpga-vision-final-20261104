"""Verify the complete delivery ZIP with standard Python 3 (no dependencies)."""
from pathlib import Path
from zipfile import ZipFile
import hashlib,json,sys
if len(sys.argv) != 2:
    raise SystemExit('Usage: python verify_package.py delivery.zip')
with ZipFile(Path(sys.argv[1])) as z:
    manifests=[n for n in z.namelist() if n.endswith('/交付清单.json')]
    if len(manifests)!=1: raise ValueError('Expected one delivery manifest')
    manifest_name=manifests[0]
    manifest_raw=z.read(manifest_name)
    manifest=json.loads(manifest_raw)
    prefix=manifest['archive_root']+'/'
    expected={prefix+entry['path'] for entry in manifest['files']}
    expected.update([manifest_name,prefix+'SHA256SUMS_交付.txt'])
    if len(z.namelist())!=len(expected) or set(z.namelist())!=expected:
        raise ValueError('Missing, extra, or duplicate archive members')
    hashes={}
    for entry in manifest['files']:
        digest=hashlib.sha256();size=0
        with z.open(prefix+entry['path']) as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):
                digest.update(chunk);size+=len(chunk)
        if size!=entry['size_bytes'] or digest.hexdigest()!=entry['sha256']:
            raise ValueError('Content mismatch: '+entry['path'])
        hashes[entry['path']]=entry['sha256']
    hashes['交付清单.json']=hashlib.sha256(manifest_raw).hexdigest()
    for line in z.read(prefix+'SHA256SUMS_交付.txt').decode('utf-8').splitlines():
        digest,path=line.split('  ',1)
        if hashes.pop(path,None)!=digest: raise ValueError('Checksum list mismatch: '+path)
    if hashes: raise ValueError('Checksum list is incomplete')
print('PASS: all payload files, sizes, SHA256 hashes, ZIP CRCs and manifest verified.')
