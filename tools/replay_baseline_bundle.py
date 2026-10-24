"""Reuse an independently frozen replay without relinking candidate adapters."""
import hashlib
import json
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reuse_baseline(bundle, output, expected):
    bundle, output = Path(bundle).resolve(), Path(output).resolve()
    if bundle == output or bundle in output.parents or output in bundle.parents:
        raise ValueError('baseline bundle must be separate from generated outputs')
    manifest_path = bundle / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    for key in ['baseline_ref', 'baseline_core', 'baseline_source', 'fixture_sha256']:
        if manifest.get(key) != expected[key]:
            raise ValueError('frozen baseline input differs: ' + key)
    artifact = manifest['artifacts']['baseline']
    for name, key in [('baseline', 'binary'), ('baseline.cc', 'source')]:
        if digest(bundle / name) != artifact[key]:
            raise ValueError('frozen baseline artifact changed: ' + name)
    # Validate every input before replacing any output. The bundle is never
    # written, and its generated source stays available for review.
    for name in ['baseline', 'baseline.cc']:
        shutil.copy2(bundle / name, output / name)
    return artifact, {'path': str(bundle), 'manifest_sha256': digest(manifest_path)}
