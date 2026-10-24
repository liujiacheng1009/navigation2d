"""Cache replay compilation by complete preprocessed input and codegen flags.

Linking is always repeated by the caller. Sources/dependencies must remain
stable during a build, as for the parent CMake target. No project files change.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compiler_identity(compiler):
    driver = Path(shutil.which(compiler) or compiler).resolve(strict=True)
    identity = {str(driver): digest(driver)}
    for component in ['cc1plus', 'as']:
        name = subprocess.check_output([compiler, f'-print-prog-name={component}'], text=True).strip()
        path = Path(shutil.which(name) or name)
        if path.is_file():
            identity[str(path.resolve())] = digest(path)
    return identity


def compile_cached(compiler, flags, source, obj, force=False):
    source, obj = Path(source), Path(obj)
    preprocessed = obj.with_suffix('.ii')
    metadata = obj.with_suffix('.compile.json')
    # Preprocessing on every invocation also catches new include shadows and
    # system-header changes, unlike checking only a previous dependency list.
    subprocess.run([compiler, *flags, '-E', str(source), '-o', str(preprocessed)], check=True)
    inputs = {'preprocessed_sha256': digest(preprocessed), 'flags': flags,
              'compiler': compiler_identity(compiler), 'cwd': os.getcwd(),
              'cache_helper_sha256': digest(__file__)}
    try:
        cached = json.loads(metadata.read_text())
        hit = (not force and cached['inputs'] == inputs and obj.is_file()
               and cached['object_sha256'] == digest(obj))
    except (OSError, ValueError, KeyError):
        hit = False
    if not hit:
        temporary = obj.with_suffix('.o.tmp')
        # Compile the original input with the exact target flags. This retains
        # source-location/file symbols and the uncached builder's object bytes.
        subprocess.run([compiler, *flags, '-c', str(source), '-o', str(temporary)], check=True)
        temporary.replace(obj)
        cached = {'inputs': inputs, 'object_sha256': digest(obj)}
        metadata.write_text(json.dumps(cached, indent=2) + '\n')
    return {'cache_hit': hit, 'object_sha256': cached['object_sha256'],
            'preprocessed_sha256': inputs['preprocessed_sha256']}
