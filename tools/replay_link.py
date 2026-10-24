"""Record every explicit replay object/archive/DSO and reject concurrent changes."""
import hashlib
from pathlib import Path
import subprocess


def link_inputs(command, directory):
    directory = Path(directory).resolve(strict=True)
    paths = [Path(arg) if Path(arg).is_absolute() else directory / arg
             for arg in command if not arg.startswith('-') and
             (arg.endswith(('.o', '.a', '.so')) or '.so.' in Path(arg).name)]
    return {str(path.resolve(strict=True)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def link_replay(command, directory):
    before = link_inputs(command, directory)
    subprocess.run(command, cwd=directory, check=True)
    if link_inputs(command, directory) != before:
        raise RuntimeError('replay link inputs changed while linking')
    return {'command': command, 'directory': str(Path(directory).resolve()),
            'input_sha256': before}
