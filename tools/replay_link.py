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


def replace_node_entry_object(command, replacement):
    """Bind one registered production entry; never retain an unadapted main."""
    names = ('autonomous_explorer_node.cc.o', 'autonomous_explorer_main.cc.o')
    matches = [index for index, arg in enumerate(command)
               if Path(arg).name in names and not arg.startswith('-')]
    if len(matches) != 1:
        raise ValueError('expected one registered explorer entry object')
    result = list(command)
    result[matches[0]] = str(replacement)
    return result


def node_entry_form(source):
    """Recognize only registered production layouts, never a generic no-main file."""
    mains = source.count('int main(int argc, char** argv)')
    factory = ('#include "navigation2d/ros/exploration/explorer_node.h"' in source and
               source.count('MakeAutonomousExplorer(') == 1)
    if mains == 1:
        return 'legacy'
    if mains == 0 and factory:
        return 'factory'
    raise ValueError('unsupported node main/factory; update the explicit replay adapter')


def node_compile_directory(build, source):
    """Compile the copied node with its producer flags, not just CLI main flags."""
    name = ('navigation2d_explorer_ros' if node_entry_form(source) == 'factory'
            else 'sweepnav_autonomous_explorer')
    target = Path(build) / 'CMakeFiles' / (name + '.dir')
    if not (target / 'flags.make').is_file():
        raise FileNotFoundError('production node compile flags are missing: ' + str(target))
    return target
