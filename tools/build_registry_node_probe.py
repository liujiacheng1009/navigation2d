#!/usr/bin/env python3
"""Bind the actual node's selected registry to real planner/controller calls.

Uses the same captured, endpoint-free source seam as the frozen replay; the
separate fixture/binary never changes that replay's performance workload.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
from build_explorer_replays import adapt
from replay_startup import without_endpoints
from replay_object_cache import compile_cached
from replay_link import link_replay, replace_node_entry_object, node_compile_directory


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sanitize', action='store_true',
                        help='ASan/UBSan node/test TU; linked ROS/core dependencies stay native')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = args.build.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    production = root / 'ros/autonomous_explorer_node.cc'
    fixture = root / 'test/explorer_registry_scenario.h'
    inputs = [production, fixture, Path(__file__), *[Path(__file__).with_name(name) for name in
              ['build_explorer_replays.py', 'replay_startup.py', 'replay_link.py']]]
    hashes = {str(path): digest(path) for path in inputs}
    source = adapt(without_endpoints(production.read_text()), 'registry')
    entry = 'int main(int argc, char** argv)'
    if source.count(entry) != 1:
        raise ValueError('unexpected generated replay entry')
    source = source.replace(entry, 'int UnusedExplorerReplayMain(int argc, char** argv)', 1)
    source += '''
#include "navigation2d/test/explorer_registry_scenario.h"
int main(int argc, char** argv) {
  return navigation2d::test::RunExplorerRegistryProbe<AutonomousExplorer>(argc, argv);
}
'''
    target = build / 'CMakeFiles/sweepnav_autonomous_explorer.dir'
    compile_target = node_compile_directory(build, production.read_text())
    flags = []
    for line in (compile_target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    flags.append('-UNDEBUG')
    link = shlex.split((target / 'link.txt').read_text())
    if args.sanitize:
        flags.extend(['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'])
        link.insert(1, '-fsanitize=address,undefined')
    cpp, obj, binary = [output / ('registry' + suffix) for suffix in ['.cc', '.o', '']]
    cpp.write_text(source)
    compilation = compile_cached(link[0], flags, cpp, obj, False)
    link = replace_node_entry_object(link, obj)
    link[link.index('-o') + 1] = str(binary)
    linkage = link_replay(link, build)
    if hashes != {str(path): digest(path) for path in inputs}:
        raise RuntimeError('registry probe inputs changed while building')
    (output / 'manifest.json').write_text(json.dumps({
        'production_sha256': digest(production), 'fixture_sha256': digest(fixture),
        'builder_sha256': digest(Path(__file__)), 'binary_sha256': digest(binary),
        'sanitize_node_test_tu_only': args.sanitize, 'input_sha256': hashes,
        'compilation': compilation,
        'linkage': linkage}, indent=2) + '\n')


if __name__ == '__main__':
    main()
