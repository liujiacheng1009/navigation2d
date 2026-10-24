#!/usr/bin/env python3
"""Build direct completion-phase probes with independent frozen headers/core.

Separate from the default node replay so extending phase coverage does not
silently change that replay's binary or its frozen performance calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
from build_explorer_replays import adapt
from replay_baseline_headers import prepare, snapshot, baseline_link
from replay_object_cache import compile_cached
from replay_startup import without_endpoints


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def completion_source(source, label):
    budget = 'completing_probe_rounds_ < 3' if 'completing_probe_rounds_' in source else 'completion_.CanProbe()'
    source = adapt(without_endpoints(source), label)
    start = source.index(' public:', source.index('class AutonomousExplorer')) + len(' public:')
    source = (source[:start] + '\n  bool ReplayCanCompleteProbe() const { return ' + budget + '; }\n'
              + source[start:])
    token = 'int main(int argc, char** argv)'
    if source.count(token) != 1:
        raise ValueError('unexpected generated replay main')
    source = source.replace(token, 'int ExplorerReplayMain(int argc, char** argv)', 1)
    source += '''
#include "navigation2d/test/completion_replay/scenario.h"
int main(int argc, char** argv) {
  return navigation2d::test::RunCompletionReplay<AutonomousExplorer>(argc, argv);
}
'''
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True, type=Path, help='parent ROS build')
    parser.add_argument('--baseline-source', required=True, type=Path)
    parser.add_argument('--baseline-headers', required=True, type=Path)
    parser.add_argument('--baseline-core', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = args.build.resolve(strict=True)
    core = args.baseline_core.resolve(strict=True)
    if core == (build / 'navigation2d_build/libnavigation2d_core.a').resolve():
        parser.error('baseline must be independently frozen')
    headers = prepare(args.baseline_source, args.baseline_headers)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    target = build / 'CMakeFiles/sweepnav_autonomous_explorer.dir'
    flags = []
    for line in (target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    template = shlex.split((target / 'link.txt').read_text())
    fixture_paths = [root / 'test/completion_replay/scenario.h', *(root / 'test/replay').glob('*.h')]
    fixtures = {str(p.relative_to(root)): digest(p) for p in fixture_paths}
    manifest = {'execution_mode': 'logic-only-completion', 'baseline_headers': headers[1],
                'baseline_core': digest(core),
                'candidate_core': digest(build / 'navigation2d_build/libnavigation2d_core.a'),
                'fixtures': fixtures, 'builder': digest(Path(__file__)), 'artifacts': {}}
    for label, path in [('baseline', args.baseline_source),
                        ('candidate', root / 'ros/autonomous_explorer_node.cc')]:
        cpp, obj, binary = [output / (label + suffix) for suffix in ['.cc', '.o', '']]
        original_digest = digest(path)
        cpp.write_text(completion_source(path.read_text(), label))
        selected = ['-I' + str(headers[0]), *flags] if label == 'baseline' else flags
        compilation = compile_cached(template[0], selected, cpp, obj)
        link = template.copy()
        objects = [i for i, value in enumerate(link) if value.endswith('autonomous_explorer_node.cc.o')]
        if len(objects) != 1:
            raise ValueError('unexpected explorer target objects')
        link[objects[0]] = str(obj)
        link[link.index('-o') + 1] = str(binary)
        if label == 'baseline':
            link = baseline_link(link, obj, core)
        subprocess.run(link, cwd=build, check=True)
        manifest['artifacts'][label] = {'source': digest(cpp), 'binary': digest(binary),
                                        'original_source': original_digest, 'compilation': compilation}
        if digest(path) != original_digest:
            raise RuntimeError('node source changed while compiling')
    if snapshot(headers[0]) != headers[1] or fixtures != {str(p.relative_to(root)): digest(p) for p in fixture_paths}:
        raise RuntimeError('replay inputs changed while compiling')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
