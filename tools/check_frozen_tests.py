#!/usr/bin/env python3
"""Compare standalone test scenarios with a frozen core and its own headers.

Uses the actual CMake target's compiler/linker inputs. Never rebuilds or mutates
that target. Existing failed assertions are compared, not silently allowed;
new candidate-only production headers and archives cannot enter the old build.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess

from replay_baseline_bundle import digest
from replay_baseline_headers import baseline_link, prepare, snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True, help='navigation2d build directory')
    parser.add_argument('--baseline-core', type=Path, required=True)
    parser.add_argument('--baseline-include-root', type=Path, required=True)
    parser.add_argument('--tests', nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    headers, header_hashes = prepare(None, args.baseline_include_root)
    args.output.mkdir(parents=True, exist_ok=False)
    core_hash = digest(args.baseline_core)
    summary = []
    for name in args.tests:
        if not re.fullmatch(r'[A-Za-z0-9_]+', name):
            raise ValueError('invalid test target: ' + name)
        source = root / 'test' / (name + '.cc')
        source_hash = digest(source)
        target = args.build / 'test' / 'CMakeFiles' / (name + '.dir')
        flags = []
        for line in (target / 'flags.make').read_text().splitlines():
            if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
                flags.extend(shlex.split(line.split('=', 1)[1]))
        link = shlex.split((target / 'link.txt').read_text())
        directory = args.output / name
        directory.mkdir()
        obj, binary = directory / 'baseline.o', directory / name
        subprocess.run([link[0], '-I' + str(headers), *flags, '-MD', '-MF',
                        str(directory / 'baseline.d'), '-c', str(source), '-o', str(obj)], check=True)
        dependencies = shlex.split((directory / 'baseline.d').read_text().replace('\\\n', ' ').split(':', 1)[1])
        leaked = [str(p) for p in map(lambda value: Path(value).resolve(), dependencies)
                  if root in p.parents and root / 'test' not in p.parents]
        if leaked:
            raise ValueError('candidate production headers entered frozen build: ' + repr(leaked))
        objects = [index for index, value in enumerate(link) if value.endswith('.cc.o')]
        if len(objects) != 1:
            raise ValueError('only standalone single-source test targets are supported: ' + name)
        link[objects[0]] = str(obj)
        link[link.index('-o') + 1] = str(binary)
        link = baseline_link(link, obj, args.baseline_core.resolve())
        subprocess.run(link, cwd=target.parent.parent, check=True)
        outcomes = []
        for label, executable in [('baseline', binary), ('candidate', args.build / name)]:
            result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=120)
            outcome = {'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
            (directory / (label + '.json')).write_text(json.dumps(outcome, indent=2) + '\n')
            outcomes.append(outcome)
        if source_hash != digest(source):
            raise ValueError('test source changed during verification: ' + name)
        summary.append({'test': name, 'equal': outcomes[0] == outcomes[1],
                        'baseline_returncode': outcomes[0]['returncode'],
                        'source_sha256': source_hash, 'baseline_sha256': digest(binary),
                        'candidate_sha256': digest(args.build / name)})
    if snapshot(headers) != header_hashes or digest(args.baseline_core) != core_hash:
        raise ValueError('frozen inputs changed during verification')
    report = {'baseline_core_sha256': core_hash, 'baseline_headers': header_hashes, 'tests': summary}
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if all(item['equal'] for item in summary) else 1


if __name__ == '__main__':
    raise SystemExit(main())
