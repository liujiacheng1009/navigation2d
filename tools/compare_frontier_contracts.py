#!/usr/bin/env python3
"""Compare exploration-domain outputs against a separately frozen header/core pair.

Run in the native CMake environment. This diagnostic driver supplements the
normal tests; it neither disables their assertions nor approves known failures.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True, type=Path, help='navigation2d build directory')
    parser.add_argument('--baseline-headers', required=True, type=Path,
                        help='include root containing frozen navigation2d headers')
    parser.add_argument('--baseline-core', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = args.build.resolve(strict=True)
    headers = args.baseline_headers.resolve(strict=True)
    baseline_core = args.baseline_core.resolve(strict=True)
    if baseline_core == (build / 'libnavigation2d_core.a').resolve():
        parser.error('baseline core must be independently frozen')
    if not (headers / 'navigation2d/exploration/frontier_explorer.h').is_file():
        parser.error('baseline header tree is incomplete')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    header_hashes = {str(p.relative_to(headers)): digest(p)
                     for p in (headers / 'navigation2d').rglob('*.h')}
    project_build = next((p for p in [build, *build.parents]
                          if (p / 'CMakeCache.txt').is_file()), None)
    if project_build is None:
        parser.error('build directory does not belong to a configured CMake tree')
    subprocess.run(['cmake', '--build', str(project_build), '--target',
                    'navigation2d_frontier_replay', '--parallel', '2'], check=True)
    target = build / 'test/CMakeFiles/navigation2d_frontier_replay.dir'
    flags = []
    for line in (target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    link = shlex.split((target / 'link.txt').read_text())
    source = root / 'test/frontier_replay/contract_replay.cc'
    obj = output / 'baseline.o'
    binary = output / 'baseline'
    subprocess.run([link[0], '-I' + str(headers), *flags, '-c', str(source), '-o', str(obj)],
                   cwd=build / 'test', check=True)
    rewritten = []
    objects = cores = 0
    for token in link:
        if token.endswith('.o'):
            rewritten.append(str(obj))
            objects += 1
        elif token.endswith('libnavigation2d_core.a'):
            rewritten.append(str(baseline_core))
            cores += 1
        elif token.endswith('libnavigation2d_geometry.a'):
            # The frozen pre-split core already owns its geometry implementation.
            continue
        else:
            rewritten.append(token)
    if objects != 1 or cores != 1:
        raise ValueError('unsupported replay link: expected one object and one core')
    rewritten[rewritten.index('-o') + 1] = str(binary)
    subprocess.run(rewritten, cwd=build / 'test', check=True)
    current_hashes = {str(p.relative_to(headers)): digest(p)
                      for p in (headers / 'navigation2d').rglob('*.h')}
    if header_hashes != current_hashes:
        raise ValueError('baseline headers changed during build')
    results = {}
    captures = {}
    for label, executable in [('baseline', binary),
                              ('candidate', build / 'test/navigation2d_frontier_replay')]:
        capture = subprocess.check_output([str(executable)], text=True)
        (output / (label + '.jsonl')).write_text(capture)
        captures[label] = capture
        results[label] = [json.loads(line) for line in capture.splitlines()]
    snapshots = [row for row in results['baseline'] if 'scenario' in row]
    coverage = {'snapshots': len(snapshots),
                'scenarios': len({row['scenario'] for row in snapshots}),
                'selected': sum(bool(row['selected']) for row in snapshots),
                'tours': sum(bool(row['tour']) for row in snapshots),
                'leftovers': sum(bool(row['leftovers']) for row in snapshots),
                'goal_queries': sum('settled' in row for row in results['baseline'])}
    if (coverage['snapshots'] != 72 or coverage['scenarios'] != 12 or
            not coverage['selected'] or not coverage['goal_queries']):
        raise ValueError('contract driver did not exercise its required scenarios')
    # Exact text also preserves signed zero in round-trip double formatting.
    equal = captures['baseline'] == captures['candidate']
    report = {'equal': equal, 'coverage': coverage, 'records': {k: len(v) for k, v in results.items()},
              'baseline_core_sha256': digest(baseline_core),
              'candidate_core_sha256': digest(build / 'libnavigation2d_core.a'),
              'source_sha256': digest(source),
              'fixture_sha256': {p.name: digest(p) for p in source.parent.glob('*.h')},
              'baseline_binary_sha256': digest(binary),
              'candidate_binary_sha256': digest(build / 'test/navigation2d_frontier_replay'),
              'baseline_headers': header_hashes,
              'baseline_compile': [link[0], '-I' + str(headers), *flags],
              'baseline_link': rewritten}
    if not equal:
        report['first_difference'] = next((i for i, pair in enumerate(zip(
            captures['baseline'].splitlines(), captures['candidate'].splitlines()))
            if pair[0] != pair[1]), None)
    (output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'equal': equal, 'records': report['records']}))
    if not equal:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
