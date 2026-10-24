#!/usr/bin/env python3
"""Compare exploration-domain outputs against a separately frozen header/core pair.

Run in the native CMake environment. This diagnostic driver supplements the
normal tests; it neither disables their assertions nor approves known failures.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from frozen_core_probe import build_probe


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
    source = root / 'test/frontier_replay/contract_replay.cc'
    binary, candidate, provenance = build_probe(
        build, 'navigation2d_frontier_replay', source, headers, baseline_core, output)
    results = {}
    captures = {}
    for label, executable in [('baseline', binary),
                              ('candidate', candidate)]:
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
    report = {'equal': equal, 'coverage': coverage,
              'records': {k: len(v) for k, v in results.items()}, **provenance,
              'fixture_sha256': {p.name: digest(p) for p in source.parent.glob('*.h')}}
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
