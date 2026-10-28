#!/usr/bin/env python3
"""Compare scalar C++ allocation and behavior traces using independent cores.

Explicit evidence target, not a latency benchmark or whole-process memory test.
No cycle is filtered. Frozen headers/core must belong to the same baseline.
"""
import argparse
import csv
import io
import json
from pathlib import Path
import subprocess

from frozen_core_probe import build_probe, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True, help='navigation2d CMake build')
    parser.add_argument('--baseline-headers', type=Path, required=True)
    parser.add_argument('--baseline-core', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    old, new, provenance = build_probe(
        args.build, 'navigation_allocation_probe', root / 'test/navigation_allocation_probe.cc',
        args.baseline_headers, args.baseline_core, args.output)
    width, height = 100, 60
    cells = [254 if x in [0, width - 1] or y in [0, height - 1] else 0
             for y in range(height) for x in range(width)]
    world = args.output / 'world.json'
    world.write_text(json.dumps({'width': width, 'height': height, 'resolution': .05,
                                'cells': cells}))
    provenance['world_sha256'] = digest(world)
    provenance['checker_sha256'] = digest(Path(__file__))
    (args.output / 'manifest.json').write_text(json.dumps(provenance, indent=2) + '\n')
    results = []
    for case in ['normal', 'stuck']:
        records = []
        for label, executable in [('baseline', old), ('candidate', new)]:
            result = subprocess.run([str(executable), str(world.resolve()), case],
                                    capture_output=True, text=True)
            (args.output / (label + '-' + case + '.csv')).write_text(result.stdout)
            (args.output / (label + '-' + case + '.stderr')).write_text(result.stderr)
            result.check_returncode()
            rows = [list(map(int, row)) for row in csv.reader(io.StringIO(result.stdout))]
            if (len(rows) != 400 or any(len(row) != 6 for row in rows) or
                    [row[0] for row in rows] != list(range(400))):
                raise ValueError('incomplete allocation trace')
            records.append(rows)
        behavior_equal = all(a[3:] == b[3:] for a, b in zip(*records))
        differences = [{'tick': a[0], 'baseline': a[1:3], 'candidate': b[1:3]}
                       for a, b in zip(*records) if a[1:3] != b[1:3]]
        no_increase = all(b[1] <= a[1] and b[2] <= a[2] for a, b in zip(*records))
        totals = [{'calls': sum(row[1] for row in rows), 'bytes': sum(row[2] for row in rows)}
                  for rows in records]
        results.append({'case': case, 'behavior_equal': behavior_equal, 'totals': totals,
                        'no_cycle_increase': no_increase, 'allocation_differences': differences})
        print(case, 'behavior_equal=', behavior_equal, 'no_cycle_increase=', no_increase,
              'totals=', totals, flush=True)
    passed = all(case['behavior_equal'] and case['no_cycle_increase'] for case in results)
    report = {'all_pass': passed, 'cases': results,
              'scope': 'scalar C++ new inside ComputeCommand; not malloc/aligned/SDK/RSS',
              'timing_claim': False}
    (args.output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
