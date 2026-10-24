#!/usr/bin/env python3
"""Compare stateful RPP commands and diagnostics against frozen original APIs."""
import argparse
import json
import math
from pathlib import Path
import subprocess
from compare_benchmarks import behavior
from frozen_core_probe import build_probe, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['build', 'baseline-headers', 'baseline-core', 'output']:
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    source = root / 'test/pursuit_replay/contract_replay.cc'
    baseline, candidate, provenance = build_probe(
        args.build, 'navigation2d_pursuit_replay', source,
        args.baseline_headers, args.baseline_core, output)
    config = root / 'config/navigation2d.yaml'
    raw, records = {}, {}
    for label, binary in [('baseline', baseline), ('candidate', candidate)]:
        for benchmark in [False, True]:
            mode = 'benchmark' if benchmark else 'plain'
            process = subprocess.run([str(binary), str(config), *(['--benchmark'] if benchmark else [])],
                                     text=True, capture_output=True, timeout=120)
            (output / (label + '-' + mode + '.stdout')).write_text(process.stdout)
            (output / (label + '-' + mode + '.stderr')).write_text(process.stderr)
            if process.returncode:
                raise RuntimeError(label + '/' + mode + ' failed; inspect ' + str(output))
            observed = json.loads(process.stdout)
            if benchmark:
                if behavior(observed) != {'trace': records[label]}:
                    raise RuntimeError('measurement mode changed commands/diagnostics')
            else:
                raw[label], records[label] = process.stdout, observed
                keys = {(r['profile'], r['map'], r['task'], r['step']) for r in observed}
                if len(observed) != 960 or len(keys) != 960:
                    raise RuntimeError('pursuit scenario capture is incomplete')
                for row in observed:
                    if not all(math.isfinite(row[key]) for key in ['vx', 'w']):
                        raise RuntimeError('nonfinite pursuit command')
                if {r['maneuver'] for r in observed} != {0, 1, 2, 3}:
                    raise RuntimeError('required tracking/stop/rotate/infeasible states not reached')
    equal = raw['baseline'] == raw['candidate']
    (output / 'comparison.json').write_text(json.dumps(
        {'equal': equal, 'records': 960, 'input_sha256': digest(config), **provenance}, indent=2) + '\n')
    print('960 stateful RPP records:', 'equal' if equal else 'DIFFERENT')
    if not equal:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
