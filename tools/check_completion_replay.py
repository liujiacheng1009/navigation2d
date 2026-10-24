#!/usr/bin/env python3
"""Check direct completion-phase outputs and explicit benchmark measurements."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
from compare_benchmarks import behavior


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('ros_arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    ros = args.ros_arguments[1:] if args.ros_arguments[:1] == ['--'] else args.ros_arguments
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, SWEEPNAV_EXPLORATION_RESULT=str(
        output.parent / 'completion-node-artifacts/result.json'))
    raw, results = {}, {}
    for label in ['baseline', 'candidate']:
        executable = str(getattr(args, label).resolve(strict=True))
        for benchmark in [False, True]:
            mode = 'benchmark' if benchmark else 'plain'
            process = subprocess.run([executable, *(['--benchmark'] if benchmark else []), *ros],
                                     env=env, capture_output=True, text=True, timeout=120)
            prefix = output.parent / (label + '-' + mode)
            prefix.with_suffix('.stdout').write_text(process.stdout)
            prefix.with_suffix('.stderr').write_text(process.stderr)
            if process.returncode:
                raise RuntimeError(f'{label}/{mode} failed ({process.returncode}); inspect {prefix}')
            observed = json.loads(process.stdout)
            if not benchmark:
                raw[label], results[label] = process.stdout, observed
            else:
                if behavior(observed) != {'trace': results[label]}:
                    raise RuntimeError('benchmark changed completion behavior')
                for key in ['empty_us', 'residual_us', 'blocked_us', 'finish_us',
                            'wall_seconds', 'peak_rss_kb']:
                    if not math.isfinite(observed[key]) or observed[key] < 0:
                        raise RuntimeError('invalid measurement: ' + key)
    equal = raw['baseline'] == raw['candidate']
    output.write_text(json.dumps({'equal': equal, 'outputs': results}, indent=2) + '\n')
    if not equal:
        raise RuntimeError('completion outputs differ; inspect ' + str(output))
    rows = {row['case']: row for row in results['candidate']}
    if set(rows) != {'empty', 'residual', 'blocked', 'finish'} or len(results['candidate']) != 4:
        raise RuntimeError('completion scenario set is incomplete')
    if rows['residual']['phase'] != 1 or rows['empty']['phase'] == 1:
        raise RuntimeError('completion/empty-return branches were not reached')
    print('four completion cases match exactly; measurements preserve behavior')
    for name, row in rows.items():
        print(name, 'phase', row['phase'], 'navigation', row['navigation'], 'finished', row['finished'])


if __name__ == '__main__':
    main()
