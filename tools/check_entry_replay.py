#!/usr/bin/env python3
"""Compare frozen/current entry flows and require that each intended branch ran."""
import argparse
import json
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('ros_arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    ros = args.ros_arguments
    if ros[:1] == ['--']:
        ros = ros[1:]
    results = {}
    for label in ['baseline', 'candidate']:
        process = subprocess.run([str(getattr(args, label).resolve()), 'entry', *ros],
                                 check=True, capture_output=True, text=True)
        results[label] = json.loads(process.stdout)
    results['behavior_equal'] = results['baseline'] == results['candidate']
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    if not results['behavior_equal']:
        raise RuntimeError('entry outputs differ; inspect ' + str(args.output))
    rows = {row['case']: row for row in results['candidate']}
    expected = {
        'approach': (1, 1, 1, 0, 1),
        'unavailable': (0, 1, 1, 1, 0),
        'probe': (1, 1, 1, 1, 1),
        'inside': (0, 0, 0, 0, 0),
        'cooling': (0, 1, 0, 0, 0),
    }
    if set(rows) != set(expected) or len(results['candidate']) != len(expected):
        raise RuntimeError('entry scenario set is incomplete')
    fields = ['driving', 'pending', 'locked', 'attempts', 'navigation']
    for name, values in expected.items():
        actual = tuple(rows[name][key] for key in fields)
        if actual != values:
            raise RuntimeError(f'{name}: intended branch not reached: {actual} != {values}')
    # Crossing acknowledges the entrance, not completion of the whole region.
    if (rows['inside']['completed'] != 0 or rows['inside']['publishes'] != 1
            or rows['cooling']['publishes'] != 0):
        raise RuntimeError('entry completion/cooling effects differ from the contract')
    print('five entry branches reached; all frozen/current outputs equal')


if __name__ == '__main__':
    main()
