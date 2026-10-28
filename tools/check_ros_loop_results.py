#!/usr/bin/env python3
"""Check completed isolated ROS runs and default frozen-node spatial equivalence.

Real-time scheduling changes publication positions slightly: use the fixed
5 cm whole-trajectory envelope, independently of the exact captured-tick replay.
No trajectory points are removed. This is behavior evidence, not a timing gate.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import tarfile


def manifest(folder):
    value = json.loads((folder / 'manifest.json').read_text())
    if not value['cases'] or any(case['returncode'] != 0 for case in value['cases']):
        raise ValueError('ROS cases failed or did not execute: ' + str(folder))
    return value


def load_case(archive, case):
    prefix = 'explorer-ros-loops/' + '-'.join([case['scenario'], case['planner'], case['controller']])
    report = json.load(archive.extractfile(prefix + '/result.json'))
    with io.TextIOWrapper(archive.extractfile(prefix + '/commands.csv')) as stream:
        points = [(float(row['x']), float(row['y'])) for row in csv.DictReader(stream)]
    if not points or not all(math.isfinite(x) and math.isfinite(y) for x, y in points):
        raise ValueError('missing or invalid trajectory')
    error = report['return_error_m']
    if (report['status'] != 'COMPLETE' or not math.isfinite(error) or error < 0. or error > .35 or
            report['planning'] != 'navigation2d/' + case['planner'] + '+' + case['controller']):
        raise ValueError('ROS mission result contract failed: ' + prefix)
    return report, points


def directed_distance(first, second):
    return max(min(math.hypot(x - u, y - v) for u, v in second) for x, y in first)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--scenarios', nargs='+', choices=['return', 'frontier', 'obstacle', 'scan_pause'],
                        default=['return', 'frontier'])
    parser.add_argument('--expected-candidate-cases', type=int, default=32)
    parser.add_argument('--output', type=Path, required=True, help='new report file')
    args = parser.parse_args()
    old, new = manifest(args.baseline), manifest(args.candidate)
    if not old.get('binary_provenance') or old['image'] != new['image']:
        raise ValueError('frozen binary provenance or identical runtime image missing')
    for source in ['test/explorer_ros_loop_test.cc', 'test/explorer_ros_loop_scene.h', 'test/ros_loop_room.h']:
        if old['sources'][source] != new['sources'][source]:
            raise ValueError('baseline and candidate use different synthetic fixtures')
    old_keys = {(case['scenario'], case['planner'], case['controller']) for case in old['cases']}
    required = {(scenario, 'theta_star', 'rpp') for scenario in args.scenarios}
    if not required.issubset(old_keys) or len(old_keys) != len(old['cases']):
        raise ValueError('default frozen scenarios incomplete or duplicated')
    if any(case['planner'] != 'theta_star' or case['controller'] != 'rpp' for case in old['cases']):
        raise ValueError('original node supports only the default selection')
    selected_old = [case for case in old['cases'] if case['scenario'] in args.scenarios]
    if len(selected_old) != len(args.scenarios) or len(new['cases']) != args.expected_candidate_cases:
        raise ValueError('incomplete ROS case set')
    candidate_keys = {(case['scenario'], case['planner'], case['controller']) for case in new['cases']}
    if args.expected_candidate_cases == len(args.scenarios) * 16:
        complete = {(scenario, planner, controller) for scenario in args.scenarios
                    for planner in ['dijkstra', 'astar', 'theta_star', 'state_lattice']
                    for controller in ['rpp', 'dwa', 'mppi', 'mpc']}
        if candidate_keys != complete:
            raise ValueError('planner/controller ROS matrix incomplete')
    if len(candidate_keys) != len(new['cases']):
        raise ValueError('duplicate ROS case')
    comparisons = []
    with tarfile.open(args.baseline / 'raw-results.tgz') as baseline, \
            tarfile.open(args.candidate / 'raw-results.tgz') as candidate:
        for case in new['cases']:
            load_case(candidate, case)
        for case in old['cases']:
            load_case(baseline, case)
        for case in selected_old:
            if case['planner'] != 'theta_star' or case['controller'] != 'rpp':
                raise ValueError('original node comparison is limited to its default selection')
            if (case['scenario'], case['planner'], case['controller']) not in candidate_keys:
                raise ValueError('default baseline counterpart missing')
            before, first = load_case(baseline, case)
            after, second = load_case(candidate, case)
            fields = ['status', 'planning', 'pose_source', 'map_source', 'known_cells',
                      'completed_goals', 'failed_goals']
            differences = [field for field in fields if before[field] != after[field]]
            distance = max(directed_distance(first, second), directed_distance(second, first))
            travel_delta = abs(before['travel_distance_m'] - after['travel_distance_m'])
            equal = not differences and distance <= .05 and travel_delta <= .05
            comparisons.append({'scenario': case['scenario'], 'equal': equal,
                                'different_fields': differences, 'spatial_envelope_m': distance,
                                'travel_distance_delta_m': travel_delta,
                                'baseline_points': len(first), 'candidate_points': len(second)})
    result = {'candidate_cases': len(new['cases']), 'default_comparisons': comparisons,
              'all_pass': all(case['equal'] for case in comparisons),
              'scope': 'synthetic real ROS behavior only; no performance/physics/hardware claim'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps(result, indent=2))
    return 0 if result['all_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
