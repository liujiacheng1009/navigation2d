#!/usr/bin/env python3
"""Paired deterministic replay and timing comparison of two benchmark binaries.

Run --calibrate on the frozen baseline first. Timing margins come only from
that A/A run. Reports are evidence, not proof of hardware WCET or equivalence
on a different platform. Exit 2 means insufficient/noisy data, 1 a regression.
"""

import argparse
import hashlib
import json
import math
import os
import platform
from pathlib import Path
import statistics
import subprocess

METRICS = ('wall_seconds', 'control_cycle_p50_us', 'control_cycle_p95_us',
           'control_cycle_p99_us', 'controller_solve_p99_us', 'peak_rss_kb')


# Explicit measured fields only: a future timeout_seconds or budget_us is
# behavior until the output schema deliberately declares it a measurement.
TIMING_FIELDS = {'wall_seconds', 'peak_rss_kb', 'global_plan_seconds',
                 'global_plan_first_solution_seconds', 'tick_us', 'approach_us',
                 'unavailable_us', 'probe_us', 'inside_us', 'cooling_us'} | {
    f'{family}_p{percentile}_us'
    for family in ('control_cycle', 'controller_solve', 'pure_solver', 'global_plan',
                   'global_plan_first_solution', 'tick')
    for percentile in (50, 95, 99)}


def behavior(result):
    """Keep every output except explicitly declared measured timing/RSS fields."""
    return {key: value for key, value in result.items() if key not in TIMING_FIELDS}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(binary, command):
    result = subprocess.run([binary, *command], check=True, text=True,
                            capture_output=True, timeout=120)
    return json.loads(result.stdout)


def interval(values):
    mean = statistics.mean(values)
    # Conservative large-sample normal approximation; raw pairs are retained
    # so a reviewer can use batch bootstrap if the distribution is unsuitable.
    half = 1.96 * statistics.stdev(values) / math.sqrt(len(values))
    return [mean - half, mean + half]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-files', nargs='*', type=Path, default=[])
    parser.add_argument('--metrics', nargs='+', default=list(METRICS))
    parser.add_argument('--scope', default='NavigationSystem simulation only; excludes ROS exploration Tick and board validation.')
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate')
    parser.add_argument('--cases', type=Path, required=True,
                        help='JSON list of benchmark argument lists, excludes executable')
    parser.add_argument('--pairs', type=int, default=30)
    parser.add_argument('--calibrate', action='store_true')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not set(args.metrics) <= TIMING_FIELDS - {'tick_us'}:
        parser.error('unknown measured metric; extend the explicit output schema first')
    if args.pairs < 30:
        parser.error('at least 30 independent pairs required')
    if not args.calibrate and (not args.candidate or not args.calibration):
        parser.error('comparison requires --candidate and frozen --calibration')
    cases = json.loads(args.cases.read_text())
    if not cases or not all(isinstance(c, list) and all(isinstance(x, str) for x in c) for c in cases):
        parser.error('cases must be a nonempty list of string argument lists')
    input_hashes = {str(Path(value).resolve()): digest(value) for case in cases for value in case
                    if Path(value).is_file()}
    input_hashes.update({str(p.resolve()): digest(p) for p in args.input_files})
    calibration = None if args.calibrate else json.loads(args.calibration.read_text())
    if calibration and (calibration['cases'] != cases or
                        calibration['baseline_sha256'] != digest(args.baseline) or
                        calibration.get('input_sha256') != input_hashes):
        parser.error('baseline or cases differ from calibration')
    if calibration and (calibration.get('metric_names', list(METRICS)) != args.metrics or
                        calibration.get('scope') != args.scope):
        parser.error('metric names or measurement scope differ from calibration')
    candidate = args.baseline if args.calibrate else args.candidate
    report = {'baseline_sha256': digest(args.baseline),
              'candidate_sha256': digest(candidate), 'cases': cases,
              'pairs': args.pairs, 'calibration': args.calibrate, 'results': [],
              'metric_names': args.metrics,
              'input_sha256': input_hashes, 'platform': platform.platform(),
              'cpu_affinity': sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else None}
    failed = False
    inconclusive = False
    for case_index, command in enumerate(cases):
        run(args.baseline, command)
        run(candidate, command)
        pairs = []
        reference = None
        deterministic = True
        for index in range(args.pairs):
            if index % 2:
                b = run(candidate, command)
                a = run(args.baseline, command)
            else:
                a = run(args.baseline, command)
                b = run(candidate, command)
            observed = behavior(a)
            reference = observed if reference is None else reference
            deterministic &= observed == reference and behavior(b) == reference
            pairs.append({'baseline': a, 'candidate': b})
        entry = {'behavior_equal': deterministic, 'samples': pairs, 'metrics': {}}
        failed |= not deterministic
        for metric in args.metrics:
            differences = [p['candidate'][metric] - p['baseline'][metric] for p in pairs]
            bounds = interval(differences)
            # Freeze empirical A/A difference distribution before candidate
            # measurement; it is a noise estimate, not a degradation budget.
            if args.calibrate:
                margin = sorted(abs(x) for x in differences)[math.ceil(.95 * len(differences)) - 1]
                status = 'CALIBRATED'
            else:
                margin = calibration['results'][case_index]['metrics'][metric]['margin']
                if all(d == 0 for d in differences):
                    status = 'EQUAL'
                elif margin > 0 and bounds[0] >= -margin and bounds[1] <= margin:
                    status = 'WITHIN_AA_NOISE'
                elif bounds[0] > margin:
                    status = 'REGRESSION'
                    failed = True
                else:
                    status = 'INCONCLUSIVE'
                    inconclusive = True
            entry['metrics'][metric] = {'mean_difference': statistics.mean(differences),
                                         'ci95': bounds, 'margin': margin, 'status': status}
        report['results'].append(entry)
        print(f'case {case_index}: behavior_equal={deterministic}', flush=True)
    if calibration and not all(r['behavior_equal'] for r in calibration['results']):
        inconclusive = True
    report['status'] = ('FAILED' if failed else 'INCONCLUSIVE' if inconclusive
                        else 'CALIBRATED' if args.calibrate else 'WITHIN_MEASURED_NOISE')
    report['scope'] = args.scope
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(report['status'])
    return 1 if failed else 2 if inconclusive else 0


if __name__ == '__main__':
    raise SystemExit(main())
