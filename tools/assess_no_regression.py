#!/usr/bin/env python3
"""Apply the approved improvement-permitted policy to an immutable A/B report.

Retains the producer's paired 95% confidence interval and frozen A/A margin.
All declared metrics are lower-is-better. Does not recalibrate or drop samples.
Exit codes: 0 PASS, 1 FAILED/invalid evidence, 2 INCONCLUSIVE.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from compare_benchmarks import TIMING_FIELDS, behavior, interval


def finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def assess(report: dict) -> dict:
    """Validate raw evidence, then apply the positive regression boundary."""
    names = report['metric_names']
    pairs = report['pairs']
    if (report['calibration'] is not False or not isinstance(pairs, int)
            or isinstance(pairs, bool) or pairs < 30 or not names
            or len(names) != len(set(names))
            or not set(names) <= TIMING_FIELDS - {'tick_us'}
            or not report['cases'] or len(report['results']) != len(report['cases'])):
        raise ValueError('invalid A/B scope, sample count or measured metrics')
    results = []
    for case in report['results']:
        if len(case['samples']) != pairs or set(case['metrics']) != set(names):
            raise ValueError('missing pairs or metric evidence')
        reference = behavior(case['samples'][0]['baseline'])
        equal = all(behavior(p[side]) == reference
                    for p in case['samples'] for side in ('baseline', 'candidate'))
        if type(case['behavior_equal']) is not bool or case['behavior_equal'] != equal:
            raise ValueError('behavior summary disagrees with raw samples')
        metrics = {}
        for name in names:
            raw = case['metrics'][name]
            values = [p[side][name] for p in case['samples']
                      for side in ('baseline', 'candidate')]
            if not all(finite(v) and v >= 0 for v in values):
                raise ValueError('invalid timing or RSS sample')
            differences = [p['candidate'][name] - p['baseline'][name]
                           for p in case['samples']]
            bounds = interval(differences)
            margin = raw['margin']
            if (not finite(margin) or margin < 0 or raw['ci95'] != bounds
                    or not finite(raw['mean_difference'])
                    or not bounds[0] <= raw['mean_difference'] <= bounds[1]):
                raise ValueError('invalid interval or frozen margin')
            status = ('PASS' if bounds[1] <= margin else
                      'FAILED' if bounds[0] > margin else 'INCONCLUSIVE')
            metrics[name] = {'ci95': bounds, 'margin': margin, 'status': status}
        statuses = [m['status'] for m in metrics.values()]
        status = ('FAILED' if not equal or 'FAILED' in statuses else
                  'INCONCLUSIVE' if 'INCONCLUSIVE' in statuses else 'PASS')
        results.append({'behavior_equal': equal, 'metrics': metrics, 'status': status})
    statuses = [r['status'] for r in results]
    return {'policy': 'allow improvements; prohibit regressions (2026-10-29)',
            'scope': report['scope'], 'pairs': pairs,
            'baseline_sha256': report['baseline_sha256'],
            'candidate_sha256': report['candidate_sha256'],
            'results': results,
            'status': ('FAILED' if 'FAILED' in statuses else 'INCONCLUSIVE'
                       if 'INCONCLUSIVE' in statuses else 'PASS')}


def validate_calibration(report: dict, calibration: dict) -> None:
    """Reject a different or behaviorally unstable A/A reference."""
    keys = ('cases', 'metric_names', 'baseline_sha256', 'input_sha256',
            'platform', 'cpu_affinity', 'scope')
    if (calibration['calibration'] is not True
            or calibration['candidate_sha256'] != calibration['baseline_sha256']
            or any(report[k] != calibration[k] for k in keys)):
        raise ValueError('A/A reference does not match the A/B provenance')
    checked = assess(dict(calibration, calibration=False))
    if not all(case['behavior_equal'] for case in checked['results']):
        raise ValueError('A/A behavior is unstable')
    for measured, frozen in zip(report['results'], calibration['results']):
        if any(measured['metrics'][name]['margin'] != frozen['metrics'][name]['margin']
               for name in report['metric_names']):
            raise ValueError('A/B margin differs from frozen A/A')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    args = parser.parse_args()
    try:
        raw = args.report.read_bytes()
        report = json.loads(raw)
        frozen = args.calibration.read_bytes()
        validate_calibration(report, json.loads(frozen))
        result = assess(report)
        result['calibration_sha256'] = hashlib.sha256(frozen).hexdigest()
        result['source_report_sha256'] = hashlib.sha256(raw).hexdigest()
        result['source_report'] = str(args.report.resolve())
        # Exclusive creation preserves both the input and previous assessments.
        with args.output.open('x') as output:
            output.write(json.dumps(result, indent=2, allow_nan=False) + '\n')
    except (KeyError, ValueError, TypeError, OSError) as error:
        parser.exit(1, f'invalid evidence or output: {error}\n')
    print(result['status'])
    return {'PASS': 0, 'FAILED': 1, 'INCONCLUSIVE': 2}[result['status']]


if __name__ == '__main__':
    raise SystemExit(main())
