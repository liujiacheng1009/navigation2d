"""Acceptance-policy boundaries, using complete paired raw evidence."""
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from assess_no_regression import assess, validate_calibration
from compare_benchmarks import interval


def report(differences, margin=1):
    return {'calibration': False, 'pairs': len(differences), 'cases': [[]],
            'metric_names': ['wall_seconds'], 'scope': 'test',
            'baseline_sha256': 'baseline', 'candidate_sha256': 'candidate',
            'input_sha256': {}, 'platform': 'test', 'cpu_affinity': [0],
            'results': [{'behavior_equal': True,
                         'samples': [{'baseline': {'wall_seconds': 100, 'state': 1},
                                      'candidate': {'wall_seconds': 100 + d, 'state': 1}}
                                     for d in differences],
                         'metrics': {'wall_seconds': {
                             'ci95': interval(differences), 'margin': margin,
                             'mean_difference': sum(differences) / len(differences)}}}]}


class NoRegressionContract(unittest.TestCase):
    def test_speedup_and_boundary_are_accepted(self):
        for differences, margin in [([-10.] * 30, 1), ([1.] * 30, 1), ([0.] * 30, 0)]:
            self.assertEqual(assess(report(differences, margin))['status'], 'PASS')

    def test_regression_and_uncertainty_are_distinct(self):
        self.assertEqual(assess(report([2.] * 30))['status'], 'FAILED')
        self.assertEqual(assess(report([0., 2.] * 15))['status'], 'INCONCLUSIVE')

    def test_behavior_failure_overrides_speedup(self):
        raw = report([-10.] * 30)
        raw['results'][0]['samples'][0]['candidate']['state'] = 2
        raw['results'][0]['behavior_equal'] = False
        self.assertEqual(assess(raw)['status'], 'FAILED')

    def test_incomplete_or_forged_evidence_is_rejected(self):
        original = report([-10.] * 30)
        changes = [lambda r: r.update(metric_names=['unknown']),
                   lambda r: r['results'][0]['samples'].pop(),
                   lambda r: r['results'][0]['metrics']['wall_seconds'].update(margin=float('nan')),
                   lambda r: r['results'][0]['metrics']['wall_seconds'].update(ci95=[-100, -100]),
                   lambda r: r['results'][0]['samples'][0]['candidate'].update(state=2),
                   lambda r: r['results'][0]['samples'][0]['candidate'].update(wall_seconds=float('inf'))]
        for change in changes:
            raw = copy.deepcopy(original)
            change(raw)
            with self.subTest(raw=raw), self.assertRaises((ValueError, KeyError)):
                assess(raw)

    def test_calibration_provenance_and_margin_are_required(self):
        raw = report([-10.] * 30)
        frozen = report([0.] * 30)
        frozen.update(calibration=True, candidate_sha256='baseline')
        validate_calibration(raw, frozen)
        for key, value in [('baseline_sha256', 'other'), ('scope', 'other'),
                           ('input_sha256', {'changed': 'input'}), ('cpu_affinity', [1])]:
            changed = copy.deepcopy(raw)
            changed[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_calibration(changed, frozen)
        frozen['results'][0]['metrics']['wall_seconds']['margin'] = 2
        with self.assertRaises(ValueError):
            validate_calibration(raw, frozen)

    def test_cli_never_overwrites_evidence(self):
        tool = Path(__file__).resolve().parents[1] / 'tools/assess_no_regression.py'
        with tempfile.TemporaryDirectory() as directory:
            import json
            path = Path(directory) / 'raw.json'
            path.write_text(json.dumps(report([-10.] * 30)))
            before = path.read_bytes()
            calibration = report([0.] * 30)
            calibration.update(calibration=True, candidate_sha256='baseline')
            frozen = Path(directory) / 'aa.json'
            frozen.write_text(json.dumps(calibration))
            result = subprocess.run([sys.executable, str(tool), str(path), '--output', str(path),
                                     '--calibration', str(frozen)],
                                    capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 1)
            self.assertEqual(path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
