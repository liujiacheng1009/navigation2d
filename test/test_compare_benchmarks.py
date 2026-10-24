"""Checks that the performance helper retains behavioral regressions."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "compare_benchmarks", Path(__file__).resolve().parents[1] / "tools/compare_benchmarks.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ComparisonContract(unittest.TestCase):
    def test_timing_does_not_hide_behavior_changes(self):
        baseline = {"status": "SUCCEEDED", "steps": 40, "trace": [{"x": 1.}],
                    "wall_seconds": 1., "control_cycle_p99_us": 20.}
        timing = dict(baseline, wall_seconds=2., control_cycle_p99_us=40.)
        self.assertEqual(module.behavior(baseline), module.behavior(timing))
        for candidate in (dict(baseline, status="FAILED"), dict(baseline, steps=39),
                          dict(baseline, trace=[{"x": 1.1}])):
            self.assertNotEqual(module.behavior(baseline), module.behavior(candidate))

    def test_unknown_units_are_not_silently_treated_as_measurements(self):
        baseline = {'status': 'RUNNING', 'timeout_seconds': 5, 'budget_us': 100}
        self.assertNotEqual(module.behavior(baseline),
                            module.behavior(dict(baseline, timeout_seconds=6)))
        self.assertNotEqual(module.behavior(baseline),
                            module.behavior(dict(baseline, budget_us=200)))

    def test_pair_interval_identifies_constant_slowdown(self):
        lower, upper = module.interval([5.] * 30)
        self.assertEqual((lower, upper), (5., 5.))
        lower, upper = module.interval([-1., 1.] * 15)
        self.assertLess(lower, 0.)
        self.assertGreater(upper, 0.)


if __name__ == "__main__":
    unittest.main()
