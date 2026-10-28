"""Reject incomplete or unsafe ROS evidence before comparing trajectories."""
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'ros_results', Path(__file__).resolve().parents[1] / 'tools/check_ros_loop_results.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RosLoopResults(unittest.TestCase):
    case = {'scenario': 'obstacle', 'planner': 'theta_star', 'controller': 'rpp'}

    def archive(self, report=None, commands='x,y\n1.0,2.0\n1.2,2.0\n'):
        result = {'status': 'COMPLETE', 'return_error_m': .04,
                  'planning': 'navigation2d/theta_star+rpp'}
        result.update(report or {})
        data = io.BytesIO()
        with tarfile.open(fileobj=data, mode='w') as archive:
            prefix = 'explorer-ros-loops/obstacle-theta_star-rpp/'
            for name, text in [('result.json', json.dumps(result)), ('commands.csv', commands)]:
                payload = text.encode()
                info = tarfile.TarInfo(prefix + name)
                info.size = len(payload)
                archive.addfile(info, io.BytesIO(payload))
        data.seek(0)
        return tarfile.open(fileobj=data)

    def test_valid_complete_case(self):
        with self.archive() as archive:
            report, points = module.load_case(archive, self.case)
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertEqual(points, [(1., 2.), (1.2, 2.)])

    def test_failed_partial_wrong_algorithm_and_home_error_are_rejected(self):
        for override in [{'status': 'FAILED'}, {'status': 'PARTIAL'},
                         {'planning': 'navigation2d/astar+rpp'}, {'return_error_m': .351},
                         {'return_error_m': float('nan')}, {'return_error_m': float('inf')},
                         {'return_error_m': -.1}]:
            with self.subTest(override=override), self.archive(override) as archive:
                with self.assertRaisesRegex(ValueError, 'result contract'):
                    module.load_case(archive, self.case)

    def test_invalid_trajectory_is_never_filtered(self):
        for commands in ['x,y\n', 'x,y\nnan,2\n', 'x,y\n1,inf\n',
                         'x,y\n1,2\n1,nan\n1.1,2\n']:
            with self.subTest(commands=commands), self.archive(commands=commands) as archive:
                with self.assertRaisesRegex(ValueError, 'trajectory'):
                    module.load_case(archive, self.case)

    def test_manifest_requires_every_case_to_finish(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            path = folder / 'manifest.json'
            for cases in [[], [{'returncode': 0}, {'returncode': 1}],
                          [{'returncode': 0}, {'returncode': -9}]]:
                path.write_text(json.dumps({'cases': cases}))
                with self.assertRaisesRegex(ValueError, 'failed or did not execute'):
                    module.manifest(folder)
            path.write_text(json.dumps({'cases': [{'returncode': 0}]}))
            self.assertEqual(len(module.manifest(folder)['cases']), 1)

    def test_whole_trajectory_distance_preserves_outlier(self):
        first = [(0., 0.), (1., 0.), (2., 0.)]
        second = [(0., 0.), (1., .2), (2., 0.)]
        self.assertAlmostEqual(module.directed_distance(first, second), .2)
        self.assertAlmostEqual(module.directed_distance(second, first), .2)


if __name__ == '__main__':
    unittest.main()
