"""Baseline headers and links must remain independent from candidate adapters."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'headers', Path(__file__).resolve().parents[1] / 'tools/replay_baseline_headers.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BaselineInputs(unittest.TestCase):
    def test_complete_tree_and_snapshot_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / 'navigation2d/application/navigation_system.h'
            header.parent.mkdir(parents=True)
            header.write_text('frozen declaration')
            source = root / 'navigation2d/ros/node.cc'
            selected, hashes = module.prepare(source, None)
            self.assertEqual(selected, root)
            self.assertEqual(len(hashes), 1)
            header.write_text('changed')
            self.assertNotEqual(module.snapshot(root), hashes)
            with self.assertRaisesRegex(ValueError, 'complete frozen'):
                module.prepare(None, root / 'missing')

    def test_strip_candidate_objects_and_bind_exactly_one_core(self):
        command = ['c++', 'baseline.o', 'candidate_adapter.cc.o', '-o', 'baseline',
                   '/current/libnavigation2d_core.a', '-lrclcpp']
        self.assertEqual(module.baseline_link(command, 'baseline.o', '/frozen/core.a'),
                         ['c++', 'baseline.o', '-o', 'baseline', '/frozen/core.a', '-lrclcpp'])
        with self.assertRaisesRegex(ValueError, 'link contract'):
            module.baseline_link(['c++', 'baseline.o'], 'baseline.o', '/frozen/core.a')


if __name__ == '__main__':
    unittest.main()
