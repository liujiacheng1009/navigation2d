"""Logic scope removes communication setup while preserving startup decisions."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('startup', ROOT / 'tools/replay_startup.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayStartup(unittest.TestCase):
    def test_actual_node_keeps_parameters_and_initial_status(self):
        source = (ROOT / 'ros/autonomous_explorer_node.cc').read_text()
        result = module.without_endpoints(source)
        for setup in ['create_publisher<', 'create_subscription<', 'create_wall_timer(']:
            self.assertNotIn(setup, result)
        self.assertIn('ReadExplorerParameters(', result)
        self.assertIn('PublishStatus("WAITING", "waiting for "', result)
        self.assertIn('start_parameter_event_publisher(false)', result)
        self.assertIn('void Tick()', result)
        self.assertNotIn('MakeRosMotionOutput(*this)', result)
        with self.assertRaisesRegex(ValueError, 'constructor'):
            module.without_endpoints(source.replace('Node("sweepnav_autonomous_explorer")', 'Node("other")'))
        with self.assertRaisesRegex(ValueError, 'communication setup'):
            module.without_endpoints(source.replace('create_wall_timer', 'other_timer'))


if __name__ == '__main__':
    unittest.main()
