"""The new motion port stays an in-memory sink in generated replays."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('motion', ROOT / 'tools/replay_motion_output.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayMotionOutput(unittest.TestCase):
    def test_captures_all_planar_commands(self):
        source = '''class AutonomousExplorer {
 public:
  void Run() { motion_output_->Send({}); motion_output_ -> Send(velocity); }
};'''
        result, count = module.adapt_motion_output(source)
        self.assertEqual(count, 2)
        self.assertNotIn('motion_output_', result)
        self.assertIn('ReplaySendMotion({});', result)
        self.assertIn('ReplaySendMotion(velocity);', result)
        self.assertIn('replay_.ProbeCommand(message);', result)
        self.assertIn('message.linear.x = value.linear;', result)
        self.assertIn('message.angular.z = value.angular;', result)

    def test_rejects_unknown_port_operation(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            module.adapt_motion_output(
                'class AutonomousExplorer { public: void Run(){motion_output_->SendStop();}};')

    def test_legacy_source_is_byte_identical(self):
        source = 'class AutonomousExplorer { public: void Run(){ command_publisher_->publish(stop); }};'
        self.assertEqual(module.adapt_motion_output(source), (source, 0))

    def test_rejects_uncaptured_call(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            module.adapt_motion_output(
                'class AutonomousExplorer { public: void Run(){motion_output_->Send();}};')


if __name__ == '__main__':
    unittest.main()
