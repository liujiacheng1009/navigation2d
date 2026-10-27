"""The new motion port stays an in-memory sink in generated replays."""
import importlib.util
from pathlib import Path
import unittest
import shutil
import subprocess
import tempfile

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

    @unittest.skipUnless(shutil.which('c++'), 'C++ compiler required for generated-port contract')
    def test_core_motion_reference_uses_capture_sink_without_ros(self):
        source = r'''#include <memory>
#include <stdexcept>
#include "navigation2d/application/exploration/motion_output.h"
namespace geometry_msgs::msg {
struct Twist { struct Axis { double x = 0., z = 0.; }; Axis linear, angular; };
}
struct Outputs {
  int commands = 0;
  void ProbeCommand(const geometry_msgs::msg::Twist& message) {
    if (message.linear.x != .3 || message.angular.z != -.4) throw std::runtime_error("command");
    ++commands;
  }
};
struct Mission {
  void Tick(navigation2d::exploration::MotionOutput& output) { output.Send({.3, -.4}); }
};
class AutonomousExplorer {
 public:
  void Run() { mission_.Tick(*motion_output_); }
  int Count() const { return replay_.commands; }
 private:
  Mission mission_;
  std::unique_ptr<navigation2d::exploration::MotionOutput> motion_output_;
  Outputs replay_;
};
int main() { AutonomousExplorer node; node.Run(); return node.Count() == 1 ? 0 : 1; }
'''
        generated, count = module.adapt_motion_output(source)
        self.assertEqual(count, 1)
        self.assertNotIn('*motion_output_', generated)
        self.assertIn('mission_.Tick(replay_motion_sink_);', generated)
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'capture.cc'
            binary = Path(directory) / 'capture'
            cpp.write_text(generated)
            subprocess.run(['c++', '-std=c++20', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(ROOT.parent), str(cpp), '-o', str(binary)],
                           check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True)

    def test_rejects_unknown_port_operation(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            module.adapt_motion_output(
                'class AutonomousExplorer { public: void Run(){motion_output_->SendStop();}};')

    def test_prunes_only_unused_logic_motion_object(self):
        command = ['c++', 'node.o', '/build/ros/motion_output.cc.o', 'application.a']
        self.assertEqual(module.without_unused_motion_object(command, '', True),
                         ['c++', 'node.o', 'application.a'])
        self.assertEqual(module.without_unused_motion_object(command, '', False), command)
        with self.assertRaisesRegex(ValueError, 'still constructs'):
            module.without_unused_motion_object(command, 'MakeRosMotionOutput(node)', True)
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            module.without_unused_motion_object(command + ['/other/motion_output.cc.o'], '', True)

    def test_subscription_object_is_pruned_only_in_logic_scope(self):
        command = ['c++', 'node.o', '/build/explorer_subscriptions.cc.o', 'core.a']
        self.assertEqual(module.without_unused_motion_object(command, '', True),
                         ['c++', 'node.o', 'core.a'])
        self.assertEqual(module.without_unused_motion_object(command, '', False), command)
        with self.assertRaisesRegex(ValueError, 'still constructs'):
            module.without_unused_motion_object(command, 'MakeExplorerSubscriptions(node)', True)
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            module.without_unused_motion_object(
                command + ['/other/explorer_subscriptions.cc.o'], '', True)

    def test_legacy_source_is_byte_identical(self):
        source = 'class AutonomousExplorer { public: void Run(){ command_publisher_->publish(stop); }};'
        self.assertEqual(module.adapt_motion_output(source), (source, 0))

    def test_rejects_uncaptured_call(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            module.adapt_motion_output(
                'class AutonomousExplorer { public: void Run(){motion_output_->Send();}};')


if __name__ == '__main__':
    unittest.main()
