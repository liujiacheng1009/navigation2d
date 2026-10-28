"""Generated replay capture consumes debug messages without touching ROS owners."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('debug_capture', ROOT / 'tools/replay_debug_output.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayDebugOutput(unittest.TestCase):
    def test_legacy_source_is_unchanged(self):
        source = 'class AutonomousExplorer { public: void Run() {} };'
        self.assertEqual(module.adapt_debug_output(source), (source, 0))

    def test_unknown_port_operation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            module.adapt_debug_output('class AutonomousExplorer { public: void Run(){debug_output_->Flush();} };')

    def test_logic_link_removes_only_unused_debug_endpoint(self):
        spec = importlib.util.spec_from_file_location('motion_link', ROOT / 'tools/replay_motion_output.py')
        link = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(link)
        command = ['c++', 'node.o', '/build/debug_output.cc.o', '/build/debug_reporter.cc.o', 'core.a']
        self.assertEqual(link.without_unused_motion_object(command, '', True),
                         ['c++', 'node.o', '/build/debug_reporter.cc.o', 'core.a'])
        self.assertEqual(link.without_unused_motion_object(command, '', False), command)
        with self.assertRaisesRegex(ValueError, 'still constructs'):
            link.without_unused_motion_object(command, 'MakeRosDebugOutput(node)', True)
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            link.without_unused_motion_object(command + ['/other/debug_output.cc.o'], '', True)

    @unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
    def test_compiled_reporter_reference_captures_every_message_kind(self):
        source = r'''#include "navigation2d/ros/exploration/debug_output.h"
#include <stdexcept>
using namespace navigation2d::exploration_ros;
struct Outputs {
  int count = 0;
  template<class Message> void ProbePublish(const Message& message) {
    if (message.value != ++count) throw std::runtime_error("message sequence");
  }
};
void Report(DebugReportSink& sink) {
  sink.SendPath(DebugPathChannel::kGlobal, nav_msgs::msg::Path{1});
  sink.SendPath(DebugPathChannel::kLocal, nav_msgs::msg::Path{2});
  sink.SendCarrot(geometry_msgs::msg::PoseStamped{3});
  sink.SendPath(DebugPathChannel::kTour, nav_msgs::msg::Path{4});
  sink.SendFrontiers(geometry_msgs::msg::PoseArray{5});
  sink.SendFrontierDebug(std_msgs::msg::String{6});
  sink.SendMap(DebugMapChannel::kPlanningInput, nav_msgs::msg::OccupancyGrid{7});
  sink.SendMap(DebugMapChannel::kInflated, nav_msgs::msg::OccupancyGrid{8});
}
class AutonomousExplorer {
 public:
  void Run() { Report(*debug_output_); }
  int Count() const { return replay_.count; }
 private:
  std::unique_ptr<DebugReportSink> debug_output_;
  Outputs replay_;
};
int main() { AutonomousExplorer node; node.Run(); return node.Count() == 8 ? 0 : 1; }
'''
        generated, count = module.adapt_debug_output(source)
        self.assertEqual(count, 1)
        self.assertNotIn('*debug_output_', generated)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for package, header, name in [('geometry_msgs', 'pose_array', 'PoseArray'),
                                           ('geometry_msgs', 'pose_stamped', 'PoseStamped'),
                                           ('nav_msgs', 'occupancy_grid', 'OccupancyGrid'),
                                           ('nav_msgs', 'path', 'Path'),
                                           ('std_msgs', 'string', 'String')]:
                target = root / package / 'msg' / (header + '.hpp')
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#pragma once\nnamespace ' + package + '::msg { struct ' + name + ' { int value; }; }\n')
            cpp, binary = root / 'capture.cc', root / 'capture'
            cpp.write_text(generated)
            subprocess.run(['c++', '-std=c++20', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(root), '-I' + str(ROOT.parent), str(cpp), '-o', str(binary)],
                           check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
