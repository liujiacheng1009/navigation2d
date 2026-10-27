"""Input ownership migrations preserve a single fixture interface for both binaries."""
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from build_explorer_replays import adapt


class ReplayObservations(unittest.TestCase):
    def test_mission_owner_without_inline_finished_query(self):
        source = ('class AutonomousExplorer { private: '
                  'navigation2d::exploration::ExplorationMission mission_; '
                  'void Send(){command_publisher_->publish(stop); status_publisher_->publish(status);} '
                  '}; int main(int argc, char** argv) {}')
        generated = adapt(source, 'unit')
        self.assertIn('bool ReplayFinished() const { return mission_.finished(); }', generated)

    def test_ownership_variants(self):
        for field, call in [('observations_', 'observations_.ObserveLocalization(pose);'),
                            ('pose_feedback_', 'pose_feedback_.ObserveLocalization(pose);'),
                            ('pose_', 'pose_ = pose;')]:
            source = ('class AutonomousExplorer { private: int ' + field + '; '
                      'void Send(){command_publisher_->publish(stop); status_publisher_->publish(status);} '
                      '}; int main(int argc, char** argv) {}')
            generated = adapt(source, 'unit')
            self.assertIn(call, generated)
            for method in ['ReplayObserveMap', 'ReplayObserveScan', 'ReplayObserveVelocity']:
                self.assertIn(method, generated)
            if field == 'observations_':
                self.assertIn('observations_.ObserveMap(value);', generated)
                self.assertIn('observations_.ObserveScan(value);', generated)
                self.assertIn('observations_.ObserveVelocity(value, stamp);', generated)
            else:
                self.assertIn('map_ = value;', generated)
                self.assertIn('scan_ = value;', generated)
                self.assertIn('velocity_ = value; velocity_stamp_ = stamp;', generated)


if __name__ == '__main__':
    unittest.main()
