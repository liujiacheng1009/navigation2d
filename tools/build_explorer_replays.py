#!/usr/bin/env python3
"""Build controlled old/new ROS-node replays from the actual executable flags.

Requires the parent ROS build and an independently built frozen baseline core.
Only generated test sources are adapted; production sources are never changed.
Run inside the ROS build environment after building the current explorer target.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def adapt(source, label):
    if source.count('int main(int argc, char** argv)') != 1:
        raise ValueError('unsupported node main; update the explicit replay adapter')
    source = '#include "navigation2d/test/replay/ros_capture.h"\n' + source
    source = source.replace(' private:', ' public:')
    source = source.replace('int main(int argc, char** argv)',
                            'int OriginalMain(int argc, char** argv)')
    start = source.index(' public:', source.index('class AutonomousExplorer')) + len(' public:')
    old = 'entry_probe_attempts_' in source
    entry = 'entry_probe_attempts_' if old else 'entry_.attempts()'
    escapes = 'return_escape_attempts_' if old else 'return_.escapes()'
    navigation = 'navigation_ != nullptr' if old else 'navigation_.has_session()'
    pending = 'pending_entry_goal_' if old else 'entry_.pending()'
    locked = 'locked_entry_mouth_' if old else 'entry_.locked()'
    set_pending = 'pending_entry_goal_ = goal;' if old else 'entry_.SetPending(goal);'
    if 'pose_feedback_' in source:
        observe_pose = 'pose_feedback_.ObserveLocalization(pose);'
    else:
        observe_pose = """
    pose_ = pose;
    const double x = pose.pose.position.x, y = pose.pose.position.y;
    if (trajectory_.empty() ||
        std::hypot(x - trajectory_.back().first, y - trajectory_.back().second) > .05) {
      if (!trajectory_.empty()) {
        total_distance_m_ += std::hypot(x - trajectory_.back().first, y - trajectory_.back().second);
      }
      trajectory_.emplace_back(x, y);
    }
"""
    phase = 'mission_.phase()' if 'mission_.phase()' in source else 'phase_'
    finished = 'mission_.finished()' if 'mission_.finished()' in source else 'finished_'
    capture = f'''
  void ReplayObservePose(const geometry_msgs::msg::PoseStamped& pose) {{ {observe_pose} }}
  int ReplayPhase() const {{ return static_cast<int>({phase}); }}
  bool ReplayFinished() const {{ return {finished}; }}
  void ReplaySetPendingEntry(const Goal& goal) {{ {set_pending} }}
  bool ReplayPendingEntry() const {{ return {pending}.has_value(); }}
  bool ReplayLockedEntry() const {{ return {locked}.has_value(); }}
  navigation2d::test::ReplayOutputs replay_;
  rclcpp::Time now() const {{ return replay_.probe_time_; }}
  int ReplayEntryAttempts() const {{ return {entry}; }}
  int ReplayReturnEscapes() const {{ return {escapes}; }}
  bool ReplayHasNavigation() const {{ return {navigation}; }}
'''
    source = source[:start] + capture + source[start:]
    source, commands = re.subn(r'command_publisher_->publish\(([^;]+)\);',
                               r'replay_.ProbeCommand(\1);', source)
    source, publications = re.subn(r'\w+_publisher_->publish\(([^;]+)\);',
                                   r'replay_.ProbePublish(\1);', source)
    if not commands or not publications or re.search(r'(?:->|\.)\s*publish\s*\(', source):
        raise ValueError('publisher adaptation incomplete; refusing a replay with real outputs')
    source = source.replace('/tmp/sweepnav-navigation-map-', f'/tmp/replay-{label}-map-')
    source += '''
#include "navigation2d/test/replay/explorer_scenario.h"
#include "navigation2d/test/replay/entry_probe_scenario.h"
int main(int argc, char** argv) {
  if (argc > 1 && (std::string(argv[1]) == "entry" ||
                   std::string(argv[1]) == "entry-benchmark")) {
    return navigation2d::test::RunEntryProbeReplay<AutonomousExplorer>(argc, argv);
  }
  return navigation2d::test::RunExplorerReplay<AutonomousExplorer>(argc, argv);
}
'''
    return source


def main():
    from replay_object_cache import compile_cached
    from replay_baseline_bundle import reuse_baseline

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True, help='parent online_mapping build')
    parser.add_argument('--baseline-core', type=Path, required=True,
                        help='frozen libnavigation2d_core.a, not the candidate library')
    parser.add_argument('--baseline-bundle', type=Path,
                        help='verified frozen baseline/manifest/source bundle; do not relink it')
    parser.add_argument('--baseline-source', type=Path,
                        help='frozen node source when baseline Git metadata is unavailable')
    parser.add_argument('--baseline-ref', default='a947deba48765ee4dc57168ebcf1be5e2c49e205')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--no-object-cache', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = args.build.resolve()
    output = args.output.resolve()
    baseline_core = args.baseline_core.resolve(strict=True)
    current_core = build / 'navigation2d_build/libnavigation2d_core.a'
    if baseline_core == current_core.resolve():
        parser.error('baseline and candidate core must be independently frozen builds')
    output.mkdir(parents=True, exist_ok=True)
    target = build / 'CMakeFiles/sweepnav_autonomous_explorer.dir'
    flags = []
    for line in (target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    link_template = shlex.split((target / 'link.txt').read_text())
    if args.baseline_source:
        baseline = args.baseline_source.read_text()
    else:
        try:
            baseline = subprocess.check_output(
                ['git', '-C', str(root), 'show', f'{args.baseline_ref}:ros/autonomous_explorer_node.cc'],
                text=True, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError:
            parser.error('baseline Git source unavailable; provide --baseline-source from its frozen tree')
    fixtures = {str(p.relative_to(root)): digest(p) for p in (root / 'test/replay').glob('*.h')}
    manifest = {'fixture_sha256': fixtures, 'builder_sha256': digest(Path(__file__)),
                'cache_helper_sha256': digest(Path(__file__).with_name('replay_object_cache.py')),
                'bundle_helper_sha256': digest(Path(__file__).with_name('replay_baseline_bundle.py')),
                'baseline_ref': args.baseline_ref, 'baseline_core': digest(baseline_core),
                'candidate_core': digest(current_core),
                'baseline_source': hashlib.sha256(baseline.encode()).hexdigest(), 'artifacts': {}}
    for label, source in [('baseline', baseline),
                          ('candidate', (root / 'ros/autonomous_explorer_node.cc').read_text())]:
        if label == 'baseline' and args.baseline_bundle:
            artifact, provenance = reuse_baseline(args.baseline_bundle, output, manifest)
            manifest['artifacts'][label] = artifact
            manifest['baseline_bundle'] = provenance
            continue
        cpp, obj, executable = [output / f'{label}{suffix}' for suffix in ['.cc', '.o', '']]
        cpp.write_text(adapt(source, label))
        compilation = compile_cached(link_template[0], flags, cpp, obj, args.no_object_cache)
        link = link_template.copy()
        link[link.index('-o') + 1] = str(executable)
        objects = [i for i, value in enumerate(link) if value.endswith('autonomous_explorer_node.cc.o')]
        if len(objects) != 1:
            raise ValueError('unexpected explorer target object list')
        link[objects[0]] = str(obj)
        if label == 'baseline':
            for i, value in enumerate(link):
                if value.endswith('libnavigation2d_core.a'):
                    link[i] = str(baseline_core)
        subprocess.run(link, cwd=build, check=True)
        manifest['artifacts'][label] = {'source': digest(cpp), 'binary': digest(executable),
                                        'compilation': compilation}
    if fixtures != {str(p.relative_to(root)): digest(p) for p in (root / 'test/replay').glob('*.h')}:
        raise RuntimeError('fixture changed while compiling; discard and rebuild both replays')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
