#!/usr/bin/env python3
"""Bind the synthetic ROS loop fixture to the original, unmodified explorer.

Only assembly changes: the original node has no public factory or algorithm
registry. The default theta_star/rpp case uses its original constructor. Sensor,
chassis, timers, assertions and output checks stay in the same fixture sources.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

from replay_baseline_bundle import digest
from replay_baseline_headers import baseline_link, prepare, snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True, help='parent ROS build')
    parser.add_argument('--baseline-core', type=Path, required=True)
    parser.add_argument('--baseline-include-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    headers, original_headers = prepare(None, args.baseline_include_root)
    frozen_node = headers / 'navigation2d/ros/autonomous_explorer_node.cc'
    source = root / 'test/explorer_ros_loop_test.cc'
    fixture = root / 'test/explorer_ros_loop_scene.h'
    before = {str(path): digest(path) for path in [source, fixture, root / 'test/ros_loop_room.h', frozen_node, args.baseline_core]}
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    adapted = source.read_text()
    include = '#include "navigation2d/ros/exploration/explorer_node.h"'
    assembly = ('auto explorer = navigation2d::exploration_ros::MakeAutonomousExplorer(\n'
                '        navigation2d::AlgorithmRegistry::Builtins(), options);')
    if adapted.count(include) != 1 or adapted.count(assembly) != 1:
        raise ValueError('ROS loop assembly changed; review the frozen binding')
    adapted = adapted.replace(include, '#include "rclcpp/rclcpp.hpp"\n'
                              'std::shared_ptr<rclcpp::Node> MakeFrozenLoopExplorer();')
    adapted = adapted.replace(assembly, 'if (planner != "theta_star" || controller != "rpp") {\n'
                              '      throw std::runtime_error("frozen node supports default selection only");\n'
                              '    }\n    auto explorer = MakeFrozenLoopExplorer();')
    main_source = args.output / 'loop-main.cc'
    main_source.write_text(adapted)
    bridge = args.output / 'frozen-node.cc'
    bridge.write_text('#define main Navigation2dFrozenCliMain\n#include ' +
                      json.dumps(str(frozen_node)) + '\n#undef main\n'
                      'std::shared_ptr<rclcpp::Node> MakeFrozenLoopExplorer() {\n'
                      '  return std::make_shared<AutonomousExplorer>();\n}\n')
    target = args.build / 'CMakeFiles/explorer_ros_loop_test.dir'
    flags = []
    for line in (target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    command = shlex.split((target / 'link.txt').read_text())
    objects = []
    for label, unit in [('main', main_source), ('node', bridge)]:
        obj = args.output / (label + '.o')
        dep = args.output / (label + '.d')
        # The original production node was built without -Werror. Keep the
        # fixture's strict flags while preserving that original producer policy.
        unit_flags = [flag for flag in flags if label != 'node' or flag != '-Werror']
        compile_command = [command[0], '-I' + str(headers), *unit_flags, '-MD', '-MF',
                           str(dep), '-c', str(unit), '-o', str(obj)]
        subprocess.run(compile_command, check=True)
        dependencies = shlex.split(dep.read_text().replace('\\\n', ' ').split(':', 1)[1])
        leaked = [str(path) for path in map(lambda value: Path(value).resolve(), dependencies)
                  if root in path.parents and root / 'test' not in path.parents]
        if leaked:
            raise ValueError('candidate production headers entered frozen node: ' + repr(leaked))
        objects.append(obj)
    positions = [index for index, value in enumerate(command) if value.endswith('.cc.o')]
    if len(positions) != 1:
        raise ValueError('expected a single original fixture translation unit')
    command[positions[0]] = str(objects[0])
    binary = args.output / 'explorer_ros_loop_baseline'
    command[command.index('-o') + 1] = str(binary)
    command = baseline_link(command, objects[0], args.baseline_core.resolve())
    command.insert(command.index(str(objects[0])) + 1, str(objects[1]))
    subprocess.run(command, cwd=args.build, check=True)
    if original_headers != snapshot(headers) or before != {path: digest(Path(path)) for path in before}:
        raise ValueError('frozen inputs or fixture changed during compilation')
    (args.output / 'manifest.json').write_text(json.dumps({
        'scope': 'original real ROS node, default theta_star/rpp only', 'inputs_sha256': before,
        'frozen_headers_sha256': original_headers, 'binary_sha256': digest(binary),
        'adapted_main_sha256': digest(main_source), 'assembly_bridge_sha256': digest(bridge),
        'link_command': command}, indent=2) + '\n')
    print(binary)


if __name__ == '__main__':
    main()
