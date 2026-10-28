#!/usr/bin/env python3
"""Run the real explorer with a synthetic chassis inside an isolated container.

Build explorer_ros_loop_test in the desktop build first. This runner copies that
binary into a separate, read-only-root container with no network, private IPC,
and no devices. It never starts exploration in the existing desktop container.
All case logs, exact binaries, source digests and raw command traces are retained.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import uuid


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-container', default='sweepnav-simulation-desktop-1')
    parser.add_argument('--binary', default='/workspace/artifacts/desktop-build/explorer_ros_loop_test')
    parser.add_argument('--output', type=Path, required=True, help='new evidence directory')
    parser.add_argument('--binary-provenance', type=Path, help='frozen builder manifest, when running the original node')
    parser.add_argument('--scenarios', nargs='+', choices=['return', 'frontier'], default=['return', 'frontier'])
    parser.add_argument('--planners', nargs='+', choices=['dijkstra', 'astar', 'theta_star', 'state_lattice'], default=['dijkstra', 'astar', 'theta_star', 'state_lattice'])
    parser.add_argument('--controllers', nargs='+', choices=['rpp', 'dwa', 'mppi', 'mpc'], default=['rpp', 'dwa', 'mppi', 'mpc'])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    image = subprocess.check_output(['docker', 'inspect', '-f', '{{.Image}}', args.build_container], text=True).strip()
    binary = args.output / 'explorer_ros_loop_test'
    run(['docker', 'cp', args.build_container + ':' + args.binary, binary])
    provenance = json.loads(args.binary_provenance.read_text()) if args.binary_provenance else None
    if provenance and provenance.get('binary_sha256') != digest(binary):
        raise ValueError('copied binary does not match its supplied provenance')
    sources = [root / 'test/explorer_ros_loop_test.cc', root / 'test/explorer_ros_loop_scene.h',
               root / 'ros/autonomous_explorer_node.cc', root / 'config/navigation2d.yaml', Path(__file__).resolve()]
    source_digests = {str(path.relative_to(root)): digest(path) for path in sources}
    manifest = {'image': image, 'binary_sha256': digest(binary), 'sources': source_digests,
                'fixture_repository_revision': subprocess.check_output(['git', '-C', root, 'rev-parse', 'HEAD'], text=True).strip(),
                'binary_provenance': provenance,
                'git_status': subprocess.check_output(['git', '-C', root, 'status', '--short'], text=True),
                'cases': [], 'scope': provenance['scope'] if provenance else 'real ROS factory + synthetic sensors/chassis; no simulator/hardware'}
    name = 'navigation2d-ros-loop-' + uuid.uuid4().hex[:12]
    created = False
    try:
        run(['docker', 'create', '--name', name, '--network', 'none', '--ipc', 'private',
             '--read-only', '--tmpfs', '/tmp:rw,exec,size=256m',
             '--mount', 'type=bind,src=' + str(root) + ',dst=/opt/sweepnav/navigation2d,readonly',
             '--entrypoint', 'bash', image, '-lc', 'sleep infinity'], stdout=subprocess.DEVNULL)
        created = True
        run(['docker', 'start', name], stdout=subprocess.DEVNULL)
        inspected = json.loads(subprocess.check_output(['docker', 'inspect', name], text=True))[0]
        host = inspected['HostConfig']
        if (host['NetworkMode'] != 'none' or host['IpcMode'] != 'private' or
                not host['ReadonlyRootfs'] or host['Devices'] or host['Privileged']):
            raise RuntimeError('test container does not satisfy network/IPC/device isolation')
        (args.output / 'container.json').write_text(json.dumps(inspected, indent=2) + '\n')
        # Docker cp targets the read-only rootfs rather than the private tmpfs.
        # Stream through exec so only the worker's writable /tmp is touched.
        with binary.open('rb') as binary_input:
            run(['docker', 'exec', '-i', name, 'bash', '-lc',
                 'cat > /tmp/explorer_ros_loop_test && chmod 755 /tmp/explorer_ros_loop_test'],
                stdin=binary_input)
        for scenario, planner, controller in itertools.product(args.scenarios, args.planners, args.controllers):
            label = '-'.join([scenario, planner, controller])
            print('Running isolated ROS loop:', label, flush=True)
            command = ['docker', 'exec', '-e', 'ROS_DOMAIN_ID=213', '-e', 'ROS_LOG_DIR=/tmp/ros-log',
                       '-e', 'NAVIGATION2D_ISOLATED_ROS_TEST=1', name, 'bash', '-lc',
                       'source /opt/ros/jazzy/setup.bash && exec timeout --kill-after=5s 115s "$@"',
                       'ros-loop', '/tmp/explorer_ros_loop_test', scenario, planner, controller]
            with (args.output / (label + '.log')).open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=130)
            manifest['cases'].append({'scenario': scenario, 'planner': planner, 'controller': controller,
                                      'returncode': result.returncode})
            (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            print(label, 'PASS' if result.returncode == 0 else 'FAIL', flush=True)
        if source_digests != {str(path.relative_to(root)): digest(path) for path in sources}:
            raise RuntimeError('loop source changed during verification')
    finally:
        (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        if created:
            try:
                directories = [path for path in ['explorer-ros-loops', 'ros-log']
                               if subprocess.run(['docker', 'exec', name, 'test', '-d', '/tmp/' + path]).returncode == 0]
                if directories:
                    with (args.output / 'raw-results.tgz').open('wb') as archive:
                        run(['docker', 'exec', name, 'tar', 'czf', '-', '-C', '/tmp', *directories], stdout=archive)
            finally:
                run(['docker', 'rm', '-f', name], stdout=subprocess.DEVNULL)
    return 0 if manifest['cases'] and all(case['returncode'] == 0 for case in manifest['cases']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
