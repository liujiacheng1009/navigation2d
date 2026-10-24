"""Replay output isolation and frozen calibration CLI checks."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('builder', ROOT / 'tools/build_explorer_replays.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


class ReplayContract(unittest.TestCase):
    def test_actual_node_has_no_remaining_output_publish_calls(self):
        source = (ROOT / 'ros/autonomous_explorer_node.cc').read_text()
        adapted = builder.adapt(source, 'fixture')
        self.assertNotRegex(adapted, r'(?:->|\.)\s*publish\s*\(')
        self.assertIn('replay_.ProbeCommand(', adapted)
        self.assertIn('RunExplorerReplay<AutonomousExplorer>', adapted)
        for output in ['other->publish(command);', 'other.publish(command);']:
            with self.assertRaisesRegex(ValueError, 'real outputs'):
                builder.adapt(source + output, 'fixture')

    def test_scope_metrics_and_explicit_input_are_frozen(self):
        with tempfile.TemporaryDirectory() as temporary:
            work = Path(temporary)
            program = work / 'fixture'
            program.write_text('#!' + sys.executable + '\nprint(\'{"trace":[1],"tick_p50_us":1}\')\n')
            program.chmod(0o755)
            cases = work / 'cases.json'
            cases.write_text('[[]]')
            configuration = work / 'config.yaml'
            configuration.write_text('version: 1\n')
            report = work / 'aa.json'
            command = [sys.executable, str(ROOT / 'tools/compare_benchmarks.py'),
                       '--baseline', str(program), '--cases', str(cases),
                       '--metrics', 'tick_p50_us', '--scope', 'controlled replay',
                       '--input-files', str(configuration), '--output', str(report)]
            subprocess.run(command + ['--calibrate'], check=True, capture_output=True)
            data = json.loads(report.read_text())
            self.assertEqual(data['metric_names'], ['tick_p50_us'])
            self.assertIn(str(configuration.resolve()), data['input_sha256'])
            compare = command + ['--candidate', str(program), '--calibration', str(report)]
            for flag, value in [('--scope', 'other scope'), ('--metrics', 'tick_p99_us')]:
                changed = compare.copy()
                changed[changed.index(flag) + 1] = value
                result = subprocess.run(changed, capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn('scope differ', result.stderr)
            configuration.write_text('version: 2\n')
            result = subprocess.run(compare, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('differ from calibration', result.stderr)


if __name__ == '__main__':
    unittest.main()
