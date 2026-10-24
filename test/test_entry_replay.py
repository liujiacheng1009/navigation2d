"""Entry replay equality must also prove intended branches were exercised."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

CHECKER = Path(__file__).resolve().parents[1] / 'tools/check_entry_replay.py'


class EntryReplayCheck(unittest.TestCase):
    def run_check(self, baseline_rows, candidate_rows):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            programs = []
            for label, rows in [('baseline', baseline_rows), ('candidate', candidate_rows)]:
                program = root / label
                program.write_text('#!' + sys.executable + '\nprint(' + repr(json.dumps(rows)) + ')\n')
                program.chmod(0o755)
                programs.append(program)
            report = root / 'report.json'
            result = subprocess.run([sys.executable, str(CHECKER), '--baseline', str(programs[0]),
                                     '--candidate', str(programs[1]), '--output', str(report)],
                                    capture_output=True, text=True)
            return result, json.loads(report.read_text())

    @staticmethod
    def rows():
        expected = {'approach': (1, 1, 1, 0, 1), 'unavailable': (0, 1, 1, 1, 0),
                    'probe': (1, 1, 1, 1, 1), 'inside': (0, 0, 0, 0, 0),
                    'cooling': (0, 1, 0, 0, 0)}
        return [dict(case=name, completed=0, publishes=int(name == 'inside'),
                     **dict(zip(['driving', 'pending', 'locked', 'attempts', 'navigation'], values)))
                for name, values in expected.items()]

    def test_equal_branches_pass_and_behavior_difference_fails(self):
        rows = self.rows()
        result, report = self.run_check(rows, rows)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(report['behavior_equal'])
        changed = self.rows()
        changed[0]['commands'] = 1
        result, report = self.run_check(rows, changed)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(report['behavior_equal'])

    def test_equal_outputs_with_missing_branch_do_not_pass(self):
        rows = self.rows()
        rows[0]['driving'] = 0
        result, report = self.run_check(rows, rows)
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(report['behavior_equal'])
        self.assertIn('intended branch not reached', result.stderr)


if __name__ == '__main__':
    unittest.main()
