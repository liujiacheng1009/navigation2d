"""Explicit private link inputs must remain frozen during replay construction."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('link', ROOT / 'tools/replay_link.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayLink(unittest.TestCase):
    def test_records_objects_private_archives_and_shared_libraries(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ['node.o', 'application.a', 'typesupport.so.1']:
                (root / name).write_text(name)
            command = [sys.executable, '-c', 'pass', 'node.o', 'application.a', 'typesupport.so.1']
            provenance = module.link_replay(command, root)
            self.assertEqual(set(provenance['input_sha256']), {
                str(root / name) for name in ['node.o', 'application.a', 'typesupport.so.1']})
            self.assertEqual(provenance['command'], command)
            with self.assertRaises(FileNotFoundError):
                module.link_inputs(['missing.a'], root)

    def test_rejects_input_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'application.a').write_text('before')
            command = [sys.executable, '-c',
                       "from pathlib import Path; Path('application.a').write_text('changed')",
                       'application.a']
            with self.assertRaisesRegex(RuntimeError, 'inputs changed'):
                module.link_replay(command, root)


if __name__ == '__main__':
    unittest.main()
