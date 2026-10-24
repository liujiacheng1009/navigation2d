import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from frozen_core_probe import archive_digests


class LinkInputCaptureTest(unittest.TestCase):
    def test_private_archive_changes_are_visible_when_core_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            core = root / 'libnavigation2d_core.a'
            private = root / 'libnavigation2d_exploration.a'
            core.write_bytes(b'unchanged facade')
            private.write_bytes(b'first implementation')
            command = ['c++', 'driver.o', '-o', 'driver', str(core), private.name, '-pthread']
            before = archive_digests(command, root)
            private.write_bytes(b'changed implementation')
            after = archive_digests(command, root)
            self.assertEqual(before[str(core)], after[str(core)])
            self.assertNotEqual(before[str(private)], after[str(private)])
            self.assertEqual(set(before), {str(core), str(private)})

    def test_missing_link_input_is_not_silently_omitted(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                archive_digests(['c++', 'missing.a'], Path(directory))


if __name__ == '__main__':
    unittest.main()
