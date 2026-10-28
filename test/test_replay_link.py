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

    def test_binds_only_one_registered_old_or_new_entry(self):
        for name in ['autonomous_explorer_node.cc.o', 'autonomous_explorer_main.cc.o']:
            command = ['c++', '/objects/' + name, 'helper.cc.o', 'library.a']
            self.assertEqual(module.replace_node_entry_object(command, Path('/tmp/probe.o')),
                             ['c++', '/tmp/probe.o', 'helper.cc.o', 'library.a'])
            self.assertEqual(command[1], '/objects/' + name)
        for objects in [[], ['helper.cc.o'], ['autonomous_explorer_node.cc.o',
                                             'autonomous_explorer_main.cc.o']]:
            with self.assertRaisesRegex(ValueError, 'one registered'):
                module.replace_node_entry_object(['c++', *objects], 'probe.o')

    def test_node_compile_flags_follow_producer_target(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            old = root / 'CMakeFiles/sweepnav_autonomous_explorer.dir'
            new = root / 'CMakeFiles/navigation2d_explorer_ros.dir'
            for target in [old, new]:
                target.mkdir(parents=True)
                (target / 'flags.make').write_text('CXX_FLAGS = -std=c++20')
            legacy = 'int main(int argc, char** argv) {}'
            factory = ('#include "navigation2d/ros/exploration/explorer_node.h"\n'
                       'void MakeAutonomousExplorer() {}')
            self.assertEqual(module.node_compile_directory(root, legacy), old)
            self.assertEqual(module.node_compile_directory(root, factory), new)
            (new / 'flags.make').unlink()
            with self.assertRaises(FileNotFoundError):
                module.node_compile_directory(root, factory)
            with self.assertRaisesRegex(ValueError, 'unsupported node main/factory'):
                module.node_compile_directory(root, 'void unrelated() {}')


if __name__ == '__main__':
    unittest.main()
