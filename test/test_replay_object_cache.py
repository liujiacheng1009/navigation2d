"""Exercise real compiler cache invalidation without ROS dependencies."""
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('replay_cache',
    Path(__file__).resolve().parents[1] / 'tools/replay_object_cache.py')
CACHE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CACHE)


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class ObjectCacheContract(unittest.TestCase):
    def test_headers_flags_and_object_integrity(self):
        with tempfile.TemporaryDirectory(prefix='replay cache ') as temporary:
            root = Path(temporary)
            source, header, obj = [root / name for name in ['main.cc', 'value.h', 'main.o']]
            source.write_text('#include "value.h"\nint f() { return value; }\n')
            header.write_text('constexpr int value = 1;\n')
            flags = ['-std=c++20', '-O0']
            self.assertFalse(CACHE.compile_cached('c++', flags, source, obj)['cache_hit'])
            self.assertTrue(CACHE.compile_cached('c++', flags, source, obj)['cache_hit'])
            header.write_text('constexpr int value = 2;\n')
            self.assertFalse(CACHE.compile_cached('c++', flags, source, obj)['cache_hit'])
            self.assertFalse(CACHE.compile_cached('c++', flags + ['-O2'], source, obj)['cache_hit'])
            obj.write_bytes(b'corrupt object')
            self.assertFalse(CACHE.compile_cached('c++', flags + ['-O2'], source, obj)['cache_hit'])
            self.assertEqual(obj.read_bytes()[:4], b'\x7fELF')

    def test_new_include_shadow_is_detected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first, second = root / 'first', root / 'second'
            first.mkdir()
            second.mkdir()
            (second / 'value.h').write_text('constexpr int value = 1;\n')
            source, obj = root / 'main.cc', root / 'main.o'
            source.write_text('#include <value.h>\nint f() { return value; }\n')
            flags = ['-std=c++20', '-I', str(first), '-I', str(second)]
            CACHE.compile_cached('c++', flags, source, obj)
            self.assertTrue(CACHE.compile_cached('c++', flags, source, obj)['cache_hit'])
            (first / 'value.h').write_text('constexpr int value = 2;\n')
            self.assertFalse(CACHE.compile_cached('c++', flags, source, obj)['cache_hit'])
