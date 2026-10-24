"""Frozen replay reuse checks source, binary and all declared baseline inputs."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'bundle', Path(__file__).resolve().parents[1] / 'tools/replay_baseline_bundle.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FrozenBaseline(unittest.TestCase):
    def test_copy_requires_intact_independent_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, output = root / 'bundle', root / 'output'
            bundle.mkdir()
            output.mkdir()
            (bundle / 'baseline').write_text('frozen executable')
            (bundle / 'baseline.cc').write_text('frozen generated source')
            expected = dict(baseline_ref='frozen-ref', baseline_core='core-sha',
                            baseline_source='source-sha', fixture_sha256={'fixture.h': 'fixture-sha'})
            artifact = dict(binary=module.digest(bundle / 'baseline'),
                            source=module.digest(bundle / 'baseline.cc'))
            (bundle / 'manifest.json').write_text(json.dumps(dict(expected, artifacts={'baseline': artifact})))
            got, provenance = module.reuse_baseline(bundle, output, expected)
            self.assertEqual(got, artifact)
            self.assertEqual(provenance['manifest_sha256'], module.digest(bundle / 'manifest.json'))
            self.assertEqual((output / 'baseline').read_bytes(), (bundle / 'baseline').read_bytes())
            with self.assertRaisesRegex(ValueError, 'separate'):
                module.reuse_baseline(bundle, bundle, expected)
            with self.assertRaisesRegex(ValueError, 'separate'):
                module.reuse_baseline(bundle, bundle / 'generated', expected)
            for key in expected:
                changed = dict(expected, **{key: 'different'})
                with self.assertRaisesRegex(ValueError, key):
                    module.reuse_baseline(bundle, output, changed)
            with self.assertRaisesRegex(ValueError, 'execution_mode'):
                module.reuse_baseline(bundle, output, dict(expected, execution_mode='logic-only'))
            for name in ['baseline', 'baseline.cc']:
                original = (bundle / name).read_bytes()
                (bundle / name).write_text('corrupted')
                with self.assertRaisesRegex(ValueError, 'artifact changed'):
                    module.reuse_baseline(bundle, output, expected)
                (bundle / name).write_bytes(original)


if __name__ == '__main__':
    unittest.main()
