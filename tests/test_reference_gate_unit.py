"""Small isolated tests: no live network, no third-party dependency."""
import importlib.util
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

GATE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'v25_render_gate.py'
spec = importlib.util.spec_from_file_location('apogee_v25_gate', GATE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeSat:
    def __init__(self, line1, line2, ignored):
        self.satnum = int(line1[2:7])
        self.label = self.satnum

    def sgp4_tsince(self, minutes):
        return 0, (float(self.satnum), float(minutes), 0.), (0., 1., 0.)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.ctx = tempfile.TemporaryDirectory()
        self.addCleanup(self.ctx.cleanup)
        self.root = Path(self.ctx.name)
        self.tle = self.root / 'SGP4-VER.TLE'
        self.vec = self.root / 'tcppver.out'
        cases = list(range(10000, 10020)) + [10004]  # repeat ID: must be supported
        tle_lines = []
        vector_lines = []
        for n in cases:
            # Mock deliberately lacks a standard checksum; gate must not attempt
            # to correct test TLEs or change their bytes.
            tle_lines += [f'1 {n}U FOO', f'2 {n} FOO']
            vector_lines.append(f'{n} xx')
            for t in range(6):
                vector_lines.append(f'{t} {n}.0 {t}.0 0.0 0.0 1.0 0.0')
        self.tle.write_text('\n'.join(tle_lines)+'\n')
        self.vec.write_text('\n'.join(vector_lines)+'\n')
        self.original = self.tle.read_bytes()
        fake_api = types.ModuleType('sgp4.api')
        fake_api.Satrec = types.SimpleNamespace(twoline2rv=FakeSat)
        fake_api.WGS72 = object()
        fake = types.ModuleType('sgp4')
        fake.api = fake_api
        previous = {name: sys.modules.get(name) for name in ('sgp4', 'sgp4.api')}
        sys.modules['sgp4'] = fake
        sys.modules['sgp4.api'] = fake_api
        def restore():
            for name, value in previous.items():
                if value is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = value
        self.addCleanup(restore)

    def test_expected_vectors_and_repeated_case_identifiers(self):
        report = module.audit(self.tle, self.vec)
        self.assertEqual(report['release_gate'], 'PASS')
        self.assertEqual(report['validated_rows'], 126)
        self.assertEqual(report['reference_cases'], 21)
        self.assertEqual(report['validated_satellites'], 20)
        self.assertEqual(self.original, self.tle.read_bytes())

    def test_numeric_mismatch_fails(self):
        contents = self.vec.read_text().replace('0 10000.0 0.0 0.0', '0 10001.0 0.0 0.0', 1)
        self.vec.write_text(contents)
        report = module.audit(self.tle, self.vec)
        self.assertEqual(report['release_gate'], 'FAIL')
        self.assertGreater(len(report['mismatch_examples']), 0)

    def test_mismatched_case_order_rejected(self):
        contents = self.vec.read_text().replace('10000 xx', '10003 xx', 1)
        self.vec.write_text(contents)
        with self.assertRaisesRegex(ValueError, 'case order'):
            module.audit(self.tle, self.vec)

    def test_missing_data_rejected(self):
        self.vec.write_text('10000 xx\n0 1 2 3 4 5 6\n')
        with self.assertRaisesRegex(ValueError, 'Insufficient'):
            module.audit(self.tle, self.vec)


if __name__ == '__main__':
    unittest.main()
