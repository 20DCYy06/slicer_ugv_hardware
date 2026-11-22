"""Offline config-to-header checks; never build or connect to hardware."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('firmware_build', ROOT / 'build.py')
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class BuildLimitTests(unittest.TestCase):
    def generate_at(self, permille):
        cfg = yaml.safe_load((ROOT / 'config.yaml').read_text())
        cfg['hardware']['pwm_limit_permille'] = permille
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'config.yaml').write_text(yaml.safe_dump(cfg))
            previous = build.HERE
            try:
                build.HERE = root
                _, out = build.generate()
                return (out / 'generated.h').read_text()
            finally:
                build.HERE = previous

    def test_requested_ceiling_generates_actual_ticks(self):
        header = self.generate_at(982)
        self.assertIn('#define PWM_CAP 1571u\n', header)
        self.assertIn('#define PWM_LIMIT_PERMILLE 982u\n', header)
        self.assertIn('#define TEST_PWM_CAP 1200u\n', header)
        self.assertIn('#define WATCHDOG_MS 500u\n', header)
        self.assertIn('#define STALL_TIMEOUT_MS 5000u\n', header)

    def test_physical_duty_bound_and_rollback(self):
        self.assertIn('#define PWM_CAP 1280u\n', self.generate_at(800))
        for value in (0, 983):
            with self.subTest(permille=value), self.assertRaises(AssertionError):
                self.generate_at(value)


if __name__ == '__main__':
    unittest.main()
