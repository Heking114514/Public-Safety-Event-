#!/usr/bin/env python3

from pathlib import Path
import subprocess
import tempfile
import time
import unittest


WORKSPACE = Path(__file__).resolve().parents[2]
HELPER = WORKSPACE / "scripts" / "build_lock.sh"


class BuildLockTest(unittest.TestCase):
    def holder(self, root: Path, delay: float) -> subprocess.Popen:
        script = f"""
set -euo pipefail
WORKSPACE_ROOT={str(root)!r}
source {str(HELPER)!r}
acquire_workspace_build_lock
printf 'acquired\\n'
sleep {delay}
release_workspace_build_lock
"""
        return subprocess.Popen(
            ["bash", "-c", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def test_builders_are_serialized(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = self.holder(root, 0.4)
            self.assertEqual("acquired\n", first.stdout.readline())
            second = self.holder(root, 0.0)
            try:
                time.sleep(0.1)
                self.assertIsNone(second.poll())
                first_stdout, first_stderr = first.communicate(timeout=3)
                second_stdout, second_stderr = second.communicate(timeout=3)
            finally:
                for process in (first, second):
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=2)
            self.assertEqual("", first_stdout)
            self.assertEqual("", first_stderr)
            self.assertEqual("acquired\n", second_stdout)
            self.assertEqual("", second_stderr)
            self.assertEqual(0, first.returncode)
            self.assertEqual(0, second.returncode)


if __name__ == "__main__":
    unittest.main()
