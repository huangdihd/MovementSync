from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_collision_shapes as VERIFY


class VerifyCollisionShapesTest(unittest.TestCase):
    def test_rejects_windows_absolute_paper_jar_path(self) -> None:
        original_manifest = VERIFY.MANIFEST
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                manifest = json.loads(original_manifest.read_text(encoding="utf-8"))
                manifest["paper_jar"] = r"C:\private\paper-1.21.11.jar"
                candidate = Path(temp_dir) / "manifest.json"
                candidate.write_text(json.dumps(manifest), encoding="utf-8")
                VERIFY.MANIFEST = candidate
                with self.assertRaisesRegex(RuntimeError, "paper_jar portability"):
                    with contextlib.redirect_stdout(io.StringIO()):
                        VERIFY.main()
        finally:
            VERIFY.MANIFEST = original_manifest


if __name__ == "__main__":
    unittest.main()
