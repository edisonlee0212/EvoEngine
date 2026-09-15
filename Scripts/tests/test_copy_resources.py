from __future__ import annotations

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "cmake/EvoEngineCopyResources.cmake"


class CopyResourcesTest(unittest.TestCase):
    def test_copies_owned_root_without_name_filters_and_preserves_permissions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            files = {
                "Shaders/main.slang",
                "Shaders/EntitySelectionHighlight.slang",
                "Editor/settings.json",
                "Gizmos/mesh.obj",
                "Icons/app.png",
                ".metadata",
            }
            for name in files:
                path = source / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(name)
            destination = root / "destination"
            destination.mkdir(mode=0o700)
            original_mode = stat.S_IMODE(destination.stat().st_mode)
            subprocess.run([
                "cmake", f"-DSOURCE={source}", f"-DDESTINATION={destination}",
                "-P", str(SCRIPT),
            ], check=True, capture_output=True, text=True)
            actual = {p.relative_to(destination).as_posix() for p in destination.rglob("*") if p.is_file()}
            self.assertEqual(actual, files)
            if os.name != "nt":
                self.assertEqual(stat.S_IMODE(destination.stat().st_mode), original_mode)

    def test_empty_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            subprocess.run([
                "cmake", f"-DSOURCE={source}", f"-DDESTINATION={root / 'destination'}",
                "-P", str(SCRIPT),
            ], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
