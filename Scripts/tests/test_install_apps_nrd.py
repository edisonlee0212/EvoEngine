from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from install_apps import ensure_nrd_source, nrd_source_for_install


class InstallNrdSourceTest(unittest.TestCase):
    def test_preset_cache_and_command_line_select_effective_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            build = root / "build"
            build.mkdir()
            preset = {
                "cacheVariables": {
                    "EVOENGINE_ENABLE_NRD": "ON",
                    "EVOENGINE_NRD_SOURCE_DIR": "${sourceDir}/Extern/3rdParty/NRD/NRD",
                }
            }
            default = root / "Extern" / "3rdParty" / "NRD" / "NRD"
            self.assertEqual(nrd_source_for_install(root, preset, build, [], False), default)
            self.assertIsNone(nrd_source_for_install(root, preset, build, ["-DEVOENGINE_ENABLE_NRD=OFF"], False))
            custom = root / "my-nrd"
            self.assertEqual(
                nrd_source_for_install(root, preset, build, [f"-DEVOENGINE_NRD_SOURCE_DIR={custom}"], False), custom
            )
            (build / "CMakeCache.txt").write_text(
                f"EVOENGINE_ENABLE_NRD:BOOL=ON\nEVOENGINE_NRD_SOURCE_DIR:PATH={custom}\n", encoding="utf-8"
            )
            self.assertEqual(nrd_source_for_install(root, preset, build, [], True), custom)

    def test_initializes_missing_submodule_once_and_preserves_existing_checkout(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            source = root / "Extern" / "3rdParty" / "NRD" / "NRD"

            def fake_update(_: str, command: list[str]) -> None:
                self.assertEqual(command, ["git", "submodule", "update", "--init", "--", "Extern/3rdParty/NRD/NRD"])
                (source / "Include").mkdir(parents=True)
                (source / "Include" / "NRD.h").write_text("header", encoding="utf-8")

            with patch("install_apps.run_step", side_effect=fake_update) as update:
                ensure_nrd_source(root, source, source)
                ensure_nrd_source(root, source, source)
            self.assertEqual(update.call_count, 1)
            self.assertTrue((source / "Include" / "NRD.h").is_file())
            custom = root / "custom"
            (custom / "Include").mkdir(parents=True)
            (custom / "Include" / "NRD.h").write_text("custom", encoding="utf-8")
            with patch("install_apps.run_step") as update:
                ensure_nrd_source(root, custom, source)
            update.assert_not_called()

    def test_initializes_empty_submodule_directory_and_rejects_custom_missing_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            source = root / "Extern" / "3rdParty" / "NRD" / "NRD"
            source.mkdir(parents=True)
            with patch("install_apps.run_step") as update, self.assertRaisesRegex(SystemExit, "incomplete"):
                ensure_nrd_source(root, source, source)
            update.assert_called_once()
            with self.assertRaisesRegex(SystemExit, "missing"):
                ensure_nrd_source(root, root / "custom", source)

    def test_disabled_or_empty_source_does_not_initialize(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            self.assertIsNone(nrd_source_for_install(root, {"cacheVariables": {}}, root, [], False))
            with self.assertRaisesRegex(SystemExit, "empty"):
                nrd_source_for_install(root, {"cacheVariables": {"EVOENGINE_ENABLE_NRD": "ON"}}, root, [], False)


if __name__ == "__main__":
    unittest.main()
