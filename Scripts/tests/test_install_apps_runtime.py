from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from install_apps import clean_install_dir, cmake_build_option_args, publish_runtime_template, validate_runtime_identity
from prepare_runtime_template import build_template


class InstallRuntimeTemplateTest(unittest.TestCase):
    def test_identity_options_and_immutable_publication(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload = root / "runtime.dll"
            payload.write_bytes(b"runtime")
            identity = {
                "schema_version": 1,
                "platform": "Windows",
                "architecture": "x64",
                "configuration": "Release",
                "sdk_source_id": "sdk-source",
                "compiler_id": "MSVC",
                "compiler_version": "19.51",
                "with_editor": False,
                "packages": {},
                "build_options": {"Z_OPTION": "value", "A_OPTION": True},
                "package_options": {"PACKAGE_OPTION": False},
            }
            identity_path = root / "identity.json"
            identity_path.write_text(json.dumps(identity), encoding="utf-8")
            layout = root / "layout.json"
            layout.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "identity_file": str(identity_path.resolve()),
                        "files": [{"source": str(payload.resolve()), "destination": "bin/runtime.dll"}],
                        "resource_directories": [],
                        "packages": [],
                        "host": None,
                    }
                ),
                encoding="utf-8",
            )
            template_dir = root / "template"
            template = build_template(layout, template_dir)
            editor = {"identity": {**identity, "with_editor": True}}

            validate_runtime_identity(editor, template)
            self.assertEqual(
                cmake_build_option_args(editor),
                ["-DA_OPTION=ON", "-DPACKAGE_OPTION=OFF", "-DZ_OPTION=value"],
            )
            install_dir = root / "install"
            published = publish_runtime_template(template_dir, install_dir)
            self.assertEqual(published.name, template["template_id"])
            self.assertEqual(
                json.loads((published.parent / "current.json").read_text(encoding="utf-8"))["template_id"],
                template["template_id"],
            )
            self.assertEqual(publish_runtime_template(template_dir, install_dir), published)
            (published / "unknown.txt").write_text("unknown", encoding="utf-8")
            with self.assertRaises(SystemExit):
                publish_runtime_template(template_dir, install_dir)

    def test_identity_mismatch_fails(self) -> None:
        editor = {
            "configuration": "Debug",
            "sdk_source_id": "sdk",
            "compiler_id": "MSVC",
            "compiler_version": "1",
            "platform": "Windows",
            "architecture": "x64",
            "packages": {},
            "with_editor": True,
        }
        runtime = {"identity": {**editor, "configuration": "Release", "with_editor": False}}
        with self.assertRaisesRegex(SystemExit, "configuration"):
            validate_runtime_identity(editor, runtime)
        runtime["identity"]["configuration"] = "Debug"
        runtime["identity"]["packages"] = {"Example": "changed"}
        with self.assertRaisesRegex(SystemExit, "package source identities"):
            validate_runtime_identity(editor, runtime)

    def test_clean_install_preserves_runtime_template_versions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            install = root / "out" / "install" / "preset"
            templates = install / "bin" / "RuntimeTemplates"
            templates.mkdir(parents=True)
            (templates / "old-version.txt").write_text("old", encoding="utf-8")
            (install / "bin" / "editor.exe").write_text("editor", encoding="utf-8")
            (install / "stale.txt").write_text("stale", encoding="utf-8")

            clean_install_dir(root, install)

            self.assertEqual((templates / "old-version.txt").read_text(encoding="utf-8"), "old")
            self.assertFalse((install / "bin" / "editor.exe").exists())
            self.assertFalse((install / "stale.txt").exists())


if __name__ == "__main__":
    unittest.main()
