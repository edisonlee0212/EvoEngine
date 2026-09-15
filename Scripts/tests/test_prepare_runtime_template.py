from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import prepare_runtime_template
from prepare_runtime_template import TemplateError, build_template


class RuntimeTemplateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.identity = self.root / "identity.json"
        self.identity.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "platform": "Windows",
                    "architecture": "x64",
                    "configuration": "RelWithDebInfo",
                    "sdk_source_id": "sdk-source",
                    "compiler_id": "MSVC",
                    "compiler_version": "19.51",
                    "with_editor": False,
                    "packages": {},
                    "build_options": {"EVOENGINE_ENABLE_Universe_PACKAGE": True},
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def layout(self, **values: object) -> Path:
        layout = {
            "schema_version": 1,
            "identity_file": str(self.identity.resolve()),
            "files": [],
            "resource_directories": [],
            "packages": [],
            "host": None,
            **values,
        }
        path = self.root / "layout.json"
        path.write_text(json.dumps(layout), encoding="utf-8")
        return path

    def test_missing_required_payload_fails(self) -> None:
        layout = self.layout(
            files=[
                {
                    "source": str((self.root / "missing.dll").resolve()),
                    "destination": "bin/missing.dll",
                }
            ]
        )
        with self.assertRaisesRegex(TemplateError, "Missing required payload"):
            build_template(layout, self.root / "template")

    def test_destination_collision_and_traversal_fail(self) -> None:
        first = self.root / "first.dll"
        second = self.root / "second.dll"
        first.write_bytes(b"first")
        second.write_bytes(b"second")
        with self.subTest("case-insensitive collision"):
            layout = self.layout(
                files=[
                    {"source": str(first.resolve()), "destination": "bin/package.dll"},
                    {"source": str(second.resolve()), "destination": "BIN/PACKAGE.DLL"},
                ]
            )
            with self.assertRaisesRegex(TemplateError, "collision"):
                build_template(layout, self.root / "collision-template")
        with self.subTest("traversal"):
            layout = self.layout(files=[{"source": str(first.resolve()), "destination": "../package.dll"}])
            with self.assertRaisesRegex(TemplateError, "traversing"):
                build_template(layout, self.root / "traversal-template")

    def test_resource_filtering_and_hash_inventory(self) -> None:
        package_library = self.root / "UniversePackage.dll"
        package_manifest = self.root / "UniversePackage.json"
        package_pdb = self.root / "UniversePackage.pdb"
        package_library.write_bytes(b"library")
        package_manifest.write_bytes(b"manifest")
        package_pdb.write_bytes(b"symbols")
        resources = self.root / "resources"
        (resources / "Keep").mkdir(parents=True)
        (resources / "Icons").mkdir()
        (resources / "Keep" / "runtime.slang").write_bytes(b"runtime shader")
        (resources / "Keep" / "skip.txt").write_bytes(b"excluded")
        (resources / "Icons" / "editor.png").write_bytes(b"editor")
        identity = json.loads(self.identity.read_text(encoding="utf-8"))
        identity["packages"] = {"Universe": "universe-source"}
        self.identity.write_text(json.dumps(identity), encoding="utf-8")
        layout = self.layout(
            files=[
                {"source": str(package_library.resolve()), "destination": "Packages/UniversePackage.dll"},
                {"source": str(package_manifest.resolve()), "destination": "Packages/UniversePackage.json"},
                {"source": str(package_pdb.resolve()), "destination": "Packages/UniversePackage.pdb"},
                {"source": "", "destination": "Packages/empty-genex.pdb"},
            ],
            resource_directories=[
                {
                    "source": str(resources.resolve()),
                    "destination": "UniverseResources",
                    "exclude": ["Icons", "skip.txt"],
                }
            ],
            packages=[
                {
                    "name": "Universe",
                    "dependencies": [],
                    "library": "Packages/UniversePackage.dll",
                    "manifest": "Packages/UniversePackage.json",
                    "pdb": "Packages/UniversePackage.pdb",
                    "resources": ["UniverseResources"],
                }
            ],
        )
        output = self.root / "template"
        template = build_template(layout, output)

        paths = {item["path"]: item for item in template["files"]}
        self.assertIn("UniverseResources/Keep/runtime.slang", paths)
        self.assertFalse(any("Icons" in path or path.endswith("skip.txt") for path in paths))
        runtime = output / "UniverseResources" / "Keep" / "runtime.slang"
        self.assertEqual(paths["UniverseResources/Keep/runtime.slang"]["size"], len(b"runtime shader"))
        self.assertEqual(paths["UniverseResources/Keep/runtime.slang"]["sha256"], hashlib.sha256(b"runtime shader").hexdigest())
        self.assertEqual(template["packages"][0]["source_id"], "universe-source")
        self.assertEqual(template["identity"]["packages"], {"Universe": "universe-source"})
        self.assertIsNone(template["host"])
        self.assertEqual(build_template(layout, output)["template_id"], template["template_id"])

    def test_omitted_file_destination_uses_source_name_at_root(self) -> None:
        source = self.root / "native-runtime.dll"
        source.write_bytes(b"runtime")
        template = build_template(
            self.layout(files=[{"source": str(source.resolve())}]), self.root / "template"
        )
        self.assertEqual([item["path"] for item in template["files"]], ["native-runtime.dll"])

    def test_repeated_replacement_publishes_latest_payload(self) -> None:
        source = self.root / "native-runtime.dll"
        source.write_bytes(b"first")
        layout = self.layout(files=[{"source": str(source.resolve())}])
        output = self.root / "template"
        first = build_template(layout, output)

        source.write_bytes(b"second")
        second = build_template(layout, output)
        source.write_bytes(b"third")
        third = build_template(layout, output)

        self.assertNotEqual(first["template_id"], second["template_id"])
        self.assertNotEqual(second["template_id"], third["template_id"])
        self.assertEqual((output / source.name).read_bytes(), b"third")
        self.assertFalse(any(path.name.startswith(".template.old-") for path in self.root.iterdir()))

    def test_publication_failure_restores_previous_template(self) -> None:
        source = self.root / "native-runtime.dll"
        source.write_bytes(b"previous")
        layout = self.layout(files=[{"source": str(source.resolve())}])
        output = self.root / "template"
        previous = build_template(layout, output)
        previous_manifest = (output / "template.json").read_bytes()
        source.write_bytes(b"replacement")
        rename = prepare_runtime_template._rename_with_retry

        def fail_staging_publish(source_path: Path, destination_path: Path) -> None:
            if source_path.name.startswith(".template.tmp-"):
                raise PermissionError("simulated publish failure")
            rename(source_path, destination_path)

        with mock.patch.object(prepare_runtime_template, "_rename_with_retry", side_effect=fail_staging_publish):
            with self.assertRaisesRegex(PermissionError, "simulated publish failure"):
                build_template(layout, output)

        self.assertEqual((output / source.name).read_bytes(), b"previous")
        self.assertEqual((output / "template.json").read_bytes(), previous_manifest)
        self.assertEqual(json.loads(previous_manifest)["template_id"], previous["template_id"])


if __name__ == "__main__":
    unittest.main()
