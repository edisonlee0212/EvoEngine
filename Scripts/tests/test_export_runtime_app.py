from __future__ import annotations

import hashlib
import ctypes
from ctypes import wintypes
import json
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class ExportFixture:
    def __init__(self, root: Path, *, large_shared: bool = False) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        self.template = root / "模板 runtime"
        self.project = root / "source project Ω"
        self.output = root / "export output 应用"
        self.request = root / "request.json"
        self.template.mkdir()
        (self.project / "Assets").mkdir(parents=True)
        self.runtime_identity = {
            "schema_version": 1,
            "sdk_source_id": "a" * 64,
            "compiler_id": "MSVC",
            "compiler_version": "19.51",
            "configuration": "RelWithDebInfo",
            "platform": "Windows",
            "architecture": "x64",
            "with_editor": False,
            "packages": {"Core": "b" * 64, "Optional": "c" * 64},
        }
        self.editor_identity = {**self.runtime_identity, "with_editor": True}
        self._file("EvoEngineRuntime.exe", b"host")
        self._file("EvoEngine_SDK.dll", b"sdk")
        self._file("DefaultResources/Icons/EvoEngine.png", b"icon")
        self._file("Packages/Core.dll", b"core dll")
        self._file("Packages/Core.evepackage", b"core manifest")
        self._file("Packages/Core.pdb", b"core pdb")
        self._file("CoreResources/runtime.slang", b"core resource")
        self._file("Packages/Optional.dll", b"optional dll")
        self._file("Packages/Optional.evepackage", b"optional manifest")
        self._file("Packages/Optional.pdb", b"optional pdb")
        self._file("OptionalResources/runtime.slang", b"optional resource")
        if large_shared:
            self._file("large-shared.bin", b"x" * (64 * 1024 * 1024))
        files = []
        for path in sorted(self.template.rglob("*")):
            if path.is_file():
                files.append(
                    {
                        "path": path.relative_to(self.template).as_posix(),
                        "size": path.stat().st_size,
                        "sha256": sha256(path),
                    }
                )
        template = {
            "schema_version": 1,
            "template_id": "d" * 64,
            "identity": self.runtime_identity,
            "host": {"executable": "EvoEngineRuntime.exe", "pdb": None},
            "packages": [
                {
                    "name": "Core",
                    "source_id": "b" * 64,
                    "dependencies": [],
                    "library": "Packages/Core.dll",
                    "manifest": "Packages/Core.evepackage",
                    "pdb": "Packages/Core.pdb",
                    "resources": ["CoreResources"],
                },
                {
                    "name": "Optional",
                    "source_id": "c" * 64,
                    "dependencies": ["Core"],
                    "library": "Packages/Optional.dll",
                    "manifest": "Packages/Optional.evepackage",
                    "pdb": "Packages/Optional.pdb",
                    "resources": ["OptionalResources"],
                },
            ],
            "files": files,
        }
        (self.template / "template.json").write_text(json.dumps(template), encoding="utf-8")
        self.project_file = self.project / "EditorProject.eveproj"
        self.project_file.write_text(
            json.dumps({"start_scene_handle": 999, "EditorLayer": {"must_not_ship": True}}), encoding="utf-8"
        )
        self.scene = self.project / "Assets" / "Startup.evescene"
        self.scene.write_bytes(b"independent scene bytes")
        (self.project / "Assets" / "Startup.evescene.evefilemeta").write_bytes(b"scene sidecar")
        (self.project / "Assets" / "Loose.asset.evefilemeta").write_bytes(b"loose sidecar")
        self.write_request()

    def _file(self, relative: str, contents: bytes) -> None:
        path = self.template / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)

    def source_inventory(self) -> list[dict[str, object]]:
        files = [self.project_file, *(path for path in (self.project / "Assets").rglob("*") if path.is_file())]
        return [
            {
                "path": path.relative_to(self.project).as_posix(),
                "size": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in sorted(files)
        ]

    def write_request(self, **updates: object) -> None:
        request = {
            "schema_version": 1,
            "application_name": "My: Runtime 应用",
            "startup_scene_handle": 123456789,
            "project": str(self.project_file),
            "editor_identity": self.editor_identity,
            "loaded_packages": [{"name": "Core", "source_id": "b" * 64}],
            "runtime_config": {
                "show_console": True,
                "window": {"width": 800, "height": 450},
                "graphics": {"use_mesh_shader": False, "use_ray_tracing": False},
            },
            "source_inventory": self.source_inventory(),
            **updates,
        }
        self.request.write_text(json.dumps(request), encoding="utf-8")


class RuntimeExporterBlackBoxTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        executable = os.environ.get("EVOENGINE_RUNTIME_EXPORTER")
        if not executable or not Path(executable).is_file():
            raise unittest.SkipTest("Set EVOENGINE_RUNTIME_EXPORTER to the staged/final exporter executable.")
        cls.executable = Path(executable).resolve()

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="EvoEngine exporter Ω ")
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_long_template_and_output_paths(self) -> None:
        fixture = ExportFixture(self.root / ("nested " * 12).strip() / ("template path " * 8).strip())
        self.assertGreater(len(str(fixture.template.resolve())), 260)
        result = self.run_export(fixture)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((fixture.output / "runtime.yaml").is_file())

    def run_export(self, fixture: ExportFixture) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                str(self.executable),
                "--request",
                str(fixture.request),
                "--template",
                str(fixture.template),
                "--output",
                str(fixture.output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )

    def start_export(self, fixture: ExportFixture) -> subprocess.Popen[str]:
        return subprocess.Popen(
            [
                str(self.executable),
                "--request",
                str(fixture.request),
                "--template",
                str(fixture.template),
                "--output",
                str(fixture.output),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def wait_for_staging(self, fixture: ExportFixture, process: subprocess.Popen[str]) -> None:
        deadline = time.monotonic() + 10
        pattern = f".{fixture.output.name}.tmp-*"
        while time.monotonic() < deadline and process.poll() is None:
            if any(fixture.output.parent.glob(pattern)):
                return
            time.sleep(0.001)
        self.fail("Exporter did not expose a staging directory before completion.")

    def assert_no_staging(self, fixture: ExportFixture) -> None:
        self.assertFalse(any(fixture.output.parent.glob(f".{fixture.output.name}.tmp-*")))

    def test_success_filters_packages_preserves_pdb_and_scene_bytes(self) -> None:
        fixture = ExportFixture(self.root)
        scene_before = fixture.scene.read_bytes()
        result = self.run_export(fixture)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((fixture.output / "My_ Runtime 应用.exe").is_file())
        self.assertTrue((fixture.output / "Packages/Core.dll").is_file())
        self.assertTrue((fixture.output / "Packages/Core.pdb").is_file())
        self.assertTrue((fixture.output / "CoreResources/runtime.slang").is_file())
        self.assertFalse((fixture.output / "Packages/Optional.dll").exists())
        self.assertFalse((fixture.output / "Packages/Optional.pdb").exists())
        self.assertFalse((fixture.output / "OptionalResources").exists())
        self.assertTrue((fixture.output / "DefaultResources/Icons/EvoEngine.png").is_file())
        self.assertEqual((fixture.output / "Project/Assets/Startup.evescene").read_bytes(), scene_before)
        exported_project = json.loads((fixture.output / "Project/My_ Runtime 应用.eveproj").read_text(encoding="utf-8"))
        self.assertEqual(exported_project["start_scene_handle"], 123456789)
        self.assertEqual(set(exported_project), {"application_name", "start_scene_handle", "startup_runtime_packages"})
        runtime = json.loads((fixture.output / "runtime.yaml").read_text(encoding="utf-8"))
        self.assertEqual(runtime["identity"], fixture.runtime_identity)
        self.assertTrue(runtime["runtime_gui_layout_revision"])
        self.assertTrue(runtime["show_console"])
        self.assertEqual(runtime["packages"], [{"name": "Core", "source_id": "b" * 64}])
        report = json.loads((fixture.output / "build-report.json").read_text(encoding="utf-8"))
        self.assertTrue(report["files"])
        self.assertNotIn(str(fixture.project), json.dumps(report))

    def test_exports_receive_distinct_gui_layout_revisions(self) -> None:
        revisions = []
        for name in ("first", "second"):
            fixture = ExportFixture(self.root / name)
            result = self.run_export(fixture)
            self.assertEqual(result.returncode, 0, result.stderr)
            runtime = json.loads((fixture.output / "runtime.yaml").read_text(encoding="utf-8"))
            revisions.append(runtime["runtime_gui_layout_revision"])
        self.assertNotEqual(*revisions)

    def test_gui_layout_revision_cannot_be_supplied_by_request(self) -> None:
        fixture = ExportFixture(self.root)
        fixture.write_request(runtime_config={"runtime_gui_layout_revision": "stale"})
        result = self.run_export(fixture)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(fixture.output.exists())

    def test_template_hash_mismatch_fails(self) -> None:
        fixture = ExportFixture(self.root)
        (fixture.template / "EvoEngine_SDK.dll").write_bytes(b"tampered")
        result = self.run_export(fixture)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(fixture.output.exists())

    def test_package_identity_and_dependency_mismatch_fail(self) -> None:
        with self.subTest("source id"):
            fixture = ExportFixture(self.root / "id")
            fixture.write_request(loaded_packages=[{"name": "Core", "source_id": "f" * 64}])
            self.assertNotEqual(self.run_export(fixture).returncode, 0)
        with self.subTest("dependency closure"):
            fixture = ExportFixture(self.root / "dependency")
            fixture.write_request(loaded_packages=[{"name": "Optional", "source_id": "c" * 64}])
            self.assertNotEqual(self.run_export(fixture).returncode, 0)

    def test_nonempty_destination_is_preserved(self) -> None:
        fixture = ExportFixture(self.root)
        fixture.output.mkdir()
        sentinel = fixture.output / "keep.txt"
        sentinel.write_text("keep", encoding="utf-8")
        result = self.run_export(fixture)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep")

    def test_output_inside_project_with_different_case_is_rejected(self) -> None:
        fixture = ExportFixture(self.root)
        fixture.output = Path(str(fixture.project).swapcase()) / "Export"
        result = self.run_export(fixture)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(fixture.output.exists())

    def test_output_junction_ancestor_is_rejected(self) -> None:
        fixture = ExportFixture(self.root)
        junction = self.root / "redirect"
        subprocess.run(["cmd", "/c", "mklink", "/J", str(junction), str(fixture.project)],
                       check=True, capture_output=True)
        try:
            fixture.output = junction / "Export"
            self.assertNotEqual(self.run_export(fixture).returncode, 0)
            self.assertFalse(fixture.output.exists())
        finally:
            junction.rmdir()

    def test_race_created_destination_is_preserved(self) -> None:
        fixture = ExportFixture(self.root, large_shared=True)
        process = self.start_export(fixture)
        self.wait_for_staging(fixture, process)
        fixture.output.mkdir()
        sentinel = fixture.output / "racer.txt"
        sentinel.write_text("preserve", encoding="utf-8")
        stdout, stderr = process.communicate(timeout=30)
        self.assertNotEqual(process.returncode, 0, stdout + stderr)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve")
        self.assert_no_staging(fixture)

    def test_transient_directory_lock_is_retried(self) -> None:
        fixture = ExportFixture(self.root, large_shared=True)
        process = self.start_export(fixture)
        self.wait_for_staging(fixture, process)
        staging = next(fixture.output.parent.glob(f".{fixture.output.name}.tmp-*"))
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                      ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        kernel.CreateFileW.restype = wintypes.HANDLE
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel.CreateFileW(str(staging), 0x80000000, 3, None, 3, 0x02000000, None)
        self.assertNotEqual(handle, ctypes.c_void_p(-1).value)
        try:
            deadline = time.monotonic() + 10
            while not (staging / "build-report.json").exists() and time.monotonic() < deadline:
                self.assertIsNone(process.poll())
                time.sleep(0.01)
            self.assertTrue((staging / "build-report.json").exists())
            time.sleep(0.3)
            self.assertIsNone(process.poll())
        finally:
            kernel.CloseHandle(handle)
            stdout, stderr = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 0, stdout + stderr)
        self.assertTrue((fixture.output / "build-report.json").is_file())
        self.assert_no_staging(fixture)

    def test_transient_extra_asset_copied_after_snapshot_is_rejected(self) -> None:
        fixture = ExportFixture(self.root, large_shared=True)
        process = self.start_export(fixture)
        self.wait_for_staging(fixture, process)
        transient = fixture.project / "Assets" / "Transient" / "extra.bin"
        transient.parent.mkdir()
        transient.write_bytes(b"must not ship")
        copied = None
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and process.poll() is None:
            staging = next(fixture.output.parent.glob(f".{fixture.output.name}.tmp-*"), None)
            copied = staging / "Project/Assets/Transient/extra.bin" if staging else None
            if copied and copied.exists():
                transient.unlink()
                transient.parent.rmdir()
                break
            time.sleep(0.001)
        stdout, stderr = process.communicate(timeout=30)
        self.assertIsNotNone(copied)
        self.assertNotEqual(process.returncode, 0, stdout + stderr)
        self.assertFalse(fixture.output.exists())
        self.assert_no_staging(fixture)

    def test_source_change_during_copy_fails(self) -> None:
        fixture = ExportFixture(self.root, large_shared=True)
        process = self.start_export(fixture)
        self.wait_for_staging(fixture, process)
        fixture.scene.write_bytes(b"changed during export")
        stdout, stderr = process.communicate(timeout=30)
        self.assertNotEqual(process.returncode, 0, stdout + stderr)
        self.assertFalse(fixture.output.exists())
        self.assert_no_staging(fixture)


if __name__ == "__main__":
    unittest.main()
