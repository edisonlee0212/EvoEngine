import copy
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from write_build_identity import generate, identity


class BuildIdentityTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for name in ("SDK", "Package", "Extern"):
            (self.root / name).mkdir()
            (self.root / name / "source.cpp").write_text("original")
        self.spec = {
            "source_root": str(self.root), "sdk_roots": ["SDK"],
            "packages": [{"name": "Example", "roots": ["Package"],
                          "header": str(self.root / "generated/package.hpp")}],
            "compiler_id": "MSVC", "compiler_version": "19.51", "configuration": "RelWithDebInfo",
            "platform": "Windows", "architecture": "x64", "with_editor": True,
            "build_options": {"EVOENGINE_ENABLE_RUNTIME_PACKAGES": "ON"},
            "package_options": {"EVOENGINE_ENABLE_Example_PACKAGE": "ON"},
            "sdk_header": str(self.root / "generated/sdk.hpp"),
            "metadata_json": str(self.root / "generated/identity.json"),
            "metadata_cmake": str(self.root / "generated/identity.cmake"),
        }

    def test_editor_and_runtime_share_compatibility_id(self):
        runtime = copy.deepcopy(self.spec)
        runtime["with_editor"] = False
        self.assertEqual(identity(self.spec)["sdk_source_id"], identity(runtime)["sdk_source_id"])

    def test_sdk_editor_changes_preserve_runtime_identities(self):
        editor = self.root / "SDK/Editor"
        editor.mkdir()
        source = editor / "inspector.cpp"
        source.write_text("original")
        before = generate(self.spec)
        timestamp = Path(self.spec["sdk_header"]).stat().st_mtime_ns
        source.write_text("changed")
        after = generate(self.spec)
        self.assertEqual(before["sdk_source_id"], after["sdk_source_id"])
        self.assertEqual(before["packages"], after["packages"])
        self.assertNotEqual(before["editor_source_id"], after["editor_source_id"])
        self.assertNotEqual(before["editor_packages"], after["editor_packages"])
        self.assertEqual(timestamp, Path(self.spec["sdk_header"]).stat().st_mtime_ns)

    def test_package_editor_change_invalidates_only_editor_dependents(self):
        for name, dependencies, editor_dependencies in (
                ("Dependent", ["Example"], ["Example"]), ("RuntimeConsumer", ["Example"], [])):
            (self.root / name).mkdir()
            (self.root / name / "source.cpp").write_text("original")
            self.spec["packages"].append({"name": name, "roots": [name], "dependencies": dependencies,
                                          "editor_dependencies": editor_dependencies})
        editor = self.root / "Package/Editor"
        editor.mkdir()
        source = editor / "inspector.cpp"
        source.write_text("original")
        before = identity(self.spec)
        source.write_text("changed")
        after = identity(self.spec)
        self.assertEqual(before["sdk_source_id"], after["sdk_source_id"])
        self.assertEqual(before["editor_source_id"], after["editor_source_id"])
        self.assertEqual(before["packages"], after["packages"])
        for name in ("Example", "Dependent"):
            self.assertNotEqual(before["editor_packages"][name], after["editor_packages"][name])
        self.assertEqual(before["editor_packages"]["RuntimeConsumer"], after["editor_packages"]["RuntimeConsumer"])

    def test_runtime_build_verification_ignores_editor_changes(self):
        self.spec["with_editor"] = False
        editor = self.root / "Package/Editor"
        editor.mkdir()
        source = editor / "inspector.cpp"
        source.write_text("original")
        before = generate(self.spec)
        source.write_text("changed")
        self.assertEqual(before, generate(self.spec, verify=True))

    def test_editor_dependency_must_be_a_runtime_dependency(self):
        self.spec["packages"][0]["editor_dependencies"] = ["Missing"]
        with self.assertRaisesRegex(ValueError, "also be runtime dependencies"):
            identity(self.spec)

    def test_package_change_preserves_sdk_identity_and_header(self):
        before = generate(self.spec)
        timestamp = Path(self.spec["sdk_header"]).stat().st_mtime_ns
        (self.root / "Package/source.cpp").write_text("changed")
        after = generate(self.spec)
        self.assertEqual(before["sdk_source_id"], after["sdk_source_id"])
        self.assertNotEqual(before["packages"], after["packages"])
        self.assertEqual(timestamp, Path(self.spec["sdk_header"]).stat().st_mtime_ns)

    def test_native_inputs_invalidate_identity(self):
        before = identity(self.spec)["sdk_source_id"]
        for field in ("configuration", "compiler_version", "architecture"):
            changed = copy.deepcopy(self.spec)
            changed[field] = "different"
            self.assertNotEqual(before, identity(changed)["sdk_source_id"])
        (self.root / "Extern/source.cpp").write_text("changed")
        self.assertNotEqual(before, identity(self.spec)["sdk_source_id"])

    def test_verify_detects_sources_changed_during_build(self):
        generate(self.spec)
        generate(self.spec, verify=True)
        (self.root / "SDK/source.cpp").write_text("changed")
        with self.assertRaisesRegex(ValueError, "changed during the build"):
            generate(self.spec, verify=True)

    def test_package_selection_is_recorded_separately(self):
        before = identity(self.spec)
        self.spec["package_options"]["EVOENGINE_ENABLE_Example_PACKAGE"] = "OFF"
        after = identity(self.spec)
        self.assertEqual(before["sdk_source_id"], after["sdk_source_id"])
        self.assertNotEqual(before["package_options"], after["package_options"])

    def test_dependency_change_invalidates_transitive_dependents_only(self):
        for name, dependencies in (("Dependent", ["Example"]), ("Transitive", ["Dependent"]), ("Unrelated", [])):
            (self.root / name).mkdir()
            (self.root / name / "api.hpp").write_text("original")
            self.spec["packages"].append({"name": name, "roots": [name], "dependencies": dependencies})
        before = identity(self.spec)
        (self.root / "Package/source.cpp").write_text("changed")
        after = identity(self.spec)
        for name in ("Example", "Dependent", "Transitive"):
            self.assertNotEqual(before["packages"][name], after["packages"][name])
        self.assertEqual(before["packages"]["Unrelated"], after["packages"]["Unrelated"])
        self.assertEqual(before["sdk_source_id"], after["sdk_source_id"])

    def test_invalid_dependency_graph_fails(self):
        self.spec["packages"][0]["dependencies"] = ["Missing"]
        with self.assertRaisesRegex(ValueError, "Missing or cyclic"):
            identity(self.spec)
        self.spec["packages"][0]["dependencies"] = ["Example"]
        with self.assertRaisesRegex(ValueError, "Missing or cyclic"):
            identity(self.spec)


if __name__ == "__main__":
    unittest.main()
