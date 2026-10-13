from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vcpkg_utils import (
    cmake_cache_needs_cgal_reconfigure,
    cmake_vcpkg_configure_args,
    prepare_build_dir_for_toolchain,
    resolve_optional_cgal_vcpkg,
    resolve_vcpkg_root,
    vcpkg_has_cgal,
)


class VcpkgUtilsTest(unittest.TestCase):
    def test_resolve_and_cgal_detection(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "vcpkg"
            toolchain = root / "scripts" / "buildsystems" / "vcpkg.cmake"
            toolchain.parent.mkdir(parents=True)
            toolchain.write_text("# toolchain\n", encoding="utf-8")
            self.assertEqual(resolve_vcpkg_root(root), root.resolve())
            self.assertFalse(vcpkg_has_cgal(root))
            self.assertEqual(resolve_optional_cgal_vcpkg(root), (None, []))

            cgal_config = root / "installed" / "x64-windows" / "share" / "cgal" / "CGALConfig.cmake"
            cgal_config.parent.mkdir(parents=True)
            cgal_config.write_text("# cgal\n", encoding="utf-8")
            self.assertTrue(vcpkg_has_cgal(root))
            self.assertEqual(
                cmake_vcpkg_configure_args(root),
                [f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"],
            )
            optional_root, optional_args = resolve_optional_cgal_vcpkg(root)
            self.assertEqual(optional_root, root.resolve())
            self.assertEqual(optional_args, [f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"])

    def test_optional_cgal_skips_missing_vcpkg(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing = Path(temporary) / "no-vcpkg"
            env = {k: v for k, v in os.environ.items() if k != "VCPKG_ROOT"}
            with mock.patch.dict(os.environ, env, clear=True), mock.patch(
                "vcpkg_utils.DEFAULT_VCPKG_ROOT_CANDIDATES",
                (Path(temporary) / "also-missing",),
            ):
                self.assertIsNone(resolve_vcpkg_root(missing))
                self.assertEqual(resolve_optional_cgal_vcpkg(missing), (None, []))

    def test_reconfigure_detection_and_toolchain_cache_reset(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            vcpkg = root / "vcpkg"
            toolchain = vcpkg / "scripts" / "buildsystems" / "vcpkg.cmake"
            toolchain.parent.mkdir(parents=True)
            toolchain.write_text("# toolchain\n", encoding="utf-8")
            cgal_config = vcpkg / "installed" / "x64-windows" / "share" / "cgal" / "CGALConfig.cmake"
            cgal_config.parent.mkdir(parents=True)
            cgal_config.write_text("# cgal\n", encoding="utf-8")

            build_dir = root / "build"
            build_dir.mkdir()
            cache = build_dir / "CMakeCache.txt"
            cache.write_text("CGAL_DIR:PATH=CGAL_DIR-NOTFOUND\n", encoding="utf-8")
            self.assertTrue(cmake_cache_needs_cgal_reconfigure(build_dir, vcpkg))
            self.assertTrue(prepare_build_dir_for_toolchain(build_dir, toolchain))
            self.assertFalse(cache.exists())


if __name__ == "__main__":
    unittest.main()
