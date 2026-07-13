import tempfile
import unittest
from pathlib import Path

from sorghum_asset_layout import duplicate_asset_handles, validate_sidecars, validate_unique_asset_handles


class AssetHandleValidationTest(unittest.TestCase):
    def test_reports_every_path_sharing_a_handle(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("first", "second"):
                (root / f"{name}.png.evefilemeta").write_text("asset_handle_: 42\n", encoding="utf-8")

            self.assertEqual(2, len(duplicate_asset_handles(root)[42]))
            with self.assertRaisesRegex(ValueError, "duplicate asset handles"):
                validate_unique_asset_handles(root)

    def test_accepts_unique_handles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "first.png.evefilemeta").write_text("asset_handle_: 1\n", encoding="utf-8")
            (root / "second.png.evefilemeta").write_text("asset_handle_: 2\n", encoding="utf-8")

            validate_unique_asset_handles(root)

    def test_reports_missing_primary_asset_sidecar(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("ManualAssets", "GeneratedAssets"):
                (root / name).mkdir()
                (root / f"{name}.evefoldermeta").write_text("", encoding="utf-8")
            (root / "ManualAssets" / "missing.png").write_bytes(b"png")

            with self.assertRaisesRegex(ValueError, "missing asset sidecar"):
                validate_sidecars(root)


if __name__ == "__main__":
    unittest.main()
