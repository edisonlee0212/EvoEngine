"""Configuration/API-routing tests only; no GPU or scene loading."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from search_prefab_field_illumination import build_cases, configure_lighting, resolve_lighting


class HdrFieldLightingTest(unittest.TestCase):
    def test_legacy_default_is_skydome(self):
        lighting = resolve_lighting({}, Path.cwd())
        self.assertEqual(lighting["type"], "skydome")
        self.assertEqual(lighting["sun_intensity"], 1)

    def test_hdr_selects_scene_api_with_no_extra_sun(self):
        with tempfile.TemporaryDirectory() as folder:
            hdr = Path(folder) / "sky.hdr"
            hdr.write_bytes(b"#?RADIANCE\n")
            lighting = resolve_lighting({"lighting": {"type": "hdr", "hdr_path": "sky.hdr",
                                                     "skylight_intensity": 2}}, Path(folder))
            evo = Mock()
            configure_lighting(evo, lighting)
            evo.ConfigureCurrentSceneCudaEnvironmentMapForCapture.assert_called_once_with(str(hdr), 2, 0, 2.2)
            evo.ConfigureCurrentSceneCudaOutdoorLightingForCapture.assert_not_called()
            self.assertEqual(lighting["sun_intensity"], 0)
            self.assertEqual(lighting["ambient_light_intensity"], 0)
            self.assertEqual(len(lighting["hdr_sha256"]), 64)

    def test_switching_sky_does_not_change_cases(self):
        import json
        config = json.loads((Path(__file__).resolve().parents[1] / "prefab_field_illumination_search.example.json").read_text())
        hdr_config = copy.deepcopy(config)
        hdr_config["lighting"] = {"type": "hdr", "hdr_path": "sky.hdr", "skylight_intensity": 2}
        self.assertEqual(build_cases(config), build_cases(hdr_config))

    def test_rejects_missing_hdr_and_nonfinite_intensity(self):
        for lighting in ({"type": "hdr", "hdr_path": "missing.hdr"}, {"skylight_intensity": float("nan")}):
            with self.assertRaises(ValueError):
                resolve_lighting({"lighting": lighting}, Path.cwd())


if __name__ == "__main__":
    unittest.main()
