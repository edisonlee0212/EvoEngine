"""Exercise the native Python cloud controls in an isolated, windowless engine."""

import importlib
import os
from pathlib import Path
import sys
import tempfile
import unittest


class CloudSettingsTest(unittest.TestCase):
    def test_layer_settings(self):
        with self.assertRaises(RuntimeError):
            evo.GetRayTracerCloudSettings()

        with tempfile.TemporaryDirectory() as folder:
            evo.PushRenderLayer()
            evo.PushRayTracerLayer()
            self.assertTrue(evo.RunWindowless(Path(folder) / "Clouds.eveproj"))
            try:
                settings = evo.GetRayTracerCloudSettings()
                self.assertFalse(settings.affect_illumination_probes)
                settings.ApplyPreset(evo.CloudFormation.BroadBank)
                settings.enabled = True
                settings.affect_illumination_probes = True
                settings.wind_direction = (3, 4)
                settings.primary_step_count = 1000
                with self.assertRaises(ValueError):
                    settings.coverage = float("nan")
                with self.assertRaises(ValueError):
                    settings.wind_direction = (0, float("inf"))
                self.assertFalse(evo.GetRayTracerCloudSettings().affect_illumination_probes)

                evo.SetRayTracerCloudSettings(settings)
                active = evo.GetRayTracerCloudSettings()
                self.assertTrue(active.enabled and active.affect_illumination_probes)
                self.assertAlmostEqual(active.base_noise_scale, 0.00045)
                self.assertAlmostEqual(active.wind_direction[1], 0.8)
                self.assertEqual(active.primary_step_count, 512)
                settings.ApplyPreset(evo.CloudFormation.Scattered)
                self.assertAlmostEqual(evo.GetRayTracerCloudSettings().coverage, 0.9)
                self.assertEqual(settings.primary_step_count, 1000)
                evo.SetRayTracerCloudSettings(settings)
                self.assertAlmostEqual(evo.GetRayTracerCloudSettings().coverage, 0.62)
            finally:
                evo.Terminate()


if __name__ == "__main__":
    runtime = Path(sys.argv[1]).resolve()
    sys.path.insert(0, str(runtime))
    if os.name == "nt":
        dll_paths = [os.add_dll_directory(str(path)) for path in (runtime, runtime / "Packages") if path.is_dir()]
    os.chdir(runtime)
    evo = importlib.import_module(sys.argv[2])
    unittest.main(argv=[sys.argv[0]])
