import math
import re
import struct
import unittest
from pathlib import Path

from Scripts.generate_restir_pairing_maps import SIZES


class RestirPairingMapsTest(unittest.TestCase):
    def test_paper_scale_maps_are_reciprocal(self):
        path = Path(__file__).resolve().parents[2] / "EvoEngine_SDK/Internals/DefaultResources/RestirPtPairing.bin"
        data = path.read_bytes()
        offset = 0
        for dimension in SIZES:
            count = dimension * dimension
            deltas = list(struct.iter_unpack("<hh", data[offset:offset + count * 4]))
            offset += count * 4
            self.assertEqual(len(deltas), count)
            sigma = math.sqrt(sum(dx * dx + dy * dy for dx, dy in deltas) / (2 * count))
            self.assertAlmostEqual(sigma, 16.0, delta=1.0)
            for index, (dx, dy) in enumerate(deltas):
                self.assertNotEqual((dx, dy), (0, 0))
                x, y = index % dimension, index // dimension
                partner = ((y + dy) % dimension) * dimension + (x + dx) % dimension
                self.assertEqual(deltas[partner], (-dx, -dy))
        self.assertEqual(offset, len(data))

    def test_shader_and_renderer_offsets_match_asset(self):
        root = Path(__file__).resolve().parents[2]
        shader = (root / "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/RestirPt.slang").read_text()
        sizes = tuple(int(value[:-1]) for value in re.findall(
            r"\d+u", re.search(r"EE_RESTIR_PT_PAIRING_SIZES\[16\] = \{([^}]+)\}", shader).group(1)))
        offsets = tuple(int(value[:-1]) for value in re.findall(
            r"\d+u", re.search(r"EE_RESTIR_PT_PAIRING_OFFSETS\[16\] = \{([^}]+)\}", shader).group(1)))
        expected_offsets = []
        words = 0
        for size in SIZES:
            expected_offsets.append(words)
            words += size * size
        self.assertEqual(sizes, SIZES)
        self.assertEqual(offsets, tuple(expected_offsets))
        renderer = (root / "EvoEngine_SDK/src/RenderLayer.cpp").read_text()
        self.assertIn(f"pairing_map_words = {words}u", renderer)


if __name__ == "__main__":
    unittest.main()
