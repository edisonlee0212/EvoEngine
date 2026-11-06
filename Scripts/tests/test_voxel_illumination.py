"""Grid layout, world-transform and cache tests; no engine or GPU required."""

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from voxel_illumination import load_result, make_grid, save_result, validate_result


def fixture():
    shape = (3, 4, 5)
    spacing = np.array([2, 3, 4])
    size = np.array(shape) * spacing
    centers = np.stack(np.meshgrid(*[(np.arange(n) + 0.5) * step - length / 2
                                    for n, step, length in zip(shape, spacing, size)], indexing="ij"), axis=-1)
    values = centers[..., 0] + 4
    return dict(resolution=shape, volume_size=size.tolist(), centers_local=centers,
                energy=np.stack([values, np.zeros(shape), np.zeros(shape)], axis=-1),
                intensity=values, sample_count=np.full(shape, 8, dtype=np.uint32),
                samples_per_voxel=8, local_to_world=np.eye(4))


class VoxelVisualizationTest(unittest.TestCase):
    def test_xyz_mapping_and_cell_center_origin(self):
        result = fixture()
        grid = make_grid(result)
        np.testing.assert_allclose(grid.points, result["centers_local"].reshape(-1, 3, order="F"))
        np.testing.assert_allclose(grid["intensity"], result["intensity"].ravel(order="F"))

    def test_isosurface_plane_and_world_transform(self):
        result = fixture()
        # Rotate local X onto world Y, with non-uniform scale and translation.
        result["local_to_world"] = np.array([[0, -2, 0, 10], [3, 0, 0, 20],
                                             [0, 0, 4, 30], [0, 0, 0, 1]], dtype=float)
        surface = make_grid(result).contour([5], scalars="intensity")
        self.assertGreater(surface.n_cells, 0)
        np.testing.assert_allclose(surface.points[:, 1], 23)  # local X=1 -> world Y=23

    def test_roundtrip_keeps_raw_arrays(self):
        result = fixture()
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "measurement"
            save_result(result, folder, {"scene": "test"})
            restored = load_result(folder)
            for key in ("energy", "intensity", "centers_local", "sample_count", "local_to_world"):
                np.testing.assert_array_equal(restored[key], result[key])

    def test_smoothing_does_not_modify_raw_result(self):
        result = fixture()
        original = result["intensity"].copy()
        grid = make_grid(result, 0.7)
        np.testing.assert_array_equal(result["intensity"], original)
        np.testing.assert_array_equal(grid["raw_intensity"], original.ravel(order="F"))
        self.assertFalse(np.array_equal(grid["intensity"], original.ravel(order="F")))

    def test_invalid_data_is_not_silently_contoured(self):
        for mutation in (lambda r: r["intensity"].fill(np.nan),
                         lambda r: r["sample_count"].fill(0),
                         lambda r: r.update(resolution=(1, 4, 5)),
                         lambda r: r["energy"].fill(0)):
            result = fixture()
            mutation(result)
            with self.assertRaises(ValueError):
                validate_result(result)
        with self.assertRaises(ValueError):
            make_grid(fixture(), -1)


if __name__ == "__main__":
    unittest.main()
