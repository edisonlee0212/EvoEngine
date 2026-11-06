import sys
import unittest
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from plot_soybean_presentation import spacing_grids


class SpacingGridTests(unittest.TestCase):
    def setUp(self):
        self.frame = pd.DataFrame([
            dict(asset_name=name, row_spacing_inches=row, column_spacing_inches=column,
                 rows=8, columns=20, roi=.5, samples=16, bounces=2, repeats=5, score=row + column)
            for name in ["broad", "narrow"] for row in [10, 12.5] for column in [1, 1.5]
        ])

    def test_full_grid_orientation_and_values(self):
        grids = spacing_grids(self.frame.sample(frac=1, random_state=1), "score")
        self.assertEqual(grids["broad"].shape, (2, 2))
        self.assertEqual(grids["narrow"].loc[12.5, 1.5], 14)
        self.assertEqual(grids["broad"].index.tolist(), [10, 12.5])
        self.assertEqual(grids["broad"].columns.tolist(), [1, 1.5])

    def test_missing_cell_rejected(self):
        with self.assertRaisesRegex(ValueError, "Missing"):
            spacing_grids(self.frame.iloc[1:], "score")

    def test_different_protocols_not_pooled(self):
        self.frame.loc[0, "samples"] = 256
        with self.assertRaisesRegex(ValueError, "one field"):
            spacing_grids(self.frame, "score")

    def test_duplicate_cell_rejected(self):
        with self.assertRaises(ValueError):
            spacing_grids(pd.concat([self.frame, self.frame.iloc[:1]]), "score")


if __name__ == "__main__":
    unittest.main()
