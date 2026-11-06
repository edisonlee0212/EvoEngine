import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('supports', Path(__file__).resolve().parents[1] / 'fit_tau_parbar_supports.py')
supports = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supports)


class ParbarSupportTest(unittest.TestCase):
    def test_extension_keeps_base_and_diameter(self):
        points = np.array([[2, 3, 5, 1], [2, 7, 5, 1], [2.1, 5, 5.1, 1]]).T
        result = supports.extend_pole(3, 7, 2) @ points
        np.testing.assert_allclose(result, np.array([[2, 3, 5, 1], [2, 9, 5, 1], [2.1, 6, 5.1, 1]]).T)

    def test_collapsed_pole_is_rejected(self):
        with self.assertRaises(ValueError):
            supports.extend_pole(3, 7, -4)

    def test_north_order_preserves_height_distance_and_bar_axis(self):
        reference = {
            'BTX:bottom': dict(center=[3., .15, 5.], axis_deg=45., span_m=1.),
            'BTX:middle': dict(center=[2., .2, 5.1], axis_deg=45., span_m=1.),
            'BTX:top': dict(center=[1., 2.3, 5.2], axis_deg=45., span_m=1.),
        }
        result = supports.north_order_reference(reference)
        before = np.array([p['center'] for p in reference.values()])
        after = np.array([p['center'] for p in result.values()])
        self.assertTrue(np.all(np.diff(after[:, 0]) > 0))
        np.testing.assert_allclose(after[:, 1], before[:, 1])
        np.testing.assert_allclose(after.mean(axis=0), before.mean(axis=0))
        np.testing.assert_allclose(np.linalg.norm(np.diff(after, axis=0), axis=1),
                                   np.linalg.norm(np.diff(before, axis=0), axis=1))
        for key in reference:
            self.assertEqual(result[key]['span_m'], reference[key]['span_m'])
            self.assertEqual((result[key]['axis_deg'] - reference[key]['axis_deg']) % 180, 0)

    def test_middle_height_preserves_other_reference_coordinates(self):
        path = Path(__file__).resolve().parents[1] / 'tau_diurnal/measured_bars.json'
        reference = json.loads(path.read_text())
        original = json.loads(path.read_text())
        rotated = supports.north_order_reference(reference)
        result = supports.supported_bar_reference(reference)
        for key, panel in result.items():
            if key.endswith(':middle'):
                self.assertEqual(panel['center'][1], .8)
                self.assertEqual(panel['center'][::2], rotated[key]['center'][::2])
                self.assertEqual(panel['span_m'], rotated[key]['span_m'])
                self.assertEqual(panel['axis_deg'], rotated[key]['axis_deg'])
            else:
                self.assertEqual(panel, rotated[key])
        self.assertEqual(reference, original)


if __name__ == '__main__':
    unittest.main()
