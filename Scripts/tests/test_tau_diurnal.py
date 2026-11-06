import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('tau', Path(__file__).resolve().parents[1] / 'run_tau_diurnal.py')
tau = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tau)


class TauSummaryTest(unittest.TestCase):
    def records(self):
        return [SimpleNamespace(cultivar=g, sensor_bar_level=level, scalar=value)
                for g in ('BTX', 'Pawaga') for level in ('top', 'middle', 'bottom')
                for value in ([2.] * 50 if level == 'top' else [0., 1.] * 25)]

    def test_ratio_of_panel_means_and_cv(self):
        result = tau.summarize(self.records())
        self.assertEqual(result['BTX_tau'], .25)
        self.assertEqual(result['Pawaga_bottom_cv'], 1.)

    def test_incomplete_panel_is_rejected(self):
        with self.assertRaises(ValueError):
            tau.summarize(self.records()[:-1])

    def test_invalid_reference_or_probe_is_rejected(self):
        for value in (0., float('nan'), -1.):
            records = self.records()
            for r in records:
                if r.sensor_bar_level == 'top':
                    r.scalar = value
            with self.assertRaises(ValueError):
                tau.summarize(records)


if __name__ == '__main__':
    unittest.main()
