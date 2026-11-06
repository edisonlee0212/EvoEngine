import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import pandas as pd

spec = importlib.util.spec_from_file_location('ensemble', Path(__file__).resolve().parents[1]/'run_tau_ensemble.py')
ensemble = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ensemble)


class EnsembleTest(unittest.TestCase):
    def band(self):
        return ensemble.summarize(pd.DataFrame([
            dict(date=d, genotype=g, time_mst=t, seed=s, tau=s/100)
            for d in ensemble.DATES for g in ('BTx623', 'Pawaga')
            for t in ('09:00', '09:15') for s in range(1, 21)]))

    def test_quantiles_and_counts(self):
        band = self.band()
        self.assertTrue((band.n == 20).all())
        self.assertAlmostEqual(band.iloc[0]['median'], .105)
        self.assertAlmostEqual(band.iloc[0].q025, .01475)
        self.assertAlmostEqual(band.iloc[0].q975, .19525)

    def test_one_unstable_endpoint_prevents_stop(self):
        a = self.band()
        b = a.copy()
        b.loc[7, 'q975'] += .011
        result = ensemble.stability(a, b, .01)
        self.assertFalse(result['passed'])
        self.assertEqual(result['worst']['statistic'], 'q975')
        self.assertTrue(ensemble.stability(a, a.sample(frac=1), .01)['passed'])

    def test_missing_time_is_rejected(self):
        a = self.band()
        with self.assertRaises(ValueError):
            ensemble.stability(a, a.iloc[:-1], .01)

    def test_reuse_rejects_changed_physics_and_runtime(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            folder = Path(temporary)
            project = folder/'test.eveproj'
            run = dict(project=str(project), samples=128, bounces=4, leaf_vsub=.02,
                       leaf_hsub=4, push_normal_distance=.001,
                       descriptors=dict(zip(('BTX', 'Pawaga'),
                           [f'ManualAssets/Descriptors/TauSeasonal/{d}' for d in ensemble.DESCRIPTORS])),
                       hashes={'binding.pyd': 'original'})
            (folder/'run.json').write_text(json.dumps(run))
            ensemble.check_run_configuration(folder, project, ensemble.DESCRIPTORS, run['hashes'])
            with self.assertRaises(ValueError):
                ensemble.check_run_configuration(folder, project, ensemble.DESCRIPTORS, {'binding.pyd': 'changed'})
            run['bounces'] = 2
            (folder/'run.json').write_text(json.dumps(run))
            with self.assertRaises(ValueError):
                ensemble.check_run_configuration(folder, project, ensemble.DESCRIPTORS, run['hashes'])

    def test_single_date_has_only_its_own_stability_grid(self):
        band = self.band().query("date == '2021-09-21'")
        self.assertEqual(len(band), 4)
        self.assertTrue(ensemble.stability(band, band, .01)['passed'])


if __name__ == '__main__':
    unittest.main()
