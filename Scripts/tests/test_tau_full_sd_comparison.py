import sys
from pathlib import Path
import unittest

import pandas as pd

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_tau_full_sd_comparison import summarize


class FullDayBandTest(unittest.TestCase):
    def setUp(self):
        self.times = ['08:15','08:30']
        self.data = pd.DataFrame([
            dict(condition=c,genotype=g,time_mst=t,seed=s,tau=value)
            for c in ('original','half_sd','quarter_sd') for g in ('BTx623','Pawaga')
            for t in self.times for s,value in ((7101,.2),(7102,.6))])

    def test_complete_matched_grid_uses_full_extrema(self):
        result = summarize(self.data,[7101,7102],self.times)
        self.assertEqual(len(result),12)
        self.assertTrue((result.n==2).all())
        self.assertTrue((result.minimum==.2).all())
        self.assertTrue((result.maximum==.6).all())
        self.assertTrue((result['median']==.4).all())

    def test_missing_pair_duplicate_or_nonfinite_value_is_rejected(self):
        missing = self.data.iloc[1:]
        duplicate = pd.concat([self.data,self.data.iloc[:1]])
        nonfinite = self.data.copy()
        nonfinite.loc[0,'tau'] = float('nan')
        for data in (missing,duplicate,nonfinite):
            with self.assertRaises(AssertionError):
                summarize(data,[7101,7102],self.times)


if __name__ == '__main__':
    unittest.main()
