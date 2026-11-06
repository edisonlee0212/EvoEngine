import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_tau_variance import scaled_descriptor


class DescriptorScaleTest(unittest.TestCase):
    def test_only_standard_deviations_change(self):
        source = {'count': {'mean': 14, 'deviation': 1},
                  'length': {'mean': {'min_value': 0, 'max_value': .9},
                             'deviation': {'min_value': .01, 'max_value': .1,
                                           'curve': {'values_': [[0, .5], [1, 1]]}}},
                  'texture': {'asset_handle_': 18446744073709551601}, 'enable_panicle': False}
        result = scaled_descriptor(source,.5)
        self.assertEqual(result['count'], {'mean':14,'deviation':.5})
        self.assertEqual(result['length']['deviation']['min_value'],.005)
        self.assertEqual(result['length']['deviation']['max_value'],.05)
        self.assertEqual(result['length']['deviation']['curve'],source['length']['deviation']['curve'])
        self.assertEqual(result['length']['mean'],source['length']['mean'])
        self.assertEqual(result['texture'],source['texture'])
        self.assertFalse(result['enable_panicle'])
        self.assertEqual(source['count']['deviation'],1)
        self.assertEqual(source['length']['deviation']['max_value'],.1)
        self.assertEqual(scaled_descriptor(source,1),source)

    def test_reject_zero_or_increased_scale(self):
        for scale in (0,-1,2):
            with self.assertRaises(ValueError):
                scaled_descriptor({},scale)


if __name__ == '__main__':
    unittest.main()
