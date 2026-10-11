import unittest
from handover.detector import Detector

class Tests(unittest.TestCase):
    def test_static_60_seconds(self):
        d=Detector(); d.arm(0)
        for k in range(1801):
            self.assertFalse(d.feed([100,200],k/30))
        self.assertEqual(d.state,'MONITORING')

    def test_pull_both_directions_and_once(self):
        for sign in (-1,1):
            d=Detector(); d.arm(0)
            for k in range(40): d.feed([100],k/30)
            hits=[k/30 for k in range(40,70) if d.feed([100+sign*30],k/30)]
            self.assertEqual(len(hits),1)
            self.assertLess(hits[0]-40/30,1)

    def test_unstable_baseline(self):
        d=Detector(); d.arm(0)
        for k in range(100): self.assertFalse(d.feed([k%2*20],k/30))
        self.assertEqual(d.state,'BASELINING')

    def test_stale_and_invalid(self):
        d=Detector(); d.arm(0); d.feed([100],0)
        self.assertEqual(d.check_stale(1),'FAULT')
        d.arm(1); self.assertFalse(d.feed([float('nan')],1))
        self.assertEqual(d.state,'FAULT')

    def test_masks_and_debounce(self):
        d=Detector(masked=[0]); d.arm(0)
        for k in range(40): d.feed([100,100],k/30)
        for k in range(40,70): self.assertFalse(d.feed([500,100],k/30))
        self.assertEqual(d.state,'MONITORING')

if __name__=='__main__': unittest.main()
