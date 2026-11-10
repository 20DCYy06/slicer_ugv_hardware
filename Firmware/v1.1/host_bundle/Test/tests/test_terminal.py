import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from terminal_wasd import TerminalKeys
from control_core import Gear,load_config
class TerminalTests(unittest.TestCase):
    def test_press_expiry_and_stop(self):
        g=Gear(load_config());k=TerminalKeys(.6);k.feed('w',1.,g)
        self.assertEqual(k.target(1.1,g),(.1,0.));self.assertEqual(k.target(1.61,g),(0.,0.))
        k.feed('a',2.,g);self.assertEqual(k.target(2.1,g),(0.,.1))
        k.feed(' ',2.2,g);self.assertEqual(k.target(2.2,g),(0.,0.))
    def test_direction_switch_gears_and_exit(self):
        g=Gear(load_config());k=TerminalKeys(.6);k.feed('wrr',1.,g)
        self.assertEqual((g.v,g.w),(.2,.3))
        k.feed('s',1.1,g);self.assertEqual(k.target(1.1,g),(-.2,0.))
        k.feed('d',1.2,g);self.assertEqual(k.target(1.2,g),(0.,-.3))
        k.feed('q',1.3,g);self.assertTrue(k.quit);self.assertEqual(k.target(1.3,g),(0.,0.))
if __name__=='__main__':unittest.main()
