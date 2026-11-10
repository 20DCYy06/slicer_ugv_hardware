import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'Hardware/Unlock'))
import unlock
class Fake:
    instances=[]
    initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
    def __init__(self,cfg):self.connected=False;self.state=dict(self.initial);self.instances.append(self)
    def read_state(self):
        if self.connected:self.state={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':False}
        return self.state
    def connect(self):self.connected=True
    def close(self):pass
class UnlockTests(unittest.TestCase):
    def run_action(self,arg):
        with patch.object(unlock,'Chassis',Fake),patch.object(unlock,'save_log'),patch.object(sys,'argv',['unlock.py',arg]):return unlock.main()
    def test_passive_check_never_handshakes(self):
        Fake.initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--check'),2);self.assertFalse(Fake.instances[-1].connected)
    def test_motor_on_refuses_unlock(self):
        Fake.initial={'motor_enabled':True,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--unlock'),3);self.assertFalse(Fake.instances[-1].connected)
    def test_motor_off_unlock(self):
        Fake.initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--unlock'),0);self.assertTrue(Fake.instances[-1].connected)
if __name__=='__main__':unittest.main()
