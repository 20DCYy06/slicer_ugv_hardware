import sys,struct,unittest
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'Host'))
from chassis import body_status,Chassis,load_config
class ProtocolTests(unittest.TestCase):
    def test_telemetry_offsets(self):
        p=bytearray(64);struct.pack_into('<4BI',p,0,1,7,0xea,3,123)
        struct.pack_into('<4h',p,8,-1,2,-3,4);struct.pack_into('<4h4H',p,48,125,1000,200,500,12000,15,10,99)
        t=body_status(p);self.assertEqual(t['delta_raw_abcd'],[-1,2,-3,4]);self.assertEqual(t['wz'],1.);self.assertEqual(t['vx'],.125)
    def test_body_command(self):
        c=object.__new__(Chassis);c.config=load_config();c.nonce=123;sent=[];c.send=lambda k,p:sent.append((k,p))
        c.speed(.6,-1.);self.assertEqual(sent,[(0x16,struct.pack('<Ihh',123,600,-1000))])
if __name__=='__main__':unittest.main()
