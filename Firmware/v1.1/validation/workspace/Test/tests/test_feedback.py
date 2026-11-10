import sys,unittest,struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnose_straight import analyze,decode,delta32,ReadOnlyLink,load_config
import yaml
class FeedbackTests(unittest.TestCase):
    def config(self):
        r=Path(__file__).resolve().parents[2]
        return load_config(),yaml.safe_load((r/'Hardware/Firmware/v1.0/src/config.yaml').read_text())
    def test_decoder(self):
        p=bytearray(64);struct.pack_into('<4BI',p,0,1,7,0xea,0,40000)
        struct.pack_into('<4h',p,16,80,-80,80,-80);struct.pack_into('<h',p,50,500)
        result=decode(0x93,p);self.assertEqual(result['wz'],.5);self.assertEqual(result['wheel_mm_s'],[80,-80,80,-80])
    def test_wrap(self):self.assertEqual(delta32(-2147483640,2147483640),16)
    def records(self,inverse=False):
        rows=[]
        for j,phase in enumerate(['left','right']):
            sign=1 if phase=='left' else -1
            for n in range(20):rows.append(dict(kind='body',phase=phase,segment=j,mcu_us=(j*20+n)*40000,imu_flags=7,wz=sign*.5,wheel_mm_s=[v*sign*(-1 if inverse else 1) for v in [80,-80,80,-80]]))
        return rows
    def test_yaw_polarity(self):
        c,f=self.config();good=analyze(self.records(),c,f);bad=analyze(self.records(True),c,f)
        self.assertEqual(good['yaw_correlation'],1.);self.assertEqual(bad['yaw_correlation'],-1.)
        self.assertGreater(good['manual_yaw_degrees']['left'],0);self.assertLess(good['manual_yaw_degrees']['right'],0)
    def test_forward_distance(self):
        c,f=self.config();s=c['serial']['encoder_signs']
        rows=[dict(kind='wheel',phase='forward',segment=1,raw_abcd=[0]*4),dict(kind='wheel',phase='forward',segment=1,raw_abcd=[n*15000 for n in s])]
        result=analyze(rows,c,f);self.assertTrue(all(x>.05 for x in result['forward_signed_distance_m']))
    def test_read_only_transport_blocks_tx(self):
        with self.assertRaises(RuntimeError):ReadOnlyLink.send(object(),b'anything')
if __name__=='__main__':unittest.main()
