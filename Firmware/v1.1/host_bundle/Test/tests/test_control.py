"""协议、调档、松键停车、差速曲线与手柄输入验证；不打开真实串口。"""
import sys,unittest,struct,math
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from control_core import *
from protocol import Parser,packet
from unittest.mock import patch
class Tests(unittest.TestCase):
    def test_gears(self):
        g=Gear(load_config())
        self.assertEqual((g.v,g.w),(.1,.1))
        for n in range(1,21):
            g.change(1);self.assertEqual(g.v,round(min(.6,.1+n*.05),3));self.assertEqual(g.w,round(min(1.,.1+n*.1),3))
        for n in range(20):g.change(-1)
        self.assertEqual((g.v,g.w),(.1,.1))
    def test_keyboard_curve_release(self):
        g=Gear(load_config());self.assertEqual(keyboard_target({'w','a'},g),(.1,.1));self.assertEqual(keyboard_target({'d'},g),(0.,-.1));self.assertEqual(keyboard_target(set(),g),(0.,0.))
        self.assertEqual(keyboard_target({'w'},g),(.1,0.))
        self.assertEqual(keyboard_target({'s'},g),(-.1,0.))
        self.assertEqual(keyboard_target({'a'},g),(0.,.1))
    def test_slew_stop(self):
        s=Slew(load_config())
        for n in range(100):v,w=s.step(.6,1.,.02)
        self.assertAlmostEqual(v,.6);self.assertAlmostEqual(w,1.)
        for n in range(100):v,w=s.step(0.,0.,.02)
        self.assertAlmostEqual(v,0.);self.assertAlmostEqual(w,0.)
    def test_gamepad_range_and_deadzone(self):
        c=gamepad_config(load_config());g=Gear(c)
        self.assertEqual(gamepad_target({2:.05,1:-.04},g,c['gamepad']),(0.,0.))
        for n in range(11):g.change(1)
        self.assertEqual(gamepad_target({2:1.,1:-1.},g,c['gamepad']),(.6,-.5))
    def test_gamepad_halved_gears_and_reversed_directions(self):
        c=gamepad_config(load_config());g=Gear(c)
        self.assertEqual((g.v,g.w),(.1,.05))
        self.assertEqual(gamepad_target({1:-1.,2:-1.},g,c['gamepad']),(.1,.05))
        for n in range(1,12):
            g.change(1);self.assertEqual(g.w,round(min(.5,.05+n*.05),3))
        self.assertEqual((g.v,g.w),(.6,.5))
        for n in range(12):g.change(-1)
        self.assertEqual((g.v,g.w),(.1,.05))
        self.assertEqual(gamepad_target({0:1.},g,c['gamepad']),(0.,0.))
    def test_frame_crc_fragmentation(self):
        f=packet(0x16,5,struct.pack('<Ihh',123,600,-1000));p=Parser();out=[]
        for x in f:out+=p.feed(bytes([x]))
        self.assertEqual(out,[(0x16,5,struct.pack('<Ihh',123,600,-1000))])
        bad=bytearray(f);bad[-1]^=1;self.assertEqual(Parser().feed(bad),[])
    def test_body_only_no_pwm(self):
        c=object.__new__(Chassis);c.config=load_config();c.nonce=123;calls=[];c.send=lambda kind,p:calls.append((kind,p))
        c.speed(.6,1.);self.assertEqual(calls,[(0x16,struct.pack('<Ihh',123,600,1000))])
        for v,w in [(float('nan'),0),(.61,0),(0,1.01)]:
            with self.assertRaises(ValueError):c.speed(v,w)
    def test_joystick_buttons_and_disconnect(self):
        from control_core import Joystick,EVENT
        j=object.__new__(Joystick);j.path='/fake';j.fd=8;j.axes={}
        packets=[EVENT.pack(1,-32767,2,1),EVENT.pack(2,1,1,5)]
        def read(*a):
            if packets:return packets.pop(0)
            raise BlockingIOError()
        with patch('os.path.exists',return_value=True),patch('os.read',side_effect=read):
            axes,buttons=j.poll();self.assertEqual(axes[1],-1.);self.assertEqual(buttons,[5])
        with patch('os.path.exists',return_value=False):
            with self.assertRaises(RuntimeError):j.poll()
if __name__=='__main__':unittest.main()
