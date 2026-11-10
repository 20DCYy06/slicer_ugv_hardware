import os,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import wasd
class DisplayTests(unittest.TestCase):
    def test_forwarded_display_does_not_inject_local_auth(self):
        with patch.dict(os.environ,{'DISPLAY':'localhost:10.0'},clear=True):
            wasd.prepare_display()
            self.assertEqual(os.environ['DISPLAY'],'localhost:10.0')
            self.assertNotIn('XAUTHORITY',os.environ)
    def test_forwarded_explicit_auth_is_preserved(self):
        with patch.dict(os.environ,{'DISPLAY':'localhost:10.0','XAUTHORITY':'/tmp/ssh-cookie'},clear=True):
            wasd.prepare_display();self.assertEqual(os.environ['XAUTHORITY'],'/tmp/ssh-cookie')
    def test_missing_display_selects_local_pair(self):
        with patch.dict(os.environ,{},clear=True),patch.object(os.path,'exists',return_value=True):
            wasd.prepare_display();self.assertEqual(os.environ['DISPLAY'],':0')
            self.assertEqual(os.environ['XAUTHORITY'],'/run/user/%d/gdm/Xauthority'%os.getuid())
    def test_explicit_local_overrides_forwarded_pair(self):
        with patch.dict(os.environ,{'DISPLAY':'localhost:10.0','XAUTHORITY':'/tmp/ssh-cookie'},clear=True),patch.object(os.path,'exists',return_value=True):
            wasd.prepare_display(True);self.assertEqual(os.environ['DISPLAY'],':0')
            self.assertEqual(os.environ['XAUTHORITY'],'/run/user/%d/gdm/Xauthority'%os.getuid())
if __name__=='__main__':unittest.main()
