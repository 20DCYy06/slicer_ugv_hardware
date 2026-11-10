"""通过真实PTY验证SSH终端按键链路；底盘替身保证不打开硬件。"""
import os,sys,json,time,select,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
if '--child' in sys.argv:
    import wasd,terminal_wasd
    class Fake:
        def __init__(self,cfg):self.body={}
        def connect(self):print('FAKE_CONNECTED',flush=True)
        def healthy(self):pass
        def speed(self,v,w):print('FAKE_TX '+json.dumps([v,w]),flush=True)
        def close(self):print('FAKE_CLOSED',flush=True)
    terminal_wasd.Chassis=Fake
    def forbidden(*args):raise AssertionError('Unexpected GUI chassis opening')
    wasd.Chassis=forbidden
    sys.argv=['wasd.py'];wasd.main()
    assert 'tkinter' not in sys.modules
    raise SystemExit(0)
master,slave=os.openpty();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONIOENCODING='utf-8');env.pop('DISPLAY',None);env.pop('XAUTHORITY',None)
p=subprocess.Popen([sys.executable,'-s',__file__,'--child'],stdin=slave,stdout=slave,stderr=slave,env=env,close_fds=True);os.close(slave)
buffer=bytearray()
def drain(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if select.select([master],[],[],.02)[0]:
            try:data=os.read(master,4096)
            except OSError:break
            if not data:break
            buffer.extend(data)
try:
    for n in range(100):
        drain(.03)
        if b'[TERMINAL]' in buffer:break
    else:raise AssertionError('Terminal did not start: '+buffer.decode(errors='replace'))
    os.write(master,b'w');drain(1.05)
    os.write(master,b'r');drain(.1)
    os.write(master,b'a');drain(.25)
    os.write(master,b' ');drain(.3)
    os.write(master,b'q');drain(.3);p.wait(timeout=3)
    text=buffer.decode(errors='replace')
    assert p.returncode==0,text
    commands=[json.loads(line.split('FAKE_TX ',1)[1]) for line in text.splitlines() if line.startswith('FAKE_TX ')]
    assert any(v>0 for v,w in commands),text
    assert any(w>0 for v,w in commands),text
    assert all(abs(v)<1e-8 and abs(w)<1e-8 for v,w in commands[-5:]),text
    assert 'FAKE_CLOSED' in text,text
    assert 'linear=0.15' in text,text
    print('TERMINAL_PTY_W_A_R_SPACE_Q_PASS samples=%d; no DISPLAY; serial never opened'%len(commands))
finally:
    if p.poll() is None:p.terminate();p.wait(timeout=3)
    os.close(master)
