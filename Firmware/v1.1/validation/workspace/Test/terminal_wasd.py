"""SSH终端WASD：不依赖X11，输入与底盘速度循环共用50Hz刷新。"""
import os,sys,time,select,termios,tty,signal
from control_core import Chassis,Gear,Slew,keyboard_target

class TerminalKeys:
    def __init__(self,timeout):self.timeout=timeout;self.key=None;self.last=0.;self.quit=False
    def feed(self,keys,now,gear):
        for key in keys.lower():
            if key in 'wasd':self.key=key;self.last=now
            elif key==' ':self.key=None
            elif key in 're':print(gear.change(1 if key=='r' else -1),flush=True)
            elif key in ['q','\x1b','\x03']:self.quit=True;self.key=None
    def target(self,now,gear):
        return keyboard_target({self.key} if self.key and now-self.last<self.timeout else set(),gear)

def run(config):
    if not sys.stdin.isatty():raise RuntimeError('终端模式需要交互式SSH终端；图形模式请加 --gui')
    fd=sys.stdin.fileno();old=termios.tcgetattr(fd);car=None
    def interrupted(*args):raise KeyboardInterrupt()
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,interrupted)
    keys=TerminalKeys(config['keyboard']['repeat_timeout_s']);gear=Gear(config);slew=Slew(config)
    try:
        car=Chassis(config);car.connect();tty.setcbreak(fd)
        print('[TERMINAL] 长按W/S前后，A/D转向；R升档 E降档；空格停；Q/Ctrl+C退出。无需回车。',flush=True)
        print('[TERMINAL] SSH不提供松键事件：%.2fs无方向键重复后平滑归零；精确松键模式用 --gui。'%keys.timeout,flush=True)
        print(gear.text(),flush=True);last=time.monotonic();last_log=0.
        while not keys.quit:
            now=time.monotonic()
            if select.select([fd],[],[],0)[0]:
                raw=os.read(fd,64)
                if not raw:break
                text=raw.decode(errors='ignore');keys.feed(text,now,gear)
                print('[INPUT]',repr(text),'direction='+str(keys.key),flush=True)
            if keys.quit:break
            car.healthy();v,w=keys.target(now,gear);v,w=slew.step(v,w,now-last);car.speed(v,w);last=now
            if now-last_log>.5:
                print('[WASD] vx=%.3f wz=%.3f measured=%s'%(v,w,car.body),flush=True);last_log=now
            time.sleep(1./config['control']['publish_hz'])
    finally:
        # 优先让底盘收到停止；即使停止写入失败，也恢复终端设置。
        try:
            if car:car.close()
        finally:termios.tcsetattr(fd,termios.TCSANOW,old)
