#!/usr/bin/env python3
"""Linux手柄摇杆连续速度控制；RB/LB和终端R/E同步调档。"""
import argparse,select,signal,sys,time,termios,tty
sys.dont_write_bytecode=True
from control_core import *

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',default=str(CONFIG));p.add_argument('--demo',action='store_true');p.add_argument('--check',action='store_true');a=p.parse_args()
    cfg=gamepad_config(load_config(a.config))
    if a.demo:demo(cfg);return
    car=joy=None;old=None
    def interrupted(*args):raise KeyboardInterrupt()
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,interrupted)
    try:
        if a.check:
            car=Chassis(cfg);print('[CHECK]',car.read_state());print('Joystick exists:',os.path.exists(cfg['gamepad']['device']));return
        joy=Joystick(cfg['gamepad']['device']);car=Chassis(cfg);car.connect()
        gear=Gear(cfg);slew=Slew(cfg);last=time.monotonic();last_log=0.
        if sys.stdin.isatty():old=termios.tcgetattr(sys.stdin.fileno());tty.setcbreak(sys.stdin.fileno())
        print('[GAMEPAD] 左Y油门、右X方向；RB升档 LB降档 B停止；终端R/E调档，空格停，Q退出',flush=True);print(gear.text(),flush=True)
        while True:
            now=time.monotonic();axes,buttons=joy.poll();car.healthy();settings=cfg['gamepad'];stop=False
            for button in buttons:
                if button==settings['up_button']:print(gear.change(1),flush=True)
                elif button==settings['down_button']:print(gear.change(-1),flush=True)
                elif button==settings['stop_button']:stop=True
            if old is not None and select.select([sys.stdin],[],[],0)[0]:
                keys=os.read(sys.stdin.fileno(),64).decode(errors='ignore').lower()
                if 'q' in keys or '\x1b' in keys:break
                for key in keys:
                    if key in ['r','e']:print(gear.change(1 if key=='r' else -1),flush=True)
                    if key==' ':stop=True
            # 停止键把当前轴快照清为中立；重新拨杆才恢复，避免下一帧立即再次前冲。
            if stop:joy.axes={};axes={}
            v,w=gamepad_target(axes,gear,settings);v,w=slew.step(v,w,now-last);car.speed(v,w);last=now
            if now-last_log>.5:
                print('[GAMEPAD] vx=%.3f wz=%.3f measured=%s'%(v,w,car.body),flush=True);last_log=now
            time.sleep(1/cfg['control']['publish_hz'])
    finally:
        if old is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,old)
        if car:car.close()
        if joy:joy.close()
        print('[GAMEPAD] STOP',flush=True)
if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as e:print('[ERROR]',e,file=sys.stderr);raise SystemExit(1)
