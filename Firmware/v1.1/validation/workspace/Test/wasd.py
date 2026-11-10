#!/usr/bin/env python3
"""WASD速度控制：默认SSH终端，可选Tk按下/松开图形模式。"""
import argparse,os,sys,time,signal
sys.dont_write_bytecode=True
from control_core import *

def prepare_display(local=False):
    # SSH转发的DISPLAY必须保留本会话的认证；不能配上GDM的本地cookie。
    # 仅在未指定显示，或用户明确要求本地桌面时，成对选择:0与GDM认证。
    if local or not os.environ.get('DISPLAY'):
        os.environ['DISPLAY']=':0'
        auth='/run/user/%d/gdm/Xauthority'%os.getuid()
        if os.path.exists(auth):os.environ['XAUTHORITY']=auth

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',default=str(CONFIG));p.add_argument('--demo',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--gui',action='store_true',help='使用按下/松开事件的图形控制窗口');p.add_argument('--local-display',action='store_true',help='在Jetson本地/RustDesk桌面显示');a=p.parse_args()
    if a.demo:demo();return
    cfg=load_config(a.config);car=None
    try:
        if a.check:
            car=Chassis(cfg);print('[CHECK]',car.read_state());return
        if not a.gui and not a.local_display:
            from terminal_wasd import run
            run(cfg);return
        import tkinter as tk
        prepare_display(a.local_display)
        window=tk.Tk();window.title('Workspace2 - WASD 车体速度控制');window.geometry('680x330')
        keys=set();release_jobs={};errors=[];gear=Gear(cfg);slew=Slew(cfg);last=time.monotonic();last_log=0.
        label=tk.StringVar(value=gear.text())
        tk.Label(window,text='W/S 前进后退   A/D 左右自转\nW+A / W+D 连续曲线\nR 升档   E 降档   空格停下\nQ / Esc 退出；松键或窗口失焦平滑停止',font=('Sans',20),justify='left').pack(pady=20)
        tk.Label(window,textvariable=label,font=('Sans',20)).pack()
        car=Chassis(cfg);car.connect();print(gear.text(),flush=True)
        def press(event):
            k=event.keysym.lower()
            if k in release_jobs:window.after_cancel(release_jobs.pop(k))
            first=k not in keys;keys.add(k)
            if k in ['escape','q']:window.destroy();return
            if k=='space':keys.clear()
            if first and k in ['r','e']:
                print(gear.change(1 if k=='r' else -1),flush=True);label.set(gear.text())
        def release(event):
            k=event.keysym.lower()
            def commit():keys.discard(k);release_jobs.pop(k,None)
            # X11自动重复可能产生紧邻的release/press；同轮事件处理后再确认松开。
            release_jobs[k]=window.after_idle(commit)
        def tick():
            nonlocal last,last_log
            try:
                now=time.monotonic();car.healthy();v,w=keyboard_target(keys,gear);v,w=slew.step(v,w,now-last);car.speed(v,w);last=now
                if now-last_log>.5:
                    print('[WASD] vx=%.3f wz=%.3f measured=%s'%(v,w,car.body),flush=True);last_log=now
                window.after(round(1000/cfg['control']['publish_hz']),tick)
            except Exception as e:errors.append(str(e));print('[ERROR]',e,flush=True);window.destroy()
        window.bind('<KeyPress>',press);window.bind('<KeyRelease>',release)
        window.bind('<FocusOut>',lambda e:keys.clear())
        for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *args:window.destroy())
        window.focus_force();window.after(20,tick);window.mainloop()
        if errors:raise RuntimeError(errors[-1])
    finally:
        if car:car.close()
        print('[WASD] STOP',flush=True)
if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as e:print('[ERROR]',e,file=sys.stderr);raise SystemExit(1)
