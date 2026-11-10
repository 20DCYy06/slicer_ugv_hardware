"""Xvfb键盘按下/松开/组合键烟测；Chassis替身，不导通串口。"""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import wasd,tkinter as tk
sent=[];instances=[]
class Fake:
    def __init__(self,cfg):self.body={};self.closed=False;instances.append(self)
    def connect(self):pass
    def healthy(self):pass
    def speed(self,v,w):sent.append((time.monotonic(),v,w))
    def close(self):self.closed=True
wasd.Chassis=Fake
real=tk.Tk
started=time.monotonic()
def factory():
    root=real()
    def event(kind,key):root.event_generate(kind,keysym=key)
    root.after(100,lambda:event('<KeyPress>','w'))
    root.after(180,lambda:event('<KeyPress>','r'))
    root.after(190,lambda:event('<KeyPress>','r'))
    root.after(210,lambda:event('<KeyRelease>','r'))
    root.after(240,lambda:event('<KeyPress>','a'))
    root.after(500,lambda:event('<KeyRelease>','w'))
    root.after(500,lambda:event('<KeyRelease>','a'))
    root.after(1100,root.destroy)
    return root
tk.Tk=factory
sys.argv=['wasd.py','--gui'];wasd.main()
assert instances and instances[0].closed
assert any(v>0 and w>0 for _,v,w in sent),'Combined curve was not emitted'
assert max(abs(v) for _,v,w in sent)<=.150001,'R repeated more than one gear'
assert max(abs(w) for _,v,w in sent)<=.200001
assert sent and all(abs(v)<1e-8 and abs(w)<1e-8 for _,v,w in sent[-5:]),'Release did not stop'
print('WASD_UI_PRESS_RELEASE_CURVE_SINGLE_GEAR_PASS calls=%d; serial_never_opened'%len(sent))
