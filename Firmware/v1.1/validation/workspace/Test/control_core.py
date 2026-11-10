"""键盘/手柄共享逻辑，支持无硬件离线验证。"""
import sys,math,struct,os,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Hardware/Host'))
from chassis import Gear,Slew,load_config,Chassis,CONFIG
EVENT=struct.Struct('<IhBB')
def keyboard_target(keys,gear):
    return (int('w' in keys)-int('s' in keys))*gear.v,(int('a' in keys)-int('d' in keys))*gear.w

def gamepad_target(axes,gear,cfg):
    def axis(n):
        x=max(-1.,min(1.,axes.get(n,0.)));d=cfg['dead_zone']
        return 0. if abs(x)<=d else math.copysign((abs(x)-d)/(1-d),x)
    return cfg.get('throttle_sign',1)*axis(cfg['throttle_axis'])*gear.v,cfg.get('steering_sign',1)*axis(cfg['steering_axis'])*gear.w

class Joystick:
    def __init__(self,path):
        self.path=path;self.fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK);self.axes={}
    def poll(self):
        buttons=[]
        if not os.path.exists(self.path):raise RuntimeError('手柄断开')
        while True:
            try:data=os.read(self.fd,EVENT.size)
            except BlockingIOError:break
            if len(data)!=EVENT.size:raise RuntimeError('手柄读取中断')
            ms,value,kind,n=EVENT.unpack(data)
            if kind&2:self.axes[n]=max(-1.,min(1.,value/32767.))
            if kind&1 and not kind&0x80 and value:buttons.append(n)
        return self.axes,buttons
    def close(self):os.close(self.fd)

def gamepad_config(config):
    config=copy.deepcopy(config)
    angular=config['gamepad']['angular_speed']
    for field in ['min','max','step']:config['speed']['angular_'+field]=angular[field]
    return config

def demo(config=None):
    g=Gear(config if config is not None else load_config());print(g.text())
    for _ in range(10):print('R ->',g.change(1))
    for _ in range(10):print('E ->',g.change(-1))
