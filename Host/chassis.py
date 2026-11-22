"""独立SLC2车体速度链路。所有引用和日志均在新工作空间内。"""
import glob,json,math,os,secrets,select,struct,subprocess,time
from pathlib import Path
import yaml
from protocol import Parser,packet
from transport import Link,identity
ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config.yaml'

def load_config(path=CONFIG):
    return yaml.safe_load(Path(path).read_text())

def device(config):
    cfg=config['serial'];expected=(cfg['vid'],cfg['pid'],str(cfg['serial_number']))
    paths=[cfg['port']] if cfg['port']!='auto' else glob.glob('/dev/ttyACM*')+glob.glob('/dev/ttyUSB*')+glob.glob('/dev/ttyCH*')
    matches=[]
    for p in paths:
        try:
            if identity(p)==expected:matches.append(os.path.realpath(p))
        except OSError:pass
    matches=sorted(set(matches))
    if len(matches)!=1:raise RuntimeError('未找到唯一配置底盘：'+str(matches))
    result=subprocess.run(['fuser',matches[0]],capture_output=True,text=True)
    if result.returncode==0 and result.stdout.strip():raise RuntimeError('底盘串口被占用，PID='+result.stdout.strip())
    return matches[0],expected

def status(data):
    v=struct.unpack('<II4iBH4i4h4hH',data);f=v[6]
    return dict(tick=v[0],ack=v[7],flags=f,fault_latched=bool(f&4),configured=bool(f&8),motor_enabled=bool(f&16),
        motor_power_present=bool(f&32),wheel_mm_s=list(v[12:16]),pwm_ticks=list(v[16:20]),battery_v=v[20]/1000)

def body_status(data):
    v=struct.unpack('<4BI4h4h3h3h3h3h4h4H',data)
    return dict(imu_flags=v[1],who=v[2],source=v[3],tick=v[4],delta_raw_abcd=list(v[5:9]),
        vx=v[25]/1000,wz=v[26]/1000,target_vx=v[27]/1000,target_wz=v[28]/1000,
        status=v[30],imu_age_ms=v[31])

class Chassis:
    def __init__(self,config):
        self.config=config;self.path,expected=device(config)
        self.link=Link(self.path,expected,restore_on_close=False)
        self.parser=Parser();self.seq=0;self.nonce=secrets.randbits(32)
        self.last_rx=0.;self.last_body_rx=0.;self.state=None;self.body=None;self.replies={};self.opened_session=False
    def poll(self,timeout=0.):
        if not select.select([self.link.fd],[],[],timeout)[0]:return
        raw=os.read(self.link.fd,4096)
        if not raw:raise RuntimeError('底盘USB已断开')
        for kind,seq,data in self.parser.feed(raw):
            if kind==0x90 and len(data)==61:self.state=status(data);self.last_rx=time.monotonic()
            elif kind==0x93 and len(data)==64 and data[0]==1:
                sample=body_status(data)
                if self.body is None or 0 < ((sample['tick']-self.body['tick']) & 0xffffffff) < 0x80000000:
                    self.last_body_rx=time.monotonic()
                    self.body=sample
            else:self.replies[kind]=data
    def wait(self,predicate,timeout=3.):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>=end:raise TimeoutError('底盘响应超时')
            self.poll(.05)
    def send(self,kind,payload=b''):
        self.seq=(self.seq+1)&65535;self.link.send(packet(kind,self.seq,payload))
        return self.seq
    def read_state(self):
        self.wait(lambda:self.state is not None);return self.state
    def connect(self):
        previous_wheel_rx,previous_body=self.last_rx,self.body
        self.opened_session=True
        self.send(1,struct.pack('<I',self.nonce));self.wait(lambda:0x81 in self.replies)
        expected=(b'SLC2',2,4,self.config['serial']['counts_per_revolution'],500,self.nonce)
        if struct.unpack('<4sBBHHI',self.replies[0x81])!=expected:raise RuntimeError('SLC2身份/会话不匹配')
        s=self.config['serial'];self.send(0x12,struct.pack('<I8b',self.nonce,*(s['pwm_signs']+s['encoder_signs'])))
        self.wait(lambda:0x82 in self.replies)
        if struct.unpack('<IB',self.replies[0x82])!=(self.nonce,1):raise RuntimeError('极性配置未确认')
        self.send(0x17,struct.pack('<I',self.nonce));self.wait(lambda:0x84 in self.replies)
        n,version,who,flags,source=struct.unpack('<IBBBB',self.replies[0x84])
        if (n,version,who)!=(self.nonce,1,0xea):raise RuntimeError('车体IMU接口未就绪')
        percent=self.config['control']['motor_output_limit_percent']
        self.send(0x18,struct.pack('<IB',self.nonce,percent));self.wait(lambda:0x85 in self.replies)
        n,p,maximum,period,ticks=struct.unpack('<IBHHH',self.replies[0x85])
        expected_cap=self.config['control'].get('mcu_pwm_accept_max_permille',800)
        expected_ticks=(1600*expected_cap//1000)*percent//100
        if (n,p,maximum,period,ticks)!=(self.nonce,percent,expected_cap,1600,expected_ticks):
            raise RuntimeError('底盘固件输出上限不匹配: expected %d‰, received %d‰' % (expected_cap,maximum))
        timeout=self.config['control']['telemetry_timeout_s']

        def telemetry_ready():
            return (
                self.state is not None
                and self.last_rx > previous_wheel_rx
                and time.monotonic()-self.last_rx <= timeout
                and self.body is not None
                and self.body is not previous_body
                and self.body['imu_flags']&7==7
            )

        self.wait(telemetry_ready,10.)
        print('[CONNECTED]',self.path,'0x16 vx/wz, PWM cap=%.3f%%, watchdog=500ms'%(100*ticks/period),flush=True)
    def speed(self,v,w):
        if not math.isfinite(v) or not math.isfinite(w):raise ValueError('速度不是有限值')
        s=self.config['speed']
        if abs(v)>s['linear_max']+1e-9 or abs(w)>s['angular_max']+1e-9:raise ValueError('速度超出测试范围')
        self.send(0x16,struct.pack('<Ihh',self.nonce,round(v*1000),round(w*1000)))
    def healthy(self):
        self.poll(0.)
        if time.monotonic()-self.last_rx>self.config['control']['telemetry_timeout_s']:raise RuntimeError('底盘遥测丢失')
        if self.state and self.state['fault_latched']:raise RuntimeError('MCU故障，先Motor OFF后使用硬件仓库的 Unlock/unlock.py --unlock')
    def close(self):
        if self.link.fd is None:return
        try:
            if self.opened_session:
                for _ in range(3):self.send(0x10,bytes(8));time.sleep(.02)
        finally:self.link.close()

class Gear:
    """用整数格数消除0.05累加误差；两轴分别在上下限饱和。"""
    def __init__(self,config):self.cfg=config['speed'];self.vi=0;self.wi=0
    def change(self,direction):
        c=self.cfg
        self.vi=max(0,min(round((c['linear_max']-c['linear_min'])/c['linear_step']),self.vi+direction))
        self.wi=max(0,min(round((c['angular_max']-c['angular_min'])/c['angular_step']),self.wi+direction))
        return self.text()
    @property
    def v(self):return round(self.cfg['linear_min']+self.vi*self.cfg['linear_step'],3)
    @property
    def w(self):return round(self.cfg['angular_min']+self.wi*self.cfg['angular_step'],3)
    def text(self):return '[GEAR] linear=%.2f m/s angular=%.2f rad/s'%(self.v,self.w)

class Slew:
    def __init__(self,config):self.c=config['control'];self.v=self.w=0.
    def step(self,v,w,dt):
        dt=max(0.,min(dt,.1))
        def approach(a,b,step):return a+max(-step,min(step,b-a))
        self.v=approach(self.v,v,self.c['linear_acceleration']*dt)
        self.w=approach(self.w,w,self.c['angular_acceleration']*dt)
        return self.v,self.w

def save_log(name,record):
    folder=ROOT/'Logs';folder.mkdir(exist_ok=True)
    path=folder/(name+'_'+str(time.time_ns())+'.json');path.write_text(json.dumps(record,ensure_ascii=False,indent=2));print('记录：',path)
