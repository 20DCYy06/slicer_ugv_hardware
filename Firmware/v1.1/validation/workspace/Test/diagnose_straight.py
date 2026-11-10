#!/usr/bin/env python3
"""只接收编码器/IMU，人工推车测量反馈方向；从不发送串口指令。"""
import argparse,json,math,os,select,signal,struct,sys,termios,time,tty
from datetime import datetime,timezone
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Hardware/Host'))
from chassis import load_config,device,CONFIG
from transport import Link
from protocol import Parser
import yaml

PHASES={'s':'still','f':'forward','l':'left','r':'right','x':'free'}
class ReadOnlyLink(Link):
    def send(self,*args,**kwargs):raise RuntimeError('只读诊断禁止发送任何串口指令')

def decode(kind,payload):
    if kind==0x90 and len(payload)==61:
        v=struct.unpack('<II4iBH4i4h4hH',payload);flags=v[6]
        return dict(kind='wheel',mcu_us=v[0],flags=flags,motor_enabled=bool(flags&16),
            motor_power_present=bool(flags&32),fault=bool(flags&4),raw_abcd=list(v[8:12]),
            wheel_mm_s=list(v[12:16]),pwm_ticks=list(v[16:20]),battery_v=v[20]/1000)
    if kind==0x93 and len(payload)==64:
        v=struct.unpack('<4BI4h4h3h3h3h3h4h4H',payload)
        return dict(kind='body',mcu_us=v[4],imu_flags=v[1],who=v[2],source=v[3],
            delta_raw_abcd=list(v[5:9]),wheel_mm_s=list(v[9:13]),gyro_raw=list(v[13:16]),
            accel_raw=list(v[16:19]),gyro_rad_s=[x/1000 for x in v[19:22]],
            accel_m_s2=[x/1000 for x in v[22:25]],vx=v[25]/1000,wz=v[26]/1000,
            target_vx=v[27]/1000,target_wz=v[28]/1000,status=v[30],imu_age_ms=v[31])

def delta32(current,previous):return ((current-previous+2**31)%2**32)-2**31

def analyze(records,cfg,firmware):
    h=firmware['hardware'];signs=cfg['serial']['encoder_signs'];side=h['wheel_side']
    counts=[0]*4;previous_wheel=None;previous_body=None
    angles={'left':0.,'right':0.};xy=[];phase_count={};still=[]
    for rec in records:
        phase=rec['phase'];phase_count[phase]=phase_count.get(phase,0)+1
        if rec['kind']=='wheel':
            if phase=='forward' and previous_wheel and previous_wheel['phase']=='forward' and previous_wheel['segment']==rec['segment']:
                for i in range(4):counts[i]+=delta32(rec['raw_abcd'][i],previous_wheel['raw_abcd'][i])
            previous_wheel=rec
        elif rec['kind']=='body':
            if phase=='still':still.append(rec)
            if phase in angles and rec['imu_flags']&7==7:
                if previous_body and previous_body['phase']==phase and previous_body['segment']==rec['segment']:
                    dt=((rec['mcu_us']-previous_body['mcu_us'])&0xffffffff)/1e6
                    if 0<dt<=1:angles[phase]+=rec['wz']*dt
                nominal=sum(s*w for s,w in zip(side,rec['wheel_mm_s']))/(2000*(h['track_width_m']+h['wheelbase_m']))
                if abs(rec['wz'])>.03 and abs(nominal)>.01:xy.append((nominal,rec['wz']))
            previous_body=rec
    distance=[n*s*math.pi*h['wheel_diameter_m']/h['counts_per_revolution'] for n,s in zip(counts,signs)]
    denom=math.sqrt(sum(x*x for x,y in xy)*sum(y*y for x,y in xy))
    correlation=sum(x*y for x,y in xy)/denom if denom else None
    notes=[]
    if all(x>.01 for x in distance):notes.append('向车头推车时四轮编码器均为正。')
    elif all(x<-.01 for x in distance):notes.append('向车头推车时四轮编码器均为负：当前vx正方向与用户定义的车头相反。')
    elif any(abs(x)>.01 for x in distance):notes.append('向前推车时各轮符号/变化不一致，先核对编码器方向与轮位。')
    else:notes.append('向前推车数据不足：按F标记后只向前推，不要在F阶段推回。')
    if angles['left']>.10 and angles['right']<-.10:notes.append('手动逆时针左转为正、顺时针右转为负：IMU偏航符号符合约定。')
    elif angles['left']<-.10 and angles['right']>.10:notes.append('手动左转为负、右转为正：IMU偏航轴方向与约定相反。')
    else:notes.append('左右转向数据不足或阶段标记与动作不一致。')
    if len(xy)>=10 and correlation is not None:
        if correlation<-.5:notes.append('轮位推导转向与IMU明显反号；这会使原来的负反馈纠偏有变成正反馈的风险，须校准轮位/编码器/IMU的坐标关系，不能盲目加PID。')
        elif correlation>.5:notes.append('轮位推导转向与IMU同号；此数据不支持直接反转IMU，应继续分析带指令时的轮速和偏航响应。')
        else:notes.append('轮位与IMU相关性不足，可能有打滑/轮位混接，暂不生成符号修改建议。')
    else:notes.append('有效转向关联样本不足；转动车身时应让车轮接地滚动。')
    return dict(commands_sent=0,phase_samples=phase_count,forward_raw_counts=counts,
        forward_signed_distance_m=[round(x,4) for x in distance],
        manual_yaw_degrees={k:round(v*180/math.pi,2) for k,v in angles.items()},
        yaw_correlation=None if correlation is None else round(correlation,4),correlation_samples=len(xy),
        mean_still_gyro_rad_s=[round(sum(f['gyro_rad_s'][i] for f in still)/len(still),5) for i in range(3)] if still else None,
        mean_still_accel_m_s2=[round(sum(f['accel_m_s2'][i] for f in still)/len(still),4) for i in range(3)] if still else None,
        findings=notes,automatic_calibration_applied=False)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(CONFIG));p.add_argument('--duration',type=float,default=120.)
    p.add_argument('--phase',choices=list(PHASES.values()),default='still')
    a=p.parse_args()
    if not math.isfinite(a.duration) or a.duration<=0:p.error('duration必须为正数')
    root=Path(__file__).resolve().parents[1];cfg=load_config(a.config)
    # 诊断日志记录实际已验证安装的版本，避免v1.1刷写后仍标记v1.0极性。
    install_path=root/'Hardware/last_install.json'
    install=json.loads(install_path.read_text()) if install_path.exists() else {}
    version=install.get('version','v1.0')
    firmware_path=(root/'Hardware/Firmware'/version/'src/config.yaml').resolve()
    firmware_path.relative_to((root/'Hardware/Firmware').resolve())
    firmware=yaml.safe_load(firmware_path.read_text())
    port,expected=device(cfg);link=None;old=None;records=[];phase=a.phase;segment=0;code=0
    folder=root/'Test/logs';folder.mkdir(exist_ok=True)
    stem='feedback_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    raw_path=folder/(stem+'.jsonl');summary_path=folder/(stem+'_summary.json')
    raw_file=raw_path.open('x',encoding='utf-8')
    raw_file.write(json.dumps({'kind':'metadata','port':port,'usb':expected,'host_config':cfg,'firmware_config':firmware,'read_only':True},ensure_ascii=False)+'\n');raw_file.flush()
    parser=Parser();started=time.monotonic();next_print=started;latest_wheel=None;latest_body=None;quit_requested=False
    def interrupted(*unused):raise KeyboardInterrupt()
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,interrupted)
    print('READ_ONLY：只收遥测；不发HELLO、速度、PWM、STOP、复位或解锁。',flush=True)
    print('先退出WASD/手柄，Motor OFF，保持板卡静止约2秒。',flush=True)
    print('S 静止采样；F 向车头方向缓慢推30~50cm；L 从上方看逆时针转车；R 顺时针转车；Q结束。',flush=True)
    print('这些键只标记采样阶段，不控制电机。转向时车轮应接地滚动，每段动作后按S；不要在F阶段推回。',flush=True)
    print('日志：',raw_path,flush=True)
    try:
        link=ReadOnlyLink(port,expected,restore_on_close=False)
        if sys.stdin.isatty():old=termios.tcgetattr(sys.stdin.fileno());tty.setcbreak(sys.stdin.fileno())
        while not quit_requested and time.monotonic()-started<a.duration:
            now=time.monotonic()
            if old is not None and select.select([sys.stdin.fileno()],[],[],0)[0]:
                for key in os.read(sys.stdin.fileno(),64).decode(errors='ignore').lower():
                    if key in ['q','\x1b']:quit_requested=True
                    elif key in PHASES:
                        phase=PHASES[key];segment+=1;print('[PHASE]',phase,flush=True)
            if not select.select([link.fd],[],[],.04)[0]:
                if latest_wheel is None and now-started>5:raise RuntimeError('未收到SLC2遥测；请检查固件/串口，不能据此认为底盘正常')
                continue
            data=os.read(link.fd,4096)
            if not data:raise RuntimeError('底盘USB断开')
            for kind,seq,payload in parser.feed(data):
                rec=decode(kind,payload)
                if rec is None:continue
                rec.update(phase=phase,segment=segment,host_elapsed_s=round(now-started,4),sequence=seq)
                raw_file.write(json.dumps(rec,ensure_ascii=False)+'\n');records.append(rec)
                if rec['kind']=='wheel':
                    latest_wheel=rec
                    if rec['motor_enabled'] or any(rec['pwm_ticks']):raise RuntimeError('Motor仍为ON或PWM非零；请先Motor OFF再手动推车，本程序未发送任何指令')
                else:latest_body=rec
            raw_file.flush()
            if now>=next_print and latest_wheel:
                text='[%s] wheel_ABCD=%s mm/s PWM=%s'%(phase,latest_wheel['wheel_mm_s'],latest_wheel['pwm_ticks'])
                if latest_body:text+=' gyroXYZ=%s wz=%+.3f target=(%+.3f,%+.3f)'%(latest_body['gyro_rad_s'],latest_body['wz'],latest_body['target_vx'],latest_body['target_wz'])
                print(text,flush=True);next_print=now+.5
    except KeyboardInterrupt:print('\n结束只读采样。',flush=True)
    except (RuntimeError,OSError) as e:print('[ERROR]',e,flush=True);code=2
    finally:
        if link:link.close()
        if old is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,old)
        raw_file.close()
        result=analyze(records,cfg,firmware);result.update(raw_log=str(raw_path),summary_file=str(summary_path),exit_code=code)
        summary_path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print('SUMMARY '+json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return code
if __name__=='__main__':raise SystemExit(main())
