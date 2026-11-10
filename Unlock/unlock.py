#!/usr/bin/env python3
"""--check只收遥测；--unlock在Motor OFF、零输出下用原HELLO机制清除软件故障。"""
import argparse,json,sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Host'))
from chassis import Chassis,load_config,save_log,CONFIG

def main():
    p=argparse.ArgumentParser(description=__doc__);g=p.add_mutually_exclusive_group()
    g.add_argument('--check',action='store_true');g.add_argument('--unlock',action='store_true');p.add_argument('--config',default=str(CONFIG));a=p.parse_args()
    car=None;record={'action':'unlock' if a.unlock else 'check','nonzero_command_sent':False};code=0
    try:
        car=Chassis(load_config(a.config));before=car.read_state();record['before']=before
        print('[STATUS]',json.dumps(before,ensure_ascii=False),flush=True)
        if a.unlock:
            if before['motor_enabled'] or any(before['pwm_ticks']):raise RuntimeError('请先Motor OFF并停车；未发送清锁握手')
            car.connect();car.state=None;after=car.read_state()
            if after['motor_enabled'] or after['fault_latched'] or any(after['pwm_ticks']):raise RuntimeError('清锁后状态未通过')
            record['after']=after;print('[UNLOCKED] fault=0，PWM=0；物理Motor仍为OFF。',flush=True)
        elif before['fault_latched']:code=2
        record['success']=code==0
    except (RuntimeError,OSError,TimeoutError,ValueError) as e:
        print('[ERROR]',e,file=sys.stderr);record['error']=str(e);code=3
    finally:
        if car:car.close()
        save_log('unlock',record)
    return code
if __name__=='__main__':raise SystemExit(main())
