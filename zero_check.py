#!/usr/bin/env python3
"""USB供电、Motor OFF验收；仅会发送全零车体速度和只读查询。"""
import sys,time,struct,json
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent/'Host'))
from chassis import Chassis,load_config,save_log
car=None;record={'verified':False,'nonzero_command_sent':False}
try:
    car=Chassis(load_config());before=car.read_state()
    if before['motor_enabled'] or before['motor_power_present'] or any(before['pwm_ticks']):raise RuntimeError('零输出验收要求Motor OFF且12V断开')
    car.connect();samples=[];previous=None
    for n in range(150):
        car.speed(0.,0.);car.poll(.02);car.healthy()
        if car.state is not previous:
            previous=car.state;samples.append(dict(car.state))
        if car.state['motor_enabled'] or car.state['motor_power_present'] or any(car.state['pwm_ticks']):raise RuntimeError('出现使能/动力/非零输出')
    car.send(0x15,struct.pack('<I',car.nonce));car.wait(lambda:0x92 in car.replies)
    registers=struct.unpack('<15I',car.replies[0x92])
    if any(registers[4:12]):raise RuntimeError('CCR输出非零')
    if len(samples)<15 or not car.body or car.body['imu_flags']&7!=7 or car.body['target_vx'] or car.body['target_wz']:raise RuntimeError('遥测/IMU/目标状态未通过')
    record.update(verified=True,samples=samples,body=car.body,ccr=list(registers[4:12]),output_cap=list(struct.unpack('<IBHHH',car.replies[0x85])[1:]))
    print('ZERO_CHECK_PASS samples=%d IMU=ready PWM=0 CCR=0 cap=%.1f%%' %
          (len(samples), record['output_cap'][1]/10),flush=True)
except BaseException as e:record['error']=repr(e);raise
finally:
    if car:car.close()
    save_log('zero_check',record)
