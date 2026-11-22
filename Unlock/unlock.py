#!/usr/bin/env python3
"""--check 只收遥测。--unlock 在零输出下清锁，Motor ON 时仅清除通信类锁存。"""
import argparse
import errno
import grp
import json
import os
import pwd
import shlex
import stat
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Host'))
from chassis_commands import OWNER_SERIAL_ENV, call_owner


def owner_unlock(serial):
    result = call_owner(serial, 'unlock')
    if result is not None:
        success, detail = result
        print('[UNLOCKED]' if success else '[ERROR]', detail, flush=True)
        return 0 if success else 3
    return None


if __name__ == '__main__' and sys.argv[1:] == ['--unlock'] and os.environ.get(OWNER_SERIAL_ENV):
    # The driver invokes this mode asynchronously. It never opens a serial port.
    try:
        code = owner_unlock(os.environ[OWNER_SERIAL_ENV])
        if code is None:
            raise RuntimeError('底盘驱动已退出')
    except (OSError, RuntimeError, ValueError) as error:
        print('[ERROR]', error, file=sys.stderr)
        code = 3
    raise SystemExit(code)


from chassis import Chassis,load_config,save_log,CONFIG
from chassis_recovery import (LinkRecovery, RECOVERY_TELEMETRY_TIMEOUT, fault_mask,
                              paired_body_status, recoverable)


def retry_with_dialout(error):
    if error.errno != errno.EACCES or not error.filename:
        return
    try:
        device = os.stat(error.filename)
        dialout = grp.getgrnam('dialout').gr_gid
        if not stat.S_ISCHR(device.st_mode) or device.st_gid != dialout:
            return
        if dialout in (os.getegid(), *os.getgroups()):
            return
        user = pwd.getpwuid(os.geteuid())
        if dialout not in os.getgrouplist(user.pw_name, user.pw_gid):
            return
        command = shlex.join([sys.executable, '-s', *sys.argv])
        os.execlp('sg', 'sg', 'dialout', '-c', 'exec ' + command)
    except (KeyError, OSError):
        # Preserve the original serial error if the device or group is unavailable.
        return


def unlock_powered(car, before):
    if not before.get('motor_power_present'):
        raise RuntimeError('Motor ON 清锁要求动力电源接通')
    if not before['fault_latched']:
        return before
    car.wait(lambda: car.body is not None, 1.)
    if not recoverable(car.body['status']):
        raise RuntimeError('当前固件或锁存原因不支持 Motor ON 清锁，请检查故障后切到 Motor OFF')
    car.connect()
    recovery = LinkRecovery()

    def cleared():
        now = time.monotonic()
        if (car.state is None or car.body is None or now-car.last_rx > RECOVERY_TELEMETRY_TIMEOUT
                or now-car.last_body_rx > RECOVERY_TELEMETRY_TIMEOUT):
            return False
        state, body_status = car.state, car.body['status']
        if not state['motor_enabled'] or not state['motor_power_present'] or any(state['pwm_ticks']):
            raise RuntimeError('Motor ON 清锁期间的电源、使能或 PWM 状态异常')
        if fault_mask(body_status) and not recoverable(body_status):
            raise RuntimeError('检测到需要人工排查的锁存，清锁已停止')
        matched = paired_body_status(body_status, state.get('tick'), car.body.get('tick'))
        _, recovered = recovery.update(car, now, state['flags'], matched, last_body_status=body_status)
        return recovered

    car.wait(cleared, 5.)
    return car.state


def main():
    p=argparse.ArgumentParser(description=__doc__);g=p.add_mutually_exclusive_group()
    g.add_argument('--check',action='store_true');g.add_argument('--unlock',action='store_true');p.add_argument('--config',default=str(CONFIG));a=p.parse_args()
    car=None;record={'action':'unlock' if a.unlock else 'check','nonzero_command_sent':False};code=0
    try:
        config = load_config(a.config)
        serial = config['serial']['serial_number'] if a.unlock else None
        if serial is not None:
            result = owner_unlock(serial)
            if result is not None:
                record['via_driver']=True
                record['success']=result==0
                return result
        try:
            car = Chassis(config)
        except PermissionError as error:
            retry_with_dialout(error)
            raise
        except (OSError,RuntimeError):
            if serial is not None:
                result = owner_unlock(serial)
                if result is not None:
                    record.update(via_driver=True,success=result==0)
                    return result
            raise
        before=car.read_state();record['before']=before
        print('[STATUS]',json.dumps(before,ensure_ascii=False),flush=True)
        if a.unlock:
            if any(before['pwm_ticks']):
                raise RuntimeError('请先停车并确认四轮 PWM 为零')
            if before['motor_enabled']:
                after=unlock_powered(car,before)
            else:
                car.connect();car.state=None;after=car.read_state()
            if (after['motor_enabled']!=before['motor_enabled'] or after['fault_latched']
                    or any(after['pwm_ticks'])):
                raise RuntimeError('清锁后状态未通过')
            record['after']=after
            motor='ON' if after['motor_enabled'] else 'OFF'
            print(f'[UNLOCKED] fault=0，PWM=0，Motor={motor}。',flush=True)
        elif before['fault_latched']:code=2
        record['success']=code==0
    except (RuntimeError,OSError,TimeoutError,ValueError) as e:
        print('[ERROR]',e,file=sys.stderr);record['error']=str(e);code=3
    finally:
        if car:car.close()
        save_log('unlock',record)
    return code
if __name__=='__main__':raise SystemExit(main())
