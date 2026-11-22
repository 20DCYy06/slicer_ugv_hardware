#!/usr/bin/env python3
"""Motor ON 且零输出时制造一次可恢复的底盘测试锁存。"""
import argparse
import sys
from pathlib import Path

sys.dont_write_bytecode = True
hardware = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(hardware / 'Host'))
sys.path.insert(0, str(hardware / 'Unlock'))
from chassis import Chassis, CONFIG, load_config, save_log
from chassis_commands import call_owner
from chassis_recovery import (RECOVERY_ARMED, RECOVERY_CAPABLE, TEST_LOCK, fault_mask,
                              paired_body_status, request_recovery, stopped)
from unlock import retry_with_dialout


def check_stopped(state):
    if not (state['motor_power_present'] and state['motor_enabled']
            and stopped(state['wheel_mm_s'], state['pwm_ticks'])):
        raise RuntimeError('锁定需要动力接通、Motor ON、车轮静止和零 PWM')


def lock_standalone(car):
    before = car.read_state()
    check_stopped(before)

    def status():
        if not car.body or not car.state:
            return None
        return paired_body_status(car.body['status'], car.state['tick'], car.body['tick'])

    car.wait(lambda: status() is not None, 1.)
    mask = fault_mask(status())
    if mask == TEST_LOCK and car.state['fault_latched']:
        return
    if mask or car.state['fault_latched']:
        raise RuntimeError('底盘已有故障锁存，请检查状态')
    if not status() & RECOVERY_CAPABLE:
        raise RuntimeError('测试锁存需要 v1.7 底盘固件')

    car.connect()
    car.wait(lambda: status() is not None, 1.)
    check_stopped(car.state)
    request_recovery(car, operation=2)
    car.wait(lambda: (status() or 0) & RECOVERY_ARMED, 1.)
    check_stopped(car.state)
    stop_seq = car.send(0x10, bytes(8))
    car.wait(lambda: status() is not None and car.state['ack'] == stop_seq, 1.)
    check_stopped(car.state)
    request_recovery(car, operation=1)
    car.wait(lambda: car.state['fault_latched'] and fault_mask(status()) == TEST_LOCK, 1.)
    check_stopped(car.state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(CONFIG))
    args = parser.parse_args()
    car = None
    record = {'action': 'test_lock', 'nonzero_command_sent': False}
    try:
        config = load_config(args.config)
        serial = config['serial']['serial_number']
        result = call_owner(serial, 'lock')
        if result is None:
            try:
                car = Chassis(config)
            except PermissionError as error:
                retry_with_dialout(error)
                raise
            except (OSError, RuntimeError):
                result = call_owner(serial, 'lock')
                if result is None:
                    raise
        if result is None:
            lock_standalone(car)
            result = True, '下位机已确认测试锁存'
        success, detail = result
        record.update(success=success, detail=detail)
        print('[LOCKED]' if success else '[ERROR]', detail, flush=True)
        return 0 if success else 3
    except (OSError, RuntimeError, ValueError) as error:
        print('[ERROR]', error, file=sys.stderr)
        record.update(success=False, error=str(error))
        return 3
    finally:
        if car is not None:
            car.close()
        save_log('test_lock', record)


if __name__ == '__main__':
    raise SystemExit(main())
