"""Shared SLC2 recovery for the standalone tool and both navigation drivers."""
from concurrent.futures import Future, TimeoutError
from queue import Empty, Full, Queue
import struct
from threading import Lock


RECOVERY_ARMED = 0x10
RECOVERY_CAPABLE = 0x40
RECOVERY_ALLOWED = 0x80
TEST_LOCK = 0x20
RECOVERABLE_FAULTS = 0x01 | 0x10 | TEST_LOCK
MOTOR_POWER_CONFIGURED = 0x38
RECOVERY_TELEMETRY_TIMEOUT = .3
MAX_TELEMETRY_GAP = 3.


def fault_mask(body_status):
    return (body_status or 0) >> 8


def recoverable(body_status):
    mask = fault_mask(body_status)
    return bool(body_status and body_status & RECOVERY_CAPABLE and mask
                and not mask & ~RECOVERABLE_FAULTS)


def paired_body_status(body_status, wheel_tick, body_tick):
    if (body_status or 0) & RECOVERY_CAPABLE and wheel_tick != body_tick:
        return None
    return body_status


def request_recovery(car, operation=0):
    car.send(0x19, struct.pack('<IB', car.nonce, operation))


def stopped(wheel_speed, pwm):
    return (wheel_speed is not None and pwm is not None and len(wheel_speed) == len(pwm) == 4
            and not any(pwm) and all(abs(speed) <= 5 for speed in wheel_speed))


class LinkRecovery:
    def __init__(self, arm_watchdog=False):
        self.active = False
        self.arm_watchdog = arm_watchdog
        self.next_arm = 0.
        self.capability_seen = False
        self.need_body_after = float('-inf')
        self.next_request = 0.
        self.next_spawn = 0.
        self.requests = Queue(maxsize=1)
        self.unlock_requests = Queue(maxsize=1)
        self.request_lock = Lock()
        self.accepting_requests = True
        self.start_unlock_command = None
        self.unlock_in_progress = None
        self.unlock_deadline = 0.
        self.test_in_progress = None
        self.test_deadline = 0.
        self.test_stop_seq = None

    def queue_request(self, queue):
        result = Future()
        with self.request_lock:
            if not self.accepting_requests:
                result.set_result((False, '底盘驱动正在退出'))
            else:
                try:
                    queue.put_nowait(result)
                except Full:
                    result.set_result((False, '已有同类底盘请求正在排队'))
        return result

    def queue_test(self):
        return self.queue_request(self.requests)

    def queue_unlock(self):
        return self.queue_request(self.unlock_requests)

    def stop_requests(self):
        result = False, '底盘驱动正在退出'
        with self.request_lock:
            self.accepting_requests = False
            for queue in (self.requests, self.unlock_requests):
                try:
                    pending = queue.get_nowait()
                except Empty:
                    continue
                if pending.set_running_or_notify_cancel():
                    pending.set_result(result)
        for pending in (self.test_in_progress, self.unlock_in_progress):
            if pending is not None and not pending.done():
                pending.set_result(result)

    def start_test_service(self, rospy):
        from std_srvs.srv import Trigger, TriggerResponse

        def handle(_request):
            result = self.queue_test()
            try:
                return TriggerResponse(*result.result(timeout=2.))
            except TimeoutError:
                result.cancel()
                return TriggerResponse(False, 'No confirmed MCU lock telemetry')

        return rospy.Service('~test_chassis_lock', Trigger, handle)

    def wait_for_telemetry(self, now, body_status, wheel_age):
        if (not self.arm_watchdog or not (body_status or 0) & RECOVERY_CAPABLE
                or wheel_age >= MAX_TELEMETRY_GAP):
            return False
        self.active = True
        self.need_body_after = now
        return True

    def update(self, car, now, flags, body_status, *, last_body_status=None,
               stationary=False, ack=None):
        """Return (hold zero output, recovered this cycle) using fresh telemetry."""
        supported = bool(body_status and body_status & RECOVERY_CAPABLE)
        self.capability_seen |= supported or bool((last_body_status or 0) & RECOVERY_CAPABLE)
        ready = (supported and flags & MOTOR_POWER_CONFIGURED == MOTOR_POWER_CONFIGURED
                 and not flags & 0xc0 and body_status & 7 == 7)
        fault = bool(flags & 4 or fault_mask(body_status))
        armed = bool(body_status and body_status & RECOVERY_ARMED)
        if self.arm_watchdog and ready and not fault and not armed and now >= self.next_arm:
            request_recovery(car, operation=2)
            self.next_arm = now + .25
        try:
            test = self.requests.get_nowait()
        except Empty:
            test = None
        if test is not None and test.set_running_or_notify_cancel():
            if not ready or fault or self.active or not armed or not stationary:
                test.set_result((False, '锁定需要底盘就绪、Motor ON、动力接通、车轮静止和零 PWM'))
            else:
                self.active = True
                self.test_in_progress = test
                self.test_stop_seq = car.send(0x10, bytes(8))
                self.test_deadline = now + 1.
                return True, False

        if self.test_in_progress is not None:
            if flags & 4 and fault_mask(body_status) & TEST_LOCK:
                if recoverable(body_status):
                    result = True, '下位机已确认测试锁存，驱动将自动清锁'
                else:
                    result = False, '测试锁存同时出现需要人工排查的故障'
                self.test_in_progress.set_result(result)
                self.test_in_progress = None
                self.test_stop_seq = None
            elif now >= self.test_deadline:
                self.test_in_progress.set_result((False, 'MCU did not confirm test lock'))
                self.test_in_progress = None
                self.test_stop_seq = None
            else:
                if (self.test_stop_seq is not None and ack is not None and ready and not fault
                        and stationary and ack == self.test_stop_seq):
                    request_recovery(car, operation=1)
                    self.test_stop_seq = None
                    self.test_deadline = now + 1.
                return True, False

        self.update_unlock_request(now, ready, stationary, fault, body_status)
        if fault:
            if self.capability_seen:
                self.active = True
            if ready and flags & 4 and recoverable(body_status) and body_status & RECOVERY_ALLOWED:
                if self.unlock_in_progress is not None or self.start_unlock_command is None:
                    if now >= self.next_request:
                        request_recovery(car)
                        self.next_request = now + .25
                elif now >= self.next_spawn:
                    try:
                        self.start_unlock_command()
                    except OSError as error:
                        print(f'[C30D] 无法启动 unlock.py: {error}', flush=True)
                    self.next_spawn = now + .25
            return self.active, False
        if self.active and ready:
            self.active = False
            return True, True
        return self.active, False

    def update_unlock_request(self, now, ready, stationary, fault, body_status):
        if self.unlock_in_progress is None:
            try:
                request = self.unlock_requests.get_nowait()
            except Empty:
                return
            if not request.set_running_or_notify_cancel():
                return
            self.unlock_in_progress = request
            self.unlock_deadline = now + 3.
        if fault_mask(body_status) and not recoverable(body_status):
            result = False, '该故障锁存需要人工排查'
        elif ready and not fault:
            result = (True, 'fault=0，PWM=0，Motor=ON') if stationary else (False, '请先停车并确认 PWM 为零')
        elif now >= self.unlock_deadline:
            result = False, '等待底盘清锁确认超时'
        else:
            return
        self.unlock_in_progress.set_result(result)
        self.unlock_in_progress = None
