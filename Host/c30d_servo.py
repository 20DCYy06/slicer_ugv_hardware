"""C30D servo protocol and a standalone connection that leaves wheel sessions alone."""
import os
import secrets
import select
import struct
import time

from chassis import device, load_config
from protocol import packet, Parser
from transport import Link


CAPABILITY_QUERY = 0x30
CAPABILITY_REPLY = 0x86
PULSE_COMMAND = 0x31
NEUTRAL_US = 1500
CAPABILITY = struct.Struct('<4sBBHH')


def decode_capability(payload):
    if len(payload) != CAPABILITY.size:
        raise RuntimeError('舵机能力回复长度不匹配，请核实固件')
    magic, version, pin, neutral, watchdog = CAPABILITY.unpack(payload)
    if ((magic, pin) not in ((b'B14P', 14), (b'B15P', 15))
            or (version, neutral, watchdog) != (1, NEUTRAL_US, 500)):
        raise RuntimeError('舵机能力接口不匹配，请核实固件')
    return pin, neutral, watchdog


def pulse_payload(value):
    return struct.pack('<H', value)


class Servo:
    def __init__(self, host_config=None):
        config = load_config() if host_config is None else load_config(host_config)
        self.path, expected = device(config)
        self.link = Link(self.path, expected, restore_on_close=False)
        self.parser = Parser()
        self.seq = secrets.randbits(16)
        self.moved = False

    def send(self, kind, data=b''):
        self.seq = (self.seq + 1) & 65535
        self.link.send(packet(kind, self.seq, data))

    def receive(self, wait=0):
        if not select.select([self.link.fd], [], [], wait)[0]:
            return []
        raw = os.read(self.link.fd, 4096)
        if not raw:
            raise RuntimeError('底盘USB已断开')
        return self.parser.feed(raw)

    def check(self, *, required_pin=None):
        # HELLO resets the wheel session, so query the servo capability directly.
        self.send(CAPABILITY_QUERY)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            for kind, _, data in self.receive(.05):
                if kind != CAPABILITY_REPLY:
                    continue
                pin, neutral, watchdog = decode_capability(data)
                if required_pin is not None and pin != required_pin:
                    raise RuntimeError(f'舵机接在 PB{required_pin}，固件报告 PB{pin}，请核实固件')
                self.pin = pin
                channel = 1 if pin == 14 else 2
                print(f'[READY] {self.path} PB{pin}/TIM12_CH{channel} 50Hz '
                      f'neutral={neutral}us watchdog={watchdog}ms', flush=True)
                return
        raise RuntimeError('未收到 C30D 舵机能力回复，请核实固件')

    def pulse(self, value):
        self.moved = True
        self.send(PULSE_COMMAND, pulse_payload(value))

    def close(self):
        try:
            if self.moved:
                self.send(PULSE_COMMAND, pulse_payload(NEUTRAL_US))
        finally:
            self.link.close()
