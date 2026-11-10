#!/usr/bin/env python3
"""无 ROS/串口副作用的协议、四轮差速运动学与里程计核心。单位：SI。"""
import math
import struct

MAGIC = b'\xa5\x5a'
VERSION = 1
HELLO, HELLO_REPLY, SPEED, ENCODERS = 0x01, 0x81, 0x10, 0x90


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc


def packet(kind, seq, payload=b''):
    if len(payload) > 64:
        raise ValueError('payload too large')
    body = struct.pack('<BBBH', VERSION, kind, len(payload), seq & 0xffff) + payload
    return MAGIC + body + struct.pack('<H', crc16(body))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b''
                break
            del self.buffer[:index]
            if len(self.buffer) < 7:
                break
            if self.buffer[2] != VERSION or self.buffer[4] > 64:
                del self.buffer[0]
                continue
            size = 9 + self.buffer[4]
            if len(self.buffer) < size:
                break
            if crc16(self.buffer[2:size-2]) != struct.unpack_from('<H', self.buffer, size-2)[0]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            frames.append((self.buffer[3], struct.unpack_from('<H', self.buffer, 5)[0], bytes(self.buffer[7:size-2])))
            del self.buffer[:size]
        return frames

