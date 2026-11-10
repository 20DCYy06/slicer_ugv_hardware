#!/usr/bin/env python3
"""STM32 ROM USART identification/backup ONLY (ST AN3155 / AN2606).

No write, erase, protect/unprotect, GO, RAM-loader or option-write implementation.
Auto-reset is permitted only with external motor power disconnected.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import select
import struct
import termios
import time

from test_factory_uart import identity

ALLOWED_COMMANDS = frozenset((0x00, 0x02, 0x11))
FLASH_BASE = 0x08000000


class ReadOnlyROM:
    def __init__(self, device, serial_number, baud):
        self.fd = None
        self.attrs = None
        self.modem = None
        self.boot_mode = None
        self.log = []
        if identity(device) != ('1a86', '55d4', serial_number):
            raise RuntimeError('USB identity mismatch; refusing RTK/unknown port')
        self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
        try:
            if os.fstat(self.fd).st_rdev != os.stat(device).st_rdev:
                raise RuntimeError('Device changed while opening')
            if identity(device) != ('1a86', '55d4', serial_number):
                raise RuntimeError('Identity changed while opening')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.attrs = termios.tcgetattr(self.fd)
            modem = bytearray(4)
            fcntl.ioctl(self.fd, termios.TIOCMGET, modem, True)
            self.modem = struct.unpack('I', modem)[0]
            a = termios.tcgetattr(self.fd)
            a[0] = a[1] = a[3] = 0
            a[2] = ((a[2] & ~(termios.CSIZE | termios.PARODD | termios.CSTOPB |
                      getattr(termios, 'CRTSCTS', 0) | termios.HUPCL)) |
                     termios.CS8 | termios.PARENB | termios.CREAD | termios.CLOCAL)
            a[6][termios.VMIN] = a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            b = bytearray(44)
            fcntl.ioctl(self.fd, 0x802c542a, b, True)
            fields = list(struct.unpack('=IIIIB19BII', b))
            fields[2] = (fields[2] & ~0x100f) | 0x1000
            fields[-2] = fields[-1] = baud
            fcntl.ioctl(self.fd, 0x402c542b,
                        struct.pack('=IIIIB19BII', *fields))
        except BaseException:
            self.close()
            raise

    def lines(self, dtr, rts):
        set_bits = (termios.TIOCM_DTR if dtr else 0) | (termios.TIOCM_RTS if rts else 0)
        all_bits = termios.TIOCM_DTR | termios.TIOCM_RTS
        fcntl.ioctl(self.fd, termios.TIOCMBIC, struct.pack('I', all_bits & ~set_bits))
        fcntl.ioctl(self.fd, termios.TIOCMBIS, struct.pack('I', set_bits))

    def enter_boot(self, mode):
        # C30D manual: DTR-low reset / RTS-high BOOT0. `levels` follows host
        # logical levels, `active-low` accounts for inverted USB modem outputs.
        # Both only change volatile pins, never flash or option bytes.
        self.boot_mode = mode
        low, high = (False, True) if mode == 'levels' else (True, False)
        self.lines(low, high)
        time.sleep(.15)
        self.lines(high, high)
        time.sleep(.20)
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read(self, count, timeout=2):
        data = bytearray()
        deadline = time.monotonic() + timeout
        while len(data) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('ROM RX timeout (%d/%d bytes, hex=%s)' %
                                   (len(data), count, data.hex()))
            if select.select([self.fd], [], [], remaining)[0]:
                chunk = os.read(self.fd, count - len(data))
                if not chunk:
                    raise RuntimeError('USB disconnected')
                data.extend(chunk)
        return bytes(data)

    def send(self, data):
        remaining = memoryview(data)
        deadline = time.monotonic() + 1
        while remaining:
            if time.monotonic() >= deadline:
                raise TimeoutError('USB TX timeout')
            if select.select([], [self.fd], [], .05)[1]:
                try:
                    count = os.write(self.fd, remaining)
                except BlockingIOError:
                    continue
                remaining = remaining[count:]

    def ack(self):
        value = self.read(1)
        if value == b'\x1f':
            raise RuntimeError('ROM NACK: stop; NEVER unprotect or erase')
        if value != b'\x79':
            raise RuntimeError('Not ROM ACK: ' + value.hex())

    def sync(self):
        self.send(b'\x7f')
        self.ack()

    def cmd(self, code):
        if code not in ALLOWED_COMMANDS:
            raise ValueError('Non-read-only command forbidden')
        self.log.append(code)
        self.send(bytes((code, code ^ 0xff)))
        self.ack()

    def identify(self):
        self.cmd(0x00)
        count = self.read(1)[0]
        if not 2 <= count <= 40:
            raise RuntimeError('Invalid GET length: ' + str(count))
        data = self.read(count + 1)
        self.ack()
        self.cmd(0x02)
        count_id = self.read(1)[0]
        if count_id != 1:
            raise RuntimeError('Unexpected chip-ID length')
        chip = int.from_bytes(self.read(count_id + 1), 'big')
        self.ack()
        return {'bootloader_version': hex(data[0]), 'chip_id': hex(chip),
                'supported_commands': [hex(x) for x in data[1:]],
                'read_memory_supported': 0x11 in data[1:]}

    def read_memory(self, address, length):
        if not 1 <= length <= 256:
            raise ValueError('Read length must be 1..256')
        # Only known F405/F407/F415/F417 regions; no arbitrary peripheral access.
        allowed = (FLASH_BASE <= address and address + length <= FLASH_BASE + 1024*1024)
        allowed |= 0x1fff7a10 <= address and address + length <= 0x1fff7a24
        allowed |= 0x1fffc000 <= address and address + length <= 0x1fffc010
        if not allowed:
            raise ValueError('Address outside read allowlist')
        self.cmd(0x11)
        data = struct.pack('>I', address)
        check = 0
        for byte in data:
            check ^= byte
        self.send(data + bytes((check,)))
        self.ack()  # Never send length after missing/NACK address response.
        n = length - 1
        self.send(bytes((n, n ^ 0xff)))
        self.ack()
        return self.read(length)

    def backup(self, directory, info, part_number=None):
        if info['chip_id'] != '0x413' or not info['read_memory_supported']:
            raise RuntimeError('Unknown family/read unavailable; do not guess flash layout')
        vector = self.read_memory(FLASH_BASE, 256)
        sp, reset = struct.unpack('<II', vector[:8])
        if not ((0x20000000 < sp <= 0x20020000 or 0x10000000 < sp <= 0x10010000)
                and reset & 1 and FLASH_BASE <= reset < FLASH_BASE + 1024*1024):
            raise RuntimeError('Implausible vector table; no verified backup')
        if part_number == 'STM32F407VET6':
            # User read the physical marking; verified against ST's VE product
            # specification. ROM v3.1 on this board NACKs the size-register area.
            size_kib = 512
            info['part_number'] = part_number
            info['capacity_source'] = 'User chip marking + https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html'
        elif part_number is None:
            size_kib = struct.unpack('<H', self.read_memory(0x1fff7a22, 2))[0]
            info['capacity_source'] = 'Flash-size register'
        else:
            raise ValueError('Unverified part/capacity mapping')
        if size_kib not in (512, 1024):
            raise RuntimeError('Unexpected flash size; refusing guessed dump: %d' % size_kib)
        size = size_kib * 1024
        directory.mkdir(parents=True, exist_ok=False)
        info.update({'flash_base': hex(FLASH_BASE), 'flash_size_bytes': size,
                     'initial_sp': hex(sp), 'reset_vector': hex(reset),
                     'timestamp_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                     'backup_verified': False, 'restore_tested': False})
        (directory / 'manifest.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
        for number in (1, 2):
            path = directory / ('flash_read_%d.bin' % number)
            digest = hashlib.sha256()
            with path.open('xb') as out:
                for offset in range(0, size, 256):
                    data = self.read_memory(FLASH_BASE + offset, min(256, size-offset))
                    out.write(data)
                    digest.update(data)
                    if offset % (64*1024) == 0:
                        print('Read %d: %d/%d KiB' % (number, offset//1024, size_kib), flush=True)
                out.flush()
                os.fsync(out.fileno())
            info['sha256_read_%d' % number] = digest.hexdigest()
            (directory / 'manifest.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
        if info['sha256_read_1'] != info['sha256_read_2']:
            raise RuntimeError('Two reads differ: backup NOT verified')
        option_bytes = self.read_memory(0x1fffc000, 16)
        (directory / 'option_bytes_readonly.bin').write_bytes(option_bytes)
        info['option_bytes_hex'] = option_bytes.hex()
        info['backup_verified'] = True
        (directory / 'manifest.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
        print('BACKUP VERIFIED (two identical reads); RESTORE NOT TESTED:', directory, flush=True)
        print(json.dumps(info, indent=2), flush=True)

    def close(self):
        if self.fd is None:
            return
        try:
            if self.boot_mode is not None:
                # C30D SN 5CF0016377 measured normal-boot sequence:
                # DTR=0/RTS=1, then DTR=0/RTS=0. The previous DTR-only
                # pulse left the device in ROM. Do not restore stale BOOT lines.
                self.lines(False, True)
                time.sleep(.15)
                self.lines(False, False)
                time.sleep(.15)
                self.modem = 0
            if self.modem is not None:
                all_bits = termios.TIOCM_DTR | termios.TIOCM_RTS
                fcntl.ioctl(self.fd, termios.TIOCMBIC, struct.pack('I', all_bits & ~self.modem))
                fcntl.ioctl(self.fd, termios.TIOCMBIS, struct.pack('I', all_bits & self.modem))
            if self.attrs is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.attrs)
        finally:
            os.close(self.fd)
            self.fd = None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', required=True)
    p.add_argument('--serial-number', required=True)
    p.add_argument('--baud', type=int, choices=(76800, 115200), default=76800)
    p.add_argument('--boot-mode', choices=('levels', 'active-low'))
    p.add_argument('--power-off-confirmed', action='store_true')
    p.add_argument('--backup-dir', type=Path)
    p.add_argument('--part-number', choices=('STM32F407VET6',),
                   help='Physical marking confirmed by user; fixes capacity at 512KiB')
    a = p.parse_args()
    if not a.power_off_confirmed:
        p.error('External motor power MUST be OFF; --power-off-confirmed required')
    rom = ReadOnlyROM(a.device, a.serial_number, a.baud)
    try:
        if a.boot_mode:
            rom.enter_boot(a.boot_mode)
        rom.sync()
        print('ROM sync ACK', flush=True)
        info = rom.identify()
        print(json.dumps(info, indent=2), flush=True)
        if a.backup_dir:
            rom.backup(a.backup_dir, info, a.part_number)
    finally:
        print('Commands used (read-only allowlist):', sorted(set(hex(c) for c in rom.log)), flush=True)
        rom.close()


if __name__ == '__main__':
    main()
