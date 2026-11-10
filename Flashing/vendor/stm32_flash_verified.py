#!/usr/bin/env python3
"""C30D VET6 verified sector-0 installer/rollback. ST AN3155.

Only sector 0 at 0x08000000..0x08003fff is writable. No mass erase,
no option writes, no protection commands. Full 512KiB readback each time.
External motor supply MUST remain disconnected and Motor OFF.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import struct
import termios
import time

from stm32_readonly import ReadOnlyROM, FLASH_BASE

ORIGINAL_SHA = '8dfeb8f87f21e290b15e0107a2c0901a56b1a8e397463e2b853a13d84f52ee3b'
SERIAL = '5CF0016377'
SIZE = 512 * 1024
SECTOR = 16 * 1024
ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def checked_images(directory, image, manifest):
    original = (directory / 'flash_read_1.bin').read_bytes()
    second = (directory / 'flash_read_2.bin').read_bytes()
    meta = json.loads((directory / 'manifest.json').read_text())
    if (len(original) != SIZE or original != second or sha(original) != ORIGINAL_SHA
            or not meta['backup_verified']):
        raise RuntimeError('Two complete original backups must match the pinned SHA256')
    binary = image.read_bytes()
    build = json.loads(manifest.read_text())
    if not 8 <= len(binary) <= SECTOR or len(binary) % 4:
        raise ValueError('Image must be aligned and fit sector 0')
    if (build['part'] != 'STM32F407VET6' or build['protocol'] != 'SLC2'
            or build['sectors_changed'] != [0] or sha(binary) != build['sha256']
            or len(binary) != build['size']):
        raise ValueError('Image manifest mismatch')
    sp, reset = struct.unpack('<II', binary[:8])
    if not 0x20000000 < sp <= 0x20020000 or not reset & 1 or not FLASH_BASE <= reset < FLASH_BASE + len(binary):
        raise ValueError('Invalid application vector table')
    new = binary + b'\xff' * (SECTOR - len(binary)) + original[SECTOR:]
    options = (directory / 'option_bytes_readonly.bin').read_bytes()
    if options.hex() != 'ffaa0055ffaa0055ffff0000ffff0000':
        raise ValueError('Original option bytes mismatch')
    return original, new, options


class SectorZeroROM(ReadOnlyROM):
    def flash_cmd(self, code):
        if code not in (0x31, 0x44):
            raise ValueError('Only write-memory and extended sector erase allowed')
        self.log.append(code)
        self.send(bytes((code, code ^ 255)))
        self.ack()

    def erase_sector_zero(self):
        self.flash_cmd(0x44)
        # N-1=0, sector index=0, XOR=0. NEVER 0xffff mass erase.
        self.send(bytes(5))
        if self.read(1, timeout=15) != b'\x79':
            raise RuntimeError('Sector-0 erase not acknowledged; stop, do not unprotect')

    def write_sector_zero(self, address, data):
        if not (FLASH_BASE <= address < FLASH_BASE + SECTOR and
                address + len(data) <= FLASH_BASE + SECTOR and
                address % 4 == 0 and 4 <= len(data) <= 256 and len(data) % 4 == 0):
            raise ValueError('Write outside sector 0/alignment forbidden')
        self.flash_cmd(0x31)
        self.send(with_xor(struct.pack('>I', address)))
        self.ack()
        self.send(with_xor(bytes((len(data) - 1,)) + data))
        self.ack()

def with_xor(data):
    check = 0
    for b in data:
        check ^= b
    return data + bytes((check,))


def read_flash(rom, label):
    data = bytearray()
    for offset in range(0, SIZE, 256):
        data.extend(rom.read_memory(FLASH_BASE + offset, 256))
        if (offset + 256) % (64 * 1024) == 0:
            print('%s %d/512 KiB' % (label, (offset + 256) // 1024), flush=True)
    return bytes(data)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/ttyACM0')
    p.add_argument('--baud', type=int, choices=(76800, 115200), default=76800)
    p.add_argument('--backup-dir', type=Path, default=ROOT / 'firmware/backups/original_VET6_20261020_011454')
    p.add_argument('--image', type=Path, default=ROOT / 'firmware/c30d_vet6/build/c30d_vet6.bin')
    p.add_argument('--manifest', type=Path, default=ROOT / 'firmware/c30d_vet6/build/manifest.json')
    p.add_argument('--mode', choices=('install', 'restore'), required=True)
    p.add_argument('--power-off-confirmed', action='store_true')
    p.add_argument('--recover-partial-sector-zero', action='store_true',
                   help='Explicitly permit recovery after interrupted sector-0 programming; other sectors still MUST match')
    a = p.parse_args()
    if not a.power_off_confirmed:
        p.error('12V disconnected AND Motor OFF required')
    original, new, options = checked_images(a.backup_dir, a.image, a.manifest)
    expected = new if a.mode == 'install' else original
    journal_dir = ROOT / 'firmware/flash_logs'
    journal_dir.mkdir(exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    journal = journal_dir / (stamp + '_' + a.mode + '.json')
    log = {'mode': a.mode, 'expected_full_sha256': sha(expected), 'verified': False,
           'original_sha256': ORIGINAL_SHA, 'sectors_changed': [0],
           'option_bytes_written': False, 'erased': False}

    def save():
        journal.write_text(json.dumps(log, indent=2), encoding='utf-8')

    save()
    rom = SectorZeroROM(a.device, SERIAL, a.baud)
    try:
        rom.enter_boot('levels')
        rom.sync()
        info = rom.identify()
        print('ROM:', info, flush=True)
        if (info['chip_id'] != '0x413' or not {'0x31', '0x44', '0x11'}.issubset(info['supported_commands'])):
            raise RuntimeError('Unexpected MCU/ROM capabilities; no erase allowed')
        if rom.read_memory(0x1fffc000, 16) != options:
            raise RuntimeError('Option bytes changed; no erase allowed')
        current = read_flash(rom, 'Before')
        log['before_sha256'] = sha(current)
        save()
        if current[SECTOR:] != original[SECTOR:]:
            raise RuntimeError('Non-sector-0 contents differ from original: refuse write')
        prior = set()
        for entry in journal_dir.glob('*_install.json'):
            record = json.loads(entry.read_text())
            if (record.get('verified') and record.get('original_sha256') == ORIGINAL_SHA
                    and record.get('sectors_changed') == [0]
                    and record.get('option_bytes_written') is False):
                prior.add(record.get('readback_sha256'))
        if current not in (original, new) and sha(current) not in prior and not a.recover_partial_sector_zero:
            raise RuntimeError('Unknown sector-0 image: refuse without explicit partial recovery')
        if current != expected:
            print('ERASE sector 0 ONLY; 12V must remain DISCONNECTED.', flush=True)
            rom.erase_sector_zero()
            log['erased'] = True
            save()
            for offset in range(0, SECTOR, 256):
                chunk = expected[offset:offset+256]
                if chunk != b'\xff' * 256:
                    rom.write_sector_zero(FLASH_BASE + offset, chunk)
            print('Write completed; verifying WHOLE 512KiB Flash...', flush=True)
            readback = read_flash(rom, 'Verify')
        else:
            print('Already desired image; no erase/write.', flush=True)
            readback = current
        if readback != expected or rom.read_memory(0x1fffc000, 16) != options:
            raise RuntimeError('Verification FAILED; keep motor power disconnected')
        log['verified'] = True
        log['readback_sha256'] = sha(readback)
        print('FULL FLASH + OPTION-BYTE VERIFY PASS:', sha(readback), flush=True)
    except BaseException as exc:
        log['error'] = repr(exc)
        raise
    finally:
        log['commands'] = sorted(set(hex(c) for c in rom.log))
        save()
        rom.close()
        print('Journal:', journal, flush=True)


if __name__ == '__main__':
    main()
