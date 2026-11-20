#!/usr/bin/env python3
"""复用原有双备份/sector0烧录器，擦除前要求现板与v1.1基线逐字节一致。"""
import hashlib
from pathlib import Path
import sys
import flash
BASE=Path(__file__).resolve().parents[1]
BASELINE=(BASE/'Firmware/v1.1/bin/firmware.bin').read_bytes()
class BaselineROM(flash.SectorZeroROM):
    def backup(self, directory, info, **kwargs):
        result=super().backup(directory,info,**kwargs)
        original=(directory/'flash_read_1.bin').read_bytes()
        expected=BASELINE+b'\xff'*(flash.SECTOR-len(BASELINE))
        if original[:flash.SECTOR]!=expected:
            raise RuntimeError('现板sector0与v1.1基线不同，保留完整备份且禁止擦写；需先确认队友改动')
        print('CURRENT_BOARD_V1_1_BASELINE_MATCH: four-motor baseline preserved',flush=True)
        return result
flash.SectorZeroROM=BaselineROM
if __name__=='__main__':
    if '--power-off-confirmed' not in sys.argv:raise SystemExit('需要--power-off-confirmed')
    sys.argv=[sys.argv[0],'--version','v1.2','--flash','--power-off-confirmed']
    flash.main()
