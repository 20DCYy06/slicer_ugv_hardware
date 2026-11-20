#!/usr/bin/env python3
"""刷入PB14候选前，先备份并检查当前板仍是v1.2；不覆盖未知固件。"""
from pathlib import Path
import sys
import flash
BASE=Path(__file__).resolve().parents[1]
BASELINE=(BASE/'Firmware/v1.2/bin/firmware.bin').read_bytes()
class BaselineROM(flash.SectorZeroROM):
    def backup(self,directory,info,**kwargs):
        result=super().backup(directory,info,**kwargs)
        original=(directory/'flash_read_1.bin').read_bytes()
        expected=BASELINE+b'\xff'*(flash.SECTOR-len(BASELINE))
        if original[:flash.SECTOR]!=expected:
            raise RuntimeError('现板与v1.2基线不同，已保留双备份，禁止擦写未知改动')
        print('CURRENT_BOARD_V1_2_BASELINE_MATCH',flush=True)
        return result
flash.SectorZeroROM=BaselineROM
if __name__=='__main__':
    if '--power-off-confirmed' not in sys.argv:raise SystemExit('需要--power-off-confirmed')
    sys.argv=[sys.argv[0],'--version','v1.4','--flash','--power-off-confirmed']
    flash.main()
