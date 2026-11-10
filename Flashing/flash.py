#!/usr/bin/env python3
"""只在Workspace2中备份、归档和烧录。仅允许sector0，不实现解除保护/选项写入。"""
import argparse,json,hashlib,os,re,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path[:0]=[str(HERE/'vendor'),str(ROOT/'Hardware/Host')]
from stm32_flash_verified import SectorZeroROM,read_flash
from chassis import load_config,device
BASE=0x08000000;SIZE=512*1024;SECTOR=16*1024

def sha(data):return hashlib.sha256(data).hexdigest()
def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--version',default='v1.0');p.add_argument('--flash',action='store_true');p.add_argument('--power-off-confirmed',action='store_true');a=p.parse_args()
    if not a.power_off_confirmed:p.error('需要确认12V物理断开、Motor OFF、仅USB供电')
    if not re.fullmatch(r'v\d+\.\d+(?:\.\d+)?',a.version):p.error('版本格式应为v1.0或v1.0.1')
    version=ROOT/'Hardware/Firmware'/a.version;binary=(version/'src/build/c30d_body.bin').read_bytes();meta=json.loads((version/'src/build/manifest.json').read_text())
    if meta['part']!='STM32F407VET6' or meta['sectors_changed']!=[0] or sha(binary)!=meta['sha256'] or len(binary)!=meta['size'] or not 8<=len(binary)<=SECTOR or len(binary)%4:raise ValueError('固件manifest/大小不匹配')
    cfg=load_config();port,expected=device(cfg)
    folder=ROOT/'Hardware/Backups'/('board_'+expected[2]+'_'+str(time.time_ns()));folder.mkdir()
    log={'version':a.version,'usb_serial':expected[2],'device':port,'verified':False,'erased':False,'option_bytes_written':False,'image_sha256':sha(binary)}
    rom=None
    try:
        rom=SectorZeroROM(port,expected[2],76800);rom.enter_boot('levels');rom.sync();info=rom.identify();print('ROM',json.dumps(info),flush=True)
        if info['chip_id']!='0x413' or not {'0x11','0x31','0x44'}.issubset(info['supported_commands']):raise RuntimeError('芯片/ROM能力不匹配')
        # 在同一独占ROM会话中完成两份完整备份后再写入，无第二个串口持有者。
        rom.backup(folder/'original',info,part_number='STM32F407VET6')
        original=(folder/'original/flash_read_1.bin').read_bytes();second=(folder/'original/flash_read_2.bin').read_bytes();options=(folder/'original/option_bytes_readonly.bin').read_bytes()
        if len(original)!=SIZE or original!=second or options[1]!=0xaa or options[3]!=0x55:raise RuntimeError('备份不一致或存在读保护；不解除保护')
        log['before_sha256']=sha(original);log['backup_verified']=True
        if not a.flash:print('BACKUP ONLY VERIFIED',flush=True);return
        target=binary+b'\xff'*(SECTOR-len(binary))+original[SECTOR:]
        log['expected_sha256']=sha(target);save(folder/'install.json',log)
        if original!=target:
            if rom.read_memory(0x1fffc000,16)!=options:raise RuntimeError('选项字节发生变化')
            print('ERASE SECTOR 0 ONLY',flush=True);log['erase_started']=True;save(folder/'install.json',log)
            rom.erase_sector_zero();log['erased']=True;save(folder/'install.json',log)
            for offset in range(0,len(binary),256):rom.write_sector_zero(BASE+offset,binary[offset:offset+256])
            print('WRITE COMPLETE; VERIFY 512 KiB',flush=True)
        readback=read_flash(rom,'Verify');(folder/'readback.bin').write_bytes(readback)
        if readback!=target or rom.read_memory(0x1fffc000,16)!=options:raise RuntimeError('整片Flash或选项字节校验失败，保持动力断电')
        log.update(verified=True,sectors_changed=[0],readback_sha256=sha(readback),options_unchanged=True)
        print('FULL FLASH AND OPTIONS VERIFY PASS',sha(readback),flush=True)
    except BaseException as e:log['error']=repr(e);raise
    finally:
        if rom:log['rom_commands']=sorted(set(hex(c) for c in rom.log));rom.close()
        save(folder/'install.json',log);(ROOT/'Hardware/last_install.json').write_text(json.dumps({'directory':str(folder),**log},indent=2));print('LOG',folder,flush=True)
if __name__=='__main__':main()
