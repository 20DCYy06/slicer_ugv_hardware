"""仅准备Workspace2内的PB14候选版本，不访问串口、不烧录。"""
from pathlib import Path
import hashlib
import json
import shutil
import tarfile
import time

root=Path(__file__).resolve().parents[1]
base=root/'Hardware/Firmware/v1.2'
new=root/'Hardware/Firmware/v1.4'
assert not new.exists(), 'v1.4已存在，请先检查，拒绝覆盖'
backup=root/'Hardware/Backups'/('before_pb14_'+str(time.time_ns())+'.tar.gz')
with tarfile.open(backup,'w:gz') as archive:
    for name in ('tools/manual/b15_servo/keyboard.py','tools/manual/b15_servo/README.md','tools/manual/b15_servo/config.json',
                 'tools/manual/combined_gamepad/combined_gamepad.py','tools/manual/combined_gamepad/test_combined_gamepad.py','Hardware/README.md'):
        archive.add(root/name,arcname=name)
new.mkdir()
shutil.copytree(base/'src',new/'src',ignore=shutil.ignore_patterns('build','__pycache__'))
p=new/'src/main.c'
s=p.read_text()
old=s[s.index('/* 独立PB15'):s.index('static void command(void)')]
section=old.replace('PB15/TIM12_CH2','PB14/TIM12_CH1').replace('开机PB15','开机PB14')
section=section.replace('TIM12->CCR2','TIM12->CCR1').replace('CCMR1=0x6800','CCMR1=0x68').replace('CCER=1u<<4','CCER=1u')
section=section.replace('3u<<30','3u<<28').replace('2u<<30','2u<<28').replace('1u<<15','1u<<14')
section=section.replace('15u<<28','15u<<24').replace('9u<<28','9u<<24')
assert section!=old
s=s.replace(old,section,1)
s=s.replace("reply[2]='5';reply[3]='P';reply[4]=1;reply[5]=15;", "reply[2]='4';reply[3]='P';reply[4]=1;reply[5]=14;",1)
p.write_text(s)
p=new/'src/tests/test_firmware.c'
s=p.read_text();a=s.index('static void servo_tests(void)');b=s.index('int main(',a)
block=s[a:b].replace('t12.CCR2','t12.CCR1').replace('gb.MODER>>30','gb.MODER>>28').replace('gb.AFR[1]>>28','gb.AFR[1]>>24').replace('PB15_','PB14_')
block=block.replace('uint8_t pulse[2];','uint8_t pulse[2];\n uint32_t pb15_mode=gb.MODER&(3u<<30),pb15_af=gb.AFR[1]&(15u<<28);')
block=block.replace('for(unsigned i=0;i<4;i++)assert(!duty[i]);',
    'assert(!memcmp(last_tx+7,"B14P",4)&&last_tx[12]==14);\n for(unsigned i=0;i<4;i++)assert(!duty[i]);',1)
block=block.replace('assert(last_tx[3]==0x87&&rd16(last_tx+7)==2000);',
    'assert(last_tx[3]==0x87&&rd16(last_tx+7)==2000);\n assert(t12.CCMR1==0x68&&t12.CCER==1u&&!t12.CCR2);\n assert((gb.MODER&(3u<<30))==pb15_mode&&(gb.AFR[1]&(15u<<28))==pb15_af);')
# 覆盖两个方向及边界；非法指令既不改脉宽，也不延长看门狗。
block=block.replace('for(unsigned i=0;i<4;i++)assert(!duty[i]);\n /* 模拟',
    'const unsigned pulses[]={500,600,1000,1500,2000,2400,2500};\n'
    ' for(unsigned i=0;i<sizeof(pulses)/sizeof(pulses[0]);i++){p16(pulse,pulses[i]);feed(0x31,pulse,2,0);assert(t12.CCR1==pulses[i]);}\n'
    ' uint32_t last_refresh=servo_last_ms;ms++;p16(pulse,2501);feed(0x31,pulse,2,0);assert(t12.CCR1==2500&&servo_last_ms==last_refresh);\n'
    ' for(unsigned i=0;i<4;i++)assert(!duty[i]);\n /* 模拟')
s=s[:a]+block+s[b:];p.write_text(s)
unchanged={}
for p in (base/'src').rglob('*'):
    rel=p.relative_to(base/'src')
    if not p.is_file() or 'build' in rel.parts or '__pycache__' in rel.parts:continue
    if str(rel) in ('main.c','tests/test_firmware.c'):continue
    assert p.read_bytes()==(new/'src'/rel).read_bytes(),str(rel)
    unchanged[str(rel)]=hashlib.sha256(p.read_bytes()).hexdigest()
(new/'baseline.json').write_text(json.dumps({'base':'v1.2','backup':str(backup),'unchanged_files':unchanged},indent=2))
(new/'README.md').write_text('''# v1.4 PB14 舵机候选固件（未刷写）

从已安装v1.2源码复制，仅将舵机从PB15/TIM12_CH2改为PB14/TIM12_CH1（AF9）。
50 Hz、500–2500 us、中位1500 us、500 ms无指令回中和0x31/0x87协议不变。
能力查询0x30→0x86改报B14P、pin=14。原PB15不再启用PWM。
四路电机、编码器、IMU、PI、速度上限0.8 m/s、80%PWM保持v1.2原值。
v1.3为独立未刷写的1 m/s候选，本版不合入其速度改动。

编译：cd /home/potato/Slicer_Workspace2 && ./Hardware/build.sh v1.4
必须按Hardware/README.md要求先物理断开12V、Motor OFF、仅USB供电，再授权刷写。
信号线从PB15移到PB14，电源与地线不变；接线前断开舵机电源。
没有进行实物舵机测试，换引脚不代表已证实PB15硬件损坏。
''')
print('PREPARED',new,'BACKUP',backup)
