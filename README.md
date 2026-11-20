# Workspace2 C30D 固件与解锁

本工程独立于旧工作空间；所有源码、编译产物、备份、日志均位于 `/home/potato/Slicer_Workspace2/`。旧目录仅在首次复制和隔离验收时只读访问。运行无需导入、source 或挂载旧工程。

## 当前安装：v1.4 PB14

舵机已由PB15/TIM12_CH2切换至PB14/TIM12_CH1，其他底盘参数沿用v1.2。
原板双份512KiB备份一致，基线匹配后仅写sector0，整片读回及选项字节校验通过。
实机能力回复PB14，零速验收：24帧，IMU ready、四轮PWM=0、CCR=0、输出上限80%。
固件、安装记录与验证结果均在Firmware/v1.4/，原v1.2与v1.3候选保留。
未运行舵机和底盘运动。改接信号线前请断开舵机电源，PB15信号线移到PB14。

## 历史安装：v1.2

新增PB15/TIM12_CH2连续舵机输出；四路电机的PID、IMU、控制参数、引脚和既有协议沿用v1.1，未调整。
已完成16360行直行/转向/换向控制轨迹逐字节回归一致性，以及舵机命令不改变运行中四轮PWM/目标/积分的离线测试。
本次刷写前双读512KiB并确认原板为v1.1基线，保留完整备份；仅改sector0，写后整片512KiB与选项字节校验通过。
只读PB15能力查询及USB供电四路零PWM/零CCR、IMU就绪验收通过；未运行舵机、未实车行驶。
安装记录：`Firmware/v1.2/install_record.json`；候选源码和二进制：`Firmware/v1.2/src/`及`bin/`。
固件参数仍与v1.1一致，集中在`Firmware/v1.2/src/config.yaml`。上位机底盘配置未改。
原v1.1/v1.0完整保留。重新编译：`./Hardware/build.sh v1.2`。
PB15键盘程序：`./Test/run_b15_servo.sh --run`；只读查询加`--check`。

## 初始基线：固件 v1.0

适用于同 PCB 接线版本 C30D / STM32F407VET6（512 KiB Flash）。继承旧工程已验证的 SLC2 解析、CRC、轮速 PI、车体线速度外环、板载 ICM20948 角速度外环、桥臂驱动、反向积分复位及比例饱和；不增加新的恢复状态机。

控制链：上位机 vx/wz → 0x16 → 编码器/IMU车体外环 → 四轮目标速度 → 轮速 PI → 电机 PWM。A右后、B左后、C右前、D左前。

当前几何：轮径 **65 mm（用户实测确认）**，轮距150 mm、轴距170 mm，60000 counts/rev。轮距、轴距、CPR沿用现场旧配置，换机械结构后应再测。引脚与电气方向继承现有 C30D 实现。

固件参数集中在 `Firmware/v1.0/src/config.yaml`；上位机设备身份、按键档位和轴绑定在 `Hardware/config.yaml`。更改固件配置后必须重编译并重新烧录；上位机配置在下次程序启动时读取。

| 参数 | 只读参考中的当前值 | v1.0 |
|---|---:|---:|
| 轮径 |65 mm|65 mm|
| 闭环PWM上限 |982‰|800‰（80%）|
| 编码器异常增量对应速度 |1.0 m/s|1.6 m/s|
| 堵转持续时间 |3 s|5 s|
| 方向不符持续时间 |1 s|2 s|
| 指令失联看门狗 |500 ms|500 ms|
| 控制/遥测周期 |10/40 ms|10/40 ms|

这里的1.6 m/s用于判断不合理编码器增量，不是车辆速度指令上限。按65 mm、60000 CPR计算，10 ms容许增量约4702 counts，并继续按实际控制间隔缩放，避免速度略有过冲就误锁。

MCU保留0.8 m/s车体/轮速、3.5 rad/s角速度能力；本次两套测试程序统一限制在0.6 m/s与1.0 rad/s，未把上位机改成PWM控制。输出百分比100仅允许PI最高用到80%PWM，0速始终对应停机。

欠压、真实堵转/方向错误、串口溢出、控制周期异常和失联停车保留。固件没有电流及绕组温度采样，不能凭空实现过流检测或承诺电机不烧毁；未扩大至98.2%输出。

## 构建与版本归档

```bash
cd /home/potato/Slicer_Workspace2
./Hardware/build.sh v1.0
```

只使用已有 gcc、arm-none-eabi-gcc、Python3和PyYAML。脚本运行实际C模块的离线测试及协议测试，然后生成 `Firmware/v1.0/bin/firmware.bin/.hex/.elf/.map` 和manifest。源码保存在同版本的src目录。后续修改先复制为v1.1再改，保留已验收版本。

## 串口协议

应用串口USART1 PA9/PA10，**115200 8N1**。外层：`A5 5A | version:u8=1 | type:u8 | payload_len:u8 | seq:u16LE | payload | CRC:u16LE`。CRC16-CCITT初值0xffff，多项式0x1021，覆盖version至payload末尾。多字节载荷均小端。

| 类型 | 含义/载荷 |
|---|---|
|0x01 / 0x81|HELLO nonce:u32；SLC2身份与同nonce回复。内部先停机，仅Motor OFF时清故障|
|0x12 / 0x82|nonce + 4路PWM极性i8 + 4路编码器极性i8；配置确认|
|0x16|nonce:u32 + vx:i16(mm/s) + wz:i16(mrad/s)，上位机50 Hz连续刷新|
|0x17 / 0x84|车体/IMU能力查询|
|0x18 / 0x85|nonce+百分比；回复硬上限800‰、period1600、最大1280 ticks|
|0x90|原61字节状态/编码器/轮速/PWM遥测|
|0x93|64字节车体、IMU、目标速度及故障状态遥测|
|0x14 / 0x91|已有故障诊断查询|
|0x15 / 0x92|只读PWM定时器寄存器快照|
|0x10 全零|兼容的全停止；退出时使用|

本次上位机不使用0x11开环PWM或单轮校准动作。

## 烧录

先静止、物理断开12V、Motor OFF，仅USB供电，确认新板接线版本。新板序列号为`5CF0016129`，设备自动按VID/PID/序列号查找，不依赖tty编号；不抢占已有串口进程。

```bash
cd /home/potato/Slicer_Workspace2
# 只备份（也会复位进入ROM，因此同样要求动力断电）
python3 -s Hardware/Flashing/flash.py --power-off-confirmed
# 备份并烧录v1.0；先自动双读512KiB，匹配后仅擦写sector0
python3 -s Hardware/Flashing/flash.py --version v1.0 --flash --power-off-confirmed
# USB供电验收：只发全零速度和查询
python3 -s Hardware/zero_check.py
```

ROM串口76800 8E1；DTR/RTS启动序列复用已确认C30D方案。烧录器仅允许读、write-memory和sector0扩展擦除，禁止全片擦除、解除保护、写选项字节、RAM loader、GO跳转。应用写入地址0x08000000。完整双备份和整片写后读回位于`Hardware/Backups/`，摘要在`last_install.json`。仅`verified=true`才代表烧录通过；编译manifest的flashed=false是编译阶段字段，不替代安装记录。

## 查询与解锁

```bash
cd /home/potato/Slicer_Workspace2
python3 -s Hardware/Unlock/unlock.py --check
# 退出其他控制程序、停车并Motor OFF后执行
python3 -s Hardware/Unlock/unlock.py --unlock
```

`--check`只接收状态，无串口下行命令，不会先清掉故障证据。`--unlock`检查Motor OFF和零PWM后，复用HELLO清软件故障、配置极性并验证状态；不恢复旧目标、不发非零速度、不绕过物理Motor输入。完成后现场人员再开Motor并启动控制程序。退出码0成功、2查询到故障锁存、3通信/状态失败。日志全部在`Hardware/Logs/`。

## 验证范围与隔离

已做无硬件控制/协议/输入测试、ARM构建；实际烧录及零输出验收见交付记录。没有替用户执行实车运动测试。

`Hardware/Audit/old_workspace_before_20261107.json`记录旧树8614项mode/size/mtime/ctime/链接目标，验收对照由`isolation_report.json`给出。旧工作空间及其子目录本次未发现Git仓库，不能伪称git clean；采用文件系统元数据与参考源码SHA256核验。读取可能按文件系统策略更新访问时间atime，不修改内容、mtime或ctime。

## PB14 v1.4（已安装）
仅将当前v1.2舵机引脚由PB15/TIM12_CH2替换为PB14/TIM12_CH1。
四轮闭环、IMU和速度参数不变，不合入独立v1.3提速候选。
二进制：Firmware/v1.4/bin/firmware.bin；验证：Firmware/v1.4/validation.json。
已备份上位机，旧run_b15_servo.sh与融合手柄入口均兼容能力查询，新别名Test/run_b14_servo.sh。
断开动力12V、Motor OFF、仅USB并确认后，使用基线核验烧录器：
`python3 -s Hardware/Flashing/flash_v1_4_verified.py --power-off-confirmed`
烧录器双备份并要求现板仍为v1.2，否则拒绝擦写。刷后应查询到B14P/pin14。
