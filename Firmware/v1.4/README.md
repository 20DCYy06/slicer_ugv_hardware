# v1.4 PB14 舵机固件（已安装并通过零输出验收）

从已安装v1.2源码复制，仅将舵机从PB15/TIM12_CH2改为PB14/TIM12_CH1（AF9）。
50 Hz、500–2500 us、中位1500 us、500 ms无指令回中和0x31/0x87协议不变。
能力查询0x30→0x86改报B14P、pin=14。原PB15不再启用PWM。
四路电机、编码器、IMU、PI、速度上限0.8 m/s、80%PWM保持v1.2原值。
v1.3为独立未刷写的1 m/s候选，本版不合入其速度改动。

编译：cd /home/potato/Slicer_Workspace2 && ./Hardware/build.sh v1.4
必须按Hardware/README.md要求先物理断开12V、Motor OFF、仅USB供电，再授权刷写。
信号线从PB15移到PB14，电源与地线不变；接线前断开舵机电源。
没有进行实物舵机测试，换引脚不代表已证实PB15硬件损坏。

安装记录：install_record.json。双备份与整片512KiB读回一致，选项字节未改；PB14能力查询和四轮零输出验收通过。未发送舵机运动指令，实际引脚波形和带载上升仍需现场测试。
